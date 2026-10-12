#include <reachability_map/reachability_map.h>
#include <ik_constraint/JointLimitConstraint.h>
#include <ik_constraint/PositionConstraint.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <exception>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <mutex>
#include <random>
#include <stdexcept>
#include <thread>

namespace reachability_map {
namespace {

void validateMap(const ReachabilityMap& map) {
  if(!std::isfinite(map.posResolution) || map.posResolution <= 0 ||
     !map.origin.allFinite() || !map.boxSize.allFinite() ||
     (map.boxSize.array() < 0).any() ||
     (!map.boxSize.isZero() && (map.boxSize.array() <= 0).any()) ||
     !std::isfinite(map.transparency) || map.transparency < 0 || map.transparency > 1)
    throw std::invalid_argument("Invalid reachability map metadata");
  for(const std::pair<Eigen::Vector3d, double>& cell : map.reachabilityMap)
    if(!cell.first.allFinite() || !std::isfinite(cell.second) || cell.second < 0 || cell.second > 1)
      throw std::invalid_argument("Invalid reachability cell");
}

void validate(const ReachabilityMapParam& p, int threads) {
  if(!p.robot || p.configuration.size() != p.robot->nq || !p.configuration.allFinite())
    throw std::invalid_argument("Invalid Pinocchio model/configuration");
  if(threads < 1 || p.testPerGrid < 1 || p.initialSolutionNum < 0 || p.pikParam.maxIteration < 1 ||
     p.endEffectors.empty() || p.robot->nv == 0)
    throw std::invalid_argument("Empty effectors/model DOFs or invalid iteration/thread count");
  if(!std::isfinite(p.posResolution) || p.posResolution <= 0 || !p.size.allFinite() ||
     (p.size.array() < 0).any() || !p.origin.allFinite() ||
     !std::isfinite(p.positionPrecision) || p.positionPrecision <= 0 ||
     !std::isfinite(p.rotationPrecision) || p.rotationPrecision <= 0 ||
     !p.weight.allFinite() || (p.weight.array() < 0).any() || p.weight.isZero())
    throw std::invalid_argument("Invalid grid, precision or weight");
  // Movable DOFs are defined by the supplied model. Lock joints/base when building it.
  for(pinocchio::JointIndex id=1; id<p.robot->njoints; ++id) {
    if(p.robot->nqs[id] != 1 || p.robot->nvs[id] != 1)
      throw std::invalid_argument("Reachability model requires scalar joints; fix the base with pinocchio_model_builder");
    const int q = p.robot->idx_qs[id];
    const double lower = p.robot->lowerPositionLimit[q], upper = p.robot->upperPositionLimit[q];
    if(!std::isfinite(lower) || !std::isfinite(upper) || lower > upper)
      throw std::invalid_argument("Model joints require finite ordered position limits");
    if(p.configuration[q] < lower || p.configuration[q] > upper)
      throw std::invalid_argument("Initial configuration violates joint limits");
  }
  for(const EndEffector& ee : p.endEffectors)
    if(ee.parent == 0 || ee.parent >= p.robot->njoints || !ee.localPose.toHomogeneousMatrix().allFinite())
      throw std::invalid_argument("Invalid end effector");
  for(const std::vector<std::shared_ptr<ik_constraint::IKConstraint>>& level : p.constraints) for(const std::shared_ptr<ik_constraint::IKConstraint>& c : level) {
    if(!c) throw std::invalid_argument("Null constraint");
    for(const std::shared_ptr<const pinocchio::Model>& model : c->getModels())
      if(model.get() != p.robot.get()) throw std::invalid_argument("Additional constraints must use robot");
  }
}

std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint>>> makeConstraints(const ReachabilityMapParam& p) {
  std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint>>> levels(1);
  for(pinocchio::JointIndex id=1; id<p.robot->njoints; ++id) {
    std::shared_ptr<ik_constraint::JointLimitConstraint> limit =
      std::make_shared<ik_constraint::JointLimitConstraint>(p.robot);
    limit->setJointId(id);
    limit->setTolerance(0);
    limit->setPrecision(1e-6);
    levels[0].push_back(limit);
  }
  for(const std::vector<std::shared_ptr<ik_constraint::IKConstraint>>& source : p.constraints) {
    levels.emplace_back();
    for(const std::shared_ptr<ik_constraint::IKConstraint>& c : source) {
      std::shared_ptr<ik_constraint::IKConstraint> copy = c->clone();
      if(!copy || copy.get() == c.get()) throw std::invalid_argument("Constraint clone must be independent");
      levels.back().push_back(std::move(copy));
    }
  }
  return levels;
}

Eigen::Matrix3d randomRotation(std::mt19937& rng) {
  std::normal_distribution<double> normal;
  Eigen::Vector4d q;
  do { for(int i=0; i<4; ++i) q[i] = normal(rng); } while(q.norm() < 1e-12);
  q.normalize();
  return Eigen::Quaterniond(q[3], q[0], q[1], q[2]).toRotationMatrix();
}

YAML::Node vectorNode(const Eigen::Vector3d& v) {
  YAML::Node node;
  for(int i=0; i<3; ++i) node.push_back(v[i]);
  node.SetStyle(YAML::EmitterStyle::Flow);
  return node;
}
Eigen::Vector3d readVector(const YAML::Node& node) {
  if(!node.IsSequence() || node.size() != 3) throw std::runtime_error("Expected a three-element vector");
  return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}
}

void createMap(const ReachabilityMapParam& p, ReachabilityMap& map, int thread_num) {
  validate(p, thread_num);
  std::size_t counts[3], total = 1;
  for(int axis=0; axis<3; ++axis) {
    const double n = std::floor(p.size[axis] / p.posResolution + 1e-9) + 1;
    if(!std::isfinite(n) || n > 10000000 || n > 10000000.0 / total)
      throw std::invalid_argument("Grid exceeds ten million cells; increase resolution");
    counts[axis] = static_cast<std::size_t>(n);
    total *= counts[axis];
  }
  std::vector<double> rates(total, 0);
  std::atomic<std::size_t> next{0};
  std::atomic<bool> stop{false};
  std::exception_ptr error;
  std::mutex errorMutex;
  std::mutex progressMutex;
  std::size_t completed = 0;
  std::function<void()> worker = [&]() {
    try {
      std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint>>> levels = makeConstraints(p);
      std::shared_ptr<ik_constraint::PositionConstraint> target = std::make_shared<ik_constraint::PositionConstraint>(p.robot);
      target->setPrecision(p.positionPrecision);
      Eigen::Matrix<double, 6, 1> weights = p.weight;
      weights.tail<3>() *= p.positionPrecision / p.rotationPrecision;
      target->setWeight(weights);
      levels.push_back({target});
      prioritized_inverse_kinematics_solver::IKParam ik = p.pikParam;
      ik.viewer.reset();
      ik.debugLevel = 0;
      ik.checkFinalState = true;
      ik.satisfiedConvergeLevel = -1;
      std::vector<std::shared_ptr<prioritized_qp_base::Task>> tasks;
      std::vector<std::shared_ptr<pinocchio::Model>> models{p.robot};
      while(!stop.load()) {
        const std::size_t index = next.fetch_add(1);
        if(index >= total) break;
        const std::size_t x = index / (counts[1] * counts[2]);
        const std::size_t y = (index / counts[2]) % counts[1], z = index % counts[2];
        const Eigen::Vector3d offset = -p.size / 2 + p.posResolution * Eigen::Vector3d(x, y, z);
        std::seed_seq seed{p.seed, static_cast<std::uint32_t>(index)};
        std::mt19937 rng(seed);
        int successes = 0;
        for(int trial=0; trial<p.testPerGrid; ++trial) {
          const pinocchio::SE3 pose(randomRotation(rng), p.origin + offset);
          target->setBLocalPose(pose);
          bool solved = false;
          for(const EndEffector& ee : p.endEffectors) {
            target->setAJointId(ee.parent);
            target->setALocalPose(ee.localPose);
            for(int attempt=0; attempt<=p.initialSolutionNum && !solved; ++attempt) {
              std::vector<Eigen::VectorXd> configurations{p.configuration};
              if(attempt) for(pinocchio::JointIndex id=1; id<p.robot->njoints; ++id) {
                const int q = p.robot->idx_qs[id];
                std::uniform_real_distribution<double> distribution(
                  p.robot->lowerPositionLimit[q], p.robot->upperPositionLimit[q]);
                configurations[0][q] = distribution(rng);
              }
              solved = prioritized_inverse_kinematics_solver::solveIKLoop(models, configurations, levels, tasks, ik);
            }
            if(solved) break;
          }
          if(solved) ++successes;
        }
        rates[index] = static_cast<double>(successes) / p.testPerGrid;
        {
          std::lock_guard<std::mutex> lock(progressMutex);
          ++completed;
          if(completed == total ||
             completed % std::max<std::size_t>(1, total / 100) == 0) {
            std::cout << "Cells evaluated: " << completed << '/' << total << std::endl;
          }
        }
      }
    } catch(...) {
      std::lock_guard<std::mutex> lock(errorMutex);
      if(!error) error = std::current_exception();
      stop.store(true);
    }
  };
  std::vector<std::thread> workers;
  try {
    for(std::size_t i=0; i<std::min<std::size_t>(thread_num, total); ++i) workers.emplace_back(worker);
  } catch(...) {
    stop.store(true);
    for(std::thread& thread : workers) thread.join();
    throw;
  }
  for(std::thread& thread : workers) thread.join();
  if(error) std::rethrow_exception(error);
  ReachabilityMap result = map;
  result.reachabilityMap.clear();
  result.origin = p.origin;
  result.posResolution = p.posResolution;
  for(std::size_t index=0; index<total; ++index) {
    if(rates[index] > 0) {
      const std::size_t x = index / (counts[1] * counts[2]);
      const std::size_t y = (index / counts[2]) % counts[1], z = index % counts[2];
      result.reachabilityMap.emplace_back(-p.size / 2 + p.posResolution * Eigen::Vector3d(x, y, z), rates[index]);
    }
  }
  validateMap(result);
  map = std::move(result);
}

void initializeReachabilityViewer(const std::string& modelPath, mujoco_viewer::Viewer& viewer) {
  char error[1024] = {};
  std::shared_ptr<mjModel> model(mj_loadXML(modelPath.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
  if(!model) throw std::runtime_error("Cannot load MuJoCo model: " + modelPath + ": " + error);
  bool hasSkybox = false;
  for(int texture=0; texture<model->ntex; ++texture) {
    if(model->tex_type[texture] != mjTEXTURE_SKYBOX) continue;
    const std::size_t count = static_cast<std::size_t>(model->tex_width[texture]) *
      model->tex_height[texture] * model->tex_nchannel[texture];
    std::fill_n(model->tex_data + model->tex_adr[texture], count, mjtByte{255});
    hasSkybox = true;
  }
  if(!hasSkybox) throw std::invalid_argument("Reachability display model requires a skybox texture");
  std::fill_n(model->vis.rgba.haze, 4, 1.0f);
  // Move floor and other world geoms into a hidden group without changing physics.
  for(int geom=0; geom<model->ngeom; ++geom)
    if(model->geom_bodyid[geom] == 0) model->geom_group[geom] = mjNGROUP - 1;
  viewer.viewModel(model);
  for(int group=0; group<mjNGROUP; ++group) {
    viewer.option()->geomgroup[group] = (group == 1);
    viewer.option()->sitegroup[group] = 0;
  }
}

void reserveSceneForMap(const ReachabilityMap& map, mujoco_viewer::Viewer& viewer) {
  validateMap(map);
  if(!viewer.model()) throw std::invalid_argument("Viewer has no model");
  viewer.updateScene();
  const std::size_t required = map.reachabilityMap.size() + viewer.scene()->ngeom;
  if(required > static_cast<std::size_t>(std::numeric_limits<int>::max()))
    throw std::invalid_argument("Map is too large for MuJoCo scene");
  if(required > static_cast<std::size_t>(viewer.scene()->maxgeom)) {
    mjv_freeScene(viewer.scene());
    mjv_defaultScene(viewer.scene());
    mjv_makeScene(viewer.model(), viewer.scene(), static_cast<int>(required));
    viewer.updateScene();
  }
}

void visualizeMap(const ReachabilityMap& map, mujoco_viewer::Viewer& viewer) {
  validateMap(map);
  if(!viewer.model()) throw std::invalid_argument("Viewer has no model");
  if(map.reachabilityMap.size() > static_cast<std::size_t>(viewer.scene()->maxgeom - viewer.scene()->ngeom))
    throw std::runtime_error("Map exceeds MuJoCo scene capacity; call reserveSceneForMap before rendering");
  const Eigen::Vector3d halfSize = (map.boxSize.isZero() ? Eigen::Vector3d::Constant(map.posResolution) : map.boxSize) / 2;
  const mjtNum size[3] = {halfSize[0], halfSize[1], halfSize[2]};
  const mjtNum rotation[9] = {1,0,0,0,1,0,0,0,1};
  for(const std::pair<Eigen::Vector3d, double>& cell : map.reachabilityMap) {
    const Eigen::Vector3d position = (map.origin + cell.first).eval();
    const mjtNum pos[3] = {position[0], position[1], position[2]};
    const double rate = cell.second;
    const float rgba[4] = {static_cast<float>(std::max(1-2*rate, 0.0)),
      static_cast<float>(1-std::abs(1-2*rate)), static_cast<float>(std::max(2*rate-1, 0.0)),
      static_cast<float>(1-map.transparency)};
    mjvGeom* geom = viewer.appendGeom();
    if(!geom) throw std::runtime_error("MuJoCo scene capacity exhausted");
    mjv_initGeom(geom, mjGEOM_BOX, size, pos, rotation, rgba);
    geom->category = mjCAT_DECOR;
  }
}

void writeMap(const std::string& path, const ReachabilityMap& map) {
  validateMap(map);
  YAML::Node root, metadata;
  metadata["origin"] = vectorNode(map.origin);
  metadata["posResolution"] = map.posResolution;
  metadata["transparency"] = map.transparency;
  metadata["boxSize"] = vectorNode(map.boxSize);
  root["reachabilityMapParam"].push_back(metadata);
  root["reachabilityMap"] = YAML::Node(YAML::NodeType::Sequence);
  for(const std::pair<Eigen::Vector3d, double>& cell : map.reachabilityMap) {
    YAML::Node node;
    node["translation"] = vectorNode(cell.first);
    node["solvability"] = cell.second;
    root["reachabilityMap"].push_back(node);
  }
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
  emitter << root;
  if(!emitter.good()) throw std::runtime_error("Cannot serialize map");
  std::ofstream output(path);
  if(!output) throw std::runtime_error("Cannot open output map: " + path);
  output << emitter.c_str() << '\n';
  output.close();
  if(!output) throw std::runtime_error("Cannot write output map: " + path);
}

void readMap(const std::string& path, ReachabilityMap& map) {
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node metadata = root["reachabilityMapParam"];
  if(!metadata.IsSequence() || metadata.size() != 1 || !root["reachabilityMap"].IsSequence())
    throw std::runtime_error("Invalid reachability YAML structure");
  ReachabilityMap result;
  result.origin = readVector(metadata[0]["origin"]);
  result.posResolution = metadata[0]["posResolution"].as<double>();
  if(metadata[0]["transparency"]) result.transparency = metadata[0]["transparency"].as<double>();
  if(metadata[0]["boxSize"]) result.boxSize = readVector(metadata[0]["boxSize"]);
  for(const YAML::Node& node : root["reachabilityMap"])
    result.reachabilityMap.emplace_back(readVector(node["translation"]), node["solvability"].as<double>());
  validateMap(result);
  map = std::move(result);
}

void setViewerConfiguration(const pinocchio::Model& model, const Eigen::VectorXd& q,
                            mujoco_viewer::Viewer& viewer) {
  if(!viewer.model() || !viewer.data() || q.size() != model.nq || !q.allFinite())
    throw std::invalid_argument("Invalid viewer/configuration");
  const mjModel* mj = viewer.model();
  Eigen::VectorXd mapped = Eigen::Map<const Eigen::VectorXd>(viewer.data()->qpos, mj->nq);
  for(pinocchio::JointIndex id=1; id<model.njoints; ++id) {
    int mjid = -1;
    if(model.nqs[id] == 7 && model.nvs[id] == 6) {
      // URDF free-flyer names are synthetic; identify the unique MJCF free joint.
      for(int j=0; j<mj->njnt; ++j) if(mj->jnt_type[j] == mjJNT_FREE) {
        if(mjid != -1) throw std::invalid_argument("Expected one MuJoCo free joint");
        mjid = j;
      }
    } else mjid = mj_name2id(mj, mjOBJ_JOINT, model.names[id].c_str());
    if(mjid < 0) throw std::invalid_argument("Missing MuJoCo joint: " + model.names[id]);
    const int dst = mj->jnt_qposadr[mjid], src = model.idx_qs[id];
    if(model.nqs[id] == 1 && (mj->jnt_type[mjid] == mjJNT_HINGE || mj->jnt_type[mjid] == mjJNT_SLIDE))
      mapped[dst] = q[src];
    else if(model.nqs[id] == 7 && mj->jnt_type[mjid] == mjJNT_FREE) {
      mapped.segment<3>(dst) = q.segment<3>(src);
      mapped[dst+3] = q[src+6];
      mapped.segment<3>(dst+4) = q.segment<3>(src+3);
    } else throw std::invalid_argument("Unsupported or mismatched viewer joint: " + model.names[id]);
  }
  Eigen::Map<Eigen::VectorXd>(viewer.data()->qpos, mj->nq) = mapped;
  mj_forward(mj, viewer.data());
}
}
