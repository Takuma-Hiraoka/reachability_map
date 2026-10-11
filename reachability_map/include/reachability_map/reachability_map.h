#pragma once

#include <ik_constraint/IKConstraint.h>
#include <prioritized_inverse_kinematics_solver/prioritized_inverse_kinematics_solver.h>
#include <pinocchio/spatial/se3.hpp>
#include <cstdint>
#include <functional>
#include <string>

namespace reachability_map {
struct EndEffector {
  pinocchio::JointIndex parent = 0;
  pinocchio::SE3 localPose = pinocchio::SE3::Identity();
};

struct ReachabilityMapParam {
  std::shared_ptr<pinocchio::Model> robot;
  Eigen::VectorXd configuration;
  // All scalar joints in robot are optimized. Fix other joints and the root
  // in the model itself, e.g. with pinocchio_model_builder.
  std::vector<EndEffector> endEffectors;
  // Additional priority levels, inserted between joint-limit constraints and target.
  // clone() must produce independent mutable state; all models must be robot.
  std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint>>> constraints;
  double posResolution = 0.1;
  Eigen::Vector3d size = Eigen::Vector3d(1.0, 1.0, 1.0);
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  int testPerGrid = 10;
  int initialSolutionNum = 3;  // Random restarts after the supplied configuration.
  double positionPrecision = 0.01;
  double rotationPrecision = 0.1;
  Eigen::Matrix<double, 6, 1> weight = Eigen::Matrix<double, 6, 1>::Ones();
  std::uint32_t seed = 1;
  prioritized_inverse_kinematics_solver::IKParam pikParam;
  // Serialized worker callback: completed, total, latest success rate. No GUI calls.
  std::function<void(std::size_t, std::size_t, double)> progress;
  ReachabilityMapParam() { pikParam.maxIteration = 50; pikParam.convergeThre = 1e-6; }
};

struct ReachabilityMap {
  double posResolution = 0.1;
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  double transparency = 0.7;
  Eigen::Vector3d boxSize = Eigen::Vector3d::Zero();
  // Positions relative to origin; only cells with positive success rates are stored.
  std::vector<std::pair<Eigen::Vector3d, double>> reachabilityMap;
};

// A trial succeeds if any end effector can reach the same sampled world pose.
// Each worker owns its constraints, configurations and solver tasks. No GUI in workers.
void createMap(const ReachabilityMapParam& param, ReachabilityMap& map, int thread_num = 1);
// Load a display model and whiten its skybox before GPU upload. Hide world geoms,
// collision meshes and sites. Robot visual meshes must use group 1 (G1 convention).
// Changes only the in-memory model; the MJCF and its assets remain untouched.
void initializeReachabilityViewer(const std::string& modelPath, mujoco_viewer::Viewer& viewer);
// Call before the render loop to reserve enough scene geometry for all cells.
// Rebuilds the scene from the current model/data, discarding previous decorations.
void reserveSceneForMap(const ReachabilityMap& map, mujoco_viewer::Viewer& viewer);
// Append colored boxes between viewer.updateScene() and viewer.drawScene().
// Throws if scene capacity is insufficient rather than silently dropping cells.
void visualizeMap(const ReachabilityMap& map, mujoco_viewer::Viewer& viewer);
void writeMap(const std::string& path, const ReachabilityMap& map);
void readMap(const std::string& path, ReachabilityMap& map);
// Map by joint name, including Pinocchio xyzw -> MuJoCo wxyz quaternion conversion.
void setViewerConfiguration(const pinocchio::Model& model,
                            const Eigen::VectorXd& configuration,
                            mujoco_viewer::Viewer& viewer);
}
