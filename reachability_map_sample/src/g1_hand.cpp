#include <reachability_map/reachability_map.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio_model_builder/pinocchio_model_builder.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

int main() {
  try {
    // Edit the sample settings here, then rebuild.
    const std::string side = "left";
    const std::string output = "g1_" + side + "_hand.yaml";
    const int threadNum = 30;
    const std::string share = ament_index_cpp::get_package_share_directory("reachability_map_sample");
    const std::string robotDir = share + "/unitree_ros/robots/g1_description";

    reachability_map::ReachabilityMapParam param;
    param.posResolution = 0.05;
    param.size = Eigen::Vector3d(1.5, 1.5, 1.5);
    param.testPerGrid = 10;
    param.initialSolutionNum = 10;
    param.positionPrecision = 0.001;
    param.rotationPrecision = 0.001;
    param.weight.setOnes();
    param.seed = 1;
    param.pikParam.maxIteration = 50;
    const double baseHeight = 0.80;
    const std::string urdfPath = robotDir + "/g1_29dof.urdf";
    // Keep a full model for displaying the fixed joints and floating-base pose in MJCF.
    pinocchio::Model displayModel;
    pinocchio_model_builder::buildModel(displayModel, urdfPath, std::vector<std::string>());
    Eigen::VectorXd displayConfiguration = pinocchio::neutral(displayModel);
    displayConfiguration[2] = baseHeight;

    const std::vector<std::string> jointNames{"shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"};
    const std::string endEffectorJointName = side + "_wrist_yaw_joint";
    std::vector<std::string> movableJointNames;
    for(const std::string& name : jointNames) {
      const std::string full = side + "_" + name + "_joint";
      if(!displayModel.existJointName(full)) throw std::runtime_error("Missing G1 joint: " + full);
      movableJointNames.push_back(full);
    }
    std::vector<std::string> fixedJointNames;
    for(pinocchio::JointIndex id=1; id<displayModel.njoints; ++id) {
      if(displayModel.nqs[id] != 1) continue;
      if(std::find(movableJointNames.begin(), movableJointNames.end(), displayModel.names[id])
         == movableJointNames.end()) fixedJointNames.push_back(displayModel.names[id]);
    }
    // An empty base composite fixes the root. Fixed URDF joints are locked at zero.
    param.robot = std::make_shared<pinocchio::Model>();
    pinocchio_model_builder::buildModel(*param.robot, urdfPath, fixedJointNames,
                                      pinocchio::JointModelComposite(0));
    // Place the fixed root at the same world height as the displayed robot.
    const pinocchio::SE3 basePose(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, baseHeight));
    for(pinocchio::JointIndex id=1; id<param.robot->njoints; ++id)
      if(param.robot->parents[id] == 0)
        param.robot->jointPlacements[id] = basePose * param.robot->jointPlacements[id];
    for(pinocchio::FrameIndex id=1; id<param.robot->frames.size(); ++id)
      if(param.robot->frames[id].parentJoint == 0)
        param.robot->frames[id].placement = basePose * param.robot->frames[id].placement;
    param.configuration = pinocchio::neutral(*param.robot);
    for(pinocchio::JointIndex id=1; id<param.robot->njoints; ++id) {
      const int index = param.robot->idx_qs[id];
      param.configuration[index] = std::clamp(param.configuration[index],
        param.robot->lowerPositionLimit[index], param.robot->upperPositionLimit[index]);
      // Map by name: fixed joints disappear and joint IDs change in the built model.
      const pinocchio::JointIndex displayId = displayModel.getJointId(param.robot->names[id]);
      displayConfiguration[displayModel.idx_qs[displayId]] = param.configuration[index];
    }
    std::cout << "IK model DOFs: " << param.robot->nv << std::endl;
    reachability_map::EndEffector ee;
    if(!param.robot->existJointName(endEffectorJointName))
      throw std::runtime_error("Missing end-effector joint: " + endEffectorJointName);
    ee.parent = param.robot->getJointId(endEffectorJointName);
    ee.localPose.translation() = Eigen::Vector3d(0.1, 0, 0);
    param.endEffectors.push_back(ee);
    pinocchio::Data data(*param.robot);
    pinocchio::forwardKinematics(*param.robot, data, param.configuration);
    param.origin = (data.oMi[ee.parent] * ee.localPose).translation();
    reachability_map::ReachabilityMap map;
    std::cout << "Computing G1 " << side << " hand reachability map" << std::endl;
    reachability_map::createMap(param, map, threadNum);
    reachability_map::writeMap(output, map);
    std::cout << "Saved " << map.reachabilityMap.size() << " reachable cells to " << output << std::endl;

    mujoco_viewer::Viewer viewer;
    reachability_map::initializeReachabilityViewer(robotDir + "/g1_29dof.xml", viewer);
    reachability_map::setViewerConfiguration(displayModel, displayConfiguration, viewer);
    reachability_map::reserveSceneForMap(map, viewer);
    while(viewer.isOpen()) {
      viewer.updateScene();
      reachability_map::visualizeMap(map, viewer);
      viewer.drawScene();
      viewer.pollEvents();
      std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    return 0;
  } catch(const std::exception& e) {
    std::cerr << "g1_hand: " << e.what() << '\n';
    return 1;
  }
}
