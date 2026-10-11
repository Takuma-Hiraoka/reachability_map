#include <reachability_map/reachability_map.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio_model_builder/pinocchio_model_builder.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

int main() {
  try {
    // Edit the saved-map path and display settings here, then rebuild.
    const std::string share = ament_index_cpp::get_package_share_directory("reachability_map_sample");
    const std::string input = share + "/config/g1_right_hand.yaml";
    const std::string robotDir = share + "/unitree_ros/robots/g1_description";
    reachability_map::ReachabilityMap map;
    reachability_map::readMap(input, map);
    // Match the original visualization sample's thin horizontal cells.
    map.boxSize = Eigen::Vector3d(map.posResolution, map.posResolution, 0.001);
    pinocchio::Model model;
    pinocchio_model_builder::buildModel(model, robotDir + "/g1_29dof.urdf", std::vector<std::string>());
    Eigen::VectorXd q = pinocchio::neutral(model);
    q[2] = 0.80;
    for(pinocchio::JointIndex id=1; id<model.njoints; ++id) if(model.nqs[id] == 1) {
      const int index = model.idx_qs[id];
      q[index] = std::clamp(q[index], model.lowerPositionLimit[index], model.upperPositionLimit[index]);
    }
    mujoco_viewer::Viewer viewer;
    reachability_map::initializeReachabilityViewer(robotDir + "/g1_29dof.xml", viewer);
    reachability_map::setViewerConfiguration(model, q, viewer);
    reachability_map::reserveSceneForMap(map, viewer);
    std::cout << "Displaying " << map.reachabilityMap.size() << " cells from " << input << std::endl;
    while(viewer.isOpen()) {
      viewer.updateScene();
      reachability_map::visualizeMap(map, viewer);
      viewer.drawScene();
      viewer.pollEvents();
      std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    return 0;
  } catch(const std::exception& e) {
    std::cerr << "visualize: " << e.what() << '\n';
    return 1;
  }
}
