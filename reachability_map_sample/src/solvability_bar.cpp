#include <reachability_map/reachability_map.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <iostream>
#include <thread>

int main() {
  try {
    reachability_map::ReachabilityMap bar;
    bar.posResolution = 0.01;
    bar.boxSize = Eigen::Vector3d(0.05, 0.05, 0.01);
    bar.transparency = 0;
    for(int i=0; i<=100; ++i)
      bar.reachabilityMap.emplace_back(Eigen::Vector3d(0, 0, 0.01*i), i/100.0);
    mujoco_viewer::Viewer viewer;
    const std::string share = ament_index_cpp::get_package_share_directory("reachability_map_sample");
    viewer.viewModel(share + "/config/solvability_bar.xml");
    if(!viewer.model()) throw std::runtime_error("Cannot load color scale MuJoCo scene");
    viewer.setWindowSize(500, 800);
    viewer.camera()->lookat[0] = 0;
    viewer.camera()->lookat[1] = 0;
    viewer.camera()->lookat[2] = 0.5;
    viewer.camera()->distance = 1.8;
    viewer.camera()->azimuth = 90;
    viewer.camera()->elevation = 0;
    reachability_map::reserveSceneForMap(bar, viewer);
    std::cout << "Solvability: bottom 0 (red), middle 0.5 (green), top 1 (blue)" << std::endl;
    while(viewer.isOpen()) {
      viewer.updateScene();
      reachability_map::visualizeMap(bar, viewer);
      viewer.drawScene();
      viewer.pollEvents();
      std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    return 0;
  } catch(const std::exception& e) {
    std::cerr << "solvability_bar: " << e.what() << '\n';
    return 1;
  }
}
