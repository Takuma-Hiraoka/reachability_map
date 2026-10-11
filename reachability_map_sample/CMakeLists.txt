cmake_minimum_required(VERSION 3.16)
project(reachability_map_sample)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)

set(dependencies
  ament_index_cpp
  reachability_map
  pinocchio
  pinocchio_model_builder
  mujoco_viewer)

foreach(dependency ${dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

foreach(target g1_hand g1_foot visualize solvability_bar)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target} ${dependencies})
  target_link_libraries(${target} pinocchio::pinocchio pinocchio_model_builder::pinocchio_model_builder)
endforeach()

install(TARGETS g1_hand g1_foot visualize solvability_bar DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config unitree_ros DESTINATION share/${PROJECT_NAME})

ament_package()
