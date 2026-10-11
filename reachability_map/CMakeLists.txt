cmake_minimum_required(VERSION 3.16)
project(reachability_map)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

set(dependencies
  Eigen3
  pinocchio
  ik_constraint
  prioritized_inverse_kinematics_solver
  mujoco_viewer
  yaml-cpp)

foreach(dependency ${dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} src/reachability_map.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen pinocchio::pinocchio yaml-cpp Threads::Threads)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} Threads)
ament_package()
