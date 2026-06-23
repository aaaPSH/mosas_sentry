cmake_minimum_required(VERSION 3.8)
project(scan_to_map_relocalization)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Qhull CONFIG REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(NOT TARGET QHULL::QHULL AND TARGET Qhull::qhull_r)
  add_library(QHULL::QHULL INTERFACE IMPORTED)
  set_property(TARGET QHULL::QHULL APPEND PROPERTY INTERFACE_LINK_LIBRARIES Qhull::qhull_r)
endif()

set(dependencies
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  tf2_ros
)

add_executable(scan_to_map_relocalization src/scan_to_map_relocalization.cpp)
target_include_directories(scan_to_map_relocalization PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(scan_to_map_relocalization ${PCL_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(scan_to_map_relocalization ${dependencies})

install(TARGETS scan_to_map_relocalization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
