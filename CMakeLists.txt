cmake_minimum_required(VERSION 3.5)
project(g_slam)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Build type (Release or Debug)" FORCE)
endif()

# TODO: ACTUALITZAR...


# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions -O3")

# Other dependencies
find_package(Boost REQUIRED COMPONENTS system)
find_package(TBB REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cat_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(as_lib CONFIG REQUIRED)


include_directories(
  include
)

add_executable(${PROJECT_NAME} src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    cat_msgs
    tf2
    tf2_ros
    tf2_eigen
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    as_lib::as_lib        
    gtsam
    ${Boost_LIBRARIES}
    ${TBB_IMPORTED_TARGETS}
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()