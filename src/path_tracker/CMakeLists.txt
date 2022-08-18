cmake_minimum_required(VERSION 3.8)
project(path_tracker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

add_executable(tracker src/path_subscriber.cpp)

include_directories(
        include
        ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(tracker rclcpp std_msgs geometry_msgs nav_msgs Eigen3)


target_include_directories(tracker PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

install(TARGETS tracker
        DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY
        launch
        param
        DESTINATION share/${PROJECT_NAME}/
        )

ament_package()
