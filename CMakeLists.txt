cmake_minimum_required(VERSION 3.8)
project(slung_pose_estimation)

# Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Declare libraries and executables
add_library(State src/State.cpp)
#add_executable(State src/State.cpp)
add_executable(slung_pose_estimation src/slungPoseEstimation.cpp)


# Specify libraries to link a library or executable target against
target_link_libraries(slung_pose_estimation
  State
)

ament_target_dependencies(State rclcpp tf2_geometry_msgs Eigen3)
ament_target_dependencies(slung_pose_estimation rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)


# Install the libraries and executables
install(TARGETS 
  State
  slung_pose_estimation
  DESTINATION lib/${PROJECT_NAME})


# Install launch files.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies
ament_package()




