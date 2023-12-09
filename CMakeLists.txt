cmake_minimum_required(VERSION 3.8)
project(slung_pose_estimation)

# Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

#set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
option(WITH_PLOTS "Build with plot support" ON) #ON


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kalmanif REQUIRED)

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

# if(WITH_PLOTS)
#   add_definitions(-DWITH_PLOTS)
#   # Include directories and link libraries necessary for sciplot
#   find_package(sciplot REQUIRED)
#   #find_package(libsciplot REQUIRED)
#   include_directories(${SCI_PLOT_INCLUDE_DIRS})
#   # Additional target_link_libraries might be required
# endif()

# if(PLOT_EXAMPLES)
#   # Look for a system installed sciplot
#   # if none found, fallback to local copy
#   find_package(sciplot QUIET)

#   if (sciplot_FOUND)
#     function(target_link_sciplot target)
#       target_link_libraries(${target} sciplot::sciplot)
#     endfunction()
#   else()



# Declare libraries and executables
add_library(State src/State.cpp)
add_library(utils src/utils.cpp)

add_executable(slung_pose_measurement src/slungPoseMeasurement.cpp)
add_executable(tester src/tester.cpp)
add_executable(slung_pose_estimation src/slungPoseEstimation.cpp)


# Specify libraries to link a library or executable target against
target_link_libraries(slung_pose_measurement
  State
  utils
)

target_link_libraries(utils
  State
)

target_link_libraries(tester #INTERFACE
  kalmanif::kalmanif
  #State
  utils
)

target_link_libraries(slung_pose_estimation
  kalmanif::kalmanif
  utils
)

target_link_libraries(State
  utils
)


ament_target_dependencies(State rclcpp tf2_geometry_msgs tf2 Eigen3)
ament_target_dependencies(utils rclcpp tf2_ros tf2_eigen Eigen3 std_msgs tf2_geometry_msgs OpenCV)
ament_target_dependencies(slung_pose_measurement rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV tf2_geometry_msgs)


# if(WITH_PLOTS)
#   # Link sciplot to your targets
#   target_link_libraries(slung_pose_estimation sciplot::sciplot)
# endif()

if(WITH_PLOTS)
  #add_definitions(-DWITH_PLOTS)
  function(target_link_sciplot target)
    target_include_directories(
      ${target} SYSTEM PRIVATE "${PROJECT_SOURCE_DIR}/external/sciplot"
    )
  endfunction()
  #endif()

  add_compile_definitions(WITH_PLOTS)
  target_link_sciplot(slung_pose_estimation)
endif()


# Install the libraries and executables
install(TARGETS
  utils
  State
  tester
  slung_pose_measurement
  slung_pose_estimation
  DESTINATION lib/${PROJECT_NAME})


# Install launch files.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install data
install(DIRECTORY
    data
    DESTINATION share/${PROJECT_NAME}/
)


# Export dependencies
ament_package()




