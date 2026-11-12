cmake_minimum_required(VERSION 3.8)
project(uwb_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(small_gicp REQUIRED)
find_package(Boost REQUIRED date_time)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(eliko_messages REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenMP REQUIRED)
find_package(visualization_msgs REQUIRED)

# GTSAM (>= 4.3, built from source: the ROS binary package is 4.2) is
# optional: without it only the Ceres pose-graph backend is built (asking for
# a GTSAM backend then fails at startup with a clear message).
option(UWB_LOCALIZATION_WITH_GTSAM "Build the GTSAM pose-graph backend if GTSAM is found" ON)
if(UWB_LOCALIZATION_WITH_GTSAM)
  find_package(GTSAM 4.3 QUIET)
endif()
if(GTSAM_FOUND)
  message(STATUS "uwb_localization: GTSAM ${GTSAM_VERSION} found, building the GTSAM pose-graph backend")
else()
  message(STATUS "uwb_localization: GTSAM >= 4.3 not found/disabled, building the Ceres pose-graph backend only")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PoseWithCovarianceStampedArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Public include dir for headers (install + build)
set(PUBLIC_INC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

include_directories(${CERES_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${pcl_conversions_INCLUDE_DIRS})
include_directories(${pcl_ros_INCLUDE_DIRS})

# --- add pose optimization library ---
add_library(pose_optimization_lib SHARED
  src/poseOptimization.cpp
  src/posegraph_backend.cpp
  src/ceres_backend.cpp
)
if(GTSAM_FOUND)
  target_sources(pose_optimization_lib PRIVATE src/gtsam_backend.cpp)
  target_link_libraries(pose_optimization_lib gtsam)
  target_compile_definitions(pose_optimization_lib PUBLIC UWB_LOCALIZATION_WITH_GTSAM)
endif()

target_include_directories(pose_optimization_lib PUBLIC ${PUBLIC_INC})
target_compile_features(pose_optimization_lib PUBLIC c_std_99 cxx_std_17)

target_link_libraries(pose_optimization_lib
  Ceres::ceres
  small_gicp::small_gicp
  ${PCL_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${pcl_ros_LIBRARIES}
  Sophus::Sophus
  OpenMP::OpenMP_CXX
)

ament_target_dependencies(pose_optimization_lib
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs
  tf2_ros tf2_geometry_msgs eliko_messages pcl_ros pcl_conversions
)

# Ensure generated typesupport is available to the lib
rosidl_target_interfaces(pose_optimization_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Export the library so ament generates env hooks (LD_LIBRARY_PATH) and proper RPATHs.
install(TARGETS pose_optimization_lib
  EXPORT export_pose_optimization_lib
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
)
ament_export_targets(export_pose_optimization_lib HAS_LIBRARY_TARGET)

# Export headers + dependencies to downstream packages (optional but tidy)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs
  tf2_ros tf2_geometry_msgs eliko_messages pcl_ros pcl_conversions
  Ceres Sophus small_gicp PCL
)

# OpenMP compile options (if you want explicit flags)
target_compile_options(pose_optimization_lib PRIVATE ${OpenMP_CXX_FLAGS})


# Pose optimization executable
add_executable(pose_optimization_node
  src/pose_optimization_node.cpp
)
target_include_directories(pose_optimization_node PUBLIC ${PUBLIC_INC})
target_compile_features(pose_optimization_node PUBLIC c_std_99 cxx_std_17)
target_link_libraries(pose_optimization_node pose_optimization_lib)
ament_target_dependencies(pose_optimization_node
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs
  tf2_ros tf2_geometry_msgs eliko_messages pcl_ros pcl_conversions
)
rosidl_target_interfaces(pose_optimization_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Make the exe look in its own folder for the shared lib at runtime
# (install dir is .../lib/uwb_localization)
set_target_properties(pose_optimization_node PROPERTIES
  BUILD_RPATH "\$ORIGIN"
  INSTALL_RPATH "\$ORIGIN"
)

# Relative localization executable
add_executable(global_opt_node_eliko src/global_opt_node_eliko.cpp)
target_include_directories(global_opt_node_eliko PUBLIC ${PUBLIC_INC})
target_compile_features(global_opt_node_eliko PUBLIC c_std_99 cxx_std_17)
target_link_libraries(global_opt_node_eliko Ceres::ceres Sophus::Sophus)
ament_target_dependencies(global_opt_node_eliko
  rclcpp std_msgs geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs eliko_messages
)

#UAV and AGV odometry nodes
add_executable(uav_odometry_node src/uav_odometry_node.cpp)
target_include_directories(uav_odometry_node PUBLIC ${PUBLIC_INC})
target_compile_features(uav_odometry_node PUBLIC c_std_99 cxx_std_17)
target_link_libraries(uav_odometry_node Sophus::Sophus)
ament_target_dependencies(uav_odometry_node rclcpp geometry_msgs nav_msgs)

add_executable(agv_odometry_node src/agv_odometry_node.cpp)
target_include_directories(agv_odometry_node PUBLIC ${PUBLIC_INC})
target_compile_features(agv_odometry_node PUBLIC c_std_99 cxx_std_17)
target_link_libraries(agv_odometry_node Sophus::Sophus)
ament_target_dependencies(agv_odometry_node rclcpp geometry_msgs sensor_msgs nav_msgs)


install(TARGETS
  pose_optimization_node
  global_opt_node_eliko
  uav_odometry_node
  agv_odometry_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
# bags/ is installed as one symbolic link to the source folder, not bag by
# bag: bags are large (a copy would duplicate them in install/) and are
# dropped in at any time (with a link they're visible without rebuilding).
# An older install put the bags there one by one; that folder is replaced
# only if it holds nothing but links, so real bag files are never deleted
# (e.g. a host folder bind-mounted there by an older Docker setup).
install(CODE "
  set(bags_link \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/bags\")
  if(IS_DIRECTORY \"\${bags_link}\" AND NOT IS_SYMLINK \"\${bags_link}\")
    file(GLOB_RECURSE bags_entries LIST_DIRECTORIES false \"\${bags_link}/*\")
    set(bags_only_links TRUE)
    foreach(entry \${bags_entries})
      if(NOT IS_SYMLINK \"\${entry}\")
        set(bags_only_links FALSE)
      endif()
    endforeach()
    if(bags_only_links)
      file(REMOVE_RECURSE \"\${bags_link}\")
    else()
      message(WARNING \"\${bags_link} holds real files, so it was left as it is instead of being linked to ${CMAKE_CURRENT_SOURCE_DIR}/bags. Move them out of there to get the link.\")
    endif()
  endif()
  if(IS_SYMLINK \"\${bags_link}\")
    file(REMOVE \"\${bags_link}\")  # removes the link itself, never its target
  endif()
  if(NOT EXISTS \"\${bags_link}\")
    file(MAKE_DIRECTORY \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\")
    file(CREATE_LINK \"${CMAKE_CURRENT_SOURCE_DIR}/bags\" \"\${bags_link}\" SYMBOLIC)
    message(STATUS \"Linked: \${bags_link} -> ${CMAKE_CURRENT_SOURCE_DIR}/bags\")
  endif()
")


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

ament_package()
