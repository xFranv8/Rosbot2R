cmake_minimum_required(VERSION 3.8)
project(data_collection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  OpenCV
)

include_directories(include)

add_executable(camera
  src/data_collection/CameraNode.cpp 
  src/camera_main.cpp
)
ament_target_dependencies(camera ${dependencies})


add_executable(lidar
  src/data_collection/LidarNode.cpp 
  src/lidar_main.cpp
)
ament_target_dependencies(lidar ${dependencies})


add_executable(imu
  src/data_collection/IMUNode.cpp 
  src/imu_main.cpp
)
ament_target_dependencies(imu ${dependencies})


add_executable(control
  src/data_collection/ControlNode.cpp 
  src/control_main.cpp
)
ament_target_dependencies(control ${dependencies})



install(TARGETS
  camera
  lidar
  imu
  control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

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
