cmake_minimum_required(VERSION 3.5)
project(yolo_tracer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

# rga
set(RGA_LIBS rga)
set(RKNN_RT_LIB ${CMAKE_SOURCE_DIR}/lib/librknnrt.so)

# camera
add_executable(img_pub src/img_pub.cpp src/postprocess.cc)
ament_target_dependencies(img_pub OpenCV PkgConfig rclcpp sensor_msgs std_msgs)
add_executable(img_sub src/img_sub.cpp)
ament_target_dependencies(img_sub OpenCV PkgConfig rclcpp sensor_msgs std_msgs)
target_link_libraries(img_pub
    ${OpenCV_LIBS}
    ${RGA_LIBS}
    ${RKNN_RT_LIB}
)
target_link_libraries(img_sub
    ${OpenCV_LIBS}
)

# motors
add_executable(control src/control.cpp)
ament_target_dependencies(control geometry_msgs rclcpp std_msgs PkgConfig)
add_executable(servo src/servo.cpp src/pwm.cpp)
ament_target_dependencies(servo geometry_msgs rclcpp std_msgs PkgConfig)

# install
install(TARGETS
  img_pub
  img_sub
  control
  servo
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
