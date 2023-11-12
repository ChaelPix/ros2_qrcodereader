cmake_minimum_required(VERSION 3.8)
project(ros2_qrcodereader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED zbar)

include_directories(${ZBAR_INCLUDE_DIRS})
link_directories(${ZBAR_LIBRARY_DIRS})
add_definitions(${ZBAR_CFLAGS_OTHER})

add_executable(qr_code_reader src/qr_code_reader.cpp)
ament_target_dependencies(qr_code_reader rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(qr_code_reader ${ZBAR_LIBRARIES})

add_executable(qr_image_publisher src/qr_image_publisher.cpp)
ament_target_dependencies(qr_image_publisher rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(qr_image_publisher ${ZBAR_LIBRARIES})

install(TARGETS
  qr_code_reader qr_image_publisher
  DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY launch imgs src
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

