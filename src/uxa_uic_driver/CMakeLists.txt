cmake_minimum_required(VERSION 3.8)
project(uxa_uic_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(uxa_serial_msgs REQUIRED)
find_package(uxa_uic_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(uxa_uic_driver src/uxa_uic_driver.cpp src/uic_packet.cpp)
ament_target_dependencies(uxa_uic_driver rclcpp uxa_serial_msgs uxa_uic_msgs std_msgs)

target_include_directories(uxa_uic_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS
    uxa_uic_driver
    DESTINATION lib/${PROJECT_NAME})

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
