cmake_minimum_required(VERSION 3.8)
project(simple_cap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

pkg_check_modules(avcodec REQUIRED libavcodec)
pkg_check_modules(avutil REQUIRED libavutil)
pkg_check_modules(swscale REQUIRED libswscale)

if(EXISTS ${avcodec})
  message(STATUS "Found libavcodec: ${avcodec}")
endif()

if(EXISTS ${avutil})
  message(STATUS "Found libavutil: ${avutil}")
endif()

if(EXISTS ${swscale})
  message(STATUS "Found libswscale: ${swscale}")
endif()

add_executable(simple_cap_node src/simple_cap_node.cpp)
ament_target_dependencies(simple_cap_node 
  rclcpp 
  sensor_msgs
)

target_link_libraries(simple_cap_node
  ${avcodec_LIBRARIES}
  ${avutil_LIBRARIES}
  ${swscale_LIBRARIES}
)

install(TARGETS
  simple_cap_node
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
