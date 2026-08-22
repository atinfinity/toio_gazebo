cmake_minimum_required(VERSION 3.22)
project(toio_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)

# Gazebo system plugin driving the indicator LED of the cube
add_library(ToioLedSystem SHARED src/toio_led_system.cpp)
target_include_directories(ToioLedSystem
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(ToioLedSystem PUBLIC gz-sim::gz-sim)

install(
  TARGETS ToioLedSystem
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  PROGRAMS
    scripts/toio_led_node.py
    scripts/toio_sound_node.py
    scripts/toio_motion_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Makes ToioLedSystem loadable by Gazebo when the workspace is sourced
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_copyright also requires CONTRIBUTING.md and a LICENSE file in a
  # layout it recognizes; skip it (the copyright test is skipped in
  # toio_description and toio_ros2 as well)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_toio_sound_node test/test_toio_sound_node.py)
  ament_add_pytest_test(test_toio_motion_node test/test_toio_motion_node.py)
endif()

ament_package()
