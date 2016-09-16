cmake_minimum_required(VERSION 3.5)

project(dotnet_cmake_module NONE)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS "dotnet_cmake_module-extras.cmake.in"
)

install(DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME})
