cmake_minimum_required(VERSION 3.5)

project(rosidl_generator_dotnet)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_export_assemblies REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_generator_c REQUIRED)

find_package(rcldotnet_common REQUIRED)

ament_export_dependencies(rosidl_cmake)

ament_python_install_package(${PROJECT_NAME})

ament_index_register_resource("rosidl_generator_packages")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()

install(
  PROGRAMS bin/rosidl_generator_dotnet
  DESTINATION lib/rosidl_generator_dotnet
)

install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package(
  CONFIG_EXTRAS "cmake/rosidl_generator_dotnet_get_typesupports.cmake"
    "cmake/register_dotnet.cmake"
    "rosidl_generator_dotnet-extras.cmake.in"
)
