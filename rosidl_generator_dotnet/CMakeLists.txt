cmake_minimum_required(VERSION 2.8.3)

project(rosidl_generator_dotnet)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_export_assemblies REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_generator_c REQUIRED)

find_package(dotnet_cmake_module REQUIRED)
find_package(DotNETExtra MODULE)

find_package(ros2_dotnet_utils REQUIRED)

ament_export_dependencies(rosidl_cmake)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(message_files
    "msg/Bool.msg"
    "msg/Byte.msg"
    "msg/Char.msg"
    "msg/Constants.msg"
    "msg/Empty.msg"
    "msg/Float32.msg"
    "msg/Float64.msg"
    "msg/Int16.msg"
    "msg/Int32.msg"
    "msg/Int64.msg"
    "msg/Int8.msg"
    "msg/Nested.msg"
    "msg/Primitives.msg"
    "msg/Strings.msg"
    "msg/Uint16.msg"
    "msg/Uint32.msg"
    "msg/Uint64.msg"
    "msg/Uint8.msg"
    "msg/Various.msg"
  )

  include(cmake/register_dotnet.cmake)
  include(cmake/rosidl_generator_dotnet_get_typesupports.cmake)

  set(rosidl_generator_dotnet_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

  rosidl_generator_dotnet_extras(
    "${CMAKE_CURRENT_SOURCE_DIR}/bin/rosidl_generator_dotnet"
    "${CMAKE_CURRENT_SOURCE_DIR}/rosidl_generator_dotnet/__init__.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/resource"
  )

  rosidl_generate_interfaces(${PROJECT_NAME} ${message_files}
    SKIP_INSTALL
  )

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
