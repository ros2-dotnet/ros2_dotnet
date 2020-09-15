cmake_minimum_required(VERSION 3.5)
project(rclcppdotnet)

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

set(CSHARP_TARGET_FRAMEWORK "netstandard2.0")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dotnet_cmake_module REQUIRED)
find_package(ament_cmake_export_assemblies REQUIRED)
find_package(dotnet_cmake_module REQUIRED)
find_package(DotNETExtra REQUIRED)
find_package(rcldotnet_common REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

set(CS_SOURCES
  TransformListener.cs
  RclCppDotnet.cs
)

foreach(_assembly_dep ${rcldotnet_common_ASSEMBLIES_DLL})
  list(APPEND _assembly_deps_dll "${_assembly_dep}")
endforeach()

add_dotnet_library(${PROJECT_NAME}_assemblies
  SOURCES
  ${CS_SOURCES}
  INCLUDE_DLLS
  ${_assembly_deps_dll}
)

install_dotnet(${PROJECT_NAME}_assemblies DESTINATION lib/${PROJECT_NAME}/dotnet)
ament_export_assemblies_dll("lib/${PROJECT_NAME}/dotnet/${PROJECT_NAME}_assemblies.dll")

set(_native_sources "rclcppdotnet;transform_listener")

foreach(_target_name ${_native_sources})
  add_library(${_target_name}_native SHARED
    ${_target_name}.cpp
  )
  ament_target_dependencies(${_target_name}_native
    "rclcpp"
    "tf2_ros"
    "geometry_msgs"
  )
  ament_export_libraries(${_target_name}_native)

  install(TARGETS ${_target_name}_native
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endforeach()

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
