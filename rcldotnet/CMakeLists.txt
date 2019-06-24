cmake_minimum_required(VERSION 3.5)

project(rcldotnet C)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake_export_assemblies REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rmw REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rosidl_generator_c REQUIRED)

find_package(dotnet_cmake_module REQUIRED)
find_package(DotNETExtra REQUIRED)

find_package(rcldotnet_common REQUIRED)
find_package(rosidl_generator_cs REQUIRED)

# Only build the library if a C typesupport exists
get_rmw_typesupport(typesupport_impls "rmw_implementation" LANGUAGE "c")
if(typesupport_impls STREQUAL "")
  message(STATUS "Skipping rclcs because no C typesupport library was found.")
  return()
endif()

function(configure_csharp_c_extension_library _library_name)
  install(TARGETS ${_library_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endfunction()

add_library(
  rclcs_native
  SHARED src/_rclcs.c
)
configure_csharp_c_extension_library(rclcs_native)
ament_target_dependencies(rclcs_native
  "rcl"
  "rcl_yaml_param_parser"
  "rcutils"
)

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")

set(CS_SOURCES
  src/dotnet/Constants.cs
  src/dotnet/Context.cs
  src/dotnet/Clock.cs
  src/dotnet/Enums.cs
  src/dotnet/Exceptions.cs
  src/dotnet/INode.cs
  src/dotnet/IPublisher.cs
  src/dotnet/IPublisherBase.cs
  src/dotnet/ISubscription.cs
  src/dotnet/ISubscriptionBase.cs
  src/dotnet/MarshallingHelpers.cs
  src/dotnet/NativeMethods.cs
  src/dotnet/NativeTypes.cs
  src/dotnet/Node.cs
  src/dotnet/Publisher.cs
  src/dotnet/QualityOfServiceProfile.cs
  src/dotnet/Rclcs.cs
  src/dotnet/Subscription.cs
  src/dotnet/Utils.cs
  src/dotnet/WaitSet.cs
)

# TODO: automate assembly_dep list based on package dependencies?
foreach(_assembly_dep ${rcldotnet_common_ASSEMBLIES_DLL})
list(APPEND _assembly_deps_dll "${_assembly_dep}")
endforeach()

foreach(_assembly_dep ${rosidl_generator_cs_ASSEMBLIES_DLL})
list(APPEND _assembly_deps_dll "${_assembly_dep}")
endforeach()

add_dotnet_library(${PROJECT_NAME}
SOURCES
  ${CS_SOURCES}
  INCLUDE_DLLS
  ${_assembly_deps_dll}
)


install_dotnet(${PROJECT_NAME} DESTINATION lib/dotnet)
ament_export_assemblies_dll("lib/dotnet/${PROJECT_NAME}.dll")

ament_export_dependencies(ament_cmake)
ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(rcl)
ament_export_dependencies(rosidl_generator_c)
ament_export_dependencies(rosidl_typesupport_c)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
