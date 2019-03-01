cmake_minimum_required(VERSION 3.5)

project(rcldotnet C)

find_package(ament_cmake_export_assemblies REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rosidl_generator_c REQUIRED)

find_package(dotnet_cmake_module REQUIRED)
find_package(DotNETExtra REQUIRED)

find_package(rcldotnet_common REQUIRED)

if(NOT WIN32)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
  elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-undefined,error")
  endif()
endif()

set(CS_SOURCES
  INode.cs
  IPublisher.cs
  ISubscription.cs
  ISubscriptionBase.cs
  Node.cs
  Publisher.cs
  RCLdotnet.cs
  Subscription.cs
)

find_package(rcldotnet_common REQUIRED)
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

set(_native_sources "rcldotnet;rcldotnet_node;rcldotnet_publisher")

foreach(_target_name ${_native_sources})
  add_library(${_target_name} SHARED
    ${_target_name}.c
  )
  set_target_properties(${_target_name}
    PROPERTIES
    OUTPUT_NAME ${_target_name}_native)
  ament_target_dependencies(${_target_name}
    "builtin_interfaces"
    "rcl"
    "rosidl_generator_c"
    "rosidl_typesupport_c"
  )
  ament_export_libraries(${_target_name}_native)

  install(TARGETS ${_target_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endforeach()

ament_export_dependencies(ament_cmake)
ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(rcl)
ament_export_dependencies(rosidl_generator_c)
ament_export_dependencies(rosidl_typesupport_c)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(test_msgs REQUIRED)
  find_package(rcldotnet REQUIRED)

  ament_lint_auto_find_test_dependencies()

  set(_assemblies_dep_dlls
      ${rcldotnet_common_ASSEMBLIES_DLL}
      ${rcldotnet_ASSEMBLIES_DLL}
      ${std_msgs_ASSEMBLIES_DLL}
      ${test_msgs_ASSEMBLIES_DLL}
      ${rcldotnet_ASSEMBLIES_DLL}
  )

  add_dotnet_test(test_messages
    test/test_messages.cs
    INCLUDE_DLLS
    ${_assemblies_dep_dlls}
    INCLUDE_REFERENCES
    "Microsoft.NET.Test.Sdk=15.9.0"
    "xunit=2.4.1"
    "xunit.runner.visualstudio=2.4.1"
  )

  install_dotnet(test_messages DESTINATION lib/${PROJECT_NAME}/dotnet)
endif()

ament_package()
