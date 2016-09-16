cmake_minimum_required(VERSION 3.5)

project(rcldotnet C)

find_package(ament_cmake_export_assemblies REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

find_package(dotnet_cmake_module REQUIRED)
find_package(DotNETExtra MODULE)

find_package(ros2_dotnet_utils REQUIRED)

set(CS_SOURCES
  Node.cs
  Publisher.cs
  RCLdotnet.cs
  Subscription.cs
)

add_assemblies("${PROJECT_NAME}_assemblies"
  ${CS_SOURCES}
  OUTPUT_NAME
  "${PROJECT_NAME}"
  INCLUDE_ASSEMBLIES_DLL
  ${ros2_dotnet_utils_ASSEMBLIES_DLL}
  INCLUDE_ASSEMBLIES_NUGET
  ${ros2_dotnet_utils_ASSEMBLIES_NUGET}
)

install_assemblies("${PROJECT_NAME}_assemblies" "share/${PROJECT_NAME}/dotnet")
ament_export_assemblies_dll("share/${PROJECT_NAME}/dotnet/${PROJECT_NAME}.dll")
ament_export_assemblies_nuget("share/${PROJECT_NAME}/dotnet/${PROJECT_NAME}.1.0.0.nupkg")

macro(target)
  if(NOT target_suffix STREQUAL "")
    get_rcl_information("${rmw_implementation}" "rcl${target_suffix}")
  endif()
  # Only build the library if a C typesupport exists for the rmw_implementation
  get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "c")
  if(typesupport_impls STREQUAL "")
    message(STATUS "Skipping rcldotnet for '${rmw_implementation}' because no C "
      "typesupport library was found.")
    return()
  endif()

  message("Building native library for ${target_suffix}")

  set(_native_sources "rcldotnet;rcldotnet_node;rcldotnet_publisher")

  foreach(_native_source ${_native_sources})

    set(_target_name "${_native_source}${target_suffix}")

    add_library(
      ${_target_name}
      SHARED ${_native_source}.c
    )
    target_compile_definitions(${_target_name}
      PRIVATE "RMW_IMPLEMENTATION_SUFFIX=${target_suffix}")

    ament_target_dependencies(${_target_name}
      "rcl${target_suffix}"
    )

    install(TARGETS ${_target_name}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )

#  ament_export_libraries(${_target_name})

  endforeach()

endmacro()

call_for_each_rmw_implementation(target)

ament_package()
