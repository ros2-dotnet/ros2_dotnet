# Copyright 2016-2018 Esteve Fernandez <esteve@apache.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

find_package(ament_cmake_export_assemblies REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)

find_package(dotnet_cmake_module REQUIRED)
find_package(DotNETExtra REQUIRED)

# Get a list of typesupport implementations from valid rmw implementations.
rosidl_generator_dotnet_get_typesupports(_typesupport_impls)

if(_typesupport_impls STREQUAL "")
  message(WARNING "No valid typesupport for .NET generator. .NET messages will not be generated.")
  return()
endif()

set(_output_path
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_dotnet/${PROJECT_NAME}")
set(_generated_cs_files "")
set(_generated_c_ts_files "")
set(_generated_h_files "")

foreach(_abs_idl_file ${rosidl_generate_interfaces_ABS_IDL_FILES})
  get_filename_component(_parent_folder "${_abs_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)

  get_filename_component(_idl_name "${_abs_idl_file}" NAME_WE)
  string_camel_case_to_lower_case_underscore("${_idl_name}" _module_name)

  if(_parent_folder STREQUAL "msg")
    list(APPEND _generated_cs_files
      "${_output_path}/${_parent_folder}/${_module_name}.cs"
      )

    list(APPEND _generated_h_files
      "${_output_path}/${_parent_folder}/rcldotnet_${_module_name}.h"
      )

    list(APPEND _generated_c_ts_files
      "${_output_path}/${_parent_folder}/${_module_name}.c"
      )
  elseif(_parent_folder STREQUAL "srv")
    list(APPEND _generated_cs_files
      "${_output_path}/${_parent_folder}/${_module_name}.cs"
      )

    list(APPEND _generated_h_files
      "${_output_path}/${_parent_folder}/rcldotnet_${_module_name}.h"
      )

    list(APPEND _generated_c_ts_files
      "${_output_path}/${_parent_folder}/${_module_name}.c"
      )
  elseif(_parent_folder STREQUAL "action")
    list(APPEND _generated_cs_files
      "${_output_path}/${_parent_folder}/${_module_name}.cs"
      )

    list(APPEND _generated_h_files
      "${_output_path}/${_parent_folder}/rcldotnet_${_module_name}.h"
      )

    list(APPEND _generated_c_ts_files
      "${_output_path}/${_parent_folder}/${_module_name}.c"
      )
  else()
    message(FATAL_ERROR "Interface file with unknown parent folder: ${_idl_file}")
  endif()
endforeach()

set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_IDL_FILES})
    set(_abs_idl_file "${${_pkg_name}_DIR}/../${_idl_file}")
    normalize_path(_abs_idl_file "${_abs_idl_file}")
    list(APPEND _dependency_files "${_abs_idl_file}")
    list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
  endforeach()
endforeach()

set(target_dependencies
  "${rosidl_generator_dotnet_BIN}"
  ${rosidl_generator_dotnet_GENERATOR_FILES}
  "${rosidl_generator_dotnet_TEMPLATE_DIR}/action.c.em"
  "${rosidl_generator_dotnet_TEMPLATE_DIR}/action.cs.em"
  "${rosidl_generator_dotnet_TEMPLATE_DIR}/action.h.em"
  "${rosidl_generator_dotnet_TEMPLATE_DIR}/idl.c.em"
  "${rosidl_generator_dotnet_TEMPLATE_DIR}/idl.cs.em"
  "${rosidl_generator_dotnet_TEMPLATE_DIR}/idl.h.em"
  "${rosidl_generator_dotnet_TEMPLATE_DIR}/msg.c.em"
  "${rosidl_generator_dotnet_TEMPLATE_DIR}/msg.cs.em"
  "${rosidl_generator_dotnet_TEMPLATE_DIR}/msg.h.em"
  "${rosidl_generator_dotnet_TEMPLATE_DIR}/srv.c.em"
  "${rosidl_generator_dotnet_TEMPLATE_DIR}/srv.cs.em"
  "${rosidl_generator_dotnet_TEMPLATE_DIR}/srv.h.em"
  ${rosidl_generate_interfaces_ABS_IDL_FILES}
  ${_dependency_files})
foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    message(FATAL_ERROR "Target dependency '${dep}' does not exist")
  endif()
endforeach()

set(generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_dotnet__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  IDL_TUPLES "${rosidl_generate_interfaces_IDL_TUPLES}"
  ROS_INTERFACE_DEPENDENCIES "${_dependencies}"
  OUTPUT_DIR "${_output_path}"
  TEMPLATE_DIR "${rosidl_generator_dotnet_TEMPLATE_DIR}"
  TARGET_DEPENDENCIES ${target_dependencies}
)

set(_target_suffix "__dotnet")

set_property(
  SOURCE
  ${_generated_cs_files} ${_generated_h_files} ${_generated_c_ts_files}
  PROPERTY GENERATED 1)

# This if is only needed because srvs/actions are currently skipped
if(_generated_cs_files)
  add_custom_command(
    OUTPUT ${_generated_cs_files} ${_generated_h_files} ${_generated_c_ts_files}
    COMMAND ${PYTHON_EXECUTABLE} ${rosidl_generator_dotnet_BIN}
    --generator-arguments-file "${generator_arguments_file}"
    --typesupport-impls "${_typesupport_impls}"
    DEPENDS ${target_dependencies}
    COMMENT "Generating C# code for ROS interfaces"
    VERBATIM
  )

  if(TARGET ${rosidl_generate_interfaces_TARGET}${_target_suffix})
    message(WARNING "Custom target ${rosidl_generate_interfaces_TARGET}${_target_suffix} already exists")
  else()
    add_custom_target(
      ${rosidl_generate_interfaces_TARGET}${_target_suffix}
      DEPENDS
      ${_generated_cs_files}
      ${_generated_h_files}
      ${_generated_c_ts_files}
    )
  endif()
endif()

macro(set_properties _build_type)
  set_target_properties(${_target_name} PROPERTIES
    COMPILE_OPTIONS "${_extension_compile_flags}"
    LIBRARY_OUTPUT_DIRECTORY${_build_type} ${_output_path}
    RUNTIME_OUTPUT_DIRECTORY${_build_type} ${_output_path}
    OUTPUT_NAME ${_target_name}_native)
endmacro()

# This if is only needed because srvs/actions are currently skipped
if(_generated_c_ts_files)
  set(_dotnetext_suffix "__dotnetext")
  set(_target_name "${rosidl_generate_interfaces_TARGET}${_dotnetext_suffix}")
  add_library(${_target_name} SHARED
    "${_generated_c_ts_files}"
    "${_generated_h_files}"
  )
  target_link_libraries(${_target_name}
    ${rosidl_generate_interfaces_TARGET}__rosidl_generator_c)
  add_dependencies(
    ${_target_name}
    ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    ${rosidl_generate_interfaces_TARGET}__rosidl_typesupport_c
  )

  set(_extension_compile_flags "")
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(_extension_compile_flags -Wall -Wextra)
  endif()
  set_properties("")
  if(WIN32)
    set_properties("_DEBUG")
    set_properties("_MINSIZEREL")
    set_properties("_RELEASE")
    set_properties("_RELWITHDEBINFO")
  endif()

  set(_extension_link_flags "")
  if(NOT WIN32)
    if(CMAKE_COMPILER_IS_GNUCXX)
      set(_extension_link_flags "-Wl,--no-undefined")
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      set(_extension_link_flags "-Wl,-undefined,error")
    endif()
  endif()
  target_link_libraries(
    ${_target_name}
    ${PROJECT_NAME}__rosidl_typesupport_c
    ${_extension_link_flags}
  )
  target_include_directories(${_target_name}
    PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_dotnet
  )

  rosidl_target_interfaces(${_target_name}
    ${rosidl_generate_interfaces_TARGET} rosidl_typesupport_c)

  ament_target_dependencies(${_target_name}
    "rosidl_runtime_c"
    "rosidl_typesupport_c"
    "rosidl_typesupport_interface"
  )
  foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
    ament_target_dependencies(${_target_name}
      ${_pkg_name}
    )
  endforeach()

  ament_target_dependencies(${_target_name}
    "rosidl_runtime_c"
    "rosidl_generator_dotnet"
  )

  if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
    install(TARGETS ${_target_name}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
  endif()
endif()

set(_assembly_deps_dll "")
set(_assembly_deps_nuget "")

find_package(rcldotnet_common REQUIRED)
foreach(_assembly_dep ${rcldotnet_common_ASSEMBLIES_NUGET})
  list(APPEND _assembly_deps_nuget "${_assembly_dep}")
  get_filename_component(_assembly_filename ${_assembly_dep} NAME_WE)
endforeach()

foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  find_package(${_pkg_name} REQUIRED)
  foreach(_assembly_dep ${${_pkg_name}_ASSEMBLIES_NUGET})
    list(APPEND _assembly_deps_nuget "${_assembly_dep}")
    get_filename_component(_assembly_filename ${_assembly_dep} NAME_WE)
  endforeach()
endforeach()

find_package(rcldotnet_common REQUIRED)
foreach(_assembly_dep ${rcldotnet_common_ASSEMBLIES_DLL})
  list(APPEND _assembly_deps_dll "${_assembly_dep}")
endforeach()

foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  find_package(${_pkg_name} REQUIRED)
  foreach(_assembly_dep ${${_pkg_name}_ASSEMBLIES_DLL})
    list(APPEND _assembly_deps_dll "${_assembly_dep}")
  endforeach()
endforeach()

# This if is only needed because srvs/actions are currently skipped
if(_generated_cs_files)
  add_dotnet_library(${PROJECT_NAME}_assemblies
    SOURCES
    ${_generated_cs_files}
    INCLUDE_DLLS
    ${_assembly_deps_dll}
  )

 add_dependencies("${PROJECT_NAME}_assemblies" "${rosidl_generate_interfaces_TARGET}${_target_suffix}")
endif()

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  if(NOT _generated_h_files STREQUAL "")
    install(
      FILES ${_generated_h_files}
      DESTINATION "include/${PROJECT_NAME}/msg"
    )
  endif()

  set(_install_assembly_dir "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}")
  if(NOT _generated_cs_files STREQUAL "")
    list(GET _generated_cs_files 0 _msg_file)
    get_filename_component(_msg_package_dir "${_msg_file}" DIRECTORY)
    get_filename_component(_msg_package_dir "${_msg_package_dir}" DIRECTORY)

    install_dotnet(${PROJECT_NAME}_assemblies DESTINATION "lib/${PROJECT_NAME}/dotnet")
    ament_export_assemblies_dll("lib/${PROJECT_NAME}/dotnet/${PROJECT_NAME}_assemblies.dll")
  endif()
endif()
