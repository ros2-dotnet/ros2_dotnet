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

find_package(rmw_implementation_cmake REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_generator_c REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)

find_package(PythonInterp 3.5 REQUIRED)

find_package(ament_cmake_export_assemblies REQUIRED)
find_package(dotnet_cmake_module REQUIRED)
find_package(DotNETExtra REQUIRED)

# Get a list of typesupport implementations from valid rmw implementations.
rosidl_generator_cs_get_typesupports(_typesupport_impls)

if(_typesupport_impls STREQUAL "")
  message(WARNING "No valid typesupport for .NET generator. .NET messages will not be generated.")
  return()
endif()

set(_output_path
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cs/${PROJECT_NAME}")
set(_generated_extension_files "")
set(_generated_msg_cs_files "")
set(_generated_msg_c_files "")
set(_generated_srv_cs_files "")
set(_generated_srv_c_files "")
set(_generated_action_py_files "")
set(_generated_action_c_files "")

# set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-undefined,error")

foreach(_typesupport_impl ${_typesupport_impls})
  set(_generated_extension_${_typesupport_impl}_files "")
endforeach()

foreach(_idl_file ${rosidl_generate_interfaces_IDL_FILES})
  get_filename_component(_parent_folder "${_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_msg_name1 "${_idl_file}" NAME_WE)
  get_filename_component(_ext "${_idl_file}" EXT)
  string_camel_case_to_lower_case_underscore("${_msg_name1}" _module_name)

  if(_parent_folder STREQUAL "msg")
    list(APPEND _generated_msg_cs_files
      "${_output_path}/${_parent_folder}/_${_module_name}.cs"
    )
    list(APPEND _generated_msg_c_files
      "${_output_path}/${_parent_folder}/_${_module_name}_s.c"
    )
  elseif(_parent_folder STREQUAL "srv")
    if("_${_module_name}_s.c" MATCHES "(.*)__response(.*)" OR "_${_module_name}_s.c" MATCHES "(.*)__request(.*)")
      list(APPEND _generated_srv_c_files
        "${_output_path}/${_parent_folder}/_${_module_name}_s.c"
      )
    endif()
    list(APPEND _generated_srv_cs_files
      "${_output_path}/${_parent_folder}/_${_module_name}.cs"
    )
  elseif(_parent_folder STREQUAL "action")
    # C files generated for <msg>.msg, <service>_Request.msg and <service>_Response.msg but not <service>.srv
    if(_ext STREQUAL ".msg")
      list(APPEND _generated_action_c_files
        "${_output_path}/${_parent_folder}/_${_module_name}_s.c"
      )
    endif()
    list(APPEND _generated_action_cs_files
      "${_output_path}/${_parent_folder}/_${_module_name}.cs"
    )
  else()
    message(FATAL_ERROR "Interface file with unknown parent folder: ${_idl_file}")
  endif()
endforeach()

file(MAKE_DIRECTORY "${_output_path}")

if(NOT _generated_msg_c_files STREQUAL "" OR NOT _generated_srv_c_files STREQUAL "" OR NOT _generated_action_c_files STREQUAL "")
    foreach(_typesupport_impl ${_typesupport_impls})
      list(APPEND _generated_extension_${_typesupport_impl}_files "${_output_path}/_${PROJECT_NAME}_s.ep.${_typesupport_impl}.c")
      list(APPEND _generated_extension_files "${_generated_extension_${_typesupport_impl}_files}")
    endforeach()
endif()

set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_INTERFACE_FILES})
    set(_abs_idl_file "${${_pkg_name}_DIR}/../${_idl_file}")
    normalize_path(_abs_idl_file "${_abs_idl_file}")
    list(APPEND _dependency_files "${_abs_idl_file}")
    list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
  endforeach()
endforeach()

set(target_dependencies
  "${rosidl_generator_cs_BIN}"
  ${rosidl_generator_cs_GENERATOR_FILES}
  "${rosidl_generator_cs_TEMPLATE_DIR}/_msg_support.c.em"
  "${rosidl_generator_cs_TEMPLATE_DIR}/_msg_pkg_typesupport_entry_point.c.em"
  "${rosidl_generator_cs_TEMPLATE_DIR}/_msg.cs.em"
  "${rosidl_generator_cs_TEMPLATE_DIR}/_srv.cs.em"
  ${_dependency_files})
foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    message(FATAL_ERROR "Target dependency '${dep}' does not exist")
  endif()
endforeach()

set(generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cs__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  ROS_INTERFACE_FILES "${rosidl_generate_interfaces_IDL_FILES}"
  ROS_INTERFACE_DEPENDENCIES "${_dependencies}"
  OUTPUT_DIR "${_output_path}"
  TEMPLATE_DIR "${rosidl_generator_cs_TEMPLATE_DIR}"
  TARGET_DEPENDENCIES ${target_dependencies}
)

if(NOT _generated_msg_cs_files STREQUAL "")
  list(GET _generated_msg_cs_files 0 _msg_file)
  get_filename_component(_msg_package_dir1 "${_msg_file}" DIRECTORY)
  get_filename_component(_msg_package_dir2 "${_msg_package_dir1}" NAME)
endif()

if(NOT _generated_srv_cs_files STREQUAL "")
  list(GET _generated_srv_cs_files 0 _srv_file)
  get_filename_component(_srv_package_dir1 "${_srv_file}" DIRECTORY)
  get_filename_component(_srv_package_dir2 "${_srv_package_dir1}" NAME)
endif()

if(NOT _generated_action_cs_files STREQUAL "")
  list(GET _generated_action_cs_files 0 _action_file)
  get_filename_component(_action_package_dir1 "${_action_file}" DIRECTORY)
  get_filename_component(_action_package_dir2 "${_action_package_dir1}" NAME)
endif()


set(_target_suffix "__cs")

# move custom command into a subdirectory to avoid multiple invocations on Windows
set(_subdir "${CMAKE_CURRENT_BINARY_DIR}/${rosidl_generate_interfaces_TARGET}${_target_suffix}")
file(MAKE_DIRECTORY "${_subdir}")
file(READ "${rosidl_generator_cs_DIR}/custom_command.cmake" _custom_command)
file(WRITE "${_subdir}/CMakeLists.txt" "${_custom_command}")
add_subdirectory("${_subdir}" ${rosidl_generate_interfaces_TARGET}${_target_suffix})
set_property(
  SOURCE
  ${_generated_extension_files} ${_generated_msg_cs_files} ${_generated_msg_c_files} ${_generated_srv_cs_files} ${_generated_srv_c_files} ${_generated_action_cs_files} ${_generated_action_c_files}
  PROPERTY GENERATED 1)

# TODO(samiam): add set_properties and set_lib_properties macros

set(_target_name_lib "${rosidl_generate_interfaces_TARGET}__csharp_native")
add_library(${_target_name_lib} SHARED
  ${_generated_msg_c_files}
  ${_generated_srv_c_files}
  ${_generated_action_c_files}
)
add_dependencies(
  ${_target_name_lib}
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
  ${rosidl_generate_interfaces_TARGET}__rosidl_typesupport_c
)

target_link_libraries(
  ${_target_name_lib}
)

target_include_directories(${_target_name_lib}
  PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cs
)

rosidl_target_interfaces(${_target_name_lib}
  ${rosidl_generate_interfaces_TARGET} rosidl_typesupport_c)

foreach(_typesupport_impl ${_typesupport_impls})
  find_package(${_typesupport_impl} REQUIRED)

  set(_csext_suffix "__csext_native")
  set(_target_name "${PROJECT_NAME}__${_typesupport_impl}${_csext_suffix}")

  add_library(${_target_name} SHARED
    ${_generated_extension_${_typesupport_impl}_files}
  )
  add_dependencies(
    ${_target_name}
    ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    ${rosidl_generate_interfaces_TARGET}__rosidl_typesupport_c
  )

  set(_extension_compile_flags "")

  target_link_libraries(
    ${_target_name}
    ${_target_name_lib}
    ${PythonExtra_LIBRARIES}
    ${rosidl_generate_interfaces_TARGET}__${_typesupport_impl}
  )

  target_include_directories(${_target_name}
    PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
    ${PythonExtra_INCLUDE_DIRS}
  )

  rosidl_target_interfaces(${_target_name}
    ${rosidl_generate_interfaces_TARGET} rosidl_typesupport_c)

  ament_target_dependencies(${_target_name}
    "rosidl_generator_c"
    "rosidl_typesupport_c"
    "rosidl_typesupport_interface"
  )

  foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
    ament_target_dependencies(${_target_name}
      ${_pkg_name}
    )
  endforeach()

  add_dependencies(${_target_name}
    ${rosidl_generate_interfaces_TARGET}__${_typesupport_impl}
  )
  ament_target_dependencies(${_target_name}
    "rosidl_generator_c"
    "rosidl_generator_py"
    "${rosidl_generate_interfaces_TARGET}__rosidl_generator_c"
  )

 # NOTE(sam): Sourcing for Unity only seems to work if destination in bin, not lib
  if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
    install(TARGETS ${_target_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  endif()

endforeach()

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  install(TARGETS ${_target_name_lib}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
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

add_dotnet_library(${PROJECT_NAME}_assembly
  SOURCES
  ${_generated_msg_cs_files}
  ${_generated_srv_cs_files}
  INCLUDE_DLLS
  ${_assembly_deps_dll}
)

add_dependencies("${PROJECT_NAME}_assembly" "${rosidl_generate_interfaces_TARGET}${_target_suffix}")

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  if(NOT _generated_msg_h_files STREQUAL "")
    install(
      FILES ${_generated_msg_h_files}
      DESTINATION "include/${PROJECT_NAME}/msg"
    )
  endif()

  set(_install_assembly_dir "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}")
  if(NOT _generated_msg_cs_files STREQUAL "")
    list(GET _generated_msg_cs_files 0 _msg_file)
    get_filename_component(_msg_package_dir "${_msg_file}" DIRECTORY)
    get_filename_component(_msg_package_dir "${_msg_package_dir}" DIRECTORY)

    install_dotnet(${PROJECT_NAME}_assembly DESTINATION "lib/dotnet")
    ament_export_assemblies_dll("lib/dotnet/${PROJECT_NAME}_assembly.dll")
  endif()
endif()
