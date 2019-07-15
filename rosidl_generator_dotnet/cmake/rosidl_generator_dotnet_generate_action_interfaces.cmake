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

# Get a list of typesupport implementations from valid rmw implementations.
rosidl_generator_dotnet_get_typesupports(_typesupport_impls)

set(_output_path
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_dotnet/${PROJECT_NAME}")
set(_generated_files "")

foreach(_idl_file ${rosidl_generate_action_interfaces_IDL_FILES})
  message(STATUS "ROS C# ACTION processing ${_idl_file}")
  get_filename_component(_extension "${_idl_file}" EXT)
  get_filename_component(_parent_folder "${_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  if(_extension STREQUAL ".action")
    set(_allowed_parent_folders "action")
    if(NOT _parent_folder IN_LIST _allowed_parent_folders)
      message(FATAL_ERROR "Interface file with unknown parent folder: ${_idl_file}")
    endif()
  else()
    message(FATAL_ERROR "Interface file with unknown extension: ${_idl_file}")
  endif()
  get_filename_component(_module_name "${_idl_file}" NAME_WE)
  string_camel_case_to_lower_case_underscore("${_module_name}" _header_name)
  list(APPEND _generated_files
    "${_output_path}/${_parent_folder}/${_module_name}.cs"
  )
endforeach()

set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_INTERFACE_FILES})
    get_filename_component(_extension "${idl_file}" EXT)
    if(_extension STREQUAL ".msg")
      set(_abs_idl_file "${${_pkg_name}_DIR}/../${_idl_file}")
      normalize_path(_abs_idl_file "${_abs_idl_file}")
      list(APPEND _dependency_files "${_abs_idl_file}")
      list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
    endif()
  endforeach()
endforeach()

set(target_dependencies
  "${rosidl_generator_dotnet_BIN}"
  ${rosidl_generator_dotnet_GENERATOR_FILES}
  "${rosidl_generator_dotnet_TEMPLATE_DIR}/action.cs.em"
  ${rosidl_generate_action_interfaces_IDL_FILES}
  ${_dependency_files})
foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    get_property(is_generated SOURCE "${dep}" PROPERTY GENERATED)
    if(NOT ${_is_generated})
      message(FATAL_ERROR "Target dependency '${dep}' does not exist")
    endif()
  endif()
endforeach()

set(generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_dotnet__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  ROS_INTERFACE_FILES "${rosidl_generate_action_interfaces_IDL_FILES}"
  ROS_INTERFACE_DEPENDENCIES "${_dependencies}"
  OUTPUT_DIR "${_output_path}"
  TEMPLATE_DIR "${rosidl_generator_dotnet_TEMPLATE_DIR}"
  TARGET_DEPENDENCIES ${target_dependencies}
)

add_custom_command(
  OUTPUT ${_generated_files}
  COMMAND ${PYTHON_EXECUTABLE} ${rosidl_generator_dotnet_BIN}
  --generator-arguments-file "${generator_arguments_file}"
  --typesupport-impls "${_typesupport_impls}"
  DEPENDS ${target_dependencies}
  COMMENT "Generating C# code for ROS interfaces"
  VERBATIM
)

set(_target_suffix "__dotnet__actions")

if(TARGET ${rosidl_generate_action_interfaces_TARGET}${_target_suffix})
  message(WARNING "Custom target ${rosidl_generate_interfaces_TARGET}${_target_suffix} already exists")
else()
  add_custom_target(
    ${rosidl_generate_action_interfaces_TARGET}${_target_suffix}
    DEPENDS
    ${_generated_files}
  )
endif()

add_dependencies(
  ${rosidl_generate_action_interfaces_TARGET}
  ${rosidl_generate_action_interfaces_TARGET}${_target_suffix}
)

if(NOT rosidl_generate_action_interfaces_SKIP_INSTALL)
  if(NOT _generated_files STREQUAL "")
    install(
      FILES ${_generated_files}
      DESTINATION "include/${PROJECT_MAME}/action"
    )
  endif()
  ament_export_include_directories(include)
endif()
