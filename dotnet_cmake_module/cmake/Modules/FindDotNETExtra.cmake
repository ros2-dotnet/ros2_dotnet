# Copyright 2016 Esteve Fernandez <esteve@apache.org>
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

function(add_assemblies _TARGET_NAME)

  cmake_parse_arguments(_add_assemblies
    ""
    "OUTPUT_DIR;OUTPUT_TYPE;OUTPUT_NAME"
    "SOURCES;INCLUDE_ASSEMBLIES_DLL;INCLUDE_ASSEMBLIES_NUGET"
    ${ARGN}
  )

  set(_DOTNET_SOURCE_FILES ${_add_assemblies_SOURCES} ${_add_assemblies_UNPARSED_ARGUMENTS})

  set(OUTPUT_TYPE "${_add_assemblies_OUTPUT_TYPE}")
  if(OUTPUT_TYPE STREQUAL "")
    set(OUTPUT_TYPE "Library")
  endif()

  set(OUTPUT_NAME "${_add_assemblies_OUTPUT_NAME}")
  if(OUTPUT_NAME STREQUAL "")
    set(OUTPUT_NAME "${_TARGET_NAME}")
  endif()


if(WIN32)

  if(CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
    set(_nuget_pack_arch "x64" )
  else()
    set(_nuget_pack_arch "x86" )
  endif()

  set(_nuget_pack_configuration "${CMAKE_BUILD_TYPE}")
  if(_nuget_pack_configuration STREQUAL "")
    set(_nuget_pack_configuration "Release")
  endif()

  set(_nuget_output_dir "${CMAKE_CURRENT_BINARY_DIR}/bin/${_nuget_pack_arch}/${_nuget_pack_configuration}")
  set(_nuget_output_path "${_nuget_output_dir}/${_TARGET_NAME}.1.0.0.nupkg")
  set(_assembly_dll_output_path "${_nuget_output_dir}/${PROJECT_NAME}.dll")
  set(_assembly_nuget_output_path "${_nuget_output_dir}/${PROJECT_NAME}.1.0.0.nupkg")

  set(_assembly_exe_output_path "${_nuget_output_dir}/${_TARGET_NAME}.exe")

  find_program(NUGET_EXECUTABLE nuget)
  if(NOT NUGET_EXECUTABLE)
    message(FATAL_ERROR "NuGet cannot be found!")
  endif()

  if(("${CMAKE_SYSTEM_NAME}" STREQUAL "WindowsStore" OR "${CMAKE_SYSTEM_NAME}" STREQUAL "WindowsPhone") AND "${CMAKE_SYSTEM_VERSION}" STREQUAL "10.0")
    set(_msbuild_project_template "${dotnet_cmake_module_DIR}/../resource/msbuild_project.csproj.uwp.in")

    configure_file(${_msbuild_project_template} "${_TARGET_NAME}_msbuild.csproj" @ONLY)

    # Always regenerate project.lock.json to prevent msbuild from running NuGet automatically
    file(
      REMOVE "${CMAKE_CURRENT_BINARY_DIR}/project.lock.json"
    )

    configure_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/project.json.uwp"
      "${CMAKE_CURRENT_BINARY_DIR}/project.json"
      COPYONLY
    )

    add_custom_command(
      OUTPUT
        "${CMAKE_CURRENT_BINARY_DIR}/project.lock.json"
      COMMAND ${NUGET_EXECUTABLE}
      ARGS restore
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/"
      DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/project.json"
      COMMENT "Generating project.lock.json"
    )

    add_custom_target(
      ${_TARGET_NAME}_project_lock_json ALL
      DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/project.lock.json"
    )

    # TODO(esteve): Add if CMAKE_SYSTEM_NAME=WindowsStore ...
    include_external_msproject(${_TARGET_NAME}_msbuild ${_TARGET_NAME}_msbuild.csproj)
    add_dependencies(${_TARGET_NAME}_msbuild ${_TARGET_NAME}_project_lock_json)

    add_custom_target(${_TARGET_NAME} ALL)

    add_dependencies(${_TARGET_NAME} ${_TARGET_NAME}_msbuild)
  else()

    set(CS_SOURCES "")
    foreach(_cs_src ${_DOTNET_SOURCE_FILES})
      get_filename_component(_cs_abs_path "${_cs_src}" ABSOLUTE)

      set(CS_SOURCES "${CS_SOURCES}<Compile Include=\"${_cs_abs_path}\"/>")
    endforeach()

    set(ASSEMBLY_DLL_DEPENDENCIES "")
    foreach(_dll_dep ${_add_assemblies_INCLUDE_ASSEMBLIES_DLL})
      get_filename_component(_dll_dep_name "${_dll_dep}" NAME_WE)
      set(ASSEMBLY_DLL_DEPENDENCIES "${ASSEMBLY_DLL_DEPENDENCIES}<Reference Include=\"${_dll_dep_name}\"><HintPath>${_dll_dep}</HintPath></Reference>")
    endforeach()

    set(_msbuild_project_template "${dotnet_cmake_module_DIR}/../resource/msbuild_project.csproj.dotnet.in")

    configure_file(${_msbuild_project_template} "${_TARGET_NAME}_msbuild.csproj" @ONLY)

    include_external_msproject(${_TARGET_NAME}_msbuild ${_TARGET_NAME}_msbuild.csproj PLATFORM "")

    set(_nuget_pack_args
      "pack"
      "${_TARGET_NAME}_msbuild.csproj"
      "-Version" "1.0.0"
      "-Properties" "Configuration=${_nuget_pack_configuration}$<SEMICOLON>Platform=${_nuget_pack_arch}"
      "-OutputDirectory" "${_nuget_output_dir}"
    )

    add_custom_target(
      ${_TARGET_NAME}_fake ALL
      DEPENDS "${_nuget_output_path}"
    )

    add_custom_command(
      OUTPUT "${_nuget_output_path}"
      COMMAND ${NUGET_EXECUTABLE}
      ARGS ${_nuget_pack_args}
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
      COMMENT "Generating NuGet"
      DEPENDS ${_TARGET_NAME}_msbuild
    )

    add_custom_target(
      ${_TARGET_NAME}_pack ALL
      DEPENDS "${_nuget_output_path}"
    )

    add_dependencies(${_TARGET_NAME}_pack ${_TARGET_NAME}_msbuild)

    add_custom_target(${_TARGET_NAME} ALL DEPENDS ${_nuget_output_path})
  endif()
else()
  set(_assembly_nuget_output_path "${CMAKE_CURRENT_BINARY_DIR}/bin/${PROJECT_NAME}.dll")
  set(_assembly_nuget_output_path "${CMAKE_CURRENT_BINARY_DIR}/bin/${PROJECT_NAME}.1.0.0.nupkg")

  find_program(DOTNET_EXECUTABLE dotnet)
  if(NOT DOTNET_EXECUTABLE)
    message(FATAL_ERROR "dotnet executable cannot be found!")
  endif()

  set(NUGET_DEPENDENCIES "")
  set(ASSEMBLY_DEPENDENCIES "")

  foreach(_dep_assembly ${_add_assemblies_INCLUDE_ASSEMBLIES_NUGET})
    get_filename_component(_nuget_dep_dir "${_dep_assembly}" DIRECTORY)
    get_filename_component(_nuget_dep_package "${_dep_assembly}" NAME_WE)
    set(NUGET_DEPENDENCIES "${NUGET_DEPENDENCIES}<add key=\"${_nuget_dep_package}\" value=\"${_nuget_dep_dir}\" />")
    set(ASSEMBLY_DEPENDENCIES "${ASSEMBLY_DEPENDENCIES},\"${_nuget_dep_package}\": \"1.0.0\"")
  endforeach()

  if(EXISTS "${PROJECT_SOURCE_DIR}/project.json.dotnet.in")
    configure_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/project.json.dotnet.in"
      "${CMAKE_CURRENT_BINARY_DIR}/project.json"
      @ONLY
    )
  elseif(EXISTS "${PROJECT_SOURCE_DIR}/project.json.dotnet")
    configure_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/project.json.dotnet"
      "${CMAKE_CURRENT_BINARY_DIR}/project.json"
      COPYONLY
    )
  endif()

  foreach(_cs_src ${_DOTNET_SOURCE_FILES})
    if(NOT IS_ABSOLUTE ${_cs_src})
      configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/${_cs_src}"
        "${CMAKE_CURRENT_BINARY_DIR}/${_cs_src}"
        COPYONLY
      )
    endif()
  endforeach()


  configure_file(
    "${dotnet_cmake_module_DIR}/../resource/nuget.config.in"
    "${CMAKE_CURRENT_BINARY_DIR}/nuget.config"
    @ONLY
  )

  add_custom_command(
    OUTPUT
      "${CMAKE_CURRENT_BINARY_DIR}/project.lock.json"
    COMMAND ${DOTNET_EXECUTABLE}
    ARGS restore
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/"
    DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/project.json;${CMAKE_CURRENT_BINARY_DIR}/nuget.config"
    COMMENT "Generating project.lock.json"
  )

  add_custom_target(
    ${_TARGET_NAME}_project_lock_json ALL
    DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/project.lock.json"
  )

  add_custom_command(
    OUTPUT "${_TARGET_NAME}_fake"
    COMMAND ${DOTNET_EXECUTABLE}
    ARGS build "${CMAKE_CURRENT_BINARY_DIR}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/"
    DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/project.lock.json"
    COMMENT "Building dotnet project"
  )

  add_custom_target(
    ${_TARGET_NAME}_build ALL
    DEPENDS "${_TARGET_NAME}_fake"
  )

  add_dependencies(${_TARGET_NAME}_build ${_TARGET_NAME}_project_lock_json)

  add_custom_command(
    OUTPUT "${_assembly_nuget_output_path}"
    COMMAND ${DOTNET_EXECUTABLE}
    ARGS pack "${CMAKE_CURRENT_BINARY_DIR}" -c "${CMAKE_BUILD_TYPE}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/"
    COMMENT "Generating NuGet"
  )

  add_custom_target(
    ${_TARGET_NAME}_pack ALL
    DEPENDS "${_assembly_nuget_output_path}"
  )

  add_dependencies(${_TARGET_NAME}_pack ${_TARGET_NAME}_build)

  add_custom_target(${_TARGET_NAME} ALL DEPENDS ${_TARGET_NAME}_pack)


endif()

  if(OUTPUT_TYPE STREQUAL "Library")

    set_property(
        TARGET
            ${_TARGET_NAME}
        PROPERTY
            INSTALL_FILES
                ${_assembly_dll_output_path}
                ${_assembly_nuget_output_path}
    )

    set_property(
        TARGET
            ${_TARGET_NAME}
        PROPERTY
            ASSEMBLIES_NUGET_FILE
                ${_assembly_nuget_output_path}
    )

    set_property(
        TARGET
            ${_TARGET_NAME}
        PROPERTY
            ASSEMBLIES_DLL_FILE
                ${_assembly_dll_output_path}
    )
  else()
    set_property(
        TARGET
            ${_TARGET_NAME}
        PROPERTY
            ASSEMBLIES_EXE_FILE
                ${_assembly_exe_output_path}
    )
  endif()

  set_property(
    TARGET ${_TARGET_NAME}
    PROPERTY DEPENDENCIES_DLL ${_add_assemblies_INCLUDE_ASSEMBLIES_DLL}
  )

endfunction()

function(install_assemblies _TARGET_NAME)
    if (ARGC EQUAL 2)
      set (_DESTINATION ${ARGV1})
    else()
      cmake_parse_arguments(_install_assemblies
        "COPY_DEPENDENCIES"
        "DESTINATION;COMPONENT"
        ""
        ${ARGN})
      if (_install_assemblies_DESTINATION)
        set (_DESTINATION ${_install_assemblies_DESTINATION})
      else()
        message(SEND_ERROR "install_assemblies: ${_TARGET_NAME}: DESTINATION must be specified.")
      endif()

      if (_install_assemblies_COMPONENT)
        set (_COMPONENT COMPONENT ${_install_assemblies_COMPONENT})
      endif()
    endif()

    get_property(__EXE_FILE
        TARGET
            ${_TARGET_NAME}
        PROPERTY
            ASSEMBLIES_EXE_FILE
    )

    get_property(__DLL_FILE
        TARGET
            ${_TARGET_NAME}
        PROPERTY
            ASSEMBLIES_DLL_FILE
    )

    get_property(__FILES
        TARGET
            ${_TARGET_NAME}
        PROPERTY
            INSTALL_FILES
    )

    get_property(__DEP_DLLS
        TARGET
            ${_TARGET_NAME}
        PROPERTY
            DEPENDENCIES_DLL
    )

    set_property(
        TARGET
            ${_TARGET_NAME}
        PROPERTY
            INSTALL_DESTINATION
            ${_DESTINATION}
    )

    if (__DEP_DLLS)
        install(
            FILES
                ${__DEP_DLLS}
            DESTINATION
                bin
        )
    endif()

    if (__EXE_FILE)
        install(
            FILES
                ${__EXE_FILE}
            DESTINATION
                bin
        )
    endif()


    if (__DLL_FILE)
        install(
            FILES
                ${__DLL_FILE}
            DESTINATION
                bin
        )
    endif()

    if (__FILES)
        install(
            FILES
                ${__FILES}
            DESTINATION
                ${_DESTINATION}
        )
    endif()
    if (NOT __FILES AND NOT __DLL_FILE AND NOT __EXE_FILE)
        message(SEND_ERROR "install_assemblies: The target ${_TARGET_NAME} is not known in this scope.")
    endif ()
endfunction()
