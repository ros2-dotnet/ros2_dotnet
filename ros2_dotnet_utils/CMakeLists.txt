cmake_minimum_required(VERSION 3.5)

project(ros2_dotnet_utils NONE)

find_package(ament_cmake REQUIRED)

find_package(dotnet_cmake_module REQUIRED)
find_package(DotNETExtra MODULE)

find_package(ament_cmake_export_assemblies REQUIRED)

set(CS_SOURCES
  DllLoadUtils.cs
  RCLdotnetProxy.cs
  Interfaces.cs
)

if(WIN32)
# TODO: add code for Windows
else()
configure_file(
"${CMAKE_CURRENT_SOURCE_DIR}/project.json.dotnet"
"${CMAKE_CURRENT_BINARY_DIR}/project.json"
COPYONLY
)
endif()

add_assemblies("${PROJECT_NAME}_assemblies"
  ${CS_SOURCES}
  OUTPUT_NAME
  "${PROJECT_NAME}"
)

install_assemblies("${PROJECT_NAME}_assemblies" "share/${PROJECT_NAME}/dotnet")
ament_export_assemblies_dll("share/${PROJECT_NAME}/dotnet/${PROJECT_NAME}.dll")
ament_export_assemblies_nuget("share/${PROJECT_NAME}/dotnet/${PROJECT_NAME}.1.0.0.nupkg")

ament_package()
