cmake_minimum_required(VERSION 3.5)

project(rcldotnet_common C)

find_package(ament_cmake REQUIRED)
find_package(dotnet_cmake_module REQUIRED)
find_package(ament_cmake_export_assemblies REQUIRED)
find_package(dotnet_cmake_module REQUIRED)
find_package(DotNETExtra REQUIRED)

set(CS_SOURCES
  DllLoadUtils.cs
  IRosActionDefinition.cs
  IRosMessage.cs
  IRosServiceDefinition.cs
)

add_dotnet_library(${PROJECT_NAME} ${CS_SOURCES})
install_dotnet(${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME}/dotnet)
ament_export_assemblies_dll("lib/${PROJECT_NAME}/dotnet/${PROJECT_NAME}.dll")

ament_package()
