cmake_minimum_required(VERSION 3.5)

project(rcldotnet_examples NONE)

find_package(ament_cmake REQUIRED)
find_package(rcldotnet REQUIRED)

find_package(dotnet_cmake_module REQUIRED)
find_package(DotNETExtra MODULE)

find_package(ros2_dotnet_utils REQUIRED)

find_package(std_msgs REQUIRED)
find_package(rcldotnet REQUIRED)

set(CS_SOURCES
  Program.cs
)

add_assemblies("rcldotnet_talker"
  "${CS_SOURCES}"
  OUTPUT_TYPE "Exe"
  INCLUDE_ASSEMBLIES_DLL
  ${ros2_dotnet_utils_ASSEMBLIES_DLL}
  ${rcldotnet_ASSEMBLIES_DLL}
  ${std_msgs_ASSEMBLIES_DLL}
  INCLUDE_ASSEMBLIES_NUGET
  ${ros2_dotnet_utils_ASSEMBLIES_NUGET}
  ${rcldotnet_ASSEMBLIES_NUGET}
  ${std_msgs_ASSEMBLIES_NUGET}
)

install_assemblies("rcldotnet_talker" COPY_DEPENDENCIES DESTINATION "share/${PROJECT_NAME}/dotnet")

ament_package()
