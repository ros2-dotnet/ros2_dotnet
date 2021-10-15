cmake_minimum_required(VERSION 3.5)

project(rcldotnet_common C)

#set(CSHARP_TARGET_FRAMEWORK "netstandard2.0")
set(CSHARP_TARGET_FRAMEWORK "netcoreapp2.0")

find_package(ament_cmake REQUIRED)
find_package(dotnet_cmake_module REQUIRED)
find_package(ament_cmake_export_assemblies REQUIRED)
find_package(DotNETExtra REQUIRED)

if("${CMAKE_SYSTEM_NAME}" STREQUAL "WindowsStore")
set(CS_SOURCES
  DllLoadUtils_uwp.cs
  Interfaces.cs
  RCLRet.cs
)
else()
set(CS_SOURCES
  DllLoadUtils.cs
  Interfaces.cs
  RCLRet.cs
)
endif()

add_dotnet_library(${PROJECT_NAME} ${CS_SOURCES})
install_dotnet(${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME}/dotnet)
ament_export_assemblies_dll("lib/${PROJECT_NAME}/dotnet/${PROJECT_NAME}.dll")

ament_package()
