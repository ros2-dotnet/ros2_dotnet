cmake_minimum_required(VERSION 3.5)

project(rcldotnet_tests C)

find_package(ament_cmake REQUIRED)
find_package(rcldotnet REQUIRED)

find_package(rcldotnet_common REQUIRED)

find_package(std_msgs REQUIRED)
find_package(test_msgs REQUIRED)
find_package(rcldotnet REQUIRED)

find_package(dotnet_cmake_module REQUIRED)

set(CSHARP_TARGET_FRAMEWORK "netcoreapp2.0")
find_package(DotNETExtra REQUIRED)

set(_assemblies_dep_dlls
    ${rcldotnet_common_ASSEMBLIES_DLL}
    ${rcldotnet_ASSEMBLIES_DLL}
    ${std_msgs_ASSEMBLIES_DLL}
    ${test_msgs_ASSEMBLIES_DLL}
    ${rcldotnet_ASSEMBLIES_DLL}
)

message("Included assemblies: ${_assemblies_dep_dlls}")

add_dotnet_test(test_messages
  test_messages.cs
  INCLUDE_DLLS
  ${_assemblies_dep_dlls}
  INCLUDE_REFERENCES
  "Microsoft.NET.Test.Sdk=15.9.0"
  "xunit=2.4.1"
  "xunit.runner.visualstudio=2.4.1"
)

install_dotnet(test_messages DESTINATION lib/${PROJECT_NAME}/dotnet)

ament_package()
