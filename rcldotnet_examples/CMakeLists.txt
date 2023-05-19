cmake_minimum_required(VERSION 3.5)

project(rcldotnet_examples C)

find_package(ament_cmake REQUIRED)
find_package(rcldotnet REQUIRED)

find_package(rcldotnet_common REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcldotnet REQUIRED)

find_package(dotnet_cmake_module REQUIRED)

set(CSHARP_TARGET_FRAMEWORK "netcoreapp2.0")
find_package(DotNETExtra REQUIRED)

set(_assemblies_dep_dlls
    ${rcldotnet_common_ASSEMBLIES_DLL}
    ${rcldotnet_ASSEMBLIES_DLL}
    ${std_msgs_ASSEMBLIES_DLL}
    ${std_srvs_ASSEMBLIES_DLL}
)

add_dotnet_executable(rcldotnet_talker
  RCLDotnetTalker.cs
  INCLUDE_DLLS
  ${_assemblies_dep_dlls}
)
add_dotnet_executable(rcldotnet_listener
  RCLDotnetListener.cs
  INCLUDE_DLLS
  ${_assemblies_dep_dlls}
)
add_dotnet_executable(rcldotnet_example_service
  RCLDotnetService.cs
  INCLUDE_DLLS
  ${_assemblies_dep_dlls}
)
# avoid naming conflict with rcldotnet_client library
add_dotnet_executable(rcldotnet_example_client
  RCLDotnetClient.cs
  INCLUDE_DLLS
  ${_assemblies_dep_dlls}
)

add_dotnet_executable(rcldotnet_guard_condition
  RCLDotnetGuardCondition.cs
  INCLUDE_DLLS
  ${_assemblies_dep_dlls}
)

install_dotnet(rcldotnet_talker DESTINATION lib/${PROJECT_NAME}/dotnet)
install_dotnet(rcldotnet_listener DESTINATION lib/${PROJECT_NAME}/dotnet)
install_dotnet(rcldotnet_example_service DESTINATION lib/${PROJECT_NAME}/dotnet)
install_dotnet(rcldotnet_example_client DESTINATION lib/${PROJECT_NAME}/dotnet)
install_dotnet(rcldotnet_guard_condition DESTINATION lib/${PROJECT_NAME}/dotnet)

ament_package()
