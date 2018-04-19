@echo off

echo "===== BUILDING BRANCH: %1 ====="

md \dev\ament\src
cd \dev\ament
curl -sk https://raw.githubusercontent.com/esteve/ros2_dotnet/%1/ament_dotnet_uwp.repos -o ament_dotnet_uwp.repos
vcs import src < ament_dotnet_uwp.repos
python src\ament\ament_tools\scripts\ament.py build --cmake-args -G "Visual Studio 15 2017 Win64" -DMSBUILD_EXECUTABLE="C:\Program Files (x86)\Microsoft Visual Studio\2017\Enterprise\MSBuild\15.0\Bin\MSBuild.exe" --

call \dev\ament\install\local_setup.bat
md \dev\ros2\src
cd \dev\ros2
curl -sk https://raw.githubusercontent.com/esteve/ros2_dotnet/%1/ros2_dotnet_uwp.repos -o ros2_dotnet_uwp.repos
vcs import src < ros2_dotnet_uwp.repos
cd \dev\ros2\src\ros2\rosidl_typesupport
patch -p1 < ..\..\ros2_dotnet/ros2_dotnet/rosidl_typesupport_ros2_uwp.patch
cd \dev\ros2
ament build --cmake-args -G "Visual Studio 15 2017" -DMSBUILD_EXECUTABLE="C:\Program Files (x86)\Microsoft Visual Studio\2017\Enterprise\MSBuild\15.0\Bin\MSBuild.exe" -DCMAKE_SYSTEM_NAME=WindowsStore -DCMAKE_SYSTEM_VERSION=10.0 -DTHIRDPARTY=ON -DCOMPILE_EXAMPLES=OFF -DCMAKE_FIND_ROOT_PATH="\dev\ament\install;\dev\ros2\install" --
