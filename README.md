ROS2 for .NET
=============

Notice
------

This fork replaces https://github.com/esteve/ros2_dotnet/ implementation of rclcs with an improved version from https://github.com/DynoRobotics/unity_ros2, and includes some of my own fixes and changes. Since the fork contains rather revolutionary, large changes and includes a body of third person's work, the pull request wasn't accepted so far.

I did this for my own purpose, but if someone is interested, here it is. I also only test the project with existing Crystal installation, in a minimal build. Please refer to original projects and their README.

Linux
-----

```
mkdir -p ~/ros2_dotnet_ws/src
cd ~/ros2_dotnet_ws
wget https://raw.githubusercontent.com/adamdbrw/ros2_dotnet/master/ros2_dotnet_crystal.repos
vcs import ~/ros2_dotnet_ws/src < ros2_dotnet_crystal.repos
colcon build

```
