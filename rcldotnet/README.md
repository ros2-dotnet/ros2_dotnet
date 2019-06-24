# rclcs

This is a simple reimplementation of firesurfers C# wrapper for [ROS2](https://github.com/ros2) for the Crystal Clemmys release.
Is was deveoped primarily for use on Ubuntu 18.04 with Unity, but it sould be possible to run on Windows as well becaude I used DllLoadUtils from esteves rcldotnet_common for all linking. 

It is meant to be built using colcon, not ament_tools like previous C# wrappers.

New features include support for arrays and nested messages. [But not arrays with nested messages yet...]

I used dotnet_cmake_module from esteve, and the message genaration is done in a similar manner to ros2_dotnet.

Also, unit tests :)
