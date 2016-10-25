Warning!
========

This is far from complete, if you want fully functional bindings .NET for ROS2, check out @firesurfer 's excellent [rclcs](https://github.com/firesurfer/rclcs).

What works so far
-----------------

- Generation of non-nested messages
- Support for Windows (tested on Windows 10 and Visual Studio Community)
- Publishers (see https://github.com/esteve/ros2_dotnet/blob/devel/rcldotnet_examples/Program.cs)

TODO
----

- The first version of this had support for .NET Core on Linux, but when I added support for Windows, I most certainly broke it.
- Nested messages (i.e. custom messages containing other custom messages)
- Subscriptions
- Clients
- Services
