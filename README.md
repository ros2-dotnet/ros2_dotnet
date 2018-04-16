ROS2 for .NET
=============

Build status
------------
[![Build status](https://ros2-dotnet.visualstudio.com/_apis/public/build/definitions/3da961d8-6dea-4d3f-88c1-f4802b69960c/4/badge)](https://ros2-dotnet.visualstudio.com/ros2-dotnet/_build/index?definitionId=4)

Introduction
------------

This is a collection of projects (bindings, code generator, examples and more) for writing ROS2
applications for .NET Core and .NET Standard 2.0

Features
--------

The current set of features include:
- Generation of all builtin ROS types
- Support for publishers and subscriptions
- Cross-platform support (Linux, Windows, Windows IoT Core, UWP)

What's missing?
---------------

Lots of things!
- Nested types
- Component nodes
- Clients and services
- Tests
- Documentation
- More examples (e.g. IoT, VB, UWP, HoloLens, etc.)

Sounds great, how can I try this out?
-------------------------------------

First of all install the standard ROS2 dependencies for your operating system of choice https://github.com/ros2/ros2/wiki/Installation#building-from-source

Next make sure you've either installed .Net Core (preferred) https://www.microsoft.com/net/learn/get-started or Mono https://www.mono-project.com/

The following steps show how to build the examples:

```
mkdir -p ~/ros2_dotnet_ws/src
cd ~/ros2_dotnet_ws
wget https://raw.githubusercontent.com/esteve/ros2_dotnet/master/ros2_dotnet.repos
vcs import ~/ros2_dotnet_ws/src < ros2_dotnet.repos
cd ~/ros2_dotnet_ws/src/ros2/rosidl_typesupport
patch -p1 < ../../ros2_dotnet/ros2_dotnet/rosidl_typesupport_ros2_dotnet.patch
cd ~/ros2_dotnet_ws
src/ament/ament_tools/scripts/ament.py build --isolated
```

Now you can just run a bunch of examples.

### Publisher and subscriber

Publisher:

```
. ~/ros2_dotnet_ws/install_isolated/local_setup.sh

ros2 run rcldotnet_examples rcldotnet_talker
```

Subscriber:

```
. ~/ros2_dotnet_ws/install_isolated/local_setup.sh

ros2 run rcldotnet_examples rcldotnet_listener
```

Enjoy!
