ROS2 for .NET
=============

Build status
------------
[![Build status](https://estevefernandez.visualstudio.com/_apis/public/build/definitions/eb47ae83-8d6a-4928-9220-843167919f4f/1/badge)](https://estevefernandez.visualstudio.com/ros2-dotnet/_build/index?definitionId=1)

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

The following steps show how to build the examples on Windows and Linux:

Windows
-------

```
md \dev\ros2_dotnet_ws\src
cd \dev\ros2_dotnet_ws
wget https://raw.githubusercontent.com/esteve/ros2_dotnet/master/ros2_dotnet.repos
vcs import \dev\ros2_dotnet_ws\src < ros2_dotnet.repos
cd \dev\ros2_dotnet_ws\src\ros2\rosidl_typesupport
patch -p1 < ..\..\ros2_dotnet\ros2_dotnet\rosidl_typesupport_ros2_dotnet.patch
cd \dev\ros2_dotnet_ws
src\ament\ament_tools\scripts\ament.py build
```

Linux
-----

```
mkdir -p ~/ros2_dotnet_ws/src
cd ~/ros2_dotnet_ws
wget https://raw.githubusercontent.com/esteve/ros2_dotnet/master/ros2_dotnet.repos
vcs import ~/ros2_dotnet_ws/src < ros2_dotnet.repos
cd ~/ros2_dotnet_ws/src/ros2/rosidl_typesupport
patch -p1 < ../../ros2_dotnet/ros2_dotnet/rosidl_typesupport_ros2_dotnet.patch
cd ~/ros2_dotnet_ws
src/ament/ament_tools/scripts/ament.py build
```

Now you can just run a bunch of examples.

### Publisher and subscriber

Publisher:

Windows
-------

```
call \dev\ros2_dotnet_ws\install\local_setup.bat

ros2 run rcldotnet_examples rcldotnet_talker
```

Linux
-----

```
. ~/ros2_dotnet_ws/install/local_setup.sh

ros2 run rcldotnet_examples rcldotnet_talker
```

Subscriber:

Windows
-------

```
call \dev\ros2_dotnet_ws\install\local_setup.bat

ros2 run rcldotnet_examples rcldotnet_listener
```

Linux
-----

```
. ~/ros2_dotnet_ws/install/local_setup.sh

ros2 run rcldotnet_examples rcldotnet_listener
```

Enjoy!
