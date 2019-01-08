ROS2 for .NET
=============

Build status
------------

| Target | Status |
|----------|--------|
| **Universal Windows Platform (x86)** | [![Build status](http://vsts-matrix-badges.herokuapp.com/repos/ros2-dotnet/ros2-dotnet/2/branches/master/1)](https://dev.azure.com/ros2-dotnet/ros2-dotnet/_build?definitionId=2) |
| **Universal Windows Platform (x64)** | [![Build status](http://vsts-matrix-badges.herokuapp.com/repos/ros2-dotnet/ros2-dotnet/2/branches/master/2)](https://dev.azure.com/ros2-dotnet/ros2-dotnet/_build?definitionId=2) |
| **Universal Windows Platform (ARM)** | [![Build status](http://vsts-matrix-badges.herokuapp.com/repos/ros2-dotnet/ros2-dotnet/2/branches/master/3)](https://dev.azure.com/ros2-dotnet/ros2-dotnet/_build?definitionId=2) |
| **Windows Desktop**                  | [![Build status](http://vsts-matrix-badges.herokuapp.com/repos/ros2-dotnet/ros2-dotnet/2/branches/master/4)](https://dev.azure.com/ros2-dotnet/ros2-dotnet/_build?definitionId=2) |

Introduction
------------

This is a collection of projects (bindings, code generator, examples and more) for writing ROS2
applications for .NET Core and .NET Standard.

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

Windows (Desktop)
-----------------

```
md \dev\ros2_dotnet_ws\src
cd \dev\ros2_dotnet_ws
wget https://raw.githubusercontent.com/esteve/ros2_dotnet/master/ros2_dotnet.repos
vcs import \dev\ros2_dotnet_ws\src < ros2_dotnet.repos
src\ament\ament_tools\scripts\ament.py build
```

Linux
-----

```
mkdir -p ~/ros2_dotnet_ws/src
cd ~/ros2_dotnet_ws
wget https://raw.githubusercontent.com/esteve/ros2_dotnet/master/ros2_dotnet.repos
vcs import ~/ros2_dotnet_ws/src < ros2_dotnet.repos
src/ament/ament_tools/scripts/ament.py build
```

Universal Windows Platform (ARM, Win32, Win64)
----------------------------------------------

We'll build this in two steps, first `ament` (the build system) and then ROS2 itself. This is because `ament` will run on the host platform, whereas `ros2-dotnet` on the target.

ament
-----

```
md \dev\ament\src
cd \dev\ament
curl -sk https://raw.githubusercontent.com/esteve/ros2_dotnet/%BUILD_SOURCEBRANCHNAME%/ament_dotnet_uwp.repos -o ament_dotnet_uwp.repos
vcs import src < ament_dotnet_uwp.repos
python src\ament\ament_tools\scripts\ament.py build --cmake-args -G "Visual Studio 15 2017 Win64" --
call install\local_setup.bat
```

UWP
---

Replace %TARGET_GENERATOR% with "Visual Studio 15 2017" for Win32, "Visual Studio 15 2017 Win64" for Win64 and "Visual Studio 15 2017 ARM" for ARM

```
md \dev\ros2\src
cd \dev\ros2
curl -sk https://raw.githubusercontent.com/esteve/ros2_dotnet/%BUILD_SOURCEBRANCHNAME%/ros2_dotnet_uwp.repos -o ros2_dotnet_uwp.repos
vcs import src < ros2_dotnet_uwp.repos
cd \dev\ament
call install\local_setup.bat
cd \dev\ros2
ament build --parallel --cmake-args -G "%TARGET_GENERATOR%" -DCMAKE_SYSTEM_NAME=WindowsStore -DCMAKE_SYSTEM_VERSION=10.0.14393 -DTHIRDPARTY=ON -DINSTALL_EXAMPLES=OFF -DCMAKE_FIND_ROOT_PATH="\dev\ament\install;\dev\ros2\install" --
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
