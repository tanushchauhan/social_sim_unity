# SEAN 3 Unity migration

This development branch targets Unity 2022.3.62f3 and ROS-TCP-Connector v0.7.0.
The ROS 2 migration is in progress. A complete Unity-to-Nav2 navigation run has
not yet been validated in this checkout. ORCA remains unimplemented.

## Migration checks

Open the project with the pinned editor, let package import finish, and run
`SEAN.Editor.Tests` in Window > General > Test Runner > EditMode. These checks
cover body-frame velocity, angular wraparound, sensor mounts under rotation,
and publisher registration after replacing a connection. They require Unity;
they do not establish end-to-end navigation or sensor fidelity.

Odometry reports ground-truth pose in `map` and body-frame twist in `base_link`,
with angular velocity in radians per second. The Jackal and Kuri prefabs request
a 0.05-second publication interval. Task changes clear velocity history; callers
that teleport the robot outside the task system must call `ResetOdometry()`.

Before collecting results, also verify a live navigation goal, participant
control, recording, repeated task resets, and scene reloads. Use the matching
scene and ROS map. The default ROS `lab` map does not match the enabled scenes
in the current player build settings.

Documentation: [sean.interactive-machines.com](https://sean.interactive-machines.com)

This is the [Unity](https://unity.com) project for SEAN: Social Environment for Autonomous Navigation

## Source Repositories

Other repositories for the project are:

  - Catkin Workspace: https://github.com/yale-sean/sim_ws

  - ROS project: https://github.com/yale-sean/social_sim_ros

  - Unity Project: https://github.com/yale-sean/social_sim_unity

  - Documentation: https://github.com/yale-sean/social_sim_docs


## Developer Information

### Generate documentation

```
cp README.md Documentation/index.md
docfx Documentation/docfx.json --force
```

Copy the documentation into the web documentation project.

First, set the web project directory:

```
export WEB_DIR=$HOME/src/yale/social_sim_docs
```

Then copy the `api` folder:

```
mkdir -p $WEB_DIR/static/api/unity
cp -r _site/api/* $WEB_DIR/static/api/unity/
cp -r _site/styles $WEB_DIR/static/api/
```


### Code Formatting and Linting

Try to keep the typical C# code style enforced by Visual Studio.

In case some files get committed or edited and no longer adhere to this style, the whole project can be re-styled using:

```
dotnet tool install -g dotnet-format
dotnet format social_sim_unity.sln
```
