# SEAN 3 Unity migration

This development branch targets Unity 2022.3.62f3 and ROS-TCP-Connector v0.7.0.
The ROS 2 migration is in progress. The Warehouse scene passes short Nav2
navigation checks at startup, after task reset, and after scene reload. This
headless smoke test uses no pedestrians; camera rendering and interactive study
workflows still need validation. ORCA remains unimplemented.

The connector is embedded under `Packages/com.unity.robotics.ros-tcp-connector`.
Its [patch record](Packages/com.unity.robotics.ros-tcp-connector/SEAN-PATCHES.md)
identifies the upstream revision and scene-lifecycle fixes. Scene destruction
now cancels the old connection worker and removes its HUD header. Use the
matching ROS integration's `sean_tcp_endpoint` for subscriber removal support.

## Pedestrian robot response

`IVI.SFAgent.robotResponse` selects the versioned `ColliderV1` model (default)
or `Legacy` for comparison. ColliderV1 uses physical colliders belonging to the
selected robot body, actual robot velocity, braking that survives the legacy
force limits, and sidestepping away from the body. Sidestepping starts within a bounded three-metre surface range, with the legacy
slow-walking side-force gain and a smooth onset. Pedestrian desired/max speed,
person/person forces and wall-force parameters retain their previous values.
An exactly centred passing-side tie keeps the legacy tangent bias. This is an
engineered response model, not a calibration against human observations.

`IVI.SFAgent.contactModel` independently selects `VelocityBodyV1` (default)
or `Legacy` contacts before Start. The new contact model applies the existing
zero-friction velocity-body material to the animated body's owned physical
colliders. The original floor friction fought root motion and made the same
no-robot control take 12.44 s at 0.02 s steps versus 16.52 s at 0.04 s; the
contact-material ablation took 9.96 s at both. This changes realized walking
pace without changing the desired-speed parameter. It models an ideal driven
capsule, not foot traction or calibrated human dynamics. A real animated-wall
regression checks collision blocking.

For the untouched baseline select both `robotResponse=Legacy` and
`contactModel=Legacy`. To isolate force-model effects, compare Legacy and
ColliderV1 with the same VelocityBodyV1 contacts. Future pedestrian controllers
must share the same contact/motion model in a controlled comparison.

The opt-in `-pedestrian-validation-output /absolute/path/report.json` diagnostic
uses production Kuri and animated pedestrians on a separate open floor in the
Warehouse player. It controls frame/physics steps explicitly and records raw
trajectories, independent horizontal body clearance and physical contacts.
`-pedestrian-validation-model Legacy` retains the old response for comparison;
`-pedestrian-validation-contact Legacy` selects original contacts;
`-pedestrian-validation-modes absent` requests control-only cases.
`-pedestrian-validation-seeds 7,42` and `-pedestrian-validation-step 0.02` select
the fixture parameters. Without the output flag the diagnostic is not installed
and does not change simulation stepping. See the coordination repository's
`PEDESTRIAN_AVOIDANCE.md` and dated result for acceptance and limitations.

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

For the navigation smoke test, use `-scene SEAN/Warehouse -scenario None
-task RandomABNav -seed 7 -external-goals`. The external-goal flag suppresses
task-generated goals while allowing a ROS action client to drive navigation.
The task timeout should exceed the test duration. `-frame-rate` defaults to 60.

Publish `true` to `/social_sim/control/task/new` to reset a task, or to
`/social_sim/control/scene/reload` to reload the active scene. Both controls are
disabled in evaluation mode. Velocity commands are cleared on task reset, and
the simulation clock keeps its epoch across scene reloads.

Before collecting study results, also verify participant control, recording,
collision measurements, and the required camera rendering. Use the matching
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

The separate opt-in `-warehouse-encounter-output /absolute/path/encounters.json`
fixture uses the actual Warehouse floor, racks and baked NavMesh with four
production animated pedestrians. It compares absent, parked, oracle-yielding,
oblivious and real Nav2 conditions in fixed mirrored aisle/rack-end layouts.
It exposes the population through the existing ROS agent publishers; Nav2
uses normal laser/odometry streams and actions. Completed people remain at
ordered terminal goals. Dense terminal queues are not qualified by this fixture.

Use the coordination repository's `scripts/check_warehouse_encounters.sh` and
`WAREHOUSE_ENCOUNTERS.md` for the external Nav2 handshake, full measurements,
acceptance rules and outcomes. Its controlled stepping and planar robot
constraints apply only when explicitly enabled. Normal scenarios retain their
existing task/control behavior. A successful diagnostic subset is not a full
populated-simulator release qualification.
