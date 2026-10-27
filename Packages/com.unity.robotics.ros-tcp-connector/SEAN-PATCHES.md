# Embedded connector provenance

Source: Unity-Technologies/ROS-TCP-Connector, tag `v0.7.0`, commit
`c27f00c6cf750d2d0564349b3039d19aa3925e7c`, package directory
`com.unity.robotics.ros-tcp-connector`. The upstream Apache-2.0 license is
included in `LICENSE.md`. Unity loads this embedded package instead of the
Git dependency so clean checkouts include the same lifecycle fixes.

Local changes:

- `ROSConnection.OnDestroy` cancels the connection task, unregisters its HUD
  header, and releases its singleton reference. Scene replacement previously
  left the old task reconnecting and retained its header delegate.
- `Connect` ignores a duplicate request while a connection task is active.
- Socket exceptions after cancellation are treated as shutdown, rather than
  reported as connection failures. Other transport failures remain visible.
- `HudPanel.UnregisterHeader` removes only the matching delegate, preserving
  headers registered by other components.

Regression coverage lives in `Assets/Tests/Editor/MigrationTests.cs`.
Keep this patch list current when updating the embedded dependency.
