This is a project that forks https://github.com/borune-k12/mav2xplane/ to support SIL simulation.

Developed by Robots Everywhere (https://www.robots-everywhere.com)

Any MAVLink UDP system (Ardupilot, PX4, etc) should be able to use this to connect to X-Plane 10, 11, or 12.

This software is made available as-is, no warranties explicit or implied. This is not complete, this is something
we built very quickly for an internal purpose and may have bugs outside of our immediate use case. Please report
any issues, or better yet fix them and send us a pull request!

Due to the use of qt and qGroundControl components, the GPLv3 license is required, though not made explicit by
the original project. No legal claim is intended by the author; this is a derivative work of a derivative work
of a GPLv3 licensed project.

At some point we intend to re-add the serial HIL connectivity functions back, and backport or replace the
original project. If you need support for HIL over COM at the moment, please use the original version.