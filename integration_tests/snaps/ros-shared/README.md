Shared ROS Demo
===============

This demo is actually made up of two snaps:

  - **ros-base**: A minimal base ROS system. This includes `roscore` as well as
    typical utilities like `rosrun` and `roslaunch`.
  - **ros-app**: A more streamlined ROS snap that doesn't include the components
    included in ros-base, but contains a talker/listener system. It requires
	the staging area from ros-base to be tarred up and used as a part during
	build-time, and requires ros-base to be installed and sharing its content
	during run-time.


## Build procedure

See the Makefile for explicit steps, but the general idea is:

1. Build ros-base snap.
2. Tar ros-base staging area: `tar czf ros-base.tar.bz2 stage/`
3. Copy that tarball into ros-app (as required by its `ros-base` part).
4. Build ros-app.
