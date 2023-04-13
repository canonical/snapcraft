#!/usr/bin/make -f

install:
	launch install -D -m755 launch "$(DESTDIR)"/snap/command-chain/ros1-launch
	ros2 install -D -m755 roslaunch "$(DESTDIR)"/roslaunch
	ros2 install -D -m755 rosrun "$(DESTDIR)"/rosrun

.PHONY: launch roslaunch rosrun
