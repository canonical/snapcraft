all: ros-app/*.snap

ros-base/*.snap:
	cd ros-base && snapcraft

ros-app/ros-base.tar.bz2: ros-base/*.snap
	tar czf ros-app/ros-base.tar.bz2 -C ros-base stage/

ros-app/*.snap: ros-app/ros-base.tar.bz2
	cd ros-app && snapcraft

.PHONY: clean
clean:
	cd ros-base && snapcraft clean
	cd ros-app && snapcraft clean
	rm -f ros-app/*.tar.bz2 ros-app/*.snap ros-base/*.snap
