# Using Snappy with ROS

The [Robot Operating System][1] (ROS) is an open-source collection of tools and
libraries meant to aid in the development of complex robotic systems. ROS is
excellent at what it does, but there are a few things it doesn't do:

- Security: How does one prevent a rogue application from interfering with one's
  finely-honed ROS ecosystem? Confinement and access control isn't one of the
  many problems solved by ROS.

- Deployment: How does one deploy an entire ROS project in one step, without
  worrying about dependencies?

- Updating: How does one allow end-users to update the entire ROS project in one
  step, while retaining for the possibility of rolling back the change if
  the update goes sideways?

These are all problems solved by Snappy; the combination of the two is a perfect
match for consumer robotic systems.


## Package your current ROS project as a .snap

Let's assume you already have a ROS project. It can be as simple or as
complicated as you like, but for this example, our project will be made up of:

- A ROS package containing a C++ "talker."
- A ROS package containing a Python "listener," as well as a launch file to
  bring both the talker and listener up at the same time with roscore.

Our objective will be to create a .snap containing these pieces and their
dependencies. The easiest way to do that is with Snapcraft, using its Catkin
plugin. But before we get to that, let's create our project. Prerequisites for
this walkthrough:

- Installed Snapcraft (see [Getting Started][2]).
- Installed/configured ROS Indigo (see the [Indigo installation tutorial][3]).
- An empty ROS workspace (see the Catkin [workspace tutorial][4]).
- General ROS experience (at least go through the [C++ pub/sub tutorial][5]).
- General Snapcraft experience (read [Your first snap][6]).


### C++ "talker"

First, create the package:

    $ catkin_create_pkg talker roscpp std_msgs

Now make sure the `package.xml` is setup correctly (comments removed for
brevity):

```xml
<?xml version="1.0"?>
<package>
  <name>talker</name>
  <version>0.0.0</version>
  <description>The talker package</description>
  <maintainer email="ubuntu@todo.todo">ubuntu</maintainer>
  <license>TODO</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>std_msgs</run_depend>
</package>
```

Also make sure the `CMakeLists.txt` is setup correctly. Importantly, make sure
that you've specified install rules, since Snapcraft uses `make install`.
Anything that doesn't have an install rule won't end up in the final .snap:

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(talker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(talker_node src/talker_node.cpp)

target_link_libraries(talker_node ${catkin_LIBRARIES})

install(TARGETS talker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

Finally, for `src/talker_node.cpp`, we have this:

```cpp
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle nodeHandle;

    ros::Publisher publisher = nodeHandle.advertise<std_msgs::String>("chatter", 1);

    ros::Rate loopRate(10);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String message;

        std::stringstream stream;
        stream << "Hello world " << count++;
        message.data = stream.str();

        ROS_INFO("%s", message.data.c_str());

        publisher.publish(message);

        ros::spinOnce();

        loopRate.sleep();
    }

    return 0;
}
```

The "talker" is now complete.


### The Python "listener"

First, create the package:

    $ catkin_create_pkg listener rospy std_msgs

Now make sure the `package.xml` is setup correctly (comments removed for
brevity):

```xml
<?xml version="1.0"?>
<package>
  <name>listener</name>
  <version>0.0.0</version>
  <description>The listener package</description>
  <maintainer email="ubuntu@todo.todo">ubuntu</maintainer>
  <license>TODO</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <run_depend>rospy</run_depend>
  <run_depend>std_msgs</run_depend>
</package>
```

Also make sure the `CMakeLists.txt` is setup correctly. Again, it's important to remember the install rules:

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(listener)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

install(PROGRAMS
  scripts/listener_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  talk_and_listen.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

Now for `scripts/listener_node`, we have this:

```python
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo('I heard %s', data.data)

def listener():
    rospy.init_node('listener')

    rospy.Subscriber('babble', String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
```

Make sure this script is executable:

    $ chmod +x scripts/listener_node

Note that the listener subscribes to the `babble` topic, but the talker
publishes on `chatter`. We need to make sure we account for that in
`talk_and_listen.launch`:

```xml
<launch>
    <node name = "talker" pkg = "talker" type = "talker_node"
          output = "screen" />

    <node name = "listener" pkg = "listener" type = "listener_node"
          output = "screen">
        <remap from = "babble" to = "chatter" />
    </node>
</launch>
```

The "listener" is now complete.


### Verify functionality

You can verify that everything works by getting in the workspace root and
running:

    $ catkin_make
    $ catkin_make install
    $ roslaunch listener talk_and_listen.launch

You should see them communicating, looking something like this:

    ...
    [INFO] [WallTime: 1449512660.316140] I heard Hello world 2
    [ INFO] [1449512660.415941529]: Hello world 3
    [INFO] [WallTime: 1449512660.416330] I heard Hello world 3
    [ INFO] [1449512660.515954882]: Hello world 4
    [INFO] [WallTime: 1449512660.516307] I heard Hello world 4
    [ INFO] [1449512660.615954473]: Hello world 5
    [INFO] [WallTime: 1449512660.616306] I heard Hello world 5
    ...

### Put it all in a .snap

Let's get back to our stated objective, which was to create a .snap containing
these pieces and their dependencies. As mentioned, the easiest way to create a
.snap with complex dependencies is to use Snapcraft. Snapcraft contains a plugin
especially for Catkin, which makes creating a ROS .snap particularly easy.

We tell Snapcraft how to create the .snap via a file named `snapcraft.yaml`;
let's create that file in the workspace root, containing the following:

```yaml
name: ros-example
version: 1.0
summary: ROS Example
description: Contains talker/listener ROS packages and a .launch file.

apps:
  launch-project:
    command: roslaunch listener talk_and_listen.launch
    uses: [listener]

uses:
  listener:
    type: migration-skill
    caps: [network-listener]

parts:
  ros-project:
    plugin: catkin
    source: .
    catkin-packages:
      - talker
      - listener
    include-roscore: true
```

Most of this file should look familiar to you if you've met the prerequisites,
but let's focus on a few specific pieces.

```yaml
# ...
apps:
  launch-project:
    command: roslaunch listener talk_and_listen.launch
    uses: [listener]
# ...
```

Even though the `talker` and `listener` packages will be installed, they'll be
within a confined ROS installation, so the user won't be able to simply call
`rosrun` or `roslaunch`. Instead, you have control over how your .snap is used,
and here we specify that we only want a single binary, called "launch-project",
which results in the `roslaunch` call you see. If this seems confusing now, it
will make more sense when we actually use it.

```yaml
# ...
parts:
  ros-project:
    plugin: catkin
    source: .
    catkin-packages:
      - talker
      - listener
    include-roscore: true
# ...
```

This is specifying that the .snap is made up of a single part, called
"ros-project," which utilizes the Catkin plugin. It states that the workspace is
in the same path as the `snapcraft.yaml`, and it specifies which ROS packages
should be included in the .snap (`talker` and `listener`). Note that no
`stage-packages` are specified here-- dependencies are extracted from the Catkin
packages via `rosdep`. Finally, and this is important, it specifies that
`roscore` should be installed into the .snap (note that `include-roscore`
defaults to true; it doesn't need to be specified. It's done here for
illustrative purposes).

That last point is worth discussing. Currently, since .snaps cannot depend upon
each other, any .snap that uses roscore must distribute roscore within it. We're
working on some new features that will enable sharing roscore between snaps, but
until then there are some limitations to keep in mind:

- `roscore` must be bundled into each .snap that requires it.
- Snappy's port negotiation feature is still a work-in-progress, which means
  that only a single .snap that runs `roscore` can be installed at a time or
  they will fight for the same port.

Now that we understand the `snapcraft.yaml`, let's create the .snap!

    $ snapcraft snap

This will take some time to pull down the dependencies etc., but in the end
you'll have a .snap.


### Take the .snap for a test drive

You can transfer your newly-minted .snap to your Ubuntu Core machine and install
it at the same time via `snappy-remote`, for example:

    $ snappy-remote --url=ssh://<host>:<port> install \
      ros-example_1.0_amd64.snap

Now on the Ubuntu Core machine, take a look in `/snaps/bin/`, and you'll see the
binary you requested, called `ros-example.launch-project`. Test it
out:

    $ ros-example.launch-project

And you should see the talker and listener communicating like before. As usual,
ctrl+c will stop it. Note also that, since ROS is running in a confined
environment, its log isn't in `$HOME/.ros` as usual, but in
`$HOME/snaps/ros-example.sideload/<version>/ros`. Note: if you make the app a
service, `$HOME` will be `/root`, so the logs will be in
`/root/snaps/ros-example.sideload/<version>/ros`.

[1]: http://www.ros.org/
[2]: get-started.md
[3]: http://wiki.ros.org/indigo/Installation/Ubuntu
[4]: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
[5]: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
[6]: your-first-snap.md
