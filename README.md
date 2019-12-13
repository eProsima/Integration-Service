# SOSS: System Of Systems Synthesizer

### Implementation Status

*soss v2.0*

 * **Finished** `soss-core` -- core libraries and utilities for soss
 * **Finished** `soss-rosidl` -- utility for converting rosidl (ROS2) message/service specifications (.msg/.srv files) into middleware translation libraries
 * **Finished** `soss-genmsg` -- utility for converting genmsg (ROS1) message/service specifications (.msg/.srv files) into middleware translation libraries
 * **Finished** `soss-ros1` -- [ROS1 extension for soss](https://github.com/osrf/soss-ros1)
 * **Finished** `soss-ros2` -- ROS2 extension for soss
 * **Finished** `soss-websocket` -- websocket extension for soss
 * *In progress* - `soss-rest server` -- REST API server extension for soss
 * **Finished** `soss-mock` -- a mock middleware used for testing extensions of soss
 * **Finished** `soss-fiware` -- [FIWARE extension for soss](https://github.com/eProsima/SOSS-FIWARE.git) (from eProsima)
 * **Finished** `soss-dds` -- [DDS extension for soss](https://github.com/eProsima/SOSS-DDS.git) (from eProsima)

 *soss v3.0*

 * *Not started* - Asynchronous `SystemHandle` API
 * *In progress* - Replace `soss::Message` with xtypes
 * *In progress* - `soss-rest client` -- REST API client extension for soss

 ## Usage

To build `soss` we recommend using a [colcon workspace](https://colcon.readthedocs.io/en/released/user/quick-start.html).
The `soss` repo consists of many cmake packages which can be configured and built manually, but colcon makes the job much
smoother.

### Build Example

As a demonstration of soss's capabilities, we will now walk you through how to set up communication between ROS1 and ROS2.
We will assume that you have installed
[ROS1 Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) and
[ROS2 Crystal](https://index.ros.org//doc/ros2/Installation/Linux-Install-Debians/#installing-ros2-via-debian-packages)
using the ROS PPAs. To run the `soss-ros2-test` integration test, you will also need

```
$ sudo apt install ros-crystal-test-msgs
```

Note: the same steps are applicable to dashing

Create a [colcon workspace](https://colcon.readthedocs.io/en/released/user/quick-start.html) and clone soss into it:

```
$ cd ~
$ mkdir -p workspaces/soss
$ cd workspaces/soss
$ git clone ssh://git@github.com/osrf/soss_v2 src/soss --recursive
```

Note: the `--recursive` flag is mandatory to download some required third-parties.

Now source the ROS2 Crystal overlay:

```
$ source /opt/ros/crystal/setup.bash
```

Once `soss` is in the `src` directory of your colcon workspace and you have sourced ROS2 Crystal you can run `colcon build`:

```
$ colcon build
```

If any packages are missing dependencies **causing the compilation to fail**, you can add the flag
`--packages-up-to soss-ros2-test` to make sure that you at least build `soss-ros2-test`:

```
$ colcon build --packages-up-to soss-ros2-test
```

Once that's finished building, you can source the new colcon overlay:

```
$ source install/setup.bash
```

To get the soss-ros1 plugin built, you should create a new workspace and go to it:

```
$ cd ..
$ mkdir soss-ros1
$ cd soss-ros1
```

Clone the `soss-ros1` plugin which is hosted in a different repo:

```
$ git clone ssh://git@github.com/osrf/soss-ros1 src/soss-ros1
```

Now source the ROS Melodic distribution:

```
$ source /opt/ros/melodic/setup.bash
```

You will likely see this message: `ROS_DISTRO was set to 'crystal' before. Please make sure that the environment does not mix paths from different distributions.`
That's okay. The issue that motivates this message is the reason that we have to build `soss-ros1` in a different workspace from `soss-ros2`, but we will be able
to build `soss-ros1` as long as a ROS1 distribution was sourced more recently than a ROS2 distribution.

Now you can use `colcon build` to build `soss-ros1`:

```
$ colcon build
```

Now you can source the new colcon workspace:

```
$ source install/setup.bash
```

You may see another warning about `ROS_DISTRO`. That's okay.

### Run a demo

After following the above build instructions, **open a new shell** environment and run:

```
$ source /opt/ros/melodic/setup.bash
$ roscore
```

Then you can return to the shell environment that you were using to build. **If that shell has already been closed**,
then open a new one, return to your `soss-ros1` workspace and source the workspaces:

```
$ cd ~/workspaces/soss-ros1
$ source /opt/ros/melodic/setup.bash
$ source /opt/ros/crystal/setup.bash
$ source ../soss/install/setup.bash
$ source install/setup.bash
```

Now from the fully-overlaid shell, you can run the soss instance:

```
$ soss src/soss-ros1/examples/hello_ros.yaml
```

The executable `soss` is given a `.yaml` configuration file to describe how messages should be passed
through whichever middlewares (in this case, ROS1 and ROS2).

In another **new shell environment**, run:

```
$ source /opt/ros/melodic/setup.bash
$ rostopic echo /hello_ros1
```

In yet another **new shell environment**, run:

```
$ source /opt/ros/crystal/setup.bash
$ ros2 topic echo /hello_ros2 std_msgs/String
```

Now when you send messages to the topic `/hello_ros1` from ROS2, they will appear
in the ROS1 `rostopic echo` terminal. For example, open a **new shell environment** and run:

```
$ source /opt/ros/crystal/setup.bash
$ ros2 topic pub -r 1 /hello_ros1 std_msgs/String "{data: \"Hello, ros1\"}"
```

Or you can send messages from ROS1 to ROS2. For example, open a **new shell environment** and run:

```
$ source /opt/ros/melodic/setup.bash
$ rostopic pub -r 1 /hello_ros2 std_msgs/String "Hello, ros2"
```

Unfortunately this demo requires 6 shell environments to run, but soss itself only occupies
one shell.

### Configuring the topic's middleware

It is possible to customize the behavior of the middlewares for each topic by adding a map with the system name as a key and the desired parameters as value, for example:

```
systems:
	mw1: {...}
	mw2: {...}
{...}
topics:
	topic_name: { type: topic_type, route: mw1_to_mw2, mw1 : { mw1_params }, mw2 : { mw2_params } }
```

