# SOSS: System Of Systems Synthesizer

### Implementation Status

*soss v2.0*

 * **Finished** `soss-core` -- core libraries and utilities for SOSS
 * **Finished** `soss-rosidl` -- utility for converting rosidl (ROS2) message/service specifications (.msg/.srv files) into middleware translation libraries
 * **Finished** `soss-genmsg` -- utility for converting genmsg (ROS1) message/service specifications (.msg/.srv files) into middleware translation libraries
 * **Finished** `soss-ros1` -- [ROS1 extension for SOSS](https://github.com/eprosima/soss-ros1.git)
 * **Finished** `soss-ros2` -- ROS2 extension for SOSS
 * **Finished** `soss-websocket` -- websocket extension for SOSS
 * **Finished** `soss-mock` -- a mock middleware used for testing extensions of SOSS
 * **Finished** `soss-fiware` -- [FIWARE extension for SOSS](https://github.com/eProsima/SOSS-FIWARE.git)
 * **Finished** `soss-dds` -- [DDS extension for SOSS](https://github.com/eProsima/SOSS-DDS.git)
 * **Finished** `xtypes` -- Replace `soss::Message` with [eProsima xtypes](https://github.com/eProsima/xtypes.git)

 *soss v3.0*

 * **In progress** `integration-service` -- Renaming from SOSS to eProsima Integration Service
 * **Done** `soss-websocket` -- Add option to disable TLS security
 * **Done** `soss-core` -- Add extensive Doxygen documentation
 * **Done** `soss-ros2` -- Support for ROS2 Foxy
 ## Usage

To build `soss` we recommend using a [colcon workspace](https://colcon.readthedocs.io/en/released/user/quick-start.html).
The `soss` repo consists of many cmake packages which can be configured and built manually, but colcon makes the job much
smoother.

### Build Example

As a demonstration of soss's capabilities, we will now walk you through how to set up communication between ROS1 and ROS2.
We will assume that you have installed
[ROS1 Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) and
[ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html)
using the ROS PPAs.

Note: the same steps are applicable to Dashing

Download some required dependencies:

```bash
$ apt update && apt install -y libyaml-cpp-dev libboost-dev libboost-program-options-dev libssl-dev libwebsocketpp-dev
```

Create a [colcon workspace](https://colcon.readthedocs.io/en/released/user/quick-start.html) and clone SOSS into it:

```
$ cd ~
$ mkdir -p workspaces/soss
$ cd workspaces/soss
$ git clone ssh://git@github.com/eProsima/soss src/soss --recursive
```

Note: the `--recursive` flag is mandatory to download some required third-parties.

Now source the ROS2 Foxy overlay:

```
$ source /opt/ros/foxy/setup.bash
```

Once `soss` is in the `src` directory of your colcon workspace and you have sourced ROS2 Foxy you can run `colcon build`:

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
$ git clone ssh://git@github.com/eProsima/soss-ros1 src/soss-ros1 -b feature/xtypes-support
```

Now source the ROS Melodic distribution:

```
$ source /opt/ros/melodic/setup.bash
```

You will likely see this message: `ROS_DISTRO was set to 'foxy' before. Please make sure that the environment does not mix paths from different distributions.`
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
$ source /opt/ros/foxy/setup.bash
$ source ../soss/install/setup.bash
$ source install/setup.bash
```

Now from the fully-overlaid shell, you can run the SOSS instance:

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
$ source /opt/ros/foxy/setup.bash
$ ros2 topic echo /hello_ros2 std_msgs/String
```

Now when you send messages to the topic `/hello_ros1` from ROS2, they will appear
in the ROS1 `rostopic echo` terminal. For example, open a **new shell environment** and run:

```
$ source /opt/ros/foxy/setup.bash
$ ros2 topic pub -r 1 /hello_ros1 std_msgs/String "{data: \"Hello, ros1\"}"
```

Or you can send messages from ROS1 to ROS2. For example, open a **new shell environment** and run:

```
$ source /opt/ros/melodic/setup.bash
$ rostopic pub -r 1 /hello_ros2 std_msgs/String "Hello, ros2"
```

Unfortunately this demo requires 6 shell environments to run, but soss itself only occupies
one shell.
