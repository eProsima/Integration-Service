# SOSS concept

SOSS is intended to be a System-Of-Systems Synthesizer. That is, if you have a
pair of complex systems, A and B, and you want to combine them to create a
larger, even more complex system, the goal is for SOSS to make that task
easier.

An underlying assumption is that the subsystems A and B use some form of
message-passing middleware. With this assumption, the idea is that by writing a
(hopefully readable) text file, you can provide a mapping between the topics
and services on System A's middleware and those on System B.

Here is the minimal example, which translates a single topic from ROS 1 to ROS 2:
```
systems:
  ros1: { type: ros1 }
  ros2: { type: ros2 }
topics:
  chatter: { type: std_msgs/String, route: {from: ros1, to: ros2} }
```

The intent is that different translations are possible by only changing the
configuration file. For example, by changing the specified middlewares, we can
obtain an instance which translates between WebSocket+JSON (as produced and
consumed by a standard Web browser) and ROS 2:
```
systems:
  web: { type: websocket_client, types-from: robot, host: localhost, port: 12345 }
  robot: { type: ros2 }
routes:
  web2robot: {from: web, to: robot}
topics:
  chatter: { type: "std_msgs/String", route: web2robot }
```

Here is a nontrivial example, which translates a number of topics and some
service clients between WebSocket+Rosbridge_v2, ROS 2, and (fictitious) automated door-opening
firmware:

```
systems:
  web: { type: websocket_server_json, types-from: ros2, port: 12345 }
  robot: { type: ros2 }
  door: { type: veridian_dynamics_proprietary_door_firmware, types-from: ros2, serial: 1765TED }

routes:
  web2robot: {from: web, to: robot}
  robot2web: {from: web, to: robot}
  door_broadcast: {from: door, to: [web, robot]}
  web_service: {server: web, clients: robot}
  door_service: {server: door, clients: [web, robot]}

topics:
  videocall_signalling_tx: { type: "rmf_msgs/SignallingMessage", route: web2robot }
  videocall_presence: { type: "std_msgs/String", route: web2robot }
  call_button_state_array: { type: "rmf_msgs/CallButtonStateArray", route: robot2web }
  videocall_signalling_rx:
    type: "rmf_msgs/SignallingMessage"
    remap: { robot: "videocall_signalling_rx/{message.message_to}" }
    route: robot2web
  door_status:
    type: "rmf_msgs/DoorStatus"
    route: door_broadcast

services:
  get_video_callers: { type: "rmf_msgs/GetVideoCallers", route: web_service }
  reserve_robot: { type: "rmf_msgs/ReserveRobot", route: web_service }
  release_robot: { type: "rmf_msgs/ReleaseRobot", route: web_service }
  open_door: { type: "rmf_msgs/OpenDoor", route: door_service }
  close_door: { type: "rmf_msgs/CloseDoor", route: door_service }
```

The idea is that each system plays some role in the overall system of systems, and we need to
specify the channels that we expect them to communicate over, as well as the direction
that information should flow over those channels. Topics can be many-to-many, one-to-many, or
many-to-one. Services must always designate one service provider, but may have one or more clients.
Some systems may have a different name for a topic or service, so the `remap` dictionary allows the
config file to specify a different name that soss should use for each system.

Here is a diagram to illustrate the concept:

![bubbles](/doc/bubbles_of_bubbles.png)

In the diagram, Robot A has a bunch of internal topics and services. It wishes
to export some (but not all) of them to a much larger collection of other
topics and services. In the process, some topic/service names will need to change,
and perhaps some other filtering will occur (for example, the rate of publishing
of its location will only be 1 Hz instead of 100 Hz, or its camera image will
be dramatically down-sampled, etc.). The SOSS configuration file will specify the
topics within Robot A that the robot needs to export, as well as what system
middlewares each exported topic needs to be forwarded to.

### Plugin Framework

The `soss-core` library defines a set of abstract interfaces and provides some utility classes
to form a plugin-based framework. A single `soss` executable instance can connect `N` number of
middlewares where each middleware has a soss-plugin associated with it. The soss-plugin for a
middleware should be a lightweight wrapper around that middleware (e.g. a ROS node or a websocket
server/client). The `soss-core` library provides cmake functions that allow these middleware
plugins to be discovered by the `soss` executable at runtime after the plugin has been installed.
Because of this, downstream users can extend `soss` to communicate with any middleware.

A single `soss` executable can route any number of topics or services to/from any number of
middlewares.

### Type definition
Some System Handles have the hability to give to soss the types definition
(using [XTypes](https://github.com/eProsima/xtypes)) that they can use.
`ros1` or `ros2` are examples of this.
Nevertheless, there are cases where the System Handle is not able to get the type specification
(`websocket`, `mock`, `dds`, `fiware`, ...) that it needs for the communication.

To give it this information to the System Handle, you have 2 ways:
- Using the `types-from` property, that _import_ the types specification from another system.
- Specify the type yourself embedded an IDL into the YAML.

Regarding to the second way, the IDL content can be provided in the YAML file as follows:

```YAML
types:
    idls:
        - >
            struct name
            {
                idl_type1 member_1_name;
                idl_type2 member_2_name;
            };

    paths: [ idl_include_path_1, idl_include_path_2 ]
```

The name for each type can be whatever the user wants, with the two following rules:

1. The name can not have spaces in it.
1. The name must be formed only by letters, numbers and underscores.

The `paths` section is optional, and it is used provide to the preprocessor a list of paths where it should search
for included IDL files within our IDL content.

The following snipped could be used to parse an IDL file instead of an IDL content.

```YAML
types:
    idls:
        - > '#include <idl_file_to_parse.idl>'

    paths: [ idl_file_to_parse_path ]
```

Note: a minimal of a structure type is required for the communication.

For more details about IDL definition, please refer to [IDL documentation](https://www.omg.org/spec/IDL/4.2/PDF).

The following is an example of a full configuration used in a dds-fiware communication using the types
definition into the IDL.

```YAML
types:
    idls:
        - >
            struct Stamp
            {
                int32 sec;
                uint32 nanosec;
            };

            struct Header
            {
                string frame_id;
                stamp stamp;
            };

systems:
    dds: { type: dds }
    fiware: { type: fiware, host: 192.168.1.59, port: 1026 }

routes:
    fiware_to_dds: { from: fiware, to: dds }
    dds_to_fiware: { from: dds, to: fiware }

topics:
    hello_dds:
      type: "Header"
      route: fiware_to_dds
    hello_fiware:
      type: "Header"
      route: dds_to_fiware
```

Note that the publisher and subscriber in the DDS world need to be configured with a compatible IDL.
That means that the type definition may differ between them.
In that case, some [QoS policies](https://github.com/eProsima/xtypes#type-consistency-qos-policies)
will enable to try to convert the type.
`soss` will notify the user with the different QoS policies enabled in the communication.
