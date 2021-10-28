<a href="https://integration-service.docs.eprosima.com"><img src="docs/images/logo.png" hspace="8" vspace="2" height="100" ></a>


# Introduction
[![Integration Service CI Status](https://github.com/eProsima/Integration-Service/actions/workflows/ci.yml/badge.svg)](https://github.com/eProsima/Integration-Service/actions)

*eProsima Integration Service* is a **Linux** tool that enables communication among
an arbitrary number of protocols that speak different languages.

This project was born as a joint effort between [Open Robotics](https://www.openrobotics.org/)
and [eProsima](https://eprosima.com), which nowadays is in charge of maintaining it.

![Integration Service general architecture](docs/images/general-architecture.png)

It works by translating the languages of the systems involved to a common representation language that follows the
[Extensible and Dynamic Topic Types for DDS](https://www.omg.org/spec/DDS-XTypes/About-DDS-XTypes/)
(**xTypes**) standard by the [OMG](https://www.omg.org/); specifically, *Integration Service*
bases its intercommunication abilities on eProsima's open source implementation
for the *xTypes* protocol, that is, [eProsima xTypes](https://github.com/eProsima/xtypes).

The translation is mediated by system-specific plugins, or **System Handles**,
allowing to communicate the middlewares involved with the **core**, that speaks the common *xTypes* representation language.
This is carried out by interfacing each entity in the user application with a complimentary
"mirror" proxy in the *System Handle* (SH), able to communicate with the former according to either a pub/sub or service pattern. Namely:

* A subscriber in the SH for a publisher in the user app.
* A publisher in the SH for a subscriber in the user app.
* A client in the SH for a server in the user app.
* A server in the SH for a client in the user app.

*Integration Service* can be launched using the command line, as follows:

```
~/is_ws$ integration-service <filename>.yaml
```

It is recommended to use [colcon](https://colcon.readthedocs.io/en/released/) to build and install the
*Integration Service* executable and its associated middleware plugins; for more information, please refer to
the `Installation manual` section in the [documentation](#documentation) chapter of this document.

# Quick start

Download [here](https://www.eprosima.com/index.php/downloads-all) the *Integration Service* docker image that contains
pre-installed *Fast DDS*, *ROS 1*, *ROS 2* and *WebSocket System Handles* and the
[documentation examples](https://integration-service.docs.eprosima.com/en/latest/examples/dockers/dockers.html). 

# Dependencies

This section provides a list of the dependencies needed in order to compile *Integration Service*.
These requirements will be different depending on the *System Handles* you want to use.

#### Integration Service Core

* [YAML-cpp](https://github.com/jbeder/yaml-cpp): A *YAML* parser and emiter for C++.
* [Boost program options](https://github.com/boostorg/program_options): Library that allows obtaining name-value pairs from the config file.

  These libraries can be installed using your Linux distribution package manager with the following command:

  ```
  sudo apt-get install -y libyaml-cpp-dev libboost-program-options-dev
  ```

#### FastDDS System Handle

* [FastDDS](https://github.com/eProsima/Fast-DDS#installation-guide): eProsima C++ implementation for *DDS*.

#### Fiware System Handle

* [Asio](https://think-async.com/Asio/): C++ library for network and low-level I/O programming.
* [cURLpp](http://www.curlpp.org/): C++ wrapper for libcURL.
* [cURL](https://curl.se/): Command-line tool for getting or sending data using URL syntax.

  These libraries can be installed using your Linux distribution package manager with the following command:

  ```
  sudo apt-get install -y libcurlpp-dev libasio-dev libcurl4-openssl-dev
  ```

#### ROS 1 System Handle

* [ROS 1](http://wiki.ros.org/ROS/Installation): *Melodic/Noetic ROS 1* distribution.

#### ROS 2 System Handle

* [ROS 2](https://docs.ros.org/en/foxy/Releases.html): *Foxy/Galactic ROS 2* distribution.

#### WebSocket System Handle

* [OpenSSL](https://www.openssl.org/): Toolkit for the Transport Layer Security (TLS) and Secure Sockets Layer (SSL) protocols.
* [WebSocket++](https://github.com/zaphoyd/websocketpp): *WebSocket* Protocol C++ library implementation.

  These libraries can be installed using your Linux distribution package manager with the following command:

  ```
  sudo apt-get install -y libssl-dev libwebsocketpp-dev
  ```

# Configuration

The *Integration Service* can be configured during runtime by means of a dedicated **YAML** file.
This configuration file must follow a specific syntax, meaning that it is required that a number
of compulsory section are opportunely filled for it to successfully configure and launch an *Integration Service* instance,
while others are optional. Both kinds are listed and reviewed below:

* `types` *(optional)*: It allows to list the [IDL](https://www.omg.org/spec/IDL/4.2/About-IDL/)
  types used by the *Integration Service* to later define the topics and services types which will
  take part in the communication process.

  This field can be omitted for certain *Integration Service* instances where one or more *System
  Handles* already include(s) static type definitions and their corresponding transformation libraries
  (*Middleware Interface Extension* or *mix* files).

  ```yaml
    types:
      idls:
        - >
          #include <GoodbyeWorld.idl>

          struct HelloWorld
          {
            string data;
            GoodbyeWorld bye;
          };
      paths: [ "/home/idl_files/goodbyeworld/" ]
  ```

  <details>
  <summary>Several parameters can be configured within this section: <i>(click to expand)</i></summary>

    * `idls`: List of IDL type definitions that can be directly embedded within the configuration file. If the `types` section is defined, this subsection is mandatory. The type can be entirely defined within the YAML file, or can be included from a preexisting IDL file; for the latter, the system path containing where the IDL file is stored must be placed into the `paths` section described below.

    * `paths` *(optional):* Using this parameter, an existing IDL type written in a separate file can be included within the *Integration Service* types section. If the IDL path is not listed here, the previous subsection `#include` preprocessor directive will fail.

  </details>

* `systems`: Specifies which middlewares will be involved in the communication process, allowing
  to configure them individually.

  Some configuration parameters are common for all the supported middlewares within the
  *Integration Service* ecosystem; while others are specific of each middleware. To see which
  parameters are relevant for a certain middleware, please refer to its dedicated *README* section
  in its corresponding GitHub repository, under the name of `https://github.com/eProsima/<MW_NAME>-SH`.

  ```yaml
    systems:
      foo: { type: foo }
      bar: { type: bar, types-from: foo }
  ```

  <details>
  <summary>In relation to the common parameters, their behaviour is explained in the following section: <i>(click to expand)</i></summary>

    * `type`: Middleware or protocol kind. To date, the supported middlewares are: *fastdds*, *fiware*, *ros1*, *ros2*, *websocket_client* and *websocket_server*. There is also a *mock* option, mostly used
    for testing purposes.

    * `types-from` *(optional)*: Configures the types inheritance from a given system to another. This allows to use types defined within *Middleware Interface Extension* files for a certain middleware into another middleware, without the need of duplicating them or writing an equivalent IDL type for the rest of systems.

  </details>

* `routes`: In this section, a list must be introduced, corresponding to which bridges are needed by
  *Integration Service* in order to fulfill the intercommunication requirements
  for a specific use case.

  At least one route is required; otherwise, running *Integration Service* would be useless.

  ```yaml
    routes:
      foo_to_bar: { from: foo, to: bar }
      bar_to_foo: { from: bar, to: foo }
      foo_server: { server: foo, clients: bar }
      bar_server: { server: bar, clients: foo }
  ```

  <details>
  <summary>There are two kinds of routes, corresponding to either a publication/subscription paradigm or a
  server/client paradigm: <i>(click to expand)</i></summary>

  * `from` - `to`: Defines a route **from** one (or several) system(s) **to** one (or several) system(s).
    A `from` system expects to connect a publisher user application with a subscriber user application in the `to` system.

  * `server` - `clients`: Defines a route for a request/reply architecture in which there are one or several
    **clients** which forward request petitions and listen to responses coming from a **server**,
    which must be unique for each service route.
  </details>

* `topics`: Specifies the topics exchanged over the `routes` listed above corresponding to the
  publication-subscription paradigm. The topics must be specified in the form of a YAML dictionary,
  meaning that two topics can never have the same name.

  For each topic, some configuration parameters are common for all the supported middlewares within the
  *Integration Service* ecosystem; while others are specific of each middleware. To see which topic
  parameters must/can be configured for a certain middleware, please refer to its dedicated *README* section
  in its corresponding GitHub repository, under the name of `https://github.com/eProsima/<MW_NAME>-SH`.

  ```yaml
    topics:
      hello_foo:
        type: HelloWorld
        route: bar_to_foo
      hello_bar:
        type: HelloWorld
        route: foo_to_bar
        remap: { bar: { topic: HelloBar } }
  ```

  <details>
  <summary>In relation to the common parameters, their behaviour is explained below: <i>(click to expand)</i></summary>

  * `type`: The topic type name. This type must be defined in the `types` section of the YAML
    configuration file, or it must be loaded by means of a `Middleware Interface Extension` file
    by any of the middleware plugins or *System Handles* involved in the communication process.

  * `route`: Communication bridge to be used for this topic. The route must be one among those defined in the
    `routes` section described above.

  * `remap` *(optional):* Allows to establish equivalences between the **topic** name and its **type**,
    for any of the middlewares defined in the used route. This means that the topic name and
    type name may vary in each user application endpoint that is being bridged, but,
    as long as the type definition is equivalent, the communication will still be possible.
  </details>

* `services`: Allows to define the services that *Integration Service* will be in charge of
  bridging, according to the service `routes` listed above for the client/server paradigm.
  The services must be specified in the form of a YAML dictionary, meaning that two services can
  never have the same name.

  For each service, some configuration parameters are common for all of the supported middlewares
  within the *Integration Service* ecosystem; while others are specific of each middleware.
  To see which parameters must/can be configured for a certain middleware in the context of a service
  definition, please refer to its dedicated *README* section in its corresponding GitHub repository,
  under the name of `https://github.com/eProsima/<MW_NAME>-SH`.

  ```yaml
  services:
    serve_foo:
      request_type: FooRequest
      reply_type: FooReply
      route: foo_server
    serve_bar:
      request_type: BarRequest
      reply_type: BarReply
      route: bar_server
      remap: { foo: { request_type: bar_req, reply_type: bar_repl, topic: ServeBar } }
  ```

  <details>
  <summary>Regarding the common parameters, they differ slightly from the <i>topics</i> section: <i>(click to expand)</i></summary>

  * `type` *(optional):* The service type. As services usually are composed of a request and a reply, this field
    only makes sense for those services which consist solely of a request action with no reply.
    Usually, within the `services` context, it is not used at all.

  * `request_type`: The service request type. This type must be defined in the `types` section of the YAML
    configuration file, or must be loaded by means of a `Middleware Interface Extension` file
    by any of the middleware plugins, or *System Handles*, involved in the communication process.

  * `reply_type`: The service reply type. This type must be defined in the `types` section of the YAML
    configuration file, or must be loaded by means of a `Middleware Interface Extension` file
    by any of the middleware plugins, or *System Handles*, involved in the communication process.

  * `route`: Communication bridge to be used for this service. The route must be one among those defined in the
    `routes` section described above and must be a route composed of a *server* and one or more *clients*.

  * `remap` *(optional):* Allows to establish equivalences between the **service** name (*topic* field) and its
    **request and reply type**, for any of the middlewares defined in the used route.
    This means that the service name and types names may vary in each user application endpoint
    that is being bridged, but, as long as the type definition is equivalent, the communication will still be possible.
  </details>

Finally, it is important to remark that both the `services` and `topics` sections are not mandatory,
meaning that an *Integration Service* instance can be launched only for publication/subscription
bridging, or solely to perform service type communications. However, they are not exclusive,
and can coexist under the same YAML configuration file.
# Supported middlewares and protocols

All of the currently protocols are integrated within *Integration Service*
by means of dedicated plugins or **System Handles**.

The complete *System Handles* set for the *Integration Service* is currently composed of the following protocols:

* [Fast DDS System Handle](https://github.com/eProsima/FastDDS-SH)

* [FIWARE System Handle](https://github.com/eProsima/FIWARE-SH)

* [ROS 1 System Handle](https://github.com/eProsima/ROS1-SH)

* [ROS 2 System Handle](https://github.com/eProsima/ROS2-SH)

* [WebSocket System Handle](https://github.com/eProsima/WebSocket-SH)

Additionally, creating a *System Handle* is a relatively easy task and allows to integrate a new
protocol to the *Integration System* infrastructure, which automatically provides the new protocol
with communication capabilities towards all of the aforementioned middlewares and protocols.

For more information, please refer to the [System Handle user manual](https://integration-service.docs.eprosima.com/en/latest/user_manual/systemhandle/sh.html#)
and [System Handle API reference](https://integration-service.docs.eprosima.com/en/latest/api_reference/core/systemhandle/api_systemhandle.html) sections, available in the official documentation.

# Compilation flags

*Integration Service* uses `CMake` for building and packaging the project.
There are several CMake flags, which can be tuned during the configuration step:

* `BUILD_LIBRARY`: This compilation flag can be used to completely disable the compilation of
  the *Integration Service* set of libraries, that is, the *Integration Service Core* and all the
  existing *System Handles* existing in the `colcon` workspace. It is enabled by default.

  This flag is useful, for example, to speed up the documentation generation process, when building the
  [API Reference](https://integration-service.docs.eprosima.com/en/latest/api_reference/api_reference.html) from the *Doxygen* source code comments.

  ```bash
  ~/is_ws$ colcon build --cmake-args -DBUILD_LIBRARY=OFF
  ```

* `BUILD_API_REFERENCE`: It is used to generate all the necessary files for building the
  [API Reference](https://integration-service.docs.eprosima.com/en/latest/api_reference/api_reference.html) section of the documentation, starting from the source code comments written
  using a *Doxygen*-like format. It is disabled by default; to use it:

  ```bash
   ~/is_ws$ colcon build --cmake-args -DBUILD_API_REFERENCE=ON
  ```

* `BUILD_TESTS`: When compiling *Integration Service*, use the `-DBUILD_TESTS=ON` CMake option
  to compile both the unitary tests for the [Integration Service Core](core/) and the unitary
  and integration tests for all the *System Handles* present in the `colcon` workspace:

  ```bash
  ~/is_ws$ colcon build --cmake-args -DBUILD_TESTS=ON
  ```

* `BUILD_EXAMPLES`: Allows to compile utilities that can be used for the several provided
  usage examples for *Integration Service*, located under the [examples/utils](examples/utils/) folder.

    | :warning: | *To use this flag, all the examples dependencies need to be installed.* |
    | :-------: | :---------------------------------------------------------------------- |

  These applications can be used to test the *Integration Service* with some of the provided *YAML* configuration
  files, which are located under the [examples/basic](examples/basic) directory:

    ```bash
    ~/is_ws$ colcon build --cmake-args -DBUILD_EXAMPLES=ON
    ```

* `BUILD_FASTDDS_EXAMPLES`: Allows to compile the *FastDDS* utilities that can be used for several of the
  provided usage examples for *Integration Service*, located under the [examples/utils/dds](examples/utils/dds)
  folder.

    | :warning: | *To compile these examples you need to have FastDDS (v.2.0.0 or superior) installed.* |
    | :-------: | :-----------------------------------------------------------------------------------: |

  These applications can be used to test the *Integration Service* with some of the provided *YAML* configuration
  files, which are located under the [examples/basic](examples/basic) directory:

    ```bash
    ~/is_ws$ colcon build --cmake-args -DBUILD_FASTDDS_EXAMPLES=ON
    ```

  <details>
  <summary>To date, the following <i>FastDDS</i> user application examples are available: <i>(click to expand)</i></summary>

  * `DDSHelloWorld`: A simple publisher/subscriber application, running under [Fast DDS](https://fast-dds.docs.eprosima.com/).
    It publishes or subscribes to a simple string topic, named *HelloWorldTopic*.

    The executable resulting will be located inside the `build/is-examples/dds` folder and named `DDSHelloWorld`. Please execute `DDSHelloWorld -h` to see a full list of supported input parameters.


  * `DDSAddTwoInts`: A simple server/client C++ application, running under [Fast DDS](https://fast-dds.docs.eprosima.com/).
    It allows performing service requests and replies to a service named `AddTwoIntsService`, which consists
    of two integer numbers as request type and answers with a single value, indicating the sum of them.

    The executable resulting will be located inside the `build/is-examples/dds` folder and named `DDSAddTwoInts`. Please execute `DDSAddTwoInts -h` to see a full list of supported input parameters.

  </details>

* `BUILD_ROS1_EXAMPLES`: Allows to compile the *ROS 1* utilities that can be used for several of the
  provided usage examples for *Integration Service*, located under the [examples/utils/ros1](examples/utils/ros1)
  folder.

  | :warning: | *To compile this example you need to have ROS 1 (Melodic or superior) installed and sourced.* |
  | :-------: | :-------------------------------------------------------------------------------------------: |

  These applications can be used to test the *Integration Service* with some of the provided *YAML* configuration
  files, which are located under the [examples/basic](examples/basic) directory:

    ```bash
    ~/is_ws$ colcon build --cmake-args -DBUILD_ROS1_EXAMPLES=ON
    ```
    <details>
    <summary>To date, the following <i>ROS 1</i> userr-defined packages and application examples are available: <i>(click to expand)</i></summary>

    * `add_two_ints_server`: A simple C++ server application, running under *ROS 1*. It listens to requests coming from *ROS 1* clients and produces an appropriate
      answer for them; specifically, it is capable of listening to a *ROS 1* service called `add_two_ints`, which consists of two integer numbers as request type
      and answers with a single value, indicating the sum of them.

      The resulting executable will be located inside the `build/is-examples/ros1/add_two_ints_server` folder, and named `add_two_ints_server_node`.
    * `example_interfaces`: *ROS 1* package containing the service type definitions for the AddTwoInts services examples, for which the *ROS 1* type support files
      will be automatically generated. As specified in the services examples tutorials, it must be compiled and installed in the system, using `catkin`:

      ```bash
      ~/is_ws$ cd examples/utils/ros1/catkin_ws/
      ~/is_ws/examples/utils/ros1/catkin_ws$ catkin_make -DBUILD_EXAMPLES=ON -DCMAKE_INSTALL_PREFIX=/opt/ros/<ROS1_DISTRO> install
      ```

    </details>

* `BUILD_WEBSOCKET_EXAMPLES`: Allows to compile the *WebSocket* utilities that can be used for several of the provided usage examples for
  *Integration Service*, located under the [examples/utils/websocket](examples/utils/websocket) folder.

  | :warning: | *To compile this example you need to have OpenSSL and WebSocket++ installed.* |
  | :-------: | :---------------------------------------------------------------------------: |

  These applications can be used to test the *Integration Service* with some of the provided *YAML* configuration
  files, which are located under the [examples/basic](examples/basic) directory:

  ```bash
  ~/is_ws$ colcon build --cmake-args -DBUILD_WEBSOCKET_EXAMPLES=ON
  ```
  <details>
  <summary>To date, the following <i>WebSocket</i> user application examples are available: <i>(click to expand)</i></summary>

  * `WebSocketAddTwoInts`: A simple server/client C++ application, running under WebSocket++. It allows performing service requests and replies to a service
    named add_two_ints, which consists of two integer numbers as request type and answers with a single value, indicating the sum of them.

    The resulting executable will be located inside the `build/is-examples/websocket` folder, and named `WebSocketAddTwoInts`. Please execute `WebSocketAddTwoInts -h` to see a full list of supported input parameters.

  </details>

# Documentation

The official documentation for *eProsima Integration Service* is hosted by Read the Docs,
and comprises the following sections:

* [Introduction](https://integration-service.docs.eprosima.com/en/latest/index.html)
* [Installation Manual](https://integration-service.docs.eprosima.com/en/latest/installation_manual/installation_manual.html)
* [User Manual](https://integration-service.docs.eprosima.com/en/latest/user_manual/user_manual.html)
* [API Reference](https://integration-service.docs.eprosima.com/en/latest/api_reference/api_reference.html)
* [Examples](https://integration-service.docs.eprosima.com/en/latest/examples/examples.html)
* [Release Notes](https://integration-service.docs.eprosima.com/en/latest/release_notes/release_notes.html)

# License

This repository is open-sourced under the *Apache-2.0* license. See the [LICENSE](LICENSE) file for more details.

# Getting help

If you need support you can reach us by mail at `support@eProsima.com` or by phone at `+34 91 804 34 48`.
