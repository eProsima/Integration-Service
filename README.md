<a href="http://www.eprosima.com"><img src="docs/images/logo.png" hspace="8" vspace="2" height="100" ></a>

# Introduction

The *eProsima Integration Service* is a tool that enables communication among
an arbitrary number of protocols that speak different languages.

![Integration Service general architecture](docs/images/general-architecture.png)

These languages are translated to a common representation language which follows the
[Extensible and Dynamic Topic Types for DDS](https://www.omg.org/spec/DDS-XTypes/About-DDS-XTypes/)
(**xTypes**) standard by the [OMG](https://www.omg.org/); specifically, the *Integration Service*
bases its intercommunication abilities on eProsima's open source implementation
for the *xTypes* protocol, that is, [eProsima xTypes](https://github.com/eProsima/xtypes).
# Configuration

The *Integration Service* can be configured during runtime by means of a dedicated **YAML** file.
This YAML configuration file must follow a specific syntax, this meaning that several sections
are required for it to sucessfully configure and launch an *Integration Service* instance;
while others are optional. Both of them are listed and reviewed here:

* `types` *(optional)*: It allows to list the [IDL](https://www.omg.org/spec/IDL/4.2/About-IDL/)
  types used by the *Integration Service* to later define the topics and services types which will
  take part in the communication process.

  This field can be omitted for certain *Integration Service* instances where one or more *System
  Handles* already include static type definitions and its corresponding transformation libraries
  (*Middleware Interface Extenion* or *mix* files).

  <details>
  <summary>Several parameters can be configured within this section:</summary>
    
    * `idls`: List of IDL type definitions which can be directly embedded within the configuration file. If the *types* section is defined, this subsection is mandatory.

    * `paths` *(optional):* Using this parameter, an existing IDL type written in a separate file can be included within the *Integration Service* types set definition.

  </details>

* `systems`: Specifies which middlewares will be involved in the communication process, allowing
  to configure them individually.

  Some configuration parameters are common for all the supported middlewares within the
  *Integration Service* ecosystem; while others are specific of each middleware. To see which
  parameters can be tuned for a certain middleware, please refer to its dedicated *README* section
  in its corresponding GitHub repository, under the name of `https://github.com/eProsima/<MW_NAME>-SH`.

  In relation to the common parameters, their behaviour is explained below:

  * `type`: Middleware or protocol kind. Up to this time, supported middlewares are:
    *Fast DDS*, *ROS1*, *ROS2* and *WebSocket*.

  * `types-from` *(optional)*: Allows certain system to inherit types from another system.
    This allows to use types defined within *Middleware Interface Extension* files for a certain
    middleware onto another middleware, without the need of duplicating them or writing an
    equivalent IDL type for the rest of systems.

* `routes`: In this section, a list must be introduced, corresponding to which bridges the
  *Integration Service* needs to create in order to fulfill the intercommunication requirements
  for a specific use case.

  At least one route is required; otherwise, running the *Integration Service* would be useless.

  There are two kinds of routes, corresponding to publication/subscription paradigm and
  server/client paradigm:

  * `from` - `to`: Define a route **from** one (or several) system(s) **to** one (or several) system(s).
    A `from` system expects to connect a publisher user application with a `to` system subscription user application.

  * `server` - `clients`: Define a request/reply architecture in which there are one or several
    **clients** which forward request petitions and listen to responses coming from a **server**,
    which must be unique for each service route.

* `topics`: Specifies the topics exchanged over the `routes` listed above corresponding to the
  publication-subscription paradigm. The topics must be specified in the form of a YAML dictionary,
  meaning that two topics can never have the same name.

  For each topic, some configuration parameters are common for all the supported middlewares within the
  *Integration Service* ecosystem; while others are specific of each middleware. To see which
  parameters can be tuned for a certain middleware, please refer to its dedicated *README* section
  in its corresponding GitHub repository, under the name of `https://github.com/eProsima/<MW_NAME>-SH`.

  In relation to the common parameters, their behaviour is explained below:

  * `type`: The topic type name. This type must be defined in the `types` section of the YAML
    configuration file, or must be loaded by means of a `Middleware Interface Extension` file
    by any of the middleware plugins involved in the communication process.

  * `route`: Communication bridge to be used for this topic. The route must be defined in the
    `routes` section described above.

  * `remap`: Allows to establish equivalences between the **topic** name and its **type**,
    for any of the middlewares defined in the used route. This means that the topic name and
    type name may vary in each user application endpoint that is desired to be bridged, but,
    as long as the type definition is equivalent, the communication will still be possible.

# Supported middlewares and protocols


These protocols are integrated within the *Integration Service* by means of dedicated plugins or *System Handles*.
Currently, we support the following protocols:

* [Fast DDS System Handle](https://github.com/eProsima/FastDDS-SH)

* [ROS 1 System Handle](https://github.com/eProsima/ROS1-SH)

* [ROS 2 System Handle](https://github.com/eProsima/ROS2-SH)

* [WebSocket System Handle](https://github.com/eProsima/WebSocket-SH)

For more information, please refer to the [official documentation](https://integration-service.docs.eprosima.com/en/latest/).

