/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 * Copyright (C) 2020 - present Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef _IS_CORE_INTERNAL_CONFIG_HPP_
#define _IS_CORE_INTERNAL_CONFIG_HPP_

#include <is/systemhandle/RegisterSystem.hpp>
#include <is/core/runtime/Search.hpp>
#include <is/core/runtime/MiddlewareInterfaceExtension.hpp>

#include <yaml-cpp/yaml.h>

#include <set>
#include <vector>

namespace eprosima {
namespace is {
namespace core {

/**
 * @brief Forward declaration.
 */
struct RequiredTypes;

namespace internal {

/**
 * @struct MiddlewareConfig
 * @brief Holds information relative to each middleware configuration.
 *
 * @var MiddlewareConfig::type
 *      @brief The name of the middleware.
 *
 * @var MiddlewareConfig::types_from
 *      @brief The name of middleware whose types want to be used.
 *
 * @var MiddlewareConfig::config_node
 *      @brief YAML configuration associated with the specific middleware.
 */
struct MiddlewareConfig
{
    std::string type;
    std::vector<std::string> types_from;
    YAML::Node config_node;
    YAML::Node types_node;
};

/**
 * @struct TopicRoute
 * @brief Stores information relative to topic routes:
 *
 * @var TopicRoute::from
 *      @brief Source middleware endpoint.
 *
 * @var TopicRoute::to
 *      @brief Destination middleware endpoint.
 */
struct TopicRoute
{
    std::set<std::string> from;
    std::set<std::string> to;

    /**
     * @brief Helper method to retrieve at once *from* and *to* sets.
     *
     * @returns A set containing all endpoints for this TopicRoute.
     */
    std::set<std::string> all() const
    {
        std::set<std::string> _all = from;
        _all.insert(to.begin(), to.end());
        return _all;
    }

};

/**
 * @struct ServiceRoute
 * @brief Stores information relative to service routes:
 *
 * @var ServiceRoute::server
 *     @brief Server endpoint.
 *
 * @var ServiceRoute::clients
 *      @brief Client endpoints.
 */
struct ServiceRoute
{
    std::set<std::string> clients;
    std::string server;

    /**
     * @brief Helper method to retrieve at once *server* and *clients* sets.
     *
     * @returns A set containing all endpoints for this ServiceRoute.
     */
    std::set<std::string> all() const
    {
        std::set<std::string> _all = clients;
        _all.insert(server);
        return _all;
    }

};

/**
 * @struct TopicInfo
 * @brief Struct containing information about a certain topic.
 *
 * @var TopicInfo::name
 *      @brief The name of the topic.
 *
 * @var TopicInfo::type
 *      @brief The name of the type for the specific topic.
 *
 */
struct TopicInfo
{
    TopicInfo() = default;

    TopicInfo(
            const std::string& m_name,
            const std::string& m_type,
            const std::string& m_reply_type = "")
        : name(m_name)
        , type(m_type)
        , reply_type(m_reply_type)
    {
    }

    std::string name;
    std::string type; //  AKA request_type for Services with both request and reply types.
    std::string reply_type; //  Only used for Services with reply_type.
};

/**
 * @brief
 *      Struct containing information about a certain service.
 *
 *      * **std::string name** \n
 *        The name of the service.
 *
 *      * **std::string type** \n
 *        The name of the request type for the specific service.
 *
 *      * **std::string reply_type** \n
 *        The name of the reply type for the specific service.
 */
using ServiceInfo = TopicInfo;

/**
 * @struct TopicConfig
 * @brief Holds the configuration provided for a certain topic.
 *
 * @var TopicConfig::message_type
 *      @brief The name of the type for the specific topic.
 *
 * @var TopicConfig::route
 *      @brief The route followed by the specific topic.
 *
 * @var TopicConfig::remap
 *      @brief A map with the remaps needed for the specific topic.
 *
 * @var TopicConfig::middleware_configs
 *      @brief A map with the YAML configuration for the specific topic.
 */
struct TopicConfig
{
    std::string message_type;
    TopicRoute route;

    std::map<std::string, TopicInfo> remap; //  The "key" is the middleware alias.

    std::map<std::string, YAML::Node> middleware_configs;
};

/**
 * @struct ServiceConfig
 * @brief This struct stores the configuration provided for a certain service.
 *
 * @var ServiceConfig::request_type
 *      @brief The name of the request type for the specific service.
 *
 * @var ServiceConfig::reply_type
 *      @brief The name of the reply type for the specific service.
 *
 * @var ServiceConfig::route
 *      @brief The route followed by the specific service.
 *
 * @var ServiceConfig::remap
 *      @brief A map with the remaps needed for the specific service.
 *
 * @var ServiceConfig::middleware_configs
 *      @brief A map with the YAML configuration for the specific service.
 */
struct ServiceConfig
{
    std::string request_type;
    std::string reply_type; //  Optional
    ServiceRoute route;

    std::map<std::string, ServiceInfo> remap; //  The "key" is the middleware alias.

    std::map<std::string, YAML::Node> middleware_configs;
};

/**
 * @class Config
 *        Internal representation of the configuration provided to the
 *        *Integration Service* instance, by means of a *YAML* file.
 */
class Config
{
public:

    /**
     * @brief Signature for the container used to store the subscription callbacks for a certain
     *        *Integration Service* instance.
     */
    using SubscriptionCallbacks =
            std::vector<std::unique_ptr<is::TopicSubscriberSystem::SubscriptionCallback> >;

    /**
     * @brief Signature for the container used to store the service request callbacks for a certain
     *        *Integration Service* instance.
     */
    using RequestCallbacks =
            std::vector<std::unique_ptr<is::ServiceClientSystem::RequestCallback> >;

    /**
     * @brief Constructor.
     *
     * @param[in] node Parsed representation of the *YAML* configuration file.
     *            It defaults to empty.
     *
     * @param[in] filename The path of the *YAML* configuration file.
     */
    Config(
            const YAML::Node& node = YAML::Node(),
            const std::string& filename = "<text>");

    /**
     * @brief Helper static constructor to retrieve a Config instance from a file path.
     *
     * @param[in] file A string containing the configuration file path.
     */
    static Config from_file(
            const std::string& file);

    /**
     * @brief Parses the provided configuration, according to the configuration file
     *        scheme defined for *Integration Service*.
     *
     * @details Configuration files typically contain the following sections:
     *          1. `types`: Specifies the `IDL` types used by *Integration Service*
     *             to transmit messages. These `IDL` definitions will be parsed using
     *             `eprosima::xtypes` parser for `IDL` files, and the resulting
     *             *Dynamic Types* will be added to the available types database.
     *
     *             The following subsections are permitted:
     *
     *             1.1. `idl`: IDL content.
     *
     *             1.2. `paths`: includes paths containing IDL definitions that will
     *                  also be parsed and added to the types database.
     *
     *          2. `systems`: Lists the middlewares involved in the communication,
     *             allowing to configure them. Custom aliases can be given to any system.
     *
     *             The following subsections are permitted:
     *
     *             2.1. `type`: to be selected among the middlewares supported by
     *                  *Integration Service*: `ros2`, `dds`, `websocket`, `ros1`...
     *
     *             2.2. `types-from`: allows to inherit type definitions from one
     *                  system to another. In this way, users do not have to redefine
     *                  types for each system.
     *
     *             2.3. Custom configuration parameters, such as `domain_id` (for ROS 2).
     *                  Each SystemHandle may define its own configuration fields,
     *                  please refer to their documentation for more details.
     *
     *          3. `routes`: Lists the communication bridges that *Integration Service*
     *             needs to establish among `systems`. Each route has a specific name.
     *
     *             The following subsections are permitted:
     *
     *             3.1. `from/to`: publisher/subscriber communication.
     *
     *             3.2. `server/clients`: server/client communication.
     *
     *          4. `topics/services`: Allows to configure the topics exchanged over the `routes`
     *             described above, in either publisher/subscriber or client/server
     *             communication, and provides detailed information about them.
     *             Each topic or service must have a unique name in the *YAML* file.
     *
     *             The following subsections are permitted:
     *
     *             4.1. `type`: Type involved in the communication. It can be a built-in
     *                   type, usually coming from a `mix` library; or a custom user-defined
     *                   type, by means of an IDL definition.
     *
     *             4.2. `route`: Communication bridge, of the ones defined above, that must
     *                  perform the communication.
     *
     *             4.3. `remap`: allows to establish equivalences between `topic` names and
     *                           `types` for each system involved in the communication.
     *
     *             4.4. Custom configuration parameters, which are specific for each middleware.
     *                  Please refer to the specific SystemHandle documentation.
     *
     * @param[in] node The parsed YAML representation of the configuration file provided.
     *
     * @param[in] filename The path of the configuration file.
     *
     * @returns `True` if the parsing was correct, `false` if some error occurred.
     */
    bool parse(
            const YAML::Node& node,
            const std::string& filename = "<text>");

    /**
     * @brief Checks if everything is okay with the configuration process.
     *
     * @returns The `_okay` boolean parameter.
     */
    bool okay() const;

    /**
     * @brief bool operator overload.
     *
     * @returns The okay() parameter.
     */
    operator bool() const;

    /**
     * @brief Performs a search and loads the dynamic libraries required
     *        for each middleware, that is, the SystemHandle entities.
     *
     * @details After the SystemHandle shared library is loaded successfully,
     *          the required types to be found during the SystemHandle configuration
     *          phase are registered, according to what was specified in the *YAML* configuration.
     *
     *          Next, the `types-from` parameter is checked, which specifies the middleware
     *          from which each SystemHandle wants to inherit types from, as declared
     *          in the configuration.
     *
     *          Finally, for each SystemHandle, the `configure` method is called.
     *          If one of the middlewares listed is not properly configured,
     *          the whole process fails.
     *
     * @param[out] info_map Map between the middlewares and their SystemHandle
     *             instances information (handle pointer, topic publisher and subscriber
     *             and service client and provider systems, as well as its type registry).
     *             This map should be filled with the information for all the SystemHandle
     *             defined in the configuration, once this method succeeds.
     *
     * @return Boolean value indicating whether the load process was successful or not.
     */
    bool load_middlewares(
            is::internal::SystemHandleInfoMap& info_map) const;

    /**
     * @brief Configures topics communication, according to the specified route,
     *        type and remapping parameters.
     *
     * @details First, compatibility between the type defined in the `from` endpoint
     *          and the `to` (destination) endpoint is checked, because it could happen that,
     *          because of a remapping, the type definition in the source and destination
     *          systems is slightly (or completely) different.
     *          To do that, `check_topic_compatibility` is called. Please refer to its
     *          documentation for more details.
     *
     *          If the types are compatible, the next step is to check the publishing capabilities
     *          of the destination endpoint, and if everything is correct, that is, if
     *          the system has an associated TopicPublisherSystem, the SystemHandle advertises
     *          the topic. This publication will transmit the data to the user application,
     *          which must define a subscriber capable of receiving and processing the data.
     *
     *          Then, in the source endpoint, the existence of subscribing capabilities is checked,
     *          and, if so, the subscriber defines a SubscriptionCallback lambda, that
     *          iterates through the previous constructed list of publishers and ensures that
     *          each defined destination endpoint gets the data published. This callback
     *          is used to call to `TopicSubscriberSystem::subscribe` method.
     *
     *          If any of the defined topics cannot find publishing or subscription capabilities
     *          (i.e. invalid routes), the returned value will be false and the process will fail.
     *
     * @param[in] info_map Map filled during the `load_middlewares` phase and containing
     *            the information for each loaded SystemHandle, in terms of its instance,
     *            supported types and publish/subscribe or client/server capabilities.
     *
     * @param[in] subscription_callbacks Reference to the map used to store all of the active
     *            subscription callbacks for a certain SystemHandle instance.
     *
     * @returns `true` if all the topics were successfully configured, `false` otherwise.
     */
    bool configure_topics(
            const is::internal::SystemHandleInfoMap& info_map,
            SubscriptionCallbacks& subscription_callbacks) const;

    /**
     * @brief Configures services, according to the specified route, type and remapping
     *        parameters.
     *
     * @details First, compatibility between the request and reply types defined
     *          in the `server` endpoint and the `clients` endpoints is checked,
     *          because it could happen that, because of a remapping, the types
     *          defined for request/reply is slightly (or completely) different
     *          in the server and client endpoints.
     *          To do that, `check_service_compatibility` is called. Please refer to its
     *          documentation for more details.
     *
     *          If the types are compatible, the next step is to check the service providing
     *          capabilities of the server endpoint, and if everything is correct, that is, if
     *          the system has an associated ServiceProviderSystem, the SystemHandle creates
     *          the corresponding service provider proxy.
     *
     *          Then, for all the defined clients, ServiceClientSystem capabilities
     *          are checked, and a request callback is defined, which basically
     *          executes the `ServiceProvider::call_service` method from the associated provider.
     *          This `call_service` is then responsible of sending back the response
     *          to the client, if applicable (that is, if a `reply_type` has been defined
     *          in the *YAML* configuration.)
     *
     *          If any of the defined services cannot find server or client capabilities
     *          (i.e. invalid routes), the returned value will be false and the process will fail.
     *
     * @param[in] info_map Map filled during the `load_middlewares` phase and containing
     *            the information for each loaded SystemHandle, in terms of its instance,
     *            supported types and publish/subscribe and client/server capabilities.
     *
     * @param[in] request_callbacks Reference to the map used to store all of the active
     *            request callbacks for a certain SystemHandle instance.
     *
     * @returns `true` if all the services were successfully configured, `false` otherwise.
     */
    bool configure_services(
            const is::internal::SystemHandleInfoMap& info_map,
            RequestCallbacks& request_callbacks) const;

    /**
     * @brief Checks compatibility between the TopicInfo registered in the endpoints responsible
     *        for a topic publish/subscribe communication in *Integration Service*.
     *
     *        This compatibility check is ensured thanks to eProsima `xtypes` library
     *        and its `TypeConsistency` definition. If types are not equal, some policies
     *        might be automatically applied to try to make them compatible,
     *        such as ignoring member names, type signs, etc.
     *
     * @param[in] info_map Map filled during the `load_middlewares` phase and containing
     *            the information for each loaded SystemHandle, in terms of its instance,
     *            supported types and publish/subscribe or client/server capabilities.
     *
     * @param[in] topic_name The topic whose compatibility will be checked between endpoints.
     *
     * @param[in] config TopicConfig structure containing information such as the type,
     *            the source/destination defined route and the remappings, as well as the
     *            specific middleware configurations for this topic.
     *
     * @returns `true` if the topic is compatible among the defined systems, `false` otherwise.
     */
    bool check_topic_compatibility(
            const is::internal::SystemHandleInfoMap& info_map,
            const std::string& topic_name,
            const TopicConfig& config) const;

    /**
     * @brief Checks compatibility between the ServiceConfig registered in the endpoints
     *        responsible of a server/client communication in the *Integration Service*.
     *
     *        This compatibility check is ensured thanks to eProsima `xtypes` library
     *        and its `TypeConsistency` definition. If types are not equal, some policies
     *        might be automatically applied to try to make them compatible,
     *        such as ignoring member names, type signs, etc.
     *
     *        The check is performed both for request and reply types.
     *
     * @param[in] info_map Map filled during the `load_middlewares` phase and containing
     *            the information for each loaded SystemHandle, in terms of its instance,
     *            supported types and publish/subscribe and client/server capabilities.
     *
     * @param[in] service_name The service whose compatibility will be checked between endpoints.
     *
     * @param[in] config ServiceConfig structure containing information such as the
     *            request and reply types, the server and clients defined route and the remappings,
     *            as well as the specific middleware configurations for this service.
     *
     * @returns `true` if the service is compatible among the defined systems, `false` otherwise.
     */
    bool check_service_compatibility(
            const is::internal::SystemHandleInfoMap& info_map,
            const std::string& service_name,
            const ServiceConfig& config) const;

    /**
     * @brief This function allows to retrieve a type member from an externally defined type containing
     *        it, to use it as the type for a certain configuration.
     *
     *        The used syntax when retrieving the inner type must be `<outer_type>.<type_member_name>`.
     *
     *        For example, if a type is defined like this in an *IDL*:
     *
     *        @code{.cpp}
     *        union MyUnion (switch uint8)
     *        {
     *              case 0: int32 _zero;
     *              case 1: int64 _one;
     *              default: int128 _default;
     *        };
     *        @endcode
     *
     *        You can define the following topic: \n
     *        `ExampleTopic: { route: "a_to_b", type: MyUnion._zero }`
     *
     * @param[in] types TypeRegistry containing all the available types where the search
     *            of the parent type will be performed.
     *
     * @param[in] path The whole type definition, as specified by the user. In the example,
     *            it would be `MyUnion._zero`.
     *
     * @returns A pointer to the inner DynamicType representing the type requested by the user.
     */
    const eprosima::xtypes::DynamicType* resolve_type(
            const TypeRegistry& types,
            const std::string& path) const;

    static utils::Logger logger;

private:

    /**
     * Class members.
     */

    bool _okay;

    std::map<std::string, MiddlewareConfig> _m_middlewares;

    std::map<std::string, TopicRoute> _m_topic_routes;

    std::map<std::string, ServiceRoute> _m_service_routes;

    std::map<std::string, TopicConfig> _m_topic_configs;

    std::map<std::string, ServiceConfig> _m_service_configs;

    std::map<std::string, RequiredTypes> _m_required_types;

    std::map<std::string, eprosima::xtypes::DynamicType::Ptr> _m_types;

};

} //  namespace internal
} //  namespace core
} //  namespace is
} //  namespace eprosima

#endif //  _IS_CORE_INTERNAL_CONFIG_HPP_
