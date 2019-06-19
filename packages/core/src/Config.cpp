/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include "Search-impl.hpp"
#include "Config.hpp"

#include <soss/MiddlewareInterfaceExtension.hpp>

#include <iostream>

namespace soss {
namespace internal {
namespace {

//==============================================================================
bool scalar_or_list_node_to_set(
    const YAML::Node& node,
    std::set<std::string>& values,
    const std::string& field,
    const std::string& route_type)
{
  bool valid = true;

  if(node.IsSequence())
  {
    for(YAML::const_iterator it = node.begin(); it != node.end(); ++it)
    {
      values.insert(it->as<std::string>());
    }
  }
  else if(node.IsScalar())
  {
    values.insert(node.as<std::string>());
  }
  else
  {
    valid = false;
  }

  if(!valid)
  {
    std::cerr << "config-file [" << field << "] entry in " << route_type
              << " route must point to a string or list of at least one string"
              << std::endl;
    valid = false;
  }

  return valid;
}

//==============================================================================
std::unique_ptr<TopicRoute> parse_topic_route(const YAML::Node& node)
{
  auto route = std::make_unique<TopicRoute>();
  bool valid = true;

  valid &= scalar_or_list_node_to_set(
        node["from"], route->from, "from", "topic");

  valid &= scalar_or_list_node_to_set(
        node["to"], route->to, "to", "topic");

  if(!valid)
    return nullptr;

  return route;
}

//==============================================================================
std::unique_ptr<ServiceRoute> parse_service_route(const YAML::Node& node)
{
  bool valid = true;
  auto route = std::make_unique<ServiceRoute>();

  const YAML::Node& server = node["server"];
  if(!server)
  {
    std::cerr << "config-file service route must contain a [server] entry!"
              << std::endl;
    valid = false;
  }

  if(!server.IsScalar() || server.as<std::string>().empty())
  {
    std::cerr << "config-file service route [server] entry must be a scalar "
              << "string! Only one middleware can be the server on a route!"
              << std::endl;
    valid = false;
  }
  else
  {
    route->server = server.as<std::string>();
  }

  valid &= scalar_or_list_node_to_set(
        node["clients"], route->clients, "clients", "service");

  if(!valid)
    return nullptr;

  return route;
}

//==============================================================================
bool add_named_route(
    const std::string& name,
    const YAML::Node& node,
    std::map<std::string, TopicRoute>& topic_routes,
    std::map<std::string, ServiceRoute>& service_routes)
{
  bool inserted = false;
  if(node["from"] && node["to"])
  {
    const auto route = parse_topic_route(node);
    if(!route)
    {
      std::cerr << "Failed to parse the route named ["
                << name << "]" << std::endl;
      return false;
    }

    inserted = topic_routes.insert(std::make_pair(name, *route)).second;
  }
  else if(node["server"] && node["clients"])
  {
    const auto route = parse_service_route(node);
    if(!route)
    {
      std::cerr << "Failed to parse the route named ["
                << name << "]" << std::endl;
      return false;
    }

    inserted = service_routes.insert(std::make_pair(name, *route)).second;
  }
  else
  {
    std::cerr << "Cannot recognize route named [" << name << "] because it "
              << "does not match a topic route {from: X, to: Y} or a service "
              << "route {server: X, clients: Y}" << std::endl;
    return false;
  }

  if(!inserted)
  {
    std::cerr << "Duplicate route named [" << name << "]!" << std::endl;
  }

  return inserted;
}

//==============================================================================
template<typename ConfigType, typename RouteType>
bool add_topic_or_service_config(
    const std::string& channel_type,
    const std::string& name,
    const YAML::Node& node,
    const std::map<std::string, RouteType>& predefined_routes,
    std::map<std::string, ConfigType>& config_map,
    std::function<void(ConfigType&, std::string)> set_type,
    std::function<void(ConfigType&, RouteType)> set_route,
    std::function<std::unique_ptr<RouteType>(const YAML::Node&)> parse_route)
{
  bool valid = true;
  ConfigType config;

  const YAML::Node& type = node["type"];
  if(!type)
  {
    std::cerr << channel_type << " configuration [" << name
              << "] is missing its [type] field!" << std::endl;
    valid = false;
  }
  else
  {
    set_type(config, type.as<std::string>());
  }

  const YAML::Node& route = node["route"];
  if(!route)
  {
    std::cerr << channel_type << " configuration [" << name
              << "] is missing its [route] field!" << std::endl;
    valid = false;
  }
  else
  {
    if(route.IsScalar())
    {
      const std::string& route_name = route.as<std::string>();
      const auto it = predefined_routes.find(route_name);
      if(it == predefined_routes.end())
      {
        std::cerr << channel_type << " configuration [" << name
                  << "] requested a route named [" << route_name << "] but a "
                  << "route with that name was never defined in the [routes] "
                  << "dictionary!" << std::endl;
        valid = false;
      }
      else
      {
        set_route(config, it->second);
      }
    }
    else if(route.IsMap())
    {
      const auto route_config = parse_route(route);
      if(!route_config)
      {
        std::cerr << "Failed to parse the route configuration for the "
                  << channel_type << " [" << name << "]!" << std::endl;
        valid = false;
      }
      else
      {
        set_route(config, std::move(*route_config));
      }
    }
  }

  const YAML::Node& remap = node["remap"];
  if(remap)
  {
    if(!remap.IsMap())
    {
      std::cerr << "A [remap] field was given for the " << channel_type
                << " configuration [" << name
                << "], but it is not a dictionary!" << std::endl;
      valid = false;
    }
    else
    {
      for(YAML::const_iterator it = remap.begin(); it != remap.end(); ++it)
      {
        config.remap[it->first.as<std::string>()] =
            it->second.as<std::string>();
      }
    }
  }

  // TODO(MXG): Come up with a yaml scene for specifying middleware
  // configurations per topic/service and then fill in the middleware_configs
  // map here

  if(valid)
  {
    if(!config_map.insert(std::make_pair(name, config)).second)
    {
      std::cerr << channel_type << " configuration [" << name
                << "] was specified twice!" << std::endl;
      valid = false;
    }
  }

  return valid;
}

//==============================================================================
bool add_topic_config(
    const std::string& name,
    const YAML::Node& node,
    const std::map<std::string, TopicRoute>& topic_routes,
    std::map<std::string, TopicConfig>& topic_configs)
{
  return add_topic_or_service_config<TopicConfig, TopicRoute>(
        "topic", name, node, topic_routes, topic_configs,
        [](TopicConfig& c, std::string s){ c.message_type = std::move(s); },
        [](TopicConfig& c, TopicRoute r){ c.route = std::move(r); },
        [](const YAML::Node& node){ return parse_topic_route(node); });
}

//==============================================================================
bool add_service_config(
    const std::string& name,
    const YAML::Node& node,
    const std::map<std::string, ServiceRoute>& service_routes,
    std::map<std::string, ServiceConfig>& service_configs)
{
  return add_topic_or_service_config<ServiceConfig, ServiceRoute>(
        "service", name, node, service_routes, service_configs,
        [](ServiceConfig& c, std::string s){ c.service_type = std::move(s); },
        [](ServiceConfig& c, ServiceRoute r){ c.route = std::move(r); },
        [](const YAML::Node& node){ return parse_service_route(node); });
}

//==============================================================================
using ReadDictEntry =
    std::function<bool(const std::string& key, const YAML::Node& value)>;

bool read_dictionary(
    const YAML::Node& config_node,
    const std::string& field_name,
    const std::string& filename,
    const ReadDictEntry& read_fcn)
{
  const YAML::Node& field = config_node[field_name];
  if(field)
  {
    if(!field.IsMap())
    {
      std::cerr << "The config file [" << filename << "] has a [" << field_name
                << "] field, but it is not pointing to a dictionary of "
                // We use this to make the field name singular
                << field_name.substr(0, field_name.size()-1)
                << "configurations!" << std::endl;
      return false;
    }

    bool configs_okay = true;
    for(YAML::const_iterator it = field.begin(); it != field.end(); ++it)
    {
      configs_okay &= read_fcn(it->first.as<std::string>(), it->second);
    }

    if(!configs_okay)
    {
      std::cerr << "Error found in an entry of the [" << field_name
                << "] dictionary of the config-file [" << filename << "]"
                << std::endl;
      return false;
    }
  }

  return true;
}

//==============================================================================
YAML::Node config_or_empty_node(
    const std::string& key,
    const std::map<std::string, YAML::Node>& map)
{
  const auto it = map.find(key);
  if(it == map.end())
    return YAML::Node();

  return it->second;
}

//==============================================================================
std::string remap_if_needed(
    const std::string& middleware,
    const std::map<std::string, std::string>& remap,
    const std::string& original)
{
  const auto it = remap.find(middleware);
  if(it == remap.end())
    return original;

  return it->second;
}

} // anonymous namespace

//==============================================================================
Config::Config(const YAML::Node& node, const std::string& filename)
{
  if(node && !node.IsNull())
    parse(node, filename);
}

//==============================================================================
Config Config::from_file(const std::string& file)
{
  YAML::Node config_node;
  try
  {
    config_node = YAML::LoadFile(file);
  }
  catch(const std::exception& e)
  {
    std::cerr << "Could not parse the config-file named [" << file
              << "]: " << e.what() << std::endl;
    return Config();
  }

  if(!config_node)
  {
    std::cerr << "Could not parse the config-file [" << file
              << "]" << std::endl;
    return Config();
  }

  return Config(config_node, file);
}

//==============================================================================
bool Config::parse(const YAML::Node& config_node, const std::string& file)
{
  _okay = false;

  if(!config_node.IsMap())
  {
    std::cerr << "The config-file [" << file << "] needs to be a map "
              << "containing [systems], and possibly [routes], [topics], "
              << "and/or [services]!" << std::endl;
    return false;
  }

  const YAML::Node& systems = config_node["systems"];
  if(!systems || !systems.IsMap())
  {
    std::cerr << "The config-file [" << file << "] is missing a "
              << "dictionary for [systems]! You must specify at least two "
              << "systems in your config-file." << std::endl;
    return false;
  }

  for(YAML::const_iterator it = systems.begin(); it != systems.end(); ++it)
  {
    const std::string middleware_alias = it->first.as<std::string>();

    const YAML::Node& config = it->second;

    const YAML::Node& type_node = config["type"];
    const std::string middleware = type_node?
          type_node.as<std::string>() : middleware_alias;

    m_middlewares.insert(
          std::make_pair(
            middleware_alias, MiddlewareConfig{middleware, config}));
  }

  if(m_middlewares.size() < 2)
  {
    std::cerr << "The config-file [" << file << "] does not specify enough "
              << "middlewares [" << m_middlewares.size() << "]! The "
              << "minimum is 2." << std::endl;
    return false;
  }

  auto read_route = [&](const std::string& key, const YAML::Node& n) -> bool
  {
    return add_named_route(key, n, m_topic_routes, m_service_routes);
  };
  if(!read_dictionary(config_node, "routes", file, read_route))
    return false;

  auto read_topic = [&](const std::string& key, const YAML::Node& n) -> bool
  {
    return add_topic_config(key, n, m_topic_routes, m_topic_configs);
  };
  if(!read_dictionary(config_node, "topics", file, read_topic))
    return false;

  auto read_service = [&](const std::string& key, const YAML::Node& n) -> bool
  {
    return add_service_config(key, n, m_service_routes, m_service_configs);
  };
  if(!read_dictionary(config_node, "services", file, read_service))
    return false;

  for(const auto& entry : m_topic_configs)
  {
    const TopicConfig& config = entry.second;
    const std::set<std::string>& middlewares = config.route.all();
    for(const std::string& mw : middlewares)
    {
      if(m_middlewares.find(mw) == m_middlewares.end())
      {
        std::cerr << "Unrecognized system [" << mw << "] requested for topic ["
                  << entry.first << "]" << std::endl;
        return false;
      }
      m_required_types[mw].messages.insert(config.message_type);
    }
  }

  for(const auto& entry : m_service_configs)
  {
    const ServiceConfig& config = entry.second;
    const std::set<std::string>& middlewares = config.route.all();
    for(const std::string& mw : middlewares)
    {
      if(m_middlewares.find(mw) == m_middlewares.end())
      {
        std::cerr << "Unrecognized system [" << mw << "] requested for service "
                  << "[" << entry.first << "]" << std::endl;
        return false;
      }
      m_required_types[mw].services.insert(config.service_type);
    }
  }

  std::size_t active_mw_count = 0;
  for(const auto& mw_entry : m_middlewares)
  {
    const std::string& mw_name = mw_entry.first;
    const auto it = m_required_types.find(mw_name);
    if(it == m_required_types.end())
    {
      std::cout << "WARNING: The middleware [" << mw_name << "] of type ["
                << mw_entry.second.type << "] is not being used in any topics "
                << "or services!" << std::endl;
    }
    else
    {
      ++active_mw_count;
    }
  }

  if(active_mw_count < 2)
  {
    std::cerr << "Fewer than 2 middlewares [" << active_mw_count << "] are "
              << "being used in this current configuration! This means that "
              << "running soss will not be useful!" << std::endl;
    return false;
  }

  _okay = true;
  return true;
}

//==============================================================================
bool Config::load_middlewares(SystemHandleInfoMap& info_map) const
{
  for(const auto& mw_entry : m_middlewares)
  {
    const std::string& mw_name = mw_entry.first;
    const MiddlewareConfig& mw_config = mw_entry.second;
    const std::string& middleware_type = mw_config.type;

    const Search::Implementation search(mw_config.type);

    std::vector<std::string> checked_paths;
    const std::string path = search.find_middleware_mix(checked_paths);
    if(path.empty())
    {
      std::string message =
          "Unable to find .mix file for middleware [" + middleware_type + "].\n"
          "The following locations were checked unsucessfully: ";
      for(const std::string& checked : checked_paths)
        message += "\n - " + checked;

      message +=
          "\nTry adding your middleware's install path to SOSS_PREFIX_PATH "
          "or SOSS_" + to_env_format(middleware_type) + "_PREFIX_PATH "
          "environment variables.";

      std::cerr << message << std::endl;
      return false;
    }

    if(!Mix::from_file(path).load())
      return false;

    // After loading the mix file, the middleware's plugin library should be
    // loaded, and it should be possible to find the middleware info in the
    // internal Register.
    internal::SystemHandleInfo info = internal::Register::get(middleware_type);

    if(!info)
      return false;

    bool configured = true;
    const auto requirements = m_required_types.find(mw_name);
    if(requirements != m_required_types.end())
      configured &=
          info.handle->configure(requirements->second, mw_config.config_node);

    if(!configured)
      return false;

    info_map.insert(std::make_pair(mw_name, std::move(info)));
  }

  return true;
}

//==============================================================================
bool Config::configure_topics(const SystemHandleInfoMap& info_map) const
{
  bool valid = true;
  for(const auto& entry : m_topic_configs)
  {
    const std::string& topic_name = entry.first;
    const TopicConfig& config = entry.second;

    std::vector<std::shared_ptr<TopicPublisher>> publishers;
    publishers.reserve(config.route.to.size());
    for(const std::string& to : config.route.to)
    {
      const auto it = info_map.find(to);
      if(it == info_map.end() || !it->second.topic_publisher)
      {
        std::cerr << "Could not find topic publishing capabilities for system "
                  << "named [" << to << "], requested for topic ["
                  << topic_name << "]" <<  std::endl;
        valid = false;
        continue;
      }

      std::shared_ptr<TopicPublisher> publisher =
          it->second.topic_publisher->advertise(
            remap_if_needed(to, config.remap, topic_name),
            config.message_type,
            config_or_empty_node(to, config.middleware_configs));

      if(!publisher)
      {
        std::cerr << "The system [" << to << "] failed to produce a publisher "
                  << "for the topic [" << topic_name << "] and message type ["
                  << config.message_type << "]" << std::endl;
        valid = false;
      }
      else
      {
        publishers.push_back(publisher);
      }
    }

    TopicSubscriberSystem::SubscriptionCallback callback =
        [=](const xtypes::DynamicData& message)
    {
      for(const std::shared_ptr<TopicPublisher>& publisher : publishers)
      {
        publisher->publish(message);
      }
    };

    for(const std::string& from : config.route.from)
    {
      const auto it = info_map.find(from);
      if(it == info_map.end() || !it->second.topic_subscriber)
      {
        std::cerr << "Could not find topic subscribing capabilities for system "
                  << "named [" << from << "], requested for topic ["
                  << topic_name << "]" << std::endl;
        valid = false;
        continue;
      }

      valid &= it->second.topic_subscriber->subscribe(
            remap_if_needed(from, config.remap, topic_name),
            config.message_type, callback,
            config_or_empty_node(from, config.middleware_configs));
    }
  }

  return valid;
}

//==============================================================================
bool Config::configure_services(const SystemHandleInfoMap& info_map) const
{
  bool valid = true;
  for(const auto& entry : m_service_configs)
  {
    const std::string& service_name = entry.first;
    const ServiceConfig& config = entry.second;

    const std::string& server = config.route.server;
    const auto it = info_map.find(server);
    if(it == info_map.end() || !it->second.service_provider)
    {
      std::cerr << "Could not find service providing capabilities for system "
                << "named [" << server << "], requested for service ["
                << service_name << "]" << std::endl;
      valid = false;
      continue;
    }

    std::shared_ptr<ServiceProvider> provider =
        it->second.service_provider->create_service_proxy(
          remap_if_needed(server, config.remap, service_name),
          config.service_type,
          config_or_empty_node(server, config.middleware_configs));

    if(!provider)
    {
      std::cerr << "Failed to create a service provider in middleware ["
                << server << "] for service type [" << config.service_type
                << "]" << std::endl;
      return false;
    }

    ServiceClientSystem::RequestCallback callback =
        [=](const xtypes::DynamicData& request,
            ServiceClient& client,
            const std::shared_ptr<void>& call_handle)
    {
      provider->call_service(request, client, call_handle);
    };

    for(const std::string& client : config.route.clients)
    {
      const auto it = info_map.find(client);
      if(it == info_map.end() || !it->second.service_client)
      {
        std::cerr << "Could not find service client capabilities for system "
                  << "named [" << client << "], requested for service ["
                  << service_name << "]" << std::endl;
        valid = false;
        continue;
      }

      valid &= it->second.service_client->create_client_proxy(
            remap_if_needed(client, config.remap, service_name),
            config.service_type, callback,
            config_or_empty_node(client, config.middleware_configs));
    }
  }

  return valid;
}

} // namespace internal
} // namespace soss
