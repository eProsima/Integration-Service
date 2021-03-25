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

  if (node.IsSequence())
  {
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
    {
      values.insert(it->as<std::string>());
    }
  }
  else if (node.IsScalar())
  {
    values.insert(node.as<std::string>());
  }
  else
  {
    valid = false;
  }

  if (!valid)
  {
    std::cerr << "config-file [" << field << "] entry in " << route_type
              << " route must point to a string or list of at least one string"
              << std::endl;
    valid = false;
  }

  return valid;
}

//==============================================================================
std::unique_ptr<TopicRoute> parse_topic_route(
    const YAML::Node& node)
{
  auto route = std::make_unique<TopicRoute>();
  bool valid = true;

  valid &= scalar_or_list_node_to_set(
    node["from"], route->from, "from", "topic");

  valid &= scalar_or_list_node_to_set(
    node["to"], route->to, "to", "topic");

  if (!valid)
  {
    return nullptr;
  }

  return route;
}

//==============================================================================
std::unique_ptr<ServiceRoute> parse_service_route(
    const YAML::Node& node)
{
  bool valid = true;
  auto route = std::make_unique<ServiceRoute>();

  const YAML::Node& server = node["server"];
  if (!server)
  {
    std::cerr << "config-file service route must contain a [server] entry!"
              << std::endl;
    valid = false;
  }

  if (!server.IsScalar() || server.as<std::string>().empty())
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

  if (!valid)
  {
    return nullptr;
  }

  return route;
}

//==============================================================================
bool add_types(
    const YAML::Node& node,
    const std::string& filename,
    std::map<std::string, xtypes::DynamicType::Ptr>& types)
{
  if (!node["types"])
  {
    return true;
  }

  if (!node["types"]["idls"])
  {
    std::cerr << "The config file [" << filename << "] has a 'types' entry which doesn't contains an 'idls' entry."
              << std::endl;
    return false;
  }

  if (!node["types"]["idls"].IsSequence())
  {
    std::cerr << "The config file [" << filename << "] has an 'idls' entry but "
              << "it's not a sequence."
              << std::endl;
    return false;
  }

  std::vector<std::string> include_paths;
  if (node["types"]["paths"])
  {
    for (auto& path : node["types"]["paths"])
    {
      include_paths.push_back(path.as<std::string>());
    }
  }

  int idl_index = 0;
  for (auto& entry: node["types"]["idls"])
  {
    xtypes::idl::Context context;
    context.allow_keyword_identifiers = true;
    if (!include_paths.empty())
    {
      context.include_paths = include_paths;
    }
    xtypes::idl::parse(entry.as<std::string>(), context);

    if (context.success)
    {
      for (auto& type: context.get_all_scoped_types())
      {
        types.insert(type);
        // Some SH expect the types without the initial "::", and others with it,
        // so we must add both of them.
        if (type.first.find("::") == 0)
        {
            types.emplace(std::make_pair(type.first.substr(2), type.second));
        }
        else
        {
            types.emplace(std::make_pair("::" + type.first, type.second));
        }
      }
      if (types.empty())
      {
          std::cout << "Parsing the idl '" << idl_index << "' placed in the yaml config. "
                    << "The parsing was successful but no types were found."
                    << std::endl;
      }
    }
    else
    {
      std::cerr << "Error parsing the idl '" << idl_index << "' placed in the yaml config. "
                << "Please, review and fix your idl specification syntax."
                << std::endl;
    }
    idl_index++;
  }

  return true;
}

//==============================================================================
bool add_named_route(
    const std::string& name,
    const YAML::Node& node,
    std::map<std::string, TopicRoute>& topic_routes,
    std::map<std::string, ServiceRoute>& service_routes)
{
  bool inserted = false;
  if (node["from"] && node["to"])
  {
    const auto route = parse_topic_route(node);
    if (!route)
    {
      std::cerr << "Failed to parse the route named ["
                << name << "]" << std::endl;
      return false;
    }

    inserted = topic_routes.insert(std::make_pair(name, *route)).second;
  }
  else if (node["server"] && node["clients"])
  {
    const auto route = parse_service_route(node);
    if (!route)
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

  if (!inserted)
  {
    std::cerr << "Duplicate route named [" << name << "]!" << std::endl;
  }

  return inserted;
}

//==============================================================================
bool set_middleware_config(
    std::map<std::string, YAML::Node>& middleware_configs,
    const TopicRoute& route,
    const YAML::Node& node)
{
  // types, route, and remap should be ignored?
  for (const std::string& from : route.from)
  {
    middleware_configs[from] = node;
  }
  for (const std::string& to : route.to)
  {
    middleware_configs[to] = node;
  }
  return true;
}

//==============================================================================
bool set_middleware_config(
    std::map<std::string, YAML::Node>& middleware_configs,
    const ServiceRoute& route,
    const YAML::Node& node)
{
  // types, route, and remap should be ignored?
  middleware_configs[route.server] = node;
  for (const std::string& client : route.clients)
  {
    middleware_configs[client] = node;
  }
  return true;
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
    std::function<void(ConfigType&, std::string)> set_reply_type,
    std::function<void(ConfigType&, RouteType)> set_route,
    std::function<std::unique_ptr<RouteType>(const YAML::Node&)> parse_route)
{
  bool valid = true;
  ConfigType config;

  const YAML::Node& type = node["type"];
  if (!type)
  {
    if (channel_type == "service")
    {
      const YAML::Node& request_type = node["request_type"];
      const YAML::Node& reply_type = node["reply_type"];

      if (!request_type && !reply_type)
      {
        std::cerr << channel_type << " configuration [" << name
                  << "] is missing its [type], or [request_type] and [reply_type] fields!" << std::endl;
        valid = false;
      }
      else
      {
        set_type(config, request_type.as<std::string>());
        set_reply_type(config, reply_type.as<std::string>());
      }
    }
    else
    {
      std::cerr << channel_type << " configuration [" << name
                << "] is missing its [type] field!" << std::endl;
      valid = false;
    }
  }
  else
  {
    set_type(config, type.as<std::string>());
  }

  const YAML::Node& route = node["route"];
  if (!route)
  {
    std::cerr << channel_type << " configuration [" << name
              << "] is missing its [route] field!" << std::endl;
    valid = false;
  }
  else
  {
    if (route.IsScalar())
    {
      const std::string& route_name = route.as<std::string>();
      const auto it = predefined_routes.find(route_name);
      if (it == predefined_routes.end())
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
    else if (route.IsMap())
    {
      const auto route_config = parse_route(route);
      if (!route_config)
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
  if (remap)
  {
    if (!remap.IsMap())
    {
      std::cerr << "A [remap] field was given for the " << channel_type
                << " configuration [" << name
                << "], but it is not a dictionary!" << std::endl;
      valid = false;
    }
    else
    {
      for (YAML::const_iterator it = remap.begin(); it != remap.end(); ++it)
      {
        TopicInfo& remap = config.remap[it->first.as<std::string>()];
        if (it->second["topic"])
        {
          remap.name = it->second["topic"].as<std::string>();
        }
        if (it->second["type"])
        {
          remap.type = it->second["type"].as<std::string>();
        }
        if (it->second["request_type"])
        {
          remap.type = it->second["request_type"].as<std::string>();
        }
        if (it->second["reply_type"])
        {
          remap.reply_type = it->second["reply_type"].as<std::string>();
        }
      }
    }
  }

  // TODO(MXG): Come up with a yaml scene for specifying middleware
  // configurations per topic/service and then fill in the middleware_configs
  // map here
  // Proposal
  if (valid)
  {
    valid = set_middleware_config(config.middleware_configs, config.route, node);
  }
  // End of proposal

  if (valid)
  {
    if (!config_map.insert(std::make_pair(name, config)).second)
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
    [](TopicConfig& c, std::string s){
          c.message_type = std::move(s);
        },
    [](TopicConfig&, std::string){},
    [](TopicConfig& c, TopicRoute r){
          c.route = std::move(r);
        },
    [](const YAML::Node& node){
          return parse_topic_route(node);
        });
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
    [](ServiceConfig& c, std::string s){
          c.request_type = std::move(s);
        },
    [](ServiceConfig& c, std::string s){
          c.reply_type = std::move(s);
        },
    [](ServiceConfig& c, ServiceRoute r){
          c.route = std::move(r);
        },
    [](const YAML::Node& node){
          return parse_service_route(node);
        });
}

//==============================================================================
using ReadDictEntry =
    std::function<bool (const std::string& key, const YAML::Node& value)>;

bool read_dictionary(
    const YAML::Node& config_node,
    const std::string& field_name,
    const std::string& filename,
    const ReadDictEntry& read_fcn)
{
  const YAML::Node& field = config_node[field_name];
  if (field)
  {
    if (!field.IsMap())
    {
      std::cerr << "The config file [" << filename << "] has a [" << field_name
                << "] field, but it is not pointing to a dictionary of "
        // We use this to make the field name singular
                << field_name.substr(0, field_name.size() - 1)
                << "configurations!" << std::endl;
      return false;
    }

    bool configs_okay = true;
    for (YAML::const_iterator it = field.begin(); it != field.end(); ++it)
    {
      configs_okay &= read_fcn(it->first.as<std::string>(), it->second);
    }

    if (!configs_okay)
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
  if (it == map.end())
  {
    return YAML::Node();
  }

  return it->second;
}

//==============================================================================
TopicInfo remap_if_needed(
    const std::string& middleware,
    const std::map<std::string, TopicInfo>& remap,
    const TopicInfo& original)
{
  const auto it = remap.find(middleware);
  if (it == remap.end())
  {
    return original;
  }

  TopicInfo config = original;
  if (!it->second.name.empty())
  {
    config.name = it->second.name;
  }
  if (!it->second.type.empty())
  {
    config.type = it->second.type;
  }
  if (!it->second.reply_type.empty())
  {
    config.reply_type = it->second.reply_type;
  }
  return config;
}

} // anonymous namespace

//==============================================================================
Config::Config(
    const YAML::Node& node,
    const std::string& filename)
{
  if (node && !node.IsNull())
  {
    parse(node, filename);
  }
}

//==============================================================================
Config Config::from_file(
    const std::string& file)
{
  YAML::Node config_node;
  try
  {
    config_node = YAML::LoadFile(file);
  }
  catch (const std::exception& e)
  {
    std::cerr << "Could not parse the config-file named [" << file
              << "]: " << e.what() << std::endl;
    return Config();
  }

  if (!config_node)
  {
    std::cerr << "Could not parse the config-file [" << file
              << "]" << std::endl;
    return Config();
  }

  return Config(config_node, file);
}

//==============================================================================
bool Config::parse(
    const YAML::Node& config_node,
    const std::string& file)
{
  _okay = false;

  if (!config_node.IsMap())
  {
    std::cerr << "The config-file [" << file << "] needs to be a map "
              << "containing [systems], and possibly [routes], [topics], "
              << "and/or [services]!" << std::endl;
    return false;
  }

  const YAML::Node& systems = config_node["systems"];
  if (!systems || !systems.IsMap())
  {
    std::cerr << "The config-file [" << file << "] is missing a "
              << "dictionary for [systems]! You must specify at least two "
              << "systems in your config-file." << std::endl;
    return false;
  }

  for (YAML::const_iterator it = systems.begin(); it != systems.end(); ++it)
  {
    const std::string middleware_alias = it->first.as<std::string>();

    const YAML::Node& config = it->second;

    const YAML::Node& type_node = config["type"];
    const std::string middleware = type_node ?
        type_node.as<std::string>() : middleware_alias;

    const YAML::Node& types_from_node = config["types-from"];
    std::vector<std::string> types_from;

    if (types_from_node)
    {
      if (types_from_node.IsSequence())
      {
        for (const YAML::Node& types : types_from_node)
        {
          types_from.push_back(types.as<std::string>());
        }
      }
      else
      {
        types_from.push_back(types_from_node.as<std::string>());
      }
    }

    m_middlewares.insert(
      std::make_pair(
        middleware_alias, MiddlewareConfig{middleware, types_from, config}));
  }

  if (m_middlewares.size() < 2)
  {
    std::cerr << "The config-file [" << file << "] does not specify enough "
              << "middlewares [" << m_middlewares.size() << "]! The "
              << "minimum is 2." << std::endl;
    return false;
  }

  add_types(config_node, file, m_types);

  auto read_route = [&](const std::string& key, const YAML::Node& n) -> bool
      {
        return add_named_route(key, n, m_topic_routes, m_service_routes);
      };
  if (!read_dictionary(config_node, "routes", file, read_route))
  {
    return false;
  }

  auto read_topic = [&](const std::string& key, const YAML::Node& n) -> bool
      {
        return add_topic_config(key, n, m_topic_routes, m_topic_configs);
      };
  if (!read_dictionary(config_node, "topics", file, read_topic))
  {
    return false;
  }

  auto read_service = [&](const std::string& key, const YAML::Node& n) -> bool
      {
        return add_service_config(key, n, m_service_routes, m_service_configs);
      };
  if (!read_dictionary(config_node, "services", file, read_service))
  {
    return false;
  }

  for (const auto& entry : m_topic_configs)
  {
    const TopicConfig& config = entry.second;
    const std::set<std::string>& middlewares = config.route.all();
    for (const std::string& mw : middlewares)
    {
      if (m_middlewares.find(mw) == m_middlewares.end())
      {
        std::cerr << "Unrecognized system [" << mw << "] requested for topic ["
                  << entry.first << "]" << std::endl;
        return false;
      }

      auto it_mw_remap = config.remap.find(mw);
      if (it_mw_remap == config.remap.end() || it_mw_remap->second.type == "")
      {
        m_required_types[mw].messages.insert(config.message_type);
      }
    }

    for (auto&& it_remap: config.remap)
    {
      if (m_middlewares.find(it_remap.first) == m_middlewares.end())
      {
        std::cerr << "Unrecognized system [" << it_remap.first << "] requested for mapping topic "
                  << "[" << entry.first << "]" << std::endl;
        return false;
      }
      m_required_types[it_remap.first].messages.insert(it_remap.second.type);
    }
  }

  for (const auto& entry : m_service_configs)
  {
    const ServiceConfig& config = entry.second;
    const std::set<std::string>& middlewares = config.route.all();
    for (const std::string& mw : middlewares)
    {
      if (m_middlewares.find(mw) == m_middlewares.end())
      {
        std::cerr << "Unrecognized system [" << mw << "] requested for service "
                  << "[" << entry.first << "]" << std::endl;
        return false;
      }

      auto it_mw_remap = config.remap.find(mw);
      if (it_mw_remap == config.remap.end() || it_mw_remap->second.type == "")
      {
        m_required_types[mw].services.insert(config.request_type);
      }
      if (it_mw_remap == config.remap.end() || it_mw_remap->second.reply_type == "")
      {
        m_required_types[mw].services.insert(config.reply_type);
      }
    }

    for (auto&& it_remap: config.remap)
    {
      if (m_middlewares.find(it_remap.first) == m_middlewares.end())
      {
        std::cerr << "Unrecognized system [" << it_remap.first << "] requested for mapping service "
                  << "[" << entry.first << "]" << std::endl;
        return false;
      }
      m_required_types[it_remap.first].services.insert(it_remap.second.type);
      if (!it_remap.second.reply_type.empty())
      {
        m_required_types[it_remap.first].services.insert(it_remap.second.reply_type);
      }
    }
  }

  std::size_t active_mw_count = 0;
  for (const auto& mw_entry : m_middlewares)
  {
    const std::string& mw_name = mw_entry.first;
    const auto it = m_required_types.find(mw_name);
    if (it == m_required_types.end())
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

  if (active_mw_count < 2)
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
bool Config::load_middlewares(
    SystemHandleInfoMap& info_map) const
{
  for (const auto& mw_entry : m_middlewares)
  {
    const std::string& mw_name = mw_entry.first;
    const MiddlewareConfig& mw_config = mw_entry.second;
    const std::string& middleware_type = mw_config.type;

    const Search::Implementation search(mw_config.type);

    std::vector<std::string> checked_paths;
    const std::string path = search.find_middleware_mix(checked_paths);
    if (path.empty())
    {
      std::string message =
          "Unable to find .mix file for middleware [" + middleware_type + "].\n"
          "The following locations were checked unsucessfully: ";
      for (const std::string& checked : checked_paths)
      {
        message += "\n - " + checked;
      }

      message +=
          "\nTry adding your middleware's install path to SOSS_PREFIX_PATH "
          "or SOSS_" + to_env_format(middleware_type) + "_PREFIX_PATH "
          "environment variables.";

      std::cerr << message << std::endl;
      return false;
    }

    if (!Mix::from_file(path).load())
    {
      return false;
    }

    // After loading the mix file, the middleware's plugin library should be
    // loaded, and it should be possible to find the middleware info in the
    // internal Register.
    internal::SystemHandleInfo info = internal::Register::get(middleware_type);

    if (!info)
    {
      return false;
    }

    bool configured = true;
    const auto requirements = m_required_types.find(mw_name);
    if (requirements != m_required_types.end())
    {
      for (const std::string& required_type: requirements->second.messages)
      {
        auto type_it = m_types.find(required_type);
        if (type_it != m_types.end())
        {
          info.types.emplace(*type_it);
        }
      }

      for (const std::string& required_type: requirements->second.services)
      {
        auto type_it = m_types.find(required_type);
        if (type_it != m_types.end())
        {
          info.types.emplace(*type_it);
        }
      }

      configured = info.handle->configure(requirements->second, mw_config.config_node, info.types);
    }

    if (configured)
    {
      info_map.insert(std::make_pair(mw_name, std::move(info)));
    }
    else
    {
      return false;
    }
  }

  for (const auto& mw_entry : m_middlewares)
  {
    const std::string& mw_name = mw_entry.first;
    const MiddlewareConfig& mw_config = mw_entry.second;
    auto& types = info_map.at(mw_name).types;

    if (!mw_config.types_from.empty())
    {
      for (const std::string& type : mw_config.types_from)
      {
        const auto it = info_map.find(type);
        if (it == info_map.end())
        {
          std::cerr << "'types-from' references to a non-existant middleware" << std::endl;
          return false;
        }

        for (auto&& it_type: info_map.at(type).types)
        {
          types.emplace(it_type.second->name(), it_type.second);
        }
      }

      const auto requirements = m_required_types.find(mw_name);
      for (const std::string& required_type: requirements->second.messages)
      {
        if (!types.count(required_type))
        {
          std::cerr << "The middleware '" << mw_name << "' must satisfy the required "
                    << "type '" << required_type << "'"
                    << std::endl;
        }
      }

      for (const std::string& required_type: requirements->second.services)
      {
        if (!types.count(required_type))
        {
          std::cerr << "The middleware '" << mw_name << "' must satisfy the required "
                    << "type '" << required_type << "'"
                    << std::endl;
        }
      }
    }
  }

  return true;
}

//==============================================================================
bool Config::configure_topics(
    const SystemHandleInfoMap& info_map) const
{
  bool valid = true;
  for (const auto& entry : m_topic_configs)
  {
    const std::string& topic_name = entry.first;
    const TopicConfig& config = entry.second;

    if (!check_topic_compatibility(info_map, topic_name, config))
    {
      valid = false;
      continue;
    }

    struct PublisherData
    {
      std::shared_ptr<TopicPublisher> publisher;
      const xtypes::DynamicType& type;
    };
    std::vector<PublisherData> publishers;

    publishers.reserve(config.route.to.size());
    for (const std::string& to : config.route.to)
    {
      const auto it_to = info_map.find(to);
      if (it_to == info_map.end() || !it_to->second.topic_publisher)
      {
        std::cerr << "Could not find topic publishing capabilities for system "
                  << "named [" << to << "], requested for topic ["
                  << topic_name << "]" <<  std::endl;
        valid = false;
        continue;
      }

      TopicInfo topic_info = remap_if_needed(to, config.remap, {topic_name, config.message_type, ""});
      const xtypes::DynamicType* pub_type = resolve_type(it_to->second.types, topic_info.type);
      //const xtypes::DynamicType& pub_type = *it_to->second.types.find(topic_info.type)->second;
      std::shared_ptr<TopicPublisher> publisher =
          it_to->second.topic_publisher->advertise(
            topic_info.name,
            //*pub_type,
            (topic_info.type.find(".") == std::string::npos
                       ? *pub_type
                       : *m_types.at(topic_info.type.substr(0, topic_info.type.find(".")))),
            config_or_empty_node(to, config.middleware_configs));

      if (!publisher)
      {
        std::cerr << "The system [" << to << "] failed to produce a publisher "
                  << "for the topic [" << topic_name << "] and message type ["
                  << config.message_type << "]" << std::endl;
        valid = false;
      }
      else
      {
        publishers.push_back({publisher, *pub_type});
      }
    }

    for (const std::string& from : config.route.from)
    {
      const auto it_from = info_map.find(from);
      if (it_from == info_map.end() || !it_from->second.topic_subscriber)
      {
        std::cerr << "Could not find topic subscribing capabilities for system "
                  << "named [" << from << "], requested for topic ["
                  << topic_name << "]" << std::endl;
        valid = false;
        continue;
      }

      TopicInfo topic_info = remap_if_needed(from, config.remap, {topic_name, config.message_type, ""});
      const xtypes::DynamicType* sub_type = resolve_type(it_from->second.types, topic_info.type);
      //const xtypes::DynamicType& sub_type = *it_from->second.types.find(topic_info.type)->second;

      struct Publication
      {
        std::shared_ptr<TopicPublisher> publisher;
        const xtypes::DynamicType& type;
        xtypes::TypeConsistency consistency;
      };

      std::vector<Publication> publications;
      publications.reserve(publishers.size());
      for (const auto& pub : publishers)
      {
        publications.push_back({pub.publisher, pub.type, pub.type.is_compatible(*sub_type)});
      }

      TopicSubscriberSystem::SubscriptionCallback callback =
          [ = ](const xtypes::DynamicData& message)
          {
            for (const Publication& publication : publications)
            {
              if (publication.consistency == xtypes::TypeConsistency::EQUALS)
              {
                publication.publisher->publish(message);
              }
              else               //previously ensured that TypeConsistency is not NONE
              {
                xtypes::DynamicData compatible_message(message, publication.type);
                publication.publisher->publish(compatible_message);
              }
            }
          };

      valid &= it_from->second.topic_subscriber->subscribe(
        topic_info.name,
        //*sub_type,
        (topic_info.type.find(".") == std::string::npos
                   ? *sub_type
                   : *m_types.at(topic_info.type.substr(0, topic_info.type.find(".")))),
        callback,
        config_or_empty_node(from, config.middleware_configs));
    }
  }

  return valid;
}

//==============================================================================
bool Config::configure_services(
    const SystemHandleInfoMap& info_map) const
{
  bool valid = true;
  for (const auto& entry : m_service_configs)
  {
    const std::string& service_name = entry.first;
    const ServiceConfig& config = entry.second;

    if (!check_service_compatibility(info_map, service_name, config))
    {
      valid = false;
      continue;
    }

    const std::string& server = config.route.server;
    const auto it = info_map.find(server);
    if (it == info_map.end() || !it->second.service_provider)
    {
      std::cerr << "Could not find service providing capabilities for system "
                << "named [" << server << "], requested for service ["
                << service_name << "]" << std::endl;
      valid = false;
      continue;
    }

    TopicInfo server_info = remap_if_needed(server, config.remap, {service_name, config.request_type, config.reply_type});
    const xtypes::DynamicType* server_type = resolve_type(it->second.types, server_info.type);
    //const xtypes::DynamicType& server_type = *it->second.types.find(server_info.type)->second;

    std::shared_ptr<ServiceProvider> provider = nullptr;
    if (!config.reply_type.empty())
    {
      const xtypes::DynamicType* server_reply_type = resolve_type(it->second.types, server_info.reply_type);

      provider =
          it->second.service_provider->create_service_proxy(
        server_info.name,
        (server_info.type.find(".") == std::string::npos
                   ? *server_type
                   : *m_types.at(server_info.type.substr(0, server_info.type.find(".")))),
        (server_info.reply_type.find(".") == std::string::npos
                   ? *server_reply_type
                   : *m_types.at(server_info.reply_type.substr(0, server_info.reply_type.find(".")))),
        config_or_empty_node(server, config.middleware_configs));
    }
    else
    {
      provider =
          it->second.service_provider->create_service_proxy(
        server_info.name,
        (server_info.type.find(".") == std::string::npos
                   ? *server_type
                   : *m_types.at(server_info.type.substr(0, server_info.type.find(".")))),
        config_or_empty_node(server, config.middleware_configs));
    }

    if (!provider)
    {
      std::cerr << "Failed to create a service provider in middleware ["
                << server << "] for service type [" << config.request_type << "]";
      if (!config.reply_type.empty())
      {
          std::cerr << " reply type [" << config.reply_type << "]";
      }
      std::cerr << std::endl;
      return false;
    }

    for (const std::string& client : config.route.clients)
    {
      const auto it = info_map.find(client);
      if (it == info_map.end() || !it->second.service_client)
      {
        std::cerr << "Could not find service client capabilities for system "
                  << "named [" << client << "], requested for service ["
                  << service_name << "]" << std::endl;
        valid = false;
        continue;
      }

      TopicInfo client_info = remap_if_needed(client, config.remap,
                                              {service_name, config.request_type, config.reply_type});
      const xtypes::DynamicType* client_type = resolve_type(it->second.types, client_info.type);
      //const xtypes::DynamicType& client_type = *it->second.types.find(client_info.type)->second;
      xtypes::TypeConsistency consistency = client_type->is_compatible(*server_type);

      ServiceClientSystem::RequestCallback callback =
          [ = ](const xtypes::DynamicData& request,
              ServiceClient& client,
              const std::shared_ptr<void>& call_handle)
          {
            if (consistency == xtypes::TypeConsistency::EQUALS)
            {
              provider->call_service(request, client, call_handle);
            }
            else             //previously ensured that TypeConsistency is not NONE
            {
              xtypes::DynamicData compatible_request(request, *server_type);
              provider->call_service(compatible_request, client, call_handle);
            }
          };

      if (client_info.reply_type.empty())
      {
        valid &= it->second.service_client->create_client_proxy(
          client_info.name,
          //*client_type,
          (client_info.type.find(".") == std::string::npos
                     ? *client_type
                     : *m_types.at(client_info.type.substr(0, client_info.type.find(".")))),
          callback,
          config_or_empty_node(client, config.middleware_configs));
      }
      else
      {
        const xtypes::DynamicType* client_reply_type = resolve_type(it->second.types, client_info.reply_type);

        valid &= it->second.service_client->create_client_proxy(
          client_info.name,
          //*client_type,
          (client_info.type.find(".") == std::string::npos
                     ? *client_type
                     : *m_types.at(client_info.type.substr(0, client_info.type.find(".")))),
          (client_info.reply_type.find(".") == std::string::npos
                     ? *client_reply_type
                     : *m_types.at(client_info.reply_type.substr(0, client_info.reply_type.find(".")))),
          callback,
          config_or_empty_node(client, config.middleware_configs));
      }
    }
  }

  return valid;
}

//==============================================================================
bool Config::check_topic_compatibility(
    const SystemHandleInfoMap& info_map,
    const std::string& topic_name,
    const TopicConfig& config) const
{
  bool valid = true;
  for (const std::string& from : config.route.from)
  {
    const auto it_from = info_map.find(from);
    TopicInfo topic_info_from = remap_if_needed(from, config.remap, {topic_name, config.message_type, ""});
    const xtypes::DynamicType* from_type = resolve_type(it_from->second.types, topic_info_from.type);
    //const auto from_type = it_from->second.types.find(topic_info_from.type);
    /*
    if (from_type == it_from->second.types.end())
    {
      std::cerr << "Type [" << topic_info_from.type << "] not defined in middleware ["
                << it_from->first << "]" << std::endl;
      valid = false;
      continue;
    }
    */

    for (const std::string& to : config.route.to)
    {
      const auto it_to = info_map.find(to);
      TopicInfo topic_info_to = remap_if_needed(to, config.remap, {topic_name, config.message_type, ""});
      const xtypes::DynamicType* to_type = resolve_type(it_from->second.types, topic_info_from.type);
      /*
      const auto to_type = it_to->second.types.find(topic_info_to.type);
      if (to_type == it_to->second.types.end())
      {
        std::cerr << "Type [" << topic_info_to.type << "] not defined in middleware ["
                  << it_to->first << "]" << std::endl;
        valid = false;
        continue;
      }
      */

      //xtypes::TypeConsistency consistency = from_type->second->is_compatible(*to_type->second);
      xtypes::TypeConsistency consistency = from_type->is_compatible(*to_type);
      if (consistency == xtypes::TypeConsistency::NONE)
      {
        std::cerr << "Remapping error: message type ["
                  << topic_info_from.type << "] from [" << it_from->first << "] is not compatible with ["
                  << topic_info_to.type << "] in [" << it_to->first << "]" << std::endl;
        valid = false;
        continue;
      }
      else if (consistency != xtypes::TypeConsistency::EQUALS)
      {
        std::cout << "The conversion between [" << topic_info_from.type << "] and ["
                  << topic_info_to.type << "] has been allowed by adding the following QoS policies: ";

        auto policy_name = [&](xtypes::TypeConsistency to_check, const std::string& name) -> std::string
            {
              return (consistency & to_check) == to_check ? "'" + name + "' " : "";
            };

        std::cout << policy_name(xtypes::TypeConsistency::IGNORE_TYPE_SIGN, "ignore type sign");
        std::cout << policy_name(xtypes::TypeConsistency::IGNORE_TYPE_WIDTH, "ignore type width");
        std::cout << policy_name(xtypes::TypeConsistency::IGNORE_SEQUENCE_BOUNDS, "ignore sequence bounds");
        std::cout << policy_name(xtypes::TypeConsistency::IGNORE_ARRAY_BOUNDS, "ignore array bounds");
        std::cout << policy_name(xtypes::TypeConsistency::IGNORE_STRING_BOUNDS, "ignore string bounds");
        std::cout << policy_name(xtypes::TypeConsistency::IGNORE_MEMBER_NAMES, "ignore member names");
        std::cout << policy_name(xtypes::TypeConsistency::IGNORE_MEMBERS, "ignore members");

        std::cout << std::endl;
      }
    }
  }

  return valid;
}

//==============================================================================
bool Config::check_service_compatibility(
    const SystemHandleInfoMap& info_map,
    const std::string& service_name,
    const ServiceConfig& config) const
{
  bool valid = true;
  for (const std::string& client : config.route.clients)
  {
    const auto it_client = info_map.find(client);
    TopicInfo topic_info_client = remap_if_needed(client, config.remap,
                                                  {service_name, config.request_type, config.reply_type});
    const xtypes::DynamicType* client_type = resolve_type(it_client->second.types, topic_info_client.type);
    /*
    const auto client_type = it_client->second.types.find(topic_info_client.type);
    if (client_type == it_client->second.types.end())
    {
      std::cerr << "Type [" << topic_info_client.type << "] not defined in middleware ["
                << it_client->first << "]" << std::endl;
      valid = false;
      continue;
    }
    */

    const auto it_server = info_map.find(config.route.server);
    TopicInfo topic_info_server = remap_if_needed(config.route.server, config.remap,
                                                  {service_name, config.request_type, config.reply_type});
    const xtypes::DynamicType* server_type = resolve_type(it_server->second.types, topic_info_server.type);
    /*
    const auto server_type = it_server->second.types.find(topic_info_server.type);
    if (server_type == it_server->second.types.end())
    {
      std::cerr << "Type [" << topic_info_server.type << "] not defined in middleware ["
                << it_server->first << "]" << std::endl;
      valid = false;
      continue;
    }
    */

    //if (client_type->second->is_compatible(*server_type->second) == xtypes::TypeConsistency::NONE)
    if (client_type->is_compatible(*server_type) == xtypes::TypeConsistency::NONE)
    {
      std::cerr << "Remapping error: service type ["
                << topic_info_client.type << "] from [" + it_client->first + "] can not be read as type ["
                << topic_info_server.type << "] in [" + it_server->first + "]" << std::endl;
      valid = false;
      continue;
    }

    if (!topic_info_client.reply_type.empty() && !topic_info_server.reply_type.empty())
    {
      const xtypes::DynamicType* client_reply = resolve_type(it_client->second.types, topic_info_client.reply_type);
      const xtypes::DynamicType* server_reply = resolve_type(it_server->second.types, topic_info_server.reply_type);

      if (client_reply->is_compatible(*server_reply) == xtypes::TypeConsistency::NONE)
      {
          std::cerr << "Remapping error: service reply type ["
                    << topic_info_client.reply_type << "] from [" + it_client->first + "] can not be read as type ["
                    << topic_info_server.reply_type << "] in [" + it_server->first + "]" << std::endl;
      }
    }
  }

  return valid;
}

const xtypes::DynamicType* Config::resolve_type(
        const TypeRegistry& types,
        const std::string& path) const
{
  if (path.find(".") == std::string::npos)
  {
    return types.find(path)->second.get();
  }

  std::string path_aux = path;
  const xtypes::DynamicType* type_ptr;
  std::string type = path_aux.substr(0, path_aux.find("."));
  std::string member;
  type_ptr = m_types.at(type).get();
  while (path_aux.find(".") != std::string::npos)
  {
    path_aux = path_aux.substr(path_aux.find(".") + 1);
    member = path_aux.substr(0, path_aux.find("."));
    if (type_ptr->is_aggregation_type())
    {
       const xtypes::AggregationType& aggregation = static_cast<const xtypes::AggregationType&>(*type_ptr);
       type_ptr = &aggregation.member(member).type();
    }
  }

  return type_ptr;
}

} // namespace internal
} // namespace soss
