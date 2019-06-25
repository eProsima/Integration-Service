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

#ifndef SOSS__INTERNAL__CONFIG_HPP
#define SOSS__INTERNAL__CONFIG_HPP

#include "register_system.hpp"

#include <yaml-cpp/yaml.h>

#include <set>
#include <vector>

namespace soss {
namespace internal {

//==============================================================================
struct MiddlewareConfig
{
  std::string type;
  YAML::Node config_node;
};

//==============================================================================
struct TopicRoute
{
  std::set<std::string> from;
  std::set<std::string> to;

  std::set<std::string> all() const
  {
    std::set<std::string> _all = from;
    _all.insert(to.begin(), to.end());
    return _all;
  }
};

//==============================================================================
struct ServiceRoute
{
  std::string server;
  std::set<std::string> clients;

  std::set<std::string> all() const
  {
    std::set<std::string> _all = clients;
    _all.insert(server);
    return _all;
  }
};

//==============================================================================
struct TopicConfig
{
  std::string message_type;
  TopicRoute route;

  /// key: middleware alias, value: that middleware's name for this topic
  std::map<std::string, std::string> remap;

  std::map<std::string, YAML::Node> middleware_configs;
};

//==============================================================================
struct ServiceConfig
{
  std::string service_type;
  ServiceRoute route;

  /// key: middleware alias, value: that middleware's name for this service
  std::map<std::string, std::string> remap;

  std::map<std::string, YAML::Node> middleware_configs;
};

//==============================================================================
class Config
{
public:

  Config(const YAML::Node& node = YAML::Node(),
         const std::string& filename="<text>");

  static Config from_file(const std::string& file);

  bool parse(const YAML::Node& node,
             const std::string& filename="<text>");

  bool okay() const { return _okay; }
  operator bool() const { return okay(); }

  bool load_middlewares(SystemHandleInfoMap& info_map);

  bool configure_topics(const SystemHandleInfoMap& info_map) const;

  bool configure_services(const SystemHandleInfoMap& info_map) const;

  std::map<std::string, MiddlewareConfig> m_middlewares;
  std::map<std::string, xtypes::DynamicType*> m_types;
  std::map<std::string, TopicRoute> m_topic_routes;
  std::map<std::string, ServiceRoute> m_service_routes;
  std::map<std::string, TopicConfig> m_topic_configs;
  std::map<std::string, ServiceConfig> m_service_configs;
  std::map<std::string, RequiredTypes> m_required_types;

private:

  bool _okay = false;

};

} // namespace internal
} // namespace soss

#endif // SOSS__INTERNAL__CONFIG_HPP
