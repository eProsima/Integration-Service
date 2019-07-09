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

#ifndef SOSS__INTERNAL__SEARCH_IMPL_HPP
#define SOSS__INTERNAL__SEARCH_IMPL_HPP

#include <soss/Search.hpp>

#include <map>
#include <list>
#include <soss/soss_export.hpp>

namespace soss {

//==============================================================================
std::string to_env_format(std::string str);

//==============================================================================
std::vector<std::string> get_environment_variable_path_list(
    const std::string& env_var);

//==============================================================================
class SOSS_EXPORT Search::Implementation
{
public:

  /// Used by the Instance class to set SOSS prefixes that were specified from
  /// the command line
  static void add_cli_soss_prefix(const std::string& path);

  /// Used by the Instance class to set middleware prefixes that were specified
  /// from the command line
  static void add_cli_middleware_prefix(
      const std::string& middleware,
      const std::string& path);

  static void set_config_file_directory(const std::string& path);

  Implementation(const std::string& middleware);

  void add_priority_middleware_prefix(const std::string& path);

  void add_fallback_middleware_prefix(const std::string& path);

  std::string find_file(
      std::string filename,
      const std::string& subdir,
      std::vector<std::string>& checked_paths) const;

  std::string find_middleware_mix(
      std::vector<std::string>& checked_paths) const;

  void search_relative_to_config(bool toggle);

  void search_relative_to_home(bool toggle);

  void search_system_prefixes(bool toggle);

  void search_soss_prefixes(bool toggle);

  void search_middleware_prefixes(bool toggle);

private:

  class PathSet
  {
  public:

    /// Add a path to this set. The most recently added paths will come first
    /// when iterating from begin() to end().
    void add_path(const std::string& path);

    std::list<std::string>::const_iterator begin() const;

    std::list<std::string>::const_iterator end() const;

    /// Toggle for whether or not this PathSet should be used
    bool active = true;

  private:
    std::map<std::string, std::list<std::string>::iterator> _added_paths;
    std::list<std::string> _path_list;
  };

  // This class is used to initialize the global paths when the library is first
  // loaded
  struct GlobalPathInitializer
  {
    GlobalPathInitializer();
  };

  std::string _middleware;

  static GlobalPathInitializer static_global_path_initializer;

  struct GlobalPaths
  {
    // SOSS prefixes set from the command line interface
    PathSet cli_soss_prefixes;

    // Middleware prefixes set from the command line interface
    std::map<std::string, PathSet> cli_middleware_prefixes;

    // SOSS prefixes for system directories
    PathSet system_soss_prefixes;

    // Path set that contains only the prefix of the config file which was
    // passed to the soss::Instance.
    PathSet config_file_prefix;

    // If prefixes for this middleware were given from the command line, this
    // will return them. Otherwise it will return an empty PathSet.
    const PathSet& get_cli_middleware_prefixes(
        const std::string& middleware) const;

    PathSet& get_cli_middleware_prefixes(
        const std::string& middleware);
  };

  // This is the static copy of the global paths, shared by all instances of
  // Search.
  static GlobalPaths static_default_global_paths;

  // This is a local copy of the global paths, which can be mutated by each
  // Search instance.
  GlobalPaths _global_paths;

  // SOSS prefixes derived from environment variables
  PathSet _env_soss_prefixes;

  // Middleware prefixes derived from environment variables
  PathSet _env_middleware_prefixes;

  // High-priority middleware prefixes set by the user of the Search object
  PathSet _priority_middleware_prefixes;

  // Fallback middleware prefixes set by the user of the Search object
  PathSet _fallback_middleware_prefixes;

  // Home directory prefix
  PathSet _home_prefix;

};

} // namespace soss

#endif // SOSS__INTERNAL__SEARCH_IMPL_HPP
