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

#include <algorithm>
#include <cctype>
#include <experimental/filesystem>
#include <iostream>

#include <cstdlib>

#ifdef WIN32
#define ENV_LIB_PATH "CMAKE_PREFIX_PATH"
#define LIB_PATH "lib/soss"
#define IS_NOT_ABSOLUTE_PATH(path) path.find("C:\\") == std::string::npos && path.find("c:\\") == std::string::npos
#else
#define ENV_LIB_PATH "LD_LIBRARY_PATH"
#define LIB_PATH "soss"
#define IS_NOT_ABSOLUTE_PATH(path) path.at(0) != '/'
#endif

namespace soss {

const std::string HomeEnvVar = "HOME";

namespace filesystem = std::experimental::filesystem;

//==============================================================================
// initialize static members
Search::Implementation::GlobalPaths
    Search::Implementation::static_default_global_paths;

Search::Implementation::GlobalPathInitializer
    Search::Implementation::static_global_path_initializer;

//==============================================================================
std::string to_env_format(std::string str)
{
  std::transform(str.begin(), str.end(), str.begin(),
                 [](unsigned char c){ return std::toupper(c); });

  for(char& c : str)
  {
    if(c == '-')
      c = '_';
  }

  return str;
}

//==============================================================================
std::vector<std::string> get_environment_variable_path_list(
    const std::string& env_var)
{
  std::vector<std::string> result;

  const char* var_value = getenv(env_var.c_str());
  if(!var_value)
    return {};

  const std::string var_str(var_value);

  std::string::const_iterator begin = var_str.begin();
  std::string::const_iterator end = var_str.end();
  for(std::string::const_iterator it = begin; it != end; ++it)
  {
#if defined(WIN32)
    if(*it == ';')
#else
    if(*it == ':')
#endif
    {
      result.emplace_back(begin, it);
      begin = it+1;
    }
  }

  if(begin != end)
    result.emplace_back(begin, end);

  return result;
}

//==============================================================================
void Search::Implementation::add_cli_soss_prefix(const std::string& path)
{
  static_default_global_paths.cli_soss_prefixes.add_path(path);
}

//==============================================================================
void Search::Implementation::add_cli_middleware_prefix(
    const std::string& middleware,
    const std::string& path)
{
  static_default_global_paths.cli_middleware_prefixes[middleware].add_path(path);
}

//==============================================================================
void Search::Implementation::set_config_file_directory(const std::string& path)
{
  static_default_global_paths.config_file_prefix.add_path(path);
}

//==============================================================================
Search::Implementation::Implementation(const std::string& middleware)
  : _middleware(middleware)
{
  _global_paths = static_default_global_paths;
  // Environment variables could change between different instantiations of the
  // Search object, so we re-check these each time a Search instance is created.

  // Add external libraries values first because they are the lowest priority
  const std::vector<std::string> ld_library_paths =
      get_environment_variable_path_list(ENV_LIB_PATH);

  for(auto it = ld_library_paths.rbegin(); it != ld_library_paths.rend(); ++it)
    _env_soss_prefixes.add_path(*it);

  // The explicitly set SOSS_PREFIX_PATH has a higher priority than
  // LD_LIBRARY_PATH so we add that next.
  const std::vector<std::string> soss_prefix_path =
      get_environment_variable_path_list("SOSS_PREFIX_PATH");
  for(auto it = soss_prefix_path.rbegin(); it != soss_prefix_path.rend(); ++it)
    _env_soss_prefixes.add_path(*it);

  // Now we get the paths specific to this middleware and add it to the
  // environment middleware paths.
  const std::vector<std::string> mw_prefix_paths =
      get_environment_variable_path_list(
        "SOSS_"+to_env_format(middleware)+"_PREFIX_PATH");

  for(auto it = mw_prefix_paths.rbegin(); it != mw_prefix_paths.rend(); ++it)
    _env_middleware_prefixes.add_path(*it);

  // Add the home path based on the $HOME environment variable, but set it to
  // inactive by default.
  _home_prefix.active = false;
  const char* home_path = getenv(HomeEnvVar.c_str());
  if(home_path)
    _home_prefix.add_path(home_path);
}

//==============================================================================
void Search::Implementation::add_priority_middleware_prefix(
    const std::string& path)
{
  _priority_middleware_prefixes.add_path(path);
}

//==============================================================================
void Search::Implementation::add_fallback_middleware_prefix(
    const std::string& path)
{
  _fallback_middleware_prefixes.add_path(path);
}

//==============================================================================
std::string Search::Implementation::find_file(
    const std::string filename,
    const std::string& subdir,
    std::vector<std::string>& checked_paths) const
{
  if(filename.at(0) == '/')
    return filename;

  auto check_and_append = [&](
      const filesystem::path test) -> bool
  {
    checked_paths.push_back(test.string());
    return filesystem::exists(test);
  };

  for(const PathSet& middleware_prefixes :
        {
          _priority_middleware_prefixes,
          _global_paths.get_cli_middleware_prefixes(_middleware),
          _env_middleware_prefixes,
          _global_paths.config_file_prefix,
          _home_prefix,
          _fallback_middleware_prefixes
        })
  {
    if(!middleware_prefixes.active)
      continue;

    for(const std::string& prefix_str : middleware_prefixes)
    {
      const filesystem::path prefix(prefix_str);

      if(!subdir.empty())
      {
        if(check_and_append(prefix/subdir/filename))
          return checked_paths.back();
      }

      {
        if(check_and_append(prefix/filename))
          return checked_paths.back();
      }
    }
  }

  for(const PathSet& soss_prefixes :
        {
          _global_paths.cli_soss_prefixes,
          _env_soss_prefixes,
          _global_paths.system_soss_prefixes
        })
  {
    if(!soss_prefixes.active)
      continue;

    for(const std::string& soss_prefix_str : soss_prefixes)
    {
      const filesystem::path soss_prefix(soss_prefix_str);

      if(!subdir.empty())
      {
        if(check_and_append(soss_prefix/_middleware/subdir/filename))
           return checked_paths.back();
      }

      {
        if(check_and_append(soss_prefix/_middleware/filename))
          return checked_paths.back();
      }

      if(!subdir.empty())
      {
        if(check_and_append(soss_prefix/LIB_PATH/_middleware/subdir/filename))
          return checked_paths.back();
      }

      {
        if(check_and_append(soss_prefix/LIB_PATH/_middleware/filename))
          return checked_paths.back();
      }
    }
  }

  return "";
}

//==============================================================================
std::string Search::Implementation::find_middleware_mix(
    std::vector<std::string>& checked_paths) const
{
  const std::string filename = _middleware + ".mix";

  auto check_and_append = [&](
      const filesystem::path test) -> bool
  {
    checked_paths.push_back(test.string());
    return filesystem::exists(test);
  };

  for(const PathSet& middleware_prefixes :
        {
          _priority_middleware_prefixes,
          _global_paths.get_cli_middleware_prefixes(_middleware)
        })
  {
    if(!middleware_prefixes.active)
      continue;

    for(const std::string& prefix_str : middleware_prefixes)
    {
      const filesystem::path prefix(prefix_str);
      if(check_and_append(prefix/filename))
        return checked_paths.back();
    }
  }

  for(const PathSet& soss_prefixes:
        {
          _global_paths.cli_soss_prefixes,
          _env_soss_prefixes,
          _global_paths.system_soss_prefixes
        })
  {
    if(!soss_prefixes.active)
      continue;

    for(const std::string& soss_prefix_str : soss_prefixes)
    {
      const filesystem::path soss_prefix(soss_prefix_str);

      {
        if(check_and_append(soss_prefix/filename))
          return checked_paths.back();
      }

      {
        if(check_and_append(soss_prefix/LIB_PATH/filename))
           return checked_paths.back();
      }

      {
        if(check_and_append(soss_prefix/LIB_PATH/_middleware/filename))
          return checked_paths.back();
      }
    }
  }

  return "";
}

//==============================================================================
void Search::Implementation::search_relative_to_config(bool toggle)
{
  _global_paths.config_file_prefix.active = toggle;
}

//==============================================================================
void Search::Implementation::search_relative_to_home(bool toggle)
{
  _home_prefix.active = toggle;
}

//==============================================================================
void Search::Implementation::search_system_prefixes(bool toggle)
{
  _global_paths.system_soss_prefixes.active = toggle;
}

//==============================================================================
void Search::Implementation::search_soss_prefixes(bool toggle)
{
  _global_paths.system_soss_prefixes.active = toggle;
  _global_paths.cli_soss_prefixes.active = toggle;
  _env_soss_prefixes.active = toggle;
}

//==============================================================================
void Search::Implementation::search_middleware_prefixes(bool toggle)
{
  _global_paths.get_cli_middleware_prefixes(_middleware).active = toggle;
}

//==============================================================================
void Search::Implementation::PathSet::add_path(const std::string& path)
{
  if(path.empty())
    return;

  if(IS_NOT_ABSOLUTE_PATH(path))
  {
    std::cerr << "[soss error] Attempting to add a prefix path that is not an "
              << "absolute path: " << path << std::endl;
    return;
  }

  // See if this path has already been added to the list
  auto insertion = _added_paths.insert({path, {}});

  // If it has been added, remove the previous entry
  if(!insertion.second)
    _path_list.erase(insertion.first->second);

  // Add the path to the front of the list
  _path_list.push_front(path);

  // Save its iterator in the map of added paths
  insertion.first->second = _path_list.begin();

  // The above procedure makes it so that if a path gets added multiple times,
  // it will be given the priority of the most recent addition, and it will not
  // get iterated over multiple times.
}

//==============================================================================
std::list<std::string>::const_iterator
Search::Implementation::PathSet::begin() const
{
  return _path_list.begin();
}

//==============================================================================
std::list<std::string>::const_iterator
Search::Implementation::PathSet::end() const
{
  return _path_list.end();
}

//==============================================================================
Search::Implementation::GlobalPathInitializer::GlobalPathInitializer()
{
  for(const std::string& path :
    {
      "/usr/lib",
      "/usr/lib/" SOSS_LIBRARY_ARCHITECTURE,
      "/usr/local/lib",
      "/usr/local/lib/" SOSS_LIBRARY_ARCHITECTURE
    })
  {
    Search::Implementation::static_default_global_paths
        .system_soss_prefixes.add_path(path);
  }

  Search::Implementation::static_default_global_paths
      .config_file_prefix.active = false;
}

//==============================================================================
auto Search::Implementation::GlobalPaths::get_cli_middleware_prefixes(
    const std::string& middleware) const -> const PathSet&
{
  return const_cast<Search::Implementation::GlobalPaths&>(*this)
      .get_cli_middleware_prefixes(middleware);
}

//==============================================================================
auto Search::Implementation::GlobalPaths::get_cli_middleware_prefixes(
    const std::string& middleware) -> PathSet&
{
  const auto& it = cli_middleware_prefixes.find(middleware);
  if(it != cli_middleware_prefixes.end())
    return it->second;

  static PathSet emptyPathSet;
  return emptyPathSet;
}

//==============================================================================
Search::Search(const std::string& middleware_name)
  : _pimpl(new Implementation(middleware_name))
{
  // Do nothing
}

//==============================================================================
Search::Search(const Search& other)
  : _pimpl(new Implementation(*other._pimpl))
{
  // Do nothing
}

//==============================================================================
Search::Search(Search&& other)
  : _pimpl(new Implementation(std::move(*other._pimpl)))
{
  // Do nothing
}

//==============================================================================
Search& Search::operator=(const Search& other)
{
  *_pimpl = *other._pimpl;
  return *this;
}

//==============================================================================
Search& Search::operator=(Search&& other)
{
  *_pimpl = std::move(*other._pimpl);
  return *this;
}

//==============================================================================
void Search::add_priority_middleware_prefix(const std::string& path)
{
  _pimpl->add_priority_middleware_prefix(path);
}

//==============================================================================
void Search::add_fallback_middleware_prefix(const std::string& path)
{
  _pimpl->add_fallback_middleware_prefix(path);
}

//==============================================================================
std::string Search::find_message_mix(
    const std::string& msg_type,
    std::vector<std::string>* checked_paths) const
{
  return find_generic_mix(msg_type, "msg", checked_paths);
}

//==============================================================================
std::string Search::find_service_mix(
    const std::string& srv_type,
    std::vector<std::string>* checked_paths) const
{
  return find_generic_mix(srv_type, "srv", checked_paths);
}

//==============================================================================
std::string Search::find_generic_mix(
    const std::string& type,
    const std::string& subdir,
    std::vector<std::string>* checked_paths) const
{
  std::vector<std::string> _checked_paths;
  const auto result = _pimpl->find_file(type + ".mix", subdir, _checked_paths);
  if(checked_paths)
    *checked_paths = _checked_paths;

  return result;
}

//==============================================================================
std::string Search::find_file(
    std::string filename,
    const std::string& subdir,
    std::vector<std::string>* checked_paths) const
{
  std::vector<std::string> _checked_paths;
  const auto result = _pimpl->find_file(
        std::move(filename), subdir, _checked_paths);

  if(checked_paths)
    *checked_paths = _checked_paths;

  return result;
}

//==============================================================================
Search& Search::relative_to_config(bool toggle)
{
  _pimpl->search_relative_to_config(toggle);
  return *this;
}

//==============================================================================
Search& Search::relative_to_home(bool toggle)
{
  _pimpl->search_relative_to_home(toggle);
  return *this;
}

//==============================================================================
Search& Search::ignore_system_prefixes(bool toggle)
{
  _pimpl->search_system_prefixes(!toggle);
  return *this;
}

//==============================================================================
Search& Search::ignore_soss_prefixes(bool toggle)
{
  _pimpl->search_soss_prefixes(!toggle);
  return *this;
}

//==============================================================================
Search& Search::ignore_middleware_prefixes(bool toggle)
{
  _pimpl->search_middleware_prefixes(!toggle);
  return *this;
}

//==============================================================================
Search::~Search()
{
  // Do nothing
}

} // namespace soss
