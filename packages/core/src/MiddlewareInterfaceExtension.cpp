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

#include <soss/MiddlewareInterfaceExtension.hpp>

#include <cassert>
#include <experimental/filesystem>
#include <iostream>

#ifdef WIN32
// WINDOWS includes
#include <Windows.h>
#include <assert.h>
#else
#include <dlfcn.h>
#endif
#include <assert.h>

#ifdef WIN32
#define OPEN_LIB(libname) LoadLibraryW(libname)
#define GET_LAST_ERROR() GetLastError()
#define LIB_EXTENSION "dll"
#else
#define OPEN_LIB(libname) dlopen(libname, RTLD_NOW)
#define GET_LAST_ERROR() dlerror()
#define LIB_EXTENSION "dl"
#endif


namespace soss {

namespace filesystem = std::experimental::filesystem;

namespace {
//==============================================================================
bool load_if_exists(
    const std::string& path_str,
    const filesystem::path& relative_to)
{
  filesystem::path path(path_str);
  if(path.is_relative())
  {
    path = relative_to / path;
  }

  if(filesystem::exists(path))
  {
    void* handle = OPEN_LIB(path.c_str());

    auto loading_error = GET_LAST_ERROR();
    if(loading_error)
    {
      std::cerr << "Error while loading the library [" << path << "]: "
                << loading_error << std::endl;
      return false;
    }

    if(!handle)
    {
      std::cerr << "Unknown error while loading the library [" << path << "]"
                << std::endl;
      return false;
    }

    return true;
  }

  std::cerr << "Could not load shared library [" << path << "] because it "
            << "could not be found!" << std::endl;

  return false;
}
} // anonymous namespace

//==============================================================================
class Mix::Implementation
{
public:

  Implementation(
      YAML::Node root,
      const std::string& absolute_file_directory_path)
    : _root(std::move(root)),
      _directory(absolute_file_directory_path)
  {
    assert(_directory.is_absolute());
    assert(filesystem::is_directory(_directory));
  }

  bool load() const
  {
    bool result = load_dl();
    // TODO(MXG): If we want to use the .mix files for more purposes, implement
    // it here.
    return result;
  }

  bool load_dl() const
  {
    bool result = true;
    const YAML::Node& dl = _root[LIB_EXTENSION];
    if(dl.IsSequence())
    {
      for(YAML::const_iterator it = dl.begin(); it != dl.end(); ++it)
        result |= load_if_exists(it->as<std::string>(), _directory);
    }
    else if(dl.IsScalar())
    {
      result |= load_if_exists(dl.as<std::string>(), _directory);
    }

    return result;
  }

private:

  YAML::Node _root;
  filesystem::path _directory;

};

//==============================================================================
Mix::MiddlewareInterfaceExtension(
    YAML::Node root,
    const std::string& absolute_file_directory_path)
  : _pimpl(new Implementation(std::move(root), absolute_file_directory_path))
{
  // Do nothing
}

//==============================================================================
Mix::MiddlewareInterfaceExtension(MiddlewareInterfaceExtension&& other)
  : _pimpl(std::move(other._pimpl))
{
  // Do nothing
}

//==============================================================================
Mix Mix::from_file(const std::string& filename)
{
  auto parentpath = filesystem::path(filename).parent_path();
  return Mix(YAML::LoadFile(filename), parentpath.string());
}

//==============================================================================
Mix Mix::from_string(
    const std::string& text,
    const std::string& absolute_file_directory_path)
{
  return Mix(YAML::Load(text), absolute_file_directory_path);
}

//==============================================================================
Mix Mix::from_node(
    YAML::Node node,
    const std::string& absolute_file_directory_path)
{
  return Mix(std::move(node), absolute_file_directory_path);
}

//==============================================================================
bool Mix::load() const
{
  return _pimpl->load();
}

//==============================================================================
Mix::~MiddlewareInterfaceExtension()
{
  // Do nothing
}

} // namespace soss
