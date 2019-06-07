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

#include "register_system.hpp"

#include <iostream>

namespace soss {
namespace internal {

//==============================================================================
// initialize static members
Register::FactoryMap Register::_info_map;
std::mutex Register::_mutex;

//==============================================================================
void Register::insert(
    std::string middleware,
    detail::SystemHandleFactory handle_factory)
{
  FactoryMap::value_type entry(
        std::move(middleware), std::move(handle_factory));

  std::unique_lock<std::mutex> lock(_mutex);
  _info_map.insert(std::move(entry));
}

//==============================================================================
SystemHandleInfo Register::get(const std::string& middleware)
{
  const FactoryMap::const_iterator it = _info_map.find(middleware);
  if(it == _info_map.end())
  {
    std::cerr << "Could not find system handle for middleware of type ["
              << middleware << "]" << std::endl;
    return SystemHandleInfo(nullptr);
  }

  return SystemHandleInfo(_info_map.at(middleware)());
}

} // namespace internal

namespace detail {

//==============================================================================
void register_system_handle_factory(
    const std::string& middleware,
    std::function<std::unique_ptr<SystemHandle>()> handle_factory)
{
  internal::Register::insert(middleware, std::move(handle_factory));
}

} // namespace detail

} // namespace soss
