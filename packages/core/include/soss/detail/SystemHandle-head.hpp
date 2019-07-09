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

#ifndef SOSS__DETAIL__SYSTEMHANDLE_HEAD_HPP
#define SOSS__DETAIL__SYSTEMHANDLE_HEAD_HPP

#include "soss/soss_export.hpp"
#include <functional>
#include <memory>
#include <type_traits>

namespace soss {

// Forward declaration
class SystemHandle;

namespace detail {

//==============================================================================
using SystemHandleFactory = std::function<std::unique_ptr<SystemHandle>()>;

//==============================================================================
void SOSS_EXPORT register_system_handle_factory(
    const std::string& middleware,
    SystemHandleFactory handle);

//==============================================================================
template<typename S>
class SystemHandleRegistrar
{
public:

  SystemHandleRegistrar(const std::string& middleware)
  {
    register_system_handle_factory(
          middleware, [](){ return std::make_unique<S>(); });
  }

};

} // namespace detail
} // namespace soss

#define DETAIL_SOSS_REGISTER_SYSTEM_HELPER( \
      UniqueID, middleware_name, HandleType) \
  namespace { \
    ::soss::detail::SystemHandleRegistrar<HandleType> \
    execute_at_load_ ## UniqueID (middleware_name); \
  } /* anonymous namespace */


// Because of C macro expansion rules, we need this middle step in order for
// __COUNTER__/UniqueID to be expanded to an integer value correctly
#define DETAIL_SOSS_REGISTER_SYSTEM_WITH_COUNTER( \
      UniqueID, middleware_name, HandleType) \
  DETAIL_SOSS_REGISTER_SYSTEM_HELPER(UniqueID, middleware_name, HandleType)



#define DETAIL_SOSS_REGISTER_SYSTEM(middleware_name, HandleType) \
  DETAIL_SOSS_REGISTER_SYSTEM_WITH_COUNTER( \
    __COUNTER__, middleware_name, HandleType)

#endif // SOSS__DETAIL__SYSTEMHANDLE_HEAD_HPP
