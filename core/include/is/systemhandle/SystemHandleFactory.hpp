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

#ifndef _IS_DETAIL_SYSTEMHANDLE_SYSTEMHANDLEFACTORY_HPP_
#define _IS_DETAIL_SYSTEMHANDLE_SYSTEMHANDLEFACTORY_HPP_

#include <is/core/export.hpp>
#include <functional>
#include <memory>
#include <type_traits>

namespace eprosima {
namespace is {

// Forward declaration
class SystemHandle;

namespace detail {

/**
 * @brief Signature of the function that gets triggered when a new SystemHandle instance is created.
 */
using SystemHandleFactoryBuilder = std::function<std::unique_ptr<SystemHandle>()>;

/**
 * @brief Wrapper method for is::internal::Register::insert.
 *
 * @param[in] middleware The middleware's name.
 *
 * @param[in] handle The handle function responsible for creating the
 *            SystemHandle instance.
 */
void IS_CORE_API register_system_handle_factory(
        std::string&& middleware,
        SystemHandleFactoryBuilder&& handle);

/**
 * @class SystemHandleRegistrar
 *         Builder class to help register any SystemHandle kind, during runtime.
 *
 * @tparam SystemHandleImplType The is::SystemHandle overridden
 *         implementation kind, for a certain middleware.
 */
template<typename SystemHandleImplType>
class SystemHandleRegistrar
{
public:

    /**
     * @brief Constructor.
     *
     * @param[in] middleware The middleware name to be registered into the factory.
     */
    SystemHandleRegistrar(
            std::string&& middleware)
    {
        register_system_handle_factory(
            std::move(middleware),
            []()
            {
                return std::make_unique<SystemHandleImplType>();
            });
    }

};

} //  namespace detail
} //  namespace is
} //  namespace eprosima

#define DETAIL_IS_REGISTER_SYSTEM_HELPER( \
        UniqueID, middleware_name, HandleType) \
    namespace { \
    eprosima::is::detail::SystemHandleRegistrar<HandleType> \
    execute_at_load_ ## UniqueID (middleware_name); \
    } /* anonymous namespace */


/**
 * Because of C macro expansion rules, we need this middle step in order for
 * __COUNTER__/UniqueID to be expanded to an integer value correctly
 */
#define DETAIL_IS_REGISTER_SYSTEM_WITH_COUNTER( \
        UniqueID, middleware_name, HandleType) \
    DETAIL_IS_REGISTER_SYSTEM_HELPER(UniqueID, middleware_name, HandleType)


#define DETAIL_IS_REGISTER_SYSTEM(middleware_name, HandleType) \
    DETAIL_IS_REGISTER_SYSTEM_WITH_COUNTER( \
        __COUNTER__, middleware_name, HandleType)

#endif //  _IS_DETAIL_SYSTEMHANDLE_SYSTEMHANDLEFACTORY_HPP_
