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
#include <iostream>
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
        std::string&& middleware_id,
        std::vector<std::string>&& middleware_aliases,
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
     * @param[in] middleware_id The middleware name to be registered into the factory.
     */
    SystemHandleRegistrar(
            std::string&& middleware_id,
            std::vector<std::string>&& middleware_aliases)
    {
        std::cout << "SystemHandleRegistrar with string " << middleware_id << std::endl;
        register_system_handle_factory(
            std::move(middleware_id),
            std::move(middleware_aliases),
            []()
            {
                return std::make_unique<SystemHandleImplType>();
            });
    }

    /**
     * @brief Constructor.
     *
     * @param[in] middleware_aliases First value the middleware name or main alias followed by the allowed aliases
     *                  that this middleware supports. Be aware that each alias needs a <alias>.mix file
     *                  referencing the middleware library.
     */
    // SystemHandleRegistrar(
    //         std::vector<std::string>&& middleware_aliases)
    // {
    //     std::cout << "SystemHandleRegistrar with array with size " << middleware_aliases.size() << std::endl;

    //     register_system_handle_factory(
    //         std::vector<std::string>(),
    //         []()
    //         {
    //             return std::make_unique<SystemHandleImplType>();
    //         });
    // }
};

} //  namespace detail
} //  namespace is
} //  namespace eprosima

// This macro is needed to call a macro with an array of values. {} are not well intepreted by macros calls.
#define _MACRO_ARRAY_CALL(...) std::initializer_list<std::string>({__VA_ARGS__}).begin()

#define DETAIL_IS_REGISTER_SYSTEM_HELPER( \
        UniqueID, middleware_name, HandleType, middleware_aliases) \
    namespace { \
    eprosima::is::detail::SystemHandleRegistrar<HandleType> \
    execute_at_load_ ## UniqueID (middleware_name, middleware_aliases); \
    } /* anonymous namespace */

/**
 * Because of C macro expansion rules, we need this middle step in order for
 * __COUNTER__/UniqueID to be expanded to an integer value correctly
 */
#define DETAIL_IS_REGISTER_SYSTEM_WITH_COUNTER( \
        UniqueID, middleware_name, HandleType, middleware_aliases) \
    DETAIL_IS_REGISTER_SYSTEM_HELPER(UniqueID, middleware_name, HandleType, middleware_aliases)


#define DETAIL_IS_REGISTER_SYSTEM(middleware_name, HandleType, middleware_aliases) \
    DETAIL_IS_REGISTER_SYSTEM_WITH_COUNTER( \
        __COUNTER__, middleware_name, HandleType, middleware_aliases)

#endif //  _IS_DETAIL_SYSTEMHANDLE_SYSTEMHANDLEFACTORY_HPP_
