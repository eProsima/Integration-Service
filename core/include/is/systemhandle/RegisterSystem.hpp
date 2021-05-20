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

#ifndef _IS_INTERNAL_SYSTEMHANDLE_REGISTER_SYSTEM_HPP_
#define _IS_INTERNAL_SYSTEMHANDLE_REGISTER_SYSTEM_HPP_

#include <is/systemhandle/SystemHandle.hpp>
#include <is/utils/Log.hpp>

#include <mutex>

namespace eprosima {
namespace is {
namespace internal {

/**
 * @class SystemHandleInfo
 * @brief
 *        Storage class that holds all the information relative to a certain
 *        SystemHandle instance.
 *
 *        This class will retrieve the corresponding TopicPublisherSystem,
 *        TopicSubscriberSystem, ServiceClientSystem and ServiceProviderSystem
 *        instances associated to the SystemHandle instance, if applicable.
 *
 *        If not applicable, these instances will just be cast to `nullptr`.
 *        Later on, this will allow to know whether a certain SystemHandle comes
 *        or not with any of these four working capabilities.
 *
 *        Also, a is::TypeRegistry is defined, where all the types that the SystemHandle
 *        instance must know prior to start performing any conversion are defined.
 */
class SystemHandleInfo
{
public:

    /**
     * @brief Constructor.
     *
     * @param[in] input The SystemHandle instance which we want to obtain information from.
     */
    SystemHandleInfo(
            std::unique_ptr<SystemHandle> input);

    /**
     * @brief SystemHandleInfo shall not be copy constructible.
     */
    SystemHandleInfo(
            const SystemHandleInfo& other) = delete;

    /**
     * @brief Move constructor.
     * @param[in] other A movable reference to other SystemHandleInfo instances.
     */
    SystemHandleInfo(
            SystemHandleInfo&& other);

    /**
     * @brief Destructor.
     */
    ~SystemHandleInfo() = default;

    /**
     * @brief `bool()` operator overload.
     *
     * @returns `true` if the pointer to the handle is not `nullptr`, `false` otherwise.
     */
    operator bool() const;

    /**
     * Class members.
     */

    std::unique_ptr<SystemHandle> handle;

    TopicPublisherSystem* topic_publisher;

    TopicSubscriberSystem* topic_subscriber;

    ServiceClientSystem* service_client;

    ServiceProviderSystem* service_provider;

    TypeRegistry types;
};

//==============================================================================
using SystemHandleInfoMap = std::map<std::string, SystemHandleInfo>;


/**
 * @class Register
 *        Static class that contains a static map of is::detail::SystemHandleFactoryBuilder instances.
 *
 *        is::detail::SystemHandleFactoryBuilder is nothing but a function signature that helps
 *        in the creation of a `std::unique_ptr<SystemHandle>` object.
 *
 *        In this way, each time a given SystemHandle instance is required,
 *        it will be created from the factory map.
 */
class Register
{
public:

    /**
     * @brief Inserts a new is::detail::SystemHandleFactoryBuilder element in the factory map.
     *
     * @param[in] middleware The middleware's name.
     *
     * @param[in] handle The handle function responsible for creating the
     *            SystemHandle instance.
     */
    static void insert(
            std::string&& middleware,
            detail::SystemHandleFactoryBuilder&& handle);

    /**
     * @brief Gets the SystemHandleInfo object associated to a given middleware.
     *
     * @param[in] middleware The middleware from which we want
     *            to obtain a SystemHandleInfo instance.
     *
     * @returns A SystemHandleInfo object which is properly initialized if the middleware
     *          exists and it is registered within the Register, or pointing
     *          to `nullptr` otherwise.
     */
    static SystemHandleInfo get(
            const std::string& middleware);

private:

    using FactoryMap = std::map<std::string, detail::SystemHandleFactoryBuilder>;

    /**
     * Class members.
     */

    static FactoryMap _info_map;

    static std::mutex _mutex;
};

} //  namespace internal
} //  namespace is
} //  namespace eprosima

#endif //  _IS_INTERNAL_SYSTEMHANDLE_REGISTER_SYSTEM_HPP_
