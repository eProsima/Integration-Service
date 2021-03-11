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

#include <is/core/systemhandle/RegisterSystem.hpp>

#include <iostream>

// namespace eprosima {
namespace is {
namespace core {
namespace internal {

//==============================================================================
SystemHandleInfo::SystemHandleInfo(
        std::unique_ptr<SystemHandle> input)
    : handle(std::move(input))
    , topic_publisher(dynamic_cast<TopicPublisherSystem*>(handle.get()))
    , topic_subscriber(dynamic_cast<TopicSubscriberSystem*>(handle.get()))
    , service_client(dynamic_cast<ServiceClientSystem*>(handle.get()))
    , service_provider(dynamic_cast<ServiceProviderSystem*>(handle.get()))
{
}

//==============================================================================
SystemHandleInfo::SystemHandleInfo(
        SystemHandleInfo&& other)
    : handle(std::move(other.handle))
    , topic_publisher(std::move(other.topic_publisher))
    , topic_subscriber(std::move(other.topic_subscriber))
    , service_client(std::move(other.service_client))
    , service_provider(std::move(other.service_provider))
    , types(std::move(other.types))
{
}

//==============================================================================
inline SystemHandleInfo::operator bool () const
{
    return static_cast<bool>(handle);
}

/**
 * Initialize Register static members.
 */
Register::FactoryMap Register::_info_map;
std::mutex Register::_mutex;

//==============================================================================
void Register::insert(
        std::string&& middleware,
        detail::SystemHandleFactoryBuilder&& handle_factory)
{
    utils::Logger logger("is::core::systemhandle::RegisterSystem");

    FactoryMap::value_type entry(
        std::move(middleware), std::move(handle_factory));

    std::unique_lock<std::mutex> lock(_mutex);

    auto res = _info_map.insert(std::move(entry));

    if (res.second)
    {
        logger << utils::Logger::Level::DEBUG
               << "A new middleware SystemHandle was added to the SystemHandle "
               << "factory map: '" << middleware << "'." << std::endl;
    }
    else
    {
        logger << utils::Logger::Level::WARN
               << "Some error occured while trying to add SystemHandle "
               << "for middleware '" << middleware << "' into the "
               << "SystemHandle factory map." << std::endl;
    }
}

//==============================================================================
SystemHandleInfo Register::get(
        const std::string& middleware)
{
    utils::Logger logger("is::core::systemhandle::RegisterSystem");

    const FactoryMap::const_iterator it_mw = _info_map.find(middleware);

    if (it_mw == _info_map.end())
    {
        logger << utils::Logger::Level::ERROR
               << "Could not find SystemHandle library for middleware '"
               << middleware << "' within the SystemHandle factory registry."
               << std::endl;

        return SystemHandleInfo(nullptr);
    }
    else
    {
        logger << utils::Logger::Level::DEBUG
               << "Found SystemHandle library for middleware '" << middleware
               << "' within the SystemHandle factory registry."
               << std::endl;
    }

    return SystemHandleInfo(_info_map.at(middleware)());
}

} //  namespace internal

namespace detail {

//==============================================================================
void register_system_handle_factory(
        const std::string& middleware,
        SystemHandleFactoryBuilder&& handle_factory)
{
    internal::Register::insert(middleware, std::move(handle_factory));
}

} //  namespace detail
} //  namespace core
} //  namespace is
// } //  namespace eprosima
