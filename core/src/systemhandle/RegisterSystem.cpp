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

#include <is/systemhandle/RegisterSystem.hpp>

#include <iostream>

namespace eprosima {
namespace is {
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
SystemHandleInfo::operator bool () const
{
    return static_cast<bool>(handle);
}

/**
 * Initializes Register of static members.
 */
Register::FactoryMap Register::_info_map;
std::map<std::string, std::string> Register::_alias_map;
std::mutex Register::_mutex;

//==============================================================================
void Register::insert(
        std::string&& middleware_id,
        std::vector<std::string>&& middleware_aliases,
        detail::SystemHandleFactoryBuilder&& handle_factory)
{
    utils::Logger logger("is::core::systemhandle::RegisterSystem");

    FactoryMap::value_type entry(
        middleware_id, std::move(handle_factory));

    std::unique_lock<std::mutex> lock(_mutex);

    // Store in map the SystemHandle associated with the main alias
    auto res = _info_map.insert(std::move(entry));

    if (res.second)
    {
        logger << utils::Logger::Level::DEBUG
               << "A new middleware SystemHandle was added to the SystemHandle "
               << "factory map: '" << res.first->first << "'." << std::endl;
    }
    else
    {
        logger << utils::Logger::Level::WARN
               << "Some error occured while trying to add SystemHandle "
               << "for middleware '" << res.first->first << "' into the "
               << "SystemHandle factory map." << std::endl;
    }

    // Always add middleware id as an alias (under same mutes as _info_map)
    _alias_map.insert(std::make_pair(middleware_id, middleware_id));

    // For each alias add it to alias map so they connect to their middleware id
    // It does not check if the alias has already been set
    for (auto alias : middleware_aliases)
    {
        _alias_map.insert(std::make_pair(alias, middleware_id));
    }
}

//==============================================================================
SystemHandleInfo Register::get(
        const std::string& middleware_alias)
{
    utils::Logger logger("is::core::systemhandle::RegisterSystem");

    std::string middleware_id;
    auto it_alias = _alias_map.find(middleware_alias);

    if (it_alias == _alias_map.end())
    {
        logger << utils::Logger::Level::ERROR
               << "Middleware alias '"
               << middleware_alias << "' does not reference any known SystemHandle."
               << std::endl;

        return SystemHandleInfo(nullptr);
    }
    else
    {
        middleware_id = it_alias->second;
        logger << utils::Logger::Level::DEBUG
               << "Alias '" << middleware_alias << "' referencing middleware: '" << middleware_id << "'"
               << std::endl;
    }

    const FactoryMap::const_iterator it_mw = _info_map.find(middleware_alias);

    if (it_mw == _info_map.end())
    {
        logger << utils::Logger::Level::ERROR
               << "Could not find SystemHandle library for middleware '"
               << middleware_alias << "' within the SystemHandle factory registry."
               << std::endl;

        return SystemHandleInfo(nullptr);
    }
    else
    {
        logger << utils::Logger::Level::DEBUG
               << "Found SystemHandle library for middleware '" << middleware_alias
               << "' within the SystemHandle factory registry."
               << std::endl;
    }

    return SystemHandleInfo(_info_map.at(middleware_alias)());
}

} //  namespace internal

namespace detail {

//==============================================================================
void register_system_handle_factory(
        std::string&& middleware_id,
        std::vector<std::string>&& middleware_aliases,
        SystemHandleFactoryBuilder&& handle_factory)
{
    internal::Register::insert(std::move(middleware_id), std::move(middleware_aliases), std::move(handle_factory));
}

} //  namespace detail
} //  namespace is
} //  namespace eprosima
