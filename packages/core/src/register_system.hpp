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

#ifndef SOSS__INTERNAL__REGISTER_SYSTEM_HPP
#define SOSS__INTERNAL__REGISTER_SYSTEM_HPP

#include <soss/SystemHandle.hpp>

#include <mutex>

namespace soss {
namespace internal {

//==============================================================================
struct SystemHandleInfo
{
  SystemHandleInfo(std::unique_ptr<SystemHandle> input)
    : handle(std::move(input)),
      topic_publisher(dynamic_cast<TopicPublisherSystem*>(handle.get())),
      topic_subscriber(dynamic_cast<TopicSubscriberSystem*>(handle.get())),
      service_client(dynamic_cast<ServiceClientSystem*>(handle.get())),
      service_provider(dynamic_cast<ServiceProviderSystem*>(handle.get()))
  {
    // Do nothing
  }

  SystemHandleInfo(SystemHandleInfo&& other)
    : handle(std::move(other.handle)),
      topic_publisher(std::move(other.topic_publisher)),
      topic_subscriber(std::move(other.topic_subscriber)),
      service_client(std::move(other.service_client)),
      service_provider(std::move(other.service_provider)),
      types(std::move(other.types))
  {
    // Do nothing
  }

  inline operator bool() const
  {
    return static_cast<bool>(handle);
  }

  std::unique_ptr<SystemHandle> handle;
  TopicPublisherSystem*         topic_publisher;
  TopicSubscriberSystem*        topic_subscriber;
  ServiceClientSystem*          service_client;
  ServiceProviderSystem*        service_provider;

  TypeRegistry types;
};

//==============================================================================
using SystemHandleInfoMap = std::map<std::string, SystemHandleInfo>;

//==============================================================================
class Register
{
public:

  static void insert(
      std::string middleware,
      detail::SystemHandleFactory handle);

  static SystemHandleInfo get(const std::string& middleware);

private:
  using FactoryMap = std::map<std::string, detail::SystemHandleFactory>;

  static FactoryMap _info_map;
  static std::mutex _mutex;
};

} // namespace internal
} // namespace soss

#endif // SOSS__INTERNAL__REGISTER_SYSTEM_HPP
