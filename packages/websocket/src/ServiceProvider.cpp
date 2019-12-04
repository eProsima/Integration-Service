/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "Endpoint.hpp"

namespace soss {
namespace websocket {

//==============================================================================
class ServiceProvider : public soss::ServiceProvider
{
public:

  ServiceProvider(
      const std::string& service,
      Endpoint& endpoint)
    : _service(service),
      _endpoint(endpoint)
  {
    // Do nothing
  }

  void call_service(
      const xtypes::DynamicData& request,
      ServiceClient& client,
      std::shared_ptr<void> call_handle)
  {
    _endpoint.call_service(_service, request, client, call_handle);
  }

  ~ServiceProvider() override = default;

private:

  const std::string _service;
  Endpoint& _endpoint;

};

//==============================================================================
std::shared_ptr<soss::ServiceProvider> make_service_provider(
    const std::string& service, Endpoint& endpoint)
{
  return std::make_shared<ServiceProvider>(service, endpoint);
}

} // namespace websocket
} // namespace soss
