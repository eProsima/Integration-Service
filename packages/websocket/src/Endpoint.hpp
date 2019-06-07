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

#ifndef SOSS__WEBSOCKET__SRC__ENDPOINT_HPP
#define SOSS__WEBSOCKET__SRC__ENDPOINT_HPP

#include "Encoding.hpp"
#include "websocket_types.hpp"

#include <soss/SystemHandle.hpp>

#include <memory>
#include <unordered_map>
#include <unordered_set>

namespace soss {
namespace websocket {

const std::string YamlEncodingKey = "encoding";
const std::string YamlEncoding_Rosbridge_v2_0 = "rosbridge_v2.0";
const std::string YamlPortKey = "port";
const std::string YamlHostKey = "host";

//==============================================================================
class Endpoint : public soss::FullSystem, public ServiceClient
{
public:

  Endpoint();

  bool configure(
      const RequiredTypes& types,
      const YAML::Node& configuration) override;

  virtual bool okay() const = 0;

  virtual bool spin_once() = 0;

  virtual ~Endpoint() = default;


  // ----------- Functions for configuring -----------

  bool subscribe(
      const std::string& topic_name,
      const std::string& message_type,
      TopicSubscriberSystem::SubscriptionCallback callback,
      const YAML::Node& configuration) override final;

  std::shared_ptr<TopicPublisher> advertise(
      const std::string& topic_name,
      const std::string& message_type,
      const YAML::Node& configuration) override final;

  bool create_client_proxy(
      const std::string& service_name,
      const std::string& service_type,
      ServiceClientSystem::RequestCallback callback,
      const YAML::Node& configuration) override final;

  std::shared_ptr<ServiceProvider> create_service_proxy(
      const std::string& service_name,
      const std::string& service_type,
      const YAML::Node& configuration) override final;


  // ----------- Functions for reacting to soss messages -----------

  bool publish(
      const std::string& topic,
      const soss::Message& message);

  void call_service(
      const std::string& service,
      const soss::Message& request,
      ServiceClient& client,
      std::shared_ptr<void> call_handle);


  // ------ Function for catching service responses from soss ------

  void receive_response(
      std::shared_ptr<void> call_handle,
      const soss::Message& response) override final;


  // --------- Functions for reacting to websocket messages --------

  void receive_topic_advertisement_ws(
      const std::string& topic_name,
      const std::string& message_type,
      const std::string& id,
      std::shared_ptr<void> connection_handle);

  void receive_topic_unadvertisement_ws(
      const std::string& topic_name,
      const std::string& id,
      std::shared_ptr<void> connection_handle);

  void receive_publication_ws(
      const std::string& topic_name,
      const soss::Message& message,
      std::shared_ptr<void> connection_handle);

  void receive_subscribe_request_ws(
      const std::string& topic_name,
      const std::string& message_type,
      const std::string& id,
      std::shared_ptr<void> connection_handle);

  void receive_unsubscribe_request_ws(
      const std::string& topic_name,
      const std::string& id,
      std::shared_ptr<void> connection_handle);

  // TODO(MXG): There are some fields in the rosbridge specification that are
  // being ignored here, namely "fragment_size" and "compression"
  void receive_service_request_ws(
      const std::string& service_name,
      const soss::Message& request,
      const std::string& id,
      std::shared_ptr<void> connection_handle);

  void receive_service_advertisement_ws(
      const std::string& service_name,
      const std::string& service_type,
      std::shared_ptr<void> connection_handle);

  void receive_service_unadvertisement_ws(
      const std::string& service_name,
      const std::string& service_type,
      std::shared_ptr<void> connection_handle);

  // TODO(MXG): We are ignoring the "result" field for now
  void receive_service_response_ws(
      const std::string& service_name,
      const soss::Message& response,
      const std::string& id,
      std::shared_ptr<void> connection_handle);

protected:

  const Encoding& get_encoding() const;

  void notify_connection_opened(
      const WsCppConnectionPtr& connection_handle);

  void notify_connection_closed(
      const std::shared_ptr<void>& connection_handle);

private:

  virtual WsCppEndpoint* configure_endpoint(
      const RequiredTypes& types,
      const YAML::Node& configuration) = 0;

  EncodingPtr _encoding;
  WsCppEndpoint* _endpoint;

  struct TopicSubscribeInfo
  {
    std::string type;
    SubscriptionCallback callback;

    // Connections whose publications we will ignore because their message type
    // does not match the one we expect.
    std::unordered_set<std::shared_ptr<void>> blacklist;
  };

  struct TopicPublishInfo
  {
    std::string type;

    using ListenerMap = std::unordered_map<
        std::shared_ptr<void>,
        std::unordered_set<std::string>>;

    // Map from connection handle to id of listeners
    ListenerMap listeners;
  };

  struct ClientProxyInfo
  {
    std::string type;
    RequestCallback callback;
  };

  struct ServiceProviderInfo
  {
    std::string type;
    std::shared_ptr<void> connection_handle;
    YAML::Node configuration;
  };

  struct ServiceRequestInfo
  {
    ServiceClient* client;
    std::shared_ptr<void> call_handle;
  };

  std::vector<std::string> _startup_messages;
  std::unordered_map<std::string, TopicSubscribeInfo> _topic_subscribe_info;
  std::unordered_map<std::string, TopicPublishInfo> _topic_publish_info;
  std::unordered_map<std::string, ClientProxyInfo> _client_proxy_info;
  std::unordered_map<std::string, ServiceProviderInfo> _service_provider_info;
  std::unordered_map<std::string, ServiceRequestInfo> _service_request_info;

  std::size_t _next_service_call_id;

};

using EndpointPtr = std::unique_ptr<Endpoint>;

//==============================================================================
std::shared_ptr<TopicPublisher> make_topic_publisher(
    const std::string& topic, Endpoint& endpoint);

std::shared_ptr<ServiceProvider> make_service_provider(
    const std::string& service, Endpoint& endpoint);

//==============================================================================
int32_t parse_port(const YAML::Node& configuration);

} // namespace websocket
} // namespace soss

#endif // SOSS__WEBSOCKET__SRC__ENDPOINT_HPP
