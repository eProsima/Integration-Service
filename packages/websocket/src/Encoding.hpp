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

#ifndef SOSS__WEBSOCKET__SRC__ENCODING_HPP
#define SOSS__WEBSOCKET__SRC__ENCODING_HPP

#include <soss/Message.hpp>

#include <yaml-cpp/yaml.h>

#include <memory>

namespace soss {
namespace websocket {

class Endpoint;

//==============================================================================
class Encoding
{
public:

  virtual void interpret_websocket_msg(
      const std::string& msg,
      Endpoint& endpoint,
      std::shared_ptr<void> connection_handle) const = 0;

  virtual std::string encode_publication_msg(
      const std::string& topic_name,
      const std::string& topic_type,
      const std::string& id,
      const soss::Message& msg) const = 0;

  virtual std::string encode_service_response_msg(
      const std::string& service_name,
      const std::string& service_type,
      const std::string& id,
      const soss::Message& response,
      bool result) const = 0;

  virtual std::string encode_subscribe_msg(
      const std::string& topic_name,
      const std::string& message_type,
      const std::string& id,
      const YAML::Node& configuration) const = 0;

  virtual std::string encode_advertise_msg(
      const std::string& topic_name,
      const std::string& message_type,
      const std::string& id,
      const YAML::Node& configuration) const = 0;

  virtual std::string encode_call_service_msg(
      const std::string& service_name,
      const std::string& service_type,
      const soss::Message& service_request,
      const std::string& id,
      const YAML::Node& configuration) const = 0;

  virtual std::string encode_advertise_service_msg(
      const std::string& service_name,
      const std::string& service_type,
      const std::string& id,
      const YAML::Node& configuration) const = 0;
};

using EncodingPtr = std::shared_ptr<Encoding>;

//==============================================================================
EncodingPtr make_rosbridge_v2_0();

} // namespace websocket
} // namespace soss

#endif // SOSS__WEBSOCKET__SRC__ENCODING_HPP
