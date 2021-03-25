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

#ifndef SOSS__WEBOSCKET__SRC__WEBSOCKET_TYPES_HPP
#define SOSS__WEBOSCKET__SRC__WEBSOCKET_TYPES_HPP

#include <websocketpp/config/asio.hpp>
#include <websocketpp/server.hpp>
#include <websocketpp/client.hpp>

namespace soss {
namespace websocket {

using TlsConfig = websocketpp::config::asio_tls;
using Connection = websocketpp::connection<TlsConfig>;

using WsCppServer = websocketpp::server<TlsConfig>;
using WsCppClient = websocketpp::client<TlsConfig>;

using WsCppEndpoint = websocketpp::endpoint<Connection, TlsConfig>;
using WsCppEndpointPtr = std::unique_ptr<WsCppEndpoint>;

using WsCppWeakConnectPtr = websocketpp::connection_hdl;
using WsCppMessagePtr = WsCppEndpoint::message_ptr;

using WsCppConnectionPtr = WsCppEndpoint::connection_ptr;

using WsCppSslContext = boost::asio::ssl::context;
using WsCppSslContextPtr = std::shared_ptr<WsCppSslContext>;

} // namespace websocket
} // namespace soss

#endif // SOSS__WEBOSCKET__SRC__WEBSOCKET_TYPES_HPP
