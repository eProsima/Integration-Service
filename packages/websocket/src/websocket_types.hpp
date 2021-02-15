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
using TcpConfig = websocketpp::config::asio;
using TlsConnection = websocketpp::connection<TlsConfig>;
using TcpConnection = websocketpp::connection<TcpConfig>;

using TlsServer = websocketpp::server<TlsConfig>;
using TcpServer = websocketpp::server<TcpConfig>;
using TlsClient = websocketpp::client<TlsConfig>;
using TcpClient = websocketpp::client<TcpConfig>;

using TlsEndpoint = websocketpp::endpoint<TlsConnection, TlsConfig>;
using TcpEndpoint = websocketpp::endpoint<TcpConnection, TcpConfig>;
//using WsCppEndpointPtr = std::unique_ptr<WsCppEndpoint>;

using ConnectionHandlePtr = websocketpp::connection_hdl;
using TlsMessagePtr = TlsEndpoint::message_ptr;
using TcpMessagePtr = TcpEndpoint::message_ptr;

using TlsConnectionPtr = TlsEndpoint::connection_ptr;
using TcpConnectionPtr = TcpEndpoint::connection_ptr;

using SslContext = boost::asio::ssl::context;
using SslContextPtr = std::shared_ptr<SslContext>;

using ErrorCode = websocketpp::lib::error_code;

} // namespace websocket
} // namespace soss

#endif // SOSS__WEBOSCKET__SRC__WEBSOCKET_TYPES_HPP
