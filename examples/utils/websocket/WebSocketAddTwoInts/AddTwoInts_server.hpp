/*
 * Copyright (C) 2021 - present Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef _IS_EXAMPLES_UTILS_WEBSOCKET__ADDTWOINTS_SERVER_
#define _IS_EXAMPLES_UTILS_WEBSOCKET__ADDTWOINTS_SERVER_

#include <websocketpp/config/asio.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/server.hpp>

#include "../../../../utils/conversion/json-xtypes/include/is/json-xtypes/json.hpp"

#include <iostream>
#include <unordered_set>
#include <sstream>

using namespace std::chrono_literals;

class AddTwoIntsServer
{
public:

    typedef websocketpp::server<websocketpp::config::asio> TcpServer;
    typedef websocketpp::endpoint<websocketpp::connection<websocketpp::config::asio>,
                    websocketpp::config::asio> TcpEndpoint;
    typedef boost::asio::ssl::context SslContext;

    AddTwoIntsServer(
            const std::string& service_name,
            uint16_t port);

    void configure_tcp_server();

    void on_tcp_message(
            websocketpp::connection_hdl hdl,
            TcpServer::message_ptr msg);

    ~AddTwoIntsServer();

private:

    const std::string _service_name;
    uint16_t _port;

    std::shared_ptr<TcpServer> _tcp_server;
    std::shared_ptr<SslContext> _context;
    std::unordered_set<TcpEndpoint::connection_ptr> _open_tcp_connections;
    std::mutex _mutex;
};

#endif //  _IS_EXAMPLES_UTILS_WEBSOCKET__ADDTWOINTS_SERVER_