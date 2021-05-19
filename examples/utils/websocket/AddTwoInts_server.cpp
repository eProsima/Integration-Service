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

#include "AddTwoInts_server.hpp"

using namespace std::chrono_literals;

AddTwoIntsServer::AddTwoIntsServer(
        const std::string& service_name,
        uint16_t port)
    : _service_name(service_name)
    , _port(port)
    , _mutex()
{
    std::cout << "Creating TCP '" << _service_name << "' WebSocket server "
              << "on port: " << _port << std::endl;
    configure_tcp_server();
}

void AddTwoIntsServer::configure_tcp_server()
{
    _context = std::make_shared<SslContext>(SslContext::tls);
    _context->set_options(
        SslContext::default_workarounds |
        SslContext::no_sslv2 |
        SslContext::no_sslv3);

    // Initialize server and define its callbacks
    _tcp_server = std::make_shared<TcpServer>(TcpServer());
    _tcp_server->set_reuse_addr(true);
    _tcp_server->set_access_channels(websocketpp::log::alevel::all);
    _tcp_server->clear_access_channels(
        websocketpp::log::alevel::frame_header |
        websocketpp::log::alevel::frame_payload);
    _tcp_server->init_asio();
    _tcp_server->set_message_handler(
        [&](websocketpp::connection_hdl hdl,
        TcpServer::message_ptr msg)
        {
            on_tcp_message(hdl, msg);
        });
    _tcp_server->set_tcp_init_handler(
        [&](websocketpp::connection_hdl hdl) ->  std::shared_ptr<SslContext>
        {
            return _context;
        });
    _tcp_server->set_open_handler(
        [&](
            websocketpp::connection_hdl handle)
        {
            // When the connection is established, send the advertise_service message so that
            // the client knows that this server manages 'client_request' requests
            std::ostringstream advertise_msg;
            advertise_msg << "{\"op\":\"advertise_service\","
                          << "\"request_type\":\"AddTwoInts_Request\","
                          << "\"reply_type\":\"AddTwoInts_Response\","
                          << "\"service\":\"" << _service_name << "\"}";

            TcpEndpoint::connection_ptr connection = _tcp_server->get_con_from_hdl(handle);
            _mutex.lock();
            _open_tcp_connections.insert(connection);
            connection->send(advertise_msg.str());
            _mutex.unlock();
        });
    _tcp_server->set_close_handler(
        [&](websocketpp::connection_hdl handle)
        {
            _mutex.lock();
            const auto connection = _tcp_server->get_con_from_hdl(handle);
            _open_tcp_connections.erase(connection);
            _mutex.unlock();
        });

    _tcp_server->listen(_port);
    _tcp_server->start_accept();
    _tcp_server->run();
}

void AddTwoIntsServer::on_tcp_message(
        websocketpp::connection_hdl hdl,
        TcpServer::message_ptr msg)
{
    const auto json_request_msg = nlohmann::json::parse(msg->get_payload());

    const auto args_it = json_request_msg.find("args");
    const auto id_it = json_request_msg.find("id");

    // Check request valid format
    if (json_request_msg.end() != args_it && json_request_msg.end() != id_it)
    {
        auto json_request_payload = args_it.value();

        if (json_request_payload.end() != json_request_payload.find("a") &&
                json_request_payload.end() != json_request_payload.find("b"))
        {
            std::cout << "WebSocket '" << _service_name << "' Server:" << std::endl;
            std::cout << "\t - Request received: [ a: "
                      << json_request_payload["a"].get<int64_t>() << ", b: "
                      << json_request_payload["b"].get<int64_t>() << " ]" << std::endl;

            int result = json_request_payload["a"].get<int64_t>() + json_request_payload["b"].get<int64_t>();

            // Send the response
            auto connection = _tcp_server->get_con_from_hdl(hdl);

            const std::string response_msg =
                    "{\"op\":\"service_response\", \"result\":\"true\", \"id\": "
                    + id_it.value().dump() + ", \"service\": \""
                    + _service_name + "\", \"values\":{\"sum\":"
                    + std::to_string(result) + "}}";

            std::cout << "\t - Sending response: '" << response_msg << "'" << std::endl;

            connection->send(response_msg);
        }
        else
        {
            std::cerr << "WebSocket '" << _service_name << "' Server received invalid "
                      << "input arguments: 'a' and 'b' expected, received [ "
                      << json_request_payload.dump() << " ]" << std::endl;
        }
    }
    else
    {
        std::cerr << "WebSocket '" << _service_name << "' Server received an invalid "
                  << "message format: [ " << json_request_msg.dump()  << " ]" << std::endl;
    }
}

AddTwoIntsServer::~AddTwoIntsServer()
{
    _mutex.lock();
    // Close all the connections before stopping the server
    for (auto connection : _open_tcp_connections)
    {
        if (connection->get_state() != websocketpp::session::state::closed)
        {
            try
            {
                connection->close(websocketpp::close::status::normal, "shutdown");
            }
            catch (websocketpp::exception e)
            {
                std::cerr << "Exception occurred while trying to close connection " << connection
                          << ". Connection status: "
                          << ((connection->get_state() == websocketpp::session::state::closed) ? "" : "not ")
                          << "closed" << std::endl;
            }
        }
    }
    _mutex.unlock();
    _tcp_server->stop();
}
