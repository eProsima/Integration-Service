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

#include <soss/Search.hpp>

#include <chrono>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <websocketpp/config/asio_client.hpp>

namespace soss {
namespace websocket {

const std::string WebsocketMiddlewareName = "websocket";
const std::string YamlClientTokenKey = "token";
const std::string WebsocketUriPrefix = "wss://";
const std::string DefaultHostname = "localhost";

const std::string YamlCertAuthoritiesKey = "cert_authorities";

const std::string YamlAuthKey = "authentication";
const std::string YamlJwtTokenKey = "jwt_secret";

using namespace std::chrono_literals;

//==============================================================================
std::string parse_hostname(const YAML::Node& configuration)
{
  if(const YAML::Node host_node = configuration[YamlHostKey])
    return host_node.as<std::string>(DefaultHostname);

  return DefaultHostname;
}

//==============================================================================
class Client : public Endpoint
{
public:

  Client()
    : _host_uri("<undefined>"),
      _closing_down(false),
      _connection_failed(false)
  {
    // Do nothing
  }

  WsCppEndpoint* configure_endpoint(
      const RequiredTypes& /*types*/,
      const YAML::Node& configuration) override
  {
    const int32_t port = parse_port(configuration);
    if(port < 0)
      return nullptr;

    const std::string hostname = parse_hostname(configuration);
    const YAML::Node auth_node = configuration[YamlAuthKey];
    if (auth_node)
      _load_auth_config(auth_node);

    const std::vector<std::string> extra_ca = [&]()
    {
      std::vector<std::string> extra_ca;
      const YAML::Node cert_authorities_node =
          configuration[YamlCertAuthoritiesKey];
      for(const auto node : cert_authorities_node)
        extra_ca.push_back(node.as<std::string>());

      return extra_ca;
    }();

    if(!configure_client(hostname, static_cast<uint16_t>(port), extra_ca))
      return nullptr;

    return &_client;
  }

  bool configure_client(
      const std::string& hostname,
      const uint16_t port,
      const std::vector<std::string>& extra_certificate_authorities)
  {
    _host_uri = WebsocketUriPrefix + hostname + ":" + std::to_string(port);

    _context = std::make_shared<WsCppSslContext>(
          boost::asio::ssl::context::tlsv1);

    boost::system::error_code ec;
    _context->set_default_verify_paths(ec);
    if(ec)
    {
      std::cerr << "[soss::websocket::Client] Failed to load the default "
                << "certificate authorities: " << ec.message() << std::endl;
      return false;
    }

    if(!extra_certificate_authorities.empty())
    {
      soss::Search ca_search = soss::Search(WebsocketMiddlewareName)
          .relative_to_config()
          .relative_to_home();
      std::vector<std::string> checked_paths;

      for(const std::string& ca_file_name : extra_certificate_authorities)
      {
        checked_paths.clear();

        const std::string ca_file_path =
            ca_search.find_file(ca_file_name, "", &checked_paths);

        if(ca_file_path.empty())
        {
          std::string err = std::string()
              + "[soss::websocket::Client] Could not find the specified "
              + "certificate authority [" + ca_file_name + "]. The following "
              + "paths were checked:\n";

          for(const std::string& checked_path : checked_paths)
            err += " -- " + checked_path + "\n";

          std::cerr << err << std::endl;
          return false;
        }

        _context->load_verify_file(ca_file_path, ec);
        if(ec)
        {
          std::cerr << "[soss::websocket::Client] Failed to load the specified "
                    << "certificate authority: " << ca_file_path << std::endl;
          return false;
        }

        std::cout << "[soss::websocket::Client] Using an extra certificate "
                  << "authority [" << ca_file_path << "]" << std::endl;
      }
    }

    _context->set_verify_mode(boost::asio::ssl::context::verify_peer, ec);
    if(ec)
    {
      std::cerr << "[soss::websocket::Client] Failed to set the verify mode: "
                << ec.message() << std::endl;
      return false;
    }

    _context->set_verify_callback(
          boost::asio::ssl::rfc2818_verification(hostname), ec);
    if(ec)
    {
      std::cerr << "[soss::websocket::Client] Failed to set the verify "
                << "callback: " << ec.message() << std::endl;
      return false;
    }

    _client.clear_access_channels(
          websocketpp::log::alevel::frame_header |
          websocketpp::log::alevel::frame_payload);

    _client.init_asio();
    _client.start_perpetual();

    _client.set_message_handler(
          [&](WsCppWeakConnectPtr handle, WsCppMessagePtr message)
    {
      this->_handle_message(std::move(handle), std::move(message));
    });

    _client.set_close_handler(
          [&](WsCppWeakConnectPtr handle)
    {
      this->_handle_close(std::move(handle));
    });

    _client.set_open_handler(
          [&](WsCppWeakConnectPtr handle)
    {
      this->_handle_opening(std::move(handle));
    });

    _client.set_fail_handler(
          [&](WsCppWeakConnectPtr handle)
    {
      this->_handle_failed_connection(std::move(handle));
    });

    _client.set_tls_init_handler(
          [&](WsCppWeakConnectPtr /*handle*/) -> WsCppSslContextPtr
    {
      return this->_context;
    });

    _client.set_socket_init_handler(
          [&](WsCppWeakConnectPtr handle, auto& /*sock*/)
    {
      this->_handle_socket_init(std::move(handle));
    });

    _client_thread = std::thread([&](){ this->_client.run(); });

    return true;
  }

  ~Client() override
  {
    _closing_down = true;

    if(_connection && _connection->get_state() == websocketpp::session::state::open)
    {
      _connection->close(websocketpp::close::status::normal, "shutdown");

      // TODO(MXG) Make these timeout parameters something that can be
      // configured by users
      using namespace std::chrono_literals;
      const auto start_time = std::chrono::steady_clock::now();
      while(_connection->get_state() != websocketpp::session::state::closed)
      {
        // Check for an update every 1/5 of a second
        std::this_thread::sleep_for(200ms);

        // Wait no more than 10 seconds total.
        if(std::chrono::steady_clock::now() - start_time > 10s)
        {
          std::cerr << "[soss::websocket::Client] Timed out while waiting for "
                    << "the remote server to acknowledge the connection "
                    << "shutdown request" << std::endl;
          break;
        }
      }
    }

    if(_client_thread.joinable())
    {
      _client.stop();
      _client_thread.join();
    }
  }

  bool okay() const override
  {
    return static_cast<bool>(_connection);
  }

  bool spin_once() override
  {
    const bool attempt_reconnect = (!_connection ||
        _connection->get_state() == websocketpp::session::state::closed)
         && (std::chrono::steady_clock::now() - _last_connection_attempt > 2s);

    if(!_has_spun_once || attempt_reconnect)
    {
      _has_spun_once = true;

      websocketpp::lib::error_code ec;
      _connection = _client.get_connection(_host_uri, ec);
      if(ec)
      {
        std::cerr << "[soss::websocket::Client] Error creating connection "
                  << "handle: " << ec.message() << std::endl;
      }
      else
      {
        _client.connect(_connection);
      }

      _last_connection_attempt = std::chrono::steady_clock::now();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return static_cast<bool>(_connection);
  }

  void runtime_advertisement(
      const std::string& topic,
      const std::string& message_type,
      const std::string& id,
      const YAML::Node& configuration) override
  {
    if(_connection)
    {
      auto msg = get_encoding().encode_advertise_msg(
        topic, message_type, id, configuration);
      _connection->send(msg.data, msg.len, msg.opcode);
    }
  }

private:

  void _handle_message(
      const WsCppWeakConnectPtr& handle,
      const WsCppMessagePtr& message)
  {
    auto incoming_handle = _client.get_con_from_hdl(handle);
    if(incoming_handle != _connection)
    {
      std::cerr << "[soss::websocket::Client::_handle_message] Unexpected "
                << "connection is sending messages: [" << incoming_handle.get()
                << "] vs [" << _connection.get() << "]" << std::endl;
      return;
    }

    get_encoding().interpret_websocket_msg(
          message->get_payload(), *this, _connection);
  }

  void _handle_close(const WsCppWeakConnectPtr& handle)
  {
    auto closing_connection = _client.get_con_from_hdl(handle);

    if(_closing_down)
    {
      std::cout << "[soss::websocket::Client] closing connection to server."
                << std::endl;
    }
    else
    {
      std::cout << "[soss::websocket::Client::_handle_close] The connection to "
                << "the server is closing early. [code "
                << closing_connection->get_remote_close_code() << "] reason: "
                << closing_connection->get_remote_close_reason() << std::endl;
    }

    notify_connection_closed(closing_connection);
  }

  void _handle_opening(const WsCppWeakConnectPtr& handle)
  {
    auto opened_connection = _client.get_con_from_hdl(handle);
    if(opened_connection != _connection)
    {
      std::cerr << "[soss::websocket::Client::_handle_opening] Unexpected "
                << "connection opened: [" << opened_connection.get()
                << "] vs [" << _connection.get() << "]" << std::endl;
      return;
    }

    _connection_failed = false;
    std::cout << "[soss::websocket::Client] Established connection to host ["
              << _host_uri << "]." << std::endl;

    notify_connection_opened(opened_connection);

    if (_jwt_token)
    {
      std::error_code ec;
      opened_connection->add_subprotocol(*_jwt_token, ec);
      if (ec) {
        std::cerr << "[soss::websocket::Client::_handle_opening]: " << ec.message();
      }
    }
  }

  void _handle_failed_connection(const WsCppWeakConnectPtr& /*handle*/)
  {
    if(!_connection_failed)
    {
      // Print this only once for each time a connection fails
      _connection_failed = true;
      std::cout << "[soss::websocket::Client] Failed to establish a connection "
                << "to the host [" << _host_uri << "]. We will periodically "
                << "attempt to reconnect." << std::endl;
    }
  }

  void _handle_socket_init(const WsCppWeakConnectPtr& handle)
  {
    if (_jwt_token)
    {
      auto connection = _client.get_con_from_hdl(handle);
      connection->add_subprotocol(*_jwt_token);
    }
  }

  void _load_auth_config(const YAML::Node& auth_node)
  {
    const YAML::Node token_node = auth_node[YamlClientTokenKey];
    if (token_node)
      _jwt_token = std::make_unique<std::string>(token_node.as<std::string>());
  }

  std::string _host_uri;
  WsCppConnectionPtr _connection;
  WsCppClient _client;
  std::thread _client_thread;
  std::chrono::steady_clock::time_point _last_connection_attempt;
  bool _has_spun_once = false;
  std::atomic_bool _closing_down;
  std::atomic_bool _connection_failed;
  WsCppSslContextPtr _context;
  std::unique_ptr<std::string> _jwt_token;

};

SOSS_REGISTER_SYSTEM("websocket_client", soss::websocket::Client)

} // namespace websocket
} // namespace soss
