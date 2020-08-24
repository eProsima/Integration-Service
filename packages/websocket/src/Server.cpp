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
#include "Errors.hpp"
#include "ServerConfig.hpp"
#include "websocket_types.hpp"
#include "JwtValidator.hpp"
#include "FileManager.hpp"

#include <soss/Search.hpp>
#include <websocketpp/endpoint.hpp>
#include <websocketpp/http/constants.hpp>

namespace soss {
namespace websocket {

const std::string HomeEnvVar = "HOME";
const std::string YamlCertificateKey = "cert";
const std::string YamlPrivateKeyKey = "key";

const std::string YamlFormatKey = "format";
const std::string YamlFormatPemValue = "pem";
const std::string YamlFormatAsn1Value = "asn.1";

const std::string YamlAuthKey = "authentication";

//==============================================================================
static std::string find_websocket_config_file(
  const YAML::Node& configuration,
  const std::string& config_key,
  const std::string& explanation)
{
  const YAML::Node node = configuration[config_key];
  if (!node)
  {
    std::cerr << "[soss::websocket::Server] websocket_server is missing a "
              << "value for the required parameter [" << config_key
              << "] " << explanation << std::endl;
    return {};
  }

  const std::string& parameter = node.as<std::string>();
  const std::string& result = FileManager::find_file(parameter);
  if (result.empty())
  {
    std::string err = std::string()
      + "[soss::websocket::Server] websocket_server failed to find the "
      + "specified file for the [" + config_key + "] parameter: [" +parameter
      + "].";
  }
  else
  {
    std::cout << "[soss::websocket::Server] For [" << config_key << "] using ["
              << result << "]" << std::endl;
  }

  return result;
}

//==============================================================================
static std::string find_certificate(const YAML::Node& configuration)
{
  return find_websocket_config_file(
    configuration, YamlCertificateKey,
    "which should point to a TLS server certificate!");
}

//==============================================================================
static std::string find_private_key(const YAML::Node& configuration)
{
  return find_websocket_config_file(
    configuration, YamlPrivateKeyKey,
    "which should point to the private key for this server!");
}

//==============================================================================
static boost::asio::ssl::context::file_format parse_format(
  const YAML::Node& configuration)
{
  const YAML::Node format = configuration[YamlFormatKey];
  if (!format)
    return boost::asio::ssl::context::pem;

  const std::string& value = format.as<std::string>();
  if (value == YamlFormatPemValue)
    return boost::asio::ssl::context::pem;

  if (value == YamlFormatAsn1Value)
    return boost::asio::ssl::context::asn1;

  throw std::runtime_error(
          "[soss::websocket::Server] Unrecognized file format type: " + value
          +". Only [" + YamlFormatPemValue + "] and [" + YamlFormatAsn1Value
          + "] formats are supported.");
}

//==============================================================================
static bool all_closed(
  const std::unordered_set<WsCppConnectionPtr>& connections)
{
  for (const auto& connection : connections)
  {
    if (connection->get_state() != websocketpp::session::state::closed)
      return false;
  }

  return true;
}

//==============================================================================
class Server : public Endpoint
{
public:

  Server()
  {
    // Do nothing
  }

  WsCppEndpoint* configure_endpoint(
    const RequiredTypes& /*types*/,
    const YAML::Node& configuration) override
  {
    const int32_t port = parse_port(configuration);
    if (port < 0)
      return nullptr;
    const uint16_t uport = static_cast<uint16_t>(port);

    const std::string cert_file = find_certificate(configuration);
    if (cert_file.empty())
      return nullptr;

    const std::string key_file = find_private_key(configuration);
    if (key_file.empty())
      return nullptr;

    const boost::asio::ssl::context::file_format format =
      parse_format(configuration);

    const YAML::Node auth_node = configuration[YamlAuthKey];
    if (auth_node)
    {
      _jwt_validator = std::make_unique<JwtValidator>();
      bool success = ServerConfig::load_auth_policy(*_jwt_validator, auth_node);
      if (!success)
      {
        std::cerr << "error loading auth config" << std::endl;
        return nullptr;
      }
    }

    if (!configure_server(uport, cert_file, key_file, format))
      return nullptr;

    return &_server;
  }

  bool configure_server(
    const uint16_t port,
    const std::string& cert_file,
    const std::string& key_file,
    const boost::asio::ssl::context::file_format format)
  {
    namespace asio = boost::asio;

    _context = std::make_shared<WsCppSslContext>(asio::ssl::context::tls);
    _context->set_options(
      asio::ssl::context::default_workarounds |
      asio::ssl::context::no_sslv2 |
      asio::ssl::context::no_sslv3);

    boost::system::error_code ec;
    _context->use_certificate_file(cert_file, format, ec);
    if (ec)
    {
      std::cerr << "[soss::websocket::Server] Failed to load certificate file ["
                << cert_file << "]: " << ec.message() << std::endl;
      return false;
    }

    // TODO(MXG): There is an alternative function
    // _context->use_private_key(key_file, format, ec);
    // which I guess is supposed to be used for keys that do not label
    // themselves as rsa private keys? We're currently using rsa private
    // keys, but this is probably something we should allow users to
    // configure from the soss config file.
    _context->use_rsa_private_key_file(key_file, format, ec);
    if (ec)
    {
      std::cerr << "[soss::websocket::Server] Failed to load private key file ["
                << key_file << "]: " << ec.message() << std::endl;
      return false;
    }

    // TODO(MXG): This helps to rerun soss more quickly if the server fell down
    // gracelessly. Is this something we really want? Are there any dangers to
    // using this?
    _server.set_reuse_addr(true);

    _server.clear_access_channels(
      websocketpp::log::alevel::frame_header |
      websocketpp::log::alevel::frame_payload);

    _server.init_asio();
    _server.start_perpetual();

    _server.set_message_handler(
      [&](WsCppWeakConnectPtr handle, WsCppMessagePtr message)
      {
        this->_handle_message(handle, message);
      });

    _server.set_close_handler(
      [&](WsCppWeakConnectPtr handle)
      {
        this->_handle_close(std::move(handle));
      });

    _server.set_open_handler(
      [&](WsCppWeakConnectPtr handle)
      {
        this->_handle_opening(std::move(handle));
      });

    _server.set_fail_handler(
      [&](WsCppWeakConnectPtr handle)
      {
        this->_handle_failed_connection(std::move(handle));
      });

    _server.set_tls_init_handler(
      [&](WsCppWeakConnectPtr /*handle*/) -> WsCppSslContextPtr
      {
        return _context;
      });

    _server.set_validate_handler(
      [&](WsCppWeakConnectPtr handle) -> bool
      {
        return this->_handle_validate(std::move(handle));
      });

    _server.listen(port);

    _server_thread = std::thread([&]() { this->_server.run(); });

    return true;
  }

  ~Server() override
  {
    _closing_down = true;

    // NOTE(MXG): _open_connections can get modified in other threads so we'll
    // make a copy of it here before using it.
    // TODO(MXG): We should probably be using mutexes to protect the operations
    // on these connections.

    // First instruct all connections to close
    const auto connection_copies = _open_connections;
    for (const auto& connection : connection_copies)
      connection->close(websocketpp::close::status::normal, "shutdown");

    // Then wait for all of them to close

    using namespace std::chrono_literals;
    const auto start_time = std::chrono::steady_clock::now();
    // TODO(MXG): Make these timeout parameters something that can be
    // configured by users.
    while (!all_closed(connection_copies))
    {
      std::this_thread::sleep_for(200ms);

      if (std::chrono::steady_clock::now() - start_time > 10s)
      {
        std::cerr << "[soss::websocket::Server] Timed out while waiting for "
                  << "the remote clients to acknowledge the connection "
                  << "shutdown request" << std::endl;
        break;
      }
    }

    if (_server_thread.joinable())
    {
      _server.stop();

      _server_thread.join();
    }
  }

  bool okay() const override
  {
    // TODO(MXG): How do we know if the server is okay?
    return true;
  }

  bool spin_once() override
  {
    if (!_has_spun_once)
    {
      _has_spun_once = true;
      _server.start_accept();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // TODO(MXG): How do we know if the server is okay?
    return true;
  }

  void runtime_advertisement(
    const std::string& topic,
    const std::string& message_type,
    const std::string& id,
    const YAML::Node& configuration) override
  {
    const std::string advertise_msg =
      get_encoding().encode_advertise_msg(
      topic, message_type, id, configuration);

    for (const WsCppConnectionPtr& connection : _open_connections)
      connection->send(advertise_msg);
  }

private:

  void _handle_message(
    const WsCppWeakConnectPtr& handle,
    const WsCppMessagePtr& message)
  {
    get_encoding().interpret_websocket_msg(
      message->get_payload(), *this, _server.get_con_from_hdl(handle));
  }

  void _handle_close(const WsCppWeakConnectPtr& handle)
  {
    const auto connection = _server.get_con_from_hdl(handle);
    std::cout << "[soss::websocket::Server] closed client connection ["
              << connection << "]" << std::endl;
    notify_connection_closed(connection);

    _open_connections.erase(connection);
  }

  void _handle_opening(const WsCppWeakConnectPtr& handle)
  {
    const auto connection = _server.get_con_from_hdl(handle);

    if (_closing_down)
    {
      connection->close(websocketpp::close::status::normal, "shutdown");
      return;
    }

    std::cout << "[soss::weboscket::Server] opened connection [" << connection
              << "]" << std::endl;
    notify_connection_opened(connection);

    _open_connections.insert(connection);
  }

  void _handle_failed_connection(const WsCppWeakConnectPtr& /*handle*/)
  {
    std::cout << "[soss::websocket::Server] An incoming client failed to "
              << "connect." << std::endl;
  }

  bool _handle_validate(const WsCppWeakConnectPtr& handle)
  {
    if (!_jwt_validator)
      return true;

    decltype(_server)::connection_ptr connection_ptr = _server.get_con_from_hdl(
      handle);
    std::vector<std::string> requested_sub_protos =
      connection_ptr->get_requested_subprotocols();
    if (requested_sub_protos.size() != 1)
    {
      connection_ptr->set_status(websocketpp::http::status_code::unauthorized);
      return false; // a valid soss client should always send exactly 1 subprotocols.
    }

    std::string token = requested_sub_protos[0]; // the subprotocol is the jwt token
    try
    {
      _jwt_validator->verify(token);
    }
    catch (const jwt::VerificationError& e)
    {
      std::cerr << e.what() << std::endl;
      connection_ptr->set_status(websocketpp::http::status_code::unauthorized);
      return false;
    }
    connection_ptr->select_subprotocol(token);
    return true;
  }

  WsCppServer _server;
  std::thread _server_thread;
  EncodingPtr _encoding;
  WsCppSslContextPtr _context;
  std::unordered_set<WsCppConnectionPtr> _open_connections;
  bool _has_spun_once = false;
  bool _closing_down = false;
  std::unique_ptr<JwtValidator> _jwt_validator;

};

SOSS_REGISTER_SYSTEM("websocket_server", soss::websocket::Server)

} // namespace websocket
} // namespace soss
