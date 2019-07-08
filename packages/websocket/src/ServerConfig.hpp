#ifndef SERVERCONFIG_HPP
#define SERVERCONFIG_HPP

#include <yaml-cpp/yaml.h>

#include "JwtValidator.hpp"
#include <soss/Soss_export.hpp>

namespace soss {
namespace websocket {

class SYSTEM_HANDLE_EXPORT ServerConfig
{
public:
  static bool load_auth_policy(JwtValidator& jwt_validator, const YAML::Node& auth_node);

private:
  static std::string glob_to_regex(const std::string& s);
};

} // namespace websocket
} // namespace soss

#endif // SERVERCONFIG_HPP
