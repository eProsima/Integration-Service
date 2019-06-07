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

#include "ServerConfig.hpp"

#include <iostream>
#include <boost/algorithm/string.hpp>

namespace soss {
namespace websocket {

const std::string YamlPoliciesKey = "policies";
const std::string YamlRulesKey = "rules";
const std::string YamlSecretKey = "secret";
const std::string YamlPubkeyKey = "pubkey";
const std::string YamlAlgoKey = "algo";

bool ServerConfig::load_auth_policy(JwtValidator& jwt_validator, const YAML::Node& auth_node)
{
  const YAML::Node& policies_node = auth_node[YamlPoliciesKey];
  for (auto& policy_node : policies_node)
  {
    std::vector<VerificationPolicies::Rule> rules;
    for (const auto& r : policy_node[YamlRulesKey]) {
      std::string regex_pattern = glob_to_regex(r.second.as<std::string>());
      rules.emplace_back(VerificationPolicies::Rule{
        r.first.as<std::string>(), regex_pattern
      });
    }

    std::string secret_or_pub;
    if (policy_node[YamlSecretKey])
      secret_or_pub = policy_node[YamlSecretKey].as<std::string>();
    else
      secret_or_pub = policy_node[YamlPubkeyKey].as<std::string>();

    std::string algo = policy_node[YamlAlgoKey].as<std::string>();

    jwt_validator.add_verification_policy(VerificationPolicies::match_all(
      rules, secret_or_pub, algo));
  }

  if ((auth_node[YamlAlgoKey] && (!auth_node[YamlSecretKey] && !auth_node[YamlPubkeyKey])) ||
    (!auth_node[YamlAlgoKey] && (auth_node[YamlSecretKey] || auth_node[YamlPubkeyKey])))
  {
    std::cerr << "missing '" << YamlAlgoKey << "', '" << YamlSecretKey
              << "' or '" << YamlPubkeyKey << "'!" << std:: endl;
    return false;
  }

  if (!auth_node[YamlAlgoKey])
    return true;

  std::string secret_or_pub;
  if (auth_node[YamlSecretKey])
    secret_or_pub = auth_node[YamlSecretKey].as<std::string>();
  else
    secret_or_pub = auth_node[YamlPubkeyKey].as<std::string>();
  std::string algo = auth_node[YamlAlgoKey].as<std::string>();
  jwt_validator.add_verification_policy(VerificationPolicies::match_all(
    {}, secret_or_pub, algo));
  return true;
}

std::string ServerConfig::glob_to_regex(const std::string& s)
{
  using namespace boost::algorithm;

  return find_format_all_copy(s, token_finder(is_any_of(".*?\\")), [](auto s)
  {
    auto c = s.begin();
    switch (*c)
    {
    case '.':
      return std::string("\\.");
    case '*':
      return std::string(".*");
    case '?':
      return std::string(".?");
    case '\\':
      return std::string("\\\\");
    default:
      return std::string(s.begin(), s.end());
    }
  });
}

} // namespace websocket
} // namespace soss
