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

#include "Errors.hpp"

#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>

namespace soss {
namespace websocket {

const std::string YamlPoliciesKey = "policies";
const std::string YamlRulesKey = "rules";
const std::string YamlSecretKey = "secret";
const std::string YamlPubkeyKey = "pubkey";
const std::string YamlAlgoKey = "algo";

bool ServerConfig::load_auth_policy(JwtValidator& jwt_validator,
  const YAML::Node& auth_node)
{
  try
  {
    const YAML::Node& policies_node = auth_node[YamlPoliciesKey];
    if (!policies_node)
    {
      const auto& default_policy_node = auth_node;
      jwt_validator.add_verification_policy(_parse_policy_yaml(
          default_policy_node));
    }
    else
    {
      for (auto& policy_node : policies_node)
        jwt_validator.add_verification_policy(_parse_policy_yaml(policy_node));
    }
  }
  catch (const ParseError& e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }

  return true;
}

std::string ServerConfig::_glob_to_regex(const std::string& s)
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

VerificationPolicy ServerConfig::_parse_policy_yaml(
  const YAML::Node& policy_node)
{
  if (!policy_node[YamlSecretKey] && !policy_node[YamlPubkeyKey])
  {
    std::stringstream ss;
    ss << "fail to load config, missing both '" << YamlSecretKey << "' and '" <<
      YamlPubkeyKey <<
      "'" << std::endl;
    throw ParseError(ss.str());
  }

  if (policy_node[YamlSecretKey] && policy_node[YamlPubkeyKey])
  {
    throw ParseError(
            "cannot have both '" + YamlSecretKey + "' and '" + YamlPubkeyKey +
            "'");
  }

  std::string secret_or_pubkey;
  if (policy_node[YamlSecretKey])
    secret_or_pubkey = policy_node[YamlSecretKey].as<std::string>();
  else
  {
    const auto filepath = policy_node[YamlPubkeyKey].as<std::string>();
    std::ifstream fs(filepath);
    fs >> secret_or_pubkey;
  }

  std::vector<VerificationPolicy::Rule> rules;
  std::vector<VerificationPolicy::Rule> header_rules;

  // The "algo" options serve as a way to restrict certain algos,
  // we shouldn't need it in the policy, we should respect the "alg" declared in the jwt header.
  // If there is a need to restrict certain algos, we can add "alg" to the list of rules.
  // For backwards compatibility, if we see an "algo" option, convert it to an "alg" rule.
  if (policy_node[YamlAlgoKey])
  {
    header_rules.emplace_back(VerificationPolicy::Rule("alg",
      _glob_to_regex(policy_node[YamlAlgoKey].as<std::string>())));
  }

  for (const auto& r : policy_node[YamlRulesKey])
  {
    std::string regex_pattern = _glob_to_regex(r.second.as<std::string>());
    rules.emplace_back(VerificationPolicy::Rule{
        r.first.as<std::string>(), regex_pattern
      });
  }

  return VerificationPolicy(
    rules, header_rules, secret_or_pubkey);
}

} // namespace websocket
} // namespace soss
