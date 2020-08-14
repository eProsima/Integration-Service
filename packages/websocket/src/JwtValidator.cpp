#include "JwtValidator.hpp"

#include <unordered_map>
#include <regex>

namespace soss {
namespace websocket {

bool JwtValidator::verify(const std::string& token)
{
  using namespace jwt::params;

  auto parts = jwt::jwt_object::three_parts(token);
  json_t header = json_t::parse(
    jwt::base64_uri_decode(parts[0].data(), parts[0].size()), nullptr, false);
  if (header.is_discarded())
    return false;
  json_t payload = json_t::parse(
    jwt::base64_uri_decode(parts[1].data(), parts[1].size()), nullptr, false);
  if (payload.is_discarded())
    return false;

  VerificationStrategy vs;
  bool has_strat = false;
  for (auto& handler : _verification_policies)
  {
    has_strat = handler(header, payload, vs);
    if (has_strat)
      break;
  }
  if (!has_strat)
    return false;

  std::error_code ec;
  jwt::decode(token, algorithms(
      {header["alg"].get_ref<std::string&>()}), ec, secret(vs.secret_or_pub));
  return !static_cast<bool>(ec);
}

void JwtValidator::add_verification_policy(const VerificationPolicy& policy)
{
  _verification_policies.emplace_back(policy);
}

VerificationPolicy VerificationPolicies::match_all(
  const std::vector<Rule>& rules,
  const std::vector<Rule>& header_rules,
  const std::string& secret_or_pub)
{
  // This is so that we don't have to create the regexes everytime the policy is used.
  std::unordered_map<std::string, std::regex> header_matchers;
  for (auto& r : header_rules)
    header_matchers[r.first] = std::regex(r.second);

  std::unordered_map<std::string, std::regex> matchers;
  for (auto& r : rules)
    matchers[r.first] = std::regex{r.second};

  return [=](
    const json_t& header, const json_t& payload,
    VerificationStrategy& vs) -> bool
    {
      for (auto& r : header_rules)
      {
        auto it = header.find(r.first);
        if (it == header.end() || !it->is_string())
          return false;
        const auto& s = it->get_ref<const std::string&>();
        if (!std::regex_match(s, header_matchers.at(r.first)))
          return false;
      }

      for (auto& r : rules)
      {
        auto it = payload.find(r.first);
        if (it == payload.end() || !it->is_string())
          return false;
        const auto& s = it->get_ref<const std::string&>();
        if (!std::regex_match(s, matchers.at(r.first)))
          return false;
      }
      vs.secret_or_pub = secret_or_pub;
      return true;
    };
}

} // namespace websocket
} // namespace soss
