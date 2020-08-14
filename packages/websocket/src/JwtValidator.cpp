#include "JwtValidator.hpp"

#include <unordered_map>
#include <regex>
#include <utility>

namespace soss {
namespace websocket {

VerificationPolicy::VerificationPolicy(std::vector<Rule> rules,
  std::vector<Rule> header_rules, std::string secret_or_pubkey)
: _secret_or_pubkey(std::move(secret_or_pubkey)), _rules(std::move(rules)),
  _header_rules(std::move(header_rules))
{
  // This is so that we don't have to create the regexes everytime the policy is used.
  for (auto& r : _rules)
    _matchers[r.first] = std::regex{r.second};
  for (auto& r : _header_rules)
    _header_matchers[r.first] = std::regex(r.second);
}

bool VerificationPolicy::check(const std::string& token,
  const json_t& header,
  const json_t& payload)
{
  std::error_code ec;
  jwt::decode(token, jwt::params::algorithms(
      {header["alg"].get_ref<const std::string&>()}), ec, jwt::params::secret(
      _secret_or_pubkey));
  if (ec)
    return false;

  for (auto& r : _header_rules)
  {
    auto it = header.find(r.first);
    if (it == header.end() || !it->is_string())
      return false;
    const auto& s = it->get_ref<const std::string&>();
    if (!std::regex_match(s, _header_matchers.at(r.first)))
      return false;
  }

  for (auto& r : _rules)
  {
    auto it = payload.find(r.first);
    if (it == payload.end() || !it->is_string())
      return false;
    const auto& s = it->get_ref<const std::string&>();
    if (!std::regex_match(s, _matchers.at(r.first)))
      return false;
  }
  return true;
}

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

  for (auto& policy : _verification_policies)
  {
    if (policy.check(token, header, payload))
      return true;
  }
  return false;
}

void JwtValidator::add_verification_policy(const VerificationPolicy& policy)
{
  _verification_policies.emplace_back(policy);
}

} // namespace websocket
} // namespace soss
