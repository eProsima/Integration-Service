#include "JwtValidator.hpp"

#include "Errors.hpp"

#include <unordered_map>
#include <regex>
#include <utility>
#include <iostream>

namespace soss {
namespace websocket {

VerificationPolicy::VerificationPolicy(
        std::vector<Rule> rules,
        std::vector<Rule> header_rules,
        std::string secret_or_pubkey
        )
    : _secret_or_pubkey(std::move(secret_or_pubkey))
    , _rules(std::move(rules))
    , _header_rules(std::move(header_rules))
{
    // This is so that we don't have to create the regexes everytime the policy is used.
    for (const auto& r : _rules)
    {
        _matchers[r.first] = std::regex{r.second};
    }
    for (const auto& r : _header_rules)
    {
        _header_matchers[r.first] = std::regex(r.second);
    }
}

void VerificationPolicy::check(
        const std::string& token,
        const json_t& header,
        const json_t& payload)
{
    jwt::decode(token, jwt::params::algorithms(
                {header["alg"].get_ref<const std::string&>()}), jwt::params::secret(
                _secret_or_pubkey));

    for (const auto& r : _header_rules)
    {
        auto it = header.find(r.first);
        if (it == header.end())
        {
            throw jwt::VerificationError("'" + r.first + "' not found in headers");
        }
        if (!it->is_string())
        {
            throw jwt::VerificationError("'" + r.first + "' expected to be string");
        }
        const auto& s = it->get_ref<const std::string&>();
        if (!std::regex_match(s, _header_matchers.at(r.first)))
        {
            throw jwt::VerificationError("'" + r.first + "' does not match policy");
        }
    }

    for (const auto& r : _rules)
    {
        auto it = payload.find(r.first);
        if (it == payload.end())
        {
            throw jwt::VerificationError("'" + r.first + "' not found in payload");
        }
        if (!it->is_string())
        {
            throw jwt::VerificationError("'" + r.first + "' expected to be string");
        }
        const auto& s = it->get_ref<const std::string&>();
        if (!std::regex_match(s, _matchers.at(r.first)))
        {
            throw jwt::VerificationError("'" + r.first + "' does not match policy");
        }
    }
}

void JwtValidator::verify(
        const std::string& token)
{
    using namespace jwt::params;

    auto parts = jwt::jwt_object::three_parts(token);
    json_t header = json_t::parse(
        jwt::base64_uri_decode(parts[0].data(), parts[0].size()), nullptr, false);
    if (header.is_discarded())
    {
        throw jwt::VerificationError("header not found");
    }
    json_t payload = json_t::parse(
        jwt::base64_uri_decode(parts[1].data(), parts[1].size()), nullptr, false);
    if (payload.is_discarded())
    {
        throw jwt::VerificationError("payload not found");
    }

    std::vector<std::string> error_msgs(_verification_policies.size());
    for (size_t i = 0; i < _verification_policies.size(); i++)
    {
        try
        {
            _verification_policies[i].check(token, header, payload);
            return;
        }
        catch (const jwt::VerificationError& e)
        {
            error_msgs[i] = e.what();
        }
    }
    std::stringstream ss;
    ss << "None of the policies pass verification" << std::endl;
    for (size_t i = 0; i < _verification_policies.size(); i++)
    {
        ss << "\tPolicy " << i + 1 << ": " << error_msgs[i] << std::endl;
    }
    throw jwt::VerificationError(ss.str());
}

void JwtValidator::add_verification_policy(
        const VerificationPolicy& policy)
{
    _verification_policies.emplace_back(policy);
}

} // namespace websocket
} // namespace soss
