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

#ifndef SOSS__WEBSOCKET__SRC__JWTVALIDATOR_HPP
#define SOSS__WEBSOCKET__SRC__JWTVALIDATOR_HPP

#include <jwt/jwt.hpp>

#include <regex>

namespace soss {
namespace websocket {

class VerificationPolicy
{
public:

    using Rule = std::pair<std::string, std::string>;

    VerificationPolicy(
            std::vector<Rule> rules,
            std::vector<Rule> header_rules,
            std::string secret_or_pubkey);

    inline const std::string& secret_or_pubkey() const
    {
        return _secret_or_pubkey;
    }

    /**
     * @throws VerificationError
     */
    void check(
            const std::string& token,
            const json_t& header,
            const json_t& payload);

private:

    std::string _secret_or_pubkey;
    std::vector<Rule> _rules;
    std::vector<Rule> _header_rules;
    std::unordered_map<std::string, std::regex> _matchers;
    std::unordered_map<std::string, std::regex> _header_matchers;
};

class JwtValidator
{
public:

    /**
     * @throws VerificationError
     */
    void verify(
            const std::string& token);

    /// \brief Adds a policy to resolve the verification strategy to use
    /// \details The VerificationPolicy should set the VerificationStrategy and returns true if
    /// it is able to provide a strategy. If there are multiple policies that can process a token,
    /// the 1st policy that matches is used. VerificationPolicyFactory contains some simple
    /// predefined policies.
    /// \remarks The idea is that JwtValidator should support verfiying in multiple use cases.
    /// For example, choosing a secret based on the issuer or other claims and any custom strategy
    /// as required. There is no way to open up such flexibility from within the class so the
    /// conclusion is to have a handler that the consumer supplies to choose the verification method.
    /// \param policy
    void add_verification_policy(
            const VerificationPolicy& policy);

private:

    std::vector<VerificationPolicy> _verification_policies;
};

} // namespace websocket
} // namespace soss

#endif // SOSS__WEBSOCKET__SRC__JWTVALIDATOR_HPP
