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

#include <gtest/gtest.h>

#include <jwt/jwt.hpp>
#include <yaml-cpp/yaml.h>

#include <JwtValidator.hpp>
#include <ServerConfig.hpp>

using namespace soss::websocket;

// { "iss": "test" } signed with secret "test" and algo "HS256"
const std::string test_token =
        "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJ0ZXN0IiwiaWF0IjoxNTU1MDQyMjY4fQ.OhPGhQdSQLFBb1K0_jf5CAWOzIy3JfdEaQUeejWHUDs";

TEST(JwtValidator, Validates_jwt_token)
{
    JwtValidator jwt_validator;
    jwt_validator.add_verification_policy(VerificationPolicies::match_all(
                {}, "test", "HS256"));

    EXPECT_TRUE(jwt_validator.verify(test_token));

    // { "iss": "test" } signed with secret "bad_token" and algo "HS256"
    std::string bad_token =
            "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJ0ZXN0IiwiaWF0IjoxNTU1MDQxOTE0fQ.uA-YwAns6NMVl7jgKueJVYSukUTenxaSV_xTxGHpF_E";
    EXPECT_FALSE(jwt_validator.verify(bad_token));

    std::string invalid_token = "xaxaxa.xaxaxa.xaxaxa";
    EXPECT_FALSE(jwt_validator.verify(invalid_token));
}

TEST(JwtValidator, Simple_verification_strategy)
{
    JwtValidator jwt_validator;

    jwt_validator.add_verification_policy(VerificationPolicies::match_all(
                {{ "iss", "test" }, { "sub", "test" }}, "test", "HS256"));
    EXPECT_FALSE(jwt_validator.verify(test_token));

    jwt_validator.add_verification_policy(VerificationPolicies::match_all(
                {{ "iss", "test" }}, "test", "HS256"));
    EXPECT_TRUE(jwt_validator.verify(test_token));
}

TEST(JwtValidator, Default_policy)
{
    JwtValidator jwt_validator;
    YAML::Node auth_node = YAML::Load(R"raw(
policies: [
  {
    secret: test,
    algo: HS256
  }
]
)raw");
    ServerConfig::load_auth_policy(jwt_validator, auth_node);
    EXPECT_TRUE(jwt_validator.verify(test_token));
}

TEST(JwtValidator, Custom_policy)
{
    JwtValidator jwt_validator;
    YAML::Node auth_node = YAML::Load(
        R"raw(
policies: [
  {
    rules: {
      iss: test
    },
    secret: test,
    algo: HS256
  }
]
)raw");
    ServerConfig::load_auth_policy(jwt_validator, auth_node);
    EXPECT_TRUE(jwt_validator.verify(test_token));
}

TEST(JwtValidator, Wildcard_pattern_rule_asterisk)
{
    JwtValidator jwt_validator;
    YAML::Node auth_node = YAML::Load(
        R"raw(
policies: [
  {
    rules: {
      iss: '*e*'
    },
    secret: test,
    algo: HS256
  }
]
)raw");
    ServerConfig::load_auth_policy(jwt_validator, auth_node);
    EXPECT_TRUE(jwt_validator.verify(test_token));

    jwt_validator = JwtValidator{};
    auth_node = YAML::Load(
        R"raw(
policies: [
  {
    rules: {
      iss: '*test*'
    },
    secret: test,
    algo: HS256
  }
]
)raw");
    ServerConfig::load_auth_policy(jwt_validator, auth_node);
    EXPECT_TRUE(jwt_validator.verify(test_token));
}

TEST(JwtValidator, Wildcard_pattern_rule_question_mark)
{
    JwtValidator jwt_validator;
    YAML::Node auth_node = YAML::Load(
        R"raw(
policies: [
  {
    rules: {
      iss: '?es?'
    },
    secret: test,
    algo: HS256
  }
]
)raw");
    ServerConfig::load_auth_policy(jwt_validator, auth_node);
    EXPECT_TRUE(jwt_validator.verify(test_token));

    jwt_validator = JwtValidator{};
    auth_node = YAML::Load(
        R"raw(
policies: [
  {
    rules: {
      iss: 't?t'
    },
    secret: test,
    algo: HS256
  }
]
)raw");
    ServerConfig::load_auth_policy(jwt_validator, auth_node);
    EXPECT_FALSE(jwt_validator.verify(test_token));
}

TEST(JwtValidator, Wildcard_pattern_rule_mixed)
{
    JwtValidator jwt_validator;
    YAML::Node auth_node = YAML::Load(
        R"raw(
policies: [
  {
    rules: {
      iss: '*[abt][a-z]s?'
    },
    secret: test,
    algo: HS256
  }
]
)raw");
    ServerConfig::load_auth_policy(jwt_validator, auth_node);
    EXPECT_TRUE(jwt_validator.verify(test_token));
}

int main(
        int argc,
        char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}