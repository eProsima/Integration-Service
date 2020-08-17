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

#include <catch2/catch.hpp>

#include "paths.hpp"

#include <JwtValidator.hpp>
#include <ServerConfig.hpp>

#include <yaml-cpp/yaml.h>

using namespace soss::websocket;

// { "iss": "test" } signed with secret "test" and algo "HS256"
const std::string hs256_token =
  "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJ0ZXN0IiwiaWF0IjoxNTU1MDQyMjY4fQ.OhPGhQdSQLFBb1K0_jf5CAWOzIy3JfdEaQUeejWHUDs";

const std::string rs256_token = "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiIxMjM0NTY3ODkwIiwibmFtZSI6IkpvaG4gRG9lIiwiaWF0IjoxNTE2MjM5MDIyfQ.DHWUIXysKCG45XM0xyVNPUvf7_DLaCgGJIvCBTjpkuPnwUCkudTvmMVBLXlyQtIqXja1FwWX_5P5X4l0wfcEQ9V1vq-LmjT5-WcMrYS16IoRCO-7U--CHsA5x4j3Z34TMtDUPr-N0WUGxEHa6UcWuIeOrXJuzQ-Z4VRD4VcrkIXDkkngTFKolllUnBeFXcyOVO_S3LTfLY4fKcc9hNGRe0DIhFj-O3gEZcFCsqj7jLCE2g_-BZXvW0uALPPHoAyKGMfX9hlWhLPGvswvV7Heo5lhlTrdljFFsAfyWsyMJsG9USU3CrU2fTWTZG7Tvj3NVoFITKnwJ1BO_zK4xWqHBg";

TEST_CASE("validates jwt token", "[Verification]")
{
  JwtValidator jwt_validator;
  jwt_validator.add_verification_policy(VerificationPolicy(
      {}, {}, "test"));

  CHECK_NOTHROW(jwt_validator.verify(hs256_token));

  // { "iss": "test" } signed with secret "bad_token" and algo "HS256"
  std::string bad_token =
    "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJ0ZXN0IiwiaWF0IjoxNTU1MDQxOTE0fQ.uA-YwAns6NMVl7jgKueJVYSukUTenxaSV_xTxGHpF_E";
  CHECK_THROWS(jwt_validator.verify(bad_token));

  std::string invalid_token = "xaxaxa.xaxaxa.xaxaxa";
  CHECK_THROWS(jwt_validator.verify(invalid_token));
}

TEST_CASE("simple verification strategy", "[Verification]")
{
  JwtValidator jwt_validator;

  jwt_validator.add_verification_policy(VerificationPolicy(
      {{ "iss", "test" }, { "sub", "test" }}, {}, "test"));
  CHECK_THROWS(jwt_validator.verify(hs256_token));

  jwt_validator.add_verification_policy(VerificationPolicy(
      {{ "iss", "test" }}, {}, "test"));
  CHECK_NOTHROW(jwt_validator.verify(hs256_token));
}

TEST_CASE("pubkey verification", "[Verification]")
{
  JwtValidator jwt_validator;
  YAML::Node auth_node = YAML::Load(
    "{ pubkey: " + test::test_dir + "/test.pub }");
  REQUIRE(ServerConfig::load_auth_policy(jwt_validator, auth_node));
  CHECK_NOTHROW(jwt_validator.verify(rs256_token));
}

TEST_CASE("no secret or pubkey", "[Load Config]")
{
  JwtValidator jwt_validator;
  YAML::Node auth_node = YAML::Load("");
  CHECK_FALSE(ServerConfig::load_auth_policy(jwt_validator, auth_node));
}

TEST_CASE("both secret and pubkey", "[Load Config]")
{
  JwtValidator jwt_validator;
  YAML::Node auth_node = YAML::Load("{ secret: test, pubkey: test }");
  CHECK_FALSE(ServerConfig::load_auth_policy(jwt_validator, auth_node));
}

TEST_CASE("default policy", "[Load Config]")
{
  JwtValidator jwt_validator;
  YAML::Node auth_node = YAML::Load(
    R"raw(
{
  secret: test,
  algo: HS256,
}
)raw");
  ServerConfig::load_auth_policy(jwt_validator, auth_node);
  CHECK_NOTHROW(jwt_validator.verify(hs256_token));
}

TEST_CASE("no alg", "[Load Config]")
{
  JwtValidator jwt_validator;
  YAML::Node auth_node = YAML::Load(R"raw(
{
  secret: test
}
)raw");
  ServerConfig::load_auth_policy(jwt_validator, auth_node);
  CHECK_NOTHROW(jwt_validator.verify(hs256_token));
}

TEST_CASE("custom policy", "[Load Config]")
{
  JwtValidator jwt_validator;
  YAML::Node auth_node = YAML::Load(
    R"raw(
{
  policies: [
    {
      rules: {
        iss: test
      },
      secret: test,
      algo: HS256
    }
  ]
}
)raw");
  REQUIRE(ServerConfig::load_auth_policy(jwt_validator, auth_node));
  CHECK_NOTHROW(jwt_validator.verify(hs256_token));
}

TEST_CASE("wildcard pattern rule '*'", "[Load Config]")
{
  JwtValidator jwt_validator;
  YAML::Node auth_node = YAML::Load(
    R"raw(
{
  policies: [
    {
      rules: {
        iss: '*e*'
      },
      secret: test,
      algo: HS256
    }
  ]
}
)raw");
  ServerConfig::load_auth_policy(jwt_validator, auth_node);
  CHECK_NOTHROW(jwt_validator.verify(hs256_token));

  jwt_validator = JwtValidator{};
  auth_node = YAML::Load(
    R"raw(
{
  policies: [
    {
      rules: {
        iss: '*test*'
      },
      secret: test,
      algo: HS256
    }
  ]
}
)raw");
  ServerConfig::load_auth_policy(jwt_validator, auth_node);
  CHECK_NOTHROW(jwt_validator.verify(hs256_token));
}

TEST_CASE("wildcard pattern rule '?'", "[Load Config]")
{
  JwtValidator jwt_validator;
  YAML::Node auth_node = YAML::Load(
    R"raw(
{
  policies: [
    {
      rules: {
        iss: '?es?'
      },
      secret: test,
      algo: HS256
    }
  ]
}
)raw");
  ServerConfig::load_auth_policy(jwt_validator, auth_node);
  CHECK_NOTHROW(jwt_validator.verify(hs256_token));

  jwt_validator = JwtValidator{};
  auth_node = YAML::Load(
    R"raw(
{
  policies: [
    {
      rules: {
        iss: 't?t'
      },
      secret: test,
      algo: HS256
    }
  ]
}
)raw");
  ServerConfig::load_auth_policy(jwt_validator, auth_node);
  CHECK_THROWS(jwt_validator.verify(hs256_token));
}

TEST_CASE("wildcard pattern rule mixed", "[Load Config]")
{
  JwtValidator jwt_validator;
  YAML::Node auth_node = YAML::Load(
    R"raw(
{
  policies: [
    {
      rules: {
        iss: '*[abt][a-z]s?'
      },
      secret: test,
      algo: HS256
    }
  ]
}
)raw");
  ServerConfig::load_auth_policy(jwt_validator, auth_node);
  CHECK_NOTHROW(jwt_validator.verify(hs256_token));
}
