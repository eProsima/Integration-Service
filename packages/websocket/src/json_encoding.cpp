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

#include "Encoding.hpp"
#include "Endpoint.hpp"

#include <soss/json/conversion.hpp>
#include <soss/json/json.hpp>

#include <unordered_set>

namespace soss {
namespace websocket {

using json::Json;
using nlohmann::detail::value_t;

//==============================================================================
// message fields
const std::string JsonOpKey = "op";
const std::string JsonIdKey = "id";
const std::string JsonTopicNameKey = "topic";
const std::string JsonTypeNameKey = "type";
const std::string JsonMsgKey = "msg";
const std::string JsonServiceKey = "service";
const std::string JsonArgsKey = "args";
const std::string JsonValuesKey = "values";
const std::string JsonResultKey = "result";


// op codes
const std::string JsonOpAdvertiseTopicKey = "advertise";
const std::string JsonOpUnadvertiseTopicKey = "unadvertise";
const std::string JsonOpPublishKey = "publish";
const std::string JsonOpSubscribeKey = "subscribe";
const std::string JsonOpUnsubscribeKey = "unsubscribe";
const std::string JsonOpServiceRequestKey = "call_service";
const std::string JsonOpAdvertiseServiceKey = "advertise_service";
const std::string JsonOpUnadvertiseServiceKey = "unadvertise_service";
const std::string JsonOpServiceResponseKey = "service_response";

// idl of ROSBRIDGE PROTOCOL messages
const std::string idl_messages =
        R"(
struct fragment
{
    string id;
    string data;
    int32 num;
    int32 total;
};

struct png
{
    string id;
    string data;
    int32 num;
    int32 total;
};

struct cbor
{
    sequence<int8> data;
};

struct set_level
{
    string id;
    string level;
};

struct status
{
    string id;
    string level;
    string msg;
};

struct auth
{
    string mac;
    string client;
    string dest;
    string rand;
    int32 t;
    string level;
    int32 end;
};

struct advertise
{
    string id;
    string topic;
    string type;
};

struct unadvertise
{
    string id;
    string topic;
};

struct publish
{
    string id;
    string topic;
    string msg;
};

struct subscribe
{
    string id;
    string topic;
    string type;
    int32 throttle_rate;
    int32 queue_length;
    int32 fragment_size;
    string compression;
};

struct unsubscribe
{
    string id;
    string topic;
};

struct call_service
{
    string id;
    string service;
    sequence<string> args;
    int32 fragment_size;
    string compression;
};

struct advertise_service
{
    string type;
    string service;
};

struct unadvertise_service
{
    string service;
};

struct service_response
{
    string id;
    string service;
    sequence<string> values;
    boolean result;
};

)";

//==============================================================================
static void throw_missing_key(
        const Json& object,
        const std::string& key)
{
    const std::string op_code = object.at("op").get<std::string>();
    throw std::runtime_error(
              "[soss::websocket::JsonEncoding] Incoming websocket message with op "
              "code [" + op_code + "] is missing the required field [" + key
              + "]:\n" + object.dump());
}

//==============================================================================
static std::string get_optional_string(
        const Json& object,
        const std::string& key)
{
    const auto it = object.find(key);

    if (it == object.end())
    {
        return "";
    }
    else
    {
        std::string temp_str = it.value().dump();
        if (temp_str.find("\"") == 0)
        {
            return temp_str.substr(1, temp_str.size() - 2);
        }
        return temp_str;
    }
}

//==============================================================================
static std::string get_required_string(
        const Json& object,
        const std::string& key)
{
    const auto it = object.find(key);
    if (it == object.end())
    {
        throw_missing_key(object, key);
    }

    std::string temp_str = it.value().dump();
    if (temp_str.find("\"") == 0)
    {
        return temp_str.substr(1, temp_str.size() - 2);
    }
    return temp_str;
}

//==============================================================================
static xtypes::DynamicData get_required_msg(
        const Json& object,
        const xtypes::DynamicType& type,
        const std::string& key)
{
    const auto it = object.find(key);
    if (it == object.end())
    {
        throw_missing_key(object, key);
    }

    return json::convert(type, it.value());
}

//==============================================================================
class JsonEncoding : public Encoding
{
public:

    JsonEncoding()
    {
        types_ = xtypes::idl::parse(idl_messages).get_all_types();
    }

    void interpret_websocket_msg(
            const std::string& msg_str,
            Endpoint& endpoint,
            std::shared_ptr<void> connection_handle) const override
    {
        const auto msg = Json::parse(msg_str);

        const auto op_it = msg.find(JsonOpKey);
        if (op_it == msg.end())
        {
            throw std::runtime_error(
                      "[soss::websocket::JsonEncoding] Incoming message was missing "
                      "the required op code: " + msg_str);
        }

        const std::string& op_str = op_it.value().get<std::string>();

        xtypes::DynamicType::Ptr type_ptr;

        auto type_it = types_.find(op_str);
        if (type_it != types_.end())
        {
            type_ptr = type_it->second;
        }

        // Publish is the most likely type of message to be received, so we'll check
        // for that type first.
        if (op_str == JsonOpPublishKey)
        {
            std::string topic_name = get_required_string(msg, JsonTopicNameKey);
            const xtypes::DynamicType& dest_type = get_type_by_topic(topic_name);
            endpoint.receive_publication_ws(
                topic_name,
                get_required_msg(msg, dest_type, JsonMsgKey),
                std::move(connection_handle));
            return;
        }

        // A service request is the roughly the second/third most likely type of
        // message to be received, so we'll check for that type next.
        if (op_str == JsonOpServiceRequestKey)
        {
            std::string topic_name = get_required_string(msg, JsonServiceKey);
            const xtypes::DynamicType& dest_type = get_type_by_topic(topic_name);
            endpoint.receive_service_request_ws(
                topic_name,
                get_required_msg(msg, dest_type, JsonArgsKey),
                get_optional_string(msg, JsonIdKey),
                std::move(connection_handle));
            return;
        }

        // A service response is the roughly the second/third most likely type of
        // message to be received, so we'll check for that type next.
        if (op_str == JsonOpServiceResponseKey)
        {
            std::string topic_name = get_required_string(msg, JsonServiceKey);
            const xtypes::DynamicType& dest_type = get_type_by_topic(topic_name);
            endpoint.receive_service_response_ws(
                get_required_string(msg, JsonServiceKey),
                get_required_msg(msg, dest_type, JsonValuesKey),
                get_optional_string(msg, JsonIdKey),
                std::move(connection_handle));
            return;
        }

        if (op_str == JsonOpAdvertiseTopicKey)
        {
            const xtypes::DynamicType& topic_type = get_type(get_required_string(msg, JsonTypeNameKey));
            endpoint.receive_topic_advertisement_ws(
                get_required_string(msg, JsonTopicNameKey),
                topic_type,
                get_optional_string(msg, JsonIdKey),
                std::move(connection_handle));
            return;
        }

        if (op_str == JsonOpUnadvertiseTopicKey)
        {
            endpoint.receive_topic_unadvertisement_ws(
                get_required_string(msg, JsonTopicNameKey),
                get_optional_string(msg, JsonIdKey),
                std::move(connection_handle));
            return;
        }

        if (op_str == JsonOpSubscribeKey)
        {
            const xtypes::DynamicType* topic_type = get_type_ptr(get_optional_string(msg, JsonTypeNameKey));
            endpoint.receive_subscribe_request_ws(
                get_required_string(msg, JsonTopicNameKey),
                topic_type,
                get_optional_string(msg, JsonIdKey),
                std::move(connection_handle));
            return;
        }

        if (op_str == JsonOpUnsubscribeKey)
        {
            endpoint.receive_unsubscribe_request_ws(
                get_required_string(msg, JsonTopicNameKey),
                get_optional_string(msg, JsonIdKey),
                std::move(connection_handle));
            return;
        }

        if (op_str == JsonOpAdvertiseServiceKey)
        {
            const xtypes::DynamicType& topic_type = get_type(get_required_string(msg, JsonTypeNameKey));
            endpoint.receive_service_advertisement_ws(
                get_required_string(msg, JsonServiceKey),
                topic_type,
                std::move(connection_handle));
            return;
        }

        if (op_str == JsonOpUnadvertiseServiceKey)
        {
            const xtypes::DynamicType* topic_type = get_type_ptr(get_optional_string(msg, JsonTypeNameKey));
            endpoint.receive_service_unadvertisement_ws(
                get_required_string(msg, JsonServiceKey),
                topic_type,
                std::move(connection_handle));
        }
    }

    std::string encode_publication_msg(
            const std::string& topic_name,
            const std::string& topic_type,
            const std::string& id,
            const xtypes::DynamicData& msg) const override
    {
        Json output;
        output[JsonOpKey] = JsonOpPublishKey;
        output[JsonTopicNameKey] = topic_name;
        output[JsonMsgKey] = json::convert(msg);
        if (!id.empty())
        {
            output[JsonIdKey] = id;
        }

        types_by_topic_[topic_name] = topic_type;

        return output.dump();
    }

    std::string encode_service_response_msg(
            const std::string& service_name,
            const std::string& service_type,
            const std::string& id,
            const xtypes::DynamicData& response,
            const bool result) const override
    {
        Json output;
        output[JsonOpKey] = JsonOpServiceResponseKey;
        output[JsonServiceKey] = service_name;
        output[JsonValuesKey] = json::convert(response);
        output[JsonResultKey] = result;
        if (!id.empty())
        {
            output[JsonIdKey] = id;
        }

        types_by_topic_[service_name] = service_type;

        return output.dump();
    }

    std::string encode_subscribe_msg(
            const std::string& topic_name,
            const std::string& message_type,
            const std::string& id,
            const YAML::Node& /*configuration*/) const override
    {
        // TODO(MXG): Consider parsing the `configuration` for details like
        // throttle_rate, queue_length, fragment_size, and compression
        Json output;
        output[JsonOpKey] = JsonOpSubscribeKey;
        output[JsonTopicNameKey] = topic_name;
        output[JsonTypeNameKey] = message_type;
        if (!id.empty())
        {
            output[JsonIdKey] = id;
        }

        types_by_topic_[topic_name] = message_type;

        return output.dump();
    }

    std::string encode_advertise_msg(
            const std::string& topic_name,
            const std::string& message_type,
            const std::string& id,
            const YAML::Node& /*configuration*/) const override
    {
        Json output;
        output[JsonOpKey] = JsonOpAdvertiseTopicKey;
        output[JsonTopicNameKey] = topic_name;
        output[JsonTypeNameKey] = message_type;
        if (!id.empty())
        {
            output[JsonIdKey] = id;
        }

        types_by_topic_[topic_name] = message_type;

        return output.dump();
    }

    std::string encode_call_service_msg(
            const std::string& service_name,
            const std::string& service_type,
            const xtypes::DynamicData& service_request,
            const std::string& id,
            const YAML::Node& /*configuration*/) const override
    {
        // TODO(MXG): Consider parsing the `configuration` for details like
        // fragment_size and compression
        Json output;
        output[JsonOpKey] = JsonOpServiceRequestKey;
        output[JsonServiceKey] = service_name;
        output[JsonArgsKey] = json::convert(service_request);
        if (!id.empty())
        {
            output[JsonIdKey] = id;
        }

        types_by_topic_[service_name] = service_type;

        return output.dump();
    }

    std::string encode_advertise_service_msg(
            const std::string& service_name,
            const std::string& service_type,
            const std::string& /*id*/,
            const YAML::Node& /*configuration*/) const override
    {
        Json output;
        output[JsonOpKey] = JsonOpAdvertiseServiceKey;
        output[JsonTypeNameKey] = service_type;
        output[JsonServiceKey] = service_name;

        types_by_topic_[service_name] = service_type;

        return output.dump();
    }

    const xtypes::DynamicType& get_type(
            const std::string& type_name) const
    {
        auto type_it = types_.find(type_name);
        if (type_it != types_.end())
        {
            return *type_it->second;
        }
        else
        {
            throw std::runtime_error(
                      "[soss::websocket::JsonEncoding] Incoming message refers an unregistered "
                      "type: " + type_name);
        }
    }

    const xtypes::DynamicType* get_type_ptr(
            const std::string& type_name) const
    {
        auto type_it = types_.find(type_name);
        if (type_it != types_.end())
        {
            return type_it->second.get();
        }
        else
        {
            throw std::runtime_error(
                      "[soss::websocket::JsonEncoding] Incoming message refers an unregistered "
                      "type: " + type_name);
        }
    }

    bool add_type(
            const xtypes::DynamicType& type,
            const std::string& type_name) override
    {
        std::string name = type_name.empty() ? type.name() : type_name;
        auto result = types_.emplace(name, type);
        return result.second;
    }

    const xtypes::DynamicType& get_type_by_topic(
            const std::string& topic_name) const
    {
        return get_type(types_by_topic_[topic_name]);
    }

protected:

    std::map<std::string, xtypes::DynamicType::Ptr> types_;
    mutable std::map<std::string, std::string> types_by_topic_;

};

//==============================================================================
EncodingPtr make_json_encoding()
{
    return std::make_shared<JsonEncoding>();
}

} // namespace websocket
} // namespace soss
