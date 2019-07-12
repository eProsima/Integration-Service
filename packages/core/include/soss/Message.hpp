<<<<<<< 82a0faa3b019fcbae4d18709be636ec90e3991d1
/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef SOSS__MESSAGE_HPP
#define SOSS__MESSAGE_HPP


#include <soss/core/export.hpp>
#include <functional>
#include <map>
#include <memory>
#include <string>

namespace soss {

// ================================ xtypes lib ==================================

#ifndef SOSS__MESSAGE_HPP
#define SOSS__MESSAGE_HPP

#include<string>
#include<map>

namespace soss
{


class MessageType
{
public:
    enum class Type { INT, STRING };

    MessageType(const std::string& name)
        :name_(name)
    {
    }

    ~MessageType() = default;

    const std::string& get_name() const
    {
        return name_;
    }

    bool operator == (const MessageType& other) const
    {
        return name_ == other.name_ && members_ == other.members_;
    }

    bool operator != (const MessageType& other) const
    {
        return !(*this == other);
    }

    Type& operator [] (const std::string& field)
    {
        return members_[field];
    }

    const std::map<std::string, Type>& get_members() const
    {
        return members_;
    }

    bool can_be_read_as(const MessageType& other) const
    {
        //TODO: Check matching by QoS
        return members_ == other.members_;
    }


private:
    std::string name_;
    std::map<std::string, Type> members_;
};


class MessageData
{
public:
    class Iterator { };

    MessageData(const MessageType& type)
        : type_(type)
    {
    }

    ~MessageData() = default;

    const MessageType& get_type() const
    {
        return type_;
    }

    bool operator == (const MessageData& other) const
    {
        return type_ == other.type_ && values_ == other.values_;
    }

    bool operator != (const MessageData& other) const
    {
        return !(*this == other);
    }

    std::string& operator [] (const std::string& field)
    {
        return values_[field];
    }

    const std::string& operator [] (const std::string& field) const
    {
        return values_.at(field);
    }

    const std::map<std::string, std::string>& get_values() const
    {
        return values_;
    }

    MessageData::Iterator get_iterator() const
    {
        //TODO
        return Iterator();
    }

private:
    const MessageType& type_;
    std::map<std::string, std::string> values_;
};


} //soss

#endif //SOSS__MESSAGE_HPP
