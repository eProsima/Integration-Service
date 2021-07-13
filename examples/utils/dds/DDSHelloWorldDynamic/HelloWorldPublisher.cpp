// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file HelloWorldPublisher.cpp
 *
 */

#include "HelloWorldPublisher.h"
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>

#include <fastrtps/types/DynamicTypeBuilderFactory.h>
#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/types/DynamicTypeBuilder.h>
#include <fastrtps/types/DynamicTypeBuilderPtr.h>
#include <fastrtps/types/DynamicType.h>

#include <thread>

using namespace eprosima::fastdds::dds;

HelloWorldPublisher::HelloWorldPublisher()
    : participant_(nullptr)
    , publisher_(nullptr)
    , topic_(nullptr)
    , writer_(nullptr)
    , m_DynType(eprosima::fastrtps::types::DynamicType_ptr(nullptr))
{
}

bool HelloWorldPublisher::init(
        const eprosima::fastdds::dds::DomainId_t domain_id,
        const std::string& topic_name)
{
    // Create basic builders
    eprosima::fastrtps::types::DynamicTypeBuilder_ptr struct_type_builder(
        eprosima::fastrtps::types::DynamicTypeBuilderFactory::get_instance()->create_struct_builder());

    // Add members to the struct.
    struct_type_builder->add_member(0, "data",
        eprosima::fastrtps::types::DynamicTypeBuilderFactory::get_instance()->create_string_type());
    struct_type_builder->set_name("std_msgs::msg::dds_::String_");

    eprosima::fastrtps::types::DynamicType_ptr dynType = struct_type_builder->build();
    m_DynType.SetDynamicType(dynType);
    m_Hello = eprosima::fastrtps::types::DynamicDataFactory::get_instance()->create_data(dynType);

    m_Hello->set_string_value("Hello DDS Dynamic World", 0);

    DomainParticipantQos pqos;
    pqos.name("Participant_pub");
    participant_ = DomainParticipantFactory::get_instance()->create_participant(domain_id, pqos);

    if (participant_ == nullptr)
    {
        return false;
    }

    //REGISTER THE TYPE
    participant_->register_type(m_DynType);

    //CREATE THE PUBLISHER
    publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);

    if (publisher_ == nullptr)
    {
        return false;
    }

    topic_ = participant_->create_topic(topic_name, "std_msgs::msg::dds_::String_", TOPIC_QOS_DEFAULT);

    if (topic_ == nullptr)
    {
        return false;
    }

    // CREATE THE WRITER
    writer_ = publisher_->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT, &listener_);

    if (writer_ == nullptr)
    {
        return false;
    }

    std::cout << "DDSHelloWorldPublisher running under DDS Domain ID: " << domain_id << std::endl;
    return true;
}

HelloWorldPublisher::~HelloWorldPublisher()
{
    if (writer_ != nullptr)
    {
        publisher_->delete_datawriter(writer_);
    }
    if (publisher_ != nullptr)
    {
        participant_->delete_publisher(publisher_);
    }
    if (topic_ != nullptr)
    {
        participant_->delete_topic(topic_);
    }
    DomainParticipantFactory::get_instance()->delete_participant(participant_);
}

void HelloWorldPublisher::PubListener::on_publication_matched(
        eprosima::fastdds::dds::DataWriter*,
        const eprosima::fastdds::dds::PublicationMatchedStatus& info)
{
    if (info.current_count_change == 1)
    {
        matched_ = info.total_count;
        firstConnected_ = true;
        std::cout << "Publisher matched." << std::endl;
    }
    else if (info.current_count_change == -1)
    {
        matched_ = info.total_count;
        std::cout << "Publisher unmatched." << std::endl;
    }
    else
    {
        std::cout << info.current_count_change
                  << " is not a valid value for PublicationMatchedStatus current count change" << std::endl;
    }
}

void HelloWorldPublisher::runThread(
        const uint32_t samples,
        const uint32_t sleep)
{
    if (samples == 0)
    {
        while (!stop_)
        {
            if (publish(false))
            {
                std::string data;
                m_Hello->get_string_value(data, 0);
                std::cout << "Message: " << data
                          << " SENT" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep));
        }
    }
    else
    {
        for (uint32_t i = 0; i < samples; ++i)
        {
            if (!publish())
            {
                --i;
            }
            else
            {
                std::string data;
                m_Hello->get_string_value(data, 0);
                std::cout << "Message: " << data
                          << " SENT" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep));
        }
    }
}

void HelloWorldPublisher::run(
        const uint32_t samples,
        const uint32_t sleep)
{
    stop_ = false;
    std::thread thread(&HelloWorldPublisher::runThread, this, samples, sleep);
    if (samples == 0)
    {
        std::cout << "Publisher running. Please press enter to stop the Publisher at any time." << std::endl;
        std::cin.ignore();
        stop_ = true;
    }
    else
    {
        std::cout << "Publisher running " << samples << " samples." << std::endl;
    }
    thread.join();
}

bool HelloWorldPublisher::publish(
        bool waitForListener)
{
    if (listener_.firstConnected_ || !waitForListener || listener_.matched_ > 0)
    {
        writer_->write(m_Hello.get());
        return true;
    }
    return false;
}
