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
 * @file HelloWorldSubscriber.cpp
 *
 */

#include "HelloWorldSubscriber.h"
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>

#include <fastrtps/types/DynamicDataHelper.hpp>
#include <fastrtps/types/DynamicDataFactory.h>

using namespace eprosima::fastdds::dds;

HelloWorldSubscriber::HelloWorldSubscriber()
    : participant_(nullptr)
    , subscriber_(nullptr)
    , listener_(this)
{
}

bool HelloWorldSubscriber::init(
        const eprosima::fastdds::dds::DomainId_t domain_id,
        const std::string& topic_name)
{
    //Do not enable entities on creation
    DomainParticipantFactoryQos factory_qos;
    factory_qos.entity_factory().autoenable_created_entities = false;
    DomainParticipantFactory::get_instance()->set_qos(factory_qos);

    DomainParticipantQos pqos;
    pqos.name("Participant_sub");
    StatusMask par_mask = StatusMask::subscription_matched() << StatusMask::data_available();
    participant_ = DomainParticipantFactory::get_instance()->create_participant(domain_id, pqos, &listener_, par_mask);

    if (participant_ == nullptr)
    {
        return false;
    }

    if (participant_->enable() != ReturnCode_t::RETCODE_OK)
    {
        DomainParticipantFactory::get_instance()->delete_participant(participant_);
        return false;
    }

    // CREATE THE COMMON READER ATTRIBUTES
    qos_ = DATAREADER_QOS_DEFAULT;
    qos_.reliability().kind = RELIABLE_RELIABILITY_QOS;

    return true;
}

HelloWorldSubscriber::~HelloWorldSubscriber()
{
    for (const auto& it : topics_)
    {
        subscriber_->delete_datareader(it.first);
        participant_->delete_topic(it.second);
    }
    if (subscriber_ != nullptr)
    {
        participant_->delete_subscriber(subscriber_);
    }

    DomainParticipantFactory::get_instance()->delete_participant(participant_);
    topics_.clear();
    readers_.clear();
    datas_.clear();
}

void HelloWorldSubscriber::SubListener::on_subscription_matched(
        DataReader*,
        const SubscriptionMatchedStatus& info)
{
    if (info.current_count_change == 1)
    {
        matched_ = info.total_count;
        std::cout << "Subscriber matched." << std::endl;
    }
    else if (info.current_count_change == -1)
    {
        matched_ = info.total_count;
        std::cout << "Subscriber unmatched." << std::endl;
    }
    else
    {
        std::cout << info.current_count_change
                  << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
    }
}

void HelloWorldSubscriber::SubListener::on_data_available(
        DataReader* reader)
{
    auto dit = h_subscriber->datas_.find(reader);

    if (dit != h_subscriber->datas_.end())
    {
        eprosima::fastrtps::types::DynamicData_ptr data = dit->second;
        SampleInfo info;
        if (reader->take_next_sample(data.get(), &info) == ReturnCode_t::RETCODE_OK)
        {
#if (FASTRTPS_VERSION_MAJOR == 2 && FASTRTPS_VERSION_MINOR < 2)
            if (info.instance_state == ALIVE)
#else
            if (info.instance_state == ALIVE_INSTANCE_STATE)
#endif //  if (FASTRTPS_VERSION_MAJOR == 2 && FASTRTPS_VERSION_MINOR < 2)
            {
                eprosima::fastrtps::types::DynamicType_ptr type = h_subscriber->readers_[reader];
                samples_++;
                std::cout << "Received data of type " << type->get_name() << std::endl;
                eprosima::fastrtps::types::DynamicDataHelper::print(data);
            }
        }
    }
}

void HelloWorldSubscriber::SubListener::on_type_discovery(
        DomainParticipant*,
        const eprosima::fastrtps::rtps::SampleIdentity&,
        const eprosima::fastrtps::string_255& topic_name,
        const eprosima::fastrtps::types::TypeIdentifier*,
        const eprosima::fastrtps::types::TypeObject*,
        eprosima::fastrtps::types::DynamicType_ptr dyn_type)
{
    TypeSupport m_type(new eprosima::fastrtps::types::DynamicPubSubType(dyn_type));
    m_type.register_type(h_subscriber->participant_);

    std::cout << "Discovered type: " << m_type->getName() << " from topic " << topic_name << std::endl;

    if (h_subscriber->subscriber_ == nullptr)
    {
        h_subscriber->subscriber_ = h_subscriber->participant_->create_subscriber(
            SUBSCRIBER_QOS_DEFAULT, nullptr);

        if (h_subscriber->subscriber_ == nullptr)
        {
            return;
        }
    }

    //CREATE THE TOPIC
    Topic* topic = h_subscriber->participant_->create_topic(
        "rt/hello_ros2",
        m_type->getName(),
        TOPIC_QOS_DEFAULT);

    if (topic == nullptr)
    {
        return;
    }

    StatusMask sub_mask = StatusMask::subscription_matched() << StatusMask::data_available();
    DataReader* reader = h_subscriber->subscriber_->create_datareader(
        topic,
        h_subscriber->qos_,
        &h_subscriber->listener_,
        sub_mask);

    h_subscriber->topics_[reader] = topic;
    h_subscriber->readers_[reader] = dyn_type;
    eprosima::fastrtps::types::DynamicData_ptr data(
        eprosima::fastrtps::types::DynamicDataFactory::get_instance()->create_data(dyn_type));
    h_subscriber->datas_[reader] = data;
}

void HelloWorldSubscriber::run()
{
    std::cout << "Subscriber running. Please press enter to stop the Subscriber" << std::endl;
    std::cin.ignore();
}

void HelloWorldSubscriber::run(
        uint32_t number)
{
    std::cout << "Subscriber running until " << number << "samples have been received" << std::endl;
    while (number > listener_.samples_)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}
