// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef _IS_EXAMPLES_UTILS_DDS__ADDTWOINTS_SERVICE_HPP_
#define _IS_EXAMPLES_UTILS_DDS__ADDTWOINTS_SERVICE_HPP_

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>

#include "AddTwoIntsPubSubTypes.h"

#include <future>

using namespace eprosima;

class DDSMiddlewareException : public std::runtime_error
{
public:

    DDSMiddlewareException(
            const std::string& message)
        : std::runtime_error(message)
    {
    }

};

class AddTwoInts_Service
    : public fastdds::dds::DataWriterListener
{
public:

    AddTwoInts_Service(
            const std::string& service_name,
            const fastdds::dds::DomainId_t& domain_id,
            bool server);

    ~AddTwoInts_Service();

    // Perform a request
    std::future<AddTwoInts_Response> request(
            AddTwoInts_Request& request);

    // Request a specific number of times
    void run(
            const uint32_t number,
            const uint32_t sleep);

    void on_publication_matched(
            fastdds::dds::DataWriter* /*writer*/,
            const fastdds::dds::PublicationMatchedStatus& /*info*/) override;

private:

    void runThread(
            const uint32_t number,
            const uint32_t sleep);

    const std::string service_name_;

    fastdds::dds::DomainParticipantFactory* participant_factory_;
    fastdds::dds::DomainParticipant* participant_;

    fastdds::dds::Subscriber* subscriber_;
    fastdds::dds::Topic* sub_topic_;
    fastdds::dds::DataReader* datareader_;

    fastdds::dds::Publisher* publisher_;
    fastdds::dds::Topic* pub_topic_;
    fastdds::dds::DataWriter* datawriter_;

    fastdds::dds::TypeSupport tsType_req_;
    fastdds::dds::TypeSupport tsType_rep_;

    std::promise<AddTwoInts_Response>* promise_ = nullptr;
    bool server_;
    bool stop_;
    std::mutex matched_mutex_;
    std::mutex mutex_;
    uint32_t pub_sub_matched_ = 0;

    class RequestListener : public fastdds::dds::DataReaderListener
    {
    public:

        RequestListener() = default;

        void publisher(
                AddTwoInts_Service* service,
                fastdds::dds::DataWriter* datawriter);

        void on_subscription_matched(
                fastdds::dds::DataReader* /*datareader*/,
                const fastdds::dds::SubscriptionMatchedStatus& info) override;

        void on_data_available(
                fastdds::dds::DataReader* reader) override;

    private:

        fastdds::dds::DataWriter* datawriter_;
        AddTwoInts_Service* service_;
    };

    RequestListener request_listener;

    class ReplyListener : public fastdds::dds::DataReaderListener
    {
    public:

        void publisher(
                AddTwoInts_Service* service);

        void on_subscription_matched(
                fastdds::dds::DataReader* /*datareader*/,
                const fastdds::dds::SubscriptionMatchedStatus& /*info*/) override;

        void on_data_available(
                fastdds::dds::DataReader* reader) override;

    private:

        AddTwoInts_Service* service_;
    };

    ReplyListener reply_listener;
};

#endif //  _IS_EXAMPLES_UTILS_DDS__ADDTWOINTS_SERVICE_HPP_