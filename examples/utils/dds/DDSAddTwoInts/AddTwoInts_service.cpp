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

#include "AddTwoInts_service.hpp"

#include <ctime>
#include <chrono>


AddTwoInts_Service::AddTwoInts_Service(
        const std::string& service_name,
        const fastdds::dds::DomainId_t& domain_id,
        bool server)
    : fastdds::dds::DataWriterListener()
    , service_name_(service_name)
    , participant_factory_(fastdds::dds::DomainParticipantFactory::get_instance())
    , tsType_req_(new AddTwoInts_RequestPubSubType())
    , tsType_rep_(new AddTwoInts_ResponsePubSubType())
    , server_(server)
    , stop_(false)
    , matched_mutex_()
    , mutex_()
{
    mutex_.lock();
    // Create participant
    fastdds::dds::DomainParticipantQos participant_qos = fastdds::dds::PARTICIPANT_QOS_DEFAULT;

    participant_qos.name(server ? (service_name + "_Server") : (service_name + "_Client"));

    participant_ = participant_factory_->create_participant(domain_id, participant_qos);

    if (!participant_)
    {
        std::ostringstream err;
        err << "Error while creating participant '" << participant_qos.name()
            << "' with default QoS attributes and Domain ID: " << domain_id;

        throw DDSMiddlewareException(err.str());
    }

    // Register types
    participant_->register_type(tsType_req_);
    participant_->register_type(tsType_rep_);

    // Create publisher entities
    publisher_ = participant_->create_publisher(fastdds::dds::PUBLISHER_QOS_DEFAULT);

    if (!publisher_)
    {
        throw DDSMiddlewareException("Publisher was not created");
    }

    pub_topic_ = participant_->create_topic(
        server ? (service_name_ + "_Reply") : (service_name_ + "_Request"),
        server ? tsType_rep_.get_type_name() : tsType_req_.get_type_name(),
        fastdds::dds::TOPIC_QOS_DEFAULT);

    if (!pub_topic_)
    {
        throw DDSMiddlewareException("Error creating publisher topic");
    }

    datawriter_ = publisher_->create_datawriter(
        pub_topic_, fastdds::dds::DATAWRITER_QOS_DEFAULT, this);

    if (!datawriter_)
    {
        std::ostringstream err;
        err << "Creating datawriter for topic '" << pub_topic_->get_name() << "' failed";

        throw DDSMiddlewareException(err.str());
    }

    if (server)
    {
        request_listener.publisher(this, datawriter_);
    }
    else
    {
        reply_listener.publisher(this);
    }

    // Create subscriber entities
    subscriber_ = participant_->create_subscriber(fastdds::dds::SUBSCRIBER_QOS_DEFAULT);

    if (!subscriber_)
    {
        throw DDSMiddlewareException("Subscriber was not created");
    }

    sub_topic_ = participant_->create_topic(
        server ? (service_name_ + "_Request") : (service_name_ + "_Reply"),
        server ? tsType_req_.get_type_name() : tsType_rep_.get_type_name(),
        fastdds::dds::TOPIC_QOS_DEFAULT);

    if (!sub_topic_)
    {
        throw DDSMiddlewareException("Creating subscriber topic failed");
    }

    fastdds::dds::DataReaderQos datareader_qos = fastdds::dds::DATAREADER_QOS_DEFAULT;
    fastdds::dds::ReliabilityQosPolicy rel_policy;
    rel_policy.kind = fastdds::dds::RELIABLE_RELIABILITY_QOS;
    datareader_qos.reliability(rel_policy);

    datareader_ = subscriber_->create_datareader(
        sub_topic_, datareader_qos,
        server ? static_cast<fastdds::dds::DataReaderListener*>(&request_listener)
        : static_cast<fastdds::dds::DataReaderListener*>(&reply_listener));

    if (!datareader_)
    {
        std::ostringstream err;
        err << "Creating datareader for topic '" << sub_topic_->get_name() << "' failed";

        throw DDSMiddlewareException(err.str());
    }

    std::cout << service_name_ << " " << (server_ ? "server" : "client")
              << " running under DDS Domain ID: " << domain_id << std::endl;
}

AddTwoInts_Service::~AddTwoInts_Service()
{
    // Unregister types
    participant_->unregister_type(tsType_req_.get_type_name());
    participant_->unregister_type(tsType_rep_.get_type_name());

    // Delete subscriber entities
    subscriber_->delete_datareader(datareader_);
    participant_->delete_subscriber(subscriber_);
    participant_->delete_topic(sub_topic_);

    // Delete publisher entities
    publisher_->delete_datawriter(datawriter_);
    participant_->delete_publisher(publisher_);
    participant_->delete_topic(pub_topic_);

    // Delete participant
    participant_factory_->delete_participant(participant_);
}

std::future<AddTwoInts_Response> AddTwoInts_Service::request(
        AddTwoInts_Request& request)
{
    if (!server_)
    {
        std::unique_lock<std::mutex> lock(mutex_);

        datawriter_->write(static_cast<void*>(&request));
        delete promise_;
        promise_ = new std::promise<AddTwoInts_Response>();

        return promise_->get_future();
    }
    else
    {
        std::ostringstream err;
        err << "Warning: cannot perform a request from a " << service_name_ << " server!" << std::endl;
        throw DDSMiddlewareException(err.str());
    }
}

void AddTwoInts_Service::run(
        const uint32_t number,
        const uint32_t sleep)
{
    mutex_.lock();

    if (!server_)
    {
        stop_ = false;
        std::thread thread(&AddTwoInts_Service::runThread, this, number, sleep);
        if (0 == number)
        {
            std::cout << service_name_ << " client running. Please press enter to stop it at any time."
                      << std::endl;

            std::cin.ignore();
            stop_ = true;
        }
        else
        {
            std::cout << service_name_ << " client performing " << number << " requests." << std::endl;
        }
        thread.join();
    }
    else
    {
        std::cout << service_name_ << " server running. Please press enter to stop it at any time." << std::endl;
        std::cin.ignore();
    }
}

void AddTwoInts_Service::on_publication_matched(
        fastdds::dds::DataWriter* /*writer*/,
        const fastdds::dds::PublicationMatchedStatus& /*info*/)
{
    std::unique_lock<std::mutex> lock(matched_mutex_);
    pub_sub_matched_++;
    if (2 == pub_sub_matched_)
    {
        mutex_.unlock();
    }
}

void AddTwoInts_Service::runThread(
        const uint32_t samples,
        const uint32_t sleep)
{
    mutex_.unlock();

    if (samples == 0)
    {
        int64_t a = 0;
        while (!stop_)
        {
            AddTwoInts_Request request;
            a = a % std::numeric_limits<int64_t>::max() - 2;
            request.a(a);
            request.b(a + 2);

            std::cout << service_name_ << " client:" << std::endl;
            std::cout << "\t- Request " << request.a()
                      << " + " << request.b() << std::endl;

            auto response = this->request(request);
            auto ready = response.wait_for(std::chrono::seconds(5));

            if (std::future_status::ready != ready)
            {
                std::cout << "\t- Error: client exceeded maximum time to wait for a response!" << std::endl;
            }
            else
            {
                std::cout << "\t - Received response: " << response.get().sum() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep));
            }
        }
    }
    else
    {
        for (int64_t i = 0; i < samples; ++i)
        {
            AddTwoInts_Request request;
            request.a(i + 1);
            request.b(i + 3);

            std::cout << service_name_ << " client:" << std::endl;
            std::cout << "\t - Request " << request.a()
                      << " + " << request.b() << std::endl;

            auto response = this->request(request);
            auto ready = response.wait_for(std::chrono::seconds(5));

            if (std::future_status::ready != ready)
            {
                std::cout << "\t - Error: client exceeded maximum time to wait for a response!" << std::endl;
            }
            else
            {
                std::cout << "\t - Received response: " << response.get().sum() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep));
            }
        }
    }
}

void AddTwoInts_Service::RequestListener::publisher(
        AddTwoInts_Service* service,
        fastdds::dds::DataWriter* datawriter)
{
    service_ = service;
    datawriter_ = datawriter;
}

void AddTwoInts_Service::RequestListener::on_subscription_matched(
        fastdds::dds::DataReader* /*datareader*/,
        const fastdds::dds::SubscriptionMatchedStatus& info)
{
    std::unique_lock<std::mutex> lock(service_->matched_mutex_);
    service_->pub_sub_matched_++;
    if (service_->pub_sub_matched_ == 2)
    {
        service_->mutex_.unlock();
    }
}

void AddTwoInts_Service::RequestListener::on_data_available(
        fastdds::dds::DataReader* reader)
{
    fastdds::dds::SampleInfo info;
    AddTwoInts_Request request;
    AddTwoInts_Response response;

    if (fastrtps::types::ReturnCode_t::RETCODE_OK ==
            reader->take_next_sample(&request, &info))
    {
#if FASTRTPS_VERSION_MINOR < 2
        if (fastdds::dds::InstanceStateKind::ALIVE == info.instance_state)
#else
        if (fastdds::dds::InstanceStateKind::ALIVE_INSTANCE_STATE == info.instance_state)
#endif //  if FASTRTPS_VERSION_MINOR < 2
        {
            fastrtps::rtps::WriteParams params;
            params.related_sample_identity(info.sample_identity);

            std::cout << service_->service_name_ << " server: processing incoming request, add "
                      << request.a() << " + " << request.b() << std::endl;

            response.sum(request.a() + request.b());

            std::cout << "\t - Sending back response: " << response.sum() << std::endl;

            datawriter_->write(static_cast<void*>(&response), params);
        }
    }
}

void AddTwoInts_Service::ReplyListener::publisher(
        AddTwoInts_Service* service)
{
    service_ = service;
}

void AddTwoInts_Service::ReplyListener::on_subscription_matched(
        fastdds::dds::DataReader* /*datareader*/,
        const fastdds::dds::SubscriptionMatchedStatus& /*info*/)
{
    std::unique_lock<std::mutex> lock(service_->matched_mutex_);
    service_->pub_sub_matched_++;
    if (service_->pub_sub_matched_ == 2)
    {
        service_->mutex_.unlock();
    }
}

void AddTwoInts_Service::ReplyListener::on_data_available(
        fastdds::dds::DataReader* reader)
{
    std::unique_lock<std::mutex> lock(service_->mutex_);

    fastdds::dds::SampleInfo info;
    AddTwoInts_Response response;

    if (fastrtps::types::ReturnCode_t::RETCODE_OK ==
            reader->take_next_sample(&response, &info))
    {
#if FASTRTPS_VERSION_MINOR < 2
        if (fastdds::dds::InstanceStateKind::ALIVE == info.instance_state)
#else
        if (fastdds::dds::InstanceStateKind::ALIVE_INSTANCE_STATE == info.instance_state)
#endif //  if FASTRTPS_VERSION_MINOR < 2
        {
            service_->promise_->set_value(std::move(response));
        }

    }
}