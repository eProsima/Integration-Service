// Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file HelloWorld_main.cpp
 *
 */

#include "HelloWorldPublisher.h"
#include "HelloWorldSubscriber.h"

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

#include <fastrtps/log/Log.h>

#include <stdexcept>

#include <unistd.h>
#include <getopt.h>

#define NS_TO_S(nanoseconds) (nanoseconds / (1000 * 1000 * 1000))

using namespace eprosima::fastdds::dds;

static struct option options[] =
{
    { "mode",                required_argument, 0, 'm' },
    { "domain",              required_argument, 0, 'd' },
    { "count",               required_argument, 0, 'c' },
    { "topic_name",          required_argument, 0, 'n' },
    { "durability",          required_argument, 0, 'v' },
    { "deadline",            required_argument, 0, 'p' },
    { "history_kind",        required_argument, 0, 'i' },
    { "history_depth",       required_argument, 0, 'a' },
    { "lifespan",            required_argument, 0, 's' },
    { "liveliness_kind",     required_argument, 0, 'l' },
    { "liveliness_duration", required_argument, 0, 'e' },
    { "reliability",         required_argument, 0, 'r' },
    { "help",                no_argument,       0, 'h' }
};

enum class OperationMode
{
    INVALID,
    PUBLISH,
    SUBSCRIBE
};

const std::string usage()
{
    std::ostringstream help;

    help << "Usage: DDSHelloWorld "  << std::endl;
    help << "-m/--mode <publisher/subscriber> " << std::endl;
    help << "-d/--domain <UNSIGNED_INTEGER> " << std::endl;
    help << "-c/--count <UNSIGNED_INTEGER> " << std::endl;
    help << "-n/--topic_name <STRING> " << std::endl;
    help << "--durability <volatile/transient_local> " << std::endl;
    help << "--deadline <UNSIGNED_INTEGER> (period in ms) " << std::endl;
    help << "--history_kind <keep_last/keep_all> " << std::endl;
    help << "--history_depth <UNSIGNED_INTEGER> " << std::endl;
    help << "--lifespan <UNSIGNED_INTEGER> (period in ms) " << std::endl;
    help << "--liveliness_kind <automatic/manual_by_topic> " << std::endl;
    help << "--liveliness_duration <UNSIGNED_INTEGER> (duration in ms) " << std::endl;
    help << "--reliability <reliable/best_effort> ";
    return help.str();
}

int main(
        int argc,
        char** argv)
{
    if (argc < 2)
    {
        std::cout << usage() << std::endl;
        return 1;
    }

    OperationMode mode(OperationMode::INVALID);
    uint32_t count = 10;
    std::string topic_name("HelloWorldTopic");
    const uint32_t sleep = 100;
    eprosima::fastdds::dds::DomainId_t domain_id(0);
    eprosima::fastdds::dds::DataWriterQos dw_qos(DATAWRITER_QOS_DEFAULT);
    eprosima::fastdds::dds::DataReaderQos dr_qos(DATAREADER_QOS_DEFAULT);
    dr_qos.reliability().kind = RELIABLE_RELIABILITY_QOS;

    while (true)
    {
        int option_index = 0;
        auto opt = getopt_long(argc, argv, "m:d:c:n:v:p:i:a:s:l:e:r:h", options, &option_index);

        if (-1 == opt)
        {
            // Reached last argument. Finish loop
            break;
        }

        switch (opt)
        {
            case 'm':
            {
                if (0 == strcmp("publisher", optarg))
                {
                    mode = OperationMode::PUBLISH;
                }
                else if (0 == strcmp("subscriber", optarg))
                {
                    mode = OperationMode::SUBSCRIBE;
                }

                if (OperationMode::INVALID == mode)
                {
                    throw std::invalid_argument("Invalid mode: please choose between 'publisher' or 'subscriber'");
                }
                break;
            }
            case 'd':
            {
                int raw_domain = atoi(optarg);
                if (raw_domain < 0)
                {
                    throw std::invalid_argument("Error while parsing provided arguments: Domain ID must be >= 0");
                }

                domain_id = static_cast<eprosima::fastdds::dds::DomainId_t>(raw_domain);
                break;
            }
            case 'c':
            {
                int raw_count = atoi(optarg);
                if (raw_count <= 0)
                {
                    throw std::invalid_argument("Topic publish count parameter must be a positive value");
                }

                count = static_cast<uint32_t>(raw_count);
                break;
            }
            case 'n':
            {
                topic_name.assign(optarg);
                break;
            }
            case 'v':
            {
                if (0 == strcmp("volatile", optarg))
                {
                    if (mode == OperationMode::PUBLISH)
                    {
                        dw_qos.durability().kind = VOLATILE_DURABILITY_QOS;
                    }
                    else if (mode == OperationMode::SUBSCRIBE)
                    {
                        dr_qos.durability().kind = VOLATILE_DURABILITY_QOS;
                    }
                }
                else if (0 == strcmp("transient_local", optarg))
                {
                    if (mode == OperationMode::PUBLISH)
                    {
                        dw_qos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
                    }
                    else if (mode == OperationMode::SUBSCRIBE)
                    {
                        dr_qos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
                    }
                }
                else
                {
                    throw std::invalid_argument("Invalid durability kind: please choose between 'volatile' or 'transient_local'");
                }
                break;
            }
            case 'p':
            {
                int period = atoi(optarg);
                if (period <= 0)
                {
                    throw std::invalid_argument("Deadline period must be a positive value");
                }

                if (mode == OperationMode::PUBLISH)
                {
                    dw_qos.deadline().period = period * 1e-3;
                }
                else if (mode == OperationMode::SUBSCRIBE)
                {
                    dr_qos.deadline().period = period * 1e-3;
                }
                break;
            }
            case 'i':
            {
                if (0 == strcmp("keep_last", optarg))
                {
                    if (mode == OperationMode::PUBLISH)
                    {
                        dw_qos.history().kind = KEEP_LAST_HISTORY_QOS;
                    }
                    else if (mode == OperationMode::SUBSCRIBE)
                    {
                        dr_qos.history().kind = KEEP_LAST_HISTORY_QOS;
                    }
                }
                else if (0 == strcmp("keep_all", optarg))
                {
                    if (mode == OperationMode::PUBLISH)
                    {
                        dw_qos.history().kind = KEEP_ALL_HISTORY_QOS;
                    }
                    else if (mode == OperationMode::SUBSCRIBE)
                    {
                        dr_qos.history().kind = KEEP_ALL_HISTORY_QOS;
                    }
                }
                else
                {
                    throw std::invalid_argument("Invalid history kind: please choose between 'keep_last' or 'keep_all'");
                }
                break;
            }
            case 'a':
            {
                int depth = atoi(optarg);
                if (depth <= 0)
                {
                    throw std::invalid_argument("History depth must be a positive value");
                }

                if (mode == OperationMode::PUBLISH)
                {
                    dw_qos.history().depth = depth;
                }
                else if (mode == OperationMode::SUBSCRIBE)
                {
                    dr_qos.history().depth = depth;
                }
                break;
            }
            case 's':
            {
                int duration = atoi(optarg);
                if (duration <= 0)
                {
                    throw std::invalid_argument("Lifespan duration must be a positive value");
                }

                if (mode == OperationMode::PUBLISH)
                {
                    dw_qos.lifespan().duration = duration * 1e-3;
                }
                else if (mode == OperationMode::SUBSCRIBE)
                {
                    dr_qos.lifespan().duration = duration * 1e-3;
                }
                break;
            }
            case 'l':
            {
                if (0 == strcmp("automatic", optarg))
                {
                    if (mode == OperationMode::PUBLISH)
                    {
                        dw_qos.liveliness().kind = AUTOMATIC_LIVELINESS_QOS;
                    }
                    else if (mode == OperationMode::SUBSCRIBE)
                    {
                        dr_qos.liveliness().kind = AUTOMATIC_LIVELINESS_QOS;
                    }
                }
                else if (0 == strcmp("manual_by_topic", optarg))
                {
                    if (mode == OperationMode::PUBLISH)
                    {
                        dw_qos.liveliness().kind = MANUAL_BY_TOPIC_LIVELINESS_QOS;
                    }
                    else if (mode == OperationMode::SUBSCRIBE)
                    {
                        dr_qos.liveliness().kind = MANUAL_BY_TOPIC_LIVELINESS_QOS;
                    }
                }
                else
                {
                    throw std::invalid_argument("Invalid liveliness kind: please choose between 'automatic' or 'manual_by_topic'");
                }
                break;
            }
            case 'e':
            {
                int lease_duration = atoi(optarg);
                if (lease_duration <= 0)
                {
                    throw std::invalid_argument("Liveliness lease duration must be a positive value");
                }

                double ns = lease_duration * 2.0 / 3.0;
                double s = NS_TO_S(ns);

                if (mode == OperationMode::PUBLISH)
                {
                    dw_qos.liveliness().lease_duration = lease_duration * 1e-3;
                    dw_qos.liveliness().announcement_period = s;
                }
                else if (mode == OperationMode::SUBSCRIBE)
                {
                    dr_qos.liveliness().lease_duration = lease_duration * 1e-3;
                    dr_qos.liveliness().announcement_period = s;
                }
                break;
            }
            case 'h':
            {
                std::cout << usage() << std::endl;
                std::cout << "\t-m/--mode\tChoose between 'publisher' or 'subscriber'" << std::endl;
                std::cout << "\t-d/--domain\t(optional) Set a custom Domain ID (default: 0)" << std::endl;
                std::cout << "\t-c/--count\t(optional) Publish a specific number of messages (default: 10)" << std::endl;
                std::cout << "\t-n/--topic_name\t(optional) Publish or subscribe to a specific topic (default: HelloWorldTopic)" << std::endl;
                std::cout << "\t--durability\t(optional) Durability kind ('volatile' or 'transient_local')" << std::endl;
                std::cout << "\t--deadline\t(optional) Deadline period in ms" << std::endl;
                std::cout << "\t--history_kind\t(optional) History kind ('keep_last' or 'keep_all')" << std::endl;
                std::cout << "\t--history_depth\t(optional) History depth" << std::endl;
                std::cout << "\t--lifespan\t(optional) Lifespan period in ms" << std::endl;
                std::cout << "\t--liveliness_kind\t(optional) Liveliness kind ('automatic' or 'manual_by_topic')" << std::endl;
                std::cout << "\t--liveliness_duration\t(optional) Liveliness lease duration in ms" << std::endl;
                std::cout << "\t--reliability\t(optional) Reliability kind ('reliable' or 'best_effort')" << std::endl;
                return 0;
            }
            default:
            {
                std::cout << usage() << std::endl;
                return 1;
            }
        }
    }

    switch (mode)
    {
        case OperationMode::INVALID:
        {
            std::cout << usage() << std::endl;
            return 1;
        }
        case OperationMode::PUBLISH:
        {
            HelloWorldPublisher mypub;
            if (mypub.init(domain_id, topic_name, dw_qos))
            {
                mypub.run(count, sleep);
            }
            break;
        }
        case OperationMode::SUBSCRIBE:
        {
            HelloWorldSubscriber mysub;
            if (mysub.init(domain_id, topic_name, dr_qos))
            {
                mysub.run();
            }
            break;
        }
    }
    Log::Reset();
    return 0;
}
