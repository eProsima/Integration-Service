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

using eprosima::fastdds::dds::Log;

static struct option options[] =
{
    { "mode",       required_argument, 0, 'm' },
    { "domain",     required_argument, 0, 'd' },
    { "count",      required_argument, 0, 'c' },
    { "topic_name", required_argument, 0, 'n'},
    { "help",       no_argument, 0, 'h' }
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

    help << "Usage: DDSHelloWorld ";
    help << "-m/--mode <publisher/subscriber> ";
    help << "-d/--domain <UNSIGNED_INTEGER> ";
    help << "-c/--count <UNSIGNED_INTEGER> ";
    help << "-n/--topic_name <STRING>";
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
    std::string topic_name("DDSHelloWorld");
    const uint32_t sleep = 100;
    eprosima::fastdds::dds::DomainId_t domain_id(0);

    while (true)
    {
        int option_index = 0;
        auto opt = getopt_long(argc, argv, "m:d:c:n:h", options, &option_index);

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
            case 'h':
            {
                std::cout << usage() << std::endl;
                std::cout << "\t-m/--mode\tChoose between 'publisher' or 'subscriber'" << std::endl;
                std::cout << "\t-d/--domain\t(optional) Set a custom Domain ID (default: 0)" << std::endl;
                std::cout << "\t-c/--count\t(optional) Publish a specific number of messages (default: 10)" << std::endl;
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
            if (mypub.init(domain_id, topic_name))
            {
                mypub.run(count, sleep);
            }
            break;
        }
        case OperationMode::SUBSCRIBE:
        {
            HelloWorldSubscriber mysub;
            if (mysub.init(domain_id, topic_name))
            {
                mysub.run();
            }
            break;
        }
    }
    Log::Reset();
    return 0;
}
