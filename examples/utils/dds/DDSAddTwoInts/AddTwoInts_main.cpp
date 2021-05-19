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

/**
 * @file AddTwoInts_main.cpp
 *
 */

#include "AddTwoInts_service.hpp"

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

#include <fastrtps/log/Log.h>

#include <stdexcept>

#include <unistd.h>
#include <getopt.h>

using eprosima::fastdds::dds::Log;

static struct option options[] =
{
    { "mode",         required_argument, 0, 'm' },
    { "domain",       required_argument, 0, 'd' },
    { "count",        required_argument, 0, 'c' },
    { "service_name", required_argument, 0, 'n'},
    { "help",         no_argument, 0, 'h' }
};

enum class OperationMode
{
    INVALID,
    SERVER,
    CLIENT
};

const std::string usage()
{
    std::ostringstream help;

    help << "Usage: DDSAddTwoInts ";
    help << "-m/--mode <server/client> ";
    help << "-d/--domain <UNSIGNED_INTEGER> ";
    help << "-c/--count <UNSIGNED_INTEGER> ";
    help << "-n/--service_name <STRING>";
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
    std::string service_name("AddTwoIntsService");
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
                if (0 == strcmp("server", optarg))
                {
                    mode = OperationMode::SERVER;
                }
                else if (0 == strcmp("client", optarg))
                {
                    mode = OperationMode::CLIENT;
                }

                if (OperationMode::INVALID == mode)
                {
                    throw std::invalid_argument("Invalid mode: please choose between 'server' or 'client'");
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
                    throw std::invalid_argument("Service client request count parameter must be a positive value");
                }

                count = static_cast<uint32_t>(raw_count);
                break;
            }
            case 'n':
            {
                service_name.assign(optarg);
                break;
            }
            case 'h':
            {
                std::cout << usage() << std::endl;
                std::cout << "\t-m/--mode\tChoose between 'server' or 'client'" << std::endl;
                std::cout << "\t-d/--domain\t(optional) Set a custom Domain ID (default: 0)" << std::endl;
                std::cout << "\t-c/--count\t(optional) Make a specific number of service client requests (default: 10)" << std::endl;
                std::cout << "\t-n/--service_name\t(optional) Request/reply to/from a specific service (default: AddTwoIntsService)" << std::endl;
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
        case OperationMode::SERVER:
        case OperationMode::CLIENT:

        {
            try
            {
                AddTwoInts_Service service(
                    service_name,
                    domain_id,
                    mode == OperationMode::SERVER ? true : false);

                service.run(count, sleep);
            }
            catch(const DDSMiddlewareException& e)
            {
                std::cerr << e.what() << std::endl;
            }

            break;
        }
    }

    Log::Reset();
    return 0;
}
