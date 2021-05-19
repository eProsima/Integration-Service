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

#include "AddTwoInts_server.hpp"

#include <stdexcept>

#include <unistd.h>
#include <getopt.h>

static struct option options[] =
{
    { "port",         required_argument, 0, 'p' },
    { "service_name", required_argument, 0, 'n' },
    { "help",         no_argument, 0, 'h' }
};

const std::string usage()
{
    std::ostringstream help;

    help << "Usage: WebSocketAddTwoInts ";
    help << "-p/--port <UNSIGNED_INTEGER> ";
    help << "-n/--service_name <STRING>";
    return help.str();
}

int main(
        int argc,
        char** argv)
{
    uint16_t port = 80;
    std::string service_name("add_two_ints");

    while (true)
    {
        int option_index = 0;
        auto opt = getopt_long(argc, argv, "p:n:h", options, &option_index);

        if (-1 == opt)
        {
            // Reached last argument. Finish loop
            break;
        }

        switch (opt)
        {
            case 'p':
            {
                int raw_port = atoi(optarg);
                if (raw_port < 0 || raw_port > 65535)
                {
                    throw std::invalid_argument("Service port parameter must be a positive uint16 value");
                }

                port = static_cast<uint16_t>(raw_port);
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
                std::cout << "\t-p/--port\t(optional) Specify a WebSocket server port (default: 80)" << std::endl;
                std::cout << "\t-n/--service_name\t(optional) Request/reply to/from"
                          << " a specific service (default: add_two_ints)" << std::endl;
                return 0;
            }
            default:
            {
                std::cout << usage() << std::endl;
                return 1;
            }
        }
    }

    AddTwoIntsServer(service_name, port);

    return 0;
}
