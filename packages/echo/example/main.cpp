#include <is/sh/mock/api.hpp>

#include <is/core/Instance.hpp>

#include <iostream>

#define LOG_PREFIX "[is-echo-example]: "

namespace is = eprosima::is;
namespace xtypes = eprosima::xtypes;

int main()
{
    is::core::InstanceHandle is_handle = is::run_instance("install/is-echo/bin/config.yaml");
    if (is_handle)
    {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(100ms);

        const is::TypeRegistry& mock_types = *is_handle.type_registry("mock");

        xtypes::DynamicData message_to(*mock_types.at("coordinate2d"));
        message_to["x"].value(2);
        message_to["y"].value(4);

        std::promise<xtypes::DynamicData> receive_msg_promise;
        is::sh::mock::subscribe("from_echo", [&](const xtypes::DynamicData& msg_from_echo)
                {
                    receive_msg_promise.set_value(msg_from_echo);
                });

        if (!is::sh::mock::publish_message("to_echo", message_to))
        {
            std::cerr << LOG_PREFIX "Error sending message" << std::endl;
            return 1;
        }

        auto receive_msg_future = receive_msg_promise.get_future();
        if (std::future_status::ready != receive_msg_future.wait_for(2s))
        {
            std::cerr << LOG_PREFIX "Error receiving message" << std::endl;
            return 2;
        }

        xtypes::DynamicData message_from = receive_msg_future.get();
        std::cout << LOG_PREFIX "Message received" << std::endl;
        if (message_to != message_from)
        {
            std::cerr << LOG_PREFIX "Error comparing message" << std::endl;
            return 3;
        }

        std::cout << LOG_PREFIX "Message is correct!" << std::endl;

        is_handle.quit().wait_for(2s);
    }

    return 0;
}
