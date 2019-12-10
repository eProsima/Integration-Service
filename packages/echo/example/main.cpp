#include <soss/mock/api.hpp>

#include <soss/Instance.hpp>

#include <iostream>

#define LOG_PREFIX "[soss-echo-example]: "

int main()
{
    soss::InstanceHandle soss_handle = soss::run_instance("install/soss-echo/bin/config.yaml");
    if(soss_handle)
    {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(100ms);

        const soss::TypeRegistry& mock_types = *soss_handle.type_registry("mock");

        xtypes::DynamicData message_to(*mock_types.at("coordinate2d"));
        message_to["x"].value(2);
        message_to["y"].value(4);

        std::promise<xtypes::DynamicData> receive_msg_promise;
        soss::mock::subscribe("from_echo", [&](const xtypes::DynamicData& msg_from_echo)
        {
            receive_msg_promise.set_value(msg_from_echo);
        });

        if(!soss::mock::publish_message("to_echo", message_to))
        {
            std::cerr << LOG_PREFIX "Error sending message" << std::endl;
            return 1;
        }

        auto receive_msg_future = receive_msg_promise.get_future();
        if(std::future_status::ready != receive_msg_future.wait_for(2s))
        {
            std::cerr << LOG_PREFIX "Error receiving message" << std::endl;
            return 2;
        }

        xtypes::DynamicData message_from = receive_msg_future.get();
        std::cout << LOG_PREFIX "Message received" << std::endl;
        if(message_to != message_from)
        {
            std::cerr << LOG_PREFIX "Error comparing message" << std::endl;
            return 3;
        }

        std::cout << LOG_PREFIX "Message is correct!" << std::endl;

        soss_handle.quit().wait_for(2s);
    }

    return 0;
}
