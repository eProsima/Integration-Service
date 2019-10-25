#include <soss/mock/api.hpp>

#include <soss/Instance.hpp>

#include <iostream>

#define LOG_PREFIX "[soss-local-example-test]: "

int main()
{
    soss::InstanceHandle soss_handle = soss::run_instance("install/soss-local-example/bin/test_config.yaml");
    if(soss_handle)
    {
        std::cout << LOG_PREFIX "Initializing test..." << std::endl;

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(100ms);

        const soss::TypeRegistry& mock_types = *soss_handle.type_registry("mock");

        xtypes::DynamicData message_to(*mock_types.at("coordinate2d"));
        message_to["x"].value(2u);
        message_to["y"].value(4u);

        std::promise<xtypes::DynamicData> receive_msg_promise;
        soss::mock::subscribe("from_middleware", [&](const xtypes::DynamicData& msg_from_middleware)
        {
            receive_msg_promise.set_value(msg_from_middleware);
        });

        std::cout << LOG_PREFIX "Sending message..." << std::endl;
        if(!soss::mock::publish_message("to_middleware", message_to))
        {
            std::cerr << LOG_PREFIX "Error sending message" << std::endl;
            return 1;
        }

        std::cout << LOG_PREFIX "Receiving message..." << std::endl;
        auto receive_msg_future = receive_msg_promise.get_future();
        if(std::future_status::ready != receive_msg_future.wait_for(2s))
        {
            std::cerr << LOG_PREFIX "Error receiving message" << std::endl;
            return 2;
        }

        xtypes::DynamicData message_from = receive_msg_future.get();
        std::cout << LOG_PREFIX "Message received!" << std::endl;
        if(message_to != message_from)
        {
            std::cerr << LOG_PREFIX "Error comparing message" << std::endl;
            return 3;
        }

        std::cout << LOG_PREFIX "Message is correct!" << std::endl;

        std::cout << LOG_PREFIX "Exiting..." << std::endl;
        soss_handle.quit().wait_for(2s);
    }

    return 0;
}
