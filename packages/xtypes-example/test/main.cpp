#include <soss/mock/api.hpp>
#include <soss/Instance.hpp>

#include <iostream>

int main()
{
    soss::InstanceHandle soss_handle = soss::run_instance("install/soss-xtypes-example/bin/test_config.yaml");
    if(soss_handle)
    {
        std::cout << "[soss-xtypes-example-test]: Initializing test..." << std::endl;

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(100ms);

        soss::MessageType coord_2d("coordinate2d");
        coord_2d["x"] = soss::MessageType::Type::INT;
        coord_2d["y"] = soss::MessageType::Type::INT;

        soss::MessageData message_to(coord_2d);
        message_to["x"] = "2";
        message_to["y"] = "4";

        std::cout << "[soss-xtypes-example-test]: Sending message..." << std::endl;
        if(!soss::mock::publish_message("to_xtypes", message_to))
        {
            std::cerr << "[soss-xtypes-example-test]: Error sending message" << std::endl;
            return 1;
        }

        std::cout << "[soss-xtypes-example-test]: Receiving message..." << std::endl;
        std::promise<soss::MessageData> receive_msg_promise;
        soss::mock::subscribe("from_xtypes", [&](const soss::MessageData& msg_from_xtypes)
        {
            receive_msg_promise.set_value(msg_from_xtypes);
        });

        auto receive_msg_future = receive_msg_promise.get_future();
        if(std::future_status::ready != receive_msg_future.wait_for(2s))
        {
            std::cerr << "[soss-xtypes-example-test]: Error receiving message" << std::endl;
            return 2;
        }

        soss::MessageData message_from = receive_msg_future.get();
        if(message_from != message_to)
        {
            std::cerr << "[soss-xtypes-example-test]: Error comparing message" << std::endl;
            return 3;
        }

        std::cout << "[soss-xtypes-example-test]: Exiting..." << std::endl;
        soss_handle.quit().wait_for(2s);

        return 0;
    }
}
