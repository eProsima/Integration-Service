#include <soss/mock/api.hpp>

#include <dds/core/xtypes/StructType.hpp>
#include <dds/core/xtypes/PrimitiveTypes.hpp>

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

        dds::core::xtypes::StructType coord_2d("coordinate2d");
        coord_2d.add_member(dds::core::xtypes::Member("x", dds::core::xtypes::primitive_type<uint32_t>()));
        coord_2d.add_member(dds::core::xtypes::Member("y", dds::core::xtypes::primitive_type<uint32_t>()));

        dds::core::xtypes::DynamicData message_to(coord_2d);
        message_to.value("x", 2);
        message_to.value("y", 4);

        std::promise<dds::core::xtypes::DynamicData> receive_msg_promise;
        soss::mock::subscribe("from_xtypes", [&](const dds::core::xtypes::DynamicData& msg_from_xtypes)
        {
            receive_msg_promise.set_value(msg_from_xtypes);
        });

        std::cout << "[soss-xtypes-example-test]: Sending message..." << std::endl;
        if(!soss::mock::publish_message("to_xtypes", message_to))
        {
            std::cerr << "[soss-xtypes-example-test]: Error sending message" << std::endl;
            return 1;
        }

        std::cout << "[soss-xtypes-example-test]: Receiving message..." << std::endl;
        auto receive_msg_future = receive_msg_promise.get_future();
        if(std::future_status::ready != receive_msg_future.wait_for(2s))
        {
            std::cerr << "[soss-xtypes-example-test]: Error receiving message" << std::endl;
            return 2;
        }

        dds::core::xtypes::DynamicData message_from = receive_msg_future.get();
        if(message_to.value<uint32_t>("x") != message_from.value<uint32_t>("x")
        || message_to.value<uint32_t>("y") != message_from.value<uint32_t>("y"))
        {
            std::cerr << "[soss-xtypes-example-test]: Error comparing message" << std::endl;
            return 3;
        }

        std::cout << "[soss-xtypes-example-test]: Message received!" << std::endl;

        std::cout << "[soss-xtypes-example-test]: Exiting..." << std::endl;
        soss_handle.quit().wait_for(2s);
    }

    return 0;
}
