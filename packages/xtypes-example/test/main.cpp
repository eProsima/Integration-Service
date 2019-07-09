#include <soss/mock/api.hpp>
#include <soss/Instance.hpp>

#include <iostream>

/*
soss::MessageData roundtrip(
        const std::string& fiware_entity,
        const std::string& topic_type,
        const soss::MessageData& message)
{
    using namespace std::chrono_literals;

    const std::string topic_sent = "mock_to_fiware_topic";
    const std::string topic_recv = "fiware_to_mock_topic";

    std::string config_yaml = gen_config_yaml(topic_type, fiware_entity, topic_sent, topic_recv);
    const YAML::Node config_node = YAML::Load(config_yaml);
    soss::InstanceHandle soss_handle = soss::run_instance(config_node);
    REQUIRE(soss_handle);

    std::this_thread::sleep_for(1s); //skip the first message from fiware, possibily from previous tests

    soss::mock::publish_message(topic_sent, message);

    std::promise<soss::Message> receive_msg_promise;
    REQUIRE(soss::mock::subscribe(
            topic_recv,
            [&](const soss::Message& msg_from_fiware)
    {
        receive_msg_promise.set_value(msg_from_fiware);
    }));

    auto receive_msg_future = receive_msg_promise.get_future();
    REQUIRE(std::future_status::ready == receive_msg_future.wait_for(5s));

    REQUIRE(0 == soss_handle.quit().wait_for(1s));
    return receive_msg_future.get();
}*/

int main()
{
    soss::InstanceHandle soss_handle = soss::run_instance("install/soss-xtypes-example/bin/test_config.yaml");
    if(soss_handle)
    {
        std::cout << "Loaded successfully" << std::endl;
    }
}
