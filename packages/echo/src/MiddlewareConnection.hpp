#ifndef SOSS__XTYPES_EXAMPLE__INTERNAL__MIDDLEWARE_CONNECTION_HPP
#define SOSS__XTYPES_EXAMPLE__INTERNAL__MIDDLEWARE_CONNECTION_HPP

#include <soss/json/conversion.hpp>

#include <map>
#include <queue>
#include <sstream>
#include <functional>
#include <thread>
#include <iostream>
#include <mutex>
#include <condition_variable>

#define MIDDLEWARE_PREFIX "[soss-echo-middleware]: "

using Json = soss::json::Json;

// Class used for emulate the remote connection with the middleware.
class MiddlewareConnection
{
    using Callback = std::function<void(const Json& message)>;

    struct MiddlewareMessageQueued
    {
        std::string topic;
        Json message;
    };

public:
    MiddlewareConnection(bool roundtrip)
        : roundtrip_(roundtrip)
    {}

    void subscribe(
            const std::string& topic,
            Callback callback)
    {
        auto it = topic_callbacks_.find(topic);
        if(it != topic_callbacks_.end())
        {
            it->second.emplace_back(callback);
        }
        else
        {
            topic_callbacks_[topic].emplace_back(callback);
        }
    }

    void publish(
            const std::string& topic,
            const Json& message)
    {
        std::cout << MIDDLEWARE_PREFIX "send to middleware:" << std::endl;
        std::cout << message.dump(4) << std::endl;
        if(roundtrip_)
        {
            receive(topic, message);
        }
    }

    void receive(
            const std::string& topic,
            const Json& message)
    {
        std::unique_lock<std::mutex> push_lock(income_mutex_);
        income_.push(MiddlewareMessageQueued{topic, message});
        push_lock.unlock();
        pop_condition_.notify_one();
    }

    void run()
    {
        running_ = true;
        listen_thread = std::thread(std::bind(&MiddlewareConnection::listen, this));
    }

    void stop()
    {
        running_ = false;
        listen_thread.join();
    }

private:
    void listen()
    {
        while(running_)
        {
            std::unique_lock<std::mutex> pop_lock(income_mutex_);
            if(!pop_condition_.wait_for(pop_lock, std::chrono::microseconds(100), [this]{ return !income_.empty(); }))
            {
                continue; //timemout
            }

            std::string topic = income_.front().topic;
            Json& message = income_.front().message;

            income_.pop();

            pop_lock.unlock();

            std::cout << MIDDLEWARE_PREFIX "receive from middleware:" << std::endl;
            std::cout << message.dump(4) << std::endl;

            auto it = topic_callbacks_.find(topic);
            if(it != topic_callbacks_.end())
            {
                for(auto&& callback: it->second)
                {
                    callback(message);
                }
            }
            else
            {
                std::cout << MIDDLEWARE_PREFIX "no callback for topic '" << topic << "'" << std::endl;
            }
        }
    }

    std::queue<MiddlewareMessageQueued> income_;
    std::mutex income_mutex_;
    std::condition_variable pop_condition_;
    std::thread listen_thread;
    bool running_;
    std::map<std::string, std::vector<Callback>> topic_callbacks_;
    bool roundtrip_;
};

#endif //SOSS__XTYPES_EXAMPLE__INTERNAL__MIDDLEWARE_CONNECTION_HPP
