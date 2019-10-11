#ifndef SOSS__XTYPES_EXAMPLE__INTERNAL__SYSTEM_CONNECTION_HPP
#define SOSS__XTYPES_EXAMPLE__INTERNAL__SYSTEM_CONNECTION_HPP

#include <map>
#include <queue>
#include <sstream>
#include <functional>
#include <thread>
#include <iostream>
#include <mutex>
#include <condition_variable>

// Class used for emulate a remote connection.

typedef std::map<std::string, uint32_t> SystemMessage;

class SystemConnection
{
    struct SystemMessageQueued
    {
        std::string topic;
        SystemMessage message;
    };

public:
    SystemConnection(bool roundtrip)
        : roundtrip_(roundtrip)
    {}

    void publish(
            const std::string& topic,
            const SystemMessage& message)
    {
        debug_print("[system]: send to system: ", message);
        if(roundtrip_)
        {
            receive(topic, message);
        }
    }

    void subscribe(
            const std::string& topic,
            const std::function<void(const SystemMessage& message)>& callback)
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

    void receive(
            const std::string& topic,
            const SystemMessage& message)
    {
        std::unique_lock<std::mutex> push_lock(income_mutex_);
        income_.push(SystemMessageQueued{topic, message});
        push_lock.unlock();
        pop_condition_.notify_one();
    }

    void run()
    {
        running_ = true;
        listen_thread = std::thread(std::bind(&SystemConnection::listen, this));
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
            SystemMessage& message = income_.front().message;

            income_.pop();

            pop_lock.unlock();

            debug_print("[system]: receive from system: ", message);

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
                std::cerr << "[system]: No exists callback for topic '" << topic << "'" << std::endl;
            }
        }
    }

    std::queue<SystemMessageQueued> income_;
    std::mutex income_mutex_;
    std::condition_variable pop_condition_;
    std::thread listen_thread;
    bool running_;
    std::map<std::string, std::vector<std::function<void(const SystemMessage& message)>>> topic_callbacks_;
    bool roundtrip_;

    static void debug_print(const std::string& prefix, const SystemMessage& message)
    {
        std::stringstream ss; //Used to avoid mixing differents outs
        ss << prefix;
        for(auto&& member: message)
        {
            ss << member.first << ": " << member.second << ", ";
        }
        ss << std::endl;

        std::cout << ss.str();
    }
};

#endif //SOSS__XTYPES_EXAMPLE__INTERNAL__SYSTEM_CONNECTION_HPP
