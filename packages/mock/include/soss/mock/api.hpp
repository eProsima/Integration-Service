#ifndef SOSS__MOCK__API_HPP
#define SOSS__MOCK__API_HPP

#include <soss/Message.hpp>

#include <chrono>
#include <functional>
#include <future>
#include <string>

namespace soss {
namespace mock {

bool publish_message(
    const std::string& topic,
    const soss::Message& msg);


using MockSubscriptionCallback = std::function<void(const soss::Message&)>;

bool subscribe(
    const std::string& topic,
    MockSubscriptionCallback callback);

/// Request a service
/// \param[in] retry
///     If a non-zero value is given for retry, the mock middleware will
///     repeatedly attempt the request until a response is received, which is
///     useful for cases of flaky middlewares or slow discovery. Even if the
///     server responds multiple times, only the first response will be
///     available to the std::shared_future.
std::shared_future<soss::Message> request(
    const std::string& topic,
    const soss::Message& request_msg,
    std::chrono::nanoseconds retry = std::chrono::seconds(0));


using MockServiceCallback = std::function<soss::Message(const soss::Message& request)>;

void serve(
    const std::string& topic,
    MockServiceCallback callback);


} // namespace mock
} // namespace soss

#endif // SOSS__MOCK__API_HPP
