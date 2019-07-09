#ifndef SOSS__MOCK__API_HPP
#define SOSS__MOCK__API_HPP

#include <soss/Message.hpp>

#include <chrono>
#include <functional>
#include <future>
#include <string>
#include <soss/mock/export.hpp>

namespace soss {
namespace mock {

bool SOSS_MOCK_API publish_message(
    const std::string& topic,
    const MessageData& msg);


using MockSubscriptionCallback = std::function<void(const MessageData&)>;

bool SOSS_MOCK_API subscribe(
    const std::string& topic,
    MockSubscriptionCallback callback);

/// Request a service
/// \param[in] retry
///     If a non-zero value is given for retry, the mock middleware will
///     repeatedly attempt the request until a response is received, which is
///     useful for cases of flaky middlewares or slow discovery. Even if the
///     server responds multiple times, only the first response will be
///     available to the std::shared_future.
std::shared_future<MessageData> SOSS_MOCK_API request(
    const std::string& topic,
    const MessageData& request_msg,
    std::chrono::nanoseconds retry = std::chrono::seconds(0));


using MockServiceCallback = std::function<MessageData(const MessageData& request)>;

void SOSS_MOCK_API serve(
    const std::string& topic,
    MockServiceCallback callback);


} // namespace mock
} // namespace soss

#endif // SOSS__MOCK__API_HPP
