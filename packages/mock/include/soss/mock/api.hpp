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
        const xtypes::DynamicData& msg);


using MockSubscriptionCallback = std::function<void (const xtypes::DynamicData&)>;

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
std::shared_future<xtypes::DynamicData> SOSS_MOCK_API request(
        const std::string& topic,
        const xtypes::DynamicData& request_msg,
        std::chrono::nanoseconds retry = std::chrono::seconds(0));


using MockServiceCallback = std::function<xtypes::DynamicData(const xtypes::DynamicData& request)>;

void SOSS_MOCK_API serve(
        const std::string& topic,
        MockServiceCallback callback,
        const std::string& type = "");


} // namespace mock
} // namespace soss

#endif // SOSS__MOCK__API_HPP
