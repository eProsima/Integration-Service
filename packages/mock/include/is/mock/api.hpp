/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 * Copyright (C) 2020 - present Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef _IS_SH_MOCK_API_HPP_
#define _IS_SH_MOCK_API_HPP_

#include <is/core/Message.hpp>

#include <is/mock/export.hpp>

#include <chrono>
#include <functional>
#include <future>
#include <string>

namespace eprosima {
namespace is {
namespace mock {

// TODO(@jamoralp) Doxygen docs
bool IS_MOCK_API publish_message(
        const std::string& topic,
        const eprosima::xtypes::DynamicData& msg);

using MockSubscriptionCallback =
        std::function<void (const eprosima::xtypes::DynamicData&)>;

bool IS_MOCK_API subscribe(
        const std::string& topic,
        MockSubscriptionCallback callback);

/// Request a service
/// \param[in] retry
///     If a non-zero value is given for retry, the mock middleware will
///     repeatedly attempt the request until a response is received, which is
///     useful for cases of flaky middlewares or slow discovery. Even if the
///     server responds multiple times, only the first response will be
///     available to the std::shared_future.
std::shared_future<eprosima::xtypes::DynamicData> IS_MOCK_API request(
        const std::string& topic,
        const eprosima::xtypes::DynamicData& request_msg,
        std::chrono::nanoseconds retry = std::chrono::seconds(0));


using MockServiceCallback =
        std::function<eprosima::xtypes::DynamicData(const eprosima::xtypes::DynamicData& request)>;

void IS_MOCK_API serve(
        const std::string& topic,
        MockServiceCallback callback,
        const std::string& type = "");


} //  namespace mock
} //  namespace is
} //  namespace eprosima

#endif //  _IS_SH_MOCK_API_HPP_
