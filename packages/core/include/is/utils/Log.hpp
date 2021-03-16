/*
 * Copyright (C) 2021 - present Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef _IS_CORE_UTILS_LOG_HPP_
#define _IS_CORE_UTILS_LOG_HPP_

#include <string>
#include <is/core/export.hpp>

namespace eprosima {
namespace is {
namespace utils {

/**
 * @class Logger
 *        Allows to easily log information into the standard output.
 *        It should be used as the preferred method for printing information
 *        within the whole Integration Service suite (core and System Handles).
 */
class IS_CORE_API Logger
{
public:

    /**
     * @class Level
     *        Enumeration holding all the possible logging values.
     *        Messages logged with a logging priority level lower
     *        than the configured maximum level will not be displayed.
     *        This level is configurable via CMake parameters and can
     *        also be set using the provided `set_logging_level` method API.
     */
    enum class Level : uint8_t
    {
        ERROR,
        WARN,
        INFO,
        DEBUG
    };

    /**
     * @brief Default constructor.
     */
    Logger() = default;

    /**
     * @brief Constructor.
     * @param[in] header The user-defined headed that will be printed
     *            at the begginning of every logger's message.
     */
    Logger(
            const std::string& header);

    /**
     * @brief Copy constructor.
     */
    Logger(
            const Logger& /*other*/) = default;

    /**
     * @brief Logger shall not be move constructible.
     */
    Logger(
            const Logger&& /*other*/) = delete;

    /**
     * @brief Destructor.
     */
    ~Logger() = default;

    /**
     * @brief Operator << overload for a certain logging Level.
     *        Sets the logging level for the char/string messages
     *        streamed aferwards, until std::endl is received.
     * @param[in] level The logging Level of a new upcoming message.
     * @returns A reference to this object.
     */
    Logger& operator <<(
            const Level& level);

    /**
     * @brief Operator << overload for a certain message.
     * @param[in] message The message to be printed to stdout.
     * @returns A reference to this object.
     */
    Logger& operator <<(
            const char* message);

    /**
     * @brief Operator << overload for a certain message.
     * @param[in] message The message to be printed to stdout.
     * @returns A reference to this object.
     */
    Logger& operator <<(
            const std::string& message);

    /**
     * @brief Operator << overload for ostream function pointer.
     *        Useful for streaming special operations, such as std::endl.
     * @param[in] func Pointer to std::ostream function.
     * @returns A reference to this object.
     */
    Logger& operator <<(
            std::basic_ostream<char, std::char_traits<char> >&
            (*func)(
                std::basic_ostream<char, std::char_traits<char> >&));

private:

    /**
     * Operations for setting on/off ostream bold characters and colors.
     */

    static std::ostream& bold_on(
            std::ostream& os);

    static std::ostream& bold_off(
            std::ostream& os);

    static std::ostream& black(
            std::ostream& os);

    static std::ostream& red(
            std::ostream& os);

    static std::ostream& green(
            std::ostream& os);

    static std::ostream& yellow(
            std::ostream& os);

    /**
     * Class members.
     */

    const std::string _header;
    Level _max_level;
    bool _level_printed;
};

} //  namespace utils
} //  namespace is
} //  namespace eprosima

#endif //  _IS_CORE_UTILS_LOG_HPP_
