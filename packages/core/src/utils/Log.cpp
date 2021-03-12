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

#include <is/utils/Log.hpp>

namespace eprosima {
namespace is {
namespace utils {

//==============================================================================
Logger::Logger(
        const std::string& header)
    : _header(header)
    , _max_level(Level::INFO) // TODO (@jamoralp): make this configurable by the user and by CMAKE_BUILD_TYPE flag
    , _level_printed(false)
{
}

//==============================================================================
Logger& Logger::operator <<(
        const Logger::Level& level)
{
    if (_max_level >= level)
    {
        switch (level)
        {
            case Level::ERROR:
            {
                std::cout << bold_on
                          << red
                          << "[Integration Service][ERROR]"
                          << bold_off;
                break;
            }
            case Level::WARN:
            {
                std::cout << bold_on
                          << yellow
                          << "[Integration Service][WARN]"
                          << bold_off;
                break;
            }
            case Level::INFO:
            {
                std::cout << bold_on
                          << "[Integration Service][INFO]"
                          << bold_off;
                break;
            }
            case Level::DEBUG:
            {
                std::cout << bold_on
                          << green
                          << "[Integration Service][DEBUG]"
                          << bold_off
                          << black;
                break;
            }
        }

        std::cout << " [" << _header "] ";
        _level_printed = true;
    }

    return *this;
}

//==============================================================================
Logger& Logger::operator <<(
        const char* message)
{
    if (!_level_printed)
    {
        // By default, INFO level will be used if the user has not specified it.
        operator <<(Level::INFO);
    }

    if (_level_printed)
    {
        std::cout << message;
    }

    return *this;
}

//==============================================================================
inline Logger& Logger::operator <<(
        const std::string& message)
{
    return operator <<(message.c_str());
}

//==============================================================================
Logger& Logger::operator <<(
        std::ostream (* func)(std::ostream&))
{
    if (_level_printed)
    {
        if (std::endl == func)
        {
            std::cout << black;
            _level_printed = false;
        }
    }

    std::cout << func;
    return *this;
}

//==============================================================================
inline std::ostream& Logger::bold_on(
        std::ostream& os)
{
    return os << "\033[1m";
}

//==============================================================================
inline std::ostream& Logger::bold_off(
        std::ostream& os)
{
    return os << "\033[0m";
}

//==============================================================================
inline std::ostream& Logger::black(
        std::ostream& os)
{
    return os << "\033[30m";
}

//==============================================================================
inline std::ostream& Logger::red(
        std::ostream& os)
{
    return os << "\033[31m";
}

//==============================================================================
inline std::ostream& Logger::green(
        std::ostream& os)
{
    return os << "\033[32m";
}

//==============================================================================
inline std::ostream& Logger::yellow(
        std::ostream& os)
{
    return os << "\033[33m";
}

} //  namespace is
} //  namespace utils
} //  namespace eprosima
