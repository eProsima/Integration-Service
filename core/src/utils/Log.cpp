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

#include <is/config.hpp>

namespace eprosima {
namespace is {
namespace utils {

//==============================================================================
Logger::Logger(
        const std::string& header)
    : _header(header)
#ifdef IS_COMPILE_DEBUG
    , _max_level(Level::DEBUG)
#else
    , _max_level(Level::INFO)     // TODO (@jamoralp): make this configurable by the user and by CMAKE_BUILD_TYPE flag
#endif //  IS_COMPILE_DEBUG
    , _status(CurrentLevelStatus::NON_SPECIFIED)
{
}

const Logger::Level& Logger::get_level() const
{
    return _max_level;
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
                          << "[Integration Service][ERROR] "
                          << reset;
                break;
            }
            case Level::WARN:
            {
                std::cout << bold_on
                          << yellow
                          << "[Integration Service][WARN] "
                          << reset;
                break;
            }
            case Level::INFO:
            {
                std::cout << bold_on
                          << "[Integration Service][INFO] "
                          << reset;
                break;
            }
            case Level::DEBUG:
            {
                std::cout << bold_on
                          << green
                          << "[Integration Service][DEBUG] "
                          << reset;
                break;
            }
        }

        if (!_header.empty())
        {
            std::cout << bold_on << "[" << _header << "]" << reset;
        }

        std::cout << " ";
        _status = CurrentLevelStatus::SPECIFIED;
    }
    else
    {
        _status = CurrentLevelStatus::SPECIFIED_BUT_HIDDEN;
    }

    return *this;
}

//==============================================================================
Logger& Logger::operator <<(
        const char* message)
{
    switch (_status)
    {
        case CurrentLevelStatus::NON_SPECIFIED:
        {
            // By default, INFO level will be used if the user has not specified it.
            operator <<(Level::INFO);
            return operator <<(message);
            break;
        }
        case CurrentLevelStatus::SPECIFIED:
        {
            std::cout << message;
            break;
        }
        case CurrentLevelStatus::SPECIFIED_BUT_HIDDEN:
        {
            // Do nothing
            break;
        }
    }

    return *this;
}

//==============================================================================
Logger& Logger::operator <<(
        const std::string& message)
{
    return operator <<(message.c_str());
}

//==============================================================================
Logger& Logger::operator <<(
        std::basic_ostream<char, std::char_traits<char> >&
        (*func)(
            std::basic_ostream<char, std::char_traits<char> >&))
{
    switch (_status)
    {
        case CurrentLevelStatus::NON_SPECIFIED:
        {
            // Do nothing
            break;
        }
        case CurrentLevelStatus::SPECIFIED:
        {
            std::cout << reset;
            std::cout << func;
            [[fallthrough]];
        }
        case CurrentLevelStatus::SPECIFIED_BUT_HIDDEN:
        {
            _status = CurrentLevelStatus::NON_SPECIFIED;
            break;
        }
    }

    return *this;
}

//==============================================================================
std::ostream& Logger::bold_on(
        std::ostream& os)
{
    return os << "\033[1m";
}

//==============================================================================
std::ostream& Logger::reset(
        std::ostream& os)
{
    return os << "\033[0m";
}

//==============================================================================
std::ostream& Logger::red(
        std::ostream& os)
{
    return os << "\033[31m";
}

//==============================================================================
std::ostream& Logger::green(
        std::ostream& os)
{
    return os << "\033[38;5;28m";
}

//==============================================================================
std::ostream& Logger::yellow(
        std::ostream& os)
{
    return os << "\033[33m";
}

} //  namespace is
} //  namespace utils
} //  namespace eprosima
