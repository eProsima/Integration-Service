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

#include <is/core/runtime/MiddlewareInterfaceExtension.hpp>

#include <cassert>
#include <experimental/filesystem>
#include <iostream>

#ifdef WIN32
// WINDOWS includes
#include <Windows.h>
#else
#include <dlfcn.h>
#endif //  WIN32

#ifdef WIN32

#define OPEN_DYNAMIC_LIB(libname) LoadLibraryW(libname)
#define GET_LAST_ERROR() GetLastError()
#define DYNAMIC_LIB_EXTENSION "dll"

#else

#define OPEN_DYNAMIC_LIB(libname) dlopen(libname, RTLD_NOW)
#define GET_LAST_ERROR() dlerror()
#define DYNAMIC_LIB_EXTENSION "dl"

#endif //  WIN32

namespace eprosima {
namespace is {
namespace core {

class MiddlewareInterfaceExtension::Implementation
{
public:

    Implementation(
            YAML::Node&& mix_content,
            const std::string& absolute_file_directory_path)
        : _mix_content(std::move(mix_content))
        , _directory(absolute_file_directory_path)
        , _logger("is::core::Mix")
    {
        assert(_directory.is_absolute());
        assert(std::experimental::filesystem::is_directory(_directory));
    }

    Implementation(
            const Implementation& /*other*/) = default;

    Implementation(
            Implementation&& /*other*/) = default;

    ~Implementation() = default;

    bool load()
    {
        bool result = true;
        const YAML::Node& dl = _mix_content[DYNAMIC_LIB_EXTENSION];

        if (dl.IsSequence())
        {
            for (YAML::const_iterator it = dl.begin(); it != dl.end(); ++it)
            {
                result |= load_if_exists(it->as<std::string>(), _directory);
            }
        }
        else if (dl.IsScalar())
        {
            result |= load_if_exists(dl.as<std::string>(), _directory);
        }

        return result;
    }

private:

    /**
     * @brief Loads a certain dynamic library if it exists; otherwise,
     *        prints an error trace and returns failure.
     *
     * @param[in] path String representation of the library path.
     *
     * @param[in] relative_to Path to which the library's path is relative to.
     *
     * @returns `true` if the shared library was loaded, `false` otherwise.
     */
    bool load_if_exists(
            const std::string& path,
            const std::experimental::filesystem::path& relative_to)
    {
        std::experimental::filesystem::path fpath(path);

        if (fpath.is_relative())
        {
            fpath = relative_to / fpath;
        }

        if (std::experimental::filesystem::exists(fpath))
        {
            void* handle = OPEN_DYNAMIC_LIB(fpath.c_str());
            auto loading_error = GET_LAST_ERROR();

            if (loading_error)
            {
                _logger << utils::Logger::Level::ERROR
                        << "Error while loading the library "
                        << fpath << ": " << loading_error
                        << std::endl;
                return false;
            }

            if (!handle)
            {
                _logger << utils::Logger::Level::ERROR
                        << "Unkown error while loading the library "
                        << fpath << std::endl;
                return false;
            }

            _logger << utils::Logger::Level::DEBUG
                    << "Loaded shared library " << fpath << std::endl;
            return true;
        }

        _logger << utils::Logger::Level::ERROR << "Could not load shared library "
                << fpath << " because it could not be found!" << std::endl;
        return false;
    }

    /**
     * Class members.
     */

    YAML::Node _mix_content;
    std::experimental::filesystem::path _directory;
    utils::Logger _logger;

};

//==============================================================================
MiddlewareInterfaceExtension::MiddlewareInterfaceExtension(
        YAML::Node&& mix_content,
        const std::string& absolute_file_directory_path)
    : _pimpl(new Implementation(
                std::move(mix_content),
                absolute_file_directory_path))
{
}

//==============================================================================
MiddlewareInterfaceExtension::MiddlewareInterfaceExtension(
        MiddlewareInterfaceExtension&& other)
    : _pimpl(std::move(other._pimpl))
{
}

//==============================================================================
MiddlewareInterfaceExtension::~MiddlewareInterfaceExtension()
{
    _pimpl.reset();
}

//==============================================================================
MiddlewareInterfaceExtension MiddlewareInterfaceExtension::from_file(
        const std::string& filename)
{
    auto parentpath = std::experimental::filesystem::path(filename).parent_path();
    return MiddlewareInterfaceExtension(YAML::LoadFile(filename), parentpath.string());
}

//==============================================================================
MiddlewareInterfaceExtension MiddlewareInterfaceExtension::from_string(
        const std::string& mix_text,
        const std::string& absolute_file_directory_path)
{
    return MiddlewareInterfaceExtension(YAML::Load(mix_text), absolute_file_directory_path);
}

//==============================================================================
MiddlewareInterfaceExtension MiddlewareInterfaceExtension::from_node(
        YAML::Node&& node,
        const std::string& absolute_file_directory_path)
{
    return MiddlewareInterfaceExtension(std::move(node), absolute_file_directory_path);
}

//==============================================================================
bool MiddlewareInterfaceExtension::load()
{
    return _pimpl->load();
}

} //  namespace core
} //  namespace is
} //  namespace eprosima
