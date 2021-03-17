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

#include <is/core/runtime/Search.hpp>

#include <algorithm>
#include <cctype>
#include <experimental/filesystem>
#include <iostream>
#include <list>
#include <map>

#include <cstdlib>

#ifdef WIN32
#define ENV_LIB_PATH "CMAKE_PREFIX_PATH"
#define LIB_PATH "lib/is"
#define IS_NOT_ABSOLUTE_PATH(path) path.find("C:\\") == std::string::npos && path.find("c:\\") == std::string::npos
#else
#define ENV_LIB_PATH "LD_LIBRARY_PATH"
#define LIB_PATH "is"
#define IS_NOT_ABSOLUTE_PATH(path) path.at(0) != '/'
#endif //  ifdef WIN32

namespace eprosima {
namespace is {
namespace core {

constexpr char HomeEnvVar[] = "HOME";

// TODO (@jamoralp): Add Logger debug/info traces here.
class Search::Implementation
{
public:

    Implementation(
            const std::string& middleware = "")
        : _middleware(middleware)
    {
        _global_paths = static_default_global_paths;

        /**
         *  Environment variables could change between different instantiations of the
         * Search object, so we re-check these each time a Search instance is created.
         */

        /**
         * Add external libraries values first because they are the lowest priority.
         */
        const std::vector<std::string> ld_library_paths =
                get_environment_variable_path_list(ENV_LIB_PATH);

        for (auto it = ld_library_paths.rbegin(); it != ld_library_paths.rend(); ++it)
        {
            _env_is_prefixes.add_path(*it);
        }

        /**
         * The explicitly set IS_PREFIX_PATH has a higher priority
         * than LD_LIBRARY_PATH so we add that next.
         */
        const std::vector<std::string> is_prefix_path =
                get_environment_variable_path_list("IS_PREFIX_PATH");

        for (auto it = is_prefix_path.rbegin(); it != is_prefix_path.rend(); ++it)
        {
            _env_is_prefixes.add_path(*it);
        }

        /**
         * Now we get the paths specific to this middleware,
         * and add it to the environment middleware paths.
         */
        const std::vector<std::string> mw_prefix_paths =
                get_environment_variable_path_list(
            "IS_" + to_env_format(middleware) + "_PREFIX_PATH");

        for (auto it = mw_prefix_paths.rbegin(); it != mw_prefix_paths.rend(); ++it)
        {
            _env_middleware_prefixes.add_path(*it);
        }

        /**
         * Add the home path based on the $HOME environment variable,
         * but set it to inactive by default.
         */
        _home_prefix.activate(false);
        const char* home_path = getenv(HomeEnvVar);
        if (home_path)
        {
            _home_prefix.add_path(home_path);
        }
    }

    Implementation(
            const Implementation& /*other*/) = default;

    Implementation& operator =(
            const Implementation& other) = default;

    Implementation(
            Implementation&& /*other*/) = default;

    Implementation& operator =(
            Implementation&& other) = default;

    ~Implementation() = default;

    static void add_cli_is_prefix(
            const std::string& path)
    {
        static_default_global_paths.cli_is_prefixes.add_path(path);
    }

    static void add_cli_middleware_prefix(
            const std::string& middleware,
            const std::string& path)
    {
        static_default_global_paths.cli_middleware_prefixes[middleware].add_path(path);
    }

    static void set_config_file_directory(
            const std::string& path)
    {
        static_default_global_paths.config_file_prefix.add_path(path);
    }

    void add_priority_middleware_prefix(
            const std::string& path)
    {
        _priority_middleware_prefixes.add_path(path);
    }

    void add_fallback_middleware_prefix(
            const std::string& path)
    {
        _fallback_middleware_prefixes.add_path(path);
    }

    const std::string find_file(
            const std::string& filename,
            const std::string& subdir,
            std::vector<std::string>& checked_paths) const
    {
        if (filename.at(0) == '/')
        {
            return filename;
        }

        auto check_and_append =
                [&](const std::experimental::filesystem::path test) -> bool
                {
                    checked_paths.push_back(test.string());
                    return std::experimental::filesystem::exists(test);
                };

        for (const PathSet& middleware_prefixes :
                    {
                        _priority_middleware_prefixes,
                        _global_paths.get_cli_middleware_prefixes(_middleware),
                        _env_middleware_prefixes,
                        _global_paths.config_file_prefix,
                        _home_prefix,
                        _fallback_middleware_prefixes
                    })
        {
            if (!middleware_prefixes.active())
            {
                continue;
            }

            for (const std::string& prefix_str : middleware_prefixes)
            {
                const std::experimental::filesystem::path prefix(prefix_str);

                if (!subdir.empty())
                {
                    if (check_and_append(prefix / subdir / filename))
                    {
                        return checked_paths.back();
                    }
                }

                {
                    if (check_and_append(prefix / filename))
                    {
                        return checked_paths.back();
                    }
                }
            }
        }

        for (const PathSet& is_prefixes :
                    {
                        _global_paths.cli_is_prefixes,
                        _env_is_prefixes,
                        _global_paths.system_is_prefixes
                    })
        {
            if (!is_prefixes.active())
            {
                continue;
            }

            for (const std::string& is_prefix_str : is_prefixes)
            {
                const std::experimental::filesystem::path is_prefix(is_prefix_str);

                if (!subdir.empty())
                {
                    if (check_and_append(is_prefix / _middleware / subdir / filename))
                    {
                        return checked_paths.back();
                    }
                }

                {
                    if (check_and_append(is_prefix / _middleware / filename))
                    {
                        return checked_paths.back();
                    }
                }

                if (!subdir.empty())
                {
                    if (check_and_append(is_prefix / LIB_PATH / _middleware / subdir / filename))
                    {
                        return checked_paths.back();
                    }
                }

                {
                    if (check_and_append(is_prefix / LIB_PATH / _middleware / filename))
                    {
                        return checked_paths.back();
                    }
                }
            }
        }

        return "";
    }

    const std::string find_middleware_mix(
            std::vector<std::string>& checked_paths) const
    {
        const std::string filename = _middleware + ".mix";

        auto check_and_append =
                [&](const std::experimental::filesystem::path test) -> bool
                {
                    checked_paths.push_back(test.string());
                    return std::experimental::filesystem::exists(test);
                };

        for (const PathSet& middleware_prefixes :
                    {
                        _priority_middleware_prefixes,
                        _global_paths.get_cli_middleware_prefixes(_middleware)
                    })
        {
            if (!middleware_prefixes.active())
            {
                continue;
            }

            for (const std::string& prefix_str : middleware_prefixes)
            {
                const std::experimental::filesystem::path prefix(prefix_str);
                if (check_and_append(prefix / filename))
                {
                    return checked_paths.back();
                }
            }
        }

        for (const PathSet& is_prefixes:
                    {
                        _global_paths.cli_is_prefixes,
                        _env_is_prefixes,
                        _global_paths.system_is_prefixes
                    })
        {
            if (!is_prefixes.active())
            {
                continue;
            }

            for (const std::string& is_prefix_str : is_prefixes)
            {
                const std::experimental::filesystem::path is_prefix(is_prefix_str);

                {
                    if (check_and_append(is_prefix / filename))
                    {
                        return checked_paths.back();
                    }
                }

                {
                    if (check_and_append(is_prefix / LIB_PATH / filename))
                    {
                        return checked_paths.back();
                    }
                }

                {
                    if (check_and_append(is_prefix / LIB_PATH / _middleware / filename))
                    {
                        return checked_paths.back();
                    }
                }
            }
        }

        return "";
    }

    void search_relative_to_config(
            bool toggle)
    {
        _global_paths.config_file_prefix.activate(toggle);
    }

    void search_relative_to_home(
            bool toggle)
    {
        _home_prefix.activate(toggle);
    }

    void search_system_prefixes(
            bool toggle)
    {
        _global_paths.system_is_prefixes.activate(toggle);
    }

    void search_is_prefixes(
            bool toggle)
    {
        _global_paths.system_is_prefixes.activate(toggle);
        _global_paths.cli_is_prefixes.activate(toggle);
        _env_is_prefixes.activate(toggle);
    }

    void search_middleware_prefixes(
            bool toggle)
    {
        _global_paths.get_cli_middleware_prefixes(_middleware).activate(toggle);
    }

    static const std::string to_env_format(
            const std::string& str)
    {
        std::string env_str(str);

        std::transform(env_str.begin(), env_str.end(), env_str.begin(),
                [](unsigned char c)
                {
                    return std::toupper(c);
                });

        std::replace(env_str.begin(), env_str.end(), '-', '_');

        return env_str;
    }

private:

    /**
     * @class PathSet
     *        Defines a set of paths that can be used for various purposes.
     */
    class PathSet
    {
    public:

        /**
         * @brief Constructor.
         */
        PathSet()
            : _active(true)
        {
        }

        /**
         * @brief Destructor.
         */
        ~PathSet() = default;

        /**
         * @brief Add a path to this set.
         *        The most recently added paths will come first
         *        when iterating from begin() to end().
         *
         * @param[in] path The path to be added.
         */
        void add_path(
                const std::string& path)
        {
            if (path.empty())
            {
                return;
            }

            if (IS_NOT_ABSOLUTE_PATH(path))
            {
                utils::Logger logger("is::core::Search::Implementation::PathSet");

                logger << utils::Logger::Level::ERROR
                       << "Attempting to add a prefix path that is not an absolute path: '"
                       << path << "'." << std::endl;

                return;
            }

            /**
             * See if this path has already been added to the list.
             */
            auto insertion = _added_paths.insert({path, {}
                            });

            /**
             * If it has been added, remove the previous entry.
             */
            if (!insertion.second)
            {
                _path_list.erase(insertion.first->second);
            }

            /**
             * Add the path to the front of the list.
             */
            _path_list.push_front(path);

            /**
             * Save its iterator in the map of added paths
             */
            insertion.first->second = _path_list.begin();

            /**
             * The above procedure makes it so that if a path gets added
             * multiple times, it will be given the priority of the most
             * recent addition, and it will not get iterated over multiple times.
             */
        }

        /**
         * @brief Get an iterator to the beginning of the set.
         *
         * @returns a const_iterator pointing to the first element.
         */
        std::list<std::string>::const_iterator begin() const
        {
            return _path_list.begin();
        }

        /**
         * @brief Get an iterator to the end of the set.
         *
         * @returns a const_iterator pointing to the end.
         */
        std::list<std::string>::const_iterator end() const
        {
            return _path_list.end();
        }

        /**
         * @brief Activate or deactivate this PathSet.
         *
         * @param[in] toggle By default, the PathSet will be activated
         *            if this funciton is called.
         */
        void activate(
                bool toggle = true)
        {
            _active = toggle;
        }

        /**
         * @brief Get the _active attribute.
         *
         * @returns the _active attribute.
         */
        bool active() const
        {
            return _active;
        }

    private:

        /**
         * Class members.
         */

        bool _active;
        std::map<std::string, std::list<std::string>::iterator> _added_paths;
        std::list<std::string> _path_list;
    };

    /**
     * @class GlobalPathInitializer
     *        This class is used to initialize the global paths
     *        when the library is first loaded.
     */
    class GlobalPathInitializer
    {
    public:

        /**
         * @brief Constructor
         */
        GlobalPathInitializer()
        {
            for (const std::string& path :
                        {
                            "/usr/lib",
                            "/usr/lib/" IS_LIBRARY_ARCHITECTURE,
                            "/usr/local/lib",
                            "/usr/local/lib/" IS_LIBRARY_ARCHITECTURE
                        })
            {
                static_default_global_paths
                .system_is_prefixes.add_path(path);
            }

            Search::Implementation::static_default_global_paths
            .config_file_prefix.activate(false);
        }

        /**
         * @brief Destructor.
         */
        ~GlobalPathInitializer() = default;
    };

    /**
     * @class GlobalPaths
     *        Holds global information regarding to every *Integration Service*
     *        System Handle, as well as the core.
     */
    class GlobalPaths
    {
    public:

        /**
         * @brief Constructor.
         */
        GlobalPaths() = default;

        /**
         * @brief Destructor.
         */
        ~GlobalPaths() = default;

        /**
         * @brief If prefixes for this middleware were given from the command line,
         *        this will return them. Otherwise, it will return an empty PathSet.
         *
         * @param[in] middleware The middleware whose prefixes will be retrieved.
         *
         * @returns A const reference to the requested PathSet.
         */
        const PathSet& get_cli_middleware_prefixes(
                const std::string& middleware) const
        {
            const auto& it = cli_middleware_prefixes.find(middleware);
            if (it != cli_middleware_prefixes.end())
            {
                return it->second;
            }

            static PathSet emptyPathSet;
            return emptyPathSet;
        }

        /**
         * @brief If prefixes for this middleware were given from the command line,
         *        this will return them. Otherwise, it will return an empty PathSet.
         *
         * @param[in] middleware The middleware whose prefixes will be retrieved.
         *
         * @returns A non-const reference to the requested PathSet.
         */
        PathSet& get_cli_middleware_prefixes(
                const std::string& middleware)
        {
            auto it = cli_middleware_prefixes.find(middleware);
            if (it != cli_middleware_prefixes.end())
            {
                return it->second;
            }

            static PathSet emptyPathSet;
            return emptyPathSet;
        }

        /**
         * Class members.
         */

        /**
         * Integration Service prefixes set from the command line interface.
         */
        PathSet cli_is_prefixes;

        /**
         * Middleware prefixes set from the command line interface.
         */
        std::map<std::string, PathSet> cli_middleware_prefixes;

        /**
         * Integration Service prefixes for system directories.
         */
        PathSet system_is_prefixes;

        /**
         * Path set that contains only the prefix of the config file
         * which was passed to the is::core::Instance.
         */
        PathSet config_file_prefix;
    };

    /**
     * @brief Get a list of paths which are present in a certain environment variable.
     *
     * @param[in] env_var The environment variable to retrieve its paths from.
     *
     * @returns A const list of the requested paths.
     */
    static const std::vector<std::string>
    get_environment_variable_path_list(
            const std::string& env_var)
    {
        std::vector<std::string> result;

        const char* var_value = getenv(env_var.c_str());
        if (!var_value)
        {
            return {};
        }

        const std::string var_str(var_value);

        std::string::const_iterator begin = var_str.begin();
        std::string::const_iterator end = var_str.end();
        for (std::string::const_iterator it = begin; it != end; ++it)
        {
#if defined(WIN32)
            if (*it == ';')
#else
            if (*it == ':')
#endif //  if defined(WIN32)
            {
                result.emplace_back(begin, it);
                begin = it + 1;
            }
        }

        if (begin != end)
        {
            result.emplace_back(begin, end);
        }

        return result;
    }

    /**
     * Class members.
     */

    std::string _middleware;

    /**
     * Static copy of the path initializer, shared by all Search instances.
     */
    static GlobalPathInitializer static_global_path_initializer;

    /**
     * This is the static copy of the global paths,
     * shared by all the instances of Search.
     */
    static GlobalPaths static_default_global_paths;

    /**
     * This is a local copy of the global paths, which can
     * be mutated by each Search instance.
     */
    GlobalPaths _global_paths;

    /**
     * Integration Service prefixes derived from environment variables.
     */
    PathSet _env_is_prefixes;

    /**
     * Middleware prefixes derived from environment variables.
     */
    PathSet _env_middleware_prefixes;

    /**
     * High-priority middleware prefixes set by the user of the Search object.
     */
    PathSet _priority_middleware_prefixes;

    /**
     * Fallback middleware prefixes set by the user of the Search object.
     */
    PathSet _fallback_middleware_prefixes;

    /**
     * Home directory prefix.
     */
    PathSet _home_prefix;
};

/**
 * Initialize Search::Implementation static members.
 */
Search::Implementation::GlobalPaths
Search::Implementation::static_default_global_paths;
Search::Implementation::GlobalPathInitializer
Search::Implementation::static_global_path_initializer;


//==============================================================================
Search::Search(
        const std::string& middleware_name)
    : _pimpl(new Search::Implementation(middleware_name))
{
}

//==============================================================================
Search::Search(
        const Search& other)
    : _pimpl(new Search::Implementation(*other._pimpl))
{
}

//==============================================================================
Search::Search(
        Search&& other)
    : _pimpl(new Search::Implementation(std::move(*other._pimpl)))
{
}

//==============================================================================
Search& Search::operator =(
        const Search& other)
{
    *_pimpl = *other._pimpl;
    return *this;
}

//==============================================================================
Search& Search::operator =(
        Search&& other)
{
    *_pimpl = std::move(*other._pimpl);
    return *this;
}

//==============================================================================
Search::~Search()
{
    _pimpl.reset();
}

//==============================================================================
void Search::add_cli_is_prefix(
        const std::string& path)
{
    Search::Implementation::add_cli_is_prefix(path);
}

//==============================================================================
void Search::add_cli_middleware_prefix(
        const std::string& middleware,
        const std::string& path)
{
    Search::Implementation::add_cli_middleware_prefix(middleware, path);
}

//==============================================================================
void Search::set_config_file_directory(
        const std::string& path)
{
    Search::Implementation::set_config_file_directory(path);
}

//==============================================================================
void Search::add_priority_middleware_prefix(
        const std::string& path)
{
    _pimpl->add_priority_middleware_prefix(path);
}

//==============================================================================
void Search::add_fallback_middleware_prefix(
        const std::string& path)
{
    _pimpl->add_fallback_middleware_prefix(path);
}

//==============================================================================
const std::string Search::find_message_mix(
        const std::string& msg_type,
        std::vector<std::string>* checked_paths) const
{
    return find_generic_mix(msg_type, "msg", checked_paths);
}

//==============================================================================
const std::string Search::find_service_mix(
        const std::string& srv_type,
        std::vector<std::string>* checked_paths) const
{
    return find_generic_mix(srv_type, "srv", checked_paths);
}

//==============================================================================
const std::string Search::find_generic_mix(
        const std::string& type,
        const std::string& subdir,
        std::vector<std::string>* checked_paths) const
{
    std::vector<std::string> _checked_paths;
    const auto result = _pimpl->find_file(type + ".mix", subdir, _checked_paths);

    if (checked_paths)
    {
        *checked_paths = _checked_paths;
    }

    return result;
}

//==============================================================================
const std::string Search::find_file(
        const std::string& filename,
        const std::string& subdir,
        std::vector<std::string>* checked_paths) const
{
    std::vector<std::string> _checked_paths;
    const auto result = _pimpl->find_file(
        std::move(filename), subdir, _checked_paths);

    if (checked_paths)
    {
        *checked_paths = _checked_paths;
    }

    return result;
}

//==============================================================================
const std::string Search::find_middleware_mix(
        std::vector<std::string>* checked_paths) const
{
    std::vector<std::string> _checked_paths;
    const auto result = _pimpl->find_middleware_mix(_checked_paths);

    if (checked_paths)
    {
        *checked_paths = _checked_paths;
    }

    return result;
}

//==============================================================================
Search& Search::relative_to_config(
        bool toggle)
{
    _pimpl->search_relative_to_config(toggle);
    return *this;
}

//==============================================================================
Search& Search::relative_to_home(
        bool toggle)
{
    _pimpl->search_relative_to_home(toggle);
    return *this;
}

//==============================================================================
Search& Search::ignore_system_prefixes(
        bool toggle)
{
    _pimpl->search_system_prefixes(!toggle);
    return *this;
}

//==============================================================================
Search& Search::ignore_is_prefixes(
        bool toggle)
{
    _pimpl->search_is_prefixes(!toggle);
    return *this;
}

//==============================================================================
Search& Search::ignore_middleware_prefixes(
        bool toggle)
{
    _pimpl->search_middleware_prefixes(!toggle);
    return *this;
}

const std::string Search::to_env_format(
        const std::string& str)
{
    return Search::Implementation::to_env_format(str);
}

} //  namespace core
} //  namespace is
} //  namespace eprosima
