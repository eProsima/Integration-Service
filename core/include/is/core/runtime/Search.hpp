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

#ifndef _IS_CORE_RUNTIME_SEARCH_HPP_
#define _IS_CORE_RUNTIME_SEARCH_HPP_

#include <is/core/export.hpp>
#include <is/utils/Log.hpp>

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace eprosima {
namespace is {
namespace core {
/**
 * @class Search
 * @brief
 *        This class searches for *Integration Service* message/service plugin
 *        resource files, called MiddlewareInterfaceExtension (.mix) files.
 *
 *        These files will be searched based on a fixed lookup scheme.
 *        This lookup scheme comprises two phases:
 *
 *        * First, it searchs based on the middleware prefixes `mw_prefix`.
 *        * Second, it searchs based on the *Integration Service* prefixes `<is_prefix>`.
 *
 *        The middleware prefixes and *Integration Service* prefixes can be
 *        passed in as command line arguments or set as environment variables.
 *        Command line arguments will take precedence over environment variables.
 *        The environment variable named `IS_PREFIX_PATH` will be added to the
 *        *Integration Service* prefixes `<is_prefix>`, and
 *        `IS_<MIDDLEWARE_NAME>_PREFIX_PATH` will be added to the middleware prefixes
 *        `mw_prefix`. The environment variables should be a colon-separated
 *        list of absolute paths.
 *
 *        Additionally, the contents of the LD_LIBRARY_PATH variable will be added to
 *        the *Integration Service* prefixes `<is_prefix>`, because the *resource*
 *        directory is expected to be inside a *lib* directory.
 *        Finally, `/usr/local/lib/<arch>`, `/usr/local/lib`, `/usr/lib/<arch>`,
 *        and `/usr/lib` will be added to the *Integration Service* prefixes
 *        `<is_prefix>` in this same order of precedence.
 *
 *        **Lookup Scheme**
 *
 *        The lookup scheme is described below, where `mw_prefix` and `<is_prefix>`
 *        are defined above. `<middleware>` refers to the name of the middleware (as
 *        given to the constructor of the Search class). `type` is the type name of
 *        the message or service. In cases of ROS `<msg|srv|*>`, messages will use `msg`
 *        while services use `srv`; when searching for things other than messages or services,
 *        a custom string can be substituted for *.
 *
 *        * `<mw_prefix>/<msg|srv|*>/<type>.mix`
 *        * `<mw_prefix>/<type>.mix`
 *        * `<is_prefix>/<middleware>/<msg|srv|*>/<type>.mix`
 *        * `<is_prefix>/<middleware>/<type>.mix`
 *        * `<is_prefix>/is/<middleware>/<msg|srv|*>/<type>.mix`
 *        * `<is_prefix>/is/<middleware>/<type>.mix`
 *
 *        The `type` value will usually look like *package_name/MessageType*. Any
 *        slashes within the `type` name will be used as a directory delimiters while searching.
 *
 *        **Lookup Pattern**
 *
 *        Similarly, the lookup pattern for the base *Integration Service* middleware interface
 *        extension file `(<middleware>.mix)` will be:
 *
 *        * `<mw_prefix>/<middleware>.mix`
 *        * `<is_prefix>/<middleware>.mix`
 *        * `<is_prefix>/is/<middleware>.mix`
 *        * `<is_prefix>/is/<middleware>/<middleware>.mix`
 */
class IS_CORE_API Search
{
public:

    /**
     * @brief Creates a Search utility instance for the specified middleware.
     *
     * @param[in] middleware_name The middleware for which a Search utility will be created.
     */
    Search(
            const std::string& middleware_name);

    /**
     * @brief Copy constructor.
     *
     * @param[in] other Const reference to the Search object to be copied.
     */
    Search(
            const Search& other);

    /**
     * @brief Move constructor.
     *
     * @param[in] other Movable reference of the Search object to be moved.
     */
    Search(
            Search&& other);

    /**
     * @brief Copy assignment operator.
     *
     * @param[in] other Right assignment operand to be copied to *this* Search object.
     *
     * @returns A reference to *this* Search instance.
     */
    Search& operator =(
            const Search& other);

    /**
     * @brief Copy assignment operator.
     *
     * @param[in] other Right assignment operand to be moved to *this* Search object.
     *
     * @returns A reference to *this* Search instance.
     */
    Search& operator =(
            Search&& other);

    /**
     * @brief Destructor.
     */
    ~Search();

    /**
     * @brief Used by the Instance class to set *Integration Service*
     *        prefixes that were specified from the command line.
     *
     * @param[in] path The path to be added as the *Integration Service* prefix.
     */
    static void add_cli_is_prefix(
            const std::string& path);

    /**
     * @brief Used by the Instance class to set middleware prefixes
     *        that were specified from the command line.
     *
     * @param[in] middleware The middleware to which the prefix will be added.
     *
     * @param[in] path The path to be added as the *Integration Service* prefix.
     */
    static void add_cli_middleware_prefix(
            const std::string& middleware,
            const std::string& path);

    /**
     * @brief Used by the Instance class to set the path where the configuration
     *        file is stored.
     *
     * @param[in] path The path to be set as the configuration directory.
     */
    static void set_config_file_directory(
            const std::string& path);


    /**
     * @brief Adds priority to the specified path. The paths given here will be used as
     *        the first option during the search.
     *
     * @param[in] path The path to prioritize.
     */
    void add_priority_middleware_prefix(
            const std::string& path);

    /**
     * @brief Adds a custom middleware prefix path. The paths given here will be used
     *        as `mw_prefix` path prefixes, and will be checked after all other
     *        middleware prefixes have been exhausted. The prefix paths passed to this
     *        function will be evaluated starting from the path most recently passed in
     *        to the first one passed in (i.e. in reverse order).
     *
     * @param[in] path An absolute path to use as a middleware prefix search path.
     */
    void add_fallback_middleware_prefix(
            const std::string& path);

    /**
     * @brief Looks for a `mix` file that provides information for a message type.
     *
     * @param[in] msg_type This type will be used for `type` in the search scheme.
     *
     * @param[out] checked_paths If given a non-nullptr, it will be filled
     *             with a list of the paths that were searched.
     *             It may be useful for debugging purposes.
     *
     * @returns The full path to the .mix file if found. If not found,
     *          it will return an empty string.
     */
    const std::string find_message_mix(
            const std::string& msg_type,
            std::vector<std::string>* checked_paths = nullptr) const;

    /**
     * @brief Looks for a `mix` file that provides information for a service type.
     *
     * @param[in] srv_type This type will be used for `type` in the search scheme.
     *
     * @param[out] checked_paths If given a non-nullptr, it will be filled
     *             with a list of the paths that were searched.
     *             It may be useful for debugging purposes.
     *
     * @returns The full path to the .mix file if found. If not found,
     *          it will return an empty string.
     */
    const std::string find_service_mix(
            const std::string& srv_type,
            std::vector<std::string>* checked_paths = nullptr) const;

    /**
     * @brief Looks for a `mix` file that provides information
     *        for a type which is not a message nor a service.
     *
     * @param[in] type This type will be used for `type` in the search scheme.
     *
     * @param[in] subdir This will replace `<msg|srv|*>` in the search scheme.
     *            Leave this as an empty string to not search in a `<msg|srv|*>` subdirectory.
     *
     * @param[out] checked_paths If given a non-nullptr, it will be filled
     *             with a list of the paths that were searched.
     *             It may be useful for debugging purposes.
     *
     * @returns The full path to the .mix file if found. If not found,
     *          it will return an empty string.
     */
    const std::string find_generic_mix(
            const std::string& type,
            const std::string& subdir = "",
            std::vector<std::string>* checked_paths = nullptr) const;

    /**
     * @brief Looks for any file (with any extension, not just .mix) that may be
     *        residing in an *Integration Service* or middleware subdirectory.
     *
     * @param[in] filename The name of the file, including its extension.
     *            This should include any nested directories that it may contain
     *            relative to the *Integration Service* or middleware directories.
     *
     * @param[in] subdir A subdirectory that might or might not be nested into
     *            the *Integration Service* or middleware directories.
     *
     * @param[out] checked_paths If given a non-nullptr, it will be filled
     *             with a list of the paths that were searched.
     *             It may be useful for debugging purposes.
     *
     * @returns The full path to the file if found. If not found,
     *          it will return an empty string.
     */
    const std::string find_file(
            const std::string& filename,
            const std::string& subdir = "",
            std::vector<std::string>* checked_paths = nullptr) const;

    /**
     * @brief Looks for a `mix` file for the middleware specified during the construction
     *        of this Search instance.
     *
     * @param[out] checked_paths If given a non-nullptr, it will be filled
     *             with a list of the paths that were searched.
     *             It may be useful for debugging purposes.
     */
    const std::string find_middleware_mix(
            std::vector<std::string>* checked_paths = nullptr) const;

    /**
     * @brief It can be used to toggle the Search to check for files
     *        relative to the directory of the config file that was used
     *        to launch the *Integration Service*.
     *
     *        By default, this behavior is turned off.
     *
     *        The config-file directory will be treated as a middleware prefix,
     *        whose priority comes directly before the "fallback" middleware prefixes.
     *        It will be searched after "priority" middleware prefixes, and after any
     *        prefix passed in as command line argument or given as environment
     *        variables.
     *
     * @param[in] toggle Boolean to enable or disable this behavior.
     *
     * @returns A reference to this very Search instance.
     */
    Search& relative_to_config(
            bool toggle = true);

    /**
     * @brief It can be used to toggle the Search to check for files
     *        relative to the user's home directory.
     *
     *        By default this behavior is turned off.
     *
     *        The home directory will be treated as a middleware prefix,
     *        whose priority is the same as the `relative_to_config()` priority,
     *        except `relative_to_config()` will have higher priority if both
     *        are activated at the same time.
     *
     * @param[in] toggle Boolean to enable or disable this behavior.
     *
     * @returns A reference to this very Search instance.
     */
    Search& relative_to_home(
            bool toggle = true);

    /**
     * @brief It can be used to toggle whether the system prefixes are ignored or not.
     *        Note that this has some overlap with the `ignore_is_prefixes` option.
     *
     *        By default these prefixes are not ignored.
     *
     * @param[in] toggle Boolean to enable or disable this behavior.
     *
     * @returns A reference to this very Search instance.
     */
    Search& ignore_system_prefixes(
            bool toggle = true);

    /**
     * @brief It can be used to toggle whether all *Integration Service* prefixes are
     *        ignored or not. Note that this has some overlap with the
     *        `ignore_system_prefixes` option.
     *
     *        By default these prefixes are not ignored.
     *
     * @param[in] toggle Boolean to enable or disable this behavior.
     *
     * @returns A reference to this very Search instance.
     */
    Search& ignore_is_prefixes(
            bool toggle = true);

    /**
     * @brief It can be used to toggle whether the middleware prefixes are
     *        ignored or not.
     *
     *        By default these prefixes are not ignored.
     *
     * @param[in] toggle Boolean to enable or disable this behavior.
     *
     * @returns A reference to this very Search instance.
     */
    Search& ignore_middleware_prefixes(
            bool toggle = true);

    /**
     * @brief Convert a given string to environment format.
     *
     * @param[in] str The string to be converted.
     *
     * @returns A properly formatted string to the env format.
     */
    static const std::string to_env_format(
            const std::string& str);

private:

    /**
     * @class Implementation
     *        Defines the actual implementation of the Search class.
     *
     *        Allows to use the *pimpl* procedure to separate the implementation
     *        from the interface of Search.
     *
     *        Methods named equal to some Search method will not be
     *        documented again. Usually, the interface class will call
     *        `_pimpl->method()`, but the functionality and parameters
     *        are exactly the same.
     */
    class Implementation;

    /**
     * Class members.
     */

    std::unique_ptr<Implementation> _pimpl;
};

} //  namespace core
} //  namespace is
} //  namespace eprosima

#endif //  _IS_CORE_RUNTIME_SEARCH_HPP_
