/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef SOSS__SEARCH_HPP
#define SOSS__SEARCH_HPP

#include <memory>
#include <set>
#include <string>
#include <vector>
#include <soss/core/export.hpp>

namespace soss {

/// \brief This class searches for soss message/service plugin resource files,
/// called Middleware Interface eXtension (.mix) files.
///
/// These files will be searched for based on a fixed lookup scheme. This lookup
/// scheme has two phases:
///
/// 1. Search based on the middleware prefixes <mw_prefix>
/// 2. Search based on the soss prefixes <soss_prefix>
///
/// The middleware prefixes and soss prefixes can be passed in as command line
/// arguments or set as environment variables. Command line arguments will take
/// precedence over environment variables. The environment variable named
/// SOSS_PREFIX_PATH will be added to the soss prefixes <soss_prefix>, and
/// SOSS_<MIDDLEWARE_NAME>_PREFIX_PATH will be added to the middleware prefixes
/// <mw_prefix>. The environment variables should be a colon-separated list of
/// absolute paths.
///
/// Additionally the contents of the LD_LIBRARY_PATH variable will be added to
/// the soss prefixes <soss_prefix>, because the soss/ resource directory is
/// expected to be inside a lib directory. Finally, /usr/local/lib/<arch>,
/// /usr/local/lib, /usr/lib/<arch>, and /usr/lib will be added to the soss
/// prefixes <soss_prefix> in that order of precedence.
///
/// The search scheme is described below, where <mw_prefix> and <soss_prefix>
/// are defined above. <middleware> refers to the name of the middleware (as
/// given to the constructor of the Search class). <type> is the type name of
/// the message or service. In cases of <msg|srv|*>, messages will use msg while
/// services use srv; when searching for things other than messages or services,
/// a custom string can be substituted for *.
///
/// 1. <mw_prefix>/<msg|srv|*>/<type>.mix
/// 2. <mw_prefix>/<type>.mix
/// 3. <soss_prefix>/<middleware>/<msg|srv|*>/<type>.mix
/// 4. <soss_prefix>/<middleware>/<type>.mix
/// 5. <soss_prefix>/soss/<middleware>/<msg|srv|*>/<type>.mix
/// 6. <soss_prefix>/soss/<middleware>/<type>.mix
///
/// The <type> value will usually look like package_name/MessageType. Any
/// slashes within the <type> name will be used as a directory delimiters while
/// searching.
///
/// Similarly, the search pattern for the base soss middleware interface
/// extension file (<middleware>.mix) will be:
///
/// 1. <mw_prefix>/<middleware>.mix
/// 2. <soss_prefix>/<middleware>.mix
/// 3. <soss_prefix>/soss/<middleware>.mix
/// 4. <soss_prefix>/soss/<middleware>/<middleware>.mix
class SOSS_CORE_API Search
{
public:

    /// \brief Create a search utility for the specified middleware.
    Search(
            const std::string& middleware_name);

    // Copy constructor
    Search(
            const Search& other);

    // Move constructor
    Search(
            Search&& other);

    // Copy assignment operator
    Search& operator =(
            const Search& other);

    // Move assignment operator
    Search& operator =(
            Search&& other);

    /// \brief Add a custom middleware prefix path. The paths given here will be
    /// used as <mw_prefix> path prefixes, and will be checked before falling back
    /// on middleware prefix paths that were passed in as command line arguments
    /// or environment variables. The prefix paths passed into this function will
    /// be evaluated starting from the path most recently passed in, to the first
    /// one passed in (i.e. in reverse order).
    ///
    /// \param[in] path
    ///   An absolute path to search as a middleware prefix path.
    void add_priority_middleware_prefix(
            const std::string& path);

    /// \brief Add a custom middleware prefix path. The paths given here will be
    /// used as <mw_prefix> path prefixes, and will be checked after all other
    /// middleware prefixes have been exhausted. The prefix paths passed into this
    /// function will be evaluated starting from the path most recently passed in,
    /// to the first one passed in (i.e. in reverse order).
    ///
    /// \param[in] path
    ///   An absolute path to search as a middleware prefix path.
    void add_fallback_middleware_prefix(
            const std::string& path);

    /// \brief Find a mix file that provides information for a message type.
    ///
    /// \param[in] msg_type
    ///   This will be used for <type> in the search scheme.
    ///
    /// \param[out] checked_paths
    ///   If given a non-nullptr, this will be filled with a list of paths that
    ///   were searched. This may be useful for debugging purposes.
    ///
    /// \returns the full path to the .mix file if found. If not found, this will
    /// be an empty string.
    std::string find_message_mix(
            const std::string& msg_type,
            std::vector<std::string>* checked_paths = nullptr) const;

    /// \brief Find a mix file that provides information for a service type.
    ///
    /// \param[in] srv_type
    ///   This will be used for <type> in the search scheme.
    ///
    /// \param[out] checked_paths
    ///   If given a non-nullptr, this will be filled with a list of paths that
    ///   were searched. This may be useful for debugging purposes.
    ///
    /// \returns the full path to the .mix file if found. If not found, this will
    /// be an empty string.
    std::string find_service_mix(
            const std::string& srv_type,
            std::vector<std::string>* checked_paths = nullptr) const;

    /// \brief Find a mix file that provides some kind of information besides a
    /// message or a service.
    ///
    /// \param[in] type
    ///   This will be used for <type> in the search scheme.
    ///
    /// \param[in] subdir
    ///   This will replace <msg|srv|*> in the search scheme. Leave this as an
    ///   empty string to not search in a <msg|srv|*> subdirectory.
    ///
    /// \param[out] checked_paths
    ///   If given a non-nullptr, this will be filled with a list of paths that
    ///   were searched. This may be useful for debugging purposes.
    ///
    /// \returns the full path to the .mix file if found. If not found, this will
    /// be an empty string.
    std::string find_generic_mix(
            const std::string& type,
            const std::string& subdir = "",
            std::vector<std::string>* checked_paths = nullptr) const;

    /// \brief Find any file (with any extension, not just .mix) that may be
    /// residing in a soss or middleware subdirectory.
    ///
    /// \param[in] filename
    ///   The name of the file, including its extension. This should include any
    ///   directories that it may be nested in relative to its soss or middleware
    ///   directory.
    ///
    /// \param[in] subdir
    ///   A subdirectory that it might or might not be nested inside of.
    ///
    /// \param[out] checked_paths
    ///   If given a non-nullptr, this will be filled with a list of paths that
    ///   were searched. This may be useful for debugging purposes.
    ///
    /// \returns the full path to the file if found. If not found, this will be
    /// an empty string.
    std::string find_file(
            std::string filename,
            const std::string& subdir = "",
            std::vector<std::string>* checked_paths = nullptr) const;

    /// \brief Use this to toggle whether the Search should check for files
    /// relative to the directory of the config file that was used to launch soss.
    ///
    /// By default this behavior is turned off.
    ///
    /// The config-file directory will be treated as a middleware prefix whose
    /// priority comes directly before the "fallback" middleware prefixes. It will
    /// will be searched after "priority" middleware prefixes, and after any
    /// prefixes passed in as command line arguments or given as environment
    /// variables.
    ///
    /// \returns a reference to this same Search instance.
    Search& relative_to_config(
            bool toggle = true);

    /// \brief Use this to toggle whether the Search should check for files
    /// relative to the user's home directory.
    ///
    /// By default this behavior is turned off.
    ///
    /// The home directory will be treated as a middleware prefix whose priority
    /// is the same as the relative_to_config() priority, except
    /// relative_to_config() will have higher priority if both are activated at
    /// the same time.
    Search& relative_to_home(
            bool toggle = true);

    /// \brief Use this to toggle whether system prefixes should be ignored. Note
    /// that this has some overlap with the ignore_soss_prefixes option.
    ///
    /// By default these prefixes are not ignored.
    ///
    /// \returns a reference to this same Search instance.
    Search& ignore_system_prefixes(
            bool toggle = true);

    /// \brief Use this to toggle whether all soss prefixes should be ignored.
    /// Note that this has some overlap with the ignore_system_prefixes option.
    ///
    /// By default these prefixes are not ignored.
    ///
    /// \returns a reference to this same Search instance.
    Search& ignore_soss_prefixes(
            bool toggle = true);

    /// \brief Use this to toggle whether middleware prefixes should be ignored.
    ///
    /// By default these prefixes are not ignored.
    ///
    /// \returns a reference to this same Search instance.
    Search& ignore_middleware_prefixes(
            bool toggle = true);

    ~Search();

    class Implementation;

private:

    std::unique_ptr<Implementation> _pimpl;

};

} // namespace soss

#endif // SOSS__SEARCH_HPP
