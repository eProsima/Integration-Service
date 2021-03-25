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
#include <is/core/Instance.hpp>

#include <yaml-cpp/yaml.h>

#include <boost/program_options.hpp> // TODO (@jamoralp): get rid of this dependency.

#include <atomic>
#include <condition_variable>
#include <experimental/filesystem>
#include <iostream>
#include <thread>

#include <csignal>

namespace eprosima {
namespace is {
namespace core {

// TODO (@jamoralp) think a better place to put these resources. Probably as static members of Implementation.
static int interruptable_instances = 0;
static bool interrupted = false;
static std::mutex change_interruption_mutex;

extern "C" void interruption_handler(
        int)
{
    interrupted = true;
    std::cout << std::endl << "\t[[ Ctrl+C detected: terminating Integration Service... ]]" << std::endl;
}

//==============================================================================
struct ArgumentStack
{
    bool begun = false;
    std::string current_flag;
    std::map<std::string, std::vector<std::string> > values;
};

//==============================================================================
class InstanceHandle::Implementation
{
public:

    friend class Instance;

    Implementation(
            internal::Config configuration)
        : m_running(true)
        , _configuration(std::move(configuration))
        , _quit(false)
        , _active_middlewares(0)
        , _return_code(0)
        , _logger("is::core::InstanceHandle")
    {
        if (!configure_integration_service())
        {
            _quit = true;
            _return_code = 1;
        }
    }

    /**
     * Use this constructor for ill-formed Integration Service instances that are dead on arrival
     */
    Implementation(
            const int return_code)
        : m_running(false)
        , _quit(true)
        , _active_middlewares(0)
        , _return_code(return_code)
    {
    }

    ~Implementation() = default;

    bool configure_integration_service()
    {
        if (!_configuration.load_middlewares(_info_map))
        {
            _logger << utils::Logger::Level::ERROR
                    << "Failed to load middlewares!" << std::endl;
            return false;
        }

        if (!_configuration.configure_topics(_info_map))
        {
            _logger << utils::Logger::Level::ERROR
                    << "Failed to configure topics!" << std::endl;
            return false;
        }

        if (!_configuration.configure_services(_info_map))
        {
            _logger << utils::Logger::Level::ERROR
                    << "Failed to configure services!" << std::endl;
            return false;
        }

        _logger << utils::Logger::Level::DEBUG
                << "Integration Service instance successfully configured." << std::endl;
        return true;
    }

    void run()
    {
        if (_quit)
        {
            m_running = false;
            return;
        }

        /**
         * It is pointless to run the *Integration Service* with less than two systems involved.
         */
        if (_info_map.size() < 2)
        {
            _logger << utils::Logger::Level::ERROR
                    << "Error: Attemtping to run an Integration Service instance without at least "
                    << "two systems (you are using: " << _info_map.size()
                    << "). An Integration Service instance with less than two systems is "
                    << "useless, so we will quit soon." << std::endl;

            _quit = true;
            _return_code = 1;
            return;
        }

        /**
         * Increments the number of active instances.
         */
        {
            std::unique_lock<std::mutex> lock(change_interruption_mutex);
            signal(SIGINT, interruption_handler);
            ++interruptable_instances;
        }

        _work_threads.reserve(_info_map.size());

        for (const auto& [mw_name, systemhandle_info] : _info_map)
        {
            /**
             * For each systemhandle, creates a working thread that will check that the
             * SystemHandle instance is alive and calls spin_once() to execute pending work.
             */
            auto runner = [&]()
                    {
                        ++_active_middlewares;

                        while (!interrupted && !_quit)
                        {
                            const bool okay = systemhandle_info.handle->spin_once();

                            if (!okay)
                            {
                                _quit = true;
                                _return_code = 1;
                                _logger << utils::Logger::Level::ERROR
                                        << "Runtime Error: SystemHandle of middleware named '"
                                        << mw_name
                                        << "' has experienced a failure! We will now quit."
                                        << std::endl;
                            }
                        }

                        --_active_middlewares;
                        if (_active_middlewares == 0)
                        {
                            _finished();
                        }
                    };

            _work_threads.emplace_back(runner);
        }
    }

    void quit()
    {
        _quit = true;
    }

    int return_code() const
    {
        return _return_code;
    }

    int wait()
    {
        for (auto& thread : _work_threads)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }

        return _return_code;
    }

    const TypeRegistry* type_registry(
            const std::string& middleware_name)
    {
        auto it = _info_map.find(middleware_name);
        if (it == _info_map.end())
        {
            _logger << utils::Logger::Level::ERROR
                    << "Trying to retrieve the type registry of the middleware named '"
                    << middleware_name
                    << ", which does not exist!"
                    << std::endl;

            return nullptr;
        }

        return &_info_map.at(middleware_name).types;
    }

    std::condition_variable m_finished;
    std::atomic_bool m_running;

private:

    friend class Instance::Implementation;

    void _finished()
    {
        {
            /**
             * If there are no more *Integration Service* instances running in this process,
             * returns the SIGINT handling to the default behaviour.
             */
            std::unique_lock<std::mutex> lock(change_interruption_mutex);
            --interruptable_instances;

            if (interruptable_instances == 0)
            {
                signal(SIGINT, SIG_DFL);
            }
        }

        m_running = false;
        m_finished.notify_all();
    }

    /**
     * Class members.
     */
    std::vector<std::thread> _work_threads;

    internal::Config _configuration;

    is::internal::SystemHandleInfoMap _info_map;

    std::atomic_bool _quit;

    std::atomic<int64_t> _active_middlewares;

    std::atomic_int _return_code;

    utils::Logger _logger;
};

//==============================================================================
class Instance::Implementation
{
public:

    Implementation(
            int argc,
            char* argv[])
        : _early_return_code(1) // Assumes that an early return should be coded as 1
        , _logger("is::core::Instance")
    {
        _run_instance = parse_arguments(argc, argv);
        if (_run_instance)
        {
            _run_instance = parse_configuration(YAML::LoadFile(_config_file));
        }
    }

    Implementation(
            const YAML::Node& config_node,
            const std::vector<std::string>& is_prefixes,
            const MiddlewarePrefixPathMap& middleware_prefixes,
            const std::string& config_file = "")
    {
        register_prefixes(is_prefixes, middleware_prefixes);
        _run_instance = parse_configuration(config_node);

        if (config_file.empty())
        {
            _config_file = "<internal>";
        }
        else
        {
            _config_file = config_file;
            const auto abs_config_path = std::experimental::filesystem::absolute(config_file);

            if (std::experimental::filesystem::exists(abs_config_path))
            {
                Search::set_config_file_directory(abs_config_path.parent_path().string());
            }
            else
            {
                _logger << utils::Logger::Level::WARN
                        << "Could not locate the config-file '"
                        << _config_file << "'. This will make it "
                        << "impossible for plugins to search for files relative to "
                        << "the input config-file." << std::endl;
            }
        }
    }

    bool parse_arguments(
            int argc,
            char* argv[])
    {
        const std::string usage =
                "eProsima Integration Service\n"
                "\n"
                "Enables independent middleware frameworks to communicate with each "
                "other using a plugin-based architecture";

        boost::program_options::options_description desc(usage);
        desc.add_options()

            ("help,h", "print this usage message")

            ("config-file", "the YAML file that describes how this instance of "
                "the eProsima Integration Service should be configured")

            ("is-prefix-path", boost::program_options::value<std::vector<std::string> >(),
                "specify a list of the eProsima Integreation Service "
                "prefix paths to use when searching for "
                "Middleware Interface eXtension (.mix) files. The environment "
                "variable IS_PREFIX_PATH can be set to a colon-separated list "
                "instead of using this flag.")

            ("*-prefix-path", boost::program_options::value<std::vector<std::string> >(),
                "replace * with the name of a middleware to specify a list of "
                "middleware prefix paths to use when searching for .mix files. The"
                "environment variable IS_*_PREFIX_PATH can be set to a "
                "colon-separated list instead of using this flag.")
        ;

        boost::program_options::positional_options_description p;
        p.add("config-file", 1);

        ArgumentStack arg_stack;

        std::function<std::pair<std::string, std::string>(const std::string&)> parse_middleware_prefix_paths =
                [&](const std::string& arg) -> std::pair<std::string, std::string>
                {
                    if (!arg_stack.begun)
                    {
                        const std::size_t mw_end = arg.find("-prefix-path");
                        if (arg.substr(0, 2) == "--" && mw_end != std::string::npos
                                && arg.size() == mw_end + 12)
                        {
                            if (arg == "--is-prefix-path" || arg == "--*-prefix-path")
                            {
                                return std::make_pair(std::string(), std::string());
                            }

                            const std::string middleware_name = arg.substr(2, mw_end - 2);
                            arg_stack.begun = true;
                            arg_stack.current_flag = middleware_name + "-prefix-path";
                            return std::make_pair(arg_stack.current_flag, std::string());
                        }

                        return std::make_pair(std::string(), std::string());
                    }

                    if (arg.substr(0, 2) == "--")
                    {
                        // We've been given something that resembles the beginning of a new
                        // argument or list, so we'll stop our collection.
                        arg_stack.begun = false;

                        // Just in case this is another --*-prefix-path, we should reparse this argument.
                        return parse_middleware_prefix_paths(arg);
                    }

                    arg_stack.values[arg_stack.current_flag].push_back(arg);

                    return std::make_pair(arg_stack.current_flag, arg);
                };

        boost::program_options::variables_map vm;
        boost::program_options::store(boost::program_options::command_line_parser(argc, argv)
                .options(desc)
                .positional(p)
                .extra_parser(parse_middleware_prefix_paths)
                .run(), vm);

        boost::program_options::notify(vm);

        if (vm.count("help") > 0)
        {
            std::cout << desc << std::endl;
            // Printing help is not a failure, so the early return code should be 0
            // instead of the default 1.
            _early_return_code = 0;
            return false;
        }

        std::vector<std::string> is_prefixes;
        if (vm.count("is-prefix-path"))
        {
            const std::vector<std::string> is_prefix_paths =
                    vm["is-prefix-path"].as<std::vector<std::string> >();

            for (const std::string& path : is_prefix_paths)
            {
                is_prefixes.push_back(path);
            }
        }

        MiddlewarePrefixPathMap middleware_prefixes;
        for (const auto& entry : arg_stack.values)
        {
            const std::string& middleware = entry.first;
            for (const std::string& path : entry.second)
            {
                middleware_prefixes[middleware].push_back(path);
            }
        }

        register_prefixes(is_prefixes, middleware_prefixes);

        if (vm.count("*-prefix-path"))
        {
            std::cerr << "You have passed the command line argument --*-prefix-path, "
                      << "but that is not a valid option! Please substitute * with "
                      << "the name of a middleware." << std::endl;

            return false;
        }

        if (vm.count("config-file") == 0)
        {
            std::cerr << "You need to provide the eprosima Integration Service "
                      << "with a config-file!" << std::endl;
            return false;
        }

        _config_file = vm["config-file"].as<std::string>();
        if (!std::experimental::filesystem::exists(_config_file))
        {
            std::cerr << "The requested config-file does not exist: " << _config_file
                      << std::endl;
            return false;
        }

        Search::set_config_file_directory(
            std::experimental::filesystem::absolute(std::experimental::filesystem::path(
                _config_file).parent_path()).string());

        return true;
    }

    void register_prefixes(
            const std::vector<std::string>& is_prefixes,
            const MiddlewarePrefixPathMap& middleware_prefixes)
    {
        for (const auto& path : is_prefixes)
        {
            Search::add_cli_is_prefix(path);
        }

        for (const auto& entry : middleware_prefixes)
        {
            const std::string& middleware = entry.first;
            for (const std::string& path : entry.second)
            {
                Search::add_cli_middleware_prefix(middleware, path);
            }
        }
    }

    bool parse_configuration(
            const YAML::Node& config_node)
    {
        _configuration = internal::Config(config_node, _config_file);
        return _configuration;
    }

    InstanceHandle run()
    {
        if (!_run_instance)
        {
            // There was an error during configuration, so we shouldn't run
            // this instance. It returns a dead InstanceHandle.
            return InstanceHandle(std::make_shared<InstanceHandle::Implementation>(
                               _early_return_code));
        }

        std::unique_lock<std::mutex> lock(_run_mutex);
        if (const auto existing_handle = _run_handle.lock())
        {
            if (existing_handle->m_running)
            {
                return InstanceHandle(existing_handle);
            }
        }

        std::shared_ptr<InstanceHandle::Implementation> handle
            = std::make_shared<InstanceHandle::Implementation>(_configuration);
        handle->run();

        // Save a weak reference to this handle so that we can keep track of whether
        // it's still running.
        _run_handle = handle;

        return InstanceHandle(std::move(handle));
    }

private:

    std::string _config_file;
    internal::Config _configuration;

    std::atomic_bool _run_instance;
    std::atomic_int _early_return_code;

    std::weak_ptr<InstanceHandle::Implementation> _run_handle; // TODO(@jamoralp): think of a better way to do this.
    std::mutex _run_mutex;

    utils::Logger _logger;
};

//==============================================================================
InstanceHandle::~InstanceHandle()
{
    if (_pimpl)
    {
        quit().wait();
    }

    _pimpl.reset();
}

//==============================================================================
bool InstanceHandle::running() const
{
    return _pimpl->m_running;
}

//==============================================================================
InstanceHandle::operator bool() const
{
    return running();
}

//==============================================================================
int InstanceHandle::wait()
{
    return _pimpl->wait();
}

//==============================================================================
InstanceHandle& InstanceHandle::wait_for(
        const std::chrono::nanoseconds& max_time)
{
    if (!_pimpl->m_running)
    {
        return *this;
    }

    std::mutex mutex;
    std::unique_lock<std::mutex> lock(mutex);

    _pimpl->m_finished.wait_for(
        lock, max_time,
        [&]() -> bool
        {
            return !_pimpl->m_running;
        });

    return *this;
}

//==============================================================================
InstanceHandle& InstanceHandle::quit()
{
    _pimpl->quit();
    return *this;
}

//==============================================================================
const TypeRegistry* InstanceHandle::type_registry(
        const std::string& middleware_name)
{
    return _pimpl->type_registry(middleware_name);
}

//==============================================================================
InstanceHandle::InstanceHandle(
        std::shared_ptr<Implementation> impl)
    : _pimpl(std::move(impl))
{
}

//==============================================================================
InstanceHandle::InstanceHandle(
        const InstanceHandle& other)
    : _pimpl(other._pimpl)
{
}

//==============================================================================
InstanceHandle::InstanceHandle(
        InstanceHandle&& other)
    : _pimpl(std::move(other._pimpl))
{
}

//==============================================================================
Instance::Instance(
        int argc,
        char* argv[])
    : _pimpl(new Implementation(argc, argv))
{
}

//==============================================================================
Instance::Instance(
        const YAML::Node& config_node,
        const std::vector<std::string>& is_prefixes,
        const MiddlewarePrefixPathMap& middleware_prefixes)
    : _pimpl(new Implementation(config_node, is_prefixes, middleware_prefixes))
{
}

//==============================================================================
Instance::Instance(
        const std::string& config_file_path,
        const std::vector<std::string>& is_prefixes,
        const MiddlewarePrefixPathMap& middleware_prefixes)
    : _pimpl(new Implementation(YAML::LoadFile(config_file_path),
            is_prefixes, middleware_prefixes,
            config_file_path))
{
}

//==============================================================================
Instance::~Instance()
{
    _pimpl.reset();
}

//==============================================================================
InstanceHandle Instance::run()
{
    return _pimpl->run();
}

} //  namespace core

//==============================================================================
core::InstanceHandle run_instance(
        int argc,
        char* argv[])
{
    return core::Instance(argc, argv).run();
}

//==============================================================================
core::InstanceHandle run_instance(
        const YAML::Node& config_node,
        const std::vector<std::string>& is_prefixes,
        const core::MiddlewarePrefixPathMap& middleware_prefixes)
{
    return core::Instance(config_node, is_prefixes, middleware_prefixes).run();
}

//==============================================================================
core::InstanceHandle run_instance(
        const std::string& config_file_path,
        const std::vector<std::string>& is_prefixes,
        const core::MiddlewarePrefixPathMap& middleware_prefixes)
{
    return core::Instance(config_file_path, is_prefixes, middleware_prefixes).run();
}

} //  namespace is
} //  namespace eprosima
