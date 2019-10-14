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

#include "Config.hpp"
#include "Search-impl.hpp"
#include "register_system.hpp"

#include <soss/Instance.hpp>
#include <soss/MiddlewareInterfaceExtension.hpp>

#include <yaml-cpp/yaml.h>

#include <boost/program_options.hpp>

#include <atomic>
#include <condition_variable>
#include <experimental/filesystem>
#include <iostream>
#include <thread>

#include <csignal>

namespace soss {

namespace bpo = boost::program_options;
namespace filesystem = std::experimental::filesystem;

static int interruptable_instances = 0;
static bool interrupted = false;
static std::mutex change_interruption_mutex;

extern "C" void interruption_handler(int)
{
  interrupted = true;
  std::cout << "\nCtrl+C detected: terminating soss" << std::endl;
}

//==============================================================================
struct ArgumentStack
{
  bool begun = false;
  std::string current_flag;
  std::map<std::string, std::vector<std::string>> values;
};

//==============================================================================
class InstanceHandle::Implementation
{
public:

  friend class Instance;

  Implementation(internal::Config configuration)
    : m_running(true),
      _configuration(std::move(configuration)),
      _quit(false),
      _active_middlewares(0),
      _return_code(0)
  {
    if(!configure_soss())
    {
      _quit = true;
      _return_code = 1;
    }
  }

  // Use this constructor for ill-formed soss instances that are dead on arrival
  Implementation(const int return_code)
    : m_running(false),
      _quit(true),
      _active_middlewares(0),
      _return_code(return_code)
  {
    // Do nothing
  }

  bool configure_soss()
  {
    if(!_configuration.load_middlewares(_info_map))
    {
      std::cerr << "Failed to load middlewares!" << std::endl;
      return false;
    }

    if(!_configuration.configure_topics(_info_map))
    {
      std::cerr << "Failed to configure topics!" << std::endl;
      return false;
    }

    if(!_configuration.configure_services(_info_map))
    {
      std::cerr << "Failed to configure services!" << std::endl;
      return false;
    }

    return true;
  }

  void run()
  {
    if(_quit)
    {
      m_running = false;
      return;
    }

    if(_info_map.size() < 2)
    {
      std::cerr << "Error: Attemtping to run a soss instance without at least "
                << "2 middlewares [you are requesting: " << _info_map.size()
                << "]. A soss instance with less than 2 middlewares is "
                << "useless, so we will quit early." << std::endl;
      _quit = true;
      _return_code = 1;
      return;
    }

    {
      std::unique_lock<std::mutex> lock(change_interruption_mutex);
      signal(SIGINT, interruption_handler);
      ++interruptable_instances;
    }

    _work_threads.reserve(_info_map.size());
    for(const auto& entry : _info_map)
    {
      auto runner = [&]()
      {
        ++_active_middlewares;
        while(!interrupted && !_quit)
        {
          const bool okay = entry.second.handle->spin_once();
          if(!okay)
          {
            _quit = true;
            _return_code = 1;
            std::cerr << "Runtime Error: middleware named [" << entry.first
                      << "] has experienced a failure! We will now quit!"
                      << std::endl;
          }
        }

        --_active_middlewares;
        if(_active_middlewares == 0)
          _finished();
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
    for(auto& thread : _work_threads)
    {
      if(thread.joinable())
        thread.join();
    }

    return _return_code;
  }

  const TypeRegistry* type_registry(const std::string& middleware_name)
  {
    auto it = _info_map.find(middleware_name);
    if(it == _info_map.end())
    {
        std::cerr << "Runtime Error: middleware named [" << middleware_name
                  << "] does not exists!"
                  << std::endl;
        throw nullptr;
    }
    return &_info_map.at(middleware_name).types;
  }

  std::condition_variable m_finished;
  std::atomic_bool m_running;

private:

  void _finished()
  {
    {
      // If there are no more soss instances running in this process, return
      // the SIGINT handling to the default.
      std::unique_lock<std::mutex> lock(change_interruption_mutex);
      --interruptable_instances;
      if(interruptable_instances == 0)
      {
        signal(SIGINT, SIG_DFL);
      }
    }

    m_running = false;
    m_finished.notify_all();
  }

  std::vector<std::thread> _work_threads;
  internal::Config _configuration;
  internal::SystemHandleInfoMap _info_map;
  std::atomic_bool _quit;
  std::atomic<int64_t> _active_middlewares;
  std::atomic_int _return_code;

};

//==============================================================================
class Instance::Implementation
{
public:

  Implementation(int argc, char* argv[])
    : _early_return_code(1) // Assume that an early return should be coded as 1
  {
    _run_instance = parse_arguments(argc, argv);
    if(_run_instance)
      _run_instance = parse_configuration(YAML::LoadFile(_config_file));
  }

  Implementation(
      const YAML::Node& config_node,
      const std::vector<std::string>& soss_prefixes,
      const MiddlewarePrefixPathMap& middleware_prefixes,
      const std::string& config_file = "")
  {
    register_prefixes(soss_prefixes, middleware_prefixes);
    _run_instance = parse_configuration(config_node);

    if(config_file.empty())
    {
      _config_file = "<internal>";
    }
    else
    {
      _config_file = config_file;
      const auto abs_config_path = filesystem::absolute(config_file);
      if(filesystem::exists(abs_config_path))
      {
        Search::Implementation::set_config_file_directory(
              abs_config_path.parent_path().string());
      }
      else
      {
        std::cerr << "[soss::Instance] WARNING: Could not locate the "
                  << "config-file [" << _config_file << "]. This will make it "
                  << "impossible for plugins to search for files relative to "
                  << "the input config-file." << std::endl;
      }
    }
  }

  bool parse_arguments(int argc, char* argv[])
  {
    const std::string usage =
        "System of Systems Synthesizer - soss\n"
        "\n"
        "Enables independent middleware frameworks to communicate with each "
        "other using a plugin-based architecture";

    bpo::options_description desc(usage);
    desc.add_options()

        ("help,h", "print this usage message")

        ("config-file", "the yaml file that describes how this instance of "
         "soss should be configured")

        ("soss-prefix-path", bpo::value<std::vector<std::string>>(),
         "specify a list of soss prefix paths to use when searching for "
         "Middleware Interface eXtension (.mix) files. The environment "
         "variable SOSS_PREFIX_PATH can be set to a colon-separated list "
         "instead of using this flag.")

        ("*-prefix-path", bpo::value<std::vector<std::string>>(),
         "replace * with the name of a middleware to specify a list of "
         "middleware prefix paths to use when searching for .mix files. The"
         "environment variable SOSS_*_PREFIX_PATH can be set to a "
         "colon-separated list instead of using this flag.")
    ;

    bpo::positional_options_description p;
    p.add("config-file", 1);

    ArgumentStack arg_stack;
    std::function<std::pair<std::string, std::string>(const std::string&)>
    parse_middleware_prefix_paths = [&](const std::string& arg)
        -> std::pair<std::string, std::string>
    {
      if(!arg_stack.begun)
      {
        const std::size_t mw_end = arg.find("-prefix-path");
        if(arg.substr(0, 2) == "--" && mw_end != std::string::npos
           && arg.size() == mw_end+12)
        {
          if(arg == "--soss-prefix-path" || arg == "--*-prefix-path")
            return std::make_pair(std::string(), std::string());

          const std::string middleware_name = arg.substr(2, mw_end-2);
          arg_stack.begun = true;
          arg_stack.current_flag = middleware_name+"-prefix-path";
          return std::make_pair(arg_stack.current_flag, std::string());
        }

        return std::make_pair(std::string(), std::string());
      }

      if(arg.substr(0, 2) == "--")
      {
        // We've been given something that resembles the beginning of a new
        // argument or list, so we'll stop our collection.
        arg_stack.begun = false;

        // Just in case this is another --*-prefix-path, we should reparse this
        // argument.
        return parse_middleware_prefix_paths(arg);
      }

      arg_stack.values[arg_stack.current_flag].push_back(arg);

      return std::make_pair(arg_stack.current_flag, arg);
    };

    bpo::variables_map vm;
    bpo::store(bpo::command_line_parser(argc, argv)
               .options(desc)
               .positional(p)
               .extra_parser(parse_middleware_prefix_paths)
               .run(), vm);

    bpo::notify(vm);

    if(vm.count("help") > 0)
    {
      std::cout << desc << std::endl;
      // Printing help is not a failure, so the early return code should be 0
      // instead of the default 1.
      _early_return_code = 0;
      return false;
    }

    std::vector<std::string> soss_prefixes;
    if(vm.count("soss-prefix-path"))
    {
      const std::vector<std::string> soss_prefix_paths =
          vm["soss-prefix-path"].as<std::vector<std::string>>();

      for(const std::string& path : soss_prefix_paths)
        soss_prefixes.push_back(path);
    }

    MiddlewarePrefixPathMap middleware_prefixes;
    for(const auto& entry : arg_stack.values)
    {
      const std::string& middleware = entry.first;
      for(const std::string& path : entry.second)
        middleware_prefixes[middleware].push_back(path);
    }

    register_prefixes(soss_prefixes, middleware_prefixes);

    if(vm.count("*-prefix-path"))
    {
      std::cerr << "You have passed the command line argument --*-prefix-path, "
                << "but that is not a valid option! Please substitute * with "
                << "the name of a middleware." << std::endl;

      return false;
    }

    if(vm.count("config-file") == 0)
    {
      std::cerr << "You need to provide soss with a config-file!" << std::endl;
      return false;
    }

    _config_file = vm["config-file"].as<std::string>();
    if(!filesystem::exists(_config_file))
    {
      std::cerr << "The requested config-file does not exist: " << _config_file
                << std::endl;
      return false;
    }

    Search::Implementation::set_config_file_directory(
          filesystem::absolute(filesystem::path(_config_file).parent_path()).string());

    return true;
  }

  void register_prefixes(
      const std::vector<std::string>& soss_prefixes,
      const MiddlewarePrefixPathMap& middleware_prefixes)
  {
    for(const auto& path : soss_prefixes)
      Search::Implementation::add_cli_soss_prefix(path);

    for(const auto& entry : middleware_prefixes)
    {
      const std::string& middleware = entry.first;
      for(const std::string& path : entry.second)
        Search::Implementation::add_cli_middleware_prefix(middleware, path);
    }
  }

  bool parse_configuration(const YAML::Node& config_node)
  {
    _configuration = internal::Config(config_node, _config_file);
    return _configuration;
  }

  InstanceHandle run()
  {
    if(!_run_instance)
    {
      // There was an error during configuration, so we really shouldn't run
      // this instance. We'll return a dead InstanceHandle.
      return InstanceHandle(std::make_shared<InstanceHandle::Implementation>(
                              _early_return_code));
    }

    std::unique_lock<std::mutex> lock(_run_mutex);
    if(const auto existing_handle = _run_handle.lock())
    {
      if(existing_handle->m_running)
        return InstanceHandle(existing_handle);
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

  std::weak_ptr<InstanceHandle::Implementation> _run_handle;
  std::mutex _run_mutex;

};

//==============================================================================
bool InstanceHandle::running() const
{
  return _pimpl->m_running;
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
  if(!_pimpl->m_running)
    return *this;

  std::mutex mutex;
  std::unique_lock<std::mutex> lock(mutex);
  _pimpl->m_finished.wait_for(
        lock, max_time,
        [&]() -> bool { return !_pimpl->m_running; });

  return *this;
}

//==============================================================================
InstanceHandle &InstanceHandle::quit()
{
  _pimpl->quit();
  return *this;
}

//==============================================================================
InstanceHandle::~InstanceHandle()
{
  if (_pimpl)
  {
    quit().wait();
  }
}

//==============================================================================
InstanceHandle::InstanceHandle(std::shared_ptr<Implementation> impl)
  : _pimpl(std::move(impl))
{
  // Do nothing
}

//==============================================================================
InstanceHandle::InstanceHandle(InstanceHandle&& other)
  : _pimpl(std::move(other._pimpl))
{
  // Do nothing
}

//==============================================================================
const TypeRegistry* InstanceHandle::type_registry(const std::string& middleware_name)
{
    _pimpl->type_registry(middleware_name);
}

//==============================================================================
Instance::Instance(int argc, char* argv[])
  : _pimpl(new Implementation(argc, argv))
{
  // Do nothing
}

//==============================================================================
Instance::Instance(
    const YAML::Node& config_node,
    const std::vector<std::string>& soss_prefixes,
    const MiddlewarePrefixPathMap& middleware_prefixes)
  : _pimpl(new Implementation(config_node, soss_prefixes, middleware_prefixes))
{
  // Do nothing
}

//==============================================================================
Instance::Instance(
    const std::string& config_file_path,
    const std::vector<std::string>& soss_prefixes,
    const MiddlewarePrefixPathMap& middleware_prefixes)
  : _pimpl(new Implementation(YAML::LoadFile(config_file_path),
                              soss_prefixes, middleware_prefixes,
                              config_file_path))
{
  // Do nothing
}

//==============================================================================
InstanceHandle Instance::run()
{
  return _pimpl->run();
}

//==============================================================================
Instance::~Instance()
{
  // Do nothing
}

//==============================================================================
InstanceHandle run_instance(int argc, char* argv[])
{
  return Instance(argc, argv).run();
}

//==============================================================================
InstanceHandle run_instance(
    const YAML::Node& config_node,
    const std::vector<std::string>& soss_prefixes,
    const MiddlewarePrefixPathMap& middleware_prefixes)
{
  return Instance(config_node, soss_prefixes, middleware_prefixes).run();
}

//==============================================================================
InstanceHandle run_instance(
    const std::string& config_file_path,
    const std::vector<std::string>& soss_prefixes,
    const MiddlewarePrefixPathMap& middleware_prefixes)
{
  return Instance(config_file_path, soss_prefixes, middleware_prefixes).run();
}

} // namespace soss
