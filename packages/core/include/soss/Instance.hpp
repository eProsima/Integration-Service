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

#ifndef SOSS__INSTANCE_HPP
#define SOSS__INSTANCE_HPP

#include <yaml-cpp/yaml.h>

#include <soss/SystemHandle.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <soss/core/export.hpp>

namespace soss {

//==============================================================================
class SOSS_CORE_API InstanceHandle
{
public:

  /// \brief True if this the soss instance is still running, false if it has
  /// quit.
  bool running() const;

  /// \brief Implicit cast to running()
  operator bool() const { return running(); }

  /// \brief Wait for the instance to finish running.
  ///
  /// The instance may be stopped by calling quit() or by sending SIGINT
  /// (ctrl+C from the terminal)
  ///
  /// \returns The return code for this instance
  int wait();

  /// \brief Wait for the instance to finish running, or until the max time has
  /// been reached.
  ///
  /// \returns a reference to this instance handle so that it can be chained
  /// with quit() or wait().
  InstanceHandle& wait_for(const std::chrono::nanoseconds& max_time);

  /// \brief Instruct the node handle to quit (this will not occur instantly).
  ///
  /// Follow this with a call to wait() in order to wait until the instance has
  /// finished and retrieve the return code.
  ///
  /// \returns a reference to this instance handle so that it can be chained
  /// with wait_for() or wait().
  InstanceHandle& quit();

  /// \brief The destructor will call quit() and then wait(), because the soss
  /// instance cannot run without the handle active.
  ~InstanceHandle();

  class Implementation;
  /// This class cannot be constructed directly. Use Instance::run() or
  /// run_instance() to get an InstanceHandle.
  InstanceHandle(std::shared_ptr<Implementation> impl);
  InstanceHandle(InstanceHandle&&);

  /// \brief Request the TypeRegitry for a middleware
  ///
  /// \returns a pointer to the TypeRegistry or nullptr if the middleware does
  /// not exits.
  const TypeRegistry* type_registry(const std::string& middleware_name);

private:
  std::shared_ptr<Implementation> _pimpl;
};

//==============================================================================
using MiddlewarePrefixPathMap =
    std::unordered_map<std::string, std::vector<std::string>>;

//==============================================================================
class SOSS_CORE_API Instance
{
public:

  /// \brief Create a soss instance
  ///
  /// Pass along the arguments that were given to main(~)
  Instance(int argc, char* argv[]);

  /// \brief Create a soss instance
  ///
  /// Explicitly set the relevant properties
  Instance(const YAML::Node& config_node,
           const std::vector<std::string>& soss_prefixes,
           const MiddlewarePrefixPathMap& middleware_prefixes);

  /// \brief Create a soss instance
  ///
  /// Explicitly set the relevant properties
  Instance(const std::string& config_file_path,
           const std::vector<std::string>& soss_prefixes,
           const MiddlewarePrefixPathMap& middleware_prefixes);

  /// \brief Run the soss instance in its own thread. The handle will allow you
  /// to wait on that thread or instruct it to quit. The handle uses RAII, so
  /// the instance will stop running automatically if the InstanceHandle dies.
  ///
  /// If you call run() again while another instance handle is still alive and
  /// running, then the new instance handle will still refer to the same running
  /// instance. Calling quit() on any of the handles will make them all quit.
  /// The automatic RAII shutdown of the instance will take effect once all
  /// handles have died.
  ///
  /// If existing handles are still alive but no longer running, then they will
  /// become detached from this instance, and calling run() will initiate a new
  /// set of instance handles.
  ///
  /// In most cases, you can simply call one of the run_instance() functions and
  /// not worry about how InstanceHandles might interact.
  InstanceHandle run();

  ~Instance();

private:
  class Implementation;
  std::unique_ptr<Implementation> _pimpl;
};

//==============================================================================
/// Create a soss instance and run it in its own thread
SOSS_CORE_API InstanceHandle run_instance(int argc, char* argv[]);

//==============================================================================
/// Create a soss instance for the specified YAML and run it in its own thread
SOSS_CORE_API InstanceHandle run_instance(
    const YAML::Node& config_node,
    const std::vector<std::string>& soss_prefixes = {},
    const MiddlewarePrefixPathMap& middleware_prefixes = {});

//==============================================================================
/// Create a soss instance for the specified config file and run it in its own
/// thread
SOSS_CORE_API InstanceHandle run_instance(
    const std::string& config_file_path,
    const std::vector<std::string>& soss_prefixes = {},
    const MiddlewarePrefixPathMap& middleware_prefixes = {});

} // namespace soss

#endif // SOSS__INSTANCE_HPP
