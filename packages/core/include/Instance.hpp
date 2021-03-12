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

#ifndef _IS_CORE_INSTANCE_HPP_
#define _IS_CORE_INSTANCE_HPP_

#include <yaml-cpp/yaml.h>

#include <is/core/systemhandle/SystemHandle.hpp>
#include <is/core/export.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// namespace eprosima {
namespace is {
namespace core {

/**
 * @class InstanceHandle
 *        This is the class responsible of handling an *Integration Service* instance.
 *
 *        It allows to perform several actions on the *Integration Service*
 *        instance, such as asking whether it is running or not or handling
 *        the threads that are launched each time a SystemHandle is launched from
 *        the core.
 *
 *        It also allows to quit the instance in a safe way, waiting for the pending
 *        jobs to finish.
 */

class IS_CORE_API InstanceHandle
{
public:

    /**
     * @brief Destructor.
     *
     * @details The destructor will call `quit()` and then `wait()`, because
     *          the *Integration Service* instance cannot run without the handle active.
     */
    ~InstanceHandle();

    /**
     * @brief It allows to check if the instance is still running.
     *
     * @returns `true` if the *Integration Service* instance is still running,
     *          or `false` otherwise.
     */
    bool running() const;

    /**
     * @brief `bool()` operator overload. It performs an implicit cast to `running()`.
     *
     * @returns `true` if the *Integration Service* instance is still running,
     *          or `false` otherwise.
     */
    operator bool() const;

    /**
     * @brief Wait for the instance to finish running.
     *
     *        The instance may be stopped by calling `quit()` or by sending `SIGINT`
     *        (*ctrl+C* from the terminal).
     *
     * @returns The return code for this instance execution process.
     */
    int wait();

    /**
     * @brief Wait for the instance to finish running, or until the max time has
     *        been reached.
     *
     * @param[in] max_time Time, in milliseconds, to wait for the instance to finish running.
     *
     * @returns A reference to this instance handle, so that it can be chained
     *          with `quit()` or `wait()`.
     */
    InstanceHandle& wait_for(
            const std::chrono::milliseconds& max_time);

    /**
     * @brief Instruct the node handle to quit (this will not occur instantly).
     *
     *        Follow this with a call to `wait()` in order to wait until the instance has
     *        finished running, and retrieve the return code.
     *
     * @returns A reference to this instance handle so that it can be chained
     *          with `wait_for()` or `wait()`.
     */
    InstanceHandle& quit();

    /**
     * @brief Request the TypeRegitry for a given middleware.
     *
     * @param[in] middleware_name The middleware whose TypeRegistry is wanted to be retrieved.
     *
     * @returns A pointer to the TypeRegistry, or `nullptr` if the middleware does not exist.
     */
    const TypeRegistry* type_registry(
            const std::string& middleware_name);

private:

    /**
     * @class Implementation
     *        Defines the actual implementation of the InstanceHandle class.
     *
     *        Allows to use the *pimpl* procedure to separate the implementation
     *        from the interface of InstanceHandle.
     *
     *        Methods named equal to some InstanceHandle method will not be
     *        documented again. Usually, the interface class will call to
     *        `_pimpl->method()`, but the functionality and parameters
     *        are exactly the same.
     */
    class Implementation;

    /**
     * @brief Constructor.
     *
     * @details This class cannot be constructed directly, hence the private constructors.
     *          Use `Instance::run()` or r`run_instance()` to get an InstanceHandle.
     *
     * @param[in] impl A pointer to the Implementation class.
     */
    InstanceHandle(
            std::shared_ptr<Implementation> impl);

    /**
     * @brief Copy constructor.
     *
     * @param[in] other The InstanceHandle object we want to copy.
     */
    InstanceHandle(
            const InstanceHandle& other);

    /**
     * @brief Move constructor.
     *
     * @param[in] other Movable reference to another InstanceHandle object.
     */
    InstanceHandle(
            InstanceHandle&&);

    /**
     * Class members.
     */

    std::shared_ptr<Implementation> _pimpl; // TODO (@jamoralp): shouldn't this be unique_ptr? Does it make sense to share InstanceHandle?
};

/**
 * @brief MiddlewarePrefixPathMap contains a map of prefixes that are available
 *        for each middleware to search its dynamic libraries when loading the SystemHandle
 *        plugin or the MiddlewareInterfaceExtension files.
 *
 * @see Search
 */
using MiddlewarePrefixPathMap =
        std::unordered_map<std::string, std::vector<std::string> >;

/**
 * @class Instance
 *        Base class for creating an *Integration Service* instance.
 *        It can be called directly, or under the wrapping methods `run_instance`.
 */
class IS_CORE_API Instance
{
public:

    /**
     * @brief Create an *Integration Service* instance, receiving arguments
     *        fed by the user from the command line.
     *
     * @param[in] argc Number of given arguments.
     *
     * @param[in] argv String representation list of provided arguments,
     *            to be parsed before launching the instance.
     */
    Instance(
            int argc,
            char* argv[]);

    /**
     * @brief Create an *Integration Service* instance, explicitly indicating
     *        the configuration and setting the relevant properties for the
     *        *Integration Service* core and the dedicated middleware SystemHandle plugins.
     *
     * @param[in] config_node The `YAML` configuration, structured as defined in Config `parse()`
     *            method documentation, that should be provided to the *Integration Service*
     *            to successfully start a bridging communicationi between two or more
     *            applications using different communication protocols.
     *
     * @param[in] is_prefixes Global prefix paths for the *Integration Service* to search
     *            for configuration files or `mix` files.
     *
     *            These act as a complement to the already existing environment variables
     *            created during compilation/installation steps by CMake.
     *
     * @param[in] middleware_prefix Prefix paths specific to a certain middleware.
     *            Used when loading a middleware's plugin, that is, its SystemHandle implementation.
     */
    Instance(
            const YAML::Node& config_node,
            const std::vector<std::string>& is_prefixes,
            const MiddlewarePrefixPathMap& middleware_prefixes);

    /**
     * @brief Create an *Integration Service* instance, explicitly indicating
     *        the configuration and setting the relevant properties for the
     *        *Integration Service* core and the dedicated middleware SystemHandle plugins.
     *
     * @param[in] config_file_path The `YAML` configuration file, structured as defined in Config `parse()`
     *            method documentation, that should be provided to the *Integration Service*
     *            to successfully start a bridging communicationi between two or more
     *            applications using different communication protocols.
     *
     * @param[in] is_prefixes Global prefix paths for the *Integration Service* to search
     *            for configuration files or `mix` files.
     *
     *            These act as a complement to the already existing environment variables
     *            created during compilation/installation steps by CMake.
     *
     * @param[in] middleware_prefix Prefix paths specific to a certain middleware.
     *            Used when loading a middleware's plugin, that is, its SystemHandle implementation.
     */
    Instance(
            const std::string& config_file_path,
            const std::vector<std::string>& is_prefixes,
            const MiddlewarePrefixPathMap& middleware_prefixes);

    /**
     * @brief Destructor.
     */
    ~Instance() = default;

    /**
     * @brief Run the *Integration Service* instance in its own thread.
     *
     * @details The handle allows to wait on that thread or instruct it to quit.
     *
     *          The handle uses RAII, so the instance will stop running automatically
     *          if the InstanceHandle dies.
     *
     *          If `run()` is called again while another instance handle is still alive
     *          and running, then the new instance handle will still refer to the same
     *          running instance.
     *          Calling `quit()` on any of the handles will make them all quit.
     *          The automatic RAII shutdown of the instance will take effect once all
     *          handles have died.
     *
     *          If existing handles are still alive but no longer running, then they will
     *          become detached from this instance, and calling `run()` will initiate a new
     *          set of instance handles.
     *
     *          In most cases, simply calling one of the `run_instance()` functions and
     *          not worrying about how InstanceHandle entities might interact is more
     *          than enough.
     *
     * @returns An InstanceHandle to manage the running *Integration Service* instance.
     */
    InstanceHandle run();

private:

    /**
     * @class Implementation
     *        Defines the actual implementation of the Instance class.
     *
     *        Allows to use the *pimpl* procedure to separate the implementation
     *        from the interface of Instance.
     *
     *        Methods named equal to some Instance method will not be
     *        documented again. Usually, the interface class will call to
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

/**
 * @brief Create an *Integration Service* instance and run it in its own thread.
 *        This is a wrapper for `core::Instance` constructor + `run()` method.
 *
 * @param[in] argc Number of given arguments.
 *
 * @param[in] argv String representation list of provided arguments,
 *            to be parsed before launching the instance.
 *
 * @returns An InstanceHandle to manage the running *Integration Service* instance.
 */
IS_CORE_API InstanceHandle run_instance(
        int argc,
        char* argv[]);

/**
 * @brief Create an *Integration Service* instance and run it in its own thread.
 *        This is a wrapper for `core::Instance` constructor + `run()` method.
 *
 * @param[in] config_node The `YAML` configuration, structured as defined in Config `parse()`
 *            method documentation, that should be provided to the *Integration Service*
 *            to successfully start a bridging communicationi between two or more
 *            applications using different communication protocols.
 *
 * @param[in] is_prefixes Global prefix paths for the *Integration Service* to search
 *            for configuration files or `mix` files.
 *
 *            These act as a complement to the already existing environment variables
 *            created during compilation/installation steps by CMake.
 *
 * @param[in] middleware_prefix Prefix paths specific to a certain middleware.
 *            Used when loading a middleware's plugin, that is, its SystemHandle implementation.
 *
 * @returns An InstanceHandle to manage the running *Integration Service* instance.
 */
IS_CORE_API InstanceHandle run_instance(
        const YAML::Node& config_node,
        const std::vector<std::string>& is_prefixes = {},
        const MiddlewarePrefixPathMap& middleware_prefixes = {});

/**
 * @brief Create an *Integration Service* instance and run it in its own thread.
 *        This is a wrapper for `core::Instance` constructor + `run()` method.
 *
 * @param[in] config_file_path The `YAML` configuration file, structured as defined in Config `parse()`
 *            method documentation, that should be provided to the *Integration Service*
 *            to successfully start a bridging communicationi between two or more
 *            applications using different communication protocols.
 *
 * @param[in] is_prefixes Global prefix paths for the *Integration Service* to search
 *            for configuration files or `mix` files.
 *
 *            These act as a complement to the already existing environment variables
 *            created during compilation/installation steps by CMake.
 *
 * @param[in] middleware_prefix Prefix paths specific to a certain middleware.
 *            Used when loading a middleware's plugin, that is, its SystemHandle implementation.
 *
 * @returns An InstanceHandle to manage the running *Integration Service* instance.
 */
IS_CORE_API InstanceHandle run_instance(
        const std::string& config_file_path,
        const std::vector<std::string>& is_prefixes = {},
        const MiddlewarePrefixPathMap& middleware_prefixes = {});

} //  namespace is
// } //  namespace eprosima

#endif //  _IS_CORE_INSTANCE_HPP_
