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

#ifndef _IS_CORE_MIDDLEWAREINTERFACEEXTENSION_HPP_
#define _IS_CORE_MIDDLEWAREINTERFACEEXTENSION_HPP_

#include <yaml-cpp/yaml.h>
#include <soss/core/export.hpp>

#include <memory>

namespace eprosima {
namespace iscore {

/**
 * TODO(jamoralp): IS_CORE_API is not defined anywhere. Create a visibility.hpp header for dllexport_ on win32
 */
class IS_CORE_API MiddlewareInterfaceExtension
{
public:

    MiddlewareInterfaceExtension(
            YAML::Node root,
            const std::string& absolute_file_directory_path);

    MiddlewareInterfaceExtension(
            MiddlewareInterfaceExtension&& other);

    static MiddlewareInterfaceExtension from_file(
            const std::string& filename);

    static MiddlewareInterfaceExtension from_string(
            const std::string& text,
            const std::string& absolute_file_directory_path);

    static MiddlewareInterfaceExtension from_node(
            YAML::Node node,
            const std::string& absolute_file_directory_path);

    bool load() const;

    ~MiddlewareInterfaceExtension();

private:

    class Implementation;
    std::unique_ptr<Implementation> _pimpl;

};

using Mix = MiddlewareInterfaceExtension;

} // namespace iscore
} // namespace eprosima

#endif // _ISCORE_MIDDLEWAREINTERFACEEXTENSION_HPP_
