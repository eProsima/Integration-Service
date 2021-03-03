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

#ifndef SOSS__MIDDLEWAREINTERFACEEXTENSION_HPP
#define SOSS__MIDDLEWAREINTERFACEEXTENSION_HPP

#include <yaml-cpp/yaml.h>
#include <soss/core/export.hpp>

#include <memory>

namespace soss {

class SOSS_CORE_API MiddlewareInterfaceExtension
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

} // namespace soss

#endif // SOSS__MIDDLEWAREINTERFACEEXTENSION_HPP
