/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef SOSS__WEBSOCKET__SRC__FILEMANAGER_HPP
#define SOSS__WEBSOCKET__SRC__FILEMANAGER_HPP

#include <string>

namespace soss {
namespace websocket {

class FileManager
{
public:
  static std::string find_file(const std::string& filepath);
};

} // namespace websocket
} // namespace soss

#endif // SOSS__WEBSOCKET__SRC__FILEMANAGER_HPP
