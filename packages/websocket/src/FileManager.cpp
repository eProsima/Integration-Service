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

#include "FileManager.hpp"

#include <soss/Search.hpp>

#include <iostream>

namespace soss {
namespace websocket {

static const std::string MiddlewareName = "websocket";

std::string soss::websocket::FileManager::find_file(const std::string& filepath)
{
  const soss::Search search = soss::Search(MiddlewareName)
    .relative_to_config()
    .relative_to_home();

  std::vector<std::string> checked_paths;

  const std::string& result = search.find_file(filepath, "", &checked_paths);
  if (result.empty())
  {
    std::string err = std::string()
      + "[soss::websocket::Server] websocket_server failed to find the "
      + "specified file '" + filepath + "'. Checked the following paths:\n";
    for (const std::string& checked_path : checked_paths)
      err += " -- " + checked_path + "\n";
    std::cerr << err << std::endl;
  }

  return result;
}

} // namespace websocket
} // namespace soss
