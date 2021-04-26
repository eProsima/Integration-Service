/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

/**
 * This very simple translation unit lets us spawn an *Integration Service* instance
 * and return its exit code once it's finished. This provides the main function
 * for the canonical `integration-service` command line program.
 */
int main(
        int argc,
        char* argv[])
{
    return eprosima::is::run_instance(argc, argv).wait();
}
