# SOSS: Windows Usage

## Build SOSS

To build `soss` we recommend using a [colcon workspace](https://colcon.readthedocs.io/en/released/user/quick-start.html).
The `soss` repo consists of many cmake packages which can be configured and built manually, but colcon makes the job much
smoother.

SOSS build has beem tested with Microsoft Visual Studio Community 2019.


### Windows dependencies
On windows colcon and CMake may not found dependencies if they are in path outside their workspace. To make colcon aware of these path you can use `--cmake-args` option in addition to `CMAKE_PREFIX_PATH` variable.

```
$ colcon build --packages-up-to soss-core --cmake-args -DCMAKE_PREFIX_PATH=<path-of-dependencies>
```
#### Yaml-cpp
Yaml-cpp library is part of ROS2 echosystem. If you have downloaded and built ROS2 following the [Installing ROS2 on Windows guide](https://index.ros.org/doc/ros2/Installation/Crystal/Windows-Install-Binary/) you could have troubles linking yaml-cpp with SOSS. This is because the yaml-cpp version in ROS2 is older than SOSS yaml needed version.

Download yaml from [their official repository](https://github.com/jbeder/yaml-cpp) and compile it with CMake and same Visual Studio configuration you are going to use in SOSS.

* If you are using ROS2 workspace with local_setup script, for simplicity, you can replace files in yaml-cpp ROS2 install directory `\ros2\install\opt\yaml_cpp_vendor`

    You should replace `yaml-cpp.dll`, `yaml-cpp.lib` and `\include` directory.

    **Note that if you rebuild ROS2, yaml-cpp will be the older version again.**

* If you are using SOSS without ROS2 simply add yaml lib directory to CMAKE_PREFIX_PATH like is refered in Windows dependencies section in this guide.

## Run test
You can use `colcon test` command tu run tests present in the workspace. You can also use the same options like colcon build in order to run specific test

```
$ colcon test --packages-select soss-ros2-test
```

## Run SOSS

In order to run SOSS properly you have to make your console a SOSS workspace. On Windows this is done with `local_setup_windows.bat` script located in SOSS install directory. This script sets path libraries needed in SOSS and System Handlers.

Assuming `C:\dev\soss` is our SOSS workspace directory: 
```
$ c:\dev\soss>cd install
$ c:\dev\soss\install>local_setup_windows.bat
$ c:\dev\soss\install>cd ..\
$ c:\dev\soss\install>soss <yaml-config-file>
```
## Note for UTF8 encodig problem in ROSIDL

If you are linking against ROS2 Dashing Diademata you could have troubles running `soss-ros2-test`. The default encoding in Windows is not UTF8 and Python by default use Windows encoding. At the moment you can ignore this test or change function `parse_message_file` in `Lib\site-packages\rosidl_adapter\parser.py` from:

```
def parse_message_file(pkg_name, interface_filename):
    basename = os.path.basename(interface_filename)
    msg_name = os.path.splitext(basename)[0]
    with open(interface_filename, 'r') as h:
        return parse_message_string(
            pkg_name, msg_name, h.read())
```

to 

```
def parse_message_file(pkg_name, interface_filename):
    basename = os.path.basename(interface_filename)
    msg_name = os.path.splitext(basename)[0]
    with open(interface_filename, 'r', encoding='UTF8') as h:
        return parse_message_string(
            pkg_name, msg_name, h.read())
```
in order Python to read the files with UTF8 encoding.

## Note for making your own System Handle on Windows

If you want your system handle to run properly on Windows you must be aware of Windows dependency libraries.

On Windows, SOSS has a bin directory inside install directory when all dll Windows dependencies files should be placed. Each System Handle is responsable to store their dependencies in that folder.

`local_setup_windows` script adds `install/bin` directory to the path and then SOSS application can find all System Handler's dependencies.