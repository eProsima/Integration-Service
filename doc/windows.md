# SOSS: Windows Usage

## Build SOSS

To build `soss` we recommend using a [colcon workspace](https://colcon.readthedocs.io/en/released/user/quick-start.html).
The `soss` repo consists of many cmake packages which can be configured and built manually, but colcon makes the job much
smoother.

SOSS build has been tested with Microsoft Visual Studio Community 2019.


### Windows dependencies
On Windows, colcon and CMake might not find dependencies if they are in a directory outside their workspace. To make colcon aware of other paths, you can use `--cmake-args` option in addition to `CMAKE_PREFIX_PATH` variable.

```
$ colcon build --packages-up-to soss-core --cmake-args -DCMAKE_PREFIX_PATH=<path-of-dependencies>
```
#### Boost

Building soss needs Boost libraries; more specifically, boost/program_options.
Any version of boost > 1.59 will do the trick. In case of a new installation we recommend compiling the library dinamically.
In case of an older version that meets the requirements but compiled statically, simply add the line `set(Boost_USE_STATIC_LIBS ON)` in `soss/CMakeLists.txt` before `find_package(Boost Required)`

#### Yaml-cpp
Yaml-cpp library is part of the ROS2 ecosystem. If you have downloaded and built ROS2 following the [Installing ROS2 on Windows guide](https://index.ros.org/doc/ros2/Installation/Crystal/Windows-Install-Binary/) you might have trouble linking yaml-cpp with SOSS. This is because the yaml-cpp version in ROS2 is older than the version needed by SOSS.

Download yaml from [their official repository](https://github.com/jbeder/yaml-cpp) and compile it with CMake using the same Visual Studio configuration you are going to use in SOSS.

* If you are using a ROS2 workspace with local_setup script, for simplicity, you can replace the following files in the yaml-cpp ROS2 install directory `\ros2\install\opt\yaml_cpp_vendor`

    You should replace `yaml-cpp.dll`, `yaml-cpp.lib` and `\include` directory.

    **Note that if you rebuild ROS2, yaml-cpp will be the older version again.**

* If you are using SOSS without ROS2, then simply add the yaml lib directory to CMAKE_PREFIX_PATH as mentioned in the Windows dependencies section of this guide.

#### Known issues compiling Yaml-cpp and Soss
A few problems have been encountered while installing Yaml-cpp on windows. Even defining the CMAKE_PREFIX_PATH might not work. in this cases there is a workaround that allows to keep building soss

1. Download two separate instances of the official repository and compile them separately, one as a release and one in debug mode, since both the archives will be needed to compile soss.
2. There might be the need to specify, in the yaml-cpp_DIR. this can be easly achieved by adding the line 
   `set(yaml-cpp_DIR "<yaml-cpp_build_folder>")`
3. Run the `colcon build` to build soss, and move the files accordingly to the `unable to find file` errors colcon will encounter. this might take some time, but the result will be a folder that can be used to compile soss.



## Run test
You can use the `colcon test` command to run tests present in the workspace. You can also use the same options like colcon build in order to run specific tests.

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
## Note about UTF-8 encoding problem in ROSIDL

If you are linking against ROS2 Dashing Diademata you could have troubles running `soss-ros2-test`. The default encoding in Windows is not UTF-8 and Python by default uses Windows encoding. At the moment you can ignore this test or change function `parse_message_file` in `Lib\site-packages\rosidl_adapter\parser.py` from:

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
in order for Python to read the files with UTF-8 encoding.

## Note for making your own System Handle on Windows

If you want your system handle to run properly on Windows you must be aware of Windows dependency libraries.

On Windows, SOSS has a bin directory inside install directory where all dll Windows dependency files should be placed. Each System Handle is responsible for storing their dependencies in that folder.

`local_setup_windows` script will add the `install/bin` directory to the path, and then SOSS applications can find all the dependencies of each System Handler.
