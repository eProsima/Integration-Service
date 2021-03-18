#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys

try:
    from rosidl_adapter.parser import parse_message_file
    from rosidl_adapter.parser import parse_service_file

except ImportError:
    print('Unable to import rosidl_adapter. Please source a ROS2 installation first.', end='', file=sys.stderr)
    sys.exit(1)

from ament_index_python.packages import get_package_share_directory


class PackageInfo:

    def __init__(self, name):
        self.pkg_name = name
        self.msg_files = []
        self.srv_files = []
        self.dependencies = []


def find_package_info(requested_pkg_name):
    info = PackageInfo(requested_pkg_name)

    share_dir = get_package_share_directory(requested_pkg_name)

    message_dir = '{}/msg'.format(share_dir)
    if os.path.exists(message_dir):
        for relative_msg_file in os.listdir(message_dir):
            if not relative_msg_file.endswith('.msg'):
                continue

            msg_file = '{}/{}'.format(message_dir, relative_msg_file)

            info.msg_files.append(msg_file)
            msg = parse_message_file(requested_pkg_name, msg_file)
            for field in msg.fields:
                if field.type.is_primitive_type():
                    continue

                info.dependencies.append(field.type.pkg_name)

    service_dir = '{}/srv'.format(share_dir)
    if os.path.exists(service_dir):
        for relative_srv_file in os.listdir(service_dir):
            if not relative_srv_file.endswith('.srv'):
                continue

            srv_file = '{}/{}'.format(service_dir, relative_srv_file)

            info.srv_files.append(srv_file)
            srv = parse_service_file(requested_pkg_name, srv_file)
            for component in [srv.request, srv.response]:
                for field in component.fields:
                    if field.type.is_primitive_type():
                        continue

                    info.dependencies.append(field.type.pkg_name)

    return info


def traverse_packages(root_pkg_name):
    package_queue = [root_pkg_name]
    inspected_packages = {}

    while package_queue:
        next_pkg = package_queue.pop()
        if next_pkg in inspected_packages:
            continue

        info = find_package_info(next_pkg)
        inspected_packages[next_pkg] = info
        for dependency in info.dependencies:
            package_queue.append(dependency)

    return inspected_packages


def print_package_info(root_pkg_name, pkg_info_dict):
    dependency_list = set(pkg_info_dict.keys())
    dependency_list.remove(root_pkg_name)
    dependency_list_str = '#'.join(dependency_list)

    message_files = pkg_info_dict[root_pkg_name].msg_files
    message_files_str = '#'.join(message_files)

    service_files = pkg_info_dict[root_pkg_name].srv_files
    service_files_str = '#'.join(service_files)

    file_dependencies = []
    for pkg, info in pkg_info_dict.items():
        file_dependencies.extend(info.msg_files)
        if pkg == root_pkg_name:
            file_dependencies.extend(info.srv_files)

    file_dependencies_str = '#'.join(file_dependencies)

    output_str = ';'.join([dependency_list_str, message_files_str, service_files_str, file_dependencies_str])
    print(output_str)


def main(cli_args):
    parser = argparse.ArgumentParser(
      description='Find dependencies for a message package.\n'
                  'First line of stdout contains a semi-colon separated list of package names.\n'
                  'Second line of stdout contains a semi-colon separated list of message file locations.')
    parser.add_argument('package', help='The packages whose dependencies should be searched for.')
    args = parser.parse_args(cli_args[1:])

    root_package_name = args.package
    print_package_info(root_package_name, traverse_packages(root_package_name))


if __name__ == '__main__':
    main(sys.argv)
