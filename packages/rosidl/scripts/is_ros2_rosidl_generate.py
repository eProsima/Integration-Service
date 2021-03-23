#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import copy
import em
from io import StringIO
import os
import sys
import subprocess

try:
    from rosidl_adapter.parser import parse_message_file
    from rosidl_adapter.parser import parse_service_file
    from rosidl_cmake import convert_camel_case_to_lower_case_underscore

except ImportError:
    print('Unable to import rosidl_adapter. Please source a ROS2 installation first.', end='', file=sys.stderr)
    sys.exit(1)


def get_message_type_name(spec):
    try:
        return spec.base_type.type
    except AttributeError:
        return spec.srv_name


def generate_file(template, destination, context):

    base_name = template.split('/')[-1]
    base_name_components = base_name.split('.')

    # Remove the last base name component if it's .em
    if base_name_components[-1] == 'em':
        base_name_components.pop(-1)

    # Add the message type name to the source file
    base_name_components[0] = base_name_components[0] + '__' + get_message_type_name(context['spec'])
    filename = '.'.join(base_name_components)
    output_file_path = '/'.join([destination, filename])

    output_buffer = StringIO()
    interpreter = em.Interpreter(output=output_buffer, globals=copy.deepcopy(context))
    interpreter.file(open(template, 'r'))

    if not os.path.exists(destination):
        os.makedirs(destination)

    with open(output_file_path, 'w') as file:
        file.write(output_buffer.getvalue())

def sys_call(command):
    dev_null = open(os.devnull, 'w')
    return subprocess.check_output((command).split(), universal_newlines = True, stderr = dev_null)

def find_idl_include_paths():
    environment = sys_call("env")
    for line in environment.splitlines():
        if line.find('AMENT_PREFIX_PATH') >= 0:
            line = line[line.find("="):]
            includes = line.replace("=", " -I").replace(":", " -I")
            idl_includes = ""
            for include in includes.split():
                idl_includes += include + "/share "
            return idl_includes

def get_idl_from_file(idl_file, includes):
    preprocess_cmd = "cpp -H " + includes + " " + idl_file
    unrolled_idl = sys_call(preprocess_cmd)
    idl = ""
    for line in unrolled_idl.splitlines():
        if line and line[0] != "#" and "structure_needs_at_least_one_member" not in line:
            idl += line + "\n"
    return idl

def generate_files(package, source_dir, header_dir, idl_files, cpp_files, hpp_files, prefix, parse_fnc):
    includes = find_idl_include_paths()
    for idl_file in idl_files:
        idl = get_idl_from_file(idl_file[:-3] + "idl", includes)

        context = {
            'idl': idl,
            'spec': parse_fnc(package, idl_file),
            'subdir': prefix,
            'get_header_filename_from_msg_name': convert_camel_case_to_lower_case_underscore
        }

        for cpp_file in cpp_files:
            generate_file(cpp_file, source_dir + '/' + prefix, context)

        for hpp_file in hpp_files:
            generate_file(hpp_file, header_dir + '/' + prefix, context)


def main(cli_args):
    parser = argparse.ArgumentParser(
        description='Generate .cpp and .hpp files for a set of messages and services given the idl files and the EmPy '
                    '(embedded python) templates for the source files.')

    parser.add_argument('--package', required=True, help='Package that the rosidl files belong to')
    parser.add_argument('--source-dir', required=True, help='Output directory for source (.cpp) files')
    parser.add_argument('--header-dir', required=True, help='Output directory for header (.hpp) files')
    parser.add_argument('--msg-idl-files', nargs='*', required=True, help='IDL files for message specifications')
    parser.add_argument('--msg-cpp-files', nargs='*', required=True,
                        help='EmPy templates for .cpp files, each one will be applied to each message idl')
    parser.add_argument('--msg-hpp-files', nargs='*', required=True,
                        help='EmPy templates for .hpp files, each one will be applied to each message idl')
    parser.add_argument('--srv-idl-files', nargs='*', required=True, help='IDL files for service specifications')
    parser.add_argument('--srv-cpp-files', nargs='*', required=True,
                        help='EmPy templates for .cpp files, each one will be applied to each service idl')
    parser.add_argument('--srv-hpp-files', nargs='*', required=True,
                        help='EmPy templates for .hpp files, each one will be applied to each service idl')

    args = parser.parse_args(cli_args[1:])

    generate_files(args.package, args.source_dir, args.header_dir,
                   args.msg_idl_files, args.msg_cpp_files, args.msg_hpp_files,
                   'msg', parse_message_file)

    generate_files(args.package, args.source_dir, args.header_dir,
                   args.srv_idl_files, args.srv_cpp_files, args.srv_hpp_files,
                   'srv', parse_service_file)


if __name__ == '__main__':
    main(sys.argv)
