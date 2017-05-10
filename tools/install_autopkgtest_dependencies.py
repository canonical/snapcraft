#!/usr/bin/env python3
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import os
import subprocess
import sys

from debian import deb822


def main():
    if len(sys.argv) != 2:
        sys.exit('Usage: {} test_name'.format(sys.argv[0]))
    test_name = sys.argv[1]
    dependencies = get_dependencies(
        os.path.join(
            os.path.dirname(__file__), '..', 'debian', 'tests', 'control'),
        test_name)
    dependencies_to_install = []
    for dependency in dependencies:
        if subprocess.call(
                ['dpkg', '-l', dependency], stdout=subprocess.DEVNULL):
            dependencies_to_install.append(dependency)
    subprocess.check_call(['sudo', 'apt', 'install'] + dependencies_to_install)
    subprocess.check_call(
        ['sudo', 'apt-mark', 'auto'] + dependencies_to_install)


def get_dependencies(control_file_path, test_name):
    with open(control_file_path) as control_file:
        for test in deb822.Deb822.iter_paragraphs(control_file):
            if test['Tests'] == test_name:
                depends = [
                    dependency.split()[0] for dependency in
                    test['Depends'].split(',\n')
                    if dependency != '@'
                ]
                return depends
        else:
            sys.exit('Test not found: {}'.format(test_name))


if __name__ == '__main__':
    main()
