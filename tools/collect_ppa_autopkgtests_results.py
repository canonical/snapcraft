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

import argparse
import os
import subprocess
import tempfile


ACTIVE_DISTROS = ("xenial", "artful", "bionic")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("day", help="The day of the results, with format yyyymmdd")
    args = parser.parse_args()
    install_autopkgtest_results_formatter()
    with tempfile.TemporaryDirectory(dir=os.environ.get("HOME")) as temp_dir:
        clone_results_repo(temp_dir)
        format_results(temp_dir, ACTIVE_DISTROS, args.day)
        commit_and_push(temp_dir, args.day)


def install_autopkgtest_results_formatter():
    subprocess.check_call(
        ["sudo", "snap", "install", "autopkgtest-results-formatter", "--edge"]
    )


def clone_results_repo(dest_dir):
    subprocess.check_call(
        ["git", "clone", "https://github.com/elopio/autopkgtest-results.git", dest_dir]
    )


def format_results(dest_dir, distros, day):
    subprocess.check_call(
        [
            "/snap/bin/autopkgtest-results-formatter",
            "--destination",
            dest_dir,
            "--distros",
            *distros,
            "--day",
            day,
        ]
    )


def commit_and_push(repo_dir, day):
    subprocess.check_call(
        ["git", "config", "--global", "user.email", "u1test+m-o@canonical.com"]
    )
    subprocess.check_call(["git", "config", "--global", "user.name", "snappy-m-o"])

    subprocess.check_call(["git", "-C", repo_dir, "add", "--all"])
    subprocess.check_call(
        [
            "git",
            "-C",
            repo_dir,
            "commit",
            "--message",
            "Add the results for {}".format(day),
        ]
    )

    subprocess.check_call(
        [
            "git",
            "-C",
            repo_dir,
            "push",
            "https://{GH_TOKEN}@github.com/elopio/autopkgtest-results.git".format(
                GH_TOKEN=os.environ.get("GH_TOKEN_PPA_AUTOPKGTEST_RESULTS")
            ),
        ]
    )


if __name__ == "__main__":
    main()
