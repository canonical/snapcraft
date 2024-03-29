#!/usr/bin/python3

# Copyright (C) 2018 Canonical Ltd
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
import logging
import os
import subprocess
from concurrent.futures import ThreadPoolExecutor
from multiprocessing import cpu_count

import yaml

# path for shellcheck binary
SHELLCHECK = os.getenv("SHELLCHECK", "shellcheck")
# set to non-empty to ignore all errors
NO_FAIL = os.getenv("NO_FAIL")
# set to non empty to enable 'set -x'
D = os.getenv("D")
# set to non-empty to enable verbose logging
V = os.getenv("V")
# set to a number to use these many threads
N = int(os.getenv("N") or cpu_count())
# file with list of files that can fail validation
CAN_FAIL = os.getenv("CAN_FAIL")

# names of sections
SECTIONS = frozenset(
    [
        "prepare",
        "prepare-each",
        "restore",
        "restore-each",
        "debug",
        "debug-each",
        "execute",
        "repack",
    ]
)


def parse_arguments():
    parser = argparse.ArgumentParser(description="spread shellcheck helper")
    parser.add_argument("-s", "--shell", default="bash", help="shell")
    parser.add_argument(
        "-n",
        "--no-errors",
        action="store_true",
        default=False,
        help="ignore all errors ",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", default=False, help="verbose logging"
    )
    parser.add_argument(
        "--can-fail",
        default=None,
        help=("file with list of files that are can fail " "validation"),
    )
    parser.add_argument(
        "-P",
        "--max-procs",
        default=N,
        type=int,
        metavar="N",
        help="run these many shellchecks in parallel (default: %(default)s)",
    )
    parser.add_argument("paths", nargs="+", help="paths to check")
    return parser.parse_args()


class ShellcheckRunError(Exception):
    def __init__(self, stderr):
        super().__init__()
        self.stderr = stderr


class ShellcheckError(Exception):
    def __init__(self, path):
        super().__init__()
        self.sectionerrors = {}
        self.path = path

    def addfailure(self, section, error):
        self.sectionerrors[section] = error

    def __len__(self):
        return len(self.sectionerrors)


class ShellcheckFailures(Exception):
    def __init__(self, failures=None):
        super().__init__()
        self.failures = set()
        if failures:
            self.failures = set(failures)

    def merge(self, otherfailures):
        self.failures = self.failures.union(otherfailures.failures)

    def __len__(self):
        return len(self.failures)

    def intersection(self, other):
        return self.failures.intersection(other)

    def difference(self, other):
        return self.failures.difference(other)

    def __iter__(self):
        return iter(self.failures)


def checksection(data):
    # spread shell snippets are executed under 'set -e' shell, make sure
    # shellcheck knows about that
    data = "set -eu\n" + data
    proc = subprocess.Popen(
        [SHELLCHECK, "-s", "bash", "-x", "-"],
        stdout=subprocess.PIPE,
        stdin=subprocess.PIPE,
    )
    stdout, _ = proc.communicate(input=data.encode("utf-8"), timeout=10)
    if proc.returncode != 0:
        raise ShellcheckRunError(stdout)


def checkfile(path):
    logging.debug("checking file %s", path)
    with open(path) as inf:
        data = yaml.safe_load(inf)

    errors = ShellcheckError(path)

    for section in SECTIONS & data.keys():
        try:
            logging.debug("%s: checking section %s", path, section)
            checksection(data[section])
        except ShellcheckRunError as serr:
            errors.addfailure(section, serr.stderr.decode("utf-8"))

    if path.endswith("spread.yaml") and "suites" in data:
        # check suites
        for suite_name, suite in data["suites"].items():
            for section in SECTIONS & suite.keys():
                try:
                    logging.debug(
                        "%s (suite %s): checking section %s", path, suite_name, section
                    )
                    checksection(suite[section])
                except ShellcheckRunError as serr:
                    errors.addfailure(
                        "suites/" + suite_name + "/" + section,
                        serr.stderr.decode("utf-8"),
                    )

    if errors:
        raise errors


def findfiles(indir):
    for root, _, files in os.walk(indir, topdown=True):
        for name in files:
            if name in ["spread.yaml", "task.yaml"]:
                yield os.path.join(root, name)


def checkpath(loc, max_workers):
    if os.path.isdir(loc):
        # setup iterator
        locations = findfiles(loc)
    else:
        locations = [loc]

    failed = []

    def check1path(path):
        try:
            checkfile(path)
        except ShellcheckError as err:
            return err
        return None

    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        for serr in executor.map(check1path, locations):
            if serr is None:
                continue
            logging.error(
                ("shellcheck failed for file %s in sections: " "%s; error log follows"),
                serr.path,
                ", ".join(serr.sectionerrors.keys()),
            )
            for section, error in serr.sectionerrors.items():
                logging.error("%s: section '%s':\n%s", serr.path, section, error)
            failed.append(serr.path)

    if failed:
        raise ShellcheckFailures(failures=failed)


def loadfilelist(flistpath):
    flist = set()
    with open(flistpath) as inf:
        for line in inf:
            if not line.startswith("#"):
                flist.add(line.strip())
    return flist


def main(opts):
    paths = opts.paths or ["."]
    failures = ShellcheckFailures()
    for pth in paths:
        try:
            checkpath(pth, opts.max_procs)
        except ShellcheckFailures as sf:
            failures.merge(sf)

    if failures:
        if opts.can_fail:
            can_fail = loadfilelist(opts.can_fail)

            unexpected = failures.difference(can_fail)
            if unexpected:
                logging.error(
                    (
                        "validation failed for the following "
                        "non-whitelisted files:\n%s"
                    ),
                    "\n".join([" - " + f for f in sorted(unexpected)]),
                )
                raise SystemExit(1)

            did_not_fail = can_fail - failures.intersection(can_fail)
            if did_not_fail:
                logging.error(
                    (
                        "the following files are whitelisted "
                        "but validated successfully:\n%s"
                    ),
                    "\n".join([" - " + f for f in sorted(did_not_fail)]),
                )
                raise SystemExit(1)

            # no unexpected failures
            return

        logging.error(
            "validation failed for the following files:\n%s",
            "\n".join([" - " + f for f in sorted(failures)]),
        )

        if NO_FAIL or opts.no_errors:
            logging.warning("ignoring errors")
        else:
            raise SystemExit(1)


if __name__ == "__main__":
    opts = parse_arguments()
    if opts.verbose or D or V:
        subprocess.call([SHELLCHECK, "--version"])
        lvl = logging.DEBUG
    else:
        lvl = logging.INFO
    logging.basicConfig(level=lvl)

    if CAN_FAIL:
        opts.can_fail = CAN_FAIL

    if NO_FAIL:
        opts.no_errors = True

    main(opts)
