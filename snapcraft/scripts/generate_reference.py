#!/usr/bin/env python3
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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
import sys

DEFAULT_REFERENCE_FILE = "docs/reference.md"


def _get_help_output(topic, lines=False):
    TRANSLATE = {"tar-content": "tar_content"}
    if topic in TRANSLATE:
        topic = TRANSLATE[topic]
    return _run_snapcraft(["help", topic], lines)


def _run_snapcraft(cmd, lines=False):
    cmd = ["./snapcraft"] + cmd
    output = subprocess.check_output(
        cmd, cwd=os.path.join(os.path.dirname(__file__), "../../bin")
    )
    output = output.decode("utf-8")
    if lines:
        return output.split("\n")[:-1]  # drop last empty line
    return output


class Reference:
    def __init__(self, output_file):
        self.docs = []
        self.topics = []
        self.plugins = []
        self.output_file = output_file
        assert os.path.isdir(
            os.path.dirname(self.output_file)
        ), "Directory {} does not exist.".format(os.path.dirname(self.output_file))
        self._read()
        self._write()

    def _read(self):
        self.topics = _get_help_output("topics", lines=True)
        self.plugins = _run_snapcraft(["list-plugins"], lines=True)
        self.topics.extend(self.plugins)
        for topic in self.topics:
            text = _get_help_output(topic)
            if not text.startswith("The plugin has no documentation"):
                self.docs += [(topic, text)]

    def _write(self):
        reference_text = """
# Snapcraft reference

`snapcraft` comes with a handy help system, just run:
```
$ snapcraft help
```
to get you started.

Here we want to provide you with the current documentation from within
snappy. It's a reference of `snapcraft`'s plugins and internals.

## Internal documentation
"""
        for entry in [e for e in self.docs if e[0] not in self.plugins]:
            reference_text += """
### Topic: {}

{}""".format(
                entry[0].title(), entry[1].replace("# ", "#### ")
            )
        reference_text += """

## snapcraft's Plugins

In this section we are going to discuss `snapcraft`'s plugins, their workflow
and their options.
"""
        for entry in [e for e in self.docs if e[0] in self.plugins]:
            reference_text += """
### The {} plugin

{}""".format(
                entry[0], entry[1]
            )
        with open(os.path.join(self.output_file), "w") as f:
            f.write(str(reference_text))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-O",
        "--output-file",
        help="specify output file, default: {}".format(DEFAULT_REFERENCE_FILE),
        default=DEFAULT_REFERENCE_FILE,
    )
    args = parser.parse_args()
    Reference(args.output_file)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Aborted.")
        sys.exit(1)
