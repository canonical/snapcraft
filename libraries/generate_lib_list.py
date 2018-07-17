#!/usr/bin/env python3

import os
import re
import subprocess
import sys
import tempfile

from snapcraft.config import load_config
from snapcraft.storeapi import download


def generate_list(file_path):
    print("Generating library list")
    output = subprocess.check_output(["unsquashfs", "-l", file_path])
    file_list = output.decode("utf-8").split("\n")
    lib_regex = re.compile(".*/lib.*.so\..*$")
    lib_list = [os.path.basename(l) for l in file_list if lib_regex.match(l)]
    lib_list.sort()

    return lib_list


def main():
    if len(sys.argv) != 2:
        print("usage {} <file>".format(sys.argv[0]))
        sys.exit(1)
    target_file = sys.argv[1]

    config = load_config()
    with tempfile.NamedTemporaryFile() as temp:
        print("Downloading")
        download("core", "stable", temp.name, config, "amd64")
        lib_list = generate_list(temp.name)

    lib_list = ("{}\n".format(l) for l in lib_list)

    with open(target_file, "w") as f:
        f.writelines(lib_list)


if __name__ == "__main__":
    main()
