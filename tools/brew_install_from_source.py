#!/usr/bin/env python3
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
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

import hashlib
import os
import subprocess
import tempfile
import shutil
import urllib.request


def main():
    try:
        temp_dir = tempfile.mkdtemp()
        compressed_snapcraft_source = download_snapcraft_source(temp_dir)
        compressed_snapcraft_sha256 = sha256_checksum(compressed_snapcraft_source)
        brew_formula_path = os.path.join(temp_dir, "snapcraft.rb")
        download_brew_formula(brew_formula_path)
        patched_dir = os.path.join(temp_dir, "patched")
        os.mkdir(patched_dir)
        brew_formula_from_source_path = os.path.join(patched_dir, "snapcraft.rb")
        patch_brew_formula_source(
            brew_formula_path,
            brew_formula_from_source_path,
            compressed_snapcraft_source,
            compressed_snapcraft_sha256,
        )
        install_brew_formula(brew_formula_from_source_path)
    finally:
        shutil.rmtree(temp_dir)


def download_snapcraft_source(dest_dir):
    dest_file = os.path.join(dest_dir, "snapcraft-0.1.tar.gz")
    branch_source = "https://github.com/{}/archive/{}.tar.gz".format(
        os.environ.get("TRAVIS_PULL_REQUEST_SLUG") or "snapcore/snapcraft",
        os.environ.get("TRAVIS_PULL_REQUEST_BRANCH") or "master",
    )
    print("Downloading branch source from {}".format(branch_source))
    urllib.request.urlretrieve(branch_source, dest_file)
    return dest_file


def sha256_checksum(filename):
    sha256 = hashlib.sha256()
    with open(filename, "rb") as f:
        for block in iter(lambda: f.read(65536), b""):
            sha256.update(block)
    return sha256.hexdigest()


def download_brew_formula(destination_path):
    brew_formula_url = (
        "https://raw.githubusercontent.com/Homebrew/homebrew-core/master/"
        "Formula/snapcraft.rb"
    )
    urllib.request.urlretrieve(brew_formula_url, destination_path)


def patch_brew_formula_source(
    original_formula_path,
    destination_formula_path,
    compressed_source,
    compressed_sha256,
):
    with open(original_formula_path, "r") as original_file:
        with open(destination_formula_path, "w") as destination_file:
            for line in original_file:
                if line.startswith("  url "):
                    destination_file.write(
                        '  url "file://{}"\n'.format(compressed_source)
                    )
                elif line.startswith("  sha256 "):
                    destination_file.write('  sha256 "{}"\n'.format(compressed_sha256))
                else:
                    destination_file.write(line)


def install_brew_formula(formula_path):
    subprocess.check_call(["brew", "install", "--build-from-source", formula_path])


if __name__ == "__main__":
    main()
