# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2019 Canonical Ltd
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
import pkgutil
import shutil
import subprocess
import sys
import textwrap
from types import ModuleType
from typing import Callable
from unittest import mock

import fixtures
import jsonschema

import snapcraft
from snapcraft.internal import elf
from snapcraft.plugins._plugin_finder import get_plugin_for_base
from tests.file_utils import get_snapcraft_path


class FakeProjectOptions(fixtures.Fixture):
    def __init__(self, **kwargs):
        self._kwargs = dict(
            arch_triplet=kwargs.pop("arch_triplet", "x86_64-gnu-linux"),
            parts_dir=kwargs.pop("parts_dir", "parts"),
            stage_dir=kwargs.pop("stage_dir", "stage"),
            prime_dir=kwargs.pop("prime_dir", "prime"),
            parallel_build_count=kwargs.pop("parallel_build_count", "1"),
        )
        if kwargs:
            raise NotImplementedError(
                "Handling of {!r} is not implemented".format(kwargs.keys())
            )

    def setUp(self):
        super().setUp()

        patcher = mock.patch("snapcraft.project.Project")
        patcher.start()
        self.addCleanup(patcher.stop)

        # Special handling is required as ProjectOptions attributes are
        # handled with the @property decorator.
        project_options_t = type(snapcraft.project.Project.return_value)
        for key in self._kwargs:
            setattr(project_options_t, key, self._kwargs[key])


class FakeMetadataExtractor(fixtures.Fixture):
    """Dynamically generate a new module containing the provided extractor"""

    def __init__(
        self,
        extractor_name: str,
        extractor: Callable[[str], snapcraft.extractors.ExtractedMetadata],
        exported_name="extract",
    ) -> None:
        super().__init__()
        self._extractor_name = extractor_name
        self._exported_name = exported_name
        self._import_name = "snapcraft.extractors.{}".format(extractor_name)
        self._extractor = extractor

    def _setUp(self) -> None:
        extractor_module = ModuleType(self._import_name)
        setattr(extractor_module, self._exported_name, self._extractor)
        sys.modules[self._import_name] = extractor_module
        self.addCleanup(self._remove_module)

        real_iter_modules = pkgutil.iter_modules

        def _fake_iter_modules(path):
            if path == snapcraft.extractors.__path__:
                yield None, self._extractor_name, False
            else:
                yield real_iter_modules(path)

        patcher = mock.patch("pkgutil.iter_modules", new=_fake_iter_modules)
        patcher.start()
        self.addCleanup(patcher.stop)

    def _remove_module(self) -> None:
        del sys.modules[self._import_name]


class FakePlugin(fixtures.Fixture):
    """Dynamically generate a new module containing the provided plugin"""

    def __init__(self, plugin_name, plugin_class):
        super().__init__()
        self._plugin_name = plugin_name
        self._plugin_class = plugin_class

    def _setUp(self):
        self.useFixture(
            fixtures.MockPatch(
                "snapcraft.plugins.get_plugin_for_base", side_effect=self.get_plugin
            )
        )

    def get_plugin(self, plugin_name, *, build_base):
        if plugin_name == self._plugin_name:
            plugin_class = self._plugin_class
        else:
            plugin_class = get_plugin_for_base(self._plugin_name, build_base=build_base)

        return plugin_class


def _fake_elffile_extract_attributes(self):
    name = os.path.basename(self.path)

    self.arch = ("ELFCLASS64", "ELFDATA2LSB", "EM_X86_64")
    self.build_id = "build-id-{}".format(name)

    if name in [
        "fake_elf-2.26",
        "fake_elf-bad-ldd",
        "fake_elf-with-core-libs",
        "fake_elf-with-missing-libs",
        "fake_elf-bad-patchelf",
        "fake_elf-with-host-libraries",
    ]:
        glibc = elf.NeededLibrary(name="libc.so.6")
        glibc.add_version("GLIBC_2.2.5")
        glibc.add_version("GLIBC_2.26")

        self.interp = "/lib64/ld-linux-x86-64.so.2"
        self.soname = ""
        self.versions = set()
        self.needed = {glibc.name: glibc}
        self.execstack_set = False
        self.is_dynamic = True
        self.has_debug_info = False

    elif name == "fake_elf-2.23":
        glibc = elf.NeededLibrary(name="libc.so.6")
        glibc.add_version("GLIBC_2.2.5")
        glibc.add_version("GLIBC_2.23")

        self.interp = "/lib64/ld-linux-x86-64.so.2"
        self.soname = ""
        self.versions = set()
        self.needed = {glibc.name: glibc}
        self.execstack_set = False
        self.is_dynamic = True
        self.has_debug_info = False

    elif name == "fake_elf-1.1":
        glibc = elf.NeededLibrary(name="libc.so.6")
        glibc.add_version("GLIBC_1.1")
        glibc.add_version("GLIBC_0.1")

        self.interp = "/lib64/ld-linux-x86-64.so.2"
        self.soname = ""
        self.versions = set()
        self.needed = {glibc.name: glibc}
        self.execstack_set = False
        self.is_dynamic = True
        self.has_debug_info = False

    elif name == "fake_elf-static":
        self.interp = "/lib64/ld-linux-x86-64.so.2"
        self.soname = ""
        self.versions = set()
        self.needed = dict()
        self.execstack_set = False
        self.is_dynamic = False
        self.has_debug_info = False

    elif name == "fake_elf-shared-object":
        openssl = elf.NeededLibrary(name="libssl.so.1.0.0")
        openssl.add_version("OPENSSL_1.0.0")

        self.interp = ""
        self.soname = "libfake_elf.so.0"
        self.versions = set()
        self.needed = {openssl.name: openssl}
        self.execstack_set = False
        self.is_dynamic = True
        self.has_debug_info = False

    elif name == "fake_elf-with-execstack":
        glibc = elf.NeededLibrary(name="libc.so.6")
        glibc.add_version("GLIBC_2.23")

        self.interp = "/lib64/ld-linux-x86-64.so.2"
        self.soname = ""
        self.versions = set()
        self.needed = {glibc.name: glibc}
        self.execstack_set = True
        self.is_dynamic = True
        self.has_debug_info = False

    elif name == "fake_elf-with-bad-execstack":
        glibc = elf.NeededLibrary(name="libc.so.6")
        glibc.add_version("GLIBC_2.23")

        self.interp = "/lib64/ld-linux-x86-64.so.2"
        self.soname = ""
        self.versions = set()
        self.needed = {glibc.name: glibc}
        self.execstack_set = True
        self.is_dynamic = True
        self.has_debug_info = False

    elif name == "libc.so.6":
        self.interp = ""
        self.soname = "libc.so.6"
        self.versions = {"libc.so.6", "GLIBC_2.2.5", "GLIBC_2.23", "GLIBC_2.26"}
        self.needed = {}
        self.execstack_set = False
        self.is_dynamic = True
        self.has_debug_info = False

    elif name == "libssl.so.1.0.0":
        self.interp = ""
        self.soname = "libssl.so.1.0.0"
        self.versions = {"libssl.so.1.0.0", "OPENSSL_1.0.0"}
        self.needed = {}
        self.execstack_set = False
        self.is_dynamic = True
        self.has_debug_info = False

    else:
        self.interp = ""
        self.soname = ""
        self.versions = set()
        self.needed = {}
        self.execstack_set = False
        self.is_dynamic = True
        self.has_debug_info = False


class FakeElf(fixtures.Fixture):
    def __getitem__(self, item):
        return self._elf_files[item]

    def __init__(self, *, root_path, patchelf_version="0.10"):
        super().__init__()

        self.root_path = root_path
        self.core_base_path = None
        self._patchelf_version = patchelf_version

    def _setUp(self):
        super()._setUp()

        self.core_base_path = self.useFixture(fixtures.TempDir()).path

        binaries_path = os.path.join(get_snapcraft_path(), "tests", "bin", "elf")

        new_binaries_path = self.useFixture(fixtures.TempDir()).path
        current_path = os.environ.get("PATH")
        new_path = "{}:{}".format(new_binaries_path, current_path)
        self.useFixture(fixtures.EnvironmentVariable("PATH", new_path))

        # Copy strip
        for f in ["strip", "execstack"]:
            shutil.copy(
                os.path.join(binaries_path, f), os.path.join(new_binaries_path, f)
            )
            os.chmod(os.path.join(new_binaries_path, f), 0o755)

        # Some values in ldd need to be set with core_path
        with open(os.path.join(binaries_path, "ldd")) as rf:
            with open(os.path.join(new_binaries_path, "ldd"), "w") as wf:
                for line in rf.readlines():
                    wf.write(line.replace("{CORE_PATH}", self.core_base_path))
        os.chmod(os.path.join(new_binaries_path, "ldd"), 0o755)

        # Some values in ldd need to be set with core_path
        self.patchelf_path = os.path.join(new_binaries_path, "patchelf")
        with open(os.path.join(binaries_path, "patchelf")) as rf:
            with open(self.patchelf_path, "w") as wf:
                for line in rf.readlines():
                    wf.write(line.replace("{VERSION}", self._patchelf_version))
        os.chmod(os.path.join(new_binaries_path, "patchelf"), 0o755)

        patcher = mock.patch.object(
            elf.ElfFile,
            "_extract_attributes",
            new_callable=lambda: _fake_elffile_extract_attributes,
        )
        patcher.start()
        self.addCleanup(patcher.stop)

        self._elf_files = {
            "fake_elf-2.26": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-2.26")
            ),
            "fake_elf-2.23": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-2.23")
            ),
            "fake_elf-1.1": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-1.1")
            ),
            "fake_elf-static": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-static")
            ),
            "fake_elf-shared-object": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-shared-object")
            ),
            "fake_elf-with-host-libraries": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-with-host-libraries")
            ),
            "fake_elf-bad-ldd": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-bad-ldd")
            ),
            "fake_elf-bad-patchelf": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-bad-patchelf")
            ),
            "fake_elf-with-core-libs": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-with-core-libs")
            ),
            "fake_elf-with-missing-libs": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-with-missing-libs")
            ),
            "fake_elf-with-execstack": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-with-execstack")
            ),
            "fake_elf-with-bad-execstack": elf.ElfFile(
                path=os.path.join(self.root_path, "fake_elf-with-bad-execstack")
            ),
            "libc.so.6": elf.ElfFile(path=os.path.join(self.root_path, "libc.so.6")),
            "libssl.so.1.0.0": elf.ElfFile(
                path=os.path.join(self.root_path, "libssl.so.1.0.0")
            ),
        }

        for elf_file in self._elf_files.values():
            with open(elf_file.path, "wb") as f:
                f.write(b"\x7fELF")
                if elf_file.path.endswith("fake_elf-bad-patchelf"):
                    f.write(b"nointerpreter")

        self.root_libraries = {
            "foo.so.1": os.path.join(self.root_path, "foo.so.1"),
            "moo.so.2": os.path.join(self.root_path, "non-standard", "moo.so.2"),
        }

        barsnap_elf = os.path.join(self.core_base_path, "barsnap.so.2")
        elf_list = [*self.root_libraries.values(), barsnap_elf]

        for root_library in elf_list:
            os.makedirs(os.path.dirname(root_library), exist_ok=True)
            with open(root_library, "wb") as f:
                f.write(b"\x7fELF")


class FakeExtension(fixtures.Fixture):
    """Dynamically generate a new module containing the provided extension"""

    def __init__(self, extension_name, extension_class):
        super().__init__()
        self._import_name = "snapcraft.internal.project_loader._extensions.{}".format(
            extension_name
        )
        self._extension_class = extension_class
        self._extension_name = extension_name

    def _setUp(self):
        extension_module = ModuleType(self._import_name)
        setattr(extension_module, self._extension_class.__name__, self._extension_class)
        sys.modules[self._import_name] = extension_module
        self.addCleanup(self._remove_module)

        json_schema_validator = jsonschema.validate

        def validate(snapcraft, schema, *, format_checker=None):
            try:
                json_schema_validator(snapcraft, schema, format_checker=format_checker)
            except jsonschema.exceptions.ValidationError as error:
                # Ignore validation errors arising from extension usage.
                if (
                    error.instance == self._extension_name
                    and error.path[0] == "apps"
                    and error.path[2] == "extensions"
                ):
                    return None
                raise

        self.useFixture(fixtures.MonkeyPatch("jsonschema.validate", validate))

    def _remove_module(self):
        del sys.modules[self._import_name]


class FakeSnapCommand(fixtures.Fixture):
    def __init__(self):
        self.calls = []
        self.install_success = True
        self.refresh_success = True
        self.download_side_effect = None
        self._email = "-"

    def _setUp(self):
        original_check_call = snapcraft.internal.repo.snaps.check_call
        original_check_output = snapcraft.internal.repo.snaps.check_output

        def side_effect_check_call(cmd, *args, **kwargs):
            return side_effect(original_check_call, cmd, *args, **kwargs)

        def side_effect_check_output(cmd, *args, **kwargs):
            if self._is_snap_command(cmd):
                self.calls.append(cmd)
                return self._fake_snap_command(cmd, *args, **kwargs)
            else:
                return side_effect(original_check_output, cmd, *args, **kwargs)

        def side_effect(original, cmd, *args, **kwargs):
            if self._is_snap_command(cmd):
                self.calls.append(cmd)
                return self._fake_snap_command(cmd, *args, **kwargs)
            else:
                return original(cmd, *args, **kwargs)

        self.useFixture(
            fixtures.MonkeyPatch(
                "snapcraft.internal.repo.snaps.check_call", side_effect_check_call
            )
        )
        self.useFixture(
            fixtures.MonkeyPatch(
                "snapcraft.internal.repo.snaps.check_output", side_effect_check_output
            )
        )

    def login(self, email):
        self._email = email

    def _get_snap_cmd(self, snap_cmd):
        try:
            snap_cmd_index = snap_cmd.index("snap")
        except ValueError:
            return ""

        try:
            return snap_cmd[snap_cmd_index + 1]
        except IndexError:
            return ""

    def _is_snap_command(self, cmd):
        return self._get_snap_cmd(cmd) in ["install", "refresh", "whoami", "download"]

    def _fake_snap_command(self, cmd, *args, **kwargs):
        cmd = self._get_snap_cmd(cmd)
        if cmd == "install" and not self.install_success:
            raise subprocess.CalledProcessError(returncode=1, cmd=cmd)
        elif cmd == "refresh" and not self.refresh_success:
            raise subprocess.CalledProcessError(returncode=1, cmd=cmd)
        elif cmd == "whoami":
            return "email: {}".format(self._email).encode()
        elif (
            cmd == "download"
            and self.download_side_effect is not None
            and not self.download_side_effect.pop(0)
        ):
            raise subprocess.CalledProcessError(returncode=1, cmd=cmd)
        elif cmd == "download":
            return "Downloaded  ".encode()


class FakeSnapcraftctl(fixtures.Fixture):
    def _setUp(self):
        super()._setUp()

        snapcraft_path = get_snapcraft_path()

        tempdir = self.useFixture(fixtures.TempDir()).path
        altered_path = "{}:{}".format(tempdir, os.environ.get("PATH"))
        self.useFixture(fixtures.EnvironmentVariable("PATH", altered_path))

        snapcraftctl_path = os.path.join(tempdir, "snapcraftctl")
        with open(snapcraftctl_path, "w") as f:
            f.write(
                textwrap.dedent(
                    """\
                #!/usr/bin/env python3

                # Make sure we can find snapcraft, even if it's not installed
                # (like in CI).
                import sys
                sys.path.append('{snapcraft_path!s}')

                import snapcraft.cli.__main__

                if __name__ == '__main__':
                    snapcraft.cli.__main__.run_snapcraftctl(
                        prog_name='snapcraftctl')
            """.format(
                        snapcraft_path=snapcraft_path
                    )
                )
            )
            f.flush()

        os.chmod(snapcraftctl_path, 0o755)


class FakeMultipass(fixtures.Fixture):
    def _setUp(self):
        super()._setUp()

        tempdir = self.useFixture(fixtures.TempDir()).path
        altered_path = "{}:{}".format(tempdir, os.environ.get("PATH"))
        self.useFixture(fixtures.EnvironmentVariable("PATH", altered_path))

        multipass = os.path.join(tempdir, "multipass")
        os.symlink("/bin/true", multipass)
