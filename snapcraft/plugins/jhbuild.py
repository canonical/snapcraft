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

"""The JHBuild plugin is used for GNOME-based parts.

JHBuild is a tool used to build components of the GNOME ecosystem from version
control. See the JHBuild manual for more details:
https://developer.gnome.org/jhbuild/stable/.

The plugin can be customized with the following keys:

  - modules: (required) a list of modules to include in the part, modules that
             should be skipped should be prefixed with '-', e.g. '-WebKit'
  - module-set: (required) the module set JHBuild should use
  - module-set-dir: the directory where the module set is located
             Either leave unset to use the default moduleset definitions, or
             set to `.` to reference your project folder.
  - jhbuild-archive: the source tarball directory on the build host
  - jhbuild-mirror: the DVCS repository directory on the build host
  - cflags: customises the build by adding into the gcc or G++ command lines
             when building. It is commonly used to add optimisation settings
             (e.g -O3 or -Os) or additional library include paths.

Advice:

  - You only need to specify a single module in most cases. JHBuild knows the
    module dependency graph.

  - The desktop-gtk3 helper is incompatible with GTK built by jhbuild. A custom
    helper is currently required.
    See github.com/diddledan/corebird for example.

  - Building WebKit requires a lot of time and computing power. If WebKit is a
    dependency of your JHBuild module, it can be skipped by adding '-WebKit' to
    the list of modules, and adding 'libwebkit2gtk-4.0-dev' to the
    build-packages and stage-packages.

  - Specify directories on the build host for jhbuild-archive and
    jhbuild-mirror to prevent repeated downloading of the JHBuild module
    sources. It's best to reserve common directories on your local machine that
    can be reused by all snaps you might want to build.

  - Add "-Os" into `cflags` to ensure that the compiled binaries are as small
    as possible.
"""

import logging
import os
import snapcraft

BUILD_PACKAGES = {
    "apt-file",
    "autoconf",
    "automake",
    "autopoint",
    "autotools-dev",
    "bison",
    "build-essential",
    "ca-certificates",
    "cvs",
    "docbook",
    "docbook-xml",
    "docbook-xsl",
    "flex",
    "gettext",
    "git",
    "intltool",
    "iso-codes",
    "libtext-csv-perl",
    "libtool",
    "libxml-parser-perl",
    "make",
    "ninja-build",
    "pkg-config",
    "python-dev",
    "python3-dev",
    "subversion",
    "symlinks",
    "yelp-tools",
    "zlib1g-dev",
}


logger = logging.getLogger(__name__)
jhbuild_repository = "https://git.gnome.org/browse/jhbuild"


class JHBuildPlugin(snapcraft.BasePlugin):
    """JHBuildPlugin provides jhbuild functionality to snapcraft"""

    @classmethod
    def schema(cls):
        schema = super().schema()

        schema["properties"] = {
            "modules": {
                "type": "array",
                "items": {"type": "string"},
                "minItems": 1,
                "uniqueItems": True,
            },
            "module-set": {"type": "string"},
            "module-set-dir": {"type": "string", "default": ""},
            "jhbuild-archive": {"type": "string", "default": ""},
            "jhbuild-mirror": {"type": "string", "default": ""},
            "cflags": {"type": "string", "default": ""},
        }

        schema["required"].append("modules")
        schema["required"].append("module-set")

        return schema

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        return [
            "modules",
            "module-set",
            "module-set-dir",
            "jhbuild-archive",
            "jhbuild-mirror",
        ]

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ["cflags"]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.modules = [module for module in self.options.modules if module[0] != "-"]

        self.skip_modules = [
            module[1:].strip() for module in self.options.modules if module[0] == "-"
        ]

        self.build_packages += list(BUILD_PACKAGES)

        self.jhbuild_src = os.path.join(self.partdir, "jhbuild", "src")
        self.jhbuild_program = os.path.join(
            self.partdir, "jhbuild", "usr", "bin", "jhbuild"
        )
        self.jhbuildrc_path = os.path.join(self.partdir, "jhbuildrc")

    def pull(self):
        logger.info("Pulling JHBuild")

        repository = self.options.jhbuild_mirror or jhbuild_repository

        if os.path.isdir(self.jhbuild_src):
            self.run(["git", "pull"], cwd=self.jhbuild_src)
        else:
            env = {"https_proxy": os.getenv("https_proxy") or ""}
            self.run(["git", "clone", repository, self.jhbuild_src], env=env)

        self._setup_jhbuild()

        self.run_jhbuild(["sanitycheck"], output=False)

        modules = self.run_jhbuild(["list"] + self.modules, output=True).splitlines()

        logger.info("Pulling modules")
        self.run_jhbuild(["update"] + modules, output=False)

    def build(self):
        self._setup_jhbuild()

        logger.info("Building modules")
        self.run_jhbuild(["build"] + self.modules, output=False)

        logger.info("Fixing symbolic links")
        self.run(["symlinks", "-c", "-d", "-r", "-s", "-v", self.installdir])

    def _setup_jhbuild(self):
        if not os.path.isfile(self.jhbuildrc_path):
            self._write_jhbuildrc()

        archive_path = os.path.join(self.partdir, "jhbuild", "packages")
        mirror_path = os.path.join(self.partdir, "jhbuild", "mirror")
        unpacked_path = os.path.join(self.partdir, "jhbuild", "unpacked")

        make_paths = [
            self.builddir,
            self.installdir,
            self.partdir,
            archive_path,
            mirror_path,
            unpacked_path,
        ]

        for folder in make_paths:
            os.makedirs(folder, exist_ok=True)

        if not os.path.exists(self.jhbuild_program):
            logger.info("Building JHBuild")

            self.maybe_sudo(
                [
                    "./autogen.sh",
                    "--prefix=%s" % os.sep
                    + os.path.join(self.partdir, "jhbuild", "usr"),
                ],
                cwd=self.jhbuild_src,
            )

            self.maybe_sudo(
                ["make", "-j%d" % self.parallel_build_count], cwd=self.jhbuild_src
            )

            self.maybe_sudo(
                ["make", "-j%d" % self.parallel_build_count, "install"],
                cwd=self.jhbuild_src,
            )

    def _write_jhbuildrc(self):
        """
        _write_jhbuildrc() - write out the jhbuildrc file for build dependency
        specification
        """
        with open(self.jhbuildrc_path, "w") as jhbuildrc_file:
            config = """
moduleset = {module_set!r}
modulesets_dir = {modulesets_dir!r}
tarballdir = {tarballdir!r}
dvcs_mirror_dir = {dvcs_mirror_dir!r}
checkoutroot = {checkoutroot!r}
prefix = {prefix!r}
buildroot = {buildroot!r}
use_local_modulesets = True
skip = [{skip!s}]
module_autogenargs['gdk-pixbuf'] = '--disable-gio-sniffing'
extra_prefixes = [{extra_prefixes!s}]
cflags = {cflags!r}
"""

            extra_prefixes = [
                os.path.join(self.project.stage_dir, "usr"),
                os.path.join(self.project.stage_dir, "usr", "local"),
            ]

            jhbuildrc_file.write(
                config.format(
                    module_set=self.options.module_set,
                    modulesets_dir=self.options.module_set_dir,
                    tarballdir=self.options.jhbuild_archive
                    or os.path.join(self.partdir, "jhbuild", "packages"),
                    dvcs_mirror_dir=os.path.join(self.partdir, "jhbuild", "mirror"),
                    checkoutroot=os.path.join(self.partdir, "jhbuild", "unpacked"),
                    buildroot=os.path.join(self.builddir, "jhbuild"),
                    prefix=os.path.join(self.installdir, "usr"),
                    skip=", ".join(["'%s'" % module for module in self.skip_modules]),
                    extra_prefixes=", ".join(
                        ["'%s'" % prefix for prefix in extra_prefixes]
                    ),
                    cflags=self.options.cflags,
                )
            )

    def run_jhbuild(self, args, output=True, **kwargs):
        """Run JHBuild in the build step.
        :param list args: command arguments without executable
        :return: the output of the command if captured
        :rtype: str
        """
        cmd = [self.jhbuild_program, "--no-interact", "-f", self.jhbuildrc_path]

        return self.maybe_sudo(cmd + args, output=output, **kwargs)

    def maybe_sudo(self, args, output=True, **kwargs):
        """Run a command with sudo if we're root to drop privileges"""
        cwd = kwargs.pop("cwd", os.getcwd())

        env = os.environ.copy()
        env["JHBUILD_RUN_AS_ROOT"] = "1"

        run = self.run_output if output else self.run
        return run(args, cwd=cwd, env=env, **kwargs)
