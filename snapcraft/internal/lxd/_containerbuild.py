#!/usr/bin/python3
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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

import collections
import json
import logging
import os
import pipes
import sys
import contextlib
import subprocess
import time
from textwrap import dedent
from typing import List

from xdg import BaseDirectory

from . import errors
from ._lxc_command import LXDInstanceProvider
from snapcraft.project import Project
from snapcraft.internal.errors import InvalidContainerImageInfoError
from snapcraft.project._project_options import _get_deb_arch
from snapcraft.internal.build_providers._snap import SnapInjector


logger = logging.getLogger(__name__)

_NETWORK_PROBE_COMMAND = dedent(
    """
    import urllib.request
    import sys

    check_url = "http://start.ubuntu.com/connectivity-check.html"
    try:
        urllib.request.urlopen(check_url, timeout=5)
    except urllib.error.URLError as e:
        sys.exit('Failed to open {!r}: {!s}'.format(check_url, e.reason))
    except Exception as e:
        sys.exit('Failed to open {!r}: {!s}'.format(check_url, e))
    """
)
_PROXY_KEYS = ["http_proxy", "https_proxy", "no_proxy", "ftp_proxy"]


class Containerbuild:
    def __init__(
        self,
        *,
        source: str,
        project: Project,
        container_name: str,
        output: str = None,
        remote: str = None
    ) -> None:
        if project.info.architectures is None:
            architecture = project.deb_arch
        elif len(project.info.architectures) == 1:
            architecture = project.info.architectures[0]
        else:
            architecture = "multi"

        if output is None and project.info.version:
            output = "{}_{}_{}.snap".format(
                project.info.name, project.info.version, architecture
            )
        elif output is None:
            output = "{}_{}.snap".format(project.info.name, architecture)
        self.snap_filename = output

        self._source = os.path.realpath(source)
        self._project = project
        self._user = "root"
        self._project_folder = "/root/build_{}".format(project.info.name)

        if remote is None:
            remote = _get_default_remote()
        self._remote = remote

        self._container_name = "snapcraft-{}".format(container_name)
        self._image = "ubuntu:xenial"
        # Use a temporary folder the 'lxd' snap can access
        self._lxd_common_dir = os.path.expanduser(
            os.path.join("~", "snap", "lxd", "common")
        )
        os.makedirs(self._lxd_common_dir, exist_ok=True)

        self._lxd_instance = LXDInstanceProvider(
            instance_name="{}:{}".format(remote, self._container_name)
        )

        # TODO migrate to an actual provider
        self.provider_project_dir = os.path.join(
            BaseDirectory.save_data_path("snapcraft"),
            "projects",
            "lxd",
            project.info.name,
        )

    @contextlib.contextmanager
    def _container_running(self):
        with self._ensure_started():
            try:
                yield
            except errors.ContainerRunError as e:
                self._finish(success=False)
                if self._project.debug:
                    logger.info("Debug mode enabled, dropping into a shell")
                    self._container_run(["bash", "-i"])
                else:
                    raise e
            finally:
                self._finish()

    def _ensure_remote(self):
        _verify_remote(self._remote)
        self._container_name = "{}:{}".format(self._remote, self._container_name)

    @contextlib.contextmanager
    def _ensure_started(self):
        self._ensure_remote()
        try:
            self._ensure_container()
            yield
        finally:
            status = self._get_container_status()
            if status and status["status"] == "Running":
                # Stopping takes a while and lxc doesn't print anything.
                print("Stopping {}".format(self._container_name))
                subprocess.check_call(["lxc", "stop", "-f", self._container_name])

    def _get_container_status(self):
        containers = json.loads(
            subprocess.check_output(
                ["lxc", "list", "--format=json", self._container_name]
            ).decode()
        )
        for container in containers:
            if container["name"] == self._container_name.split(":")[-1]:
                return container

    def _configure_container(self):
        subprocess.check_call(
            [
                "lxc",
                "config",
                "set",
                self._container_name,
                "raw.idmap",
                "both {} {}".format(os.getenv("SUDO_UID", os.getuid()), 0),
            ]
        )
        subprocess.check_call(
            [
                "lxc",
                "config",
                "set",
                self._container_name,
                "environment.SNAPCRAFT_SETUP_CORE",
                "1",
            ]
        )
        subprocess.check_call(
            [
                "lxc",
                "config",
                "set",
                self._container_name,
                "environment.SNAPCRAFT_MANAGED_HOST",
                "yes",
            ]
        )
        for snapcraft_env_var in (
            "SNAPCRAFT_ENABLE_SILENT_REPORT",
            "SNAPCRAFT_PARTS_URI",
            "SNAPCRAFT_BUILD_INFO",
        ):
            if os.getenv(snapcraft_env_var):
                subprocess.check_call(
                    [
                        "lxc",
                        "config",
                        "set",
                        self._container_name,
                        "environment.{}".format(snapcraft_env_var),
                        os.getenv(snapcraft_env_var),
                    ]
                )
        # Necessary to read asset files with non-ascii characters.
        subprocess.check_call(
            [
                "lxc",
                "config",
                "set",
                self._container_name,
                "environment.LC_ALL",
                "C.UTF-8",
            ]
        )
        self._set_image_info_env_var()

    def _set_image_info_env_var(self):
        FAILURE_WARNING_FORMAT = (
            "Failed to get container image info: {}\n"
            "It will not be recorded in manifest."
        )
        try:
            # This command takes the same image name as used to create a new
            # container. But we must always use the form distro:series/arch
            # here so that we get only the image we're actually using!
            image_info_command = [
                "lxc",
                "image",
                "list",
                "--format=json",
                "{}/{}".format(self._image, self._get_container_arch()),
            ]
            image_info = json.loads(
                subprocess.check_output(image_info_command).decode()
            )
        except subprocess.CalledProcessError as e:
            message = (
                "`{command}` returned with exit code {returncode}, "
                "output: {output}".format(
                    command=" ".join(image_info_command),
                    returncode=e.returncode,
                    output=e.output,
                )
            )
            logger.warning(FAILURE_WARNING_FORMAT.format(message))
            return
        except json.decoder.JSONDecodeError as e:
            logger.warning(FAILURE_WARNING_FORMAT.format("Not in JSON format"))
            return
        edited_image_info = collections.OrderedDict()
        for field in ("fingerprint", "architecture", "created_at"):
            if field in image_info[0]:
                edited_image_info[field] = image_info[0][field]

        # Pick up existing image info if set
        image_info_str = os.environ.get("SNAPCRAFT_IMAGE_INFO")
        if image_info_str:
            try:
                edited_image_info.update(json.loads(image_info_str))
            except json.decoder.JSONDecodeError as e:
                raise InvalidContainerImageInfoError(image_info_str) from e

        # Pass the image info to the container so it can be used when recording
        # information about the build environment.
        subprocess.check_call(
            [
                "lxc",
                "config",
                "set",
                self._container_name,
                "environment.SNAPCRAFT_IMAGE_INFO",
                json.dumps(edited_image_info),
            ]
        )

    def execute(self, step: str = "snap", args=None):
        with self._container_running():
            self._setup_project()
            command = ["snapcraft", step]
            if step == "snap":
                command += ["--output", self.snap_filename]
            # Pass on target arch if specified
            # If not specified it defaults to the LXD architecture
            if self._project.target_arch:
                command += ["--target-arch", self._project.target_arch]
            if args:
                command += args
            self._container_run(command, cwd=self._project_folder, user=self._user)

    def _container_run(
        self, cmd: List[str], cwd=None, user="root", hide_output=False, **kwargs
    ):
        sh = ""
        original_cmd = cmd.copy()
        if cwd:
            sh += "cd {}; ".format(cwd)
        if sh:
            cmd = [
                "sh",
                "-c",
                "{}{}".format(sh, " ".join(pipes.quote(arg) for arg in cmd)),
            ]
        if user != "root":
            cmd = ["sudo", "-H", "-E", "-u", user] + cmd
        if hide_output:
            runnable = subprocess.check_output
        else:
            runnable = subprocess.check_call  # type: ignore
        try:
            runnable(["lxc", "exec", self._container_name, "--"] + cmd, **kwargs)
        except subprocess.CalledProcessError as e:
            if original_cmd[0] == "snapcraft":
                raise errors.ContainerSnapcraftCmdError(
                    command=original_cmd, exit_code=e.returncode
                )
            else:
                raise errors.ContainerRunError(
                    command=original_cmd, exit_code=e.returncode
                )

    def _finish(self, success=True):
        """Run any cleanup tasks."""

    def _setup_project(self):
        # Must be implemented by subclasses
        raise NotImplementedError

    def _wait_for_network(self):
        logger.info("Waiting for a network connection...")
        not_connected = True
        retry_count = 5
        while not_connected:
            time.sleep(5)
            try:
                self._container_run(["python3", "-c", _NETWORK_PROBE_COMMAND])
                not_connected = False
            except errors.ContainerRunError as e:
                retry_count -= 1
                if retry_count == 0:
                    raise errors.ContainerNetworkError("start.ubuntu.com")
        logger.info("Network connection established")

    def _wait_for_cloud_init(self) -> None:
        self._container_run(["cloud-init", "status", "--wait"], hide_output=True)

    def _inject_snapcraft(self):
        registry_filepath = os.path.join(
            self.provider_project_dir, "snap-registry.yaml"
        )

        snap_injector = SnapInjector(
            snap_dir=self._lxd_instance._SNAPS_MOUNTPOINT,
            registry_filepath=registry_filepath,
            runner=self._lxd_instance.run,
            snap_dir_mounter=self._lxd_instance.mount_snaps_directory,
            snap_dir_unmounter=self._lxd_instance.unmount_snaps_directory,
            file_pusher=self._lxd_instance.push_file,
            snap_arch=self._get_container_arch(),
            # We cannot inject in unpriviledged containers without going an extra
            # mile.
            # TODO work with the snapd team for proper API to retrieve the current
            #      snap installed on a host.
            inject_from_host=False,
        )

        snap_injector.add(snap_name="core")
        snap_injector.add(snap_name="snapcraft")
        snap_injector.apply()

    def _get_container_arch(self):
        info = subprocess.check_output(["lxc", "info", self._container_name]).decode(
            "utf-8"
        )
        for line in info.splitlines():
            if line.startswith("Architecture:"):
                with contextlib.suppress(IndexError, KeyError):
                    return _get_deb_arch(line.split(None, 1)[1].strip())
        raise errors.ContainerArchitectureError(info)


def _get_default_remote():
    """Query and return the default lxd remote.

    Use the lxc command to query for the default lxd remote. In most
    cases this will return the local remote.

    :returns: default lxd remote.
    :rtype: string.
    :raises snapcraft.internal.errors.ContainerConnectionError:
        raised if the lxc call fails.
    """
    try:
        default_remote = subprocess.check_output(["lxc", "remote", "get-default"])
    except FileNotFoundError:
        raise errors.ContainerLXDNotInstalledError()
    except subprocess.CalledProcessError:
        raise errors.ContainerLXDSetupError()
    return default_remote.decode(sys.getfilesystemencoding()).strip()


def _remote_is_valid(remote):
    """Verify that the given string is a valid remote name

    :param str remote: the LXD remote to verify.
    """
    # No colon because it separates remote from container name
    # No slash because it's used for images
    # No spaces
    return not (":" in remote or " " in remote or "/" in remote)


def _verify_remote(remote):
    """Verify that the lxd remote exists.

    :param str remote: the lxd remote to verify.
    :raises snapcraft.internal.errors.ContainerConnectionError:
        raised if the lxc call listing the remote fails.
    """
    # There is no easy way to grep the results from `lxc remote list`
    # so we try and execute a simple operation against the remote.
    try:
        subprocess.check_output(["lxc", "list", "{}:".format(remote)])
    except FileNotFoundError:
        raise errors.ContainerLXDNotInstalledError()
    except subprocess.CalledProcessError as e:
        raise errors.ContainerLXDRemoteNotFoundError(remote) from e
