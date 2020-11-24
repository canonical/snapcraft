# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

import gzip
import logging
import os
import shutil
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence, Type
from urllib.parse import unquote, urlsplit

import requests
from launchpadlib.launchpad import Launchpad
from lazr import restfulclient
from lazr.restfulclient.resource import Entry
from xdg import BaseDirectory

import snapcraft
from snapcraft.internal.sources._git import Git
from snapcraft.internal.sources.errors import SnapcraftPullError
from snapcraft.project import Project

from . import errors

_LP_POLL_INTERVAL = 30
_LP_SUCCESS_STATUS = "Successfully built"
_LP_FAIL_STATUS = "Failed to build"

logger = logging.getLogger(__name__)


def _is_build_pending(build: Dict[str, Any]) -> bool:
    # Possible values according to API documentation:
    # - "Needs building"
    # - "Successfully built"
    # - "Failed to build"
    # - "Dependency wait"
    # - "Chroot problem"
    # - "Build for superseded Source"
    # - "Currently building"
    # - "Failed to upload"
    # - "Uploading build"
    # - "Cancelling build"
    # - "Cancelled build"
    if _is_build_status_success(build) or _is_build_status_failure(build):
        return False

    return True


def _is_build_status_success(build: Dict[str, Any]) -> bool:
    build_state = build["buildstate"]
    return build_state == "Successfully built"


def _is_build_status_failure(build: Dict[str, Any]) -> bool:
    build_state = build["buildstate"]
    return build_state in ["Failed to build", "Cancelled build"]


def _get_url_basename(url: str):
    path = urlsplit(url).path
    return unquote(path).split("/")[-1]


class LaunchpadClient:
    """Launchpad remote builder operations."""

    def __init__(
        self,
        *,
        project: Project,
        build_id: str,
        architectures: Sequence[str],
        git_branch: str = "master",
        core18_channel: str = "stable",
        snapcraft_channel: str = "stable",
        deadline: int = 0,
        git_class: Type[Git] = Git,
        running_snapcraft_version: str = snapcraft.__version__,
    ) -> None:
        self._git_class = git_class
        if not self._git_class.check_command_installed():
            raise errors.GitNotFoundProviderError(provider="Launchpad")

        self._snap_name = project.info.name
        self._build_id = build_id

        self.architectures = architectures

        self._lp_name = build_id
        self._lp_git_branch = git_branch

        self._core18_channel = core18_channel
        self._snapcraft_channel = snapcraft_channel
        self._running_snapcraft_version = running_snapcraft_version

        self._cache_dir = self._create_cache_directory()
        self._data_dir = self._create_data_directory()
        self._credentials = os.path.join(self._data_dir, "credentials")

        self._lp: Launchpad = self.login()
        self.user = self._lp.me.name

        self.deadline = deadline

    @property
    def architectures(self) -> Sequence[str]:
        return self._architectures

    @architectures.setter
    def architectures(self, architectures: Sequence[str]) -> None:
        self._lp_processors: Optional[Sequence[str]] = None

        if architectures:
            self._lp_processors = ["/+processors/" + a for a in architectures]

        self._architectures = architectures

    @property
    def user(self) -> str:
        return self._lp_user

    @user.setter
    def user(self, user: str) -> None:
        self._lp_user = user
        self._lp_owner = f"/~{user}"

    def _check_timeout_deadline(self) -> None:
        if self.deadline <= 0:
            return

        if int(time.time()) >= self.deadline:
            raise errors.RemoteBuildTimeoutError()

    def _create_data_directory(self) -> str:
        data_dir = BaseDirectory.save_data_path("snapcraft", "provider", "launchpad")
        os.makedirs(data_dir, mode=0o700, exist_ok=True)
        return data_dir

    def _create_cache_directory(self) -> str:
        cache_dir = BaseDirectory.save_cache_path("snapcraft", "provider", "launchpad")
        os.makedirs(cache_dir, mode=0o700, exist_ok=True)
        return cache_dir

    def _fetch_artifacts(self, snap: Entry) -> None:
        """Fetch build arftifacts (logs and snaps)."""
        builds = self._get_builds(snap)

        logger.debug("Downloading artifacts...")
        for build in builds:
            self._download_build_artifacts(build)
            self._download_log(build)

    def _get_builds_collection_entry(self, snap: Entry) -> Optional[Entry]:
        logger.debug("Fetching builds collection information from Launchpad...")
        url = snap.builds_collection_link
        return self._lp_load_url(url)

    def _get_builds(self, snap: Entry) -> List[Dict[str, Any]]:
        bc = self._get_builds_collection_entry(snap)
        if bc is None:
            return []

        return bc.entries

    def _get_snap(self) -> Optional[Entry]:
        try:
            return self._lp.snaps.getByName(name=self._lp_name, owner=self._lp_owner)
        except restfulclient.errors.NotFound:
            return None

    def _issue_build_request(self, snap: Entry) -> Entry:
        dist = self._lp.distributions["ubuntu"]
        archive = dist.main_archive
        return snap.requestBuilds(
            archive=archive,
            channels={
                "core18": self._core18_channel,
                "snapcraft": self._snapcraft_channel,
            },
            pocket="Updates",
        )

    def _lp_load_url(self, url: str) -> Entry:
        """Load Launchpad url with a retry in case the connection is lost."""
        try:
            return self._lp.load(url)
        except ConnectionResetError:
            self._lp = self.login()
            return self._lp.load(url)

    def _wait_for_build_request_acceptance(
        self, build_request: Entry, timeout: int = 0
    ) -> None:
        # Not be be confused with the actual build(s), this is
        # ensuring that Launchpad accepts the build request.
        while build_request.status == "Pending":
            # Check to see if we've run out of time.
            self._check_timeout_deadline()

            logger.debug("Waiting on Launchpad build request...")
            logger.debug(
                f"status={build_request.status} error={build_request.error_message}"
            )

            time.sleep(1)

            # Refresh status.
            build_request.lp_refresh()

        if build_request.status == "Failed":
            # Build request failed.
            self.cleanup()
            raise errors.RemoteBuilderError(builder_error=build_request.error_message)
        elif build_request.status != "Completed":
            # Shouldn't end up here.
            self.cleanup()
            raise errors.RemoteBuilderError(
                builder_error="Unknown builder error - reported status: {}".format(
                    build_request.status
                )
            )
        elif not build_request.builds.entries:
            # Shouldn't end up here either.
            self.cleanup()
            raise errors.RemoteBuilderError(
                builder_error="Unknown builder error - no build entries found."
            )

        build_number = _get_url_basename(build_request.self_link)
        logger.debug(f"Build request accepted: {build_number}")

    def login(self) -> Launchpad:
        try:
            return Launchpad.login_with(
                "snapcraft remote-build {}".format(snapcraft.__version__),
                "production",
                self._cache_dir,
                credentials_file=self._credentials,
                version="devel",
            )
        except (ConnectionRefusedError, TimeoutError):
            raise errors.LaunchpadHttpsError()

    def get_git_repo_path(self) -> str:
        return f"~{self._lp_user}/+git/{self._lp_name}"

    def get_git_https_url(self, token: Optional[str] = None) -> str:
        if token:
            return f"https://{self._lp_user}:{token}@git.launchpad.net/~{self._lp_user}/+git/{self._lp_name}/"
        else:
            return f"https://{self._lp_user}@git.launchpad.net/~{self._lp_user}/+git/{self._lp_name}/"

    def _create_git_repository(self, force=False) -> Entry:
        """Create git repository."""
        if force:
            self._delete_git_repository()

        logger.debug(
            f"creating git repo: name={self._lp_name}, owner={self._lp_owner}, target={self._lp_owner}"
        )
        return self._lp.git_repositories.new(
            name=self._lp_name, owner=self._lp_owner, target=self._lp_owner
        )

    def _delete_git_repository(self) -> None:
        """Delete git repository."""
        git_path = self.get_git_repo_path()
        git_repo = self._lp.git_repositories.getByPath(path=git_path)

        # git_repositories.getByPath returns None if git repo does not exist.
        if git_repo is None:
            return

        logger.debug("Deleting source repository from Launchpad...")
        git_repo.lp_delete()

    def _create_snap(self, force=False) -> Entry:
        """Create a snap recipe. Use force=true to replace existing snap."""
        git_url = self.get_git_https_url()

        if force:
            self._delete_snap()

        optional_kwargs = dict()
        if self._lp_processors:
            optional_kwargs["processors"] = self._lp_processors

        logger.debug("Registering snap job on Launchpad...")
        logger.debug(f"url=https://launchpad.net{self._lp_owner}/+snap/{self._lp_name}")

        return self._lp.snaps.new(
            name=self._lp_name,
            owner=self._lp_owner,
            git_repository_url=git_url,
            git_path=self._lp_git_branch,
            auto_build=False,
            auto_build_archive="/ubuntu/+archive/primary",
            auto_build_pocket="Updates",
            **optional_kwargs,
        )

    def _delete_snap(self) -> None:
        """Remove snap info and all associated files."""
        snap = self._get_snap()
        if snap is None:
            return

        logger.debug("Removing snap job from Launchpad...")
        snap.lp_delete()

    def cleanup(self) -> None:
        """Delete snap and git repository from launchpad."""
        self._delete_snap()
        self._delete_git_repository()

    def start_build(self, timeout: int = 0) -> None:
        """Start build with specified timeout (time.time() in seconds)."""
        snap = self._create_snap(force=True)

        logger.debug("Issuing build request on Launchpad...")
        build_request = self._issue_build_request(snap)
        self._wait_for_build_request_acceptance(build_request, timeout=timeout)

    def monitor_build(
        self, interval: int = _LP_POLL_INTERVAL, timeout: int = 0
    ) -> None:
        """Check build progress, and download artifacts when ready."""
        snap = self._get_snap()

        while True:
            # Check to see if we've run out of time.
            self._check_timeout_deadline()

            builds = self._get_builds(snap)
            pending = False
            timestamp = str(datetime.now())
            logger.info(f"Build status as of {timestamp}:")
            for build in builds:
                state = build["buildstate"]
                arch = build["arch_tag"]
                logger.info(f"\tarch={arch}\tstate={state}")

                if _is_build_pending(build):
                    pending = True

            if pending is False:
                break

            time.sleep(interval)

        # Build is complete - download build artifacts.
        self._fetch_artifacts(snap)

    def get_build_status(self) -> Dict[str, str]:
        """Get status of builds."""
        snap = self._get_snap()
        builds = self._get_builds(snap)
        build_status: Dict[str, str] = dict()
        for build in builds:
            state = build["buildstate"]
            arch = build["arch_tag"]
            build_status[arch] = state

        return build_status

    def _get_logfile_name(self, arch: str) -> str:
        n = 0
        base_name = "{}_{}".format(self._snap_name, arch)
        log_name = f"{base_name}.txt"

        while os.path.isfile(log_name):
            n += 1
            log_name = f"{base_name}.{n}.txt"

        return log_name

    def _download_log(self, build: Dict[str, Any]) -> None:
        url = build["build_log_url"]
        arch = build["arch_tag"]
        if url is None:
            logger.info(f"No build log available for {arch!r}.")
        else:
            log_name = self._get_logfile_name(arch)
            self._download_file(url=url, dst=log_name, gunzip=True)
            logger.info(f"Build log available at {log_name!r}")

        if _is_build_status_failure(build):
            logger.error(f"Build failed for arch {arch!r}.")

    def _download_file(self, *, url: str, dst: str, gunzip: bool = False) -> None:
        # TODO: consolidate with, and use indicators.download_requests_stream
        logger.debug(f"Downloading: {url}")
        try:
            with requests.get(url, stream=True) as response:
                # Wrap response with gzipfile if gunzip is requested.
                stream = response.raw
                if gunzip:
                    stream = gzip.GzipFile(fileobj=stream)
                with open(dst, "wb") as f_dst:
                    shutil.copyfileobj(stream, f_dst)
                response.raise_for_status()
        except requests.exceptions.RequestException as e:
            logger.error(f"Error downloading {url}: {str(e)}")

    def _download_build_artifacts(self, build: Dict[str, Any]) -> None:
        arch = build["arch_tag"]
        snap_build = self._lp_load_url(build["self_link"])
        urls = snap_build.getFileUrls()

        if not urls:
            logger.error(f"Snap file not available for arch {arch!r}.")
            return

        for url in urls:
            file_name = _get_url_basename(url)

            self._download_file(url=url, dst=file_name)

            if file_name.endswith(".snap"):
                logger.info(f"Snapped {file_name}")
            else:
                logger.info(f"Fetched {file_name}")

    def _gitify_repository(self, repo_dir: str) -> Git:
        """Git-ify source repository tree.

        :return: Git handler instance to git repository.
        """
        git_handler = self._git_class(repo_dir, repo_dir, silent=True)

        # Init repo.
        git_handler.init()

        # Add all files found in repo.
        for f in os.listdir(repo_dir):
            if f != ".git":
                git_handler.add(f)

        # Commit files.
        git_handler.commit(
            f"committed by snapcraft version: {self._running_snapcraft_version}"
        )

        return git_handler

    def has_outstanding_build(self) -> bool:
        """Check if there is an existing build configured on Launchpad."""
        snap = self._get_snap()
        return snap is not None

    def push_source_tree(self, repo_dir: str) -> str:
        """Push source tree to Launchpad, returning URL."""
        git_handler = self._gitify_repository(repo_dir)
        lp_repo = self._create_git_repository(force=True)
        token = lp_repo.issueAccessToken()

        url = self.get_git_https_url(token=token)
        stripped_url = self.get_git_https_url(token="<token>")

        logger.info(f"Sending build data to Launchpad... ({stripped_url})")

        try:
            git_handler.push(url, f"HEAD:{self._lp_git_branch}", force=True)
        except SnapcraftPullError as error:
            # Strip token from command.
            command = error.command.replace(token, "<token>")  # type: ignore
            exit_code = error.exit_code  # type: ignore
            raise errors.LaunchpadGitPushError(command=command, exit_code=exit_code)

        return url
