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

import logging
import os.path
import shutil
import subprocess
import sys
import tempfile

import requests
import simplejson

from snapcraft.file_utils import calculate_sha3_384
from snapcraft.internal.build_providers.errors import (
    ProviderMultipassDownloadFailed,
    ProviderMultipassInstallationFailed,
)
from snapcraft.internal.indicators import download_requests_stream

if sys.platform == "win32":
    import winreg


logger = logging.getLogger(__name__)


_MULTIPASS_RELEASES_API_URL = (
    "https://api.github.com/repos/CanonicalLtd/multipass/releases/latest"
)
_MULTIPASS_DL_VERSION = "0.8.0"
_MULTIPASS_DL_NAME = "multipass-{version}+win-win64.exe".format(
    version=_MULTIPASS_DL_VERSION
)
_MULTIPASS_DL_SHA3_384 = "a1ac2eeb77b2a98fe5dee198be70dbf1a985d94b9707ce33ea0d3828dbc90d07fccb9662b7c97a3cfa194895b4f56676"


def windows_reload_multipass_path_env():
    """Update PATH to include installed Multipass, if not already set."""

    assert sys.platform == "win32"

    key = winreg.OpenKey(winreg.HKEY_CURRENT_USER, "Environment")

    paths = os.environ["PATH"].split(";")

    # Drop empty placeholder for trailing comma, if present.
    if paths[-1] == "":
        del paths[-1]

    reg_user_path, _ = winreg.QueryValueEx(key, "Path")
    for path in reg_user_path.split(";"):
        if path not in paths and "Multipass" in path:
            paths.append(path)

    # Restore path with trailing comma.
    os.environ["PATH"] = ";".join(paths) + ";"


def _run_installer(installer_path: str, echoer):
    """Execute multipass installer."""

    echoer.info("Installing Multipass...")

    # Multipass requires administrative privileges to install, which requires
    # the use of `runas` functionality. Some of the options included:
    # (1) https://stackoverflow.com/a/34216774
    # (2) ShellExecuteW and wait on installer by attempting to delete it.
    # Windows would prevent us from deleting installer with a PermissionError:
    # PermissionError: [WinError 32] The process cannot access the file because
    # it is being used by another process: <path>
    # (3) Use PowerShell's "Start-Process" with RunAs verb as shown below.
    # None of the options are quite ideal, but #3 will do.
    cmd = """
    & {{
        try {{
            $Output = Start-Process -FilePath {path!r} -Args /S -Verb RunAs -Wait -PassThru
        }} catch {{
            [Environment]::Exit(1)
        }}
      }}
    """.format(
        path=installer_path
    )

    try:
        subprocess.check_call(["powershell.exe", "-Command", cmd])
    except subprocess.CalledProcessError:
        raise ProviderMultipassInstallationFailed("error launching installer")

    # Reload path environment to see if we can find multipass now.
    windows_reload_multipass_path_env()

    if not shutil.which("multipass.exe"):
        # Installation failed.
        raise ProviderMultipassInstallationFailed(
            "installation did not complete successfully"
        )

    echoer.info("Multipass installation completed successfully.")


def _requests_exception_hint(e: requests.RequestException) -> str:
    # Use the __doc__ description to give the user a hint. It seems to be a
    # a decent option over trying to enumerate all of possible types.
    if e.__doc__:
        split_lines = e.__doc__.splitlines()
        if split_lines:
            return e.__doc__.splitlines()[0].strip()

    # Should never get here.
    return "unknown download error"


def _fetch_installer_url() -> str:
    """Fetch latest installer executable from github.

    If the latest release on github is newer than what snapcraft knows
    about in _MULTIPASS_DL_NAME_*, we skip the download and inform the
    user to go download it manually.  This way we will only ever directly
    execute whitelisted executables on behalf of the user.  Verify the
    installer using a SHA3-384 digest.
    """

    try:
        resp = requests.get(_MULTIPASS_RELEASES_API_URL)
    except requests.RequestException as e:
        raise ProviderMultipassDownloadFailed(_requests_exception_hint(e))

    try:
        data = resp.json()
    except simplejson.scanner.JSONDecodeError:
        raise ProviderMultipassDownloadFailed(
            "failed to fetch valid release data from {}".format(
                _MULTIPASS_RELEASES_API_URL
            )
        )

    for asset in data.get("assets", list()):
        # Find matching name.
        if asset.get("name") != _MULTIPASS_DL_NAME:
            continue

        return asset.get("browser_download_url")

    # Something changed we don't know about - we will simply categorize
    # all possible events as an updated version we do not yet know about.
    raise ProviderMultipassDownloadFailed(
        "an updated version is available that snapcraft does not yet know about"
    )


def _download_multipass(dl_dir: str, echoer) -> str:
    """Creates temporary Downloads installer to temp directory."""

    dl_url = _fetch_installer_url()
    dl_basename = os.path.basename(dl_url)
    dl_path = os.path.join(dl_dir, dl_basename)

    echoer.info("Downloading Multipass installer...\n{} -> {}".format(dl_url, dl_path))

    try:
        request = requests.get(dl_url, stream=True, allow_redirects=True)
        request.raise_for_status()
        download_requests_stream(request, dl_path)
    except requests.RequestException as e:
        raise ProviderMultipassDownloadFailed(_requests_exception_hint(e))

    digest = calculate_sha3_384(dl_path)
    if digest != _MULTIPASS_DL_SHA3_384:
        raise ProviderMultipassDownloadFailed(
            "download failed verification (expected={} but found={})".format(
                _MULTIPASS_DL_SHA3_384, digest
            )
        )

    echoer.info("Verified installer successfully...")
    return dl_path


def windows_install_multipass(echoer) -> None:
    """Download and install multipass."""

    assert sys.platform == "win32"

    dl_dir = tempfile.mkdtemp()
    dl_path = _download_multipass(dl_dir, echoer)
    _run_installer(dl_path, echoer)

    # Cleanup.
    shutil.rmtree(dl_dir)
