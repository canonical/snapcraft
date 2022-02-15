# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

"""Craft-parts lifecycle wrapper."""

import pathlib
from typing import Any, Dict, List, Optional

import craft_parts
from craft_cli import emit
from craft_parts import ActionType, Step
from xdg import BaseDirectory  # type: ignore

from snapcraft import errors
from snapcraft.repo import AptKeyManager, AptSourcesManager, PackageRepository

_LIFECYCLE_STEPS = {
    "pull": Step.PULL,
    "overlay": Step.OVERLAY,
    "build": Step.BUILD,
    "stage": Step.STAGE,
    "prime": Step.PRIME,
}


class PartsLifecycle:
    """Create and manage the parts lifecycle.

    :param all_parts: A dictionary containing the parts defined in the project.
    :param work_dir: The working directory for parts processing.
    :param assets_dir: The directory containing project assets.

    :raises PartsLifecycleError: On error initializing the parts lifecycle.
    """

    def __init__(
        self,
        all_parts: Dict[str, Any],
        *,
        work_dir: pathlib.Path,
        assets_dir: pathlib.Path,
        package_repositories: Optional[List[PackageRepository]] = None,
    ):
        self._assets_dir = assets_dir

        if package_repositories:
            self._package_repositories = package_repositories
        else:
            self._package_repositories = []

        emit.progress("Initializing parts lifecycle")

        # set the cache dir for parts package management
        cache_dir = BaseDirectory.save_cache_path("snapcraft")

        extra_build_packages = []
        if self._package_repositories:
            # Install pre-requisite packages for apt-key, if not installed.
            # FIXME: package names should be plataform-specific
            extra_build_packages.extend(["gnupg", "dirmngr"])

        try:
            self._lcm = craft_parts.LifecycleManager(
                {"parts": all_parts},
                application_name="snapcraft",
                work_dir=work_dir,
                cache_dir=cache_dir,
                ignore_local_sources=["*.snap"],
            )
        except craft_parts.PartsError as err:
            raise errors.PartsLifecycleError(str(err)) from err

    @property
    def prime_dir(self) -> pathlib.Path:
        """Return the parts prime directory path."""
        return self._lcm.project_info.prime_dir

    @property
    def target_arch(self) -> str:
        """Return the parts project target architecture."""
        return self._lcm.project_info.target_arch

    def run(self, step_name: str) -> None:
        """Run the parts lifecycle.

        :param target_step: The final step to execute.

        :raises PartsLifecycleError: On error during lifecycle.
        :raises RuntimeError: On unexpected error.
        """
        target_step = _LIFECYCLE_STEPS.get(step_name)
        if not target_step:
            raise RuntimeError(f"Invalid target step {step_name!r}")

        try:
            actions = self._lcm.plan(target_step)

            emit.progress("Installing package repositories...")

            if self._package_repositories:
                refresh_required = self._install_package_repositories()
                if refresh_required:
                    self._lcm.refresh_packages_list()

            emit.message("Installed package repositories", intermediate=True)

            emit.progress("Executing parts lifecycle...")

            with self._lcm.action_executor() as aex:
                for action in actions:
                    message = _action_message(action)
                    emit.progress(f"Executing parts lifecycle: {message}")
                    aex.execute(action)

            emit.message("Executed parts lifecycle", intermediate=True)
        except RuntimeError as err:
            raise RuntimeError(f"Parts processing internal error: {err}") from err
        except OSError as err:
            msg = err.strerror
            if err.filename:
                msg = f"{err.filename}: {msg}"
            raise errors.PartsLifecycleError(msg) from err
        except Exception as err:
            raise errors.PartsLifecycleError(str(err)) from err

    def _install_package_repositories(self) -> bool:
        key_assets = self._assets_dir / "keys"
        key_manager = AptKeyManager(key_assets=key_assets)
        sources_manager = AptSourcesManager()

        refresh_required = False
        for package_repo in self._package_repositories:
            refresh_required |= key_manager.install_package_repository_key(
                package_repo=package_repo
            )
            refresh_required |= sources_manager.install_package_repository_sources(
                package_repo=package_repo
            )

        _verify_all_key_assets_installed(
            key_assets=key_assets, key_manager=key_manager
        )

        return refresh_required

def _verify_all_key_assets_installed(
    *, key_assets: pathlib.Path, key_manager: AptKeyManager,
) -> None:
    """Verify all configured key assets are utilized, error if not."""
    for key_asset in key_assets.glob("*"):
        key = key_asset.read_text()
        for key_id in key_manager.get_key_fingerprints(key=key):
            if not key_manager.is_key_installed(key_id=key_id):
                raise errors.SnapcraftError("Found unused key asset {key_asset!r}.",
                    details="All configured key assets must be utilized.",
                    resolution="Verify key usage and remove all unused keys.")


def _action_message(action: craft_parts.Action) -> str:
    msg = {
        Step.PULL: {
            ActionType.RUN: "pull",
            ActionType.RERUN: "repull",
            ActionType.SKIP: "skip pull",
            ActionType.UPDATE: "update sources for",
        },
        Step.OVERLAY: {
            ActionType.RUN: "overlay",
            ActionType.RERUN: "re-overlay",
            ActionType.SKIP: "skip overlay",
            ActionType.UPDATE: "update overlay for",
            ActionType.REAPPLY: "reapply",
        },
        Step.BUILD: {
            ActionType.RUN: "build",
            ActionType.RERUN: "rebuild",
            ActionType.SKIP: "skip build",
            ActionType.UPDATE: "update build for",
        },
        Step.STAGE: {
            ActionType.RUN: "stage",
            ActionType.RERUN: "restage",
            ActionType.SKIP: "skip stage",
        },
        Step.PRIME: {
            ActionType.RUN: "prime",
            ActionType.RERUN: "re-prime",
            ActionType.SKIP: "skip prime",
        },
    }

    message = f"{msg[action.step][action.action_type]} {action.part_name}"

    if action.reason:
        message += f" ({action.reason})"

    return message
