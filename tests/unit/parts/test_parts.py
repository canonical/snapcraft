# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2024 Canonical Ltd.
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

from pathlib import Path
from unittest.mock import ANY, call

import craft_parts
import pytest

from snapcraft import errors
from snapcraft.parts import PartsLifecycle
from snapcraft.parts.yaml_utils import CURRENT_BASES


@pytest.fixture
def parts_data():
    yield {
        "p1": {"plugin": "nil"},
    }


@pytest.mark.parametrize("step_name", ["pull", "build", "stage", "prime"])
def test_parts_lifecycle_run(mocker, parts_data, step_name, new_dir, emitter):
    mocker.patch("craft_parts.executor.executor.Executor._install_build_snaps")
    lcm_spy = mocker.spy(craft_parts, "LifecycleManager")
    lifecycle = PartsLifecycle(
        parts_data,
        work_dir=new_dir,
        assets_dir=new_dir,
        base="core22",
        project_base="core22",
        confinement="strict",
        parallel_build_count=8,
        part_names=[],
        package_repositories=[],
        adopt_info=None,
        project_name="test-project",
        parse_info={},
        project_vars={"version": "1", "grade": "stable"},
        extra_build_snaps=["core22"],
        track_stage_packages=True,
        target_arch="amd64",
        partitions=None,
    )
    lifecycle.run(step_name)
    assert lifecycle.prime_dir == Path(new_dir, "prime")
    assert lifecycle.prime_dir.is_dir()
    assert lcm_spy.mock_calls == [
        call(
            {"parts": {"p1": {"plugin": "nil"}}},
            application_name="snapcraft",
            work_dir=ANY,
            cache_dir=ANY,
            arch="x86_64",
            base="core22",
            ignore_local_sources=["*.snap"],
            extra_build_snaps=["core22"],
            track_stage_packages=True,
            parallel_build_count=8,
            project_name="test-project",
            project_vars_part_name=None,
            project_vars={"version": "1", "grade": "stable"},
            confinement="strict",
            project_base="core22",
            partitions=None,
        )
    ]


@pytest.mark.usefixtures("enable_partitions_feature")
@pytest.mark.parametrize("base", CURRENT_BASES)
@pytest.mark.parametrize("step_name", ["pull", "build", "stage", "prime"])
def test_parts_lifecycle_run_with_components(
    mocker, base, parts_data, step_name, new_dir
):
    """Verify usage of the partitions feature."""
    lcm_spy = mocker.spy(craft_parts, "LifecycleManager")
    lifecycle = PartsLifecycle(
        parts_data,
        work_dir=new_dir,
        assets_dir=new_dir,
        base=base,
        project_base=base,
        confinement="strict",
        parallel_build_count=8,
        part_names=[],
        package_repositories=[],
        adopt_info=None,
        project_name="test-project",
        parse_info={},
        project_vars={"version": "1", "grade": "stable"},
        extra_build_snaps=None,
        track_stage_packages=True,
        target_arch="amd64",
        partitions=["default", "component/foo", "component/bar"],
    )
    lifecycle.run(step_name)

    # default partition
    assert lifecycle.prime_dir == Path(new_dir, "prime")
    assert lifecycle.prime_dir.is_dir()

    # component/foo partition
    assert (
        lifecycle.get_prime_dir_for_component(component="foo")
        == Path(new_dir) / "partitions" / "component" / "foo" / "prime"
    )
    assert lifecycle.get_prime_dir_for_component(component="foo").is_dir()

    # component/bar partition
    assert (
        lifecycle.get_prime_dir_for_component(component="bar")
        == Path(new_dir) / "partitions" / "component" / "bar" / "prime"
    )
    assert lifecycle.get_prime_dir_for_component(component="bar").is_dir()

    # partitions
    assert lcm_spy.call_args[1]["partitions"] == [
        "default",
        "component/foo",
        "component/bar",
    ]
    assert craft_parts.Features().enable_partitions


@pytest.mark.usefixtures("enable_partitions_feature")
@pytest.mark.parametrize("base", CURRENT_BASES)
def test_parts_lifecycle_get_prime_dir_non_existent_component(
    base, parts_data, new_dir
):
    """Raise an error when getting the prime directory of a non-existent component."""
    lifecycle = PartsLifecycle(
        parts_data,
        work_dir=new_dir,
        assets_dir=new_dir,
        base=base,
        project_base=base,
        confinement="strict",
        parallel_build_count=8,
        part_names=[],
        package_repositories=[],
        adopt_info=None,
        project_name="test-project",
        parse_info={},
        project_vars={"version": "1", "grade": "stable"},
        extra_build_snaps=None,
        track_stage_packages=True,
        target_arch="amd64",
        partitions=["default", "component/foo", "component/bar"],
    )

    with pytest.raises(errors.SnapcraftError) as raised:
        lifecycle.get_prime_dir_for_component("bad")

    assert str(raised.value) == (
        "Could not get prime directory for component 'bad' because it does not exist."
    )


def test_parts_lifecycle_run_bad_step(parts_data, new_dir):
    lifecycle = PartsLifecycle(
        parts_data,
        work_dir=new_dir,
        assets_dir=new_dir,
        base="core22",
        project_base="core22",
        confinement="strict",
        parallel_build_count=8,
        part_names=[],
        package_repositories=[],
        adopt_info=None,
        parse_info={},
        project_name="test-project",
        project_vars={"version": "1", "grade": "stable"},
        target_arch="amd64",
        track_stage_packages=True,
        partitions=None,
    )
    with pytest.raises(RuntimeError) as raised:
        lifecycle.run("invalid")
    assert str(raised.value) == "Invalid target step 'invalid'"


def test_parts_lifecycle_run_internal_error(parts_data, new_dir, mocker):
    lifecycle = PartsLifecycle(
        parts_data,
        work_dir=new_dir,
        assets_dir=new_dir,
        base="core22",
        project_base="core22",
        confinement="strict",
        parallel_build_count=8,
        part_names=[],
        package_repositories=[],
        adopt_info=None,
        project_name="test-project",
        parse_info={},
        project_vars={"version": "1", "grade": "stable"},
        target_arch="amd64",
        track_stage_packages=True,
        partitions=None,
    )
    mocker.patch("craft_parts.LifecycleManager.plan", side_effect=RuntimeError("crash"))
    with pytest.raises(RuntimeError) as raised:
        lifecycle.run("prime")
    assert str(raised.value) == "Parts processing internal error: crash"


def test_parts_lifecycle_run_parts_error(new_dir):
    lifecycle = PartsLifecycle(
        {"p1": {"plugin": "dump", "source": "foo"}},
        work_dir=new_dir,
        assets_dir=new_dir,
        base="core22",
        project_base="core22",
        confinement="strict",
        parallel_build_count=8,
        part_names=[],
        package_repositories=[],
        adopt_info=None,
        project_name="test-project",
        parse_info={},
        project_vars={"version": "1", "grade": "stable"},
        target_arch="amd64",
        track_stage_packages=True,
        partitions=None,
    )
    with pytest.raises(errors.PartsLifecycleError) as raised:
        lifecycle.run("prime")
    assert str(raised.value) == (
        "Failed to pull source: unable to determine source type of 'foo'."
    )


def test_parts_lifecycle_clean(parts_data, new_dir, emitter):
    lifecycle = PartsLifecycle(
        parts_data,
        work_dir=new_dir,
        assets_dir=new_dir,
        base="core22",
        project_base="core22",
        confinement="strict",
        parallel_build_count=8,
        part_names=[],
        package_repositories=[],
        adopt_info=None,
        project_name="test-project",
        parse_info={},
        project_vars={"version": "1", "grade": "stable"},
        target_arch="amd64",
        track_stage_packages=True,
        partitions=None,
    )
    lifecycle.clean(part_names=None)
    emitter.assert_progress("Cleaning all parts")


def test_parts_lifecycle_clean_parts(parts_data, new_dir, emitter):
    lifecycle = PartsLifecycle(
        parts_data,
        work_dir=new_dir,
        assets_dir=new_dir,
        base="core22",
        project_base="core22",
        confinement="strict",
        parallel_build_count=8,
        part_names=[],
        package_repositories=[],
        adopt_info=None,
        project_name="test-project",
        parse_info={},
        project_vars={"version": "1", "grade": "stable"},
        target_arch="amd64",
        track_stage_packages=True,
        partitions=None,
    )
    lifecycle.clean(part_names=["p1"])
    emitter.assert_progress("Cleaning parts: p1")


def test_parts_lifecycle_initialize_with_package_repositories_deps_not_installed(
    mocker,
    parts_data,
    new_dir,
):
    mocker.patch(
        "craft_parts.packages.Repository.is_package_installed", return_value=False
    )
    install_packages_mock = mocker.patch(
        "craft_parts.packages.Repository.install_packages"
    )
    mocker.patch(
        "craft_archives.repo.apt_key_manager."
        "AptKeyManager.install_package_repository_key",
        return_value=True,
    )
    mocker.patch(
        "craft_archives.repo.apt_sources_manager."
        "AptSourcesManager.install_package_repository_sources"
    )
    mocker.patch(
        "craft_archives.repo.apt_preferences_manager.AptPreferencesManager.write"
    )
    mocker.patch("craft_parts.packages.Repository.refresh_packages_list")

    parts_lifecycle = PartsLifecycle(
        parts_data,
        work_dir=new_dir,
        assets_dir=new_dir,
        base="core22",
        project_base="core22",
        confinement="strict",
        parallel_build_count=8,
        part_names=[],
        package_repositories=[
            {
                "type": "apt",
                "ppa": "test/somerepo",
            },
        ],
        adopt_info=None,
        project_name="test-project",
        parse_info={},
        project_vars={"version": "1", "grade": "stable"},
        extra_build_snaps=["core22"],
        track_stage_packages=True,
        target_arch="amd64",
        partitions=None,
    )

    parts_lifecycle._install_package_repositories()

    assert install_packages_mock.mock_calls == [
        call(["gnupg", "dirmngr"], refresh_package_cache=True)
    ]


def test_parts_lifecycle_initialize_with_package_repositories_deps_installed(
    mocker,
    parts_data,
    new_dir,
):
    mocker.patch(
        "craft_parts.packages.Repository.is_package_installed", return_value=True
    )
    install_packages_mock = mocker.patch(
        "craft_parts.packages.Repository.install_packages"
    )
    mocker.patch(
        "craft_archives.repo.apt_key_manager.AptKeyManager.install_package_repository_key",
        return_value=True,
    )
    mocker.patch(
        "craft_archives.repo.apt_sources_manager."
        "AptSourcesManager.install_package_repository_sources"
    )
    mocker.patch(
        "craft_archives.repo.apt_preferences_manager.AptPreferencesManager.write"
    )
    mocker.patch("craft_parts.packages.Repository.refresh_packages_list")

    parts_lifecycle = PartsLifecycle(
        parts_data,
        work_dir=new_dir,
        assets_dir=new_dir,
        base="core22",
        project_base="core22",
        confinement="strict",
        parallel_build_count=8,
        part_names=[],
        package_repositories=[
            {
                "type": "apt",
                "ppa": "test/somerepo",
            },
        ],
        adopt_info=None,
        project_name="test-project",
        parse_info={},
        project_vars={"version": "1", "grade": "stable"},
        extra_build_snaps=["core22"],
        track_stage_packages=True,
        target_arch="amd64",
        partitions=None,
    )

    parts_lifecycle._install_package_repositories()

    assert install_packages_mock.mock_calls == []


def test_parts_lifecycle_bad_architecture(parts_data, new_dir):
    with pytest.raises(errors.InvalidArchitecture) as raised:
        PartsLifecycle(
            parts_data,
            work_dir=new_dir,
            assets_dir=new_dir,
            base="core22",
            project_base="core22",
            confinement="strict",
            parallel_build_count=8,
            track_stage_packages=True,
            part_names=[],
            package_repositories=[],
            adopt_info=None,
            parse_info={},
            project_name="test-project",
            project_vars={"version": "1", "grade": "stable"},
            target_arch="bad-arch",
            partitions=None,
        )

    assert str(raised.value) == "Architecture 'bad-arch' is not supported."


def test_parts_lifecycle_run_with_all_architecture(mocker, parts_data, new_dir):
    """`target_arch=all` should use the host architecture."""

    mocker.patch("craft_parts.executor.executor.Executor._install_build_snaps")
    lcm_spy = mocker.spy(craft_parts, "LifecycleManager")
    lifecycle = PartsLifecycle(
        parts_data,
        work_dir=new_dir,
        assets_dir=new_dir,
        base="core22",
        project_base="core22",
        confinement="strict",
        parallel_build_count=8,
        track_stage_packages=True,
        part_names=[],
        package_repositories=[],
        adopt_info=None,
        parse_info={},
        project_name="test-project",
        project_vars={"version": "1", "grade": "stable"},
        target_arch="amd64",
        partitions=None,
    )
    lifecycle.run("prime")

    assert lcm_spy.mock_calls == [
        call(
            {"parts": {"p1": {"plugin": "nil"}}},
            application_name="snapcraft",
            work_dir=ANY,
            cache_dir=ANY,
            arch="x86_64",
            base="core22",
            ignore_local_sources=["*.snap"],
            extra_build_snaps=None,
            track_stage_packages=True,
            parallel_build_count=8,
            project_name="test-project",
            project_vars_part_name=None,
            project_vars={"version": "1", "grade": "stable"},
            project_base="core22",
            confinement="strict",
            partitions=None,
        )
    ]
