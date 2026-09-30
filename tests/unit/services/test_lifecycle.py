# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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

"""Tests for the Snapcraft Lifecycle service."""

import json
import platform
import shutil
import sys
from pathlib import Path
from unittest import mock

import pytest
import pytest_subprocess
from craft_platforms import DebianArchitecture, DistroBase

import snapcraft.parts
from snapcraft import __version__, models, os_release


@pytest.fixture
def lifecycle_service(default_project, fake_services, setup_project):
    setup_project(
        fake_services,
        {**default_project.marshal(), "parts": {"my-part": {"plugin": "nil"}}},
    )
    return fake_services.get("lifecycle")


def test_lifecycle_installs_base(lifecycle_service, mocker):
    install_snaps = mocker.patch("craft_parts.packages.snaps.install_snaps")

    mocker.patch(
        "craft_platforms.DistroBase.from_linux_distribution",
        return_value=DistroBase("ubuntu", "24.04"),
    )

    lifecycle_service.run("pull")

    info = lifecycle_service.project_info
    assert info.base == "core24"
    install_snaps.assert_called_once_with(
        {"core24"},
    )


def test_post_prime_no_patchelf(fp, tmp_path, lifecycle_service, default_project):
    mock_step_info = mock.Mock()
    mock_step_info.configure_mock(
        **{
            "base": "core24",
            "build_attributes": [],
            "state.files": ["usr/bin/ls"],
            "prime_dir": tmp_path / "prime",
            "part_name": "my-part",
        }
    )

    lifecycle_service.post_prime(mock_step_info)

    assert not fp.calls


@pytest.mark.skipif(sys.platform != "linux", reason="Patchelf only works on Linux")
@pytest.mark.xfail(
    not shutil.which("patchelf"), reason="Expects a real patchelf binary."
)
@pytest.mark.parametrize(
    ("confinement", "use_system_libs"),
    [
        ("strict", True),
        ("devmode", True),
        # classic snaps should not use libraries from the system
        ("classic", False),
    ],
)
def test_post_prime_patchelf(
    fp: pytest_subprocess.FakeProcess,
    tmp_path,
    default_project,
    fake_services,
    setup_project,
    mocker,
    confinement,
    use_system_libs,
):
    setup_project(
        fake_services,
        {
            **default_project.marshal(),
            "confinement": confinement,
            "parts": {"my-part": {"plugin": "nil"}},
        },
    )
    lifecycle_service = fake_services.get("lifecycle")
    patchelf_spy = mocker.spy(snapcraft.parts, "patch_elf")

    mock_step_info = mock.Mock()
    mock_step_info.configure_mock(
        **{
            "base": "core24",
            "build_attributes": ["enable-patchelf"],
            "state.files": ["usr/bin/ls"],
            "prime_dir": tmp_path / "prime",
            "part_name": "my-part",
        }
    )

    prime_bin = tmp_path / "prime" / "usr" / "bin"
    prime_bin.mkdir(parents=True)
    shutil.copy("/bin/ls", prime_bin)
    (tmp_path / "prime/usr/bin/hello").touch()

    linux_arch = platform.machine().replace("_", "-")
    patchelf_command = [
        fp.program("patchelf"),
        "--set-interpreter",
        f"/snap/core24/current/lib64/ld-linux-{linux_arch}.so.2",
        fp.any(),  # Temporary file containing the file to patch.
    ]
    fp.register(["/usr/bin/ldd", str(tmp_path / "prime" / "usr/bin/ls")])
    fp.register(patchelf_command)

    lifecycle_service.post_prime(mock_step_info)

    patchelf_spy.assert_called_with(mock_step_info, use_system_libs=use_system_libs)
    assert fp.call_count(patchelf_command) == 1


@pytest.mark.parametrize(
    ("parts", "stage_packages"),
    [
        ({}, []),
        (
            {
                "simplified": {
                    "plugin": "nil",
                    "stage-packages": ["hello"],
                    "stage": [],
                    "prime": [],
                    "build-packages": [],
                },
                "traditional": {
                    "plugin": "nil",
                    "stage-packages": ["hello-traditional"],
                    "stage": [],
                    "prime": [],
                    "build-packages": [],
                },
            },
            ["hello", "hello-traditional"],
        ),
    ],
)
@pytest.mark.parametrize("image_info", [{}, {"custom_image": True}])
def test_generate_manifest(
    monkeypatch,
    default_project,
    fake_services,
    setup_project,
    image_info,
    parts,
    stage_packages,
):
    setup_project(
        fake_services,
        {
            **default_project.marshal(),
            "parts": parts,
        },
    )
    lifecycle_service = fake_services.get("lifecycle")
    monkeypatch.setenv("SNAPCRAFT_IMAGE_INFO", json.dumps(image_info))
    osrel = os_release.OsRelease()
    lifecycle_service.get_pull_assets = mock.Mock(return_value={})
    lifecycle_service.get_primed_stage_packages = mock.Mock(return_value=stage_packages)

    manifest = lifecycle_service.generate_manifest()

    assert manifest == models.Manifest(
        snapcraft_version=__version__,
        snapcraft_started_at=lifecycle_service._start_time.isoformat("T") + "Z",
        snapcraft_os_release_id=osrel.name().lower(),
        snapcraft_os_release_version_id=osrel.version_id().lower(),
        name=default_project.name,
        version=default_project.version,
        summary=default_project.summary,
        description=default_project.description,
        base=default_project.base,
        grade=default_project.grade,
        confinement=default_project.confinement,
        parts=parts,
        architectures=[str(DebianArchitecture.from_host())],
        image_info=image_info,
        build_packages=default_project.build_packages or [],
        build_snaps=default_project.build_snaps or [],
        primed_stage_packages=stage_packages,
        apps=None,
    )

    assert lifecycle_service.get_pull_assets.mock_calls == [
        mock.call(part_name=part_name) for part_name in parts
    ]
    assert lifecycle_service.get_primed_stage_packages.mock_calls == [
        mock.call(part_name=part_name) for part_name in parts
    ]


@pytest.mark.parametrize("base", ["core24", None])
@pytest.mark.parametrize("confinement", ["strict", "devmode", "classic"])
def test_lifecycle_custom_arguments(
    default_project, fake_services, setup_project, base, confinement
):
    """Test that the lifecycle project has the correct project base and confinement."""

    # Set the base and confinement on the project. This roundabout way here
    # is because the validators make it hard to update the base from/to None
    # without "type" being already correct.
    new_attrs = {"base": base, "confinement": confinement}
    if base is None:
        new_attrs["type"] = "base"
        new_attrs["build-base"] = "core24"
    setup_project(fake_services, {**default_project.marshal(), **new_attrs})
    lifecycle_service = fake_services.get("lifecycle")

    info = lifecycle_service.project_info

    expected_confinement = confinement
    expected_base = base if base is not None else ""

    assert info.project_base == expected_base
    assert info.confinement == expected_confinement
    assert info.project_name == default_project.name == "default"


@pytest.mark.usefixtures("default_project")
def test_lifecycle_get_prime_dir(lifecycle_service):
    assert lifecycle_service.get_prime_dir() == lifecycle_service._work_dir / "prime"


@pytest.mark.usefixtures("default_project")
def test_lifecycle_prime_dirs(lifecycle_service):
    assert lifecycle_service.prime_dirs == {None: lifecycle_service._work_dir / "prime"}


@pytest.fixture()
def package_repositories_params(extra_project_params):
    """Add package-repositories configuration to the default project."""
    extra_project_params["package_repositories"] = [{"type": "apt", "ppa": "test/ppa"}]


@pytest.mark.usefixtures("package_repositories_params", "default_project")
def test_lifecycle_installs_gpg_dirmngr(
    default_project, fake_services, setup_project, mocker
):
    mock_is_installed = mocker.patch(
        "craft_parts.packages.deb.Ubuntu.is_package_installed", return_value=False
    )
    mock_install = mocker.patch("craft_parts.packages.deb.Ubuntu.install_packages")
    setup_project(fake_services, default_project.marshal())

    fake_services.get("lifecycle")

    assert mock_is_installed.called
    mock_install.assert_called_once_with(
        ["gpg", "dirmngr"], refresh_package_cache=False
    )


@pytest.mark.parametrize(
    ("project_location", "assets_dir", "expected_keys_path"),
    [
        # These locations come from yaml_utils._SNAP_PROJECT_FILES
        # snapcraft.yaml locations where the keys are expected to be in snap/keys
        ("snapcraft.yaml", "snap", "snap/keys"),
        ("snap/snapcraft.yaml", "snap", "snap/keys"),
        (".snapcraft.yaml", "snap", "snap/keys"),
        # snapcraft.yaml location where the keys are expected to be somewhere else
        ("build-aux/snap/snapcraft.yaml", "build-aux/snap", "build-aux/snap/keys"),
    ],
)
def test_local_keys_path(
    lifecycle_service, project_location, assets_dir, expected_keys_path, in_project_path
):
    """Check that _get_local_keys_path() is correct given the location of snapcraft.yaml."""
    snap_dir = in_project_path / Path(project_location).parent
    snap_dir.mkdir(exist_ok=True, parents=True)

    # The project file itself doesn't really matter, but must exist
    project_yaml = snap_dir / "snapcraft.yaml"
    project_yaml.touch()

    keys_dir = Path(assets_dir) / "keys"

    # If the keys dir doesn't exist the method should return None
    assert not keys_dir.is_dir()
    assert lifecycle_service._get_local_keys_path() is None

    keys_dir.mkdir(exist_ok=True, parents=True)
    assert lifecycle_service._get_local_keys_path() == Path(expected_keys_path)
