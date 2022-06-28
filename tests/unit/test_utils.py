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

import os
from textwrap import dedent

import pytest

from snapcraft import utils


@pytest.mark.parametrize(
    "value",
    [
        "y",
        "Y",
        "yes",
        "YES",
        "Yes",
        "t",
        "T",
        "true",
        "TRUE",
        "True",
        "On",
        "ON",
        "oN",
        "1",
    ],
)
def test_strtobool_true(value: str):
    assert utils.strtobool(value) is True


@pytest.mark.parametrize(
    "value",
    [
        "n",
        "N",
        "no",
        "NO",
        "No",
        "f",
        "F",
        "false",
        "FALSE",
        "False",
        "off",
        "OFF",
        "oFF",
        "0",
    ],
)
def test_strtobool_false(value: str):
    assert utils.strtobool(value) is False


@pytest.mark.parametrize(
    "value",
    [
        "not",
        "yup",
        "negative",
        "positive",
        "whatever",
        "2",
        "3",
    ],
)
def test_strtobool_value_error(value: str):
    with pytest.raises(ValueError):
        utils.strtobool(value)


#####################
# Get Host Platform #
#####################


@pytest.mark.parametrize(
    "base,build_base,project_type,name,expected_base",
    [
        (None, "build_base", "base", "name", "build_base"),
        ("base", "build_base", "base", "name", "build_base"),
        (None, None, "base", "name", "name"),
        ("base", None, "base", "name", "name"),
        (None, None, "other", "name", None),
        ("base", "build_base", "other", "name", "build_base"),
        ("base", None, "other", "name", "base"),
    ],
)
def test_get_effective_base(base, build_base, project_type, name, expected_base):
    result = utils.get_effective_base(
        base=base, build_base=build_base, project_type=project_type, name=name
    )
    assert result == expected_base


def test_get_os_platform_linux(tmp_path, mocker):
    """Utilize an /etc/os-release file to determine platform."""
    # explicitly add commented and empty lines, for parser robustness
    filepath = tmp_path / "os-release"
    filepath.write_text(
        dedent(
            """
        # the following is an empty line

        NAME="Ubuntu"
        VERSION="20.04.1 LTS (Focal Fossa)"
        ID=ubuntu
        ID_LIKE=debian
        PRETTY_NAME="Ubuntu 20.04.1 LTS"
        VERSION_ID="20.04"
        HOME_URL="https://www.ubuntu.com/"
        SUPPORT_URL="https://help.ubuntu.com/"
        BUG_REPORT_URL="https://bugs.launchpad.net/ubuntu/"

        # more in the middle; the following even would be "out of standard", but
        # we should not crash, just ignore it
        SOMETHING-WEIRD

        PRIVACY_POLICY_URL="https://www.ubuntu.com/legal/terms-and-policies/privacy-policy"
        VERSION_CODENAME=focal
        UBUNTU_CODENAME=focal
        """
        )
    )
    mocker.patch("platform.machine", return_value="x86_64")
    mocker.patch("platform.system", return_value="Linux")

    os_platform = utils.get_os_platform(filepath)

    assert os_platform.system == "ubuntu"
    assert os_platform.release == "20.04"
    assert os_platform.machine == "x86_64"


@pytest.mark.parametrize(
    "name",
    [
        ('"foo bar"', "foo bar"),  # what's normally found
        ("foo bar", "foo bar"),  # no quotes
        ('"foo " bar"', 'foo " bar'),  # quotes in the middle
        ('foo bar"', 'foo bar"'),  # unbalanced quotes (no really enclosing)
        ('"foo bar', '"foo bar'),  # unbalanced quotes (no really enclosing)
        ("'foo bar'", "foo bar"),  # enclosing with single quote
        ("'foo ' bar'", "foo ' bar"),  # single quote in the middle
        ("foo bar'", "foo bar'"),  # unbalanced single quotes (no really enclosing)
        ("'foo bar", "'foo bar"),  # unbalanced single quotes (no really enclosing)
        ("'foo bar\"", "'foo bar\""),  # unbalanced mixed quotes
        ("\"foo bar'", "\"foo bar'"),  # unbalanced mixed quotes
    ],
)
def test_get_os_platform_alternative_formats(tmp_path, mocker, name):
    """Support different ways of building the string."""
    source, result = name
    filepath = tmp_path / "os-release"
    filepath.write_text(
        dedent(
            f"""
            ID={source}
            VERSION_ID="20.04"
            """
        )
    )
    # need to patch this to "Linux" so actually uses /etc/os-release...
    mocker.patch("platform.system", return_value="Linux")

    os_platform = utils.get_os_platform(filepath)

    assert os_platform.system == result


def test_get_os_platform_windows(mocker):
    """Get platform from a patched Windows machine."""
    mocker.patch("platform.system", return_value="Windows")
    mocker.patch("platform.release", return_value="10")
    mocker.patch("platform.machine", return_value="AMD64")

    os_platform = utils.get_os_platform()

    assert os_platform.system == "Windows"
    assert os_platform.release == "10"
    assert os_platform.machine == "AMD64"


@pytest.mark.parametrize(
    "platform_machine,platform_architecture,deb_arch",
    [
        ("AMD64", ("64bit", "ELF"), "amd64"),
        ("aarch64", ("64bit", "ELF"), "arm64"),
        ("aarch64", ("32bit", "ELF"), "armhf"),
        ("armv7l", ("64bit", "ELF"), "armhf"),
        ("ppc", ("64bit", "ELF"), "powerpc"),
        ("ppc64le", ("64bit", "ELF"), "ppc64el"),
        ("x86_64", ("64bit", "ELF"), "amd64"),
        ("x86_64", ("32bit", "ELF"), "i386"),
        ("unknown-arch", ("64bit", "ELF"), "unknown-arch"),
    ],
)
def test_get_host_architecture(
    platform_machine, platform_architecture, mocker, deb_arch
):
    """Test all platform mappings in addition to unknown."""
    mocker.patch("platform.machine", return_value=platform_machine)
    mocker.patch("platform.architecture", return_value=platform_architecture)

    assert utils.get_host_architecture() == deb_arch


########################
# Parallel build count #
########################


def test_get_parallel_build_count(mocker):
    mocker.patch("os.sched_getaffinity", return_value=[1] * 13)
    assert utils.get_parallel_build_count() == 13


def test_get_parallel_build_count_no_affinity(mocker):
    mocker.patch("os.sched_getaffinity", side_effect=AttributeError)
    mocker.patch("multiprocessing.cpu_count", return_value=17)
    assert utils.get_parallel_build_count() == 17


def test_get_parallel_build_count_disable(mocker):
    mocker.patch("os.sched_getaffinity", side_effect=AttributeError)
    mocker.patch("multiprocessing.cpu_count", side_effect=NotImplementedError)
    assert utils.get_parallel_build_count() == 1


@pytest.mark.parametrize(
    "max_count,count",
    [("", 13), ("xx", 13), ("0", 13), ("1", 1), ("8", 8), ("13", 13), ("14", 13)],
)
def test_get_parallel_build_count_limited(mocker, max_count, count):
    mocker.patch("os.sched_getaffinity", return_value=[1] * 13)
    mocker.patch.dict(os.environ, {"SNAPCRAFT_MAX_PARALLEL_BUILD_COUNT": max_count})
    assert utils.get_parallel_build_count() == count


#################
# Humanize List #
#################


@pytest.mark.parametrize(
    "items,conjunction,expected",
    (
        ([], "and", ""),
        (["foo"], "and", "'foo'"),
        (["foo", "bar"], "and", "'bar' and 'foo'"),
        (["foo", "bar", "baz"], "and", "'bar', 'baz', and 'foo'"),
        (["foo", "bar", "baz", "qux"], "and", "'bar', 'baz', 'foo', and 'qux'"),
        ([], "or", ""),
        (["foo"], "or", "'foo'"),
        (["foo", "bar"], "or", "'bar' or 'foo'"),
        (["foo", "bar", "baz"], "or", "'bar', 'baz', or 'foo'"),
        (["foo", "bar", "baz", "qux"], "or", "'bar', 'baz', 'foo', or 'qux'"),
    ),
)
def test_humanize_list(items, conjunction, expected):
    assert utils.humanize_list(items, conjunction) == expected


#################
# Library Paths #
#################


@pytest.mark.parametrize(
    "lib_dirs,expected_env",
    [
        (["lib"], "$SNAP/lib"),
        (["lib", "usr/lib"], "$SNAP/lib:$SNAP/usr/lib"),
        (
            ["lib/i286-none-none", "usr/lib/i286-none-none"],
            "$SNAP/lib:$SNAP/usr/lib:$SNAP/lib/i286-none-none:$SNAP/usr/lib/i286-none-none",
        ),
    ],
)
def test_get_ld_library_paths(tmp_path, lib_dirs, expected_env):
    for d in lib_dirs:
        (tmp_path / d).mkdir(parents=True)

    expected_env = (
        f"${{SNAP_LIBRARY_PATH}}${{LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}}:{expected_env}"
    )
    assert utils.get_ld_library_paths(tmp_path, "i286-none-none") == expected_env


###################
# Process Version #
###################


def test_process_version_empty(mocker):
    """``version:None`` raises an error."""
    with pytest.raises(ValueError):
        utils.process_version(None)


def test_process_version_dirty(mocker):
    """A version string should be returned unmodified."""
    assert utils.process_version("1.2.3") == "1.2.3"


def test_process_version_git(mocker):
    """``version:git`` must be correctly handled."""
    mocker.patch(
        "craft_parts.sources.git_source.GitSource.generate_version",
        return_value="1.2.3-dirty",
    )

    assert utils.process_version("git") == "1.2.3-dirty"
