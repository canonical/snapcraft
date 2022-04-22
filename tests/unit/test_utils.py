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
    "platform_arch,deb_arch",
    [
        ("AMD64", "amd64"),
        ("aarch64", "arm64"),
        ("armv7l", "armhf"),
        ("ppc", "powerpc"),
        ("ppc64le", "ppc64el"),
        ("x86_64", "amd64"),
        ("unknown-arch", "unknown-arch"),
    ],
)
def test_get_host_architecture(platform_arch, mocker, deb_arch):
    """Test all platform mappings in addition to unknown."""
    mocker.patch("platform.machine", return_value=platform_arch)

    assert utils.get_host_architecture() == deb_arch
