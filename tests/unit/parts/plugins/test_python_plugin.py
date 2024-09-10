# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License version 3 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from textwrap import dedent

import pytest
from craft_parts import Part, PartInfo, ProjectInfo, Step, StepInfo, errors

from snapcraft.parts.plugins import PythonPlugin


@pytest.fixture
def part_info(new_dir):
    yield PartInfo(
        project_info=ProjectInfo(
            application_name="test",
            project_name="test-snap",
            base="core22",
            confinement="strict",
            project_base="core22",
            cache_dir=new_dir,
        ),
        part=Part("my-part", {}),
    )


@pytest.fixture
def plugin(part_info):
    properties = PythonPlugin.properties_class.unmarshal({"source": "."})
    yield PythonPlugin(properties=properties, part_info=part_info)


def test_get_build_snaps(plugin):
    assert plugin.get_build_snaps() == set()


def test_get_build_packages(plugin):
    assert plugin.get_build_packages() == {"python3-venv", "python3-dev", "findutils"}


def test_get_build_environment(plugin, new_dir):
    assert plugin.get_build_environment() == {
        "PARTS_PYTHON_INTERPRETER": "python3",
        "PARTS_PYTHON_VENV_ARGS": "",
        "PATH": f"{str(new_dir)}/parts/my-part/install/bin:${{PATH}}",
    }


def test_get_build_commands(plugin, new_dir):

    assert plugin.get_build_commands() == [
        f'"${{PARTS_PYTHON_INTERPRETER}}" -m venv ${{PARTS_PYTHON_VENV_ARGS}} "{new_dir}/parts/my-part/install"',
        f'PARTS_PYTHON_VENV_INTERP_PATH="{new_dir}/parts/my-part/install/bin/${{PARTS_PYTHON_INTERPRETER}}"',
        f"{new_dir}/parts/my-part/install/bin/pip install  -U pip setuptools wheel",
        f"[ -f setup.py ] || [ -f pyproject.toml ] && {new_dir}/parts/my-part/install/bin/pip install  -U .",
        f'find "{new_dir}/parts/my-part/install" -type f -executable -print0 | xargs -0 \\\n'
        '    sed -i "1 s|^#\\!${PARTS_PYTHON_VENV_INTERP_PATH}.*$|#!/usr/bin/env ${PARTS_PYTHON_INTERPRETER}|"\n',
        dedent(
            f"""\
            # look for a provisioned python interpreter
            opts_state="$(set +o|grep errexit)"
            set +e
            install_dir="{new_dir}/parts/my-part/install/usr/bin"
            stage_dir="{new_dir}/stage/usr/bin"

            # look for the right Python version - if the venv was created with python3.10,
            # look for python3.10
            basename=$(basename $(readlink -f ${{PARTS_PYTHON_VENV_INTERP_PATH}}))
            echo Looking for a Python interpreter called \\"${{basename}}\\" in the payload...
            payload_python=$(find "$install_dir" "$stage_dir" -type f -executable -name "${{basename}}" -print -quit 2>/dev/null)

            if [ -n "$payload_python" ]; then
                # We found a provisioned interpreter, use it.
                echo Found interpreter in payload: \\"${{payload_python}}\\"
                installed_python="${{payload_python##{new_dir}/parts/my-part/install}}"
                if [ "$installed_python" = "$payload_python" ]; then
                    # Found a staged interpreter.
                    symlink_target="..${{payload_python##{new_dir}/stage}}"
                else
                    # The interpreter was installed but not staged yet.
                    symlink_target="..$installed_python"
                fi
            else
                # Otherwise use what _get_system_python_interpreter() told us.
                echo "Python interpreter not found in payload."
                symlink_target="/usr/bin/python3.10"
            fi

            if [ -z "$symlink_target" ]; then
                echo "No suitable Python interpreter found, giving up."
                exit 1
            fi

            eval "${{opts_state}}"
            """
        ),
        'ln -sf "${symlink_target}" "${PARTS_PYTHON_VENV_INTERP_PATH}"',
    ]


def test_should_remove_symlinks(plugin):
    assert plugin._should_remove_symlinks() is False

    assert plugin._get_system_python_interpreter() == "/usr/bin/python3.10"


@pytest.mark.parametrize(
    "confinement,interpreter",
    [
        ("strict", "/usr/bin/python3.10"),
        ("classic", None),
        ("devmode", "/usr/bin/python3.10"),
    ],
)
def test_get_system_python_interpreter(confinement, interpreter, new_dir):
    part_info = PartInfo(
        project_info=ProjectInfo(
            application_name="test",
            project_name="test-snap",
            base="core22",
            confinement=confinement,
            project_base="core22",
            cache_dir=new_dir,
        ),
        part=Part("my-part", {}),
    )
    properties = PythonPlugin.properties_class.unmarshal({"source": "."})
    plugin = PythonPlugin(properties=properties, part_info=part_info)

    assert plugin._get_system_python_interpreter() == interpreter


@pytest.mark.parametrize(
    "confinement,interpreter",
    [
        ("strict", None),
        ("classic", None),
        ("devmode", None),
    ],
)
def test_get_system_python_interpreter_base_bare(confinement, interpreter, new_dir):
    part_info = PartInfo(
        project_info=ProjectInfo(
            application_name="test",
            project_name="test-snap",
            base="bare",
            build_base="core22",
            confinement=confinement,
            project_base="bare",
            cache_dir=new_dir,
        ),
        part=Part("my-part", {}),
    )
    properties = PythonPlugin.properties_class.unmarshal({"source": "."})
    plugin = PythonPlugin(properties=properties, part_info=part_info)

    assert plugin._get_system_python_interpreter() == interpreter


@pytest.mark.parametrize("confinement", ["strict", "devmode"])
def test_get_system_python_interpreter_unknown_base(confinement, new_dir):
    """Test that unknown bases raise errors for confined snaps."""
    part_info = PartInfo(
        project_info=ProjectInfo(
            application_name="test",
            project_name="test-snap",
            base="core22",
            confinement=confinement,
            project_base="core10",  # A base that the PythonPlugin doesn't know
            cache_dir=new_dir,
        ),
        part=Part("my-part", {}),
    )
    properties = PythonPlugin.properties_class.unmarshal({"source": "."})
    plugin = PythonPlugin(properties=properties, part_info=part_info)

    expected_error = "Don't know which interpreter to use for base core10"
    with pytest.raises(errors.PartsError, match=expected_error):
        plugin._get_system_python_interpreter()


@pytest.mark.parametrize("home_attr", ["part_install_dir", "stage_dir"])
def test_fix_pyvenv(new_dir, home_attr):
    part_info = PartInfo(
        project_info=ProjectInfo(
            application_name="test",
            project_name="test-snap",
            base="core24",
            confinement="classic",
            project_base="core24",
            cache_dir=new_dir,
        ),
        part=Part("my-part", {"plugin": "python"}),
    )

    prime_dir = part_info.prime_dir
    prime_dir.mkdir()

    pyvenv = prime_dir / "pyvenv.cfg"
    pyvenv.write_text(
        dedent(
            f"""\
        home = {getattr(part_info, home_attr)}/usr/bin
        include-system-site-packages = false
        version = 3.12.3
        executable = /root/parts/my-part/install/usr/bin/python3.12
        command = /root/parts/my-part/install/usr/bin/python3 -m venv /root/parts/my-part/install
        """
        )
    )

    step_info = StepInfo(part_info, Step.PRIME)

    PythonPlugin.post_prime(step_info)

    new_contents = pyvenv.read_text()
    assert "home = /snap/test-snap/current/usr/bin" in new_contents
