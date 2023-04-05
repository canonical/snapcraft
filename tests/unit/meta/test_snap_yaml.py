# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2023 Canonical Ltd.
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

import textwrap
from pathlib import Path

import pydantic
import pytest
import yaml

from snapcraft.meta import snap_yaml
from snapcraft.meta.snap_yaml import ContentPlug, ContentSlot, SnapMetadata
from snapcraft.projects import Project


def _override_data(to_dict, from_dict):
    for key, value in from_dict.items():
        if key in to_dict:
            if isinstance(to_dict[key], dict) and isinstance(value, dict):
                _override_data(to_dict[key], value)
            else:
                to_dict[key] = value
        else:
            to_dict[key] = value


@pytest.fixture
def simple_project():
    def _simple_project(**kwargs):
        snapcraft_config = {
            "name": "mytest",
            "version": "1.29.3",
            "base": "core22",
            "summary": "Single-line elevator pitch for your amazing snap",
            "description": "test-description",
            "confinement": "strict",
            "parts": {
                "part1": {
                    "plugin": "nil",
                },
            },
            "apps": {
                "app1": {
                    "command": "bin/mytest",
                },
            },
        }
        _override_data(snapcraft_config, kwargs)
        return Project.unmarshal(snapcraft_config)

    yield _simple_project


def test_simple_snap_yaml(simple_project, new_dir):
    snap_yaml.write(
        simple_project(),
        prime_dir=Path(new_dir),
        arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )
    yaml_file = Path("meta/snap.yaml")
    assert yaml_file.is_file()

    content = yaml_file.read_text()
    assert content == textwrap.dedent(
        """\
        name: mytest
        version: 1.29.3
        summary: Single-line elevator pitch for your amazing snap
        description: test-description
        architectures:
        - amd64
        base: core22
        apps:
          app1:
            command: bin/mytest
        confinement: strict
        grade: stable
        environment:
          LD_LIBRARY_PATH: ${SNAP_LIBRARY_PATH}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
          PATH: $SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH
        """
    )


@pytest.fixture
def complex_project():
    snapcraft_yaml = textwrap.dedent(
        """\
        name: mytest
        version: 1.29.3
        base: core22
        type: app
        summary: Single-line elevator pitch for your amazing snap
        description: |
          This is my-snap's description. You have a paragraph or two to tell the
          most important story about your snap. Keep it under 100 words though,
          we live in tweetspace and your description wants to look good in the snap
          store.

        grade: devel
        confinement: strict

        environment:
          GLOBAL_VARIABLE: test-global-variable

        parts:
          part1:
            plugin: nil

        apps:
          app1:
            command: bin/mytest
            autostart: test-app.desktop
            common-id: test-common-id
            bus-name: test-bus-name
            completer: test-completer
            stop-command: test-stop-command
            post-stop-command: test-post-stop-command
            start-timeout: 1s
            stop-timeout: 2s
            watchdog-timeout: 3s
            reload-command: test-reload-command
            restart-delay: 4s
            timer: test-timer
            daemon: simple
            after: [test-after-1, test-after-2]
            before: [test-before-1, test-before-2]
            refresh-mode: endure
            stop-mode: sigterm
            restart-condition: on-success
            install-mode: enable
            aliases: [test-alias-1, test-alias-2]
            plugs: [test-plug-1, test-plug-2]
            slots: [test-slot-1, test-slot-2]
            environment:
              APP_VARIABLE: test-app-variable
            command-chain:
            - cc-test1
            - cc-test2
            sockets:
              test-socket-1:
                listen-stream: /tmp/test-socket.sock
                socket-mode: 0
              test-socket-2:
                listen-stream: 100
                socket-mode: 1

        plugs:
          empty-plug:
          string-plug: home
          dict-plug:
            string-parameter: foo
            bool-parameter: True
          content-interface:
            interface: content
            target: test-target
            content: test-content
            default-provider: test-provider

        slots:
          empty-slot:
          string-slot: slot
          dict-slot:
            string-parameter: foo
            bool-parameter: True
          content-interface:
            interface: content
            read:
              - /

        hooks:
          configure:
            command-chain: ["test"]
            environment:
              test-variable-1: "test"
              test-variable-2: "test"
            plugs:
              - home
              - network
          install:
            environment:
              environment-var-1: "test"

        system-usernames:
          snap_daemon:
            scope: shared
          snap_microk8s: shared
          snap_aziotedge: shared
          snap_aziotdu:
            scope: shared

        layout:
          /usr/share/libdrm:
            bind: $SNAP/gnome-platform/usr/share/libdrm
          /usr/lib/x86_64-linux-gnu/webkit2gtk-4.0:
            bind: $SNAP/gnome-platform/usr/lib/x86_64-linux-gnu/webkit2gtk-4.0
          /usr/share/xml/iso-codes:
            bind: $SNAP/gnome-platform/usr/share/xml/iso-codes

        provenance: test-provenance-1
        """
    )
    data = yaml.safe_load(snapcraft_yaml)
    yield Project.unmarshal(data)


def test_complex_snap_yaml(complex_project, new_dir):
    snap_yaml.write(
        complex_project,
        prime_dir=Path(new_dir),
        arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )
    yaml_file = Path("meta/snap.yaml")
    assert yaml_file.is_file()

    content = yaml_file.read_text()
    assert content == textwrap.dedent(
        """\
        name: mytest
        version: 1.29.3
        summary: Single-line elevator pitch for your amazing snap
        description: |
          This is my-snap's description. You have a paragraph or two to tell the
          most important story about your snap. Keep it under 100 words though,
          we live in tweetspace and your description wants to look good in the snap
          store.
        type: app
        architectures:
        - amd64
        base: core22
        assumes:
        - command-chain
        apps:
          app1:
            command: bin/mytest
            autostart: test-app.desktop
            common-id: test-common-id
            bus-name: test-bus-name
            completer: test-completer
            stop-command: test-stop-command
            post-stop-command: test-post-stop-command
            start-timeout: 1s
            stop-timeout: 2s
            watchdog-timeout: 3s
            reload-command: test-reload-command
            restart-delay: 4s
            timer: test-timer
            daemon: simple
            after:
            - test-after-1
            - test-after-2
            before:
            - test-before-1
            - test-before-2
            refresh-mode: endure
            stop-mode: sigterm
            restart-condition: on-success
            install-mode: enable
            plugs:
            - test-plug-1
            - test-plug-2
            slots:
            - test-slot-1
            - test-slot-2
            aliases:
            - test-alias-1
            - test-alias-2
            environment:
              APP_VARIABLE: test-app-variable
            command-chain:
            - cc-test1
            - cc-test2
            sockets:
              test-socket-1:
                listen-stream: /tmp/test-socket.sock
                socket-mode: 0
              test-socket-2:
                listen-stream: 100
                socket-mode: 1
        confinement: strict
        grade: devel
        environment:
          GLOBAL_VARIABLE: test-global-variable
          LD_LIBRARY_PATH: ${SNAP_LIBRARY_PATH}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
          PATH: $SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH
        plugs:
          empty-plug: null
          string-plug: home
          dict-plug:
            string-parameter: foo
            bool-parameter: true
          content-interface:
            content: test-content
            interface: content
            target: test-target
            default-provider: test-provider
        slots:
          empty-slot: null
          string-slot: slot
          dict-slot:
            string-parameter: foo
            bool-parameter: true
          content-interface:
            interface: content
            read:
            - /
        hooks:
          configure:
            command-chain:
            - test
            environment:
              test-variable-1: test
              test-variable-2: test
            plugs:
            - home
            - network
          install:
            environment:
              environment-var-1: test
        layout:
          /usr/share/libdrm:
            bind: $SNAP/gnome-platform/usr/share/libdrm
          /usr/lib/x86_64-linux-gnu/webkit2gtk-4.0:
            bind: $SNAP/gnome-platform/usr/lib/x86_64-linux-gnu/webkit2gtk-4.0
          /usr/share/xml/iso-codes:
            bind: $SNAP/gnome-platform/usr/share/xml/iso-codes
        system-usernames:
          snap_daemon:
            scope: shared
          snap_microk8s: shared
          snap_aziotedge: shared
          snap_aziotdu:
            scope: shared
        provenance: test-provenance-1
        """
    )


def test_hook_command_chain_assumes(simple_project, new_dir):
    hooks = {
        "hook": {
            "command-chain": ["c1"],
        },
    }

    snap_yaml.write(
        simple_project(hooks=hooks),
        prime_dir=Path(new_dir),
        arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )
    yaml_file = Path("meta/snap.yaml")
    assert yaml_file.is_file()

    content = yaml_file.read_text()
    assert content == textwrap.dedent(
        """\
        name: mytest
        version: 1.29.3
        summary: Single-line elevator pitch for your amazing snap
        description: test-description
        architectures:
        - amd64
        base: core22
        assumes:
        - command-chain
        apps:
          app1:
            command: bin/mytest
        confinement: strict
        grade: stable
        environment:
          LD_LIBRARY_PATH: ${SNAP_LIBRARY_PATH}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
          PATH: $SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH
        hooks:
          hook:
            command-chain:
            - c1
        """
    )


def test_project_environment_ld_library_path_and_path_defined(simple_project, new_dir):
    """Test behavior of defining LD_LIBRARY_PATH and PATH variables."""
    environment = {
        "LD_LIBRARY_PATH": "test-1",
        "PATH": "test-2",
    }
    snap_yaml.write(
        simple_project(environment=environment),
        prime_dir=Path(new_dir),
        arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )
    yaml_file = Path("meta/snap.yaml")
    assert yaml_file.is_file()

    content = yaml_file.read_text()
    assert content == textwrap.dedent(
        """\
        name: mytest
        version: 1.29.3
        summary: Single-line elevator pitch for your amazing snap
        description: test-description
        architectures:
        - amd64
        base: core22
        apps:
          app1:
            command: bin/mytest
        confinement: strict
        grade: stable
        environment:
          LD_LIBRARY_PATH: test-1
          PATH: test-2
        """
    )


def test_project_environment_ld_library_path_defined(simple_project, new_dir):
    """LD_LIBRARY_PATH can be overridden without affecting PATH."""
    environment = {"LD_LIBRARY_PATH": "test-1"}

    snap_yaml.write(
        simple_project(environment=environment),
        prime_dir=Path(new_dir),
        arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )
    yaml_file = Path("meta/snap.yaml")
    assert yaml_file.is_file()

    content = yaml_file.read_text()
    assert content == textwrap.dedent(
        """\
        name: mytest
        version: 1.29.3
        summary: Single-line elevator pitch for your amazing snap
        description: test-description
        architectures:
        - amd64
        base: core22
        apps:
          app1:
            command: bin/mytest
        confinement: strict
        grade: stable
        environment:
          LD_LIBRARY_PATH: test-1
          PATH: $SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH
        """
    )


def test_project_environment_path_defined(simple_project, new_dir):
    """PATH can be overridden without affecting LD_LIBRARY_PATH."""
    environment = {"PATH": "test-2"}
    snap_yaml.write(
        simple_project(environment=environment),
        prime_dir=Path(new_dir),
        arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )
    yaml_file = Path("meta/snap.yaml")
    assert yaml_file.is_file()

    content = yaml_file.read_text()
    assert content == textwrap.dedent(
        """\
        name: mytest
        version: 1.29.3
        summary: Single-line elevator pitch for your amazing snap
        description: test-description
        architectures:
        - amd64
        base: core22
        apps:
          app1:
            command: bin/mytest
        confinement: strict
        grade: stable
        environment:
          PATH: test-2
          LD_LIBRARY_PATH: ${SNAP_LIBRARY_PATH}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
        """
    )


def test_project_environment_ld_library_path_null(simple_project, new_dir):
    """LD_LIBRARY_PATH can be overridden without affecting PATH."""
    environment = {"LD_LIBRARY_PATH": None}
    snap_yaml.write(
        simple_project(environment=environment),
        prime_dir=Path(new_dir),
        arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )
    yaml_file = Path("meta/snap.yaml")
    assert yaml_file.is_file()

    content = yaml_file.read_text()
    assert content == textwrap.dedent(
        """\
        name: mytest
        version: 1.29.3
        summary: Single-line elevator pitch for your amazing snap
        description: test-description
        architectures:
        - amd64
        base: core22
        apps:
          app1:
            command: bin/mytest
        confinement: strict
        grade: stable
        environment:
          PATH: $SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH
        """
    )


def test_version_git(simple_project, new_dir, mocker):
    """Version in projects with ``version:git`` must be correctly handled."""
    mocker.patch(
        "craft_parts.sources.git_source.GitSource.generate_version",
        return_value="1.2.3",
    )

    snap_yaml.write(
        simple_project(version="git"),
        prime_dir=Path(new_dir),
        arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )

    yaml_file = Path("meta/snap.yaml")
    content = yaml_file.read_text()

    data = yaml.safe_load(content)
    assert data["version"] == "1.2.3"


def test_get_provider_content_directories(new_dir, mocker):
    mocker.patch(
        "snapcraft.meta.snap_yaml.read",
        return_value=SnapMetadata.unmarshal(
            {
                "name": "provider-snap",
                "version": "1",
                "summary": "summary",
                "description": "description",
                "confinement": "strict",
                "grade": "stable",
                "architectures": ["amd64"],
                "slots": {
                    "slot1": {
                        "interface": "content",
                        "content": "content-text",
                        "read": ["read"],
                        "write": ["write"],
                    }
                },
            }
        ),
    )
    mocker.patch("pathlib.Path.exists", return_value=True)

    yaml_data = textwrap.dedent(
        """\
        name: mytest
        version: 1.29.3
        summary: Single-line elevator pitch for your amazing snap
        description: test-description
        architectures:
        - amd64
        confinement: strict
        grade: stable
        plugs:
          test-plug:
            interface: content
            content: content-test
            target: target
            default_provider: my-test-provider
        """
    )

    metadata = SnapMetadata.unmarshal(yaml.safe_load(yaml_data))
    content_dirs = metadata.get_provider_content_directories()

    assert content_dirs == [
        Path("/snap/my-test-provider/current/read"),
        Path("/snap/my-test-provider/current/write"),
    ]


@pytest.mark.parametrize("grade", ["devel", "stable"])
def test_grade(grade, simple_project, new_dir):
    """Use the grade provided by the project."""
    snap_yaml.write(
        project=simple_project(grade=grade),
        prime_dir=Path(new_dir),
        arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )
    yaml_file = Path("meta/snap.yaml")
    assert yaml_file.is_file()

    data = yaml.safe_load(yaml_file.read_text())

    assert data["grade"] == grade


def test_grade_default(emitter, simple_project, new_dir):
    """Grade should default to `stable` when not provided by the project."""
    snap_yaml.write(
        project=simple_project(),
        prime_dir=Path(new_dir),
        arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )
    yaml_file = Path("meta/snap.yaml")
    assert yaml_file.is_file()

    data = yaml.safe_load(yaml_file.read_text())

    assert data["grade"] == "stable"

    emitter.assert_debug("Grade not specified, using default value 'stable'.")


def test_grade_build_base_devel(emitter, simple_project, new_dir):
    """Grade should default to `devel` when build_base is `devel`."""
    snap_yaml.write(
        project=simple_project(build_base="devel"),
        prime_dir=Path(new_dir),
        arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )
    yaml_file = Path("meta/snap.yaml")
    assert yaml_file.is_file()

    data = yaml.safe_load(yaml_file.read_text())

    assert data["grade"] == "devel"

    emitter.assert_debug("Setting grade to 'devel' because build_base is 'devel'.")


#####################
# Test content plug #
#####################


def test_content_plug_unmarshal():
    plug_dict = {"interface": "content", "content": "foo", "target": "target"}
    plug = ContentPlug.unmarshal(plug_dict)

    assert plug.interface == "content"
    assert plug.content == "foo"
    assert plug.target == "target"
    assert plug.provider is None


def test_content_plug_invalid_target():
    with pytest.raises(pydantic.ValidationError) as raised:
        ContentPlug.unmarshal({"interface": "content", "target": ""})

    err = raised.value.errors()
    assert len(err) == 1
    assert err[0]["loc"] == ("target",)
    assert err[0]["type"] == "value_error"
    assert err[0]["msg"] == "value cannot be empty"


def test_content_plug_provider():
    plug_dict = {
        "interface": "content",
        "content": "foo",
        "target": "target",
        "default-provider": "gtk-common-themes:gtk-3-themes",
    }
    plug = ContentPlug.unmarshal(plug_dict)

    assert plug.provider == "gtk-common-themes"


def test_get_content_plugs():
    yaml_data = textwrap.dedent(
        """\
        name: mytest
        version: 1.29.3
        summary: Single-line elevator pitch for your amazing snap
        description: test-description
        architectures:
        - amd64
        confinement: strict
        grade: stable
        plugs:
          plug1:
            interface: foo
          plug2:
            interface: content
            content: test
            target: target2
            default_provider: gtk-common-themes
            extra-stuff: anything
          plug3:
            interface: content
            target: target3
          plug4:
        """
    )

    metadata = SnapMetadata.unmarshal(yaml.safe_load(yaml_data))
    content_plugs = metadata.get_content_plugs()

    assert content_plugs == [
        ContentPlug(
            interface="content",
            target="target2",
            content="test",
            default_provider="gtk-common-themes",
        ),
        ContentPlug(
            interface="content",
            target="target3",
            content="plug3",
            default_provider=None,
        ),
    ]


#####################
# Test content slot #
#####################


def test_content_slot_empty():
    slot = ContentSlot.unmarshal({"interface": "content"})

    assert slot.interface == "content"
    assert slot.read == []
    assert slot.write == []


def test_content_slot_explicit_content():
    slot_dict = {
        "interface": "content",
        "content": "content-test",
        "read": ["read1", "read2"],
        "write": ["write1", "write2"],
    }
    slot = ContentSlot.unmarshal(slot_dict)

    assert slot.interface == "content"
    assert slot.content == "content-test"
    assert slot.read == ["read1", "read2"]
    assert slot.write == ["write1", "write2"]


def test_get_content_slots():
    yaml_data = textwrap.dedent(
        """\
        name: mytest
        version: 1.29.3
        summary: Single-line elevator pitch for your amazing snap
        description: test-description
        architectures:
        - amd64
        confinement: strict
        grade: stable
        slots:
          slot1:
            interface: foo
          slot2:
            interface: content
            content: test
            read:
              - read1
            write:
              - write1
          slot3:
            interface: content
            read:
              - read1
              - read2
          slot4:
        """
    )

    metadata = SnapMetadata.unmarshal(yaml.safe_load(yaml_data))
    content_slots = metadata.get_content_slots()

    assert content_slots == [
        ContentSlot(
            interface="content",
            content="test",
            read=["read1"],
            write=["write1"],
        ),
        ContentSlot(
            interface="content",
            content="slot3",
            read=["read1", "read2"],
            write=[],
        ),
    ]


def test_project_passthrough_snap_yaml(simple_project, new_dir):
    snap_yaml.write(
        simple_project(
            passthrough={
                "somefield": [
                    "some",
                    "value",
                ],
            },
        ),
        prime_dir=Path(new_dir),
        arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )
    yaml_file = Path("meta/snap.yaml")
    assert yaml_file.is_file()

    content = yaml_file.read_text()
    assert content == textwrap.dedent(
        """\
        name: mytest
        version: 1.29.3
        summary: Single-line elevator pitch for your amazing snap
        description: test-description
        architectures:
        - amd64
        base: core22
        apps:
          app1:
            command: bin/mytest
        confinement: strict
        grade: stable
        environment:
          LD_LIBRARY_PATH: ${SNAP_LIBRARY_PATH}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
          PATH: $SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH
        somefield:
        - some
        - value
        """
    )


def test_app_passthrough_snap_yaml(simple_project, new_dir):
    snap_yaml.write(
        simple_project(
            apps={
                "app1": {
                    "passthrough": {
                        "somefield": [
                            "some",
                            "value",
                        ],
                    },
                },
            },
        ),
        prime_dir=Path(new_dir),
        arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )
    yaml_file = Path("meta/snap.yaml")
    assert yaml_file.is_file()

    content = yaml_file.read_text()
    assert content == textwrap.dedent(
        """\
        name: mytest
        version: 1.29.3
        summary: Single-line elevator pitch for your amazing snap
        description: test-description
        architectures:
        - amd64
        base: core22
        apps:
          app1:
            command: bin/mytest
            somefield:
            - some
            - value
        confinement: strict
        grade: stable
        environment:
          LD_LIBRARY_PATH: ${SNAP_LIBRARY_PATH}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
          PATH: $SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH
        """
    )
