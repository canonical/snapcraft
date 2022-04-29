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

import textwrap
from pathlib import Path

import pytest
import yaml

from snapcraft.meta import snap_yaml
from snapcraft.projects import Project


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
            **kwargs,
        }
        return Project.unmarshal(snapcraft_config)

    yield _simple_project


def test_simple_snap_yaml(simple_project, new_dir):
    snap_yaml.write(
        simple_project(),
        prime_dir=Path(new_dir),
        target_arch="amd64",
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
        architectures:
          - arch

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

        layout:
          /usr/share/libdrm:
            bind: $SNAP/gnome-platform/usr/share/libdrm
          /usr/lib/x86_64-linux-gnu/webkit2gtk-4.0:
            bind: $SNAP/gnome-platform/usr/lib/x86_64-linux-gnu/webkit2gtk-4.0
          /usr/share/xml/iso-codes:
            bind: $SNAP/gnome-platform/usr/share/xml/iso-codes
        """
    )
    data = yaml.safe_load(snapcraft_yaml)
    yield Project.unmarshal(data)


def test_complex_snap_yaml(complex_project, new_dir):
    snap_yaml.write(
        complex_project,
        prime_dir=Path(new_dir),
        target_arch="amd64",
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
        target_arch="amd64",
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
        target_arch="amd64",
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
        target_arch="amd64",
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
        target_arch="amd64",
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
        target_arch="amd64",
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
        target_arch="amd64",
        arch_triplet="x86_64-linux-gnu",
    )

    yaml_file = Path("meta/snap.yaml")
    content = yaml_file.read_text()

    data = yaml.safe_load(content)
    assert data["version"] == "1.2.3"
