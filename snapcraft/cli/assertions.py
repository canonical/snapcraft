# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import click

import snapcraft


@click.group()
def assertionscli():
    """Store assertion commands"""
    pass


@assertionscli.command('list-keys')
def list_keys():
    """List the keys available to sign assertions.

    This command has an alias of `keys`.
    """
    snapcraft.list_keys()


@assertionscli.command('create-key')
@click.argument('key-name', metavar='<key-name>', required=False)
def create_key(key_name):
    """Create a key to sign assertions."""
    snapcraft.create_key(key_name)


@assertionscli.command('register-key')
@click.argument('key-name', metavar='<key-name>', required=False)
def register_key(key_name):
    """Register a key with the store to sign assertions."""
    snapcraft.register_key(key_name)


@assertionscli.command('sign-build')
@click.option('--key-name', metavar='<key-name>')
@click.argument('snap-file', metavar='<snap-file>',
                type=click.Path(exists=True,
                                readable=True,
                                resolve_path=True,
                                dir_okay=False))
@click.option('--local', is_flag=True,
              help='Do not push the generated assertion to the store')
def sign_build(snap_file, key_name, local):
    """Sign a built snap file and assert it using the developer's key."""
    snapcraft.sign_build(snap_file, key_name=key_name, local=local)


@assertionscli.command()
@click.argument('snap-name', metavar='<snap-name>')
@click.argument('validations', metavar='<validation>...',
                nargs=-1, required=True)
@click.option('--key-name', metavar='<key-name>')
def validate(snap_name, validations, key_name):
    """Validate a gated snap."""
    snapcraft.validate(snap_name, validations, key=key_name)


@assertionscli.command()
@click.argument('snap-name', metavar='<snap-name>')
def gated(snap_name):
    """Get the list of snaps and revisions gating a snap."""
    snapcraft.gated(snap_name)
