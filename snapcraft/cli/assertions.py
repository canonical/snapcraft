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

import sys

import click

import snapcraft
from snapcraft.storeapi import errors
from . import echo


@click.group()
def assertionscli():
    """Store assertion commands"""
    pass


@assertionscli.command('list-keys')
def list_keys():
    """List the keys available to sign assertions.

    This command has an alias of `keys`.
    """
    try:
        snapcraft.list_keys()
    except Exception as e:
        echo.error(e)
        sys.exit(1)


@assertionscli.command('create-key')
@click.argument('key-name', metavar='<key-name>', required=False)
def create_key(key_name):
    """Create a key to sign assertions."""
    try:
        snapcraft.create_key(key_name)
    except Exception as e:
        echo.error(e)
        sys.exit(1)


@assertionscli.command('register-key')
@click.argument('key-name', metavar='<key-name>', required=False)
def register_key(key_name):
    """Register a key with the store to sign assertions."""
    try:
        snapcraft.register_key(key_name)
    except Exception as e:
        echo.error(e)
        sys.exit(1)


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
    try:
        snapcraft.sign_build(snap_file, key_name=key_name, local=local)
    except Exception as e:
        echo.error(e)
        sys.exit(1)


@assertionscli.command()
@click.argument('snap-name', metavar='<snap-name>')
@click.argument('validations', metavar='<validation>...',
                nargs=-1, required=True)
@click.option('--key-name', metavar='<key-name>')
def validate(snap_name, validations, key_name):
    """Validate a gated snap."""
    try:
        snapcraft.validate(snap_name, validations, key=key_name)
    except errors.StoreError as e:
        echo.error(e)
        sys.exit(1)
    # This one is here until an assertions refactor
    except RuntimeError:
        sys.exit(1)


@assertionscli.command()
@click.argument('snap-name', metavar='<snap-name>')
def gated(snap_name):
    """Get the list of snaps and revisions gating a snap."""
    try:
        snapcraft.gated(snap_name)
    except errors.StoreError as e:
        echo.error(e)
        sys.exit(1)
    # This one is here until an assertions refactor
    except RuntimeError:
        sys.exit(1)
