# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016, 2017 Canonical Ltd
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

import json
import logging
import os
import re
import urllib.parse

from pyramid import response
import pymacaroons

from snapcraft.tests.fake_servers import base


logger = logging.getLogger(__name__)


class FakeStoreAPIServer(base.BaseFakeServer):

    _DEV_API_PATH = '/dev/api/'

    def __init__(self, fake_store, server_address):
        super().__init__(server_address)
        self.fake_store = fake_store
        self.account_keys = []
        self.registered_names = []

    def configure(self, configurator):
        # POST
        configurator.add_route(
            'acl', urllib.parse.urljoin(self._DEV_API_PATH, 'acl/'),
            request_method='POST')
        configurator.add_view(self.acl, route_name='acl')

        configurator.add_route(
            'account_key',
            urllib.parse.urljoin(self._DEV_API_PATH, 'account/account-key'),
            request_method='POST')
        configurator.add_view(self.account_key, route_name='account_key')

        configurator.add_route(
            'snap_builds',
            urllib.parse.urljoin(self._DEV_API_PATH, 'snaps/{dummy}/builds'),
            request_method='POST')
        configurator.add_view(self.snap_builds, route_name='snap_builds')

        configurator.add_route(
            'snap_close',
            urllib.parse.urljoin(self._DEV_API_PATH, 'snaps/{dummy}/close'),
            request_method='POST')
        configurator.add_view(self.snap_close, route_name='snap_close')

        configurator.add_route(
            'snap_push', urllib.parse.urljoin(
                self._DEV_API_PATH, 'snap-push/'),
            request_method='POST')
        configurator.add_view(self.snap_push, route_name='snap_push')

        configurator.add_route(
            'snap_release',
            urllib.parse.urljoin(self._DEV_API_PATH, 'snap-release/'),
            request_method='POST')
        configurator.add_view(self.snap_release, route_name='snap_release')

        configurator.add_route(
            'register_name',
            urllib.parse.urljoin(self._DEV_API_PATH, 'register-name/'),
            request_method='POST')
        configurator.add_view(self.register_name, route_name='register_name')

        configurator.add_route(
            'agreement', urllib.parse.urljoin(
                self._DEV_API_PATH, 'agreement/'),
            request_method='POST')
        configurator.add_view(self.agreement, route_name='agreement')

        # GET
        configurator.add_route(
            'details',
            urllib.parse.urljoin(
                self._DEV_API_PATH, '/details/upload-id/{snap}'),
            request_method='GET')
        configurator.add_view(self.details, route_name='details')

        configurator.add_route(
            'account', urllib.parse.urljoin(self._DEV_API_PATH, 'account'),
            request_method='GET')
        configurator.add_view(self.account, route_name='account')

        configurator.add_route(
            'snap_history',
            urllib.parse.urljoin(self._DEV_API_PATH, 'snaps/{dummy}/history'),
            request_method='GET')
        configurator.add_view(self.snap_history, route_name='snap_history')

        configurator.add_route(
            'snap_state',
            urllib.parse.urljoin(self._DEV_API_PATH, 'snaps/{dummy}/state'),
            request_method='GET')
        configurator.add_view(self.snap_state, route_name='snap_state')

        configurator.add_route(
            'snap_validations',
            urllib.parse.urljoin(
                self._DEV_API_PATH, 'snaps/{snap_id}/validations'),
            request_method='GET')
        configurator.add_view(
            self.snap_validations, route_name='snap_validations')

        configurator.add_route(
            'snap_developers',
            urllib.parse.urljoin(
                self._DEV_API_PATH, 'snaps/{snap_id}/developers'),
            request_method='GET')
        configurator.add_view(
            self.snap_developers, route_name='snap_developers')

        # PUT
        configurator.add_route(
            'put_snap_validations',
            urllib.parse.urljoin(
                self._DEV_API_PATH, 'snaps/{snap_id}/validations'),
            request_method='PUT')
        configurator.add_view(
            self.put_snap_validations, route_name='put_snap_validations')

        configurator.add_route(
            'put_snap_developers',
            urllib.parse.urljoin(
                self._DEV_API_PATH, 'snaps/{snap_id}/developers'),
            request_method='PUT')
        configurator.add_view(
            self.put_snap_developers, route_name='put_snap_developers')

    def _refresh_error(self):
        error = {
            'code': 'macaroon-permission-required',
            'message': 'Authorization Required',
        }
        payload = json.dumps({'error_list': [error]}).encode()
        response_code = 401
        content_type = 'application/json'
        headers = [
            ('Content-Type', content_type),
            ('WWW-Authenticate', 'Macaroon needs_refresh=1')
        ]
        return response.Response(payload, response_code, headers)

    # POST

    def acl(self, request):
        permission = request.path.split('/')[-1]
        logger.debug(
            'Handling ACL request for {}'.format(permission))
        sso_host = urllib.parse.urlparse(os.environ.get(
            'UBUNTU_SSO_API_ROOT_URL', "http://localhost")).netloc
        macaroon = pymacaroons.Macaroon(
            caveats=[
                pymacaroons.Caveat(
                    caveat_id='test caveat',
                    location=sso_host,
                    verification_key_id='test verifiacion')
            ])
        payload = json.dumps({'macaroon': macaroon.serialize()}).encode()
        response_code = 200
        content_type = 'application/json'
        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    def account_key(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        data = request.json_body
        logger.debug(
            'Handling account-key request with content {}'.format(data))
        account_key_request = data['account_key_request']

        if account_key_request == 'test-not-implemented':
            return self._account_key_not_implemented()
        elif account_key_request == 'test-invalid-data':
            return self._account_key_invalid_field()
        else:
            return self._account_key_successful(account_key_request)

    def _account_key_not_implemented(self):
        payload = b'Not Implemented'
        response_code = 501
        content_type = 'text/plain'
        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    def _account_key_invalid_field(self):
        error = {
            'error_list': [{
                'code': 'invalid-field',
                'message': 'The account-key-request assertion is not valid.',
            }]
        }
        payload = json.dumps(error).encode()
        response_code = 400
        content_type = 'application/json'
        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    def _account_key_successful(self, account_key_request):
        # Extremely basic assertion parsing, just enough to make tests work.
        # Don't copy this.
        key_name = re.search(
            '^name: (.*)$', account_key_request, flags=re.MULTILINE).group(1)
        key_id = re.search(
            '^public-key-sha3-384: (.*)$', account_key_request,
            flags=re.MULTILINE).group(1)
        self.account_keys.append(
            {'name': key_name, 'public-key-sha3-384': key_id})
        account_key = {
            'account_key': {
                'account-id': 'abcd',
                'name': key_name,
                'public-key-sha3-384': key_id,
            },
        }
        payload = json.dumps(account_key).encode()
        response_code = 200
        content_type = 'application/json'
        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    def snap_builds(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        logger.debug('Handling sign-build request')
        snap_build = request.json_body['assertion']
        if snap_build == 'test-not-implemented':
            payload = json.dumps({
                'error_list': [
                    {'code': 'feature-disabled',
                     'message': ('The snap-build assertions are currently '
                                 'disabled.')},
                ],
            }).encode()
            response_code = 501
            content_type = 'application/json'
        elif snap_build == 'test-invalid-data':
            payload = json.dumps({
                'error_list': [
                    {'code': 'invalid-field',
                     'message': 'The snap-build assertion is not valid.'},
                ],
            }).encode()
            response_code = 400
            content_type = 'application/json'
        elif snap_build == 'test-unexpected-data':
            payload = b'unexpected chunk of data'
            response_code = 500
            content_type = 'text/plain'
        else:
            payload = json.dumps({
                'type': 'snap-build',
                'foo': 'bar',
            }).encode()
            response_code = 200
            content_type = 'application/json'
        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    def snap_close(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        logger.debug('Handling close request')
        channels = request.json_body['channels']

        if channels == ['invalid']:
            payload = json.dumps({
                'error_list': [
                    {'code': 'invalid-field',
                     'message': ('The \'channels\' field content is not '
                                 'valid.')},
                ],
            }).encode()
            response_code = 400
            content_type = 'application/json'
        elif channels == ['unexpected']:
            payload = b'unexpected chunk of data'
            response_code = 500
            content_type = 'text/plain'
        elif channels == ['broken-plain']:
            payload = b'plain data'
            response_code = 200
            content_type = 'text/plain'
        elif channels == ['broken-json']:
            payload = json.dumps({
                'closed_channels': channels,
            }).encode()
            response_code = 200
            content_type = 'application/json'
        else:
            payload = json.dumps({
                'closed_channels': channels,
                'channel_map_tree': {
                    'latest': {
                        '16': {
                            'amd64': [
                                {'channel': 'stable', 'info': 'none'},
                                {'channel': 'candidate', 'info': 'none'},
                                {'channel': 'beta', 'info': 'specific',
                                 'version': '1.1', 'revision': 42},
                                {'channel': 'edge', 'info': 'tracking'}
                            ]

                        }
                    }
                },
            }).encode()
            response_code = 200
            content_type = 'application/json'

        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    def snap_push(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        logger.debug(
            'Handling upload request with content {}'.format(
                request.json_body))

        name = request.json_body['name']
        if name == 'test-snap-unregistered':
            payload = b''
            response_code = 404
            content_type = 'text/plain'
        else:
            response_code = 202
            content_type = 'application/json'
            if name == 'test-review-snap':
                details_path = 'details/upload-id/review-snap'
            elif name == 'test-duplicate-snap':
                details_path = 'details/upload-id/duplicate-snap'
            else:
                details_path = 'details/upload-id/good-snap'
            payload = json.dumps({
                'status_details_url': urllib.parse.urljoin(
                    'http://localhost:{}/'.format(self.server_port),
                    details_path
                    ),
            }).encode()
        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    def snap_release(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        logger.debug(
            'Handling release request with content {}'.format(
                request.json_body))
        response_code = 200
        content_type = 'application/json'
        name = request.json_body['name']
        channels = request.json_body['channels']
        revision = request.json_body['revision']
        if name == 'test-snap-unregistered':
            response_code = 404
            content_type = 'text/plain'
            payload = b''
        elif 'alpha' in channels:
            response_code = 400
            payload = json.dumps({
                'errors': 'Not a valid channel: alpha',
            }).encode()
        elif (
                name == 'test-snap' or
                name.startswith('snapcrafttest')):
            payload = json.dumps({
                'opened_channels': channels,
                'channel_map': [
                    {'channel': 'stable', 'info': 'none'},
                    {'channel': 'candidate', 'info': 'none'},
                    {'revision': int(revision),
                     'channel': 'beta',
                     'version': '0', 'info': 'specific'},
                    {'channel': 'edge', 'info': 'tracking'}
                ]
            }).encode()
        elif name.startswith('arm-'):
            payload = json.dumps({
                'opened_channels': channels,
                'channel_map_tree': {
                    '0.1': {
                        '16': {
                            'armhf':
                                [
                                    {'channel': 'stable', 'info': 'none'},
                                    {'channel': 'candidate', 'info': 'none'},
                                    {'revision': int(revision),
                                     'channel': 'beta',
                                     'version': '0', 'info': 'specific'},
                                    {'channel': 'edge', 'info': 'tracking'}
                                ]
                        }
                    }
                }
            }).encode()
        elif name.startswith('multiarch-'):
            payload = json.dumps({
                'opened_channels': channels,
                'channel_map_tree': {
                    '0.1': {
                        '16': {
                            'amd64':
                                [
                                    {'channel': 'stable', 'info': 'none'},
                                    {'channel': 'candidate', 'info': 'none'},
                                    {'revision': int(revision),
                                     'channel': 'beta',
                                     'version': '0', 'info': 'specific'},
                                    {'channel': 'edge', 'info': 'tracking'}
                                ],
                            'armhf':
                                [
                                    {'channel': 'stable', 'info': 'none'},
                                    {'channel': 'candidate', 'info': 'none'},
                                    {'revision': int(revision),
                                     'channel': 'beta',
                                     'version': '0', 'info': 'specific'},
                                    {'channel': 'edge', 'info': 'tracking'}
                                ]
                        }
                    }
                }
            }).encode()
        else:
            raise NotImplementedError(
                'Cannot handle release request for {!r}'.format(name))
        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    def register_name(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        logger.debug(
            'Handling registration request with content {}'.format(
                request.json_body))
        snap_name = request.json_body['snap_name']

        if snap_name == 'test-already-registered-snap-name':
            return self._register_name_409_error('already_registered')
        elif snap_name == 'test-reserved-snap-name':
            return self._register_name_409_error('reserved_name')
        elif snap_name == 'test-already-owned-snap-name':
            return self._register_name_409_error('already_owned')
        elif snap_name.startswith('test-too-fast'):
            return self._register_name_429_error('register_window')
        elif (
                snap_name.startswith('test_invalid') or
                len(snap_name) > 40):
            return self._register_name_invalid(snap_name)
        elif snap_name == 'snap-name-no-clear-error':
            return self._register_name_unclear_error()
        else:
            return self._register_name_successful(
                snap_name, request.json_body['is_private'])

    def _register_name_409_error(self, error_code):
        payload = json.dumps({
            'status': 409,
            'code': error_code,
            'register_name_url': 'https://myapps.com/register-name/',
        }).encode()
        response_code = 409
        content_type = 'application/json'
        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    def _register_name_429_error(self, error_code):
        error = {
            'status': 429,
            'code': error_code,
        }
        if error_code == 'register_window':
            error['retry_after'] = 177
        payload = json.dumps(error).encode()
        response_code = 429
        content_type = 'application/json'
        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    def _register_name_invalid(self, snap_name):
        # Emulates the current Store behaviour and never combines errors.
        # It's either using invalid chars or too long, never both.
        if len(snap_name) > 40:
            msg = ('The name {} should not be longer than 40 characters.'
                   .format(snap_name))
        else:
            msg = (
                'The name {!r} is not valid. It can only contain dashes, '
                'numbers and lowercase ascii letters.'.format(snap_name))

        payload = json.dumps({
            'error_list': [
                {'code': 'invalid', 'message': msg},
            ]
        }).encode()
        response_code = 400
        content_type = 'application/json'
        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    def _register_name_unclear_error(self):
        payload = json.dumps({
            'status': 409,
            'code': 'unexistent_error_code',
        }).encode()
        response_code = 409
        content_type = 'application/json'
        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    def _register_name_successful(self, name, is_private):
        self.registered_names.append((name, is_private))
        payload = json.dumps({'snap_id': 'test-snap-id'}).encode()
        response_code = 201
        content_type = 'application/json'
        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    def agreement(self, request):
        if 'STORE_DOWN' in os.environ:
            response_code = 500
            content_type = 'text/plain'
            payload = b'Broken'
        else:
            if request.json_body['latest_tos_accepted'] is not True:
                response_code = 400
                content_type = 'application/json'
                payload = json.dumps({
                    "error_list": [{
                        "message": "`latest_tos_accepted` must be `true`",
                        "code": "bad-request",
                        "extra": {"latest_tos_accepted": "true"}}]
                }).encode()
            else:
                response_code = 200
                content_type = 'application/json'
                payload = json.dumps({"content": {
                    "latest_tos_accepted": True,
                    "tos_url": 'http://fake-url.com',
                    "latest_tos_date": '2000-01-01',
                    "accepted_tos_date": '2010-10-10'
                    }
                }).encode()

        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    # GET

    def details(self, request):
        snap = request.matchdict['snap']
        if snap == 'duplicate-snap':
            logger.debug('Handling duplicate snap request')
            payload = json.dumps({
                'code': 'processing_error',
                'url': '/dev/click-apps/5349/rev/1',
                'can_release': False,
                'revision': '1',
                'processed': True,
                'errors': [
                    {'message': 'Duplicate snap already uploaded'},
                ]
            }).encode()
        else:
            logger.debug('Handling scan complete request')
            if snap == 'good-snap':
                can_release = True
                code = 'ready_to_release'
            elif snap == 'review-snap':
                can_release = False
                code = 'need_manual_review'
            payload = json.dumps({
                'code': code,
                'url': '/dev/click-apps/5349/rev/1',
                'can_release': can_release,
                'revision': '1',
                'processed': True
            }).encode()

        response_code = 200
        content_type = 'application/json'
        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    def account(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        logger.debug('Handling account request')
        snaps = {
            'basic': {'snap-id': 'snap-id', 'status': 'Approved',
                      'private': False, 'price': None,
                      'since': '2016-12-12T01:01:01Z'},
            'test-snap-with-no-validations': {
                'snap-id': 'test-snap-id-with-no-validations',
                'status': 'Approved',
                'private': False, 'price': None,
                'since': '2016-12-12T01:01:01Z'},
            'test-snap-with-dev': {'snap-id': 'test-snap-id-with-dev',
                                   'status': 'Approved',
                                   'private': False, 'price': None,
                                   'since': '2016-12-12T01:01:01Z'},
            'ubuntu-core': {'snap-id': 'good', 'status': 'Approved',
                            'private': False, 'price': None,
                            'since': '2016-12-12T01:01:01Z'},
            'core-no-dev': {'snap-id': 'no-dev', 'status': 'Approved',
                            'private': False, 'price': None,
                            'since': '2016-12-12T01:01:01Z'},
            'badrequest': {'snap-id': 'badrequest', 'status': 'Approved',
                           'private': False, 'price': None,
                           'since': '2016-12-12T01:01:01Z'},
            }
        snaps.update({
            name: {'snap-id': 'fake-snap-id', 'status': 'Approved',
                   'private': private, 'price': None,
                   'since': '2016-12-12T01:01:01Z'}
            for name, private in self.registered_names})
        payload = json.dumps({
            'account_id': 'abcd',
            'account_keys': self.account_keys,
            'snaps': {'16': snaps},
        }).encode()
        response_code = 200
        content_type = 'application/json'
        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    def snap_history(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        logger.debug('Handling account request')
        revisions = [{
            'series': ['16'],
            'channels': [],
            'version': '2.0.1',
            'timestamp': '2016-09-27T19:23:40Z',
            'current_channels': ['beta', 'edge'],
            'arch': 'i386',
            'revision': 2
        }, {
            'series': ['16'],
            'channels': ['stable', 'edge'],
            'version': '2.0.2',
            'timestamp': '2016-09-27T18:38:43Z',
            'current_channels': ['stable', 'candidate', 'beta'],
            'arch': 'amd64',
            'revision': 1,
        }]

        parsed_qs = urllib.parse.parse_qs(
            urllib.parse.urlparse(request.url).query)
        if 'arch' in parsed_qs:
            output = [
                rev for rev in revisions if rev['arch'] in parsed_qs['arch']]
        else:
            output = revisions
        response_code = 200
        content_type = 'application/json'
        return response.Response(
            json.dumps(output).encode(), response_code,
            [('Content-Type', content_type)])

    def snap_state(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        logger.debug('Handling snap state request')
        channel_map = {
            'channel_map_tree': {
                'latest': {
                    '16': {
                        'i386': [
                            {
                                'info': 'none',
                                'channel': 'stable'
                            },
                            {
                                'info': 'none',
                                'channel': 'beta'
                            },
                            {
                                'info': 'specific',
                                'version': '1.0-i386',
                                'channel': 'edge',
                                'revision': 3
                            },
                        ],
                        'amd64': [
                            {
                                'info': 'specific',
                                'version': '1.0-amd64',
                                'channel': 'stable',
                                'revision': 2
                            },
                            {
                                'info': 'specific',
                                'version': '1.1-amd64',
                                'channel': 'beta',
                                'revision': 4
                            },
                            {
                                'info': 'tracking',
                                'channel': 'edge'
                            },
                        ],

                    }
                }
            }
        }

        parsed_qs = urllib.parse.parse_qs(
            urllib.parse.urlparse(request.url).query)
        if 'architecture' in parsed_qs:
            arch = parsed_qs['architecture'][0]
            series = channel_map['channel_map_tree']['latest']['16']
            if arch in series:
                output = {
                    'channel_map_tree': {
                        'latest': {
                            '16': {
                                arch: series[arch]
                            }
                        }
                    }
                }
            else:
                output = {}
        else:
            output = channel_map
        response_code = 200
        content_type = 'application/json'
        return response.Response(
            json.dumps(output).encode(), response_code,
            [('Content-Type', content_type)])

    def snap_validations(self, request):
        logger.debug('Handling validation request')
        snap_id = request.matchdict['snap_id']
        if snap_id == 'good':
            validation = [{
                "approved-snap-id": "snap-id-1",
                "approved-snap-revision": "3",
                "approved-snap-name": "snap-1",
                "authority-id": "dev-1",
                "series": "16",
                "sign-key-sha3-384": "1234567890",
                "snap-id": "snap-id-gating",
                "timestamp": "2016-09-19T21:07:27.756001Z",
                "type": "validation",
                "revoked": "false",
                "required": True,
            }, {
                "approved-snap-id": "snap-id-2",
                "approved-snap-revision": "5",
                "approved-snap-name": "snap-2",
                "authority-id": "dev-1",
                "series": "16",
                "sign-key-sha3-384": "1234567890",
                "snap-id": "snap-id-gating",
                "timestamp": "2016-09-19T21:07:27.756001Z",
                "type": "validation",
                "revoked": "false",
                "required": False,
            }, {
                "approved-snap-id": "snap-id-3",
                "approved-snap-revision": "-",
                "approved-snap-name": "snap-3",
                "authority-id": "dev-1",
                "series": "16",
                "sign-key-sha3-384": "1234567890",
                "snap-id": "snap-id-gating",
                "timestamp": "2016-09-19T21:07:27.756001Z",
                "type": "validation",
                "revoked": "false",
                "required": True,
            }]
            payload = json.dumps(validation).encode()
            response_code = 200
        elif snap_id == 'bad':
            payload = 'foo'.encode()
            response_code = 200
        elif snap_id == 'test-snap-id-with-no-validations':
            payload = json.dumps([]).encode()
            response_code = 200
        elif snap_id == 'err':
            payload = json.dumps({
                'error_list': [{'message': 'error'}]
            }).encode()
            response_code = 503
        content_type = 'application/json'
        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    def snap_developers(self, request):
        logger.debug('Handling snap developers request')
        snap_id = request.matchdict['snap_id']
        if snap_id == 'good':
            payload = json.dumps({'snap_developer': {}}).encode()
            response_code = 200
        elif snap_id == 'test-snap-id-with-dev':
            payload = json.dumps({
                'snap_developer': {
                    'type': 'snap-developer',
                    'authority-id': 'dummy',
                    'publisher-id': 'dummy',
                    'snap-id': 'test-snap-id-with-dev',
                    'developers': [{
                        'developer-id': 'test-dev-id',
                        'since': '2017-02-10T08:35:00.390258Z',
                        'until': '2018-02-10T08:35:00.390258Z'
                    }]
                }
            }).encode()
            response_code = 200
        elif snap_id == 'no-dev':
            payload = json.dumps({'error_list': [
                {'message': 'error',
                 'code': 'snap-developer-not-found'}]
            }).encode()
            response_code = 403
        elif snap_id == 'badrequest':
            payload = json.dumps({'snap_developer': {}}).encode()
            response_code = 200
        content_type = 'application/json'
        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    # PUT

    def put_snap_validations(self, request):
        snap_id = request.matchdict['snap_id']
        if snap_id == 'good':
            payload = request.body
            response_code = 200
        elif snap_id == 'err':
            payload = json.dumps({
                'error_list': [{'message': 'error'}]
            }).encode()
            response_code = 501
        elif snap_id == 'bad':
            payload = 'foo'.encode()
            response_code = 200
        content_type = 'application/json'
        return response.Response(
            payload, response_code, [('Content-Type', content_type)])

    def put_snap_developers(self, request):
        snap_id = request.matchdict['snap_id']
        if snap_id == 'good':
            payload = request.body
            response_code = 200
        elif snap_id == 'no-dev':
            payload = request.body
            response_code = 200
        elif snap_id == 'badrequest':
            payload = json.dumps({'error_list': [
                {'message': "The given `snap-id` does not match the "
                            "assertion's.",
                 'code': 'invalid-request'}]}).encode()
            response_code = 400
        content_type = 'application/json'
        return response.Response(
            payload, response_code, [('Content-Type', content_type)])
