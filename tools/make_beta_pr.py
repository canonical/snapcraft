#!/usr/bin/python3
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

import os
import subprocess
import sys
import tempfile
import urllib.parse

import git
import github


def main():
    username = os.getenv('GITHUB_TEST_USER_NAME', None)
    if not username:
        sys.exit('Environment variable GITHUB_TEST_USER_NAME is not set')
    password = os.getenv('GITHUB_TEST_PASSWORD', None)
    if not password:
        sys.exit('Environment variable GITHUB_TEST_PASSWORD is not set')

    # Push the latest master to the beta branch of the test user.
    with tempfile.TemporaryDirectory() as temp_dir:
        git.Git(temp_dir).clone('https://github.com/snapcore/snapcraft')
        repo = git.Repo(os.path.join(temp_dir, 'snapcraft'))
        remote = repo.create_remote(
            'remote', 'https://{0}:{1}@github.com/{0}/snapcraft'.format(
                username, password))
        remote.push('refs/heads/master:refs/heads/beta')

    # Make a pull request.
    hub = github.Github(username, password)
    repo = hub.get_user('snapcore').get_repo('snapcraft')
    head = '{}:beta'.format(username)
    try:
        pr = repo.create_pull(
            title='beta', body='PR for beta testing',
            base='beta', head=head)
    except github.GithubException as e:
        pr_exists = 'A pull request already exists for {}.'.format(head)
        if e.status == 422 and e.data['errors'][0]['message'] == pr_exists:
            # Use the existing pull request.
            open_prs = repo.get_pulls('open')
            for pr in open_prs:
                if pr.user.login == username and pr.head.label == head:
                    break
            else:
                sys.exit('Existing beta pull request not found')
        else:
            raise

    # Trigger the autopkgtests.
    for test in ('xenial:amd64', 'xenial:armhf', 'xenial:arm64',
                 'yakkety:amd64', 'yakkety:armhf', 'yakkety:amd64',
                 'zesty:amd64', 'zesty:armhf', 'zesty:amd64'):
        subprocess.check_call(
            ['tools/retry_autopkgtest.sh', str(pr.number), test])


if __name__ == '__main__':
    main()
