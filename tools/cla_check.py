#!/usr/bin/env python

# XXX copied from https://github.com/kyrofa/cla-check
import collections
import os
import sys
from subprocess import check_output


try:
    from launchpadlib.launchpad import Launchpad
except ImportError:
    sys.exit('Install launchpadlib: sudo apt install python-launchpadlib')

Commit = collections.namedtuple('Commit', ['email', 'hash'])


def get_commits_for_range(range):
    output = check_output(['git', 'log', range, '--pretty=%aE|%H'])
    split_output = (i.split('|') for i in output.split('\n') if i != '')
    commits = [Commit(email=i[0], hash=i[1]) for i in split_output]
    return commits


def get_emails_from_commits(commits):
    emails = set()
    print('remotes', check_output(['git', 'remote', '-v']))
    for c in commits:
        output = check_output(['git', 'branch', '--contains', c.hash])
        if 'master' not in output.split('\n'):
            print('TRACE: "master" not in {}'.format(output))
            emails.add(c.email)
        else:  # just for debug
            print('Skipping {}, found in:\n{}'.format(c.hash, output))
    return emails


def main():
    travis_commit_range = os.getenv('TRAVIS_COMMIT_RANGE', '')
    commits = get_commits_for_range(travis_commit_range)
    emails = get_emails_from_commits(commits)
    if not emails:
        return

    print('Logging into Launchpad...')
    lp = Launchpad.login_anonymously('check CLA', 'production')
    cla_folks = lp.people['contributor-agreement-canonical'].participants

    for email in emails:
        if email.endswith('@canonical.com'):
            print('Skipping @canonical.com account {}'.format(email))
            continue
        print('Checking for a Launchpad account associated with {}...'.format(
            email))
        contributor = lp.people.getByEmail(email=email)
        if not contributor:
            sys.exit('The contributor does not have a Launchpad account.')

        print('Contributor account for {}: {}'.format(email, contributor))
        if contributor in cla_folks:
            print('The contributor has signed the CLA.')
        else:
            sys.exit('The contributor {} has not signed the CLA.'.format(
                email))


if __name__ == '__main__':
    main()
