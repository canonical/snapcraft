# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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


"""Tests for the pygit2 wrapper class."""

import re
import subprocess
from pathlib import Path
from unittest.mock import ANY

import pygit2
import pytest

from snapcraft.remote import GitError, GitRepo, is_repo, is_shallow_repo


def test_is_repo(new_dir):
    """Check if directory is a repo."""
    GitRepo(new_dir)

    assert is_repo(new_dir)


def test_is_not_repo(new_dir):
    """Check if a directory is not a repo."""
    assert not is_repo(new_dir)


def test_is_shallow_repo(new_dir):
    """Check if directory is a shallow cloned repo."""
    root_path = Path(new_dir)
    git_normal_path = root_path / "normal"
    git_normal_path.mkdir()
    git_shallow_path = root_path / "shallow"

    repo_normal = GitRepo(git_normal_path)
    (repo_normal.path / "1").write_text("1")
    repo_normal.add_all()
    repo_normal.commit("1")

    (repo_normal.path / "2").write_text("2")
    repo_normal.add_all()
    repo_normal.commit("2")

    (repo_normal.path / "3").write_text("3")
    repo_normal.add_all()
    repo_normal.commit("3")

    # pygit2 does not support shallow cloning, so we use git directly
    subprocess.run(
        [
            "git",
            "clone",
            "--depth",
            "1",
            git_normal_path.absolute().as_uri(),
            git_shallow_path.absolute().as_posix(),
        ],
        check=True,
    )

    assert is_shallow_repo(git_shallow_path)


def test_is_repo_path_only(new_dir):
    """Only look at the path for a repo."""
    Path("parent-repo/not-a-repo/child-repo").mkdir(parents=True)
    # create the parent and child repos
    GitRepo(Path("parent-repo"))
    GitRepo(Path("parent-repo/not-a-repo/child-repo"))

    assert is_repo(Path("parent-repo"))
    assert not is_repo(Path("parent-repo/not-a-repo"))
    assert is_repo(Path("parent-repo/not-a-repo/child-repo"))


def test_is_repo_error(new_dir, mocker):
    """Raise an error if git fails to check a repo."""
    mocker.patch("pygit2.discover_repository", side_effect=pygit2.GitError)

    with pytest.raises(GitError) as raised:
        assert is_repo(new_dir)

    assert raised.value.details == (
        f"Could not check for git repository in {str(new_dir)!r}."
    )


def test_init_repo(new_dir):
    """Initialize a GitRepo object."""
    repo = GitRepo(new_dir)

    assert is_repo(new_dir)
    assert repo.path == new_dir


def test_init_existing_repo(new_dir):
    """Initialize a GitRepo object in an existing git repository."""
    # initialize a repo
    GitRepo(new_dir)

    # creating a new GitRepo object will not re-initialize the repo
    repo = GitRepo(new_dir)

    assert is_repo(new_dir)
    assert repo.path == new_dir


def test_init_repo_no_directory(new_dir):
    """Raise an error if the directory is missing."""
    with pytest.raises(FileNotFoundError) as raised:
        GitRepo(new_dir / "missing")

    assert str(raised.value) == (
        "Could not initialize a git repository because "
        f"{str(new_dir / 'missing')!r} does not exist or is not a directory."
    )


def test_init_repo_not_a_directory(new_dir):
    """Raise an error if the path is not a directory."""
    Path("regular-file").touch()

    with pytest.raises(FileNotFoundError) as raised:
        GitRepo(new_dir / "regular-file")

    assert str(raised.value) == (
        "Could not initialize a git repository because "
        f"{str(new_dir / 'regular-file')!r} does not exist or is not a directory."
    )


def test_init_repo_error(new_dir, mocker):
    """Raise an error if the repo cannot be initialized."""
    mocker.patch("pygit2.init_repository", side_effect=pygit2.GitError)

    with pytest.raises(GitError) as raised:
        GitRepo(new_dir)

    assert raised.value.details == (
        f"Could not initialize a git repository in {str(new_dir)!r}."
    )


def test_add_all(new_dir):
    """Add all files."""
    repo = GitRepo(new_dir)
    (repo.path / "foo").touch()
    (repo.path / "bar").touch()
    repo.add_all()

    status = pygit2.Repository(new_dir).status()

    assert status == {
        "foo": pygit2.GIT_STATUS_INDEX_NEW,
        "bar": pygit2.GIT_STATUS_INDEX_NEW,
    }


def test_add_all_no_files_to_add(new_dir):
    """`add_all` should succeed even if there are no files to add."""
    repo = GitRepo(new_dir)
    repo.add_all()

    status = pygit2.Repository(new_dir).status()

    assert status == {}


def test_add_all_error(new_dir, mocker):
    """Raise an error if the changes could not be added."""
    mocker.patch("pygit2.Index.add_all", side_effect=pygit2.GitError)
    repo = GitRepo(new_dir)

    with pytest.raises(GitError) as raised:
        repo.add_all()

    assert raised.value.details == (
        f"Could not add changes for the git repository in {str(new_dir)!r}."
    )


def test_commit(new_dir):
    """Commit a file and confirm it is in the tree."""
    repo = GitRepo(new_dir)
    (repo.path / "test-file").touch()
    repo.add_all()

    repo.commit()

    # verify commit (the `isinstance` checks are to satsify pyright)
    commit = pygit2.Repository(new_dir).revparse_single("HEAD")
    assert isinstance(commit, pygit2.Commit)
    assert commit.message == "auto commit"
    assert commit.committer.name == "auto commit"
    assert commit.committer.email == "auto commit"

    # verify tree
    tree = commit.tree
    assert isinstance(tree, pygit2.Tree)
    assert len(tree) == 1

    # verify contents of tree
    blob = tree[0]
    assert isinstance(blob, pygit2.Blob)
    assert blob.name == "test-file"


def test_commit_write_tree_error(new_dir, mocker):
    """Raise an error if the tree cannot be created."""
    mocker.patch("pygit2.Index.write_tree", side_effect=pygit2.GitError)
    repo = GitRepo(new_dir)
    (repo.path / "test-file").touch()
    repo.add_all()

    with pytest.raises(GitError) as raised:
        repo.commit()

    assert raised.value.details == (
        f"Could not create a tree for the git repository in {str(new_dir)!r}."
    )


def test_commit_error(new_dir, mocker):
    """Raise an error if the commit cannot be created."""
    mocker.patch("pygit2.Repository.create_commit", side_effect=pygit2.GitError)
    repo = GitRepo(new_dir)
    (repo.path / "test-file").touch()
    repo.add_all()

    with pytest.raises(GitError) as raised:
        repo.commit()

    assert raised.value.details == (
        f"Could not create a commit for the git repository in {str(new_dir)!r}."
    )


def test_is_clean(new_dir):
    """Check if a repo is clean."""
    repo = GitRepo(new_dir)

    assert repo.is_clean()

    (repo.path / "foo").touch()

    assert not repo.is_clean()


def test_is_clean_error(new_dir, mocker):
    """Check if git fails when checking if the repo is clean."""
    mocker.patch("pygit2.Repository.status", side_effect=pygit2.GitError)
    repo = GitRepo(new_dir)

    with pytest.raises(GitError) as raised:
        repo.is_clean()

    assert raised.value.details == (
        f"Could not check if the git repository in {str(new_dir)!r} is clean."
    )


def test_push_url(new_dir):
    """Push the default ref (HEAD) to a remote branch."""
    # create a local repo and make a commit
    Path("local-repo").mkdir()
    repo = GitRepo(Path("local-repo"))
    (repo.path / "test-file").touch()
    repo.add_all()
    repo.commit()
    # create a bare remote repo
    Path("remote-repo").mkdir()
    remote = pygit2.init_repository(Path("remote-repo"), True)

    repo.push_url(
        remote_url=f"file://{str(Path('remote-repo').absolute())}",
        remote_branch="test-branch",
    )

    # verify commit in remote (the `isinstance` checks are to satsify pyright)
    commit = remote.revparse_single("test-branch")
    assert isinstance(commit, pygit2.Commit)
    assert commit.message == "auto commit"
    assert commit.committer.name == "auto commit"
    assert commit.committer.email == "auto commit"
    # verify tree in remote
    tree = commit.tree
    assert isinstance(tree, pygit2.Tree)
    assert len(tree) == 1
    # verify contents of tree in remote
    blob = tree[0]
    assert isinstance(blob, pygit2.Blob)
    assert blob.name == "test-file"


def test_push_url_detached_head(new_dir):
    """Push a detached HEAD to a remote branch."""
    # create a local repo and make two commits
    Path("local-repo").mkdir()
    repo = GitRepo(Path("local-repo"))
    (repo.path / "test-file-1").touch()
    repo.add_all()
    repo.commit()
    (repo.path / "test-file-2").touch()
    repo.add_all()
    repo.commit()
    # detach HEAD to first commit
    first_commit = repo._repo.revparse_single("HEAD~1")
    repo._repo.checkout_tree(first_commit)
    repo._repo.set_head(first_commit.id)
    # create a bare remote repo
    Path("remote-repo").mkdir()
    remote = pygit2.init_repository(Path("remote-repo"), True)

    # push the detached HEAD to the remote
    repo.push_url(
        remote_url=f"file://{str(Path('remote-repo').absolute())}",
        remote_branch="test-branch",
    )

    # verify commit in remote (the `isinstance` checks are to satsify pyright)
    commit = remote.revparse_single("test-branch")
    assert isinstance(commit, pygit2.Commit)
    assert commit.message == "auto commit"
    assert commit.committer.name == "auto commit"
    assert commit.committer.email == "auto commit"
    # verify tree in remote
    tree = commit.tree
    assert isinstance(tree, pygit2.Tree)
    assert len(tree) == 1
    # verify contents of tree in remote are from the first commit
    blob = tree[0]
    assert isinstance(blob, pygit2.Blob)
    assert blob.name == "test-file-1"


def test_push_url_branch(new_dir):
    """Push a branch to a remote branch."""
    # create a local repo and make a commit
    Path("local-repo").mkdir()
    repo = GitRepo(Path("local-repo"))
    (repo.path / "test-file").touch()
    repo.add_all()
    repo.commit()
    # create a bare remote repo
    Path("remote-repo").mkdir()
    remote = pygit2.init_repository(Path("remote-repo"), True)

    repo.push_url(
        remote_url=f"file://{str(Path('remote-repo').absolute())}",
        remote_branch="test-branch",
        # use the branch name
        ref=repo._repo.head.shorthand,
    )

    # verify commit in remote (the `isinstance` checks are to satsify pyright)
    commit = remote.revparse_single("test-branch")
    assert isinstance(commit, pygit2.Commit)
    assert commit.message == "auto commit"
    assert commit.committer.name == "auto commit"
    assert commit.committer.email == "auto commit"
    # verify tree in remote
    tree = commit.tree
    assert isinstance(tree, pygit2.Tree)
    assert len(tree) == 1
    # verify contents of tree in remote
    blob = tree[0]
    assert isinstance(blob, pygit2.Blob)
    assert blob.name == "test-file"


def test_push_tags(new_dir):
    """Verify that tags are push by trying to ref them from the remote."""
    # create a local repo and make a commit
    Path("local-repo").mkdir()
    repo = GitRepo(Path("local-repo"))
    (repo.path / "test-file").touch()
    repo.add_all()
    commit = repo.commit()
    tag = "tag1"
    repo._repo.create_reference(f"refs/tags/{tag}", commit)
    # create a bare remote repo
    Path("remote-repo").mkdir()
    remote = pygit2.init_repository(Path("remote-repo"), True)

    repo.push_url(
        remote_url=f"file://{str(Path('remote-repo').absolute())}",
        remote_branch="test-branch",
        push_tags=True,
    )

    # verify commit through tag in remote (the `isinstance` checks are to satsify pyright)
    commit = remote.revparse_single(tag)
    assert isinstance(commit, pygit2.Commit)
    assert commit.message == "auto commit"
    assert commit.committer.name == "auto commit"
    assert commit.committer.email == "auto commit"
    # verify tree in remote
    tree = commit.tree
    assert isinstance(tree, pygit2.Tree)
    assert len(tree) == 1
    # verify contents of tree in remote
    blob = tree[0]
    assert isinstance(blob, pygit2.Blob)
    assert blob.name == "test-file"


def test_push_url_refspec_unknown_ref(new_dir):
    """Raise an error for an unknown refspec."""
    repo = GitRepo(new_dir)

    with pytest.raises(GitError) as raised:
        repo.push_url(remote_url="test-url", remote_branch="test-branch", ref="bad-ref")

    assert raised.value.details == (
        "Could not resolve reference 'bad-ref' for the git repository "
        f"in {str(new_dir)!r}."
    )


@pytest.mark.parametrize(
    ("url", "expected_url"),
    [
        # no-op if token is not in url
        ("fake-url", "fake-url"),
        # hide single occurrence of the token
        ("fake-url/test-token", "fake-url/<token>"),
        # hide multiple occurrences of the token
        ("fake-url/test-token/test-token", "fake-url/<token>/<token>"),
    ],
)
def test_push_url_hide_token(url, expected_url, mocker, new_dir):
    """Hide the token in the log and error output."""
    mock_logs = mocker.patch("logging.Logger.debug")

    repo = GitRepo(new_dir)
    (repo.path / "test-file").touch()
    repo.add_all()
    repo.commit()
    expected_error_details = (
        f"Could not push 'HEAD' to {expected_url!r} with refspec "
        "'.*:refs/heads/test-branch' for the git repository "
        f"in {str(new_dir)!r}."
    )

    with pytest.raises(GitError) as raised:
        repo.push_url(
            remote_url=url,
            remote_branch="test-branch",
            token="test-token",
        )

    # token should be hidden in the log output
    mock_logs.assert_called_with(
        # The last argument is the refspec `.*:refs/heads/test-branch`, which can only
        # be asserted with regex. It is not relevant to this test, so `ANY` is used.
        "Pushing %r to remote %r with refspec %r.",
        "HEAD",
        expected_url,
        ANY,
    )

    # token should be hidden in the error message
    assert raised.value.details is not None
    assert re.match(expected_error_details, raised.value.details)


def test_push_url_refspec_git_error(mocker, new_dir):
    """Raise an error if git fails when looking for a refspec."""
    mocker.patch(
        "pygit2.Repository.lookup_reference_dwim",
        side_effect=pygit2.GitError,
    )
    repo = GitRepo(new_dir)

    with pytest.raises(GitError) as raised:
        repo.push_url(remote_url="test-url", remote_branch="test-branch", ref="bad-ref")

    assert raised.value.details == (
        "Could not resolve reference 'bad-ref' for the git repository "
        f"in {str(new_dir)!r}."
    )


def test_push_url_push_error(new_dir):
    """Raise an error when the refspec cannot be pushed."""
    repo = GitRepo(new_dir)
    (repo.path / "test-file").touch()
    repo.add_all()
    repo.commit()
    expected_error_details = (
        "Could not push 'HEAD' to 'bad-url' with refspec "
        "'.*:refs/heads/test-branch' for the git repository "
        f"in {str(new_dir)!r}."
    )

    with pytest.raises(GitError) as raised:
        repo.push_url(remote_url="bad-url", remote_branch="test-branch")

    assert raised.value.details is not None
    assert re.match(expected_error_details, raised.value.details)
