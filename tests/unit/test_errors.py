# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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
import requests.exceptions
from requests.packages import urllib3
from subprocess import CalledProcessError

from unittest import mock
from testtools.matchers import Equals

from snapcraft.internal import errors, pluginhandler, steps
from snapcraft.internal.meta import _errors as meta_errors
from snapcraft.internal.repo import errors as repo_errors
from snapcraft.storeapi import errors as store_errors
from snapcraft.internal.project_loader.inspection import errors as inspection_errors
from tests import unit


def _fake_error_response(status_code):
    response = mock.Mock()
    response.status_code = status_code
    return response


class ErrorFormattingTestCase(unit.TestCase):

    scenarios = (
        (
            "MissingStateCleanError",
            {
                "exception": errors.MissingStateCleanError,
                "kwargs": {"step": steps.PULL},
                "expected_message": (
                    "Failed to clean: "
                    "Missing state for 'pull'. "
                    "To clean the project, run `snapcraft clean`."
                ),
            },
        ),
        (
            "StepOutdatedError dependents",
            {
                "exception": errors.StepOutdatedError,
                "kwargs": {
                    "step": steps.PULL,
                    "part": "test-part",
                    "dependents": ["test-dependent"],
                },
                "expected_message": (
                    "Failed to reuse files from previous run: "
                    "The 'pull' step of 'test-part' is out of date:\n"
                    "The 'pull' step for 'test-part' needs to be run again, "
                    "but 'test-dependent' depends on it.\n"
                    "To continue, clean that part's 'pull' step by running "
                    "`snapcraft clean test-dependent -s pull`."
                ),
            },
        ),
        (
            "StepOutdatedError dirty_properties",
            {
                "exception": errors.StepOutdatedError,
                "kwargs": {
                    "step": steps.PULL,
                    "part": "test-part",
                    "dirty_report": pluginhandler.DirtyReport(
                        dirty_properties=["test-property1", "test-property2"]
                    ),
                },
                "expected_message": (
                    "Failed to reuse files from previous run: "
                    "The 'pull' step of 'test-part' is out of date:\n"
                    "The 'test-property1' and 'test-property2' part properties "
                    "appear to have changed.\n"
                    "To continue, clean that part's 'pull' step by running "
                    "`snapcraft clean test-part -s pull`."
                ),
            },
        ),
        (
            "StepOutdatedError dirty_project_options",
            {
                "exception": errors.StepOutdatedError,
                "kwargs": {
                    "step": steps.PULL,
                    "part": "test-part",
                    "dirty_report": pluginhandler.DirtyReport(
                        dirty_project_options=["test-option"]
                    ),
                },
                "expected_message": (
                    "Failed to reuse files from previous run: "
                    "The 'pull' step of 'test-part' is out of date:\n"
                    "The 'test-option' project option appears to have changed.\n"
                    "To continue, clean that part's 'pull' step by running "
                    "`snapcraft clean test-part -s pull`."
                ),
            },
        ),
        (
            "StepOutdatedError changed_dependencies",
            {
                "exception": errors.StepOutdatedError,
                "kwargs": {
                    "step": steps.PULL,
                    "part": "test-part",
                    "dirty_report": pluginhandler.DirtyReport(
                        changed_dependencies=[
                            pluginhandler.Dependency(
                                part_name="another-part", step=steps.PULL
                            )
                        ]
                    ),
                },
                "expected_message": (
                    "Failed to reuse files from previous run: "
                    "The 'pull' step of 'test-part' is out of date:\n"
                    "A dependency has changed: 'another-part'\n"
                    "To continue, clean that part's "
                    "'pull' step by running "
                    "`snapcraft clean test-part -s pull`."
                ),
            },
        ),
        (
            "StepOutdatedError multiple changed_dependencies",
            {
                "exception": errors.StepOutdatedError,
                "kwargs": {
                    "step": steps.PULL,
                    "part": "test-part",
                    "dirty_report": pluginhandler.DirtyReport(
                        changed_dependencies=[
                            pluginhandler.Dependency(
                                part_name="another-part1", step=steps.PULL
                            ),
                            pluginhandler.Dependency(
                                part_name="another-part2", step=steps.PULL
                            ),
                        ]
                    ),
                },
                "expected_message": (
                    "Failed to reuse files from previous run: "
                    "The 'pull' step of 'test-part' is out of date:\n"
                    "Some dependencies have changed: 'another-part1' and "
                    "'another-part2'\n"
                    "To continue, clean that part's "
                    "'pull' step by running "
                    "`snapcraft clean test-part -s pull`."
                ),
            },
        ),
        (
            "StepOutdatedError previous step updated",
            {
                "exception": errors.StepOutdatedError,
                "kwargs": {
                    "step": steps.STAGE,
                    "part": "test-part",
                    "outdated_report": pluginhandler.OutdatedReport(
                        previous_step_modified=steps.BUILD
                    ),
                },
                "expected_message": (
                    "Failed to reuse files from previous run: "
                    "The 'stage' step of 'test-part' is out of date:\n"
                    "The 'build' step has run more recently.\n"
                    "To continue, clean that part's "
                    "'stage' step by running "
                    "`snapcraft clean test-part -s stage`."
                ),
            },
        ),
        (
            "StepOutdatedError source updated",
            {
                "exception": errors.StepOutdatedError,
                "kwargs": {
                    "step": steps.PULL,
                    "part": "test-part",
                    "outdated_report": pluginhandler.OutdatedReport(
                        source_updated=True
                    ),
                },
                "expected_message": (
                    "Failed to reuse files from previous run: "
                    "The 'pull' step of 'test-part' is out of date:\n"
                    "The source has changed on disk.\n"
                    "To continue, clean that part's "
                    "'pull' step by running "
                    "`snapcraft clean test-part -s pull`."
                ),
            },
        ),
        (
            "SnapcraftEnvironmentError",
            {
                "exception": errors.SnapcraftEnvironmentError,
                "kwargs": {"message": "test-message"},
                "expected_message": "test-message",
            },
        ),
        (
            "SnapcraftMissingLinkerInBaseError",
            {
                "exception": errors.SnapcraftMissingLinkerInBaseError,
                "kwargs": {
                    "base": "core18",
                    "linker_path": "/snap/core18/current/lib64/ld-linux.so.2",
                },
                "expected_message": (
                    "Cannot find the linker to use for the target base 'core18'.\n"
                    "Please verify that the linker exists at the expected path "
                    "'/snap/core18/current/lib64/ld-linux.so.2' and try again. If "
                    "the linker does not exist contact the author of the base "
                    "(run `snap info core18` to get information for this "
                    "base)."
                ),
            },
        ),
        (
            "IncompatibleBaseError",
            {
                "exception": errors.IncompatibleBaseError,
                "kwargs": {
                    "base": "core18",
                    "linker_version": "2.23",
                    "file_list": dict(a="2.26", b="2.27"),
                },
                "expected_message": (
                    "The linker version '2.23' used by the base 'core18' is "
                    "incompatible with files in this snap:\n"
                    "    a (2.26)\n"
                    "    b (2.27)"
                ),
            },
        ),
        (
            "PrimeFileConflictError",
            {
                "exception": errors.PrimeFileConflictError,
                "kwargs": {"fileset": {"test-file"}},
                "expected_message": (
                    "Failed to filter files: "
                    "The following files have been excluded by the `stage` "
                    "keyword, but included by the `prime` keyword: "
                    "{'test-file'}. "
                    "Edit the `snapcraft.yaml` to make sure that the files "
                    "included in `prime` are also included in `stage`."
                ),
            },
        ),
        (
            "InvalidAppCommandError",
            {
                "exception": errors.InvalidAppCommandError,
                "kwargs": {"command": "test-command", "app": "test-app"},
                "expected_message": (
                    "Failed to generate snap metadata: "
                    "The specified command 'test-command' defined in the app "
                    "'test-app' does not exist or is not executable"
                ),
            },
        ),
        (
            "InvalidContainerRemoteError",
            {
                "exception": errors.InvalidContainerRemoteError,
                "kwargs": {"remote": "test-remote"},
                "expected_message": (
                    "Failed to use LXD remote: "
                    "'test-remote' is not a valid name.\n"
                    "Use a LXD remote without colons, spaces and slashes in the "
                    "name.\n"
                ),
            },
        ),
        (
            "InvalidDesktopFileError",
            {
                "exception": errors.InvalidDesktopFileError,
                "kwargs": {"filename": "test-file", "message": "test-message"},
                "expected_message": (
                    "Failed to generate desktop file: "
                    "Invalid desktop file 'test-file': test-message."
                ),
            },
        ),
        (
            "SnapcraftPartMissingError",
            {
                "exception": errors.SnapcraftPartMissingError,
                "kwargs": {"part_name": "test-part"},
                "expected_message": (
                    "Failed to get part information: "
                    "Cannot find the definition for part 'test-part'. "
                    "If it is a remote part, run `snapcraft update` "
                    "to refresh the remote parts cache. "
                    "If it is a local part, make sure that it is defined in the "
                    "`snapcraft.yaml`."
                ),
            },
        ),
        (
            "PartNotInCacheError",
            {
                "exception": errors.PartNotInCacheError,
                "kwargs": {"part_name": "test-part"},
                "expected_message": (
                    "Failed to get remote part information: "
                    "Cannot find the part name 'test-part' in the cache. "
                    "If it is an existing remote part, run `snapcraft update` "
                    "and try again. If it has not been defined, consider going to "
                    "https://wiki.ubuntu.com/snapcraft/parts to add it."
                ),
            },
        ),
        (
            "PluginError",
            {
                "exception": errors.PluginError,
                "kwargs": {"message": "test-message"},
                "expected_message": "Failed to load plugin: test-message",
            },
        ),
        (
            "SnapcraftPartConflictError",
            {
                "exception": errors.SnapcraftPartConflictError,
                "kwargs": {
                    "part_name": "test-part",
                    "other_part_name": "test-other-part",
                    "conflict_files": ("test-file1", "test-file2"),
                },
                "expected_message": (
                    "Failed to stage: "
                    "Parts 'test-other-part' and 'test-part' have the following "
                    "files, but with different contents:\n"
                    "    test-file1\n"
                    "    test-file2\n"
                    "\n"
                    "Snapcraft offers some capabilities to solve this by use of "
                    "the following keywords:\n"
                    "    - `filesets`\n"
                    "    - `stage`\n"
                    "    - `snap`\n"
                    "    - `organize`\n"
                    "\n"
                    "To learn more about these part keywords, run "
                    "`snapcraft help plugins`."
                ),
            },
        ),
        (
            "MissingCommandError",
            {
                "exception": errors.MissingCommandError,
                "kwargs": {"required_commands": ["test-command1", "test-command2"]},
                "expected_message": (
                    "Failed to run command: "
                    "One or more packages are missing, please install:"
                    " ['test-command1', 'test-command2']"
                ),
            },
        ),
        (
            "InvalidWikiEntryError",
            {
                "exception": errors.InvalidWikiEntryError,
                "kwargs": {"error": "test-error"},
                "expected_message": "Invalid wiki entry: 'test-error'",
            },
        ),
        (
            "PluginOutdatedError",
            {
                "exception": errors.PluginOutdatedError,
                "kwargs": {"message": "test-message"},
                "expected_message": "This plugin is outdated: test-message",
            },
        ),
        (
            "RequiredCommandFailure",
            {
                "exception": errors.RequiredCommandFailure,
                "kwargs": {"command": "test-command"},
                "expected_message": "'test-command' failed.",
            },
        ),
        (
            "RequiredCommandNotFound",
            {
                "exception": errors.RequiredCommandNotFound,
                "kwargs": {"cmd_list": ["test-command", "test-argument"]},
                "expected_message": "'test-command' not found.",
            },
        ),
        (
            "RequiredPathDoesNotExist",
            {
                "exception": errors.RequiredPathDoesNotExist,
                "kwargs": {"path": "test-path"},
                "expected_message": "Required path does not exist: 'test-path'",
            },
        ),
        (
            "SnapcraftPathEntryError",
            {
                "exception": errors.SnapcraftPathEntryError,
                "kwargs": {"value": "test-path", "key": "test-key", "app": "test-app"},
                "expected_message": (
                    "Failed to generate snap metadata: "
                    "The path 'test-path' set for 'test-key' in 'test-app' does "
                    "not exist. Make sure that the files are in the `prime` "
                    "directory."
                ),
            },
        ),
        (
            "InvalidPullPropertiesError",
            {
                "exception": errors.InvalidPullPropertiesError,
                "kwargs": {
                    "plugin_name": "test-plugin",
                    "properties": ["test-property1", "test-property2"],
                },
                "expected_message": (
                    "Failed to load plugin: "
                    "Invalid pull properties specified by 'test-plugin' plugin: "
                    "['test-property1', 'test-property2']"
                ),
            },
        ),
        (
            "InvalidBuildPropertiesError",
            {
                "exception": errors.InvalidBuildPropertiesError,
                "kwargs": {
                    "plugin_name": "test-plugin",
                    "properties": ["test-property1", "test-property2"],
                },
                "expected_message": (
                    "Failed to load plugin: "
                    "Invalid build properties specified by 'test-plugin' plugin: "
                    "['test-property1', 'test-property2']"
                ),
            },
        ),
        (
            "StagePackageDownloadError",
            {
                "exception": errors.StagePackageDownloadError,
                "kwargs": {"part_name": "test-part", "message": "test-message"},
                "expected_message": (
                    "Failed to fetch stage packages: "
                    "Error downloading packages for part "
                    "'test-part': test-message."
                ),
            },
        ),
        (
            "InvalidContainerImageInfoError",
            {
                "exception": errors.InvalidContainerImageInfoError,
                "kwargs": {"image_info": "test-image-info"},
                "expected_message": (
                    "Failed to parse container image info: "
                    "SNAPCRAFT_IMAGE_INFO is not a valid JSON string: "
                    "test-image-info"
                ),
            },
        ),
        # meta errors.
        (
            "AdoptedPartMissingError",
            {
                "exception": meta_errors.AdoptedPartMissingError,
                "kwargs": {"part": "test-part"},
                "expected_message": (
                    "Failed to generate snap metadata: "
                    "'adopt-info' refers to a part named 'test-part', but it is "
                    "not defined in the 'snapcraft.yaml' file."
                ),
            },
        ),
        (
            "AdoptedPartNotParsingInfo",
            {
                "exception": meta_errors.AdoptedPartNotParsingInfo,
                "kwargs": {"part": "test-part"},
                "expected_message": (
                    "Failed to generate snap metadata: "
                    "'adopt-info' refers to part 'test-part', but that part is "
                    "lacking the 'parse-info' property."
                ),
            },
        ),
        (
            "MissingSnapcraftYamlKeysError",
            {
                "exception": meta_errors.MissingSnapcraftYamlKeysError,
                "kwargs": {"keys": ["test-key1", "test-key2"]},
                "expected_message": (
                    "Failed to generate snap metadata: "
                    "Missing required key(s) in snapcraft.yaml: "
                    "'test-key1' and 'test-key2'. Either specify the missing "
                    "key(s), or use 'adopt-info' to get them from a part."
                ),
            },
        ),
        (
            "AmbiguousPassthroughKeyError",
            {
                "exception": meta_errors.AmbiguousPassthroughKeyError,
                "kwargs": {"keys": ["key1", "key2"]},
                "expected_message": (
                    "Failed to generate snap metadata: "
                    "The following keys are specified in their regular location "
                    "as well as in passthrough: 'key1' and 'key2'. "
                    "Remove duplicate keys."
                ),
            },
        ),
        (
            "MissingMetadataFileError",
            {
                "exception": errors.MissingMetadataFileError,
                "kwargs": {"part_name": "test-part", "path": "test/path"},
                "expected_message": (
                    "Failed to generate snap metadata: "
                    "Part 'test-part' has a 'parse-info' referring to metadata "
                    "file 'test/path', which does not exist."
                ),
            },
        ),
        (
            "UnhandledMetadataFileTypeError",
            {
                "exception": errors.UnhandledMetadataFileTypeError,
                "kwargs": {"path": "test/path"},
                "expected_message": (
                    "Failed to extract metadata from 'test/path': "
                    "This type of file is not supported for supplying "
                    "metadata."
                ),
            },
        ),
        (
            "InvalidExtractorValueError",
            {
                "exception": errors.InvalidExtractorValueError,
                "kwargs": {"path": "test/path", "extractor_name": "extractor"},
                "expected_message": (
                    "Failed to extract metadata from 'test/path': "
                    "Extractor 'extractor' didn't return ExtractedMetadata as "
                    "expected."
                ),
            },
        ),
        (
            "PatcherNewerPatchelfError",
            {
                "exception": errors.PatcherNewerPatchelfError,
                "kwargs": {
                    "elf_file": "test/path",
                    "patchelf_version": "patchelf 0.9",
                    "process_exception": CalledProcessError(
                        cmd=["patchelf"], returncode=-1
                    ),
                },
                "expected_message": (
                    "'test/path' cannot be patched to function properly in a "
                    "classic confined snap: patchelf failed with exit code -1.\n"
                    "'patchelf 0.9' may be too old. A newer version of patchelf "
                    "may be required.\n"
                    "Try adding the `after: [patchelf]` and a `patchelf` part "
                    "that would filter out files from prime `prime: [-*]` or "
                    "`build-snaps: [patchelf/latest/edge]` to the failing part "
                    "in your `snapcraft.yaml` to use a newer patchelf."
                ),
            },
        ),
        (
            "PatcherGenericError",
            {
                "exception": errors.PatcherGenericError,
                "kwargs": {
                    "elf_file": "test/path",
                    "process_exception": CalledProcessError(
                        cmd=["patchelf"], returncode=-1
                    ),
                },
                "expected_message": (
                    "'test/path' cannot be patched to function properly in a "
                    "classic confined snap: patchelf failed with exit code -1"
                ),
            },
        ),
        (
            "StagePackageMissingError",
            {
                "exception": errors.StagePackageMissingError,
                "kwargs": {"package": "libc6"},
                "expected_message": (
                    "'libc6' is required inside the snap for this "
                    "part to work properly.\nAdd it as a `stage-packages` "
                    "entry for this part."
                ),
            },
        ),
        (
            "RemotePartsUpdateConnectionError",
            {
                "exception": errors.RemotePartsUpdateConnectionError,
                "kwargs": {
                    "requests_exception": requests.exceptions.ConnectionError(
                        "I'm a naughty error"
                    )
                },
                "expected_message": (
                    "Failed to update cache of remote parts: A Connection error "
                    "occurred.\nPlease try again."
                ),
            },
        ),
        (
            "SnapcraftCommandError",
            {
                "exception": errors.SnapcraftCommandError,
                "kwargs": {
                    "command": "pip install foo",
                    "call_error": CalledProcessError(
                        cmd=["/bin/sh"], returncode=1, output="failed"
                    ),
                },
                "expected_message": (
                    "Failed to run 'pip install foo': Exited with code 1."
                ),
            },
        ),
        (
            "SnapcraftPluginCommandError string command",
            {
                "exception": errors.SnapcraftPluginCommandError,
                "kwargs": {
                    "command": "make install",
                    "exit_code": -1,
                    "part_name": "make_test",
                },
                "expected_message": (
                    "Failed to run 'make install' for 'make_test': "
                    "Exited with code -1.\n"
                    "Verify that the part is using the correct parameters and try "
                    "again."
                ),
            },
        ),
        (
            "SnapcraftPluginCommandError list command",
            {
                "exception": errors.SnapcraftPluginCommandError,
                "kwargs": {
                    "command": ["make", "install"],
                    "exit_code": 2,
                    "part_name": "make_test",
                },
                "expected_message": (
                    "Failed to run 'make install' for 'make_test': "
                    "Exited with code 2.\n"
                    "Verify that the part is using the correct parameters and try "
                    "again."
                ),
            },
        ),
        (
            "CacheUpdateFailedError",
            {
                "exception": repo_errors.CacheUpdateFailedError,
                "kwargs": {"errors": ""},
                "expected_message": (
                    "Failed to update the package cache: "
                    "Some files could not be downloaded: "
                    "Check that the sources on your host are configured correctly."
                ),
            },
        ),
        (
            "CacheUpdateFailedError",
            {
                "exception": repo_errors.CacheUpdateFailedError,
                "kwargs": {"errors": "foo, bar"},
                "expected_message": (
                    "Failed to update the package cache: "
                    "Some files could not be downloaded:\n\nfoo\nbar\n\n"
                    "Check that the sources on your host are configured correctly."
                ),
            },
        ),
        (
            "StoreNetworkError generic error",
            {
                "exception": store_errors.StoreNetworkError,
                "kwargs": {
                    "exception": requests.exceptions.ConnectionError("bad error")
                },
                "expected_message": ("There seems to be a network error: bad error"),
            },
        ),
        (
            "StoreNetworkError max retry error",
            {
                "exception": store_errors.StoreNetworkError,
                "kwargs": {
                    "exception": requests.exceptions.ConnectionError(
                        urllib3.exceptions.MaxRetryError(
                            pool="test-pool", url="test-url"
                        )
                    )
                },
                "expected_message": (
                    "There seems to be a network error: maximum retries exceeded "
                    "trying to reach the store.\n"
                    "Check your network connection, and check the store status at "
                    "https://status.snapcraft.io/"
                ),
            },
        ),
        (
            "SnapcraftCopyFileNotFoundError",
            {
                "exception": errors.SnapcraftCopyFileNotFoundError,
                "kwargs": {"path": "test-path"},
                "expected_message": (
                    "Failed to copy 'test-path': no such file or directory.\n"
                    "Check the path and try again."
                ),
            },
        ),
        (
            "StoreServerError 500",
            {
                "exception": store_errors.StoreServerError,
                "kwargs": {"response": _fake_error_response(500)},
                "expected_message": (
                    "The Snap Store encountered an error while processing your "
                    "request: internal server error (code 500).\nThe operational "
                    "status of the Snap Store can be checked at "
                    "https://status.snapcraft.io/"
                ),
            },
        ),
        (
            "StoreServerError 501",
            {
                "exception": store_errors.StoreServerError,
                "kwargs": {"response": _fake_error_response(501)},
                "expected_message": (
                    "The Snap Store encountered an error while processing your "
                    "request: not implemented (code 501).\nThe operational "
                    "status of the Snap Store can be checked at "
                    "https://status.snapcraft.io/"
                ),
            },
        ),
        (
            "MountPointNotFoundError",
            {
                "exception": errors.MountPointNotFoundError,
                "kwargs": {"mount_point": "test-mount-point"},
                "expected_message": ("Nothing is mounted at 'test-mount-point'"),
            },
        ),
        (
            "RootNotMountedError",
            {
                "exception": errors.RootNotMountedError,
                "kwargs": {"root": "test-root"},
                "expected_message": ("'test-root' is not mounted"),
            },
        ),
        (
            "InvalidMountinfoFormat",
            {
                "exception": errors.InvalidMountinfoFormat,
                "kwargs": {"row": [1, 2, 3]},
                "expected_message": ("Unable to parse mountinfo row: [1, 2, 3]"),
            },
        ),
        (
            "InvalidStepError",
            {
                "exception": errors.InvalidStepError,
                "kwargs": {"step_name": "test-step-name"},
                "expected_message": ("'test-step-name' is not a valid lifecycle step"),
            },
        ),
        (
            "NoLatestStepError",
            {
                "exception": errors.NoLatestStepError,
                "kwargs": {"part_name": "test-part-name"},
                "expected_message": ("The 'test-part-name' part hasn't run any steps"),
            },
        ),
        (
            "NoNextStepError",
            {
                "exception": errors.NoNextStepError,
                "kwargs": {"part_name": "test-part-name"},
                "expected_message": (
                    "The 'test-part-name' part has run through its entire lifecycle"
                ),
            },
        ),
        (
            "StepHasNotRunError",
            {
                "exception": errors.StepHasNotRunError,
                "kwargs": {"part_name": "test-part-name", "step": steps.BUILD},
                "expected_message": (
                    "The 'test-part-name' part has not yet run the 'build' step"
                ),
            },
        ),
        (
            "ScriptletDuplicateFieldError",
            {
                "exception": errors.ScriptletDuplicateFieldError,
                "kwargs": {"field": "foo", "step": steps.PULL},
                "expected_message": (
                    "Unable to set foo: it was already set in the 'pull' step."
                ),
            },
        ),
        (
            "ToolMissingError",
            {
                "exception": errors.ToolMissingError,
                "kwargs": {"command_name": "runnable"},
                "expected_message": (
                    "A tool snapcraft depends on could not be found: 'runnable'.\n"
                    "Ensure the tool is installed and available, and try again."
                ),
            },
        ),
        (
            "NoSuchFileError",
            {
                "exception": inspection_errors.NoSuchFileError,
                "kwargs": {"path": "test-path"},
                "expected_message": (
                    "Failed to find part that provided path: 'test-path' does not "
                    "exist.\n"
                    "Check the file path and try again."
                ),
            },
        ),
        (
            "ProvidesInvalidFilePathError",
            {
                "exception": inspection_errors.ProvidesInvalidFilePathError,
                "kwargs": {"path": "test-path"},
                "expected_message": (
                    "Failed to find part that provides path: 'test-path' is not "
                    "in the staging or priming area.\n"
                    "Ensure the path is in the staging or priming area and try "
                    "again."
                ),
            },
        ),
        (
            "UntrackedFileError",
            {
                "exception": inspection_errors.UntrackedFileError,
                "kwargs": {"path": "test-path"},
                "expected_message": (
                    "No known parts provided 'test-path'. It may have been "
                    "provided by a scriptlet."
                ),
            },
        ),
        (
            "NoStepsRunError",
            {
                "exception": inspection_errors.NoStepsRunError,
                "kwargs": {},
                "expected_message": "Failed to get latest step: no steps have run",
            },
        ),
    )

    def test_error_formatting(self):
        self.assertThat(
            str(self.exception(**self.kwargs)), Equals(self.expected_message)
        )
