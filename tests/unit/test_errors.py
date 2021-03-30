# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2020 Canonical Ltd
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
from subprocess import CalledProcessError
from typing import List

from snapcraft.internal import errors, pluginhandler, steps
from snapcraft.internal.project_loader import errors as project_loader_errors
from snapcraft.internal.project_loader.inspection import errors as inspection_errors
from snapcraft.internal.repo import errors as repo_errors


def test_details_from_called_process_error():
    error = CalledProcessError(
        -1, ["test-command", "flags", "quote$me"], "test stdout", "test stderr"
    )

    details = errors.details_from_called_process_error(error)

    assert details == textwrap.dedent(
        """\
            * Command that failed: "test-command flags 'quote$me'"
            * Command exit code: -1
            * Command output: 'test stdout'
            * Command standard error output: 'test stderr'"""
    )


def test_details_from_command_error():
    details = errors.details_from_command_error(
        returncode=-1, cmd=["test-command", "flags", "quote$me"],
    )

    assert details == textwrap.dedent(
        """\
            * Command that failed: "test-command flags 'quote$me'"
            * Command exit code: -1"""
    )


def test_details_from_command_error_with_output_strings():
    details = errors.details_from_command_error(
        returncode=-1,
        cmd=["test-command", "flags", "quote$me"],
        stdout="test stdout",
        stderr="test stderr",
    )

    assert details == textwrap.dedent(
        """\
            * Command that failed: "test-command flags 'quote$me'"
            * Command exit code: -1
            * Command output: 'test stdout'
            * Command standard error output: 'test stderr'"""
    )


def test_details_from_command_error_with_output_bytes():
    details = errors.details_from_command_error(
        returncode=-1,
        cmd=["test-command", "flags", "quote$me"],
        stdout=bytes.fromhex("00 FF"),
        stderr=bytes.fromhex("01 FE"),
    )

    assert details == textwrap.dedent(
        """\
            * Command that failed: "test-command flags 'quote$me'"
            * Command exit code: -1
            * Command output: b'\\x00\\xff'
            * Command standard error output: b'\\x01\\xfe'"""
    )


class TestErrorFormatting:
    scenarios = (
        (
            "IncompatibleBaseError",
            {
                "exception_class": errors.IncompatibleBaseError,
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
                "exception_class": errors.PrimeFileConflictError,
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
            "SnapcraftAfterPartMissingError",
            {
                "exception_class": project_loader_errors.SnapcraftAfterPartMissingError,
                "kwargs": {"part_name": "test-part1", "after_part_name": "test-part2"},
                "expected_message": (
                    "Failed to get part information: "
                    "Cannot find the definition for part 'test-part2', required by "
                    "part 'test-part1'.\n"
                    "Remote parts are not supported with bases, so make sure that this "
                    "part is defined in the `snapcraft.yaml`."
                ),
            },
        ),
        (
            "PluginError",
            {
                "exception_class": errors.PluginError,
                "kwargs": {"message": "test-message"},
                "expected_message": "Failed to load plugin: test-message",
            },
        ),
        (
            "PluginBaseError",
            {
                "exception_class": errors.PluginBaseError,
                "kwargs": {"part_name": "go-part", "base": "arch"},
                "expected_message": "The plugin used by part 'go-part' does not support snaps using base 'arch'.",
            },
        ),
        (
            "SnapcraftPartConflictError",
            {
                "exception_class": errors.SnapcraftPartConflictError,
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
            "InvalidWikiEntryError",
            {
                "exception_class": errors.InvalidWikiEntryError,
                "kwargs": {"error": "test-error"},
                "expected_message": "Invalid wiki entry: 'test-error'",
            },
        ),
        (
            "PluginOutdatedError",
            {
                "exception_class": errors.PluginOutdatedError,
                "kwargs": {"message": "test-message"},
                "expected_message": "This plugin is outdated: test-message",
            },
        ),
        (
            "RequiredCommandFailure",
            {
                "exception_class": errors.RequiredCommandFailure,
                "kwargs": {"command": "test-command"},
                "expected_message": "'test-command' failed.",
            },
        ),
        (
            "RequiredCommandNotFound",
            {
                "exception_class": errors.RequiredCommandNotFound,
                "kwargs": {"cmd_list": ["test-command", "test-argument"]},
                "expected_message": "'test-command' not found.",
            },
        ),
        (
            "RequiredPathDoesNotExist",
            {
                "exception_class": errors.RequiredPathDoesNotExist,
                "kwargs": {"path": "test-path"},
                "expected_message": "Required path does not exist: 'test-path'",
            },
        ),
        (
            "SnapcraftPathEntryError",
            {
                "exception_class": errors.SnapcraftPathEntryError,
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
                "exception_class": errors.InvalidPullPropertiesError,
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
                "exception_class": errors.InvalidBuildPropertiesError,
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
                "exception_class": errors.StagePackageDownloadError,
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
                "exception_class": errors.InvalidContainerImageInfoError,
                "kwargs": {"image_info": "test-image-info"},
                "expected_message": (
                    "Failed to parse container image info: "
                    "SNAPCRAFT_IMAGE_INFO is not a valid JSON string: "
                    "test-image-info"
                ),
            },
        ),
        (
            "MissingMetadataFileError",
            {
                "exception_class": errors.MissingMetadataFileError,
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
                "exception_class": errors.UnhandledMetadataFileTypeError,
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
                "exception_class": errors.InvalidExtractorValueError,
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
                "exception_class": errors.PatcherNewerPatchelfError,
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
                "exception_class": errors.PatcherGenericError,
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
                "exception_class": errors.StagePackageMissingError,
                "kwargs": {"package": "libc6"},
                "expected_message": (
                    "'libc6' is required inside the snap for this "
                    "part to work properly.\nAdd it as a `stage-packages` "
                    "entry for this part."
                ),
            },
        ),
        (
            "SnapcraftCommandError",
            {
                "exception_class": errors.SnapcraftCommandError,
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
                "exception_class": errors.SnapcraftPluginCommandError,
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
                "exception_class": errors.SnapcraftPluginCommandError,
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
            "CrossCompilationNotSupported",
            {
                "exception_class": errors.CrossCompilationNotSupported,
                "kwargs": {"part_name": "my-part"},
                "expected_message": (
                    "The plugin used by 'my-part' does not support "
                    "cross-compiling to a different target architecture."
                ),
            },
        ),
        (
            "CacheUpdateFailedError",
            {
                "exception_class": repo_errors.CacheUpdateFailedError,
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
                "exception_class": repo_errors.CacheUpdateFailedError,
                "kwargs": {"errors": "foo, bar"},
                "expected_message": (
                    "Failed to update the package cache: "
                    "Some files could not be downloaded:\n\nfoo\nbar\n\n"
                    "Check that the sources on your host are configured correctly."
                ),
            },
        ),
        (
            "SnapcraftCopyFileNotFoundError",
            {
                "exception_class": errors.SnapcraftCopyFileNotFoundError,
                "kwargs": {"path": "test-path"},
                "expected_message": (
                    "Failed to copy 'test-path': no such file or directory.\n"
                    "Check the path and try again."
                ),
            },
        ),
        (
            "MountPointNotFoundError",
            {
                "exception_class": errors.MountPointNotFoundError,
                "kwargs": {"mount_point": "test-mount-point"},
                "expected_message": "Nothing is mounted at 'test-mount-point'",
            },
        ),
        (
            "RootNotMountedError",
            {
                "exception_class": errors.RootNotMountedError,
                "kwargs": {"root": "test-root"},
                "expected_message": "'test-root' is not mounted",
            },
        ),
        (
            "InvalidMountinfoFormat",
            {
                "exception_class": errors.InvalidMountinfoFormat,
                "kwargs": {"row": [1, 2, 3]},
                "expected_message": "Unable to parse mountinfo row: [1, 2, 3]",
            },
        ),
        (
            "InvalidStepError",
            {
                "exception_class": errors.InvalidStepError,
                "kwargs": {"step_name": "test-step-name"},
                "expected_message": "'test-step-name' is not a valid lifecycle step",
            },
        ),
        (
            "NoLatestStepError",
            {
                "exception_class": errors.NoLatestStepError,
                "kwargs": {"part_name": "test-part-name"},
                "expected_message": "The 'test-part-name' part hasn't run any steps",
            },
        ),
        (
            "NoNextStepError",
            {
                "exception_class": errors.NoNextStepError,
                "kwargs": {"part_name": "test-part-name"},
                "expected_message": (
                    "The 'test-part-name' part has run through its entire lifecycle"
                ),
            },
        ),
        (
            "StepHasNotRunError",
            {
                "exception_class": errors.StepHasNotRunError,
                "kwargs": {"part_name": "test-part-name", "step": steps.BUILD},
                "expected_message": (
                    "The 'test-part-name' part has not yet run the 'build' step"
                ),
            },
        ),
        (
            "ScriptletDuplicateFieldError",
            {
                "exception_class": errors.ScriptletDuplicateFieldError,
                "kwargs": {"field": "foo", "step": steps.PULL},
                "expected_message": (
                    "Unable to set foo: it was already set in the 'pull' step."
                ),
            },
        ),
        (
            "ToolMissingError",
            {
                "exception_class": errors.ToolMissingError,
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
                "exception_class": inspection_errors.NoSuchFileError,
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
                "exception_class": inspection_errors.ProvidesInvalidFilePathError,
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
                "exception_class": inspection_errors.UntrackedFileError,
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
                "exception_class": inspection_errors.NoStepsRunError,
                "kwargs": {},
                "expected_message": "Failed to get latest step: no steps have run",
            },
        ),
    )

    def test_error_formatting(self, exception_class, expected_message, kwargs):
        assert str(exception_class(**kwargs)) == expected_message


class StrangeExceptionSimple(errors.SnapcraftException):
    def get_brief(self):
        return "something's strange, in the neighborhood"

    def get_resolution(self):
        return "who you gonna call? ghostbusters!!"

    def get_details(self):
        return "i ain't afraid of no ghosts"

    def get_docs_url(self):
        return "https://docs.snapcraft.io/the-snapcraft-format/8337"


class StrangeExceptionWithFormatting(errors.SnapcraftException):
    def __init__(self, neighborhood: str, contact: str, ghosts: List[str]) -> None:
        self._neighborhood = neighborhood
        self._contact = contact
        self._ghosts = ghosts

    def get_brief(self):
        return "something's strange, in the neighborhood of {}".format(
            self._neighborhood
        )

    def get_resolution(self):
        return "who you gonna call? {}!!".format(self._contact)

    def get_details(self):
        return "i ain't afraid of no ghosts: {}".format(self._ghosts)

    def get_docs_url(self):
        return "https://docs.snapcraft.io/the-snapcraft-format/8337"


class TestSnapcraftExceptionTests:

    scenarios = (
        (
            "StrangeExceptionSimple",
            {
                "exception_class": StrangeExceptionSimple,
                "kwargs": {},
                "expected_brief": "something's strange, in the neighborhood",
                "expected_resolution": "who you gonna call? ghostbusters!!",
                "expected_details": "i ain't afraid of no ghosts",
                "expected_docs_url": "https://docs.snapcraft.io/the-snapcraft-format/8337",
                "expected_reportable": False,
            },
        ),
        (
            "StrangeExceptionWithFormatting",
            {
                "exception_class": StrangeExceptionWithFormatting,
                "kwargs": {
                    "neighborhood": "Times Square",
                    "ghosts": ["slimer", "puft", "vigo"],
                    "contact": "Janine Melnitz",
                },
                "expected_brief": "something's strange, in the neighborhood of Times Square",
                "expected_resolution": "who you gonna call? Janine Melnitz!!",
                "expected_details": "i ain't afraid of no ghosts: ['slimer', 'puft', 'vigo']",
                "expected_docs_url": "https://docs.snapcraft.io/the-snapcraft-format/8337",
                "expected_reportable": False,
            },
        ),
        (
            "SnapcraftHostToolNotFoundError",
            {
                "exception_class": errors.SnapcraftHostToolNotFoundError,
                "kwargs": {"command_name": "foo", "package_name": "foo-pkg"},
                "expected_brief": "A tool snapcraft depends on could not be found: 'foo'",
                "expected_resolution": "Ensure that 'foo-pkg' is installed.",
                "expected_details": None,
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
        (
            "MissingStateCleanError",
            {
                "exception_class": errors.MissingStateCleanError,
                "kwargs": {"step": steps.PULL},
                "expected_brief": "Failed to clean for step 'pull'.",
                "expected_resolution": "Run `snapcraft clean` and retry build.",
                "expected_details": None,
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
        (
            "StepOutdatedError dirty_properties",
            {
                "exception_class": errors.StepOutdatedError,
                "kwargs": {
                    "step": steps.PULL,
                    "part": "test-part",
                    "dirty_report": pluginhandler.DirtyReport(
                        dirty_properties=["test-property1", "test-property2"]
                    ),
                },
                "expected_brief": "Failed to reuse files from previous run.",
                "expected_resolution": "Run `snapcraft clean` and retry build.",
                "expected_details": "The 'test-property1' and 'test-property2' part properties appear to have changed.\n",
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
        (
            "StepOutdatedError dirty_project_options",
            {
                "exception_class": errors.StepOutdatedError,
                "kwargs": {
                    "step": steps.PULL,
                    "part": "test-part",
                    "dirty_report": pluginhandler.DirtyReport(
                        dirty_project_options=["test-option"]
                    ),
                },
                "expected_brief": "Failed to reuse files from previous run.",
                "expected_resolution": "Run `snapcraft clean` and retry build.",
                "expected_details": "The 'test-option' project option appears to have changed.\n",
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
        (
            "StepOutdatedError changed_dependencies",
            {
                "exception_class": errors.StepOutdatedError,
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
                "expected_brief": "Failed to reuse files from previous run.",
                "expected_resolution": "Run `snapcraft clean` and retry build.",
                "expected_details": "A dependency has changed: 'another-part'\n",
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
        (
            "StepOutdatedError multiple changed_dependencies",
            {
                "exception_class": errors.StepOutdatedError,
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
                "expected_brief": "Failed to reuse files from previous run.",
                "expected_resolution": "Run `snapcraft clean` and retry build.",
                "expected_details": "Some dependencies have changed: 'another-part1' and 'another-part2'\n",
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
        (
            "StepOutdatedError previous step updated",
            {
                "exception_class": errors.StepOutdatedError,
                "kwargs": {
                    "step": steps.STAGE,
                    "part": "test-part",
                    "outdated_report": pluginhandler.OutdatedReport(
                        previous_step_modified=steps.BUILD
                    ),
                },
                "expected_brief": "Failed to reuse files from previous run.",
                "expected_resolution": "Run `snapcraft clean` and retry build.",
                "expected_details": "The 'build' step has run more recently.\n",
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
        (
            "StepOutdatedError source updated",
            {
                "exception_class": errors.StepOutdatedError,
                "kwargs": {
                    "step": steps.PULL,
                    "part": "test-part",
                    "outdated_report": pluginhandler.OutdatedReport(
                        source_updated=True
                    ),
                },
                "expected_brief": "Failed to reuse files from previous run.",
                "expected_resolution": "Run `snapcraft clean` and retry build.",
                "expected_details": "The source has changed on disk.\n",
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
        (
            "SnapcraftEnvironmentError",
            {
                "exception_class": errors.SnapcraftEnvironmentError,
                "kwargs": {"message": "test-message"},
                "expected_brief": "test-message",
                "expected_resolution": "",
                "expected_details": None,
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
        (
            "SnapcraftDataDirectoryMissingError",
            {
                "exception_class": errors.SnapcraftDataDirectoryMissingError,
                "kwargs": {},
                "expected_brief": "Cannot find snapcraft's data files.",
                "expected_resolution": "Re-install snapcraft or verify installation is correct.",
                "expected_details": None,
                "expected_docs_url": None,
                "expected_reportable": True,
            },
        ),
        (
            "SnapcraftMissingLinkerInBaseError",
            {
                "exception_class": errors.SnapcraftMissingLinkerInBaseError,
                "kwargs": {
                    "base": "core18",
                    "linker_path": "/snap/core18/current/lib64/ld-linux.so.2",
                },
                "expected_brief": "Cannot find the linker to use for the target base 'core18'.",
                "expected_resolution": "Verify that the linker exists at the expected path '/snap/core18/current/lib64/ld-linux.so.2' and try again. If the linker does not exist contact the author of the base (run `snap info core18` to get information for this base).",
                "expected_details": None,
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
        (
            "SnapcraftPluginAssertionError",
            {
                "exception_class": errors.SnapcraftPluginAssertionError,
                "kwargs": {"name": "part-name", "reason": "missing important file"},
                "expected_brief": "Unable to build 'part-name': missing important file",
                "expected_resolution": "Ensure the part's configuration and sources are correct.",
                "expected_details": "",
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
        (
            "XAttributeError",
            {
                "exception_class": errors.XAttributeError,
                "kwargs": {"path": "/tmp/foo", "key": "foo", "action": "read"},
                "expected_brief": "Unable to read extended attribute.",
                "expected_resolution": "Check that your filesystem supports extended attributes.",
                "expected_details": "Failed to read attribute 'foo' on '/tmp/foo'.",
                "expected_docs_url": None,
                "expected_reportable": True,
            },
        ),
        (
            "XAttributeTooLongError",
            {
                "exception_class": errors.XAttributeTooLongError,
                "kwargs": {"path": "/tmp/foo", "key": "foo", "value": "bar"},
                "expected_brief": "Unable to write extended attribute as the key and/or value is too long.",
                "expected_resolution": "This issue is generally resolved by addressing/truncating the data source of the long data value. In some cases, the filesystem being used will limit the allowable size.",
                "expected_details": "Failed to write attribute to '/tmp/foo':\nkey='foo' value='bar'",
                "expected_docs_url": None,
                "expected_reportable": True,
            },
        ),
        (
            "SnapcraftPluginBuildError",
            {
                "exception_class": errors.SnapcraftPluginBuildError,
                "kwargs": {"part_name": "foo"},
                "expected_brief": "Failed to build 'foo'.",
                "expected_resolution": "Check the build logs and ensure the part's configuration and sources are correct.",
                "expected_details": None,
                "expected_docs_url": None,
                "expected_reportable": False,
            },
        ),
    )

    def test_snapcraft_exception_handling(
        self,
        exception_class,
        expected_brief,
        expected_details,
        expected_docs_url,
        expected_reportable,
        expected_resolution,
        kwargs,
    ):
        exception = exception_class(**kwargs)

        assert exception.get_brief() == expected_brief
        assert exception.get_resolution() == expected_resolution
        assert exception.get_details() == expected_details
        assert exception.get_docs_url() == expected_docs_url
        assert exception.get_reportable() == expected_reportable
