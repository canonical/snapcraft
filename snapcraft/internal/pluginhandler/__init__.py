# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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

import collections
import contextlib
import copy
import filecmp
import logging
import os
import shutil
import subprocess
import sys
from glob import glob, iglob
from typing import cast, Dict, Set, Sequence  # noqa: F401

import yaml

import snapcraft.extractors
from snapcraft import file_utils
from snapcraft.internal import common, elf, errors, repo, sources, states, steps
from snapcraft.internal.mangling import clear_execstack

from ._build_attributes import BuildAttributes
from ._metadata_extraction import extract_metadata
from ._plugin_loader import load_plugin  # noqa
from ._runner import Runner
from ._patchelf import PartPatcher
from ._dirty_report import Dependency, DirtyReport  # noqa
from ._outdated_report import OutdatedReport

logger = logging.getLogger(__name__)


class PluginHandler:
    @property
    def name(self):
        return self.plugin.name

    def __init__(
        self,
        *,
        plugin,
        part_properties,
        project_options,
        part_schema,
        definitions_schema,
        stage_packages_repo,
        grammar_processor,
        snap_base_path,
        base,
        confinement,
        snap_type,
        soname_cache
    ):
        self.valid = False
        self.plugin = plugin
        self._part_properties = _expand_part_properties(part_properties, part_schema)
        self.stage_packages = []
        self._stage_packages_repo = stage_packages_repo
        self._grammar_processor = grammar_processor
        self._snap_base_path = snap_base_path
        self._base = base
        self._confinement = confinement
        self._snap_type = snap_type
        self._soname_cache = soname_cache
        self._source = grammar_processor.get_source()
        if not self._source:
            self._source = part_schema["source"].get("default")

        self._pull_state = None  # type: states.PullState
        self._build_state = None  # type: states.BuildState
        self._stage_state = None  # type: states.StageState
        self._prime_state = None  # type: states.PrimeState

        self._project_options = project_options
        self.deps = []

        self.stagedir = project_options.stage_dir
        self.primedir = project_options.prime_dir

        # We don't need to set the source_handler on systems where we do not
        # build
        if sys.platform == "linux":
            self.source_handler = self._get_source_handler(self._part_properties)
        else:
            self.source_handler = None

        self._build_attributes = BuildAttributes(
            self._part_properties["build-attributes"]
        )

        # Scriptlet data is a dict of dicts for each step
        self._scriptlet_metadata = collections.defaultdict(
            snapcraft.extractors.ExtractedMetadata
        )
        self._runner = Runner(
            part_properties=self._part_properties,
            sourcedir=self.plugin.sourcedir,
            builddir=self.plugin.build_basedir,
            stagedir=self.stagedir,
            primedir=self.primedir,
            builtin_functions={
                steps.PULL.name: self._do_pull,
                steps.BUILD.name: self.plugin.build,
                steps.STAGE.name: self._do_stage,
                steps.PRIME.name: self._do_prime,
                "set-version": self._set_version,
                "set-grade": self._set_grade,
            },
        )

        self._migrate_state_file()

    def get_pull_state(self) -> states.PullState:
        if not self._pull_state:
            self._pull_state = cast(states.PullState, self.get_state(steps.PULL))
        return self._pull_state

    def get_build_state(self) -> states.BuildState:
        if not self._build_state:
            self._build_state = cast(states.BuildState, self.get_state(steps.BUILD))
        return self._build_state

    def get_stage_state(self) -> states.StageState:
        if not self._stage_state:
            self._stage_state = cast(states.StageState, self.get_state(steps.STAGE))
        return self._stage_state

    def get_prime_state(self) -> states.PrimeState:
        if not self._prime_state:
            self._prime_state = cast(states.PrimeState, self.get_state(steps.PRIME))
        return self._prime_state

    def get_state(self, step) -> states.PartState:
        return states.get_state(self.plugin.statedir, step)

    def _get_source_handler(self, properties):
        """Returns a source_handler for the source in properties."""
        # TODO: we cannot pop source as it is used by plugins. We also make
        # the default '.'
        source_handler = None
        if self._source:
            handler_class = sources.get_source_handler(
                self._source, source_type=properties["source-type"]
            )
            source_handler = handler_class(
                self._source,
                self.plugin.sourcedir,
                source_checksum=properties["source-checksum"],
                source_branch=properties["source-branch"],
                source_tag=properties["source-tag"],
                source_depth=properties["source-depth"],
                source_commit=properties["source-commit"],
            )

        return source_handler

    def _set_version(self, *, version):
        try:
            self._set_scriptlet_metadata(
                snapcraft.extractors.ExtractedMetadata(version=version)
            )
        except errors.ScriptletDuplicateDataError as e:
            raise errors.ScriptletDuplicateFieldError("version", e.other_step)

    def _set_grade(self, *, grade):
        try:
            self._set_scriptlet_metadata(
                snapcraft.extractors.ExtractedMetadata(grade=grade)
            )
        except errors.ScriptletDuplicateDataError as e:
            raise errors.ScriptletDuplicateFieldError("grade", e.other_step)

    def _set_scriptlet_metadata(self, metadata: snapcraft.extractors.ExtractedMetadata):
        step = self.next_step()

        # First, ensure the metadata set here doesn't conflict with metadata
        # already set for this step
        conflicts = metadata.overlap(self._scriptlet_metadata[step])
        if len(conflicts) > 0:
            raise errors.ScriptletDuplicateDataError(step, step, list(conflicts))

        # Now ensure the metadata from this step doesn't conflict with
        # metadata from any other step (if any)
        with contextlib.suppress(errors.NoLatestStepError):
            latest_step = self.latest_step()
            required_steps = latest_step.previous_steps() + [latest_step]
            for other_step in reversed(required_steps):
                state = states.get_state(self.plugin.statedir, other_step)
                conflicts = metadata.overlap(state.scriptlet_metadata)
                if len(conflicts) > 0:
                    raise errors.ScriptletDuplicateDataError(
                        step, other_step, list(conflicts)
                    )

        self._scriptlet_metadata[step].update(metadata)

    def makedirs(self):
        dirs = [
            self.plugin.sourcedir,
            self.plugin.builddir,
            self.plugin.installdir,
            self.plugin.statedir,
            self.stagedir,
            self.primedir,
        ]
        for d in dirs:
            os.makedirs(d, exist_ok=True)

    def _migrate_state_file(self):
        # In previous versions of Snapcraft, the state directory was a file.
        # Rather than die if we're running on output from an old version,
        # migrate it for them.
        if os.path.isfile(self.plugin.statedir):
            with open(self.plugin.statedir, "r") as f:
                step = f.read()

            if step:
                os.remove(self.plugin.statedir)
                os.makedirs(self.plugin.statedir)
                self.mark_done(steps.get_step_by_name(step))

    def working_directory_for_step(self, step: steps.Step) -> str:
        if step == steps.PULL:
            return self.plugin.sourcedir
        elif step == steps.BUILD:
            return self.plugin.builddir
        elif step == steps.STAGE:
            return self.stagedir
        elif step == steps.PRIME:
            return self.primedir

        raise errors.InvalidStepError(step.name)

    def latest_step(self):
        for step in reversed(steps.STEPS):
            if os.path.exists(states.get_step_state_file(self.plugin.statedir, step)):
                return step

        raise errors.NoLatestStepError(self.name)

    def next_step(self):
        latest_step = None
        with contextlib.suppress(errors.NoLatestStepError):
            latest_step = self.latest_step()
        next_step = steps.next_step(latest_step)
        if not next_step:
            raise errors.NoNextStepError(self.name)
        return next_step

    def is_clean(self, step):
        """Return true if the given step hasn't run (or has been cleaned)."""

        try:
            latest_step = self.latest_step()
            return step > latest_step
        except errors.NoLatestStepError:
            return True

    def is_outdated(self, step: steps.Step) -> bool:
        """Return true if the given step is outdated.

        See get_outdated_report() for more information.

        :param steps.Step step: The step to be checked.
        """

        return self.get_outdated_report(step) is not None

    def get_outdated_report(self, step: steps.Step) -> OutdatedReport:
        """Return an OutdatedReport class describing why the step is outdated.

        A step is considered to be outdated if an earlier step in the lifecycle
        has been run more recently, or if the source code changed on disk.
        This means the step needs to be updated by taking modified files from
        the previous step. This is in contrast to a "dirty" step, which must
        be cleaned and run again.

        :param steps.Step step: The step to be checked.
        :returns: OutdatedReport if the step is outdated, None otherwise.
        """

        try:
            return getattr(self, "check_{}".format(step.name))()
        except AttributeError:
            with contextlib.suppress(errors.StepHasNotRunError):
                timestamp = self.step_timestamp(step)

                for previous_step in reversed(step.previous_steps()):
                    # Has a previous step run since this one ran? Then this
                    # step needs to be updated.
                    with contextlib.suppress(errors.StepHasNotRunError):
                        if timestamp < self.step_timestamp(previous_step):
                            return OutdatedReport(previous_step_modified=previous_step)
            return None

    def is_dirty(self, step: steps.Step) -> bool:
        """Return true if the given step is dirty.

        See get_dirty_report() for more information.

        :param steps.Step step: The step to be checked.
        """

        return self.get_dirty_report(step) is not None

    def get_dirty_report(self, step: steps.Step) -> DirtyReport:
        """Return a DirtyReport class describing why the step is dirty.

        A step is considered to be dirty if either YAML properties used by it
        (`stage-packages` are used by the `pull` step, for example), or project
        options used by it (`--target-arch` is used by the `pull` step as well)
        have changed since the step was run. This means the step needs to be
        cleaned and run again. This is in contrast to an "outdated" step, which
        typically doesn't need to be cleaned, just updated with files from an
        earlier step in the lifecycle.

        :param steps.Step step: The step to be checked.
        :returns: DirtyReport if the step is dirty, None otherwise.
        """

        # Retrieve the stored state for this step (assuming it has already run)
        state = states.get_state(self.plugin.statedir, step)
        if state:
            # state.properties contains the old YAML that this step cares
            # about, and we're comparing it to those same keys in the current
            # YAML (self._part_properties). If they've changed, then this step
            # is dirty and needs to run again.
            properties = state.diff_properties_of_interest(self._part_properties)

            # state.project_options contains the old project options that this
            # step cares about, and we're comparing it to those same options in
            # the current project. If they've changed, then this step is dirty
            # and needs to run again.
            options = state.diff_project_options_of_interest(self._project_options)

            if properties or options:
                return DirtyReport(
                    dirty_properties=properties, dirty_project_options=options
                )

        return None

    def should_step_run(self, step, force=False):
        return force or self.is_clean(step)

    def step_timestamp(self, step):
        try:
            return os.stat(
                states.get_step_state_file(self.plugin.statedir, step)
            ).st_mtime
        except FileNotFoundError as e:
            raise errors.StepHasNotRunError(self.name, step) from e

    def mark_done(self, step, state=None):
        if not state:
            state = {}

        with open(states.get_step_state_file(self.plugin.statedir, step), "w") as f:
            f.write(yaml.dump(state))

    def mark_cleaned(self, step):
        state_file = states.get_step_state_file(self.plugin.statedir, step)
        if os.path.exists(state_file):
            os.remove(state_file)

        if os.path.isdir(self.plugin.statedir) and not os.listdir(self.plugin.statedir):
            os.rmdir(self.plugin.statedir)

    def _fetch_stage_packages(self):
        stage_packages = self._grammar_processor.get_stage_packages()
        if stage_packages:
            logger.debug("Fetching stage-packages {!r}".format(stage_packages))
            try:
                self.stage_packages = self._stage_packages_repo.get(stage_packages)
            except repo.errors.PackageNotFoundError as e:
                raise errors.StagePackageDownloadError(self.name, e.message)

    def _unpack_stage_packages(self):
        stage_packages = self._grammar_processor.get_stage_packages()
        if stage_packages:
            logger.debug(
                "Unpacking stage-packages to {!r}".format(self.plugin.installdir)
            )
            self._stage_packages_repo.unpack(self.plugin.installdir)

    def prepare_pull(self, force=False):
        self.makedirs()
        self._fetch_stage_packages()
        self._unpack_stage_packages()

    def pull(self, force=False):
        # Ensure any previously-failed pull is cleared out before we try again
        if os.path.islink(self.plugin.sourcedir) or os.path.isfile(
            self.plugin.sourcedir
        ):
            os.remove(self.plugin.sourcedir)
        elif os.path.isdir(self.plugin.sourcedir):
            shutil.rmtree(self.plugin.sourcedir)

        self.makedirs()
        self._runner.pull()
        self.mark_pull_done()

    def check_pull(self):
        # Check to see if pull needs to be updated
        state_file = states.get_step_state_file(self.plugin.statedir, steps.PULL)

        # Not all sources support checking for updates
        with contextlib.suppress(sources.errors.SourceUpdateUnsupportedError):
            if self.source_handler.check(state_file):
                return OutdatedReport(source_updated=True)
        return None

    def update_pull(self):
        self.source_handler.update()
        self.mark_pull_done()

    def _do_pull(self):
        if self.source_handler:
            self.source_handler.pull()
        self.plugin.pull()

    def mark_pull_done(self):
        pull_properties = self.plugin.get_pull_properties()

        # Add the annotated list of build packages
        part_build_packages = self._part_properties.get("build-packages", [])
        part_build_snaps = self._part_properties.get("build-snaps", [])

        # Extract any requested metadata available in the source directory
        metadata = snapcraft.extractors.ExtractedMetadata()
        metadata_files = []
        for path in self._part_properties.get("parse-info", []):
            file_path = os.path.join(self.plugin.sourcedir, path)
            with contextlib.suppress(errors.MissingMetadataFileError):
                metadata.update(extract_metadata(self.name, file_path))
                metadata_files.append(path)

        self.mark_done(
            steps.PULL,
            states.PullState(
                pull_properties,
                part_properties=self._part_properties,
                project=self._project_options,
                stage_packages=self.stage_packages,
                build_snaps=part_build_snaps,
                build_packages=part_build_packages,
                source_details=self.source_handler.source_details,
                metadata=metadata,
                metadata_files=metadata_files,
                scriptlet_metadata=self._scriptlet_metadata[steps.PULL],
            ),
        )

    def clean_pull(self):
        if self.is_clean(steps.PULL):
            return

        # Remove ubuntu cache (where stage packages are fetched)
        if os.path.exists(self.plugin.osrepodir):
            shutil.rmtree(self.plugin.osrepodir)

        if os.path.exists(self.plugin.sourcedir):
            if os.path.islink(self.plugin.sourcedir):
                os.remove(self.plugin.sourcedir)
            else:
                shutil.rmtree(self.plugin.sourcedir)

        self.plugin.clean_pull()
        self.mark_cleaned(steps.PULL)

    def prepare_build(self, force=False):
        self.makedirs()
        # Stage packages are fetched and unpacked in the pull step, but we'll
        # unpack again here just in case the build step has been cleaned.
        self._unpack_stage_packages()

    def build(self, force=False):
        self.makedirs()

        if not self.plugin.out_of_source_build:
            if os.path.exists(self.plugin.build_basedir):
                shutil.rmtree(self.plugin.build_basedir)

            # FIXME: It's not necessary to ignore here anymore since it's now
            # done in the Local source. However, it's left here so that it
            # continues to work on old snapcraft trees that still have src
            # symlinks.
            def ignore(directory, files):
                if directory == self.plugin.sourcedir:
                    snaps = glob(os.path.join(directory, "*.snap"))
                    if snaps:
                        snaps = [os.path.basename(s) for s in snaps]
                        return common.SNAPCRAFT_FILES + snaps
                    else:
                        return common.SNAPCRAFT_FILES
                else:
                    return []

            # No hard-links being used here in case the build process modifies
            # these files.
            shutil.copytree(
                self.plugin.sourcedir,
                self.plugin.build_basedir,
                symlinks=True,
                ignore=ignore,
            )

        self._do_build()

    def update_build(self):
        if not self.plugin.out_of_source_build:
            # Use the local source to update. It's important to use
            # file_utils.copy instead of link_or_copy, as the build process
            # may modify these files
            source = sources.Local(
                self.plugin.sourcedir,
                self.plugin.build_basedir,
                copy_function=file_utils.copy,
            )
            if not source.check(
                states.get_step_state_file(self.plugin.statedir, steps.BUILD)
            ):
                return
            source.update()

        self._do_build()

    def _do_build(self):
        self._runner.prepare()
        self._runner.build()
        self._runner.install()

        # Organize the installed files as requested. We do this in the build
        # step for two reasons:
        #
        #   1. So cleaning and re-running the stage step works even if
        #      `organize` is used
        #   2. So collision detection takes organization into account, i.e. we
        #      can use organization to get around file collisions between
        #      parts when staging.
        self._organize()

        self.mark_build_done()

    def mark_build_done(self):
        build_properties = self.plugin.get_build_properties()
        plugin_manifest = self.plugin.get_manifest()
        machine_manifest = self._get_machine_manifest()

        # Extract any requested metadata available in the build directory,
        # followed by the install directory (which takes precedence)
        metadata_files = []
        metadata = snapcraft.extractors.ExtractedMetadata()
        for path in self._part_properties.get("parse-info", []):
            file_path = os.path.join(self.plugin.builddir, path)
            found = False
            with contextlib.suppress(errors.MissingMetadataFileError):
                metadata.update(extract_metadata(self.name, file_path))
                found = True

            file_path = os.path.join(self.plugin.installdir, path)
            with contextlib.suppress(errors.MissingMetadataFileError):
                metadata.update(extract_metadata(self.name, file_path))
                found = True

            if found:
                metadata_files.append(path)
            else:
                # If this metadata file is not found in either build or
                # install, check the pull state to make sure it was found
                # there. If not, we need to let the user know.
                state = self.get_pull_state()
                if not state or path not in state.extracted_metadata["files"]:
                    raise errors.MissingMetadataFileError(self.name, path)

        self.mark_done(
            steps.BUILD,
            states.BuildState(
                property_names=build_properties,
                part_properties=self._part_properties,
                project=self._project_options,
                plugin_assets=plugin_manifest,
                machine_assets=machine_manifest,
                metadata=metadata,
                metadata_files=metadata_files,
                scriptlet_metadata=self._scriptlet_metadata[steps.BUILD],
            ),
        )

    def _get_machine_manifest(self):
        # Use subprocess directly here. common.run_output will use binaries out
        # of the snap, and we want to use the one on the host.
        try:
            output = subprocess.check_output(
                [
                    "uname",
                    "--kernel-name",
                    "--kernel-release",
                    "--kernel-version",
                    "--machine",
                    "--processor",
                    "--hardware-platform",
                    "--operating-system",
                ]
            )
        except subprocess.CalledProcessError as e:
            logger.warning(
                "'uname' exited with code {}: unable to record machine "
                "manifest".format(e.returncode)
            )
            return {}

        try:
            uname = output.decode(sys.getfilesystemencoding()).strip()
        except UnicodeEncodeError:
            logger.warning("Could not decode output for 'uname' correctly")
            uname = output.decode("latin-1", "surrogateescape").strip()

        return {
            "uname": uname,
            "installed-packages": repo.Repo.get_installed_packages(),
            "installed-snaps": repo.snaps.get_installed_snaps(),
        }

    def clean_build(self):
        if self.is_clean(steps.BUILD):
            return

        if os.path.exists(self.plugin.build_basedir):
            shutil.rmtree(self.plugin.build_basedir)

        if os.path.exists(self.plugin.installdir):
            shutil.rmtree(self.plugin.installdir)

        self.plugin.clean_build()
        self.mark_cleaned(steps.BUILD)

    def migratable_fileset_for(self, step):
        plugin_fileset = self.plugin.snap_fileset()
        fileset = self._get_fileset(step.name).copy()
        includes = _get_includes(fileset)
        # If we're priming and we don't have an explicit set of files to prime
        # include the files from the stage step
        if step == steps.PRIME and (fileset == ["*"] or len(includes) == 0):
            stage_fileset = self._get_fileset(steps.STAGE.name).copy()
            fileset = _combine_filesets(stage_fileset, fileset)

        fileset.extend(plugin_fileset)

        return _migratable_filesets(fileset, self.plugin.installdir)

    def _get_fileset(self, option, default=None):
        if default is None:
            default = ["*"]

        fileset = getattr(self.plugin.options, option, default)
        return fileset if fileset else default

    def _organize(self):
        fileset = self._get_fileset("organize", {})

        _organize_filesets(fileset.copy(), self.plugin.installdir)

    def stage(self, force=False):
        self.makedirs()
        self._runner.stage()

        # Only mark this step done if _do_stage() didn't run, in which case
        # we have no directories or files to track.
        if self.is_clean(steps.STAGE):
            self.mark_stage_done(set(), set())

    def _do_stage(self):
        snap_files, snap_dirs = self.migratable_fileset_for(steps.STAGE)

        def fixup_func(file_path):
            if os.path.islink(file_path):
                return
            if not file_path.endswith(".pc"):
                return
            repo.fix_pkg_config(self.stagedir, file_path, self.plugin.installdir)

        _migrate_files(
            snap_files,
            snap_dirs,
            self.plugin.installdir,
            self.stagedir,
            fixup_func=fixup_func,
        )
        # TODO once `snappy try` is in place we will need to copy
        # dependencies here too

        self.mark_stage_done(snap_files, snap_dirs)

    def mark_stage_done(self, snap_files, snap_dirs):
        self.mark_done(
            steps.STAGE,
            states.StageState(
                snap_files,
                snap_dirs,
                self._part_properties,
                self._project_options,
                self._scriptlet_metadata[steps.STAGE],
            ),
        )

    def clean_stage(self, project_staged_state):
        if self.is_clean(steps.STAGE):
            return

        state = states.get_state(self.plugin.statedir, steps.STAGE)

        try:
            self._clean_shared_area(self.stagedir, state, project_staged_state)
        except AttributeError:
            raise errors.MissingStateCleanError(steps.STAGE)

        self.mark_cleaned(steps.STAGE)

    def prime(self, force=False) -> None:
        self.makedirs()
        self._runner.prime()

        # Only mark this step done if _do_prime() didn't run, in which case
        # we have no files, directories, or dependency paths to track.
        if self.is_clean(steps.PRIME):
            self.mark_prime_done(set(), set(), set())

    def _do_prime(self) -> None:
        snap_files, snap_dirs = self.migratable_fileset_for(steps.PRIME)
        _migrate_files(snap_files, snap_dirs, self.stagedir, self.primedir)

        if self._snap_type == "app":
            dependency_paths = self._handle_elf(snap_files)
        else:
            dependency_paths = set()

        self.mark_prime_done(snap_files, snap_dirs, dependency_paths)

    def _handle_elf(self, snap_files: Sequence[str]) -> Set[str]:
        elf_files = elf.get_elf_files(self.primedir, snap_files)
        all_dependencies = set()
        # TODO: base snap support
        core_path = common.get_core_path(self._base)

        # Clear the cache of all libs that aren't already in the primedir
        self._soname_cache.reset_except_root(self.primedir)
        for elf_file in elf_files:
            all_dependencies.update(
                elf_file.load_dependencies(
                    root_path=self.primedir,
                    core_base_path=core_path,
                    soname_cache=self._soname_cache,
                )
            )

        dependency_paths = self._handle_dependencies(all_dependencies)

        if not self._build_attributes.keep_execstack():
            clear_execstack(elf_files=elf_files)

        if self._build_attributes.no_patchelf():
            logger.warning(
                "The primed files for part {!r} will not be verified for "
                "correctness or patched: build-attributes: [no-patchelf] "
                "is set.".format(self.name)
            )
        else:
            part_patcher = PartPatcher(
                elf_files=elf_files,
                plugin=self.plugin,
                project=self._project_options,
                confinement=self._confinement,
                core_base=self._base,
                snap_base_path=self._snap_base_path,
                stagedir=self.stagedir,
                primedir=self.primedir,
                stage_packages=self._part_properties.get("stage-packages", []),
            )
            part_patcher.patch()

        return dependency_paths

    def mark_prime_done(self, snap_files, snap_dirs, dependency_paths):
        self.mark_done(
            steps.PRIME,
            states.PrimeState(
                snap_files,
                snap_dirs,
                dependency_paths,
                self._part_properties,
                self._project_options,
                self._scriptlet_metadata[steps.PRIME],
            ),
        )

    def clean_prime(self, project_primed_state, hint=""):
        if self.is_clean(steps.PRIME):
            return

        state = self.get_prime_state()

        try:
            self._clean_shared_area(self.primedir, state, project_primed_state)
        except AttributeError:
            raise errors.MissingStateCleanError(steps.PRIME)

        self.mark_cleaned(steps.PRIME)

    def _clean_shared_area(self, shared_directory, part_state, project_state):
        primed_files = part_state.files
        primed_directories = part_state.directories

        # We want to make sure we don't remove a file or directory that's
        # being used by another part. So we'll examine the state for all parts
        # in the project and leave any files or directories found to be in
        # common.
        for other_name, other_state in project_state.items():
            if other_state and (other_name != self.name):
                primed_files -= other_state.files
                primed_directories -= other_state.directories

        # Finally, clean the files and directories that are specific to this
        # part.
        _clean_migrated_files(primed_files, primed_directories, shared_directory)

    def _handle_dependencies(self, all_dependencies: Set[str]):
        # Split the necessary dependencies into their corresponding location.
        # We'll both migrate and track the system dependencies, but we'll only
        # track the part and staged dependencies, since they should have
        # already been primed by other means, and migrating them again could
        # potentially override the `stage` or `snap` filtering.
        (in_part, staged, primed, system) = _split_dependencies(
            all_dependencies, self.plugin.installdir, self.stagedir, self.primedir
        )
        part_dependency_paths = {os.path.dirname(d) for d in in_part}
        staged_dependency_paths = {os.path.dirname(d) for d in staged}
        dependency_paths = part_dependency_paths | staged_dependency_paths

        if not self._build_attributes.no_system_libraries():
            system_dependency_paths = {os.path.dirname(d) for d in system}
            dependency_paths.update(system_dependency_paths)

            if system:
                # Lots of dependencies are linked with a symlink, so we need to
                # make sure we follow those symlinks when we migrate the
                # dependencies.
                _migrate_files(
                    system,
                    system_dependency_paths,
                    "/",
                    self.primedir,
                    follow_symlinks=True,
                )
                formatted_system = "\n".join(sorted(system))
                logger.warning(
                    "Files from the build host were migrated into the snap to "
                    "satisfy dependencies that would otherwise not be met. "
                    "This feature will be removed in a future release. If "
                    "these libraries are needed in the final snap, ensure "
                    "that the following are either satisfied by a "
                    "stage-packages entry or through a part:\n{}".format(
                        formatted_system
                    )
                )
        return dependency_paths

    def get_primed_dependency_paths(self):
        dependency_paths = set()
        state = self.get_prime_state()
        if state:
            for path in state.dependency_paths:
                dependency_paths.add(os.path.join(self.primedir, path.lstrip("/")))

        return dependency_paths

    def env(self, root):
        return self.plugin.env(root)

    def clean(self, project_staged_state=None, project_primed_state=None, step=None):
        if not project_staged_state:
            project_staged_state = {}

        if not project_primed_state:
            project_primed_state = {}

        try:
            self._clean_steps(project_staged_state, project_primed_state, step)
        except errors.MissingStateCleanError:
            # If one of the step cleaning rules is missing state, it must be
            # running on the output of an old Snapcraft. In that case, if we
            # were specifically asked to clean that step we need to fail.
            # Otherwise, just clean like the old Snapcraft did, and blow away
            # the entire part directory.
            if step:
                raise

            logger.info("Cleaning up for part {!r}".format(self.name))
            if os.path.exists(self.plugin.partdir):
                shutil.rmtree(self.plugin.partdir)

        # Remove the part directory if it's completely empty (i.e. all steps
        # have been cleaned).
        if os.path.exists(self.plugin.partdir) and not os.listdir(self.plugin.partdir):
            os.rmdir(self.plugin.partdir)

    def _clean_steps(self, project_staged_state, project_primed_state, step=None):
        if step:
            if step not in steps.STEPS:
                raise RuntimeError(
                    "{!r} is not a valid step for part {!r}".format(step, self.name)
                )

        if not step or step <= steps.PRIME:
            self.clean_prime(project_primed_state)

        if not step or step <= steps.STAGE:
            self.clean_stage(project_staged_state)

        if not step or step <= steps.BUILD:
            self.clean_build()

        if not step or step <= steps.PULL:
            self.clean_pull()


def _split_dependencies(dependencies, installdir, stagedir, primedir):
    """Split dependencies into their corresponding location.

    Return a tuple of sets for each location.
    """

    part_dependencies = set()
    staged_dependencies = set()
    primed_dependencies = set()
    system_dependencies = set()

    for file_path in dependencies:
        if file_path.startswith(installdir):
            part_dependencies.add(os.path.relpath(file_path, installdir))
        elif file_path.startswith(stagedir):
            staged_dependencies.add(os.path.relpath(file_path, stagedir))
        elif file_path.startswith(primedir):
            primed_dependencies.add(os.path.relpath(file_path, primedir))
        else:
            file_path = file_path.lstrip("/")

            # This was a dependency that was resolved to be on the system.
            # However, it's possible that this library is actually included in
            # the snap and we just missed it because it's in a non-standard
            # path. Let's make sure it isn't already in the part, stage dir, or
            # prime dir. If so, add it to that set.
            if os.path.exists(os.path.join(installdir, file_path)):
                part_dependencies.add(file_path)
            elif os.path.exists(os.path.join(stagedir, file_path)):
                staged_dependencies.add(file_path)
            elif os.path.exists(os.path.join(primedir, file_path)):
                primed_dependencies.add(file_path)
            else:
                system_dependencies.add(file_path)

    return (
        part_dependencies,
        staged_dependencies,
        primed_dependencies,
        system_dependencies,
    )


def _expand_part_properties(part_properties, part_schema):
    """Returns properties with all part schema properties included.

    Any schema properties not set will contain their default value as defined
    in the schema itself.
    """

    # First make a deep copy of the part schema. It contains nested mutables,
    # and we'd rather not change them.
    part_schema = copy.deepcopy(part_schema)

    # Come up with a dictionary of part schema properties and their default
    # values as defined in the schema.
    properties = {}
    for schema_property, subschema in part_schema.items():
        properties[schema_property] = subschema.get("default")

    # Now expand (overwriting if necessary) the default schema properties with
    # the ones from the actual part.
    properties.update(part_properties)

    return properties


def _migratable_filesets(fileset, srcdir):
    includes, excludes = _get_file_list(fileset)

    include_files = _generate_include_set(srcdir, includes)
    exclude_files, exclude_dirs = _generate_exclude_set(srcdir, excludes)

    # And chop files, including whole trees if any dirs are mentioned
    snap_files = include_files - exclude_files
    for exclude_dir in exclude_dirs:
        snap_files = set([x for x in snap_files if not x.startswith(exclude_dir + "/")])

    # Separate dirs from files
    snap_dirs = set(
        [
            x
            for x in snap_files
            if os.path.isdir(os.path.join(srcdir, x))
            and not os.path.islink(os.path.join(srcdir, x))
        ]
    )
    snap_files = snap_files - snap_dirs

    # Make sure we also obtain the parent directories of files
    for snap_file in snap_files:
        dirname = os.path.dirname(snap_file)
        while dirname:
            snap_dirs.add(dirname)
            dirname = os.path.dirname(dirname)

    return snap_files, snap_dirs


def _migrate_files(
    snap_files,
    snap_dirs,
    srcdir,
    dstdir,
    missing_ok=False,
    follow_symlinks=False,
    fixup_func=lambda *args: None,
):

    for directory in snap_dirs:
        src = os.path.join(srcdir, directory)
        dst = os.path.join(dstdir, directory)

        snapcraft.file_utils.create_similar_directory(src, dst)

    for snap_file in snap_files:
        src = os.path.join(srcdir, snap_file)
        dst = os.path.join(dstdir, snap_file)

        snapcraft.file_utils.create_similar_directory(
            os.path.dirname(src), os.path.dirname(dst)
        )

        if missing_ok and not os.path.exists(src):
            continue

        # If the file is already here and it's a symlink, leave it alone.
        if os.path.islink(dst):
            continue

        # Otherwise, remove and re-link it.
        if os.path.exists(dst):
            os.remove(dst)

        if src.endswith(".pc"):
            shutil.copy2(src, dst, follow_symlinks=follow_symlinks)
        else:
            file_utils.link_or_copy(src, dst, follow_symlinks=follow_symlinks)

        fixup_func(dst)


def _organize_filesets(fileset, base_dir):
    for key in sorted(fileset, key=lambda x: ["*" in x, x]):
        src = os.path.join(base_dir, key)
        # Remove the leading slash if there so os.path.join
        # actually joins
        dst = os.path.join(base_dir, fileset[key].lstrip("/"))

        sources = iglob(src, recursive=True)

        for src in sources:
            if os.path.isdir(src) and "*" not in key:
                file_utils.link_or_copy_tree(src, dst)
                # TODO create alternate organization location to avoid
                # deletions.
                shutil.rmtree(src)
            elif os.path.isfile(dst):
                raise errors.SnapcraftEnvironmentError(
                    "Trying to organize file {key!r} to {dst!r}, "
                    "but {dst!r} already exists".format(
                        key=key, dst=os.path.relpath(dst, base_dir)
                    )
                )
            else:
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                shutil.move(src, dst)


def _clean_migrated_files(snap_files, snap_dirs, directory):
    for snap_file in snap_files:
        os.remove(os.path.join(directory, snap_file))

    # snap_dirs may not be ordered so that subdirectories come before
    # parents, and we want to be able to remove directories if possible, so
    # we'll sort them in reverse here to get subdirectories before parents.
    snap_dirs = sorted(snap_dirs, reverse=True)

    for snap_dir in snap_dirs:
        migrated_directory = os.path.join(directory, snap_dir)
        if not os.listdir(migrated_directory):
            os.rmdir(migrated_directory)


def _get_file_list(stage_set):
    includes = []
    excludes = []

    for item in stage_set:
        if item.startswith("-"):
            excludes.append(item[1:])
        elif item.startswith("\\"):
            includes.append(item[1:])
        else:
            includes.append(item)

    _validate_relative_paths(includes + excludes)

    includes = includes or ["*"]

    return includes, excludes


def _generate_include_set(directory, includes):
    include_files = set()
    for include in includes:
        if "*" in include:
            pattern = os.path.join(directory, include)
            matches = iglob(pattern, recursive=True)
            include_files |= set(matches)
        else:
            include_files |= set([os.path.join(directory, include)])

    include_dirs = [x for x in include_files if os.path.isdir(x)]
    include_files = set([os.path.relpath(x, directory) for x in include_files])

    # Expand includeFiles, so that an exclude like '*/*.so' will still match
    # files from an include like 'lib'
    for include_dir in include_dirs:
        for root, dirs, files in os.walk(include_dir):
            include_files |= set(
                [os.path.relpath(os.path.join(root, d), directory) for d in dirs]
            )
            include_files |= set(
                [os.path.relpath(os.path.join(root, f), directory) for f in files]
            )

    return include_files


def _generate_exclude_set(directory, excludes):
    exclude_files = set()

    for exclude in excludes:
        pattern = os.path.join(directory, exclude)
        matches = iglob(pattern, recursive=True)
        exclude_files |= set(matches)

    exclude_dirs = [
        os.path.relpath(x, directory) for x in exclude_files if os.path.isdir(x)
    ]
    exclude_files = set([os.path.relpath(x, directory) for x in exclude_files])

    return exclude_files, exclude_dirs


def _validate_relative_paths(files):
    for d in files:
        if os.path.isabs(d):
            raise errors.PluginError('path "{}" must be relative'.format(d))


def _file_collides(file_this, file_other):
    if not file_this.endswith(".pc"):
        return not filecmp.cmp(file_this, file_other, shallow=False)

    pc_file_1 = open(file_this)
    pc_file_2 = open(file_other)

    try:
        for lines in zip(pc_file_1, pc_file_2):
            for line in zip(lines[0].split("\n"), lines[1].split("\n")):
                if line[0].startswith("prefix="):
                    continue
                if line[0] != line[1]:
                    return True
    except Exception as e:
        raise e from e
    finally:
        pc_file_1.close()
        pc_file_2.close()
    return False


def check_for_collisions(parts):
    """Raises a SnapcraftPartConflictError if conflicts are found."""
    parts_files = {}
    for part in parts:
        # Gather our own files up
        part_files, part_directories = part.migratable_fileset_for(steps.STAGE)
        part_contents = part_files | part_directories

        # Scan previous parts for collisions
        for other_part_name in parts_files:
            common = part_contents & parts_files[other_part_name]["files"]
            conflict_files = []
            for f in common:
                this = os.path.join(part.plugin.installdir, f)
                other = os.path.join(parts_files[other_part_name]["installdir"], f)

                if _paths_collide(this, other):
                    conflict_files.append(f)

            if conflict_files:
                raise errors.SnapcraftPartConflictError(
                    other_part_name=other_part_name,
                    part_name=part.name,
                    conflict_files=conflict_files,
                )

        # And add our files to the list
        parts_files[part.name] = {
            "files": part_contents,
            "installdir": part.plugin.installdir,
        }


def _paths_collide(path1: str, path2: str) -> bool:
    if not (os.path.lexists(path1) and os.path.lexists(path2)):
        return False

    path1_is_dir = os.path.isdir(path1)
    path2_is_dir = os.path.isdir(path2)
    path1_is_link = os.path.islink(path1)
    path2_is_link = os.path.islink(path2)

    # Paths collide if they're both symlinks, but pointing to different places
    if path1_is_link and path2_is_link:
        return os.readlink(path1) != os.readlink(path2)

    # Paths collide if one is a symlink, but not the other
    elif path1_is_link or path2_is_link:
        return True

    # Paths collide if one is a directory, but not the other
    elif path1_is_dir != path2_is_dir:
        return True

    # Paths collide if neither path is a directory, and the files have
    # different contents
    elif not (path1_is_dir and path2_is_dir) and _file_collides(path1, path2):
        return True

    # Otherwise, paths do not conflict
    else:
        return False


def _get_includes(fileset):
    return [x for x in fileset if x[0] != "-"]


def _get_excludes(fileset):
    return [x[1:] for x in fileset if x[0] == "-"]


def _combine_filesets(starting_fileset, modifying_fileset):
    """
    Combine filesets if modifying_fileset is an explicit or implicit
    wildcard.
    """

    starting_excludes = set(_get_excludes(starting_fileset))
    modifying_includes = set(_get_includes(modifying_fileset))

    contradicting_fileset = set.intersection(starting_excludes, modifying_includes)

    if contradicting_fileset:
        raise errors.PrimeFileConflictError(fileset=contradicting_fileset)

    to_combine = False
    # combine if starting_fileset has a wildcard
    # XXX: should this only be a single wildcard and possibly excludes?
    if "*" in modifying_fileset:
        to_combine = True
        modifying_fileset.remove("*")

    # combine if modifying_fileset is only excludes
    if set([x[0] for x in modifying_fileset]) == set("-"):
        to_combine = True

    if to_combine:
        return list(set(starting_fileset + modifying_fileset))
    else:
        return modifying_fileset
