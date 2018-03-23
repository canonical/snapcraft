TODO: sergiusens to add the process for releasing.

# Tests

The stable release update needs to be very well tested, because a broken update will cause problems to a lot of users. And releasing a fix is not fast, so there is not much room for mistakes.

When the changelog for the new version is ready, we have to document the list of tests to run on proposed. It's not necessary to test all the functionalities because we have a good suite of tests that will prevent most of the regressions, so specify tests only for the functionalities that are afected by the changes listed in the changelog. These tests should be open ended and should not have all the steps specified, because the intention is for them to guide the exploratory testing. If you feel that a very specific scenario needs to be executed during this phase, that scenario needs to be automated and run by the autopkgtests instead.

Paste on the SRU bug and on this documet the list of tests to run. Then start a call for testing on the forum, and send the link to twitter.

Most of the testing can be done on LXD containers, except the ones that require GUI and the ones that require a real board. Ideally, similar scenarios for each test should be run on all the distro releases that will get the update, but that can take a long time, so use the available time wisely spreading the tests on the different machines, until you have enough confidence.

These tests should not be fabricated, we have the snapd integration tests to build and run simple snaps. For these, use real projects that have something interesting and challenging for snapcraft, and will let you exercise the features in a real world context. There is a list of potential test subjects here: https://gist.github.com/elopio/54d11955039e97d2083227cd291af039

Once you are confident with your testing, and the feedback from the call for testing is positive, mark the SRU bug as verified.

## 2.39

Commands:
 * Run `snapcraft help`.

Metadata:
  * Build snaps with `adopt-info` and `parse-info` in the snapcraft.yaml, and an appstream metadata file that provides icon and desktop files.

Store:
 * Run `snapcraft export-login` with expiration.
 * Run `snapcraft push`.
 * Push a snap without architectures.

Containers:
 * Build snaps in docker using the beta and edge images.

Exploratory:
 * Build classic snaps.
 * Build snaps without UTF-8 locale.
 * Build snaps with `$SNAPCRAFT_CONTAINER_BUILDS=local snapcraft`.
 * Build snaps with socket activation.
 * Try to build snaps with invalid sources.

## 2.38

Metadata:
 * Build snaps with `adopt-info` and `parse-info` in the snapcraft.yaml, and an appstream metadata file.

Exploratory:
 * Build snaps in docker.
 * Build snaps with `build-snaps` in the `snapcraft.yaml`, with valid and invalid values.
 * Build snaps with `$SNAPCRAFT_CONTAINER_BUILDS=local snapcraft`.
 * Build snaps with advanced grammar in sources.

## 2.37

snapcraft-tests-2.37.mdStore:
 * Run the export-login command.
 * Try to register a reserved name.
 * Run snapcraft list-keys with no keys.
 * Run the push metadata manual tests.

Exploratory:

 * Run snapcraft help with multiple options.
 * Build and run classic snaps.
 * Run snapcraft commands that will return errors.

## 2.36

snapcraft.yaml:
 * Build a snap with `$SNAPCRAFT_ARCH_TRIPLET` in the `snapcraft.yaml`.

Store:
 * Run `snapcraft push-metadata`.

Exploratory:
 * Build and run python snaps.
 * Build snaps with `$SNAPCRAFT_CONTAINER_BUILDS=local snapcraft`, and run different clean commands.

## 2.35

Sources:

 * Build a snap with a deb as the source.

Scripts:

 * Try to run snapcraft with a script (prepare, build or install) that exits non-zero.
 * Run snapcraft with a script (prepare, build or install), edit the script and then run snapcraft again.

Metadata:

 * Make a snap and check that the resulting prime/snap/meta.yaml is nicely ordered.

Docker:

 * Build a classic snap in the snapcore/snapcraft docker container.

Aliases:

 * Build snaps with aliases. Check the deprecation message.

Validation:

 * Try to build a snap with an invalid app name.
 * Try to build a snap with an invalid hook name.
 * Try to build a snap with an invalid part name.

Plugins:

 * Cross-compile an autotools snap.

Exploratory:

 * Run store commands.
 * Run the snapcraft pack command.
 * Try to build snaps that will throw errors, using container builds and cleanbuild.
 * Build node snaps.
 * Build dotnet snaps.
 * Build python snaps.
 * Build ruby snaps.
 * Build snaps recording the manifest.

## 2.34

Container builds:

 * Clean the project without arguments:

     `SNAPCRAFT_CONTAINER_BUILDS=1 snapcraft clean`

Plugins:

 * Build a snap with the catkin plugin.
 * Build a snap with the jhbuild plugin.

Errors:

 * Try to build a snap without mksquashfs.

Exploratory:

 * Use build-snaps to build snaps.
 * Build the same snap multiple times, to check the caching.
 * Build snaps with the nodejs plugin and yarn.
 * Build snaps with SNAPCRAFT_CONTAINER_BUILDS=1 using local and remote LXD.
 * Build snaps with SNAPCRAFT_BUILD_INFO=1, check the manifest.yaml in the prime/snap directory.
 * Try to get as many errors as possible, check the error messages or the trace returned when running with `--debug`.
 * Build snaps with grammar in build and stage packages.

## 2.33

Plugins:

  * Cross-compile and run an autotools snap.
  * Cross-compile and run an waf snap.

Cleanbuild:

  * Test a build failure in cleanbuild with --debug.

Snapd:

  * Test a snap with bash completion.
  * Test a snap with reload-command.

Exploratory:

  * Run clean with various states of dependent parts.
  * Make snaps with yaml merge tags.
  * Make python snaps.
  * Make snaps with `--target-arch` and other snapcraft arguments in different order.

## 2.32

Plugins:

  * Build snaps with the autotools plugin.
  * Build rust snaps for a different architecture.

Integrations:

  * Enable the travis-ci integration in one project.
    * Check that the snap is pushed to the edge channel.

Store:

  * Run the whoami command.

## 2.31

Plugins:
 * Build a Qt snap using the default version.
 * Build go snaps for different architectures using --target-arch.

Exploratory:

 * Build snaps with build-packages.
 * Build classic snaps.
 * Run all the snapcraft commands in different scenarios including failure conditions.

## 2.30

Plugins:
 * Run the kernel manual tests documented in `manual-tests.md`.
 * Build a snap with that uses meson.

Sources:
 * Build a snap with a 7zip source.

Store:
 * Run the release, close and status commands in snaps with channels.

Exploratory:

 * Build snaps with SNAPCRAFT_BUILD_INFO=1
   * Check that a snap can be build with the resulting `snap/prime/snapcraft.yaml`.
 * Run all the snapcraft commands in different scenarios including failure conditions.
 * Cleanbuild snaps with local and remote containers.

## 2.29

Plugins:

 * Build nodejs snaps.
   * Check that by default they use the newer nodejs LTS version.
 * Build a nodejs snap using yarn.
 * Build a classic rust snap.
 * Build the snapcraft snap.
   * Check that the final snap doesn't include any libraries from the /snap/ dir.

Version:

 * Build a snap with `version: git`.
 * Build a snap with `version-script`.

Store:

 * Build classic snaps with SNAPCRAFT_SETUP_CORE.

Exploratory:
 * Build snaps using the snapcraft snap.
 * Build snaps in elementary.
 * Build snaps in kde neon.
 * Cleanbuild snaps in different platforms.
 * Push snaps to the store.

## 2.28

plugins:
  * Build a snap using a python staged on the same snap.
  * Build the pc kernel snap. https://github.com/snapcore/snapcraft/blob/master/manual-tests.md#test-the-pc-kernel
    * Check that an image generated with this kernel works in kvm.
  * Build the dragonboard snap. https://github.com/snapcore/snapcraft/blob/master/manual-tests.md#test-the-dragonboard-410c-kernel
    * Check that an image generated with this kernel works in a dragonboard.
  * Build and run the ROS demo snaps.

Packages:
  * Build a snap with stage packages:
    * Check that the stage packages are saved in parts/$part_name/pull/state
  * Build a snap with a specific version on stage packages.
  * Build a snap with a specific version on build packages.

store:
  * Push a snap, modify it, and push again.
    * Check that the second push is faster, just pushing the delta.
  * Run the `history`, `list-revision` and `revisions` commands.

exploratory:
  * Build snaps with the `source-checksum` keyword.
  * Build snaps with python.
  * Build snaps with cleanbuild, using lxd from a deb and a snap.
  * Build snaps with the snapcraft snap: `sudo snap install snapcraft --edge`.
  * Build and run classic snaps.

## 2.27

plugins:
  * Build and run the checkbox snap.

classic:
  * Build a classic snap without core installed.
  * Snap asciinema in xenial and run it in trusty.

cleanbuild:
  * Cleanbuild a snap using --remote.
  * Cleanbuild a snap with the snapcraft.yaml in the snap directory.

errors and warnings:
  * Build a snap with icon and desktop file in setup/gui.
    * check the deprecation warning.
  * Try to build a snap with a wrong organize valie:
    ```
    organize:
      foo:
    ```
  * Try to push to the store the same snap twice.

exploratory:
  * Build python snaps
  * Build and run snaps with the environment keyword.
  * Build snaps with complex stage-packages grammar. https://github.com/snapcore/snapcraft/blob/master/snapcraft/__init__.py#L147
  * Build snaps, change parts and then try to build again.
    * Check the error messages about the parts and steps that need to be cleaned.
  * Build snaps with multiple periods (`.`) in the icon path.
  * Push snaps with the `DELTA_UPLOADS_EXPERIMENTAL=1` environment variable.

## 2.26

plugins:
  * Make a godeps snap with the go-packages keyword.
  * Make a gradle snap without gradlew.
    * Check that the gradle binary is called instead.
  * Make a catkin snap.
    * Check that the compilers don't end up in the snap.
  * Make a snap with the autotools plugin.

sources:
  * Make a snap with a symlink to a directory.
    * Check that the symlink is preserved in the snap.

tour:
  * Make a cleanbuild of 20-PARTS-PLUGINS/01-reusable-part

parser:
  * Run snapcraft-parser.
    * Check that the parser cache is saved to $HOME/.cache

errors:
  * Try to make a snap with a summary too long.
  * Try to run snapcraft-parser using an unexisting index URL.
  * Try to run snapcraft-parser using a malformed index yaml.
  * Try to run snapcraft-parser with a missing part.
    * Check that the exit code is greater than 0.

store:
  * Try to log in with the wrong password.
    * Check that the error message and its colors make sense.

exploratory:
  * Make snaps with the desktop keyword in an app and push them to the edge channel in the store.
    * Check that the desktop files generated in the snap match the app name.
  * Make snaps with the snapcraft.yaml in the root of the project.
  * Make snaps with the snapcraft.yaml in the snap directory.
  * Make snaps with custom plugins in the parts/plugins directory.
  * Make snaps with custom plugins in the snap/plugins directory.
  * Make snaps with desktop and icon in the setup/gui directory.
  * Make snaps with desktop and icon in the snap/gui directory.
  * Make snaps with stage packages from other architectures.

## 2.25

snapcraft-parser:
  * Run snapcraft-parser -h
  * Try to parse an origin without snapcraft.yaml
  * Run with the debug flag.

plugins:
  * Make snaps with ant, autotools, godeps, catkin, copy, make, nodejs, go, python, qmake gulp and kernel plugins.
    * Check that the build_properties deprecation warning no longer appears.
  * Make a rust snap with conditional compilation.
  * Make a rust snap with the source in a different repository.
  * Make a rust snap with the source in a subdirectory.
  * Make a rust snap with rust-version.
  * Make a nodejs snap with the source in a different repository.
  * Make a godep snap with GOBIN already set.

sources:
  * Make a snap with libgweather-3-6_3.18.2-0ubuntu0.1_amd64.deb as the source.

filters:
  * Make a snap with the prime keyword.
  * Make a snap with the snap keyword.
    * Check that a deprecation message was displayed.

clean:
  * Make a snap, modify the snapcraft.yaml to make it invalid, and run snapcraft clean.
    * Check that all the snapcraft directories are deleted.

confinement:
  * Try to make a snap without the core snap installed.
  * Try to define a part that doesn't exist.

store:
  * Try to push a snap without registering it first.

exploratory:
  * Make snaps with different combinations of stage and prime entries.
  * Make snaps with hooks: https://github.com/snapcore/snapcraft/blob/master/docs/hooks.md
  * Make snaps with the desktop keyword in an app.

## 2.24

help:
 * Run snapcraft plugins.

plugins:
 * Pull and build using a rust part.
   * Check that the download happens only during pull.
 * Build a python snap that has a read-only file.
 * Build snaps with the maven, gradle, cmake, waf and scons plugins.
   * Check that there is no build_properties deprecation warning.

parser:
 * Parse using a part with origin-branch.
 * Run snapcraft-parser -v

store:
 * Run snapcraft registered and snapcraft list-registered.

errors:
 * Try to build using an unrecognized source.

work-in-progress:
 * Build a snap with aliases.
   * Check that the snap yaml contains the aliases.
 * Build a snap with classic confinement.
   * Check that the snap yaml contains the right confinement.

snaps:
 * Build the tomcat demo in an arch that's not amd64.
 * Build the pulse audio snap from https://bugs.launchpad.net/snapcraft/+bug/1587358
   * Check that libraries that end up on the snap are not the system libraries.

exploratory:
 * Build snaps with prepare, build and install scriptlets.

## 2.22

Types:
 * Build a gadget snap.

Commands:
 * Run snapcraft --version.

Store:
 * Exploratory on store commands.

Packages:
 * Build snaps and check that the cache is stored in $HOME/.cache/snapcraft/apt/
 * Build snaps again and check that the cache is used.
 * Build snaps with packages of a specific architecture.

Plugins:
 * Build a catkin snap and check that gcc and g++ are installed in the snap.
 * Build a go and check that the build tags are called for all the packages installed.

Sources:
 * Build a snap using an rpm source.

## 2.20

Plugins:
 * Try to build a snap using an invalid ros distro.
 * Build a snap that uses the waf plugin.
 * Build a snap using the python plugin and bzr.
 * Exploratory testing on the python plugin.

Store:
 * Create a key.
 * Use sign-build with multiple keys.
 * Review the history of a snap.
 * Review the status of a snap.
 * Close a channel.
 * Exploratory testing on gated and validate.

## 2.18.1

Plugins:
 * Build a snap with a python constraints file from a URL.
 * Build a snap with a python requirements file from a URL.
 * Build snaps with the deprecated python2 and python3 plugins.
 * Build a snap using the autotools plugin, with a bootstrap directory in the source.
 * Build a snap with the nodejs plugin and dev-deps.
 * Build a snap with cross-compilation and the dump plugin.
 * Build a snap with the make plugin and artifacts in a dir.
 * Build the dogpile.cache snap with python2.

Sources:
 * Build a snap using a deb file as the source.
 * Build snaps with and without source-depth.

Lifecycle:
 * Measure the speed of various lifecycle commands using mhall's yaml from https://bugs.launchpad.net/snapcraft/+bug/1590599
 * Run an exploratory test on the various lifecycle commands.

Cleanbuild:
 * Build a snap using cleanbuild.

Wiki:
 * Build a snap that depends on a remote part with filesets.
 * Run snapcraft search, check that the results are ordered.
 * Build the plainbox-provider-snappy snap.
 * Try to parse with missing hg, git, bzr and svn.

Store:
 * Run snapcraft login.
 * Build a snap that requires to download another snap.

Errors:
 * Try to build a snap with an error in the yaml, check that the right line for the error is reported.

## 2.17

Plugins:
 * Build snaps with the python, python2 and python3 plugins.
 * Build snaps with the gulp plugin.

Organize:
 * Build a snap with a part that puts some file in a directory, and then another part uses organize on the same directory.
 * Build a snap that uses * in the left side of organize.

Init:
 * Run snapcraft init.

Store:
 * Run snapcraft register-key.
 * Run snapcraft list-keys.

Wiki:
 * Run snapcraft-parse with SNAPCRAFT_PROJECT_NAME and SNAPCRAFT_PROJECT_VERSION in the source.

## 2.16

Variables:
 * Build and run a snap using snapcraft variables in the build.
 * Build a snap using snapcraft variables in the yaml.

Plugins:
 * Build python2 and python3 snaps and check that they include no .pyc, .pth nor __pycache__.
 * Build a nodejs snap in arm64.
 * Build a nodejs snap in launchpad.

Grade:
 * Build snaps with grade devel and stable.
 * Push the snaps to the store and release them.

Cleanbuild:
 * Make a faulty snap and run cleanbuild with --debug.

Errors:
 * Build a snap with file conflicts.

Wiki:
 * Run snapcraft-parse with the wiki.

Store:
 * Register a private snap.
 * Push a private snap.
 * Release a private snap.
