# Snapcraft CLI reference

## lifecycle and misc

### version

    $ snapcraft --version
    2.12.1

### help

    $ snapcraft help
    Usage:
      snapcraft [options] [--enable-geoip --no-parallel-build]
      snapcraft [options] init
      snapcraft [options] pull [<part> ...]  [--enable-geoip]
      snapcraft [options] build [<part> ...] [--no-parallel-build]
      snapcraft [options] stage [<part> ...]
      snapcraft [options] prime [<part> ...]
      snapcraft [options] strip [<part> ...]
      snapcraft [options] clean [<part> ...] [--step <step>]
      snapcraft [options] snap [<directory> --output <snap-file>]
      snapcraft [options] cleanbuild
      snapcraft [options] login
      snapcraft [options] logout
      snapcraft [options] register <snap-name>
      snapcraft [options] upload <snap-file>
      snapcraft [options] push <snap-file> [--release <channels>]
      snapcraft [options] release <snap-name> <revision> <channel>
      snapcraft [options] list-plugins
      snapcraft [options] tour [<directory>]
      snapcraft [options] update
      snapcraft [options] define <part-name>
      snapcraft [options] search [<query> ...]
      snapcraft [options] help (topics | <plugin> | <topic>) [--devel]
      snapcraft (-h | --help)


    $ snapcraft --help
    $ snapcraft -h
    snapcraft

    Usage:
      snapcraft [options] [--enable-geoip --no-parallel-build]
      snapcraft [options] init
      snapcraft [options] pull [<part> ...]  [--enable-geoip]
      snapcraft [options] build [<part> ...] [--no-parallel-build]
      snapcraft [options] stage [<part> ...]
      snapcraft [options] prime [<part> ...]
      snapcraft [options] strip [<part> ...]
      snapcraft [options] clean [<part> ...] [--step <step>]
      snapcraft [options] snap [<directory> --output <snap-file>]
      snapcraft [options] cleanbuild
      snapcraft [options] login
      snapcraft [options] logout
      snapcraft [options] register <snap-name>
      snapcraft [options] upload <snap-file>
      snapcraft [options] list-plugins
      snapcraft [options] tour [<directory>]
      snapcraft [options] update
      snapcraft [options] define <part-name>
      snapcraft [options] search [<query> ...]
      snapcraft [options] help (topics | <plugin> | <topic>) [--devel]
      snapcraft (-h | --help)
      snapcraft --version

    Options:
      -h --help                             show this help message and exit
      -v --version                          show program version and exit
      -d --debug                            print debug information while executing
                                            (including backtraces)
      --target-arch ARCH                    EXPERIMENTAL: sets the target
                                            architecture. Very few plugins support
                                            this.

    Options specific to pulling:
      --enable-geoip         enables geoip for the pull step if stage-packages
                             are used.

    Options specific to building:
      --no-parallel-build                   use only a single build job per part
                                            (the default number of jobs per part is
                                            equal to the number of CPUs)

    Options specific to cleaning:
      -s <step>, --step <step>              only clean the specified step and those
                                            that depend upon it. <step> can be one
                                            of: pull, build, stage or strip.

    Options specific to snapping:
      -o <snap-file>, --output <snap-file>  used in case you want to rename the
                                            snap.

    The available commands are:
      help         Obtain help for a certain plugin or topic
      init         Initialize a snapcraft project.
      list-plugins List the available plugins that handle different types of part.
      login        Authenticate session against Ubuntu One SSO.
      logout       Clear session credentials.
      register     Register the package name in the store.
      tour         Setup the snapcraft examples tour in the specified directory,
                   or ./snapcraft-tour/.
      upload       Upload a snap to the Ubuntu Store.

    The available lifecycle commands are:
      clean        Remove content - cleans downloads, builds or install artifacts.
      cleanbuild   Create a snap using a clean environment managed by lxd.
      pull         Download or retrieve artifacts defined for a part.
      build        Build artifacts defined for a part. Build systems capable of
                   running parallel build jobs will do so unless
                   "--no-parallel-build" is specified.
      stage        Stage the part's built artifacts into the common staging area.
      prime        Final copy and preparation for the snap.
      snap         Create a snap.

    Parts ecosystem commands
      update       Updates the parts listing from the cloud.
      define       Shows the definition for the cloud part.
      search       Searches the remotes part cache for matching parts.

    Calling snapcraft without a COMMAND will default to 'snap'

    The cleanbuild command requires a properly setup lxd environment that
    can connect to external networks. Refer to the "Ubuntu Desktop and
    Ubuntu Server" section on
    https://linuxcontainers.org/lxd/getting-started-cli
    to get started.

    For more help, visit the documentation:
    http://developer.ubuntu.com/snappy/snapcraft


    $ man snapcraft
    No manual entry for snapcraft
    See 'man 7 undocumented' for help when manual pages are not available.


    $ snapcraft help topics
    plugins
    sources


    $ snapcraft help plugins
    Snapcraft plugins drive different build systems

    Each part has a build system . Most parts are built from source using one of
    a range of build systems such as CMake or Scons. Some parts are pre-built
    and just copied into place, for example parts that reuse existing binary
    packages.

    You tell snapcraft which build system it must drive by specifying the
    snapcraft plugin for that part. Every part must specify a plugin explicitly
    (when you see a part that does not specify a plugin, thats because the
    actual part definition is in the cloud, where the plugin is specified!)

    These plugins implement a lifecycle over the following steps:

      - pull:   retrieve the source for the part from the specified location
      - build:  drive the build system determined by the choice of plugin
      - stage:  consolidate desireable files from all the parts in one tree
      - prime:  distill down to only the files which will go into the snap
      - snap:   compress the prime tree into the installable snap file

    These steps correspond to snapcraft commands. So when you initiate a
    'snapcraft pull' you will invoke the respective plugin for each part in
    the snap, in sequence, to handle the source pull. Each part will then have a
    fully populated parts/<part-name>/src/ directory. Similarly, if you then say
    'snapcraft build' you will invoke the plugin responsible for each part in
    turn, to build the part.

    # Snapcraft Lifecycle

    ## Pull

    In this first step, source material is retrieved from the specified
    location, whether that is a URL for a tarball, a local path to a source tree
    inside the snap, a revision control reference to checkout, or something
    specific to the plugin such as PyPI. The plugin might also download
    necessary artifacts, such as the Java SDK, which are not specific to the
    particular part but which are needed by the plugin to handle its type of
    build system.

    All the downloaded content for each part goes into the
    `parts/<part-name>/src/` directory, which acts as a cache to prevent
    re-fetching content. You can clean that cache out with 'snapcraft clean'.

    ## Build

    Snapcraft calculates an appropriate sequence to build the parts, based on
    explicit 'after' references and the order of the parts in the
    snapcraft.yaml. Each part is built in the `parts/<part-name>/build`
    directory and installed into `parts/<part-name>/install`.

    Note the install step - we might actually want to use built artifacts from
    one part in the build process of another, so the `parts/<part-name>/install`
    directory is useful as a 'working fresh install' of the part.

    Between the plugin, the part defintion YAML, and the build system of the
    part, it is expected that the part can be built and installed in the right
    place.

    At this point you have a tree under `parts/` with a subdirectory for every
    part, and underneath those, separate src, build and install trees for each
    part.

    ## Stage

    We now need to start consolidating the important pieces of each part into a
    single tree. We do this twice - once in a very sweeping way that will
    produce a lot of extraneous materials but is useful for debugging. This is
    the 'stage' step of the lifecycle, because we move a lot of the build output
    from each part into a consolidated tree under `stage/` which has the
    structure of a snap but has way too much extra information.

    The important thing about the staging area is that it lets you get all the
    shared libraries in one place and lets you find overlapping content in the
    parts. You can also try this directory as if it were a snap, and you'll have
    all the debugging information in the tree, which is useful for developers.

    Each part describes its own staging content - the files that should be
    staged. The part will often describe "chunks" of content, called filesets,
    so that they can be referred to as a useful set rather than having to call
    out individual files.

    ## Prime

    It is useful to have a directory tree which exactly mirrors the structure of
    the final snap. This is the `prime/` directory, and the lifecycle includes a
    'prime' step which copies only that final, required content from the
    `stage/` directory into the `prime/` directory.

    So the `prime/` directory contains only the content that will be put into
    the final snap, unlike the staging area which may include debug and
    development files not destined for your snap.

    The snap metadata will also be placed in `./prime/meta` during the prime
    step, so this `./prime` directory is useful for inspecting exactly what is
    going into your snap or to conduct any final post-processing on snapcraft's
    output.

    ## Snap

    The final step in the snapcraft lifecycle builds a snap out of the `prime/`
    directory. It will be in the top level directory, alongside snapcraft.yaml,
    called <name>-<version>-<arch>.snap


    # Standard part definition keywords

    There are several builtin keywords which can be used in any part regardless
    of the choice of plugin.

      - after: [part, part, part...]

        Snapcraft will make sure that it builds all of the listed parts before
        it tries to build this part. Essentially these listed dependencies for
        this part, useful when the part needs a library or tool built by another
        part.

        If such a dependency part is not defined in this snapcraft.yaml, it must
        be defined in the cloud parts library, and snapcraft will retrieve the
        definition of the part from the cloud. In this way, a shared library of
        parts is available to every snap author - just say 'after' and list the
        parts you want that others have already defined.

      - build-packages: [deb, deb, deb...]

        A list of Ubuntu packages to install on the build host before building
        the part. The files from these packages will not go into the final snap
        unless they are also explicitly described in stage-packages.

      - stage-packages: [deb, deb, deb...]

        A list of Ubuntu packages must be unpacked in the `stage/` directory.
        XXX before build? Before stage?

      - organize: YAML

        Snapcraft will rename files according to this YAML sub-section. The
        content of the 'organize' section consists of old path keys, and their
        new values after the renaming.

        This can be used to avoid conflicts between parts that use the same
        name, or to map content from different parts into a common conventional
        file structure. For example:

          organize:
            usr/oldfilename: usr/newfilename
            usr/local/share/: usr/share/

        The key is the internal part filename, the value is the exposed filename
        that will be used during the staging process. You can rename whole
        subtrees of the part, or just specific files.

        Note that the path is relative (even though it is "usr/local") because
        it refers to content underneath parts/<part-name>/install which is going
        to be mapped into the stage and prime areas.

      - filesets: YAML

        When we map files into the stage and prime areas on the way to putting
        them into the snap, it is convenient to be able to refer to groups of
        files as well as individual files.  Snapcraft lets you name a fileset
        and then use it later for inclusion or exclusion of those files from the
        resulting snap.

        For example, consider man pages of header files.. You might want them
        in, or you might want to leave them out, but you definitely don't want
        to repeatedly have to list all of them either way.

        This section is thus a YAML map of fileset names (the keys) to a list of
        filenames. The list is built up by adding individual files or whole
        subdirectory paths (and all the files under that path) and wildcard
        globs, and then pruning from those paths.

        The wildcard * globs all files in that path. Exclusions are denoted by
        an initial `-`.

        For example you could add usr/local/* then remove usr/local/man/*:

          filesets:
            allbutman: [ usr/local/*, -usr/local/man/* ]
            manpages: [ usr/local/man ]

        Filenames are relative to the part install directory in
        `parts/<part-name>/install`. If you have used 'organize' to rename files
        then the filesets will be built up from the names after organization.

      - stage: YAML file and fileset list

        A list of files from a part install directory to copy into `stage/`.
        Rules applying to the list here are the same as those of filesets.
        Referencing of fileset keys is done with a $ prefixing the fileset key,
        which will expand with the value of such key.

        For example:

          stage:
            - usr/lib/*   # Everything under parts/<part-name>/install/usr/lib
            - -usr/lib/libtest.so   # Excludng libtest.so
            - $manpages             # Including the 'manpages' fileset

      - prime: YAML file and fileset list

        A list of files from a part install directory to copy into `prime/`.
        This section takes exactly the same form as the 'stage' section  but the
        files identified here will go into the ultimate snap (because the
        `prime/` directory reflects the file structure of the snap with no
        extraneous content).


    $ snapcraft help sources
    Common 'source' options.

    Unless the part plugin overrides this behaviour, a part can use these
    'source' keys in its definition. They tell snapcraft where to pull source
    code for that part, and how to unpack it if necessary.

      - source: url-or-path

        A URL or path to some source tree to build. It can be local
        ('./src/foo') or remote ('https://foo.org/...'), and can refer to a
        directory tree or a tarball or a revision control repository
        ('git:...').

      - source-type: git, bzr, hg, svn, tar, or zip

        In some cases the source string is not enough to identify the version
        control system or compression algorithim. The source-type key can tell
        snapcraft exactly how to treat that content.

      - source-branch: <branch-name>

        Snapcraft will checkout a specific branch from the source tree. This
        only works on multi-branch repositories from git and hg (mercurial).

      - source-tag: <tag>

        Snapcraft will checkout the specific tag from the source tree revision
        control system.

      - source-subdir: path

        Snapcraft will checkout the repository or unpack the archive referred to
        by the 'source' keyword into parts/<part-name>/src/ but it will only
        copy the specified subdirectory into parts/<part-name>/build/

    Note that plugins might well define their own semantics for the 'source'
    keywords, because they handle specific build systems, and many languages
    have their own built-in packaging systems (think CPAN, PyPI, NPM). In those
    cases you want to refer to the help text for the specific plugin.

      snapcraft help <plugin>


#### Plugins help

    $ snapcraft help ant
    The ant plugin is useful for ant based parts.

    The ant build system is commonly used to build Java projects.
    The plugin requires a build.xml in the root of the source tree.

    This plugin uses the common plugin keywords as well as those for "sources".
    For more information check the 'plugins' topic for the former and the
    'sources' topic for the latter.


    $ snapcraft help autotools
    The autotools plugin is used for autotools based parts.

    Autotools based projects are the ones that have the usual
    `./configure && make && make install` instruction set.

    The plugin tries to build using ./configure first, if it is not there
    it will run ./autogen and if autogen is not there it will run autoreconf.

    This plugin uses the common plugin keywords as well as those for "sources".
    For more information check the 'plugins' topic for the former and the
    'sources' topic for the latter.

    In additon, this plugin uses the following plugin-specific keywords:

        - configflags:
          (list of strings)
          configure flags to pass to the build such as those shown by running
          './configure --help'
        - install-via:
          (enum, 'destdir' or 'prefix')
          Whether to install via DESTDIR or by using --prefix (default is
          'destdir')


    $ snapcraft help catkin
    The catkin plugin is useful for building ROS parts.

    This plugin uses the common plugin keywords as well as those for "sources".
    For more information check the 'plugins' topic for the former and the
    'sources' topic for the latter.

    Additionally, this plugin uses the following plugin-specific keywords:

        - catkin-packages:
          (list of strings)
          List of catkin packages to build.
        - source-space:
          (string)
          The source space containing Catkin packages. By default this is 'src'.
        - include-roscore:
          (boolean)
          Whether or not to include roscore with the part. Defaults to true.


    $ snapcraft help cmake
    The cmake plugin is useful for building cmake based parts.

    These are projects that have a CMakeLists.txt that drives the build.
    The plugin requires a CMakeLists.txt in the root of the source tree.

    This plugin uses the common plugin keywords as well as those for "sources".
    For more information check the 'plugins' topic for the former and the
    'sources' topic for the latter.

    Additionally, this plugin uses the following plugin-specific keywords:

        - configflags:
          (list of strings)
          configure flags to pass to the build using the common cmake semantics.


    $ snapcraft help copy
    The copy plugin is useful for assets or other sources with no build system.

    This plugin uses the common plugin keywords as well as those for 'sources'
    (though the 'source' keyword is optional). For more information check the
    'plugins' topic for the former and the 'sources' topic for the latter.

    Additionally, this plugin uses the following plugin-specific keywords:

        - files:
          (object)
          A dictionary of key-value pairs. The key is the current location of the
          file relative to snapcraft.yaml (unless `source` is specified, in which
          case it's relative to the root of the source). The value is where to
          place the file in-snap, and is relative to the root of the snap. This
          works like `cp -r <key> <value>`. Note that globbing is supported for the
          key, allowing one to use *, ?, and character ranges expressed with [].


    $ snapcraft help go
    The go plugin can be used for go projects using `go get`.

    This plugin uses the common plugin keywords, for more information check the
    'plugins' topic.

    This plugin uses the common plugin keywords as well as those for "sources".
    For more information check the 'plugins' topic for the former and the
    'sources' topic for the latter.

    Additionally, this plugin uses the following plugin-specific keywords:

        - go-packages:
          (list of strings)
          Go packages to fetch, these must be a "main" package. Dependencies
          are pulled in automatically by `go get`.
          Packages that are not "main" will not cause an error, but would
          not be useful either.

        - go-importpath:
          (string)
          This entry tells the checked out `source` to live within a certain path
          within `GOPATH`.
          This is not needed and does not affect `go-packages`.


    $ snapcraft help gulp
    This plugin is used for gulp.js, the streaming build system.

    The plugin uses gulp to drive the build. It requires a gulpfile.js in
    the root of the source.

    This plugin uses the common plugin keywords as well as those for "sources".
    For more information check the 'plugins' topic for the former and the
    'sources' topic for the latter.

    Additionally, this plugin uses the following plugin-specific keywords:

        - gulp-tasks:
          (list)
          A list of gulp tasks to run.
        - node-engine:
          (string)
          The version of nodejs to use for the build.


    $ snapcraft help jdk
    The plugin has no documentation


    $ snapcraft help kbuild
    The kbuild plugin is used for building kbuild based projects as snapcraft
    parts.

    This plugin is based on the snapcraft.BasePlugin and supports the properties
    provided by that plus the following kbuild specific options with semantics as
    explained above:

        - kdefconfig:
          (list of kdefconfigs)
          defconfig target to use as the base configuration. default: "defconfig"

        - kconfigfile:
          (filepath)
          path to file to use as base configuration. If provided this option wins
          over kdefconfig. default: None

        - kconfigs
          (list of strings)
          explicit list of configs to force; this will override the configs that
          were set as base through kdefconfig and kconfigfile and dependent configs
          will be fixed using the defaults encoded in the kbuild config
          definitions.  If you don't want default for one or more implicit configs
          coming out of these, just add them to this list as well.

    The plugin applies your selected defconfig first by running

        make defconfig

    and then uses the kconfigs flag to augment the resulting config by prepending
    the configured kconfigs values to the .config and running

        "yes" "" | make oldconfig

    to create an updated .config file.

    If kconfigfile is provided this plugin will use the provided config file
    wholesale as the starting point instead of make $kdefconfig. In case user
    configures both a kdefconfig as well as kconfigfile, kconfigfile approach will
    be used.


    $ snapcraft help kernel
    The kernel plugin refines the generic kbuild plugin to allow building
    kernel snaps with all the bells and whistles in one shot...

    WARNING: this plugin's API is unstable. The cross compiling support is
             experimental.

    The following kernel specific options are provided by this plugin:

        - kernel-image-target:
          (string; default: bzImage)
          the kernel image make target to build; maps to make target.

        - kernel-initrd-modules:
          (array of string)
          list of modules to include in initrd; note that kernel snaps do not
          provide the core bootlogic which comes from snappy Ubuntu Core
          OS snap. Include all modules you need for mounting rootfs here.

        - kernel-with-firmware:
          (boolean; default: True)
          use this flag to disable shipping binary firmwares

        - kernel-initrd-firmware:
          (array of string)
          list of firmware files to include in the initrd; these need to be
          relative paths to .installdir and this option does not work if you
          disable building firmware

        - kernel-initrd-compression:
          (string; default: gz)
          initrd compression to use; the only supported value now is 'gz'.

        - kernel-device-trees:
          (array of string)
          list of device trees to build, the format is <device-tree-name>.dts.


    $ snapcraft help make
    The make plugin is useful for building make based parts.

    Make based projects are projects that have a Makefile that drives the
    build.

    This plugin uses the common plugin keywords as well as those for "sources".
    For more information check the 'plugins' topic for the former and the
    'sources' topic for the latter.

    Additionally, this plugin uses the following plugin-specific keyword:

        - makefile:
          (string)
          Use the given file as the makefile.

        - make-parameters:
          (list of strings)
          Pass the given parameters to the make command.


    $ snapcraft help maven
    This plugin is useful for building parts that use maven.

    The maven build system is commonly used to build Java projects.
    The plugin requires a pom.xml in the root of the source tree.

    This plugin uses the common plugin keywords as well as those for "sources".
    For more information check the 'plugins' topic for the former and the
    'sources' topic for the latter.

    Additionally, this plugin uses the following plugin-specific keywords:

        - maven-options:
          (list of strings)
          flags to pass to the build using the maven semantics for parameters.


    $ snapcraft help nil
    The nil plugin is useful for parts with no source.

    Using this, parts can be defined purely by utilizing properties automatically
    included by Snapcraft, e.g. stage-packages.


    $ snapcraft help nodejs
    The nodejs plugin is useful for node/npm based parts.

    The plugin uses node to install dependencies from `package.json`. It
    also sets up binaries defined in `package.json` into the `PATH`.

    This plugin uses the common plugin keywords as well as those for "sources".
    For more information check the 'plugins' topic for the former and the
    'sources' topic for the latter.

    Additionally, this plugin uses the following plugin-specific keywords:

        - node-packages:
          (list)
          A list of dependencies to fetch using npm.
        - node-engine:
          (string)
          The version of nodejs you want the snap to run on.


    $ snapcraft help python2
    The python2 plugin can be used for python 2 based parts.

    The python2 plugin can be used for python 2 projects where you would
    want to do:

        - import python modules with a requirements.txt
        - build a python project that has a setup.py
        - install sources straight from pip

    This plugin uses the common plugin keywords as well as those for "sources".
    For more information check the 'plugins' topic for the former and the
    'sources' topic for the latter.

    Additionally, this plugin uses the following plugin-specific keywords:

        - requirements:
          (string)
          path to a requirements.txt file
        - python-packages:
          (list)
          A list of dependencies to get from PyPi


    $ snapcraft help python3
    The python3 plugin can be used for python 3 based parts.

    The python3 plugin can be used for python 3 projects where you would
    want to do:

        - import python modules with a requirements.txt
        - build a python project that has a setup.py
        - install sources straight from pip

    This plugin uses the common plugin keywords as well as those for "sources".
    For more information check the 'plugins' topic for the former and the
    'sources' topic for the latter.

    Additionally, this plugin uses the following plugin-specific keywords:

        - requirements:
          (string)
          path to a requirements.txt file
        - python-packages:
          (list)
          A list of dependencies to get from PyPi


    $ snapcraft help qmake
    The qmake plugin is useful for building qmake-based parts.

    These are projects that are built using .pro files.

    This plugin uses the common plugin keywords as well as those for "sources".
    For more information check the 'plugins' topic for the former and the
    'sources' topic for the latter.

    Additionally, this plugin uses the following plugin-specific keywords:

        - options:
          (list of strings)
          additional options to pass to the qmake invocation.
        - qt-version:
          (enum, 'qt4' or 'qt5')
          Version of Qt to use with qmake.
        - project-files:
          (list of strings)
          list of .pro files to pass to the qmake invocation.


    $ snapcraft help scons
    The scons plugin is useful for building parts that build with scons.

    These are projects that have a SConstruct that drives the build.

    This plugin uses the common plugin keywords as well as those for "sources".
    For more information check the 'plugins' topic for the former and the
    'sources' topic for the latter.

    Additionally, this plugin uses the following plugin-specific keywords:

        - scons-options:
          (list of strings)
          flags to pass to the build using the scons semantics for parameters.


    $ snapcraft help tar-content
    The plugin does not exist. Run `snapcraft list-plugins` to see the available plugins.


    $ snapcraft help tar_content
    The plugin has no documentation


### tour

    $ snapcraft tour
    Snapcraft tour initialized in ./snapcraft-tour/
    Instructions are in the README, or https://snapcraft.io/create/#begin

Initialize the tour to an unexisting directory:

    $ snapcraft tour idontexist
    Snapcraft tour initialized in idontexist
    Instructions are in the README, or https://snapcraft.io/create/#begin

Initialize the tour to an existing directory:

    $ snapcraft tour iexist
    Snapcraft tour initialized in iexist
    Instructions are in the README, or https://snapcraft.io/create/#begin

### list-plugins

    $ snapcraft list-plugins
    ant        catkin  copy  gulp  kbuild  make   nil     python2  qmake  tar-content
    autotools  cmake   go    jdk   kernel  maven  nodejs  python3  scons

### init

    $ snapcraft init
    Created snapcraft.yaml.
    Edit the file to your liking or run `snapcraft` to get started

### pull

Pull in a clean project:

    $ snapcraft pull
    Preparing to pull part1
    Pulling part1
    Preparing to pull part2
    Pulling part2

Pull again:

    $ snapcraft pull
    Skipping pull part2 (already ran)
    Skipping pull part1 (already ran)

Add a part and pull again:

    $ snapcraft pull
    Skipping pull part2 (already ran)
    Preparing to pull part3
    Pulling part3
    Skipping pull part1 (already ran)

Add a part and pull again:

    $ snapcraft pull part1
    Preparing to pull part1
    Pulling part1

Pull again the part:

    $ snapcraft pull part1
    Skipping pull part1 (already ran)

### build

Build in a clean project:

    $ snapcraft build
    Preparing to pull part3
    Pulling part3
    Preparing to pull part2
    Pulling part2
    Preparing to pull part1
    Pulling part1
    Preparing to build part3
    Building part3
    Preparing to build part2
    Building part2
    Preparing to build part1
    Building part1

Build after pull was executed:

    $ snapcraft build
    Skipping pull part1 (already ran)
    Skipping pull part2 (already ran)
    Preparing to build part1
    Building part1
    Preparing to build part2
    Building part2

Build again:

    $ snapcraft build
    Skipping pull part2 (already ran)
    Skipping pull part1 (already ran)
    Skipping build part2 (already ran)
    Skipping build part1 (already ran)

Add a part and build again:

    $ snapcraft build
    Preparing to pull part3
    Pulling part3
    Skipping pull part2 (already ran)
    Skipping pull part1 (already ran)
    Preparing to build part3
    Building part3
    Skipping build part2 (already ran)
    Skipping build part1 (already ran)

### stage

Stage in a clean project:

    $ snapcraft stage
    Preparing to pull part2
    Pulling part2
    Preparing to pull part1
    Pulling part1
    Preparing to build part2
    Building part2
    Preparing to build part1
    Building part1
    Staging part2
    Staging part1

Stage after build was executed:

    $ snapcraft stage
    Skipping pull part2 (already ran)
    Skipping pull part1 (already ran)
    Skipping build part2 (already ran)
    Skipping build part1 (already ran)
    Staging part2
    Staging part1

Stage again:

    $ snapcraft stage
    Skipping pull part2 (already ran)
    Skipping pull part1 (already ran)
    Skipping build part2 (already ran)
    Skipping build part1 (already ran)
    Skipping stage part2 (already ran)
    Skipping stage part1 (already ran)

Add a part and stage again:

    $ snapcraft stage
    Preparing to pull part3
    Pulling part3
    Skipping pull part2 (already ran)
    Skipping pull part1 (already ran)
    Preparing to build part3
    Building part3
    Skipping build part2 (already ran)
    Skipping build part1 (already ran)
    Staging part3
    Skipping stage part2 (already ran)
    Skipping stage part1 (already ran)

### prime

Prime in a clean project:

    $ snapcraft prime
    Preparing to pull part2
    Pulling part2
    Preparing to pull part1
    Pulling part1
    Preparing to build part2
    Building part2
    Preparing to build part1
    Building part1
    Staging part2
    Staging part1
    Priming part2
    Priming part1

Prime after stage was executed:

    $ snapcraft prime
    Skipping pull part1 (already ran)
    Skipping pull part2 (already ran)
    Skipping build part1 (already ran)
    Skipping build part2 (already ran)
    Skipping stage part1 (already ran)
    Skipping stage part2 (already ran)
    Priming part1
    Priming part2

Prime again:

    $ snapcraft prime
    Skipping pull part1 (already ran)
    Skipping pull part2 (already ran)
    Skipping build part1 (already ran)
    Skipping build part2 (already ran)
    Skipping stage part1 (already ran)
    Skipping stage part2 (already ran)
    Skipping prime part1 (already ran)
    Skipping prime part2 (already ran)

Add a part and prime again:

    $ snapcraft prime
    Skipping pull part2 (already ran)
    Preparing to pull part3
    Pulling part3
    Skipping pull part1 (already ran)
    Skipping build part2 (already ran)
    Preparing to build part3
    Building part3
    Skipping build part1 (already ran)
    Skipping stage part2 (already ran)
    Staging part3
    Skipping stage part1 (already ran)
    Skipping prime part2 (already ran)
    Priming part3
    Skipping prime part1 (already ran)

### strip

    $ snapcraft strip
    DEPRECATED: use 'prime' instead of 'strip'
    Preparing to pull part1
    Pulling part1
    Preparing to pull part2
    Pulling part2
    Preparing to build part1
    Building part1
    Preparing to build part2
    Building part2
    Staging part1
    Staging part2
    Priming part1
    Priming part2

### clean

Clean in a clean project:

    $ snapcraft clean
    Skipping cleaning priming area for part2 (already clean)
    Skipping cleaning staging area for part2 (already clean)
    Skipping cleaning build for part2 (already clean)
    Skipping cleaning pulled source for part2 (already clean)
    Skipping cleaning priming area for part1 (already clean)
    Skipping cleaning staging area for part1 (already clean)
    Skipping cleaning build for part1 (already clean)
    Skipping cleaning pulled source for part1 (already clean)

Clean after pull:

     $ snapcraft clean
     Skipping cleaning priming area for part2 (already clean)
     Skipping cleaning staging area for part2 (already clean)
     Skipping cleaning build for part2 (already clean)
     Cleaning pulled source for part2
     Skipping cleaning priming area for part1 (already clean)
     Skipping cleaning staging area for part1 (already clean)
     Skipping cleaning build for part1 (already clean)
     Cleaning pulled source for part1
     Cleaning up parts directory

Clean after build:

    $ snapcraft clean
    Skipping cleaning priming area for part2 (already clean)
    Skipping cleaning staging area for part2 (already clean)
    Cleaning build for part2
    Cleaning pulled source for part2
    Skipping cleaning priming area for part1 (already clean)
    Skipping cleaning staging area for part1 (already clean)
    Cleaning build for part1
    Cleaning pulled source for part1

Clean after stage:

    $ snapcraft clean
    Skipping cleaning priming area for part2 (already clean)
    Cleaning staging area for part2
    Cleaning build for part2
    Cleaning pulled source for part2
    Skipping cleaning priming area for part1 (already clean)
    Cleaning staging area for part1
    Cleaning build for part1
    Cleaning pulled source for part1

Clean after prime:

    $ snapcraft clean
    Cleaning priming area for part2
    Cleaning staging area for part2
    Cleaning build for part2
    Cleaning pulled source for part2
    Cleaning priming area for part1
    Cleaning staging area for part1
    Cleaning build for part1
    Cleaning pulled source for part1
    Cleaning up snapping area

Clean a part:

    $ snapcraft clean part1
    Cleaning priming area for part1
    Cleaning staging area for part1
    Cleaning build for part1
    Cleaning pulled source for part1

Clean a step:

    $ snapcraft clean --step build
    Cleaning priming area for part2
    Cleaning staging area for part2
    Cleaning build for part2
    Cleaning priming area for part1
    Cleaning staging area for part1
    Cleaning build for part1
    Cleaning up snapping area

### snap

Snap in a clean directory:

    $ snapcraft
    Preparing to pull part1
    Pulling part1
    Preparing to pull part2
    Pulling part2
    Preparing to build part1
    Building part1
    Preparing to build part2
    Building part2
    Staging part1
    Staging part2
    Priming part1
    Priming part2
    Snapping 'my-snap' /
    Snapped my-snap_0_amd64.snap

Snap after prime:

    $ snapcraft snap
    Skipping pull part1 (already ran)
    Skipping pull part2 (already ran)
    Skipping build part1 (already ran)
    Skipping build part2 (already ran)
    Skipping stage part1 (already ran)
    Skipping stage part2 (already ran)
    Skipping prime part1 (already ran)
    Skipping prime part2 (already ran)
    Snapping 'my-snap' |
    Snapped my-snap_0_amd64.snap

Snap again:

    $ snapcraft snap
    Skipping pull part1 (already ran)
    Skipping pull part2 (already ran)
    Skipping build part1 (already ran)
    Skipping build part2 (already ran)
    Skipping stage part1 (already ran)
    Skipping stage part2 (already ran)
    Skipping prime part1 (already ran)
    Skipping prime part2 (already ran)
    Snapping 'my-snap' |
    Snapped my-snap_0_amd64.snap

Add a part and snap again:

    $ snapcraft snap
    Preparing to pull part3
    Pulling part3
    Skipping pull part2 (already ran)
    Skipping pull part1 (already ran)
    Preparing to build part3
    Building part3
    Skipping build part2 (already ran)
    Skipping build part1 (already ran)
    Staging part3
    Skipping stage part2 (already ran)
    Skipping stage part1 (already ran)
    Priming part3
    Skipping prime part2 (already ran)
    Skipping prime part1 (already ran)
    Snapping 'my-snap' /
    Snapped my-snap_0_amd64.snap

Snap a directory:

     $ snapcraft snap prime
     Snapping 'my-snap' |
     Snapped my-snap_0_amd64.snap

Snap specifying the output:

    $ snapcraft snap prime --output /tmp/test.snap
    Snapping 'my-snap' /
    Snapped /tmp/test.snap

### cleanbuild

    $ snapcraft cleanbuild
    Creating snapcraft-insupportably-unrelative-annie
    Starting snapcraft-insupportably-unrelative-annie
    Setting up container with project assets
    ./
    ./snapcraft.yaml
    Waiting for a network connection...
    Network connection established
    Hit:1 http://archive.ubuntu.com/ubuntu xenial InRelease
    [...]
    Reading package lists... Done
    Building dependency tree
    Reading state information... Done
    The following additional packages will be installed:
    [...]
    Need to get 14.2 MB of archives.
    After this operation, 64.6 MB of additional disk space will be used.
    Get:1 http://archive.ubuntu.com/ubuntu xenial/main amd64 sgml-base all 1.26+nmu4ubuntu1 [12.5 kB]
    [...]
    Processing triggers for sgml-base (1.26+nmu4ubuntu1) ...
    Downloading parts list|
    Preparing to pull part3
    Pulling part3
    Preparing to pull part2
    Pulling part2
    Preparing to pull part1
    Pulling part1
    Preparing to build part3
    Building part3
    Preparing to build part2
    Building part2
    Preparing to build part1
    Building part1
    Staging part3
    Staging part2
    Staging part1
    Priming part3
    Priming part2
    Priming part1
    Snapping 'my-snap' |
    Snapped my-snap_0_amd64.snap
    Retrieved my-snap_0_amd64.snap

### Full lifecycle in a real project

    $ snapcraft/tour/00-SNAPCRAFT/01-easy-start$ snapcraft
    Preparing to pull gnu-hello
    Pulling gnu-hello
    Downloading 'hello-2.10.tar.gz'[=====================================================] 100%
    Preparing to build gnu-hello
    Building gnu-hello
    ./configure --prefix=
    checking for a BSD-compatible install... /usr/bin/install -c
    [...]
    make -j1
    [...]
    make[1]: Leaving directory '/home/elopio/workspace/canonical/snapcraft/tour/00-SNAPCRAFT/01-easy-start/parts/gnu-hello/build'
    Staging gnu-hello
    Priming gnu-hello
    Snapping 'hello' |
    Snapped hello_2.10_amd64.snap

### Errors

Try to run a command that doesn't exist:

    $ snapcraft idontexist
    Usage:
      snapcraft [options] [--enable-geoip --no-parallel-build]
      snapcraft [options] init
      snapcraft [options] pull [<part> ...]  [--enable-geoip]
      snapcraft [options] build [<part> ...] [--no-parallel-build]
      snapcraft [options] stage [<part> ...]
      snapcraft [options] prime [<part> ...]
      snapcraft [options] strip [<part> ...]
      snapcraft [options] clean [<part> ...] [--step <step>]
      snapcraft [options] snap [<directory> --output <snap-file>]
      snapcraft [options] cleanbuild
      snapcraft [options] login
      snapcraft [options] logout
      snapcraft [options] register <snap-name>
      snapcraft [options] upload <snap-file>
      snapcraft [options] list-plugins
      snapcraft [options] tour [<directory>]
      snapcraft [options] update
      snapcraft [options] define <part-name>
      snapcraft [options] search [<query> ...]
      snapcraft [options] help (topics | <plugin> | <topic>) [--devel]
      snapcraft (-h | --help)
      snapcraft --version

Try to show help for an unexisting topic:

    $ snapcraft help idontexist
    The plugin does not exist. Run `snapcraft list-plugins` to see the available plugins.

Try to initialize the tour in a directory already initialized:

    $ snapcraft tour ialreadyhaveatour
    [Errno 17] File exists: '/tmp/test/ialreadyhaveatour/snapcraft-tour'

Try a lifecycle command without a yaml:

    $ snapcraft
    Could not find snapcraft.yaml.  Are you sure you are in the right directory?
    To start a new project, use 'snapcraft init'

Try to init in a directory with a snapcraft.yaml:

    $ snapcraft init
    snapcraft.yaml already exists!

Try to run a lifecycle command with an unexisting part:

    $ snapcraft pull idontexist
    The part named 'idontexist' is not defined in 'snapcraft.yaml'

Try to clean a part that doesn't exist:

    $ snapcraft clean idontexist
    The part named 'idontexist' is not defined in 'snapcraft.yaml'

Try to clean a step that doesn't exist:

    $ snapcraft clean --step idontexist
    'idontexist' is not a valid step for part 'part3'

Try cleanbuild without lxd:

    $ snapcraft cleanbuild
    The lxd package is not installed, in order to use `cleanbuild` you must install lxd onto your system. Refer to the "Ubuntu Desktop and Ubuntu Server" section on https://linuxcontainers
    .org/lxd/getting-started-cli/#ubuntu-desktop-and-ubuntu-server to enable a proper setup.

Try to snap a directory that doesn't exist:

    $ snapcraft snap idontexist
    [Errno 2] No such file or directory: '/tmp/test/idontexist/meta/snap.yaml'

## Store integration

### login

    $ snapcraft login
    Enter your Ubuntu One SSO credentials.
    Email: u1test@canonical.com
    Password:
    One-time password (just press enter if you don't use two-factor authentication):
    Authenticating against Ubuntu One SSO.
    Login successful.

Login again will just overwrite the existing credentials, if any:

    $ snapcraft login
    Enter your Ubuntu One SSO credentials.
    Email:

### logout

    $ snapcraft logout
    Clearing credentials for Ubuntu One SSO.
    Credentials cleared.

Logout again does the same, no message about credentials not existing:

    $ snapcraft logout
    Clearing credentials for Ubuntu One SSO.
    Credentials cleared.

### register

    $ snapcraft register snap-name
    Registering snap-name.
    Congratulations! You're now the publisher for 'snap-name'.

### push

    $ snapcraft push goodsnap_version_arch.snap
    Uploading goodsnap_version_arch.snap.
    Uploading integration_tests/snaps/basic/u1test20160920_0.1_all.snap [                  ]   0%
    Uploading integration_tests/snaps/basic/u1test20160920_0.1_all.snap [==================] 100%
    Ready to release!|
    Revision 1 of 'goodsnap' created.

### release

    $ snapcraft release pushedsnap 1 edge
    The 'edge' channel is now open.

    Channel    Version    Revision
    ---------  ---------  ----------
    stable     -          -
    candidate  -          -
    beta       -          -
    edge       0.1        1

### Errors

Try a store command without login:

    $ snapcraft register test
    Registering test.
    No valid credentials found. Have you run "snapcraft login"?
    Invalid credentials: [].

Try to register a snap without signing the agreement:

    $ snapcraft register test
    Registering test.
    Expecting value: line 1 column 1 (char 0)

Try to register a snap without developer name or country: todo

Try to register two snaps in a short period:

    $ snapcraft register second-snap
    Registering second-snap.
    You must wait 152 seconds before trying to register your next snap.

Try to register a reserved name:

    $ snapcraft register bash
    Registering bash.
    The name 'bash' is reserved.

    If you are the publisher most users expect for 'bash' then please claim the name at 'https://myapps.developer.ubuntu.com/dev/click-apps/register-name-dispute/?series=16&name=bash'

Try to register a name already registered:

    $ snapcraft register already-registered
    Registering already-registered.
    The name 'already-registered' is already taken.

    We can if needed rename snaps to ensure they match the expectations of most users. If you are the publisher most users expect for 'already-registered' then claim the name at 'https://myapps.developer.ubuntu.com/dev/click-apps/register-name-dispute/?series=16&name=u1test201607183'

Try to register a name with an invalid name:

    $ snapcraft register _invalid   
    Registering _invalid.
    Registration failed.

Try to push a file that doesn’t exist:

    $ snapcraft push idontexist
    The file 'idontexist' does not exist.

Try to push a file that’s not a snap:

    $ snapcraft push imnotasnap
    Uploading imnotasnap.
    Read on filesystem failed because EOF
    Read on filesystem failed because EOF
    Can't find a SQUASHFS superblock on imnotasnap
    Command '['unsquashfs', '-d', '/tmp/tmphedels1u/squashfs-root', 'imnotasnap', '-e', 'meta/snap.yaml']' returned non-zero exit status 1

Try to upload a snap not registered:

    $ snapcraft push notregistered_version_arch.snap
    Uploading notregistered_version_arch.snap.
    Uploading notregistered_version_arch.snap [                                                 ]   0%
    Uploading notregistered_version_arch.snap [=================================================] 100%
    Sorry, try `snapcraft register notregistered` before pushing again.

Upload a snap that requires a manual review:

    $ snapcraft push manualreview_revision_architecture.snap
    Uploading manualreview_revision_architecture.snap.
    Uploading manualreview_revision_architecture.snap [                                      ]   0%
    Uploading manualreview_revision_architecture.snap [======================================] 100%
    Will need manual review...|
    Revision 1 of 'manualreview_revision_architecture' created.
    Publishing checks failed.
    To release this to stable channel please request a review on the snapcraft list.
    Use devmode in the edge or beta channels to disable confinement.

Try to release a snap that doesn't exist:

    $ snapcraft release idontexist 1 edge
    Sorry, try `snapcraft register idontexist` before trying to release or choose an existing revision.

Try to release a revision that doesn't exist:

    $ snapcraft release u1test20160920 10 edge
    Sorry, try `snapcraft register u1test20160920` before trying to release or choose an existing revision.

Try to release using a non-integer revision:

    $ snapcraft release u1test20160920 notanumber edge
    {'revision': ['This field must be an integer.']}

Try to release to a channel that doesn't exist:

    $ snapcraft release idontexist 1 idontexist
    Not a valid channel: idontexist

## Parts cloud

### update

    $ snapcraft update
    Downloading parts list|

Update again:

    $ snapcraft update
    The parts cache is already up to date.

### search

    $ snapcraft search desktop
    PART NAME          DESCRIPTION
    desktop            Helpers for gtk2, gtk3, qt4 and qt5 or glib minimal launchers.
    It bring...
    desktop/glib-only  Helpers for gtk2, gtk3, qt4 and qt5 or glib minimal launchers.
    It bring...
    desktop/qt4        Helpers for gtk2, gtk3, qt4 and qt5 or glib minimal launchers.
    It bring...
    desktop/gtk2       Helpers for gtk2, gtk3, qt4 and qt5 or glib minimal launchers.        It bring...
    desktop/qt5        Helpers for gtk2, gtk3, qt4 and qt5 or glib minimal launchers.        It bring...
    desktop/gtk3       Helpers for gtk2, gtk3, qt4 and qt5 or glib minimal launchers.
    It bring...

### define

    $ snapcraft define desktop/gtk3
    Maintainer: 'Snapcraft community <snapcraft@lists.snapcraft.io>'
    Description: 'Helpers for gtk2, gtk3, qt4 and qt5 or glib minimal launchers.\nIt brings the necessary code and exports for binding and using those\ndesktop technologies in a relocatable fashion, enabling binding with\nglobal desktop theme, icon theme, image caching, fonts, mimetype handlers\napplication global menu and gsettings integration.\nIt also brings basics ubuntu dependency packages.\n\nUsage : \n  1. add "after: [desktop/<technology>]" to your launcher:\n     - gtk2, gtk3, qt4 and qt5 corresponds to their respective toolkit\n       main dependencies and default choices.\n     - glib-only enables to compile mime types and gsettings infos. If you\n       added your own graphical drivers, it will link them as well.\n  2. prepend your command with "desktop-launch", like:\n     commands: "desktop-launch foo" if foo is in $PATH. You can as well\n     specify: "desktop-launch $SNAP/foo".\n  3. add needed plugs to your application:\n     - for graphical application:\n       plugs: [x11 (or unity7 for appmenu integration)]. Think about adding\n       opengl if you need hw acceleration.\n     - if your application needs access to sound:\n       plugs: [pulseaudio]\n     - accessing to user\'s home directory:\n       plugs: [home]\n     - read/write to gsettings:\n       plugs: [gsettings, home]\n       (note that the home plug is needed to read new value)\n'

    desktop/gtk3:
      build-packages:
      - libgtk-3-dev
      make-parameters:
      - FLAVOR=gtk3
      plugin: make
      source: https://github.com/ubuntu/snapcraft-desktop-helpers.git
      source-subdir: gtk
      stage-packages:
      - libxkbcommon0
      - ttf-ubuntu-font-family
      - dmz-cursor-theme
      - light-themes
      - shared-mime-info
      - libgtk-3-0
      - libgdk-pixbuf2.0-0
      - libglib2.0-bin
      - libgtk-3-bin
      - unity-gtk3-module

### Errors

Search for a part that doesn't exist:

    $ snapcraft search idontexist
    No matches found, try to run `snapcraft update` to refresh the remote parts cache.

Try to define a part that doesn't exist:

    $ snapcraft define idontexist
    Cannot find the part name {!r} in the cache. Please consider going to https://wiki.ubuntu.com/snapcraft/parts to add it.
