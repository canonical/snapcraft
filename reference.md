# Snapcraft CLI reference

## version

## help

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
