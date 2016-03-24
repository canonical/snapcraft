# Snapcraft parts

Parts are the main building block to create snaps using Snapcraft. Parts have
their own private space and lifecycle. Each part uses a `plugin`, which tells
the part how to behave and what to do with the information inside it.

As seen in the [article about snapcraft.yaml syntax](snapcraft-syntax.md)
parts have general keywords that apply to all of them. In one case, you may
want to enhance your part's functionality using `stage-packages` which end up
bringing Ubuntu deb-based packages into your part, `filesets` to declare
inclusion and exclusion sets, `organize` to make the artifact output for your
part neater, `stage` and `snap` to make certain only the right set of files is
seen at each step (making use of `filesets` or not). An example integrating
these concepts for a part called `example-part` using a hypothetical plugin
called `sample` would look like:

```yaml
parts:
  example-part:
    plugin: sample
    stage-packages:
      - gpg
      - wget
    organize:
      opt/bin: bin
    filesets:
      binaries:
       - bin/*
       - usr/bin/*
      headers:
       - *.h
       - -include
    stage:
      - $binaries
      - test/bin/test_app
      - $headers
    snap:
      - $binaries
```

In this example, imagine that the `sample` plugin actually builds something in
its private *build* location using its private *source* directory as a base,
and that it *installs* the usual set of files from its private install
directory.

This `sample` plugin makes use of `stage-packages`, these packages will be
fetched from the Ubuntu deb archive using the *series* (release, i.e.; trusty,
vivid, wily, ...) that is being used on the host. In this case, the part will
be enhanced by the *gpg* and *wget* deb packages and its necessary
dependencies to work isolated inside the part.

When reaching the *stage* phase, the components in the private part's
*install* directory will be exposed there, but since we used the organize
keyword the contents in the install directory will be exposed to other parts
in a cleaner form if desired or required; it is important to notice that in
the event of using `filesets` they will follow the organized files and not
the internal layout.

The concept of `filesets` basically allows the creation of sets named after
the keywords defined within, in this case *binaries* and *headers*, these are
not necessarily needed but allow for variable expansion in the common
targets: `stage` and `snap`. An inclusion is defined by just listing the
target file, it can be globbed with `*` and a file can be explicitly
excluded by prepending a `-` (when using `*` at the beginning of a path it
needs to be quoted).

The `stage` keyword will replace *$binaries* with all the *binaries* defined
in `filesets`, but it also adds *test/bin/test_app* to the `stage` file set;
*$headers* will basically *include* all the header files except those that
live in *include* as it has a `-` in front of it. These are the files that
will make it to the *stage* directory.

The behavior for `snap` is identical to `stage` with the exception of applying
this in the snap directory, which is the final layout for the snap, this is
where everything should look clean and crisp for a good quality snap.


## Mastering the file system layout

Snaps are typically deployed under `/snaps`, but because this location might
change or multiple installation locations might be used (e.g.
`/pre-installed` and `/snaps`), it's best not to hardcode it: to relate to
resources shipped in your snap, use the `SNAP` environment variable.

The contents of installed snaps is read-only; to store application state and
data, system-wide and per-user directories are made available to snaps: use
the `SNAP_DATA` environment variable to relate to the system-wide state
directory for your snap and the `SNAP_USER_DATA` environment variable for the
per-user directory.

The layout of files under the `SNAP` is up to application authors, but it's
quite common to use `lib/` and `bin/` for application runtime libraries and
binaries, respectively. If your snap isn’t architecture-independent (such as
a script), it's best to use directories qualified with the architecture such
as `lib/arm-linux-gnueabihf/` or `bin/x86_64-linux-gnu/`.

When using Snapcraft, files will follow this recommended layout.

It's a good practice to keep apps' files relocatable, typically by using
relative pathnames and finding libraries relative to the binaries whenever
possible.

Here again, Ubuntu runtimes pulled by snapcraft into your snap - such as
python or openjdk - will load libraries relative to their installed
location, i.e. from within your snap.


## Snapcraft for Python with PIP

Snapcraft includes support for Python 2.x and Python 3.x parts; here's how a
`snapcraft.yaml` parts section will look like:

```yaml
parts:
  spongeshaker:
    plugin: python3
    source: git://github.com/markokr/spongeshaker.git
```

A Python part will typically make sure required Python packages are installed
on the build host and embed the following pieces in your snap:

 * latest Python runtime from the latest Ubuntu packages of your current
   Ubuntu release
 * latest PIP for this Python version as downloaded from PyPy
 * latest versions of your PIP requirements

The proper `PYTHONPATH` environment variable will also be set in the wrapper
scripts generated by snapcraft or when running your app locally.

Python parts support standard snapcraft options and the requirements option
to point PIP at its requirements file.

Why embed a Python runtime? While Snappy does currently include a Python
runtime, this might not be the one you need, and it might be updated to a
different version or removed in a Snappy update. This is why applications
using Python should embed their copy of the Python runtime.


## Snapcraft for Java, Maven or Ant

Snapcraft includes support for building parts with Apache Maven or Ant;
here's how a snapcraft.yaml parts section will look like:

```yaml
parts:
  webapp:
    plugin: maven
    source: git://github.com/lool/snappy-mvn-demo.git
```

A Maven part will typically:

 * make sure the tool is installed on the build host
 * embed a Java runtime in your snap
 * run `mvn package` and copy the resulting `*.jar` and `*.war` files in
   your snaps `jar/` and `war/` directories

An Ant part works similarly, except it runs ant and sets the proper
`CLASSPATH` environment variable in the wrapper scripts generated by
snapcraft or when running the app locally.

If you only need to embed a Java runtime, add a part with the jdk type. This
will pull a relocatable OpenJDK via the default-jdk Ubuntu package and will
set the proper `JAVA_HOME` and `PATH` environment variables in wrapper
scripts generated by snapcraft or when running the app locally.
