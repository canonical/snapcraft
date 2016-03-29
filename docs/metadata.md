# Adding metadata

The entire syntax of a `snapcraft.yaml` with all available keys can be
reviewed in the [snapcraft.yaml syntax section] [syntax]. Here we will
discuss the apps metadata section in more detail.

## Defining app commands

For every app you build in your snap, you can define which commands or
daemons are shipped. Declaring them in `snapcraft.yaml` will expose them in
the system. Starting with the **apps** keyword you specify each app and its
functionality. So, if there were two apps, one called *app1* and another
called *app2*, the extract would look like

```yaml
apps:
  app1:
    command: bin/app1
  app2:
    command: opt/bin/app2
    caps:
      - network-listener
```

So in the example, *app1* will declare its **command** to the relative path
`bin/app1` and use the standard security template, while *app2* defines a
different **command** being `opt/bin/app2` and using the *network-listener*
capability. More about security templating can be read about in the
[security section] [sec].


### The Snapcraft wrapper script explained

It's customary to use within your app small wrappers that will launch the
real binaries. For instance, to select the binaries for the correct
architecture or to set runtime variables such as the application state
directory.

The typical wrapper is a small shell script that sets `PATH`,
`LD_LIBRARY_PATH` or other runtime specific environment variables.

For `PATH` to work properly, it's necessary not to hardcode any pathname in
your code. For instance, don’t rely on `/usr/bin/python` or on
`/usr/bin/java` but instead run `python` or `java`.

While using snapcraft, proper wrappers will be generated for binaries
declared for your app. Snapcraft will also adjust symlinks to be relative
and work for your snap.

It's also a nice touch to add a wrapper script to handle receiving and
dumping the config of your snap as YAML. When run, the config hook reads
your application's configuration and dumps it in YAML format to stdout. When
passed a new config YAML on stdin, the config hook updates your application's
configuration (more about `config` in the section below).


## Adding daemons

Every app may build a binary that may need to be exposed in the system, with
that in mind, the way to expose a daemon is by using the **daemon**
keyword. So, if there were two services, one called *service1* and another
called *service2*, the extract would look like

```yaml
apps:
  daemon1:
    command: bin/app1
    daemon: simple
  daemon2:
    command: bin/app2 --start
    stop-command: bin/app2 --stop
    daemon: forking
```

So in the example, *daemon* will declare its start to the relative path
`bin/app1`, while *daemon2* defines a different start to `bin/app2`: it
declares an explicit stop mechanism.

Also note that *daemon1* is defined as a **simple** daemon, meaning that it
is expected that the command configured is the main process. Daemons like
*daemon2* which use **forking** have the configured command call fork() as
part of its start-up. The parent process is expected to exit when start-up is
complete and all communication channels are set up. The child continues to
run as the main daemon process. This is the behavior of traditional UNIX
daemons.

## Snap config

Snappy config is an entry point to configure your snap, in order to have the
integration with the config system in place, the **config** keyword needs to
be used. This is a relative path to the binary that will manage the
configuration for the snap. If *config* is the part that would handle this
and building its source creates `bin/config`, the extract would look like

```yaml
config: bin/config

parts:
  config:
    plugin: python3
    source: src/config
```

To find out more about config command, have a look at the [config section]
[conf].


## Fixed assets

Some metadata is provided in the form of conventions, such as license files,
icons and desktop files among others. For these fixed files to make it into
your final snap they need to be in a `setup` directory at the same level of
your `snapcraft.yaml`.

### Snap icon

Providing an icon for your snap is important, even for command-line
applications, if for nothing else than discoverability from management
interfaces such as store fronts like webdm.

To use an icon to represent the snap, just drop a PNG or SVG in `setup/gui`
named `icon.png` for the former or `icon.svg` for the latter such that the
(reduced) project tree would look like:

    setup/gui/icon.png
    snapcraft.yaml

or

    setup/gui/icon.svg
    snapcraft.yaml


[conf]: https://developer.ubuntu.com/snappy/guides/config-command/
[sec]: https://developer.ubuntu.com/snappy/guides/security-policy/
[syntax]: snapcraft-syntax.md
