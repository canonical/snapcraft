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
    plugs:
      - network
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

## Package icon

Providing an icon for your snap is important, even for command-line
applications, if for nothing else than discoverability from management
interfaces such as store fronts like snapweb.

To use an icon to represent the snap, just declare a PNG or SVG in
`snapcraft.yaml` through an `icon` entry with a path relative
to the icon inside the snap.

## Fixed assets

Some metadata is provided in the form of conventions, such as license files,
icons and desktop files among others. For these fixed files to make it into
your final snap they need to be in a `setup` directory at the same level of
your `snapcraft.yaml`.

### Desktop file

The desktop file is the entry point to your snap for end users in GUI
environments.

    setup/gui/<app-name>.desktop
    snapcraft.yaml

Where `<app-name>` is the entry corresponding to `apps` in `snapcraft.yaml`.

As an example, consider the reduced `snapcraft.yaml`:

```yaml
name: my-snap

apps:
    my-app:
        command: my-app.sh
```

Given this `snapcraft.yaml` the desktop file path should be
`setup/gui/my-app.desktop` and the contents should be:

```ini
[Desktop Entry]
Name=My App
Comment=Comment for My App
GenericName=My App
Exec=my-snap.my-app %U
Icon=${SNAP}/meta/gui/icon.png
Type=Application
StartupNotify=true
StartupWMClass=MyApp
Categories=<category>;
MimeType=x-scheme-handler/my-app;
```

[sec]: https://developer.ubuntu.com/snappy/guides/security-policy/
[syntax]: snapcraft-syntax.md
