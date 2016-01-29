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


[conf]: https://developer.ubuntu.com/snappy/guides/config-command/
[sec]: https://developer.ubuntu.com/snappy/guides/security-policy/
[syntax]: snapcraft-syntax.md
