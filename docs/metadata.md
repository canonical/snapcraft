# Adding metadata

## Adding binaries

Every part may build a binary that may need to be exposed in the system, with
that in mind, the way to expose it would be to use the **binaries** keyword
and declare each binary and its functionality. So, if there were two binaries,
one called *app1* and another called *app2*, the extract would look like

```yaml
binaries:
  app1:
    exec: bin/app1
  app2:
    exec: opt/bin/app2
    security-policy:
      apparmor: app2.apparmor
      seccomp: app2.seccomp
```

So in the example, *app1* will declare its **exec** to the relative path
`bin/app1` and use the standard security template, while *app2* defines a
different **exec** to `opt/bin/app2` and uses customized **security-policy**,
`app2.apparmor` and `app2.seccomp` are relative paths to these security
artifacts within your project. More about security templating can be read
about in the [security section] [sec].

## Adding services

Every part may build a binary that may need to be exposed in the system, with
that in mind, the way to expose a service is by using the **services**
keyword and declare each service and its functionality. So, if there were two
services, one called *service1* and another called *service2*, the extract
would look like

```yaml
services:
  service1:
    start: bin/app1
  service2:
    start: bin/app2
    stop: bin/app2 --stop
    security-policy:
      apparmor: app2.apparmor
      seccomp: app2.seccomp
```

So in the example, *service1* will declare its start to the relative path
`bin/app1` and use the standard security template, while *service2* defines
a different start to `bin/app2`, declares an explicit stop mechanism and uses
customized **security-policy**, `app2.apparmor` and `app2.seccomp` are
relative paths to these security artifacts within your project. More about
security templating can be read about in the [security section] [sec].

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

[sec]: https://developer.ubuntu.com/snappy/guides/security-policy/
