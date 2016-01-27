# Debugging

Debugging with Snappy is very similar to debugging on other linux systems.
However there are some differences because of the confinement that are
different from more traditional systems.

## Enabling coredumps
To enable coredumps for the 'hello-word' snap (version 1.0.18) run the
following command:

```
$ sudo -s
# ulimit -c unlimited
# echo "/home/ubuntu/apps/hello-world.canonical/1.0.18/core.%e.%p.%h.%t" > /proc/sys/kernel/core_pattern
```

Make sure you substitute the pattern above with the right snap name and
version for the snap you want to inspect. Note that the apparmor profile will
be taken into account, the segfault can only be written to places that the
snap can write to.

You can customize the core dump pattern with the following options

    %p: pid
    %u: uid
    %g: gid
    %s: signal number
    %t: UNIX time of dump
    %h: hostname
    %e: executable filename

## Debugging tools

There is a debug snap that can be installed with:

    $ sudo snappy install snappy-debug

As of 2016-01-26, the snappy-debug snap only contains the
**snappy-debug.security** tool for working with snappy security policy.

*Note:* It is planned to add the following debugging tools:

* `snappy-debug.gdb`: a source-level debugger
* `snappy-debug.strace`: a syscall call tracer
* `snappy-debug.ltrace`: a library call tracer

This document will explain `gdb`, `strace`, `ltrace` even though they are not
available as part of the package yet, because they are generally useful for
debugging.

The reason to put these tools in a snap separate from the base image was
because they are not needed on production devices.


### gdb

The gdb debugger is useful to debug crashes of application or libraries
written in compiled languages. In order to get most out of gdb you should
build your application with debug symbols. If you are using the gcc compiler,
it is recommended to do the debug builds with the following
'CFLAGS= "-g3 -O0"'. This will enable full debug symbols and disable all
optimizations.

The most important commands for debugging with the gdb debugger are the
`break`, `next` , `run` and `backtrace` commands. The gdb debugger supports a
 whole range of commands and options that are out of scope for this manual.

To launch `gdb` run it with the program or the coredump as the first argument.

The `break` command can be used to set a breakpoint to a specific line in the
program. An example could be `(gdb) break lala.c:2`. To start running the
program, use the `run` command. The program will stop once it reaches that
line and can be traced with the `step` and `next` commands. Alternatively the
operation of the program can be resumed via the `continue` command. If the
program crashes gdb will stop and the `backtrace` command can be used to
inspect what happened before the program crashed.

Here is an example session:

```
snappy-debug.gdb application --args additional arguments
(gdb) run
Program received signal SIGSEGV, Segmentation fault.
0x000000000040053f in main () at lala.c:4
4        printf("%i", *(int*)0);
(gdb) backtrace
#0  0x000000000040053f in main () at lala.c:4
```

### strace

The strace tool can be used to inspect what system calls an application
performs.

The command output looks like this:

```
$ snappy-debug.strace cat /etc/shadow
...
open("/etc/shadow", O_RDONLY)           = -1 EACCES (Permission denied)
...
```

This indicates that the open system call failed with the given error number.

### ltrace

The ltrace tool is similar to strace. The difference is that it tracks
library calls instead of system calls.

The command output looks like this:

```
$ snappy-debug.ltrace /etc/shadow
__libc_start_main(0x401a40, 2, 0x7ffc27e9f058, 0x408b40 <unfinished ...>
getpagesize()                                    = 4096
strrchr("cat", '/')                              = nil
setlocale(LC_ALL, "")                            = "en_US.UTF-8"
bindtextdomain("coreutils", "/usr/share/locale") = "/usr/share/locale"
textdomain("coreutils")                          = "coreutils"
__cxa_atexit(0x402ac0, 0, 0, 0x736c6974756572)   = 0
getopt_long(2, 0x7ffc27e9f058, "benstuvAET", 0x409280, nil) = -1
__fxstat(1, 1, 0x7ffc27e9eea0)                   = 0
open("/etc/shadow", 0, 0400000)                  = -1
__errno_location()                               = 0x7f1ed2527690
```

## Important system logs

Snappy Ubuntu system logs are mapped to the classic location of those logs.
The most important log file to look at is:

    /var/log/syslog

The syslog is particularly useful since kernel logs, launcher output, service
status, system logs and confinement violations (these are covered in depth
elsewhere) all get logged there.

Further, systemd has log dump facility to look at all system and snap
services. The "snappy service" command provides an easy way to monitor and
inspect snap services. To get the status of all snap services run "snappy
service status". To inspect the logs run "snappy service logs". Both commands
can take a snap name to limit to a specific snap (e.g. for the service
in `shout.sergiusens` run: `sudo snappy service logs shout`).


## Debugging binaries

### Installing binaries
In order to test a new snap on a Snappy system you need to install it first.
This is called sideloading and it can be done via
`snappy-remote --url=ssh://ubuntu@webdm.local/ install snapname.snap`.
Alternatively the snap can be copied via scp into the Snappy system and
installed via `sudo snappy install snapname.snap`.

Note that you have to use the `--allow-unauthenticated` tag for installing
unsigned snaps: `sudo snappy install --allow-unauthenticated snapname.snap`.

### Binary names on CLI

The name of the binaries will be prefixed with the snap package name. This
ensures that there are never any namespace collision in the binary names. For
example, the snap `pastebinit.mvo` contains a `pastebinit` script. The binary
on disk will be called `pastebinit.pastebinit`.

### Find a binary in the file system hierarchy

All binary names can be found in `/snaps/bin/`. The snappy tool will generate
small wrapper script that ensures that the binary in the snap is called with
the right confinement and environment.

### Testing if a binary is running
After a snap is installed the binaries are available as
`<snapname.binaryname>` on the commandline. It can be tested by simply
running it from the commandline. Common issues are that the application tries
to read/write outside of its confinement. This will result in permission
denied errors even if the app runs as root. To see if this is the case, the
`dmesg | tail` command is helpful. The errors are of the form:

```
[ 8020.798544] audit: type=1400 audit(1442568421.022:9): apparmor="DENIED" operation="open" profile="pastebinit.mvo_pastebinit_1.4.0.0.2" name="/etc/fstab" pid=1123 comm="pastebinit" requested_mask="r" denied_mask="r" fsuid=1000 ouid=0
```

To find out what paths are available to write the `hello-world` package is
helpful. After installing it the command `hello-world.env` is available that
will show the environment that the snap binaries see. The
`SNAP_USER_DATA` and `SNAP_DATA` contain the directories that the application
is allowed to write to.

### Tracing a binary

In order to trace what a binary is doing the usual tools like gdb and strace
can be used. See the section above about debugging about for details how to
get the debug snap.

## Testing a service

To test a service it must be installed first. See the section "Testing a
binary" for the various ways to do that. Once it is installed the
`snappy service status` command can be used to see if the service starts and
runs as expected. The `snappy service logs` command is available to inspect
the messages that the service send to `stdout`/`stderr`.

### Finding the logs

To see the log output of a daemon the `snappy service logs` command can be
used. The same logs are available via the `systemctl`/`journalctl` low-level
commands. Services may log additional data to syslog (`/var/log/syslog`) or
to custom log directories. Note that custom log directories must be in a path
that the service can write to (usually `SNAP_DATA`).

### Reading the logs

The `snappy service logs` command should be used to read the logs. The
standard unix tools like `less`/`more`/`grep` are available for inspecting
service specific logs that are stores in `SNAP_DATA`.


### Getting a core dump

To enable core dumps you have to configure a place to write them to through
`sysfs`. For instance you can use

    echo "/tmp/core.%e.%p" > /proc/sys/kernel/core_pattern

to ensure that your coredumps get written into the `/tmp` directory
regardless of where `CWD` of the process that received a signal was.

## Resolving sandbox issues

### Tools

The Snappy system comes with several tools to assist with understanding and
debugging security policy:

* **snappy-debug.security list**: used for listing available policy
* **snappy-debug.security scanlog**: used for scanning /var/log/syslog for
  policy violations in an easier to read format
* **snappy list**: lists installed snaps
* **snappy service**: tool to manipulate services and view logs. See `sudo
  snappy service` for details
* **sudo aa-status**: shows AppArmor policy that is loaded in the kernel

In addition to the above, it is sometimes useful to work with the raw syslog
output:

    $ sudo grep audit /var/log/syslog
    $ sudo tail -f /var/log/syslog | grep audit

`scmp_sys_resolver` can be used to resolve syscall numbers to the common
name. It must be used on the target device. Example:

    $ scmp_sys_resolver 41
    socket

### Symptoms

Common symptoms indicating there is a problem with the security policy of your
app include:

* app crashes
* app isn't operating correctly
* app can't write to files
* app doesn't start

It is easy to see if the app is being blocked by security policy by looking
at the logs (be sure to run `sudo snappy install snappy-debug` first):

```
$ sudo snappy-debug.security scanlog
= AppArmor =
Time: Oct  1 15:21:53
Log: apparmor="DENIED" operation="open" profile="hello-world.canonical_sh_1.0.18" name="/etc/ssh/ssh_host_rsa_key" pid=1060 comm="cat" requested_mask="r" denied_mask="r" fsuid=0 ouid=0

= Seccomp =
Time: Oct  1 15:27:43
Log: auid=1000 uid=0 gid=0 ses=6 pid=1101 comm="ls" exe="/bin/ls" sig=31 arch=c000003e 41(socket) compat=0 ip=0x7f37396140b7 code=0x0
Syscall: socket
```

Notice in the second line that a process running under the
`hello-world.canonical_sh_1.0.18` AppArmor label
(`'profile="hello-world.canonical_sh_1.0.18"'`) was denied access
(`'apparmor="DENIED"'`) to open `/etc/ssh/ssh_host_rsa_key`.

An AppArmor sandbox denial doesn't necessarily mean there is a problem with
the app-- a well-written app that is portable might check to see if certain
things are available to it which might trigger a denial, but fail gracefully
and continue running without issue.

A `seccomp` sandbox denial is fatal to the process encountering it and a
process attempting to use a Linux syscall not listed in its syscall filter
(ie, its `seccomp` policy) will be terminated via a `SIGSYS` signal sent by
the kernel. Apps are not allowed to change their `seccomp` policy or use a
more lenient `seccomp` policy for their child processes. If a child is sent
`SIGSYS`, the parent may continue running but if the parent receives
`SIGSYS`, it and all of its children will be terminated.

### Common problems

The most common culprits for sandbox issues are:

* Attempting to write to files outside of `SNAP_DATA` and `SNAP_USER_DATA`.
  This is a very common problem for apps that are being ported to Ubuntu Core
  or packaged as snaps for the first time. Typical issues might include:
    * writing to files in the read only `SNAP`. Fix this by writing
      to `SNAP_DATA` for `services` and either `SNAP_DATA` or
      `SNAP_USER_DATA` for `binaries`.
    * writing to system directories such as `/var/log` and `/run`. Adjust your
      app to write to `$SNAP_DATA/run` or `$SNAP_DATA/log`
      or similar
    * improperly evaluating or typoing `SNAP_` variables so the program
      uses the wrong path. Eg, using `$SNAP_DATA/foo` instead of
      `$SNAP/foo` (`SNAP_DATA` is non-existent so it would evaluate to
      `/foo` which is disallowed by security policy)
    * hard coded paths in the program. The program should be adjusted ideally
      to understand `SNAP_` variables or use relative paths
* Attempting to read files outside of `SNAP`, `SNAP_DATA` and
  `SNAP_USER`. This usually happens if your program is looking for something
  that isn't shipped by your snap or it is trying to look for it in the wrong
  place (eg, typing a `SNAP_` variable, evaluating `PATH` or hardcoded path).
  Fixes are similar to the above
* Attempting to use `/var/tmp`'. The program should be adjusted to use
  `TMPDIR` or `/tmp`
* Attempting to use `/run`. The program should be adjusted to use
  `SNAP_DATA` or `/run/shm/snaps/SNAP_FULLNAME/SNAP_VERSION`
* Access denied to hardware. To fix for development, use
  `snappy hw-assign <name> /dev/...`. When ready for production, consider
  using a gadget snap.
* Not specifying the correct `caps` for your snap (eg, not using
  `network-service` for server software or declaring the cap to use a
  particular framework service)
* Trying to execute programs on the system or programs in `/snaps/bin`. Except
  for a few common programs (eg, that are useful for shell programming),
  apps should run programs from their application directory, not the system.
  In addition, apps should not try to run their programs installed in
  `/snaps/bin`, but instead simply call them directly from `SNAP/...`
  (executing from `/snaps/bin` doesn't work because they use the privileged
  launcher to setup the sandbox, and apps aren't allowed to change their
  sandbox once they start)
* A snap uses `setuid`/`setgid` or `chown` family of syscalls. Ubuntu Core
  15.04 does not provide a mechanism of assigning users and groups to snaps,
  so the `setuid`/`setgid` and `chown` family of syscalls are blocked (since
  there is no appropriate user to change to. Optionally assigning
  users/groups to snaps is a planned feature). For example,
    * sometimes an existing application is designed to start as root and drop
      privileges to an unprivileged user (eg, to bind to a port). This
      applications will need to be adjusted to not drop privileges (at least
      until Ubuntu Core supports it)
    * the developer is trying to copy files from `SNAP` to
      `SNAP_DATA` (eg, for write access of a configuration files) and
      attempts to use a `cp -a`. This results in a `seccomp` failure when the
      command is run as root because '-a' attempts to copy the ownership
      (`chown`) of the files in `SNAP`, but these are owned by an
      unprivileged system user. To remedy, ship the files `SNAP` as
      world-readable, then use `cp -r --preserve=mode` to copy them instead,
      and optionally adjust the permissions with chmod after copying
    * the snap ships a `setuid`/`setgid` application. This is currently not
      supported and security policy will block their use.
* A snap performs privileged operations that require Linux `capabilities(7)`
  not granted by the default security policy. This can happen for a number of
  reasons:
    * sometimes the app or a dependent library is trying to read a sensitive
      file in `/proc` or `/sys` that it doesn't actually need to properly
      function. The app should be made to either not access the file or to
      fail gracefully in the event of a permission denied error
      *Note:* the kernel may report a spurious `net_admin denial` that can
      most likely be ignored (unless your app is modifying routing tables,
      firewall rules, etc)
    * some apps require certain kernel functionality to be present and might
      attempt to load kernel modules. Because loading arbitrary kernel modules
      would allow the app to escape confinement, this is not allowed by
      security policy. Use `snappy config ubuntu-core` and add/adjust the
      `load-kernel-modules` line accordingly when developing your snap. When
      ready for production with a gadget snap, make sure the modules you need
      are loaded there. Eg:
      ```yaml
      config:
        ubuntu-core:
          ...
          load-kernel-modules: [ module1, module2, ... ]
      ```

#### Porting applications

When porting an existing app to Ubuntu Core, it is important to understand
the snappy FHS and how security policy is implemented. In general, to work
within the security policy template system, you will want to make sure your
app:

* can be made to work within the application-specific directories
* does not try to change ownership of files
* does not change user/group
* does not try to setup a more lenient `seccomp` filter than that provided by
  the policy

### Debugging
When resolving sandbox issues, the first thing you should do is disable
kernel rate limiting, otherwise the kernel may choose to not log important
information needed for debugging (even with this, the kernel may still drop
log messages (rarely)-- if you feel this is the case, reboot and try again):

    $ sudo snappy install snappy-debug
    $ sudo snappy-debug.security disable-rate-limiting

This rest of this section discusses how to debug sandbox issues for an
installed snap.

If you suspect sandbox issues when running your app, simply use the
`snappy-debug.security scanlog` tool. If it doesn't report any issues, it is
unlikely that security policy is to blame but other parts of the system may
deny specific accesses. Eg:
* traditional UNIX permissions are in place. If you get a permission denied
  error with nothing in the logs, be sure to check the permissions on the
  file with `stat <file>`
* Linux capabilities are enforced. Some permission denied errors are a result
  of capabilities checks. For example, a non-root user will not be able to
  change routing tables even if the security policy allows `CAP_NET_ADMIN`
* Mount options are enforced. Some filesystems may be mounted read-only or
  `noexec`. Much of the Ubuntu Core root filesystem is mounted read only
  which is one reason why apps need to adhere to the snappy FHS.
* Hardware access to devices outside of the devices cgroup will not show up
  as denials. You can see the devices in a particular running snap's cgroup
  with:
  ```
  $ sudo cat /sys/fs/cgroup/devices/snappy.<name>.<origin>/devices.list
  ```

Keep in mind that the cgroup will only exist while the program is running, so
short-running `binaries` won't have the above cgroups entry (for debugging
cgroups, it might be helpful for the binary to drop to a shell or run a sleep
command)

It is very convenient for debugging to login to your device and launch
`snappy-debug.security scanlog`. Then in another console login, use your
snap. For example, in one console:

```
$ sudo snappy install snappy-debug
$ sudo snappy-debug.security scanlog
```

Now, login to another console and try to start a service:

```
$ sudo snappy service start xkcd-webserver
$ sudo snappy service status xkcd-webserver
Snap        Service        State
xkcd-webserver    xkcd-webserver    enabled; loaded; failed (failed)
```

We can see that the service failed to start. Let's look back at the console
running `snappy-debug.security scanlog`:

```
= Seccomp =
Time: Oct  1 17:03:30
Log: auid=4294967295 uid=0 gid=0 ses=4294967295 pid=2409 comm="xkcd-webserver" exe="/usr/bin/python3.4" sig=31 arch=c000003e 54(setsockopt) compat=0 ip=0x7f4aebf0d05a code=0x0
Syscall: setsockopt
Recommendation:
* add 'setsockopt' to seccomp policy
* add one of 'network-client, network-firewall, network-service' to 'caps'
* add 'setsockopt' to seccomp file in 'security-policy'
```

Ah, `xkcd-webserver` is a web server and we apparently forgot to include the
`network-service` in our 'caps' in the yaml. At this point, you could simply
add it to the `caps` for that service, rebuild the snap, remove the old snap
and install the new one. Eg, after adding the `network-service` cap:

```
$ sudo snappy service start xkcd-webserver
$ sudo snappy service status xkcd-webserver
Snap        Service        State
xkcd-webserver    xkcd-webserver    enabled; loaded; active (running)
```

For simple things like forgetting a cap, rebuilding and reinstalling the snap
is enough. Other times you might be developing custom policy for a specialized
snap or want to simply allow some accesses temporarily. In these cases it is
usually easier to modify policy in place on the device to get everything
working (and if working on custom policy, copying this back to your packaging
files).

IMPORTANT: whether you are using templated policy, `security-override` or
`security-policy`, the actual security policy that is applied at runtime is
autogenerated during snappy install based on the snap's packaging. On Ubuntu
Core 15.04, the autogenerated policy for AppArmor is found in
`/var/lib/apparmor/profiles` and the autogenerated policy for seccomp is in
`/var/lib/snappy/seccomp/profiles`.

Now let's walk through a couple of real world examples. Consider this in the
snappy packaging:

```yaml
name: foo
version: 1.0
vendor: Some One <some.one@example.com>
icon: meta/hello.png
binaries:
 - name: bin/bar
   caps: []
```

Now consider this output:

```
$ sudo snappy-debug.security scanlog
= AppArmor =
Time: Oct  2 03:20:49
Log: apparmor="DENIED" operation="mknod" profile="foo.sideload_bar_ICKPCGbSJVMW" name="/snaps/foo.sideload/ICKPCGbSJVMW/stamp-file" pid=2545 comm="touch" requested_mask="c" denied_mask="c" fsuid=1000 ouid=1000
File: /snaps/foo.sideload/ICKPCGbSJVMW/stamp-file (write)
Suggestion:
* adjust program to not write to SNAP
```

`snappy-debug.security scanlog`' conveniently is suggesting that the app is
trying to write to the install directory (`SNAP`). Looking at
`/snaps/foo.sideload/ICKPCGbSJVMW/bin/bar`, there is:

```sh
#!/bin/bash -e
touch ./stamp-file
```

Quickly adjust that to be:

```sh
#!/bin/bash -e
touch $SNAP_DATA/stamp-file
```

then run the app again. The logs don't show the stamp-file denial (so now
would be a good time to apply this change to `bin/bar` on your development
machine), but do show a new denial:

```
= AppArmor =
Time: Oct  2 03:30:35
Log: apparmor="DENIED" operation="open" profile="foo.sideload_bar_ICKPCGbSJVMW" name="/etc/motd" pid=2582 comm="cat" requested_mask="r" denied_mask="r" fsuid=0 ouid=0
File: /etc/motd (read)
Suggestions:
* adjust program to read necessary files from SNAP
* add '/etc/motd r,' to apparmor in 'security-policy'
```

Let's temporarily add the suggested rule to
`/var/lib/apparmor/profiles/*_foo.sideload_bar_*` (before the trailing `}`):

```
...
profile "foo.sideload_bar_ICKPCGbSJVMW" (attach_disconnected) {
  #include <abstractions/base>
  ...
  /etc/motd r
}
```

Now, reload the policy:

```
$ sudo snappy-debug.security reload foo.sideload
Reloading foo.sideload_bar_ICKPCGbSJVMW ...
AppArmor parser error for ...
```

Whoops, we forgot the trailing comma in the AppArmor rule. Adjust it to be:

```
  ...
  /etc/motd r,
}
```

Now, reload the policy again:

```
$ sudo snappy-debug.security reload foo.sideload
Reloading foo.sideload_bar_ICKPCGbSJVMW ...
```

Now, rerun the app and look at the logs:

```
= Seccomp =
Time: Oct  2 03:48:45
Log: auid=1000 uid=0 gid=0 ses=32 pid=2787 comm="xtables-multi" exe="/var/lib/apps/foo.sideload/ICKPCGbSJVMW/xtables-multi" sig=31 arch=c000003e 49(bind) compat=0 ip=0x7fcf76dbfc07 code=0x0
Syscall: bind
Suggestions:
* add 'bind' to seccomp policy
* add one of 'hello-dbus-fwk_client, network-admin, network-client, network-firewall, network-service, network-status, snapd' to 'caps'
* add 'bind' to seccomp file in 'security-policy'
```

Let's temporarily add this sycall to
`/var/lib/snappy/seccomp/profiles/foo.sideload_bar_*`:

```
...
pwrite
pwrite64
pwritev
# temporary
bind
```

Modifying the seccomp policy in this manner does not require a separate
reload step (because the launcher will handle this for us). Rerun the app
again and check the logs:

```
= AppArmor =
Time: Oct  2 03:56:24
Log: apparmor="DENIED" operation="bind" profile="foo.sideload_bar_ICKPCGbSJVMW" pid=2826 comm="xtables-multi" family="unix" sock_type="stream" protocol=0 requested_mask="bind" denied_mask="bind" addr="@xtables"
Suggestion:
* add 'unix addr="@xtables",' to apparmor in 'security-policy'
* add one of 'network-firewall' to 'caps'
```

Now temporarily add a unix rule to
`/var/lib/apparmor/profiles/*_foo.sideload_bar_*`:

    unix addr="@xtables",

As can be seen from the above example, adding temporary policy or developing
custom policy is an iterative process and can take some effort, but the steps
are not complicated:

* run the app
* check the logs
* add any necessary rules
* repeat

Once there are no denials on app start, exercise the app fully and continue
watching the logs for new denials, updating and reloading the policy until
you have all the temporary accesses you need. These temporary accesses might
be sufficient while developing your snap until you are able to remove the
need for these accesses (while these changes will survive a reboot, they will
be lost on app reinstall or upgrades). If you require the additional rules,
use `security-policy` in your yaml and follow the guidelines in
'Developing customized security policy', above, start with boilerplate policy
and copy these new rules in, and retest. Remember that store policies may
trigger a manual review for uploads of snaps specifying `security-policy`.
Notice in many of the above logs, the recommendation was to use various
`caps`. It is recommended that templated policy with caps be used whenever
possible rather than generating your own custom policy.
