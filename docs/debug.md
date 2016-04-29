# Debugging

Debugging with Snappy is very similar to debugging on other linux systems.
However there are some differences because of the confinement that are
different from more traditional systems.

## Enabling coredumps
To enable coredumps for the 'hello-word' snap (version 1.0.18) run the
following command:

    $ ulimit -c unlimited
    $ echo "$HOME/snap/hello-world.canonical/<revision>/core.%e.%p.%h.%t" | sudo tee /proc/sys/kernel/core_pattern > /dev/null

Make sure you substitute the pattern above with the right snap name and
revision for the snap you want to inspect. Note that the apparmor profile will
be taken into account, i.e. the segfault can only be written to places that the
snap can write to.

You can customize the core dump pattern with the following options

    %p: pid
    %u: uid
    %g: gid
    %s: signal number
    %t: UNIX time of dump
    %h: hostname
    %e: executable filename

## Important system logs

Snappy Ubuntu system logs are mapped to the classic location of those logs.
The most important log file to look at is:

    /var/log/syslog

The syslog is particularly useful since kernel logs, launcher output, service
status, system logs and confinement violations (these are covered in depth
elsewhere) all get logged there.

## Debugging binaries

### Installing binaries
In order to test a new snap on a Snap-based system you need to install it first.
Installing a local snap like this (as opposed to installing a snap from the
store) is called "sideloading" and it can be done with:

    $ sudo snap install <local-file.snap>

### Binary names on CLI

The name of the binaries will be prefixed with the snap package name. This
ensures that there are never any namespace collision in the binary names. For
example, the snap `pastebinit.mvo` contains a `pastebinit` script. The binary
on disk will be called `pastebinit.pastebinit`.

### Find a binary in the file system hierarchy

All binary names can be found in `/snap/bin/`. The `snap` tool will generate
a small launcher script that ensures that the binary in the snap is called with
the right confinement and environment.

### Testing if a binary is running
After a snap is installed the binaries are available as `<snapname.binaryname>`
on the commandline. It can be tested by simply running it from the commandline. Common issues are that the application tries to read/write outside of its confinement. This will result in permission denied errors even if the app runs
as root. To see if this is the case, the `dmesg | tail` command is helpful. The
errors are of the form:

    [ 8020.798544] audit: type=1400 audit(1442568421.022:9): apparmor="DENIED" operation="open" profile="pastebinit.mvo_pastebinit_1.4.0.0.2" name="/etc/fstab" pid=1123 comm="pastebinit" requested_mask="r" denied_mask="r" fsuid=1000 ouid=0

To find out what paths are available to write the `hello-world` package is
helpful. After installing it the command `hello-world.env` is available that
will show the environment that the snap binaries see. The
`SNAP_USER_DATA` and `SNAP_DATA` contain the directories that the application
is allowed to write to.

## Testing a service

To test a service it must be installed first. See the section "Testing a binary"
for the various ways to do that. Once it is installed, systemd's `systemctl`
command can be used to see if the service starts and runs as expected, for
example:

    systemctl status snap.<name>.<appname>

### Finding the logs

The `journalctl` command can be used to inspect the messages that the service
sends to `stdout`/`stderr`, for example:

    journalctl -u snap.<name>.<appname>

Services may log additional data to syslog (`/var/log/syslog`) or to custom log
directories. Note that custom log directories must be in a path that the service
can write to (usually `SNAP_DATA`).


### Getting a core dump

To enable core dumps you have to configure a place to write them to through
`sysfs`. For instance you can use

    $ echo "/tmp/core.%e.%p" > /proc/sys/kernel/core_pattern

to ensure that your coredumps get written into the `/tmp` directory
regardless of where `CWD` of the process that received a signal was.

## Resolving sandbox issues

### Tools

The Snappy system comes with several tools to assist with understanding and
debugging security policy:

* **snap list**: lists installed snaps
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
at the syslog:

    $ sudo tail -f /var/log/syslog
    audit(1461950701.631:49): apparmor="DENIED" operation="open" profile="hello-world.canonical_sh_1.0.18" name="/etc/ssh/ssh_host_rsa_key" pid=1060 comm="cat" requested_mask="r" denied_mask="r" fsuid=0 ouid=0
    <...>
    audit(1461950702.321:54): auid=1000 uid=0 gid=0 ses=6 pid=1101 comm="ls" exe="/bin/ls" sig=31 arch=c000003e 41(socket) compat=0 ip=0x7f37396140b7 code=0x0

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
      uses the wrong path. For example, using `$SNAP_DATA/foo` instead of
      `$SNAP/foo` (`SNAP_DATA` is non-existent so it would evaluate to
      `/foo` which is disallowed by security policy)
    * hard coded paths in the program. The program should be adjusted ideally
      to understand `SNAP_` variables or use relative paths
* Attempting to read files outside of `SNAP`, `SNAP_DATA` and
  `SNAP_USER_DATA`. This usually happens if your program is looking for
  something that isn't shipped by your snap or it is trying to look for it in
  the wrong place (e.g., typing a `SNAP_` variable, evaluating `PATH` or
  hardcoded path). Fixes are similar to the above.
* Attempting to use `/var/tmp`'. The program should be adjusted to use
  `TMPDIR` or `/tmp`
* Attempting to use `/run`. The program should be adjusted to use
  `SNAP_DATA` or `/run/shm/snaps/SNAP_NAME/SNAP_REVISION`
* Access denied to hardware. Such access is granted via interfaces-- make sure
  you're using the correct one, and log a bug if an interface is missing
  something you need.
* Not specifying the correct interfaces for your snap (e.g., not using
  `network-bind` for server software)
* Trying to execute programs on the system or programs in `/snap/bin`. Except
  for a few common programs (e.g., that are useful for shell programming),
  apps should run programs from their application directory, not the system.
  In addition, apps should not try to run their programs installed in
  `/snap/bin`, but instead simply call them directly from `SNAP/...`
  (executing from `/snap/bin` doesn't work because they use the privileged
  launcher to setup the sandbox, and apps aren't allowed to change their
  sandbox once they start)
* A snap uses `setuid`/`setgid` or `chown` family of syscalls. Ubuntu Core
  16 does not provide a mechanism for assigning users and groups to snaps, so
  the `setuid`/`setgid` and `chown` family of syscalls are blocked (since there
  is no appropriate user to change to. Optionally assigning users/groups to
  snaps is a planned feature). For example,
    * sometimes an existing application is designed to start as root and drop
      privileges to an unprivileged user (e.g., to bind to a port). This
      application will need to be adjusted to not drop privileges (at least
      until Ubuntu Core supports it)
    * the developer is trying to copy files from `SNAP` to
      `SNAP_DATA` (e.g., for write access of a configuration files) and
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
      are loaded there. For example:
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
log messages (rarely)-- if you feel this is the case, reboot and try again).
The rest of this section discusses how to debug sandbox issues for an installed
snap.

If you suspect sandbox issues when running your app, look for denials in
syslog. If you don't see any, it's unlikely that security policy is to blame but
other parts of the system may deny specific accesses. For example:
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
command).
