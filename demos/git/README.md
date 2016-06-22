This example builds git trunk. It demonstrates the use of the 'home'
plug to allow it access to the users home directory.

After building and installing the snap, you will need to run the following
command to grant it access to your home directory:

```sh
snap connect git:home ubuntu-core:home
```

Only access the home directory is granted. You can see this containment
in action by attempting to initialize a repository in /tmp:

```sh
$ mkdir -p /tmp/foo
$ cd /tmp/foo
$ /snap/bin/git init
fatal: Could not change back to '/tmp/foo': No such file or directory
```

Files repositories in your home directory work fine:
```sh
$ mkdir -p ~/tmp/foo
$ cd ~/tmp/foo
$ /snap/bin/git init
Initialized empty Git repository in /home/stub/tmp/foo/.git/
```
