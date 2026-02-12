.. _tutorial-craft-a-snap:

Craft a snap
============

In this tutorial, we'll build a snap package for a Python app called pyfiglet. The
concepts we'll cover transfer to both simple and complex snaps. We'll cover everything
from creating the build environment and the configuration file, to troubleshooting
missing libraries and finding which interfaces to open.

It should take 20 minutes to complete.

You won't need to come prepared with deep knowledge of software packaging, but
familiarity with Linux paradigms and terminal operations is required.

Once you complete this tutorial, you'll have experience hand-crafting snaps that serves as the basis for further work with creating snaps.


Lesson plan
-----------

Put simply, this tutorial is a run-through of the process of constructing a snap. We
show you how to:

- Start a snap project from scratch
- State the project's essential information
- Define the project source files and main program
- Package the snap
- Modify the build process to enhance the snap's contents
- Share root files with the snap


What we'll work with
--------------------

The object of this tutorial is to package `pyfiglet
<https://github.com/pwaller/pyfiglet>`_ as a personal test snap. It's a lightweight app
for displaying text as ASCII art, and is simple to build and test.

The snap will be named *ukuzama-pyfiglet*, after a fictional user. Throughout this
course, replace *ukuzama* with your own username.


What you'll need
----------------

For this tutorial, you'll need:

- An x64 system running Ubuntu 22.04 or Ubuntu 24.04
- A local user with super user privileges
- 20GB of free storage


Install Snapcraft and LXD
-------------------------

.. admonition:: Before you install

    If you have a Docker installation, you might run into conflicts with LXD over the
    course of this tutorial. As a remedy, you can let Snapcraft build with Multipass
    instead. To do so, start a fresh terminal session and run:

    .. code-block:: bash

        SNAPCRAFT_BUILD_ENVIRONMENT=multipass

    Then, proceed to :ref:`tutorial-craft-a-snap-begin-project`.

Snapcraft is itself available as a snap. Let's begin by installing it. In a terminal,
run:

.. literalinclude:: code/craft-a-snap/task.yaml
    :language: bash
    :start-at: snap install snapcraft --classic
    :end-at: snap install snapcraft --classic
    :dedent: 2

Next, let's add LXD to your system. It functions as the build provider and containerizes
the build environment.

.. literalinclude:: code/craft-a-snap/task.yaml
    :language: bash
    :start-at: snap install lxd
    :end-at: snap install lxd
    :dedent: 2

You also need to add your local user account to the ``lxd`` group so you can access the
tool's resources:

.. literalinclude:: code/craft-a-snap/task.yaml
    :language: bash
    :start-at: sudo usermod -a -G lxd $USER
    :end-at: sudo usermod -a -G lxd $USER
    :dedent: 2

Log out and back in to your account for the new group to become active. Then, check that
you're a member of the group by running:

.. literalinclude:: code/craft-a-snap/task.yaml
    :language: bash
    :start-at: groups $USER
    :end-at: groups $USER
    :dedent: 2

Look for ``lxd`` in the output.

Finally, initialize LXD with a lightweight, default configuration:

.. literalinclude:: code/craft-a-snap/task.yaml
    :language: bash
    :start-at: sudo lxd init --auto
    :end-at: sudo lxd init --auto
    :dedent: 2


.. _tutorial-craft-a-snap-begin-project:

Begin the project
-----------------

Every snap project resides in its own directory. Start by creating one in a development
space on your system:

.. literalinclude:: code/craft-a-snap/task.yaml
    :language: bash
    :start-at: mkdir ukuzama-pyfiglet
    :end-at: cd ukuzama-pyfiglet
    :dedent: 2

A snap is defined in a declarative ``snapcraft.yaml`` file, called the *project file*.
By constructing the file key by key, we'll be building the snap's particulars. Snapcraft
has an ``init`` command that spawns a template project file. Let's start with that:

.. literalinclude:: code/craft-a-snap/task.yaml
    :language: bash
    :start-at: snapcraft init
    :end-at: snapcraft init
    :dedent: 2

From here onward, we'll be working primarily in the project file. Open
``snap/snapcraft.yaml`` in a text editor.

.. _tutorial-craft-a-snap-define-package-information:

Define the package information
------------------------------

The first section inside a snap project file is typically its package information,
sometimes informally referred to as its metadata. This information tells both humans and
machines about the snap, such as its purpose, authors, license, and so on. The comments
in the template describe how to use these keys.

Replace the first four keys with:

.. literalinclude:: code/craft-a-snap/snapcraft.yaml
    :language: yaml
    :caption: snapcraft.yaml
    :start-at: name: ukuzama-pyfiglet
    :end-at: This snap is not endorsed by the pyfiglet project.

Take care *not* to erase the ``grade`` and ``confinement`` keys.

As this is a personal snap, we prepended the project name with a user name. Replace
``ukuzama`` with your own user name. You might encounter other snaps in the Snap Store
with naming that follows this pattern -- it's the recommended format, to keep personal
copies of snaps distinct from their originals.

We didn't alter the ``base`` key, since we want our project to be built on top of the
latest Ubuntu LTS release.

Since we're packaging a project authored by someone else, we ought to respect their
intentions and thinking in the description keys. So, we reused them. The ``summary`` is
a short description with a hard limit of 79 characters, but nothing like that is present
in the pyfiglet README. Instead, we sourced one from the upstream project at `figlet.org
<http://www.figlet.org>`_. The fuller ``description`` key, which has no length limit, is
taken from pyfiglet. We added a disclaimer about endorsement at the end.

.. _tutorial-craft-a-snap-define-the-target-platforms:

Define the target platforms
---------------------------

As we're building on an x64 system, and we're only running basic tests for this
tutorial, we should constrain our snap to only build on the current CPU architecture,
AMD64. At a later point, when you're able to test on other platforms, you could widen
the coverage in this configuration.

Add the ``platform`` key after the project information:

.. literalinclude:: code/craft-a-snap/snapcraft.yaml
    :language: yaml
    :caption: snapcraft.yaml
    :start-at: platforms:
    :end-at: amd64:

With this declaration, Snapcraft will only build the snap on AMD64 machines, for AMD64
machines. Take care to preserve the colon (:) in ``amd64:``.

.. _tutorial-craft-a-snap-define-the-main-part:

Define the main part
--------------------

A *part* is either a piece of software that we want to build from source or a clump of
files. In either case, the purpose of a part is to bundle files from a source into a
snap. Usually they're version-controlled components of the project itself, but you can
aggregate them from a variety of locations.

For the ``parts`` key, add an entry for our main part, the ``pyfiglet`` source code:

.. literalinclude:: code/craft-a-snap/snapcraft.yaml
    :language: yaml
    :caption: snapcraft.yaml
    :start-at: parts:
    :end-at: source: https://github.com/snapcraft-docs/pyfiglet

Parts have three important keys worth discussing.

We set ``plugin`` to ``python`` because pyfiglet is a Python project and we want to
build it from source.

We set ``source-type`` to ``git`` because the project is stored as a Git repository.

We set ``source`` to the remote location of the project. Some software projects take
responsibility for their own snaps, and store their own ``snapcraft.yaml`` file in the
source code. With pyfiglet, we're merely handling the packaging on the project's behalf,
meaning our project file is downstream of and dependent on it. By pointing to a remote
URL, Snapcraft will download the source before it packs the snap.

.. _tutorial-craft-a-snap-pack-the-snap:

Pack the snap
-------------

We have what we need for a basic snap build. Let's see what happens when we pack the
snap:

.. literalinclude:: code/craft-a-snap/task.yaml
    :language: bash
    :start-at: snapcraft pack
    :end-at: snapcraft pack
    :dedent: 2

After a few seconds, the final result is:

.. code-block:: bash

    Packed ukuzama-pyfiglet_0.1_amd64.snap

That means the snap completed successfully. You've already built your first snap!

But we're not done with it or the ``pack`` command yet. You'll find that every time we
iterate on the snap, we repack it.


Inspect the result
------------------

Let's take a look inside the snap to see what happened. At any point in crafting a snap,
we can run an interactive shell inside the build container to inspect what Snapcraft has
done with the files.

There are many steps and actions that go into packing a snap, so for now let's focus on
its final contents before it's compressed into a ``.snap`` file. Let's repack the file
but halt Snapcraft before it finishes:

.. code-block:: bash

    snapcraft pack --shell
    cd ~/prime

Adding ``--shell`` popped us into an interactive shell inside the build container. In
here, we can look around, and even touch files, like we were putting the snap together
by hand.

The ``prime`` directory contains the state of the final files before they're packed. If
we take a look at what's inside, we'll see:

.. code-block:: text

    /root/prime
    ├── bin
    │   ├── Activate.ps1
    │   ├── activate
    │   ├── activate.csh
    │   ├── activate.fish
    │   ├── pip
    │   ├── pip3
    │   ├── pip3.12
    │   ├── pyfiglet
    │   ├── python -> python3
    │   ├── python3 -> /usr/bin/python3.12
    │   ├── python3.12 -> python3
    │   └── wheel
    ├── include
    │   └── python3.12
    ├── lib
    │   └── python3.12
    │       └── site-packages
    │           │
    │           <many dependencies>
    ├── lib64 -> lib
    ├── meta
    │   ├── gui
    │   └── snap.yaml
    └── pyvenv.cfg

For a Python project, there's nothing in here that's out of the ordinary. We see the
constituent pieces, comprising the binary executables for the command itself, the
dependency packages, and some utility files from the build and from Snapcraft. This is
the state of the project as if we had built it ourselves with standard Python tooling.

If we were running into any file pathing problems with the snap, the interactive shell
would be the ideal way to investigate.

Everything looks good, so let's exit the build container:

.. code-block:: bash

    exit


Define the app
--------------

If we were to install the snap we just built, it wouldn't do anything. That's because we
need to define the snap's *apps* -- its programs that run as processes and services on
the host.

In the project file, the ``apps`` key decides all of the snap's apps. Pyfiglet has one
main program -- the ``/bin/pyfiglet`` file we saw earlier.

Add the following after the ``parts`` section:

.. literalinclude:: code/craft-a-snap/snapcraft.yaml
    :language: yaml
    :caption: snapcraft.yaml
    :start-at: apps:
    :end-at: command: bin/pyfiglet

As this is the *main* app -- in other words, the command we want to run when the user
calls the snap by name -- it should match the snap name.

The core of an app entry is its ``command`` key, which is the shell command that the
snap calls on the host. It's a path to an executable inside the snap, and can contain
arguments. It isn't strictly tied to any binary built by the snap. It could instead be,
for example, a combination of POSIX-compatible commands.

.. _tutorial-craft-a-snap-test-the-snap:

Test the snap
-------------

Let's repack the snap and try running it. First, run ``snapcraft pack`` again.

Then, install the snap locally:

.. literalinclude:: code/craft-a-snap/task.yaml
    :language: bash
    :start-at: snap install ukuzama-pyfiglet_0.1_amd64.snap --devmode --dangerous
    :end-at: snap install ukuzama-pyfiglet_0.1_amd64.snap --devmode --dangerous
    :dedent: 2

Normally, snapd prevents us from installing snaps that aren't vetted or confined. But,
if we tell it we're comfortable with installing a snap with full system access and that
doesn't declare itself as stable -- a potentially dangerous decision -- it will install.
While we're crafting and especially debugging snaps, it's easiest to install them with
these flags.

At long last, let's try running our snap.

.. terminal::
    :input: ukuzama-pyfiglet hello, world!
    :user: crafter
    :host: home

     _          _ _                             _     _ _
    | |__   ___| | | ___    __      _____  _ __| | __| | |
    | '_ \ / _ \ | |/ _ \   \ \ /\ / / _ \| '__| |/ _` | |
    | | | |  __/ | | (_) |   \ V  V / (_) | |  | | (_| |_|
    |_| |_|\___|_|_|\___( )   \_/\_/ \___/|_|  |_|\__,_(_)
                        |/

Pyfiglet can draw with different typeface styles, too. It's a fun little command.

.. terminal::
    :input: ukuzama-pyfiglet -f smscript ciao, mondo!
    :user: crafter
    :host: home

     _  o  _,   _              _         _|   _  |
    /   | / |  / \_   /|/|/|  / \_/|/|  / |  / \_|
    \__/|/\/|_/\_/o    | | |_/\_/  | |_/\/|_/\_/ o
                  /


Clean the build container
-------------------------

Before we continue, we should perform some pre-emptive housekeeping.

As we progress through a build, the contents of the build container can become dirty,
and eventually cause conflicts or break the build. It's a good idea to periodically
flush the container for the next build:

.. literalinclude:: code/craft-a-snap/task.yaml
    :language: bash
    :start-at: snapcraft clean
    :end-at: snapcraft clean
    :dedent: 2


Override the main part's build
------------------------------

If you inspect the source files in the pyfiglet source code, you'll notice that the font
files are split into two directories, with the second directory containing all fonts
with unaccounted licenses. The project has a separate Make recipe for combining these
two directories. We committed to the project's Python build, so we can't access this
second set of fonts. Since we're making this snap for personal testing purposes, let's
see if we can preserve all the fonts in the snap.

If we poke around further in the source code, it becomes clear that to copy all the
unaccounted font files we must copy them into the build's ``fonts`` directory. This
means we'll need to intrude on the regular build process of the ``pyfiglet`` part.

Think back to when we entered the build container. While inside, we could have created,
copied, or moved any files as we saw fit. With a build override, we can make manual
adjustments of that sort to the build, but through the project file.

Add the following ``override-build`` key to the ``pyfiglet`` part:

.. literalinclude:: code/craft-a-snap/snapcraft.yaml
    :language: yaml
    :caption: snapcraft.yaml
    :start-at: parts:
    :end-at: craftctl default
    :emphasize-lines: 6-10

The key does what its name suggests. It replaces the regular build step of the part's
lifecycle, running whatever shell commands we provide to special effect. In this case,
the build pre-empts the project's ``setup.py`` script by creating the font directory and
copying both font sets into it at the same time. Then, by concluding with ``craftctl
default``, we instruct the part to proceed with the build step like normal, in our
snap's case by running ``setup.py`` and autotools.

As a result, all the fonts are now copied into the snap. Let's give it a try. Repack the
snap, reinstall it, and then try it with one of the new fonts:

.. terminal::
    :input: ukuzama-pyfiglet -f thin bonjour le monde
    :user: crafter
    :host: home

    |                  o                   |                                |
    |---.,---.,---.    .,---..   .,---.    |    ,---.    ,-.-.,---.,---.,---|,---.
    |   ||   ||   |    ||   ||   ||        |    |---'    | | ||   ||   ||   ||---'
    `---'`---'`   '    |`---'`---'`        `---'`---'    ` ' '`---'`   '`---'`---'
                   `---'


.. _tutorial-craft-a-snap-connect-the-interfaces:

Connect the interfaces
----------------------

Pyfiglet adds some new functionality to FIGlet, such as a feature for installing new
fonts to the user's ``~/.local/share/pyfiglet/fonts`` directory. But, by default, snaps
block access to system resources like USB devices, the network, and home. *Interfaces*
permit access to individual resources on the host, be they software, data, or hardware.

To enable writing to the home directory, we must connect two interfaces:

- The home interface, which provides base access to the home folder
- The personal-files interface, which provides access to hidden files in the home folder

Interfaces are established on apps in your snap by the ``plug`` key.

First, let's connect our ``ukuzama-pyfiglet`` app to the home interface:

.. literalinclude:: code/craft-a-snap/snapcraft.yaml
    :language: yaml
    :caption: snapcraft.yaml
    :start-at: apps:
    :end-at: - home
    :emphasize-lines: 4-5

Next, the personal-files interface. For better confinement, personal-files can only
target specific directories for reading and writing, so we must be explicit and
configure which paths to link. Configurable interfaces must be declared at the root of
the project file, with custom aliases. The alias must be something that users and admins
can intuit. For personal-files, the convention is to start with ``dot-`` and follow with
a short description of our intent.

With all of that in mind, let's create an entry for personal-files after our apps,
granting it write access to ``~/.local/share/pyfiglet/fonts``:

.. literalinclude:: code/craft-a-snap/snapcraft.yaml
    :language: yaml
    :caption: snapcraft.yaml
    :lines: 36-

Then, add it to the plugs of the ``ukuzama-pyfiglet`` app:

.. literalinclude:: code/craft-a-snap/snapcraft.yaml
    :language: yaml
    :caption: snapcraft.yaml
    :start-at: apps:
    :end-at: - dot-pyfiglet-fonts
    :emphasize-lines: 6

If we repack and reinstall the snap, we can install a new font for pyfiglet to use.

However, before we repack, let's go back to two keys we skipped at the beginning.

.. _tutorial-craft-a-snap-secure-the-snap:

Secure the snap
---------------

Now that we're handling interfaces -- usually the last step in the crafting process --
the ``grade`` and ``confinement`` keys are relevant.

These keys account for the security and stability of the snap. The ``grade`` key is a
self-attestation of how risky the snap is. When set to ``devel``, snapd and snap stores
won't treat it as ready for production. The ``confinement`` key determines whether the
snap needs less confinement to function, where it has fewer guardrails and greater
access to the system.

We want our snap to be as safe and secure as possible, so let's change these values to:

.. literalinclude:: code/craft-a-snap/snapcraft.yaml
    :language: yaml
    :caption: snapcraft.yaml
    :start-at: grade: stable
    :end-at: confinement: strict

Now, when we build the snap, the snap's access to the host is inverted -- all sensitive
system resources are blocked unless facilitated by an interface. And when we publish the
snap, users will be able to install it without the ``--devmode`` flag.


Test the Interfaces
-------------------

Now that the snap is confined, we can test the interfaces realistically.

Build and reinstall the snap, but this time, install it like a production-ready snap:

.. literalinclude:: code/craft-a-snap/task.yaml
    :language: bash
    :start-at: snap install ukuzama-pyfiglet_0.1_amd64.snap --dangerous
    :end-at: snap install ukuzama-pyfiglet_0.1_amd64.snap --dangerous
    :dedent: 2

.. note::

    We must continue passing the ``--dangerous`` argument during installation because
    it's not live in the Snap Store, and therefore not attestable.

Next, let's gather a font that wasn't included with pyfiglet and install it.

Download `Small Braille
<https://github.com/xero/figlet-fonts/blob/master/smbraille.tlf>`_ from the figlet-fonts
project and install it with:

.. literalinclude:: code/craft-a-snap/task.yaml
    :language: bash
    :start-at: ukuzama-pyfiglet -L smbraille.tlf
    :end-at: ukuzama-pyfiglet -L smbraille.tlf
    :dedent: 2

And give it a try:

.. terminal::
    :input: ukuzama-pyfiglet -f smbraille hamba kahle
    :user: crafter
    :host: home

    ⣇⡀ ⢀⣀ ⣀⣀  ⣇⡀ ⢀⣀   ⡇⡠ ⢀⣀ ⣇⡀ ⡇ ⢀⡀
    ⠇⠸ ⠣⠼ ⠇⠇⠇ ⠧⠜ ⠣⠼   ⠏⠢ ⠣⠼ ⠇⠸ ⠣ ⠣⠭


Review the project file
-----------------------

Here's the complete code for the ukuzama-pyfiglet project. Yours should look similar to
it.

.. dropdown:: snapcraft.yaml of ukuzama-pyfiglet

    .. literalinclude:: code/craft-a-snap/snapcraft.yaml
        :language: yaml


Conclusion and next steps
-------------------------

And you're done! You have a snap of pyfiglet that works on your system.

It would be a good time to start planning for your first public snap. Ask yourself, what
software would be interesting to package? What apps would benefit the most from the
security and ease of a snap? Any reason or justification is valid. Snaps can be tools,
productivity software, games, or any traditional Linux package.

Take a look on the public `Snap Store <https://snapcraft.io>`_ to see if the apps you
use the most have public snaps. If a result comes up empty, that would be good candidate
for your first public snap.

When you're ready to begin crafting in earnest, you should :ref:`create an account and then register your snap <how-to-register-a-snap>`.
