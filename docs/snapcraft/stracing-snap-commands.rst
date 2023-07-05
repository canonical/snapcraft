.. 1433.md

.. _stracing-snap-commands:

Stracing snap commands
======================

Using ``strace`` with snaps is sometimes frustrating because of how snap commands are launched and the resulting flow of execution. Here is a high-level overview:

-  ``/snap/bin/hello-world`` is a symlink to ``/usr/bin/snap``
-  when ``/snap/bin/hello-world`` is run, the ``snap`` command looks at ``argv[0]`` to determine what snap the command belongs to, it then sets up the snap’s environment based on that and then calls ``snap-confine`` with appropriate arguments for running the command under confinement
-  ``snap-confine`` examines its arguments and launches the snap command under whatever confinement is available using ``execv()`` via ``snap-exec``

As such, something along the lines of the following is what most people try:

::

   $ strace -f -D -vv -o ./hello-world.trace /snap/bin/hello-world.env
   need to run as root or suid

Because ``snap-confine`` is setuid root, ``strace`` is saying that you must run it as root[1]. You may of course give the ``-u`` option to have ``strace`` drop privileges to the user. Eg:

::

   $ sudo strace -u <your username> -f -D -vv -o ./hello-world.trace /snap/bin/hello-world

What is interesting and annoying is that most of the time, the above command will hang and this has to do with stracing a setuid executable. One day I decided to ``strace`` the command out of /snap/bin and running the command directly out of ``/snap/<snap>/current/bin`` and compared the output and I found that certain syscalls hang such that if I remove them using the ``-e`` option, I could get reliable straces. Now the invocation is:

::

   $ sudo strace -u <your username> -e '!select,pselect6,_newselect,clock_gettime,sigaltstack,gettid,gettimeofday,nanosleep' -f -D -vv -o ./hello-world.trace /snap/bin/hello-world
   Hello World!

The above is known to work on amd64 and armhf, so it’s possible other calls should be removed (please comment in this post if you find others that should be omitted). Of course, you can also use ``-e`` to specify only the syscalls you want and so long as you don’t include the problematic syscalls, it should work fine. Eg:

::

   $ sudo strace -u <your username> -e 'open,mmap,mmap2' -f -D -vv -o ./hello-world.trace  /snap/bin/hello-world

Hopefully this will help you debug your snaps. Happy stracing!

[1] You might also be interested how the kernel might limit who can ptrace what. Many systems these days limit the use of ptrace to processes of the calling user. This can be temporarily adjusted using the ``kernel.yama.ptrace_scope`` sysctl. Eg:

::

   $ sysctl kernel.yama.ptrace_scope  # show current value
   kernel.yama.ptrace_scope = 1
   $ sudo sysctl -w kernel.yama.ptrace_scope=0 # disable ptrace_scope
   kernel.yama.ptrace_scope = 0
