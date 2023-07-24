.. 5125.md

.. _writing-local-plugins:

Writing local plugins
=====================

Snapcraft has a lot of official plugins included by default (with more being added all the time). However, sometimes one wants to write and use one’s own instead, whether that’s because:

-  The official Snapcraft plugins don’t do quite the right thing (in which case one should log a bug)
-  The build system being used isn’t supported by Snapcraft (in which case one should consider submitting one’s plugin upstream once complete)
-  One wants to use custom Debian repositories for ``stage-packages``

Regardless of the reason, the purpose of this document is to provide a simple tutorial for doing it, from scratch.

.. note::
          This feature has been deprecated. Local plugins do not work with Snapcraft 7, although earlier versions of Snapcraft can still be used.

Step 1: Create new project
--------------------------

Get into an empty directory that we’ll use for this project:

::

   $ mkdir -p ~/src/snaps/local-plugins
   $ cd ~/src/snaps/local-plugins

Now create a new snapcraft.yaml:

::

   $ snapcraft init
   Created snap/snapcraft.yaml.
   Edit the file to your liking or run :command:`snapcraft` to get started

Step 2: Create new plugin
-------------------------

Snapcraft plugins are written in Python, so we need to create a new Python module to hold our new plugin. These are placed in the ``snap/plugins/`` directory. Let’s call our plugin “my-plugin”, although the module we create must use an underscore instead of the hyphen:

::

    $ cd ~/src/snaps/local-plugins
    $ mkdir snap/plugins
    $ touch snap/plugins/my_plugin.py

Step 3: Write plugin
--------------------

Each plugin must adhere to the :ref:`Plugin API <snapcraft-plugin-api>`. We won’t discuss it in depth here, but here’s our example:

.. code:: python

   import snapcraft


   class MyPlugin(snapcraft.BasePlugin):

       @classmethod
       def schema(cls):
           schema = super().schema()

           # Add a new property called "my-property"
           schema['properties']['my-property'] = {
               'type': 'string',
           }

           # The "my-option" property is now required
           schema['required'].append('my-property')

           return schema

       def pull(self):
           super().pull()
           print('Pull done. Here is "my-property": {}'.format(
               self.options.my_property))

       def build(self):
           super().build()
           print('Build done.')

Step 4: Use plugin in ``snapcraft.yaml``
----------------------------------------

Now we can use our local plugin just like any other Snapcraft plugin:

.. code:: yaml

   name: my-snap-name # you probably want to 'snapcraft register <name>'
   version: '0.1' # just for humans, typically '1.2+git' or '1.3.2'
   summary: Single-line elevator pitch for your amazing snap # 79 char long summary
   description: |
     This is my-snap's description. You have a paragraph or two to tell the
     most important story about your snap. Keep it under 100 words though,
     we live in tweetspace and your description wants to look good in the snap
     store.

   grade: devel # must be 'stable' to release into candidate/stable channels
   confinement: devmode # use 'strict' once you have the right plugs and slots

   parts:
     my-part:
       plugin: my-plugin
       my-property: test value

Step 5: Build snap
------------------

Now we can build our snap just like normal:

::

   $ snapcraft
   Searching for local plugin for my-plugin
   Preparing to pull my-part
   Pulling my-part
   Look ma, I pulled! Here is "my-property": test value
   Preparing to build my-part
   Building my-part
   Look ma, I built!
   Staging my-part
   Priming my-part
   Snapping 'my-snap-name' |
   Snapped my-snap-name_0.1_amd64.snap

Conclusion
----------

Now you should understand the basics of writing a local plugin that you can keep alongside your ``snapcraft.yaml``. You should try deleting the ``my-property`` specification in the ``snapcraft.yaml``, and you’ll see that you’ll get an error, because we specified that it was required in the plugin.
