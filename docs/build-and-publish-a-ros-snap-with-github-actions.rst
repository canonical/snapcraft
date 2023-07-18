.. 30605.md

.. _build-and-publish-a-ros-snap-with-github-actions:

Build and Publish a ROS Snap with GitHub Actions
================================================

When deploying a :ref:`robotics application with snap <deploying-robotics-applications>`, keeping the deployment synchronised with development progress is a high priority. It’s best accomplished with a CI/CD pipeline that will automatically deploy your latest developments to your devices.

In this post, we will explore how to automatically build and publish your ROS snap using `GitHub Actions <https://docs.github.com/en/actions>`__, so that your snap always stays up-to-date on the `snapstore <https://snapcraft.io/store>`__.

For this example, we will use a simple ROS2 Foxy application publishing some dummy data, whose source code can be found on `GitHub <https://github.com/Guillaumebeuzeboc/snapped_ros2_pkg>`__. Of course, the same principle can be applied to a ROS package or another release of ROS 2.

Adding a Snap GitHub Action
---------------------------

Our example project is called *snapped_ros2_pkg* and it has a typical ROS 2 package structure:

.. code:: bash

   .
   ├── CMakeLists.txt # Compilation instructions
   ├── config
   │  └── config.yaml # Configuration file
   ├── launch
   │  └── snapped.launch.py # Python ROS 2 launchfile
   ├── package.xml # Dependencies list
   ├── README.md
   ├── snap
   │  └── snapcraft.yaml # Snapcraft entry point
   └── src
      └── snapped_ros2_pkg_node.cpp # Source of the node publishing the dummy data

GitHub’s workflows are located in .github/workflows/ in our project tree:

.. code:: bash

   ├── .github
      └── workflows
         └── snap.yaml

The file ``snap.yaml`` will contain our GitHub workflow dedicated to snap. The GitHub workflow will first build the snap before testing it publishing it to the snap store.

Here is the content of the ``snap.yaml``. Don’t worry, we’ll break this down:

.. code:: yaml

   name: snap
   on:
     push:
       tags:
         - '*'
       branches:
         - main
     pull_request:
       branches:
         - main
     workflow_dispatch:

   jobs:
     build:
       runs-on: ubuntu-latest
       outputs:
         snap-file: ${{ steps.build-snap.outputs.snap }}
       steps:
       - uses: actions/checkout@v3
       - uses: snapcore/action-build@v1
         id: build-snap

       # Make sure the snap is installable
       - run: |
           sudo snap install --dangerous ${{ steps.build-snap.outputs.snap }}
       # Do some testing with the snap
       - run: |
           gbeuzeboc-snapped-ros2-pkg.snapped-ros2-launch --print-description
       - uses: actions/upload-artifact@v3
         with:
           name: gbeuzeboc-snapped-ros2-pkg
           path: ${{ steps.build-snap.outputs.snap }}

     publish:
       if: github.ref == 'refs/heads/main' || startsWith(github.ref, 'refs/tags/')
       runs-on: ubuntu-latest
       needs: build
       steps:
       - uses: actions/download-artifact@v3
         with:
           name: gbeuzeboc-snapped-ros2-pkg
           path: .
       - uses: snapcore/action-publish@v1
         env:
           SNAPCRAFT_STORE_CREDENTIALS: ${{ secrets.STORE_LOGIN }}
         with:
           snap: ${{needs.build.outputs.snap-file}}
           release: ${{ startsWith(github.ref, 'refs/tags/') && 'candidate' || 'edge'}}

This workflow is already in use, and you can see the results `here <https://github.com/Guillaumebeuzeboc/snapped_ros2_pkg/actions>`__.

Workflow conditions
~~~~~~~~~~~~~~~~~~~

A GitHub workflow file starts with its name and the run conditions.

In our case, we will only trigger the workflow on changes on the branch ``main``, a new tag, as well as when a pull request is opened against ``main``. We additionally added a condition (``workflow_dispatch``) to be able to trigger it manually.

.. code:: yaml

   name: snap
   on: # The GitHub Action will happen when:
     push:
       tags:
         - '*' # When a new tag is created
       branches:
         - main # When a commit was created on main
     pull_request:
       branches:
         - main # When a pull request is created on main
     Workflow_dispatch: # When we trigger it manually via the API or the web interface

For more information about triggering a GitHub workflow, see, `”on” documentation <https://docs.github.com/en/actions/using-workflows/workflow-syntax-for-github-actions#on>`__.

Workflow jobs
~~~~~~~~~~~~~

In our workflow, we will define two distinct `jobs <https://docs.github.com/en/enterprise-server@3.3/actions/using-jobs/using-jobs-in-a-workflow>`__. One for building the snap and another for publishing the snap. The idea is that we want to build the snap every time we run the action, but only publish under certain conditions.

Build job
^^^^^^^^^

Before listing the steps of our build, we will define the environment and the output of our build. The output will be used to transfer the result of the build job to the ``publish`` job. This is particularly helpful since you can download the artifact - the snap - from the build webpage.

.. figure:: https://lh3.googleusercontent.com/EmJ-OvK8Mf4YmfBBqDQ0MptdPgzL6t4rE1ldb3r0xRRorVTjAeq4CXHQoB20vUkSq0WrNka4POdK9s8xK9ymN79j6xM-ntWApYpXtybfAeqXnfrJs_0Bm9QXFvTns23xhBPI0AwkDQGnGBsO4g
   :alt: \|309x85


.. code:: yaml

   build:
     runs-on: ubuntu-latest
     outputs:
       snap-file: ${{ steps.build-snap.outputs.snap }}

Here we use the variable ``steps.build-snap.outputs.snap`` that will be defined by a step right after.

For more information about the type of machine to run the job on and outputs, see, `runs-on <https://docs.github.com/en/actions/using-workflows/workflow-syntax-for-github-actions#jobsjob_idruns-on>`__ and `outputs <https://docs.github.com/en/actions/using-workflows/workflow-syntax-for-github-actions#jobsjob_idoutputs>`__.

Checkout step
'''''''''''''

GitHub Action comes with reusable actions for CI/CD provided by GitHub and the community! We are going to use some common ones as well as some that are specific to snap.

The first step is to checkout our current code using the `checkout workflow <https://github.com/marketplace/actions/checkout>`__.

.. code:: yaml

   - uses: actions/checkout@v2

Build the snap
''''''''''''''

In order to build our snap, we will use the `snapcore/action-build <https://github.com/snapcore/action-build>`__ action.

.. code:: yaml

   - uses: snapcore/action-build@v1
     id: build-snap

This will generate our ``.snap`` file and store its name in the variable ``steps.build-snap.outputs.snap``.

Test our snap
'''''''''''''

Now we should make sure that our snap is installable and can run.

.. code:: yaml

   # Make sure the snap is installable
   - run: |
       sudo snap install --dangerous ${{ steps.build-snap.outputs.snap }}
   # Do some testing with the snap
   - run: |
       gbeuzeboc-snapped-ros2-pkg.snapped-ros2-launch --print-description

Here we are reusing the variable ``steps.build-snap.outputs.snap`` to get our snap name and feed it to the snap install command. Then we simply run the ``launchfile`` accompanying our ROS 2 application with the ``--print-description`` option, which only prints the launch description to the console. Which gives us a good indication that the call to our application is fine. Of course, more extensive testing could and should be implemented.

Upload our artifact
'''''''''''''''''''

Now that our snap is generated and tested, we will upload it as an artifact. This is useful, so that we can download it from the GitHub webpage to test it locally. We will also make use of it for the second part of our workflow: publish.

.. code:: yaml

   - uses: actions/upload-artifact@v3
      with:
        name: gbeuzeboc-snapped-ros2-pkg
        path: ${{ steps.build-snap.outputs.snap }}

We are now using the `actions/upload-artifact <https://github.com/marketplace/actions/upload-a-build-artifact>`__. Note that before the steps, we defined snap-file: ``${{ steps.build-snap.outputs.snap }}`` in the output section. Later, we will refer to our uploaded artifact (our snap) as ``snap-file``.

Publish job
~~~~~~~~~~~

This job will be executed only under certain conditions. We only want to publish our snap when there are changes on the ``main`` branch or when we create a new tag. Furthermore, this job can only run when the ‘build’ job is successfully done.

.. code:: yaml

   if: github.ref == 'refs/heads/main' || startsWith(github.ref, 'refs/tags/') # main branch or tagged version
   runs-on: ubuntu-latest
   needs: build # Wait for build job to be done

By adding the ``build`` job in the needs, this job can only happen if the ``build`` job succeed. We will also be able to refer to the uploaded artifact of the ``build`` job.

Download the artifact
^^^^^^^^^^^^^^^^^^^^^

First, we need to get the previously uploaded artifact (our ``.snap`` file) with the `actions/download-artifact <https://github.com/marketplace/actions/download-a-build-artifact>`__ action.

.. code:: yaml

   - uses: actions/download-artifact@v3
        with:
          name: gbeuzeboc-snapped-ros2-pkg
          path: .

We explicitly specify ``path: .`` so that our artifact is stored at the root of our directory, without any extra directory.

Publish the snap
^^^^^^^^^^^^^^^^

Finally, we are publishing our snap with the `snapcore/action-publish <https://github.com/snapcore/action-publish>`__ action. You will have to identify to the `snapstore <https://snapcraft.io/store>`__ in order to publish your snap. Here we refer to the secret ``STORE_LOGIN`` (set in the next section). The snap to upload is referred to by the output of the build job.

Additionally, we want to release to different :term:`risk` levels depending on the situation. If it’s a change on the ``main`` branch we would like to publish to the edge risk level, so edge always has the latest changes which may not be considered as stable. On the other hand, if we are on a tagged version, we would like to publish to the ``candidate`` risk level. After thorough testing, the maintainer can manually promote the snap to ``beta/stable``.

.. code:: yaml

   - uses: snapcore/action-publish@v1
       env:
         SNAPCRAFT_STORE_CREDENTIALS: ${{ secrets.STORE_LOGIN }}
       with:
         snap: ${{needs.build.outputs.snap-file}}
         release: ${{ startsWith(github.ref, 'refs/tags/') && 'candidate' || 'edge'}}

Setting the secret
~~~~~~~~~~~~~~~~~~

The ``action-publish`` is using the secret ``STORE_LOGIN`` so let us define it.

Generate the secret
^^^^^^^^^^^^^^^^^^^

Open your terminal and enter:

.. code:: bash

   snapcraft export-login --snaps=gbeuzeboc-snapped-ros2-pkg --acls package_access,package_push,package_update,package_release exported.txt

Make sure to adjust the ``--snaps=`` to your snap name.

This will prompt you to login and will then generate an ``exported.txt`` file. The content of this file is secret.

Add the secret to your GitHub repository
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To add the secret to the project, we will need to:

-  Go to the “Settings” tab of the repository
-  Choose “Secrets” from the menu on the left and then “Actions”
-  Click on “New repository secret”.
-  Setting the name of the secret as ``STORE_LOGIN``, and paste the contents of ``exported.txt`` as the value.

We are all good to go now. The actions can be checked on the `Actions tab <https://github.com/Guillaumebeuzeboc/snapped_ros2_pkg/actions>`__ of the project. And the published snap can be seen on the `snapstore page <https://snapcraft.io/gbeuzeboc-snapped-ros2-pkg>`__.

You can of course create a more complex release behaviour, making use of all the :term:`channels` features of the snapstore.

A Real world example
--------------------

While the project presented was just an example, one can find real world use of a ROS snap CI/CD workflow. A great example is the `micro-ros-agent <https://github.com/micro-ROS/micro-ROS-Agent/tree/foxy>`__ project. The micro-ros-agent acts as a server between a DDS Network and Micro-ROS nodes inside MCUs. With the `snap GitHub workflow <https://github.com/micro-ROS/micro-ROS-Agent/actions/workflows/snap.yaml>`__ updating the `available snap <https://snapcraft.io/micro-ros-agent>`__, users can stay up to date on the latest developments of the micro-ros-agent.

If you want to give a shot at the micro-ros-agent snap, you can follow the blogpost: `“Getting started with micro-ROS on the Raspberry Pi Pico” <https://ubuntu.com/blog/getting-started-with-micro-ros-on-raspberry-pi-pico>`__.

Summary
-------

We have explored how to build and publish a snap with GitHub Actions. This allows you to easily keep your deployment in sync with your development. With channels, providing simultaneously different versions of your software is made easy. Distributing your application with snap allows you to easily benefit from a production-grade infrastructure while offering a seamless experience to your users.

If you have any feedback or ideas regarding the snapcore GitHub action, or snaps for ROS, please join\ `our forum <https://forum.snapcraft.io/>`__ and let us know what you think.

If you want to learn more about snaps for robotics applications, please visit our `snap documentation <https://snapcraft.io/docs/robotics>`__.
