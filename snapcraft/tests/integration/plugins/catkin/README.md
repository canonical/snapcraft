The catkin tests are a lot slower than the rest of the plugin integration
tests. For that reason, and because we currently don't have a good way to make
a dynamic split of tests to run them in parallel machines, we need to put the
catkin tests in a separate suite.
However, we don't want to create a new suite for every new plugin that we add,
nor we want to maintain the calls to one suite per plugin in all our CI
systems.
So, the ugly hack that we are applying here is to move the catkin tests to a
directory that doesn't have an __init__.py. That way, it will not be run as
part of the snapcraft.tests.integration.plugins suite. In order to run it,
we have to explicitly call snapcraft.tests.integration.plugins.catkin.