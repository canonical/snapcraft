# Proposed Yaml

This is NOT blessed yaml, but rather changes to the blessed yaml that are proposed by the snapcraft team.

## Plugin yaml

### No name

We don't need the name field, it is redundant with the filename.

### Specify name of python module

Add an optional 'module' field with the name of the python import to use for this plugin (if not specified, is the same as the yaml name).  But often plugins will use characters (like . or -) that can't be used in a python module name.  So we need a way to point at a module name.

### Allow config of required parts

"requires" should be a yaml subsection instead of a list of strings, so that configuration could be done.

**But what if that configuration conflicts with configuration of that same part in snapcraft.yaml?**

### Do we need resources?

Do we need this in the yaml?  Or just as part of the plugin code?  It isn't configuration.

### Add packages field

Add a "packages" field that installs Ubuntu packages into the host.

## snapcraft.yaml

### packages

This should be a list of packages to install into the host, rather than to include in the deb.  And renamed to system_packages to avoid confusion.

Installing a package into the deb should be a part, just like every other piece of code that goes in there.  Something like:

parts:
  ubuntu:
    packages:
     - fswebcam

## General

### Auto parts

There are whole classes of parts that are nearly identical and would be a pain to upload one by one to the store.  Consider the case of an "ubuntu package" part -- all we ever need is the package name.  We don't want to upload 1000 parts.

I'd like to be able to have yaml like:

parts:
  ubuntu-wget
  part2:
    key: value

I propose that if we can't find a bare part's name in the store, we see if it has a hyphen and break the name in two.  If the first piece of the part name is a plugin we know about, we treat it like a part with a single "plugin: xxx" line.  For example:

parts:
 ubuntu-wget
 part2

Here, we'd internally expand ubuntu-wget to:

parts:
 ubuntu-wget:
  plugin: ubuntu
 part2

This way, we have a world of parts with little effort (where it makes sense).

Specifically, I think this would make sense for "ubuntu", "ros", and MAYBE "pypi3" if we can guarantee that we can automatically solve grabbing C libraries for pypi modules.  -- basically any plugin that can automatically guess what the "source" line should be.

The plugin would parse the name if the expected key was missing.  (In ubuntu's case, the plugin would see that the "packages" key was missing and see if the name of the part provided it.)
