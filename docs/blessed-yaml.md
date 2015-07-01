# Blessed Yaml

This is blessed yaml, used or agreed to by Gustavo and/or Mark.

https://docs.google.com/document/d/1BA0s9SXqKxtCN-ERVNIh1mNowMpIuR5jDMQlwv5HE7A

## Plugin yaml

### name

String.
Name of plugin.

### provides

String.
Name of alias for this plugin.

- If only one part provides a name, a symlink with that name is created pointing to that part’s filesystem entry, and snapcraft considers that equivalent to the part with that name being available
- If on the other hand two parts provide the same name, the symlink is not created, and snapcraft does not consider the named part as available
- If a name provided by a part is already explicitly in use (*) by another part, the provided name is ignored
- Consequently, when a part requires a name, that name is only satisfied if either a single part provides that name, or that name is explicitly in use (*) by another part

(*) A name being explicitly in use means that there’s a key under the “parts:” field with that name.

### requires

A list of strings.
A list of part names to require.  No configuration can be done.  It just lists plugin names which will need to either exist in the snapcraft.yaml or be implicitly created by snapcraft.

### options

A yaml subsection.
Options to pass to plugin's code.  Supports "required: bool" field on each option.

### resources

A yaml subsection.
A mapping of architecture to specific binary files to be downloaded.  Or source code to build.

## snapcraft.yaml

### parts

A yaml subsection.
List of parts.

##### plugin

A string.
Specifies the plugin name to which to pass all the unknown parameters.  If omitted, the part name is used as the plugin name.

##### after

A string list.
Specifies the parts that should be built before this part is.  Similar in spirit to "requires" from the part yaml.

### packages

A string list.
A list of Ubuntu packages to install locally into the snap.

## Examples

### Simple part with implicit name (snapcraft.yaml)

parts:
  go1.4-project:
    source: http://github.com/rick/go-hello-world

### Simple part with explicit name (snapcraft.yaml)

parts:
  go-hello-world:
    using: go1.4-project
    source: http://github.com/rick/go-hello-world

### Multiple parts (snapcraft.yaml)

parts:
  go-up:
    using: go1.4-project
    source: http://github.com/rick/go-up
  go-down:
    using: go1.4-project
    source: http://github.com/rick/go-down

### go1.4-project.yaml

name: go1.4-project
requires:
  - go1.4
options:
  source:
    required: true

### go1.4.yaml

name: go1.4
provides: go
resources:
	amd64:
		go1.4: http://.../go1.4-bin.tar.gz
	source:
		golint: http://github.com/joe/golint

### Including a package from the archive into your snap

parts:
  ...
packages:
 - fswebcam
