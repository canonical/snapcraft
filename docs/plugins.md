# Write your own plugins

Is Snapcraft missing support for your preferred build-system? Here's how you
would add it!

Snapcraft can be extended by adding new part plugins. Let's add support for
running a custom build-tool named "Crafty". The goal of this plugin is to
pass a configurable target to the `crafty sometarget` command inside the
`src/crafty` source directory.

Relative to the directory of `snapcraft.yaml`, create 
`parts/plugins/x-crafty.yaml` with:

```yaml
options:
    source:
        required: true
    source-type:
    source-tag:
    source-branch:
    crafty-target:
        required: true
```

and `parts/plugins/x_crafty.py` with:

```python
import snapcraft


class XCraftyPlugin(snapcraft.BasePlugin):
    def build(self):
        return self.run(['crafty', self.options.crafty_target])

    def pull(self):
	return self.handle_source_options()
```

Note that to avoid collisions with official part plugins, the names are
prefixed with "x":  `x_crafty`, and `x-crafty`.

This part plugin is then used in `snapcraft.yaml` with:

```yaml
parts:
  crafted-bits:
    plugin: x-crafty
    source: src/crafty
    crafty-target: sometarget
```

New part plugins are typically placed in the snap's source directory along
`snapcraft.yaml` as they mature, then submitted as patches to the Snapcraft
project for everyone to use.
