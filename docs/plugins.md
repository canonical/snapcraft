# Write your own plugins

Is Snapcraft missing support for your preferred build-system? Here's how you
would add it!

Snapcraft can be extended by adding new part plugins, which are written in
Python. Let's add support for running a custom build-tool named "Crafty". The
goal of this plugin is to pass a configurable target to the `crafty sometarget`
command inside the `src/crafty` source directory.

Relative to the directory of `snapcraft.yaml`, create
`parts/plugins/x-crafty.py` with:

```python
import snapcraft


class CraftyPlugin(snapcraft.BasePlugin):
    @classmethod
    def schema(cls):
        # Use base implementation to get the `source` keywords.
        schema = super().schema()

        # Add another option for our custom target.
        schema['properties']['crafty-target'] = {
            'type': 'string'
        }

        # Our custom target is required.
        schema['required'].append('crafty-target')

        return schema

    def build(self):
        # Use base implementation to set up the 'build' directory.
        super().build()
        # Run custom build command.
        return self.run(['crafty', self.options.crafty_target])
```

Note that by starting the name of the plugin with `x-` we tell Snapcraft to
give our plugins priority over plugins of the same name shipped within Snapcraft
itself. This means that, if Snapcraft at some point ships a plugin named
`crafty`, our snap won't break. We can continue to use the name `crafty` in our
`snapcraft.yaml` though, like this:

```yaml
# ...

parts:
  crafted-bits:
    plugin: crafty
    source: src/crafty
    crafty-target: sometarget
```

New part plugins are typically placed in the snap's source directory along
`snapcraft.yaml` as they mature, then submitted as patches to the Snapcraft
project for everyone to use.

## Initializing a plugin

For the general plugin, specific initialization code may not be needed. If it
does need to do more to be setup correctly, it would need to declare
`__init__` and pass that on to the parent class, like this:

```python

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        # plugin specific initialization code.
```
