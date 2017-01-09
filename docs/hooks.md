# Snapcraft hook support

Snapcraft supports snapd's [hooks][1] mechanism using two possible methods:

1. Create a `snap/hooks/` directory in the root of the project and place hook
   executables in there. They will automatically be copied into the snap during
   the `prime` step. Note that these hooks will not be influenced by parts (e.g.
   no `$PYTHONPATH` will be defined, even if the snap includes a python part).
   A [demo][2] is available that shows this feature.

2. Install hook executables from parts into
   `$SNAPCRAFT_PART_INSTALL/snap/hooks/`. These will automatically be copied
   into the snap during the `prime` step, and wrappers will be generated for
   them to include the part's environment (e.g. `$PYTHONPATH` will be defined).
   A [demo][3] is available that shows this feature.

As long as the file name of the executable corresponds to a hook name supported
by snapd, that's all one needs to do in order to utilize a hook within their
snap. Note that hooks, like apps, are executed within a confined environment. By
default hooks will run with no plugs; if a hook needs more privileges one can
use the top-level attribute `hooks` in `snapcraft.yaml` to request plugs, like
so:

    hooks: # Top-level YAML attribute, parallel to `apps`
      configure: # Hook name, corresponds to executable name
        plugs: [network] # Or any other plugs required by this hook

Note that hooks will be called with no parameters. If they need more information
from snapd (or need to provide information to snapd) they can utilize the
`snapctl` command (for more information on `snapctl`, run `snapctl -h`).

[1]: https://github.com/snapcore/snapd/wiki/hooks
[2]: https://github.com/snapcore/snapcraft/tree/master/demos/hooks
[3]: https://github.com/snapcore/snapcraft/tree/master/demos/pyhooks
