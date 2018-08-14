# How to write a template

Create a new directory alongside this README. The name of the directory is the
name of the template. Within that directory, create a `template.yaml` that
follows this syntax:

```yaml
# Keys in the root of the yaml specify the supported bases
core18:
  apps:
    '*':
      # YAML snippet you want to add to apps using this template (e.g. `plugs`)

  parts:
    '*':
      # Yaml snippet you want to add to parts in the `snapcraft.yaml` (e.g.
      # `after`)

    # You can also put any required part definitions here, and they will be put
    # alongside the other parts in the `snapcraft.yaml`. Take note of the
    # `$SNAPCRAFT_TEMPLATES_DIR` variable that is pointing to the directory
    # containing this README.
    my-template-part:
      plugin: dump
      source: $SNAPCRAFT_TEMPLATES_DIR/my-template-name/src
  ```
