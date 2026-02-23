# Documentation subproject

The documentation build configuration is stored as its own subproject, a copy of
[Canonical's Sphinx Starter
Pack](https://github.com/canonical/sphinx-docs-starter-pack). Updating and managing this
subproject happens separately from the main app.

The [Starter Pack documentation](https://canonical-starter-pack.readthedocs-hosted.com)
describes the officially-supported features and provides guidance for customizing the docs.

## Update the docs subproject

The goal is to override the build configuration of the starter pack as little as
possible, so when changes come we don't have to create them. The process isn't completely automatic.

First, run the update script:

```bash
make docs-update
```

The script will inform you of the files that have changed. Go through each notification
and make the proper adjustments so the new and updated features work properly.

In `pyproject.toml`, remove everything in the `docs-starter-pack` group.

Then, sync the docs dependencies to the parent project:

```bash
make clean
make docs-setup
uv add -r docs/requirements.txt --group docs-starter-pack
```

For safety, test the three main doc commands:

```bash
make docs
make docs-auto
make docs-lint
```
