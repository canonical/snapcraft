# Documentation subproject

The documentation build configuration is stored as its own subproject, a copy of the
[Sphinx Stack](https://github.com/canonical/sphinx-stack). Updating and
managing this subproject happens separately from the main app.

The [Sphinx Stack documentation](https://documentation.ubuntu.com/sphinx-stack)
describes the officially-supported features and provides guidance for customizing the docs.

## Update the docs subproject

The goal is to override the build configuration of the Sphinx Stack as little as
possible, so when changes come we don't have to recreate them. The process isn't automatic.

### Gather the changes

You first need the recent history of the standard documentation implementation in Starbase. We'll provide one way you could gather it.

Inside Starbase, find the commit that most recently updated the version of the Sphinx Stack:

```bash
git log --grep "sphinx stack" -i
```

Copy the commit SHA. Then, collect the list of Sphinx Stack files that were changed between that commit and now:

```bash
git --no-pager diff <commit>~1 --name-only -- docs/ .readthedocs.yaml common.mk Makefile
```

### Apply the changes

Next, bring the updates into the subproject. It's simplest to copy over the files, and then review the resulting Git diff. It's likely some of the changes will erase sections of the files, especially in changes to the documents themselves. You'll review each change line-by-line in any case, so you can restore any unwanted deletions as you work through them.

As you review, look for instances of `Starcraft` and `TODO` comments for places where the code needs customizing. If you can't decide on how to configure a change, consult the [Sphinx Stack release notes](https://documentation.ubuntu.com/sphinx-stack/latest/) for idiomatic documentation of the feature.

In `pyproject.toml`, remove everything in the `docs-sphinx-stack` group. Then, sync the docs dependencies to the parent project:

```bash
make clean
make docs-setup
uv add -r docs/requirements.txt --group docs-sphinx-stack
```

For safety, test the three main doc commands:

```bash
make docs
make docs-auto
make docs-lint
```

Check that the new and updated features listed in the Sphinx Stack changelog work.
