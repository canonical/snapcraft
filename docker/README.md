# Docker images for `snapcraft`

OCI-compliant container images and their sources are officially supported by
https://github.com/canonical/snapcraft-rocks/ project.

To build a snap with the docker container, you need to choose an image that
matches snap `base`. For example, to build `base: core24` snap:

    docker run -it -v `pwd`:/project ghcr.io/canonical/snapcraft:8_core24

 * `8` in `8_core24` is the version of snapcraft.
 * `\; -v` construction at the end is required to see `snapcraft` output.

For more details, see official `snapcraft-rocks` repo from Canonical.

### Building snaps with `podman`

`podman` was born as a rootless alternative to Docker. It is default on Fedora
to have `podman` instead of Docker, but SELinux there doesn't allow containers
to write to volumes, so we just turn this "feature" off with
 `--security-opt label=disable`.

```sh
podman run -it --rm --security-opt label=disable \
    -v `pwd`:/project ghcr.io/canonical/snapcraft:8_core24 \; -v
```
