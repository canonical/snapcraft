FROM ubuntu:xenial

RUN apt-get update && \
  apt-get dist-upgrade --yes && \
  apt-get install --yes \
  curl sudo jq squashfs-tools && \
  curl -L $(curl -H 'X-Ubuntu-Series: 16' 'https://api.snapcraft.io/api/v1/snaps/details/core' | jq '.download_url' -r) --output core.snap && \
  mkdir -p /snap/core && unsquashfs -d /snap/core/current core.snap && rm core.snap && \
  curl -L $(curl -H 'X-Ubuntu-Series: 16' 'https://api.snapcraft.io/api/v1/snaps/details/snapcraft?channel=edge' | jq '.download_url' -r) --output snapcraft.snap && \
  mkdir -p /snap/snapcraft && unsquashfs -d /snap/snapcraft/current snapcraft.snap && rm snapcraft.snap && \
  mkdir -p /snap/bin && \
  echo "#!/bin/sh" > /snap/bin/snapcraft && \
  echo 'exec $SNAP/usr/bin/python3 $SNAP/bin/snapcraft "$@"' >> /snap/bin/snapcraft && \
  chmod a+x /snap/bin/snapcraft && \
  apt remove --yes --purge curl jq squashfs-tools && \
  apt-get autoclean --yes && \
  apt-get clean --yes


ENV SNAP=/snap/snapcraft/current
ENV SNAP_NAME=snapcraft
ENV SNAP_VERSION=edge
ENV PATH=/snap/bin:$PATH
# Required by click.
ENV LC_ALL C.UTF-8
