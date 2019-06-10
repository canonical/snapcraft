FROM ubuntu:xenial as builder

# Grab dependencies
RUN apt update
RUN apt dist-upgrade --yes
RUN apt install --yes curl jq squashfs-tools

# Grab the core snap from the stable channel and unpack it in the proper place
RUN curl -L $(curl -s https://api.snapcraft.io/v2/snaps/info/core -H "Snap-Device-Series: 16" | jq '."channel-map" | .[] | select(.channel.name=="candidate") | select(.channel.architecture=="amd64").download.url') --output core.snap
RUN mkdir -p /snap/core
RUN unsquashfs -d /snap/core/current core.snap

# Grab the snapcraft snap from the candidate channel and unpack it in the proper place
RUN curl -L $(curl -s https://api.snapcraft.io/v2/snaps/info/snapcraft -H "Snap-Device-Series: 16" | jq '."channel-map" | .[] | select(.channel.name=="candidate") | select(.channel.architecture=="amd64").download.url') --output core.snap
RUN mkdir -p /snap/snapcraft
RUN unsquashfs -d /snap/snapcraft/current snapcraft.snap

# Multi-stage build, only need the snaps from the builder. Copy them one at a
# time so they can be cached.
FROM ubuntu:xenial
COPY --from=builder /snap/core /snap/core
COPY --from=builder /snap/snapcraft /snap/snapcraft

# Install the snapcraft runner
COPY bin/snapcraft-wrapper /snap/bin/snapcraft

# Generate locale
RUN apt update && apt dist-upgrade --yes && apt install --yes sudo locales && locale-gen en_US.UTF-8

# Set the proper environment
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8
ENV PATH=/snap/bin:$PATH
