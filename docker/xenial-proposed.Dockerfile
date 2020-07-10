FROM ubuntu:xenial

# Enable proposed and pin snapcraft
RUN echo "deb http://archive.ubuntu.com/ubuntu/ xenial-proposed restricted main multiverse universe" >> /etc/apt/sources.list
RUN echo 'Package: *' > /etc/apt/preferences.d/snapcraft-proposed
RUN echo 'Pin: release a=xenial-proposed' >> /etc/apt/preferences.d/snapcraft-proposed
RUN echo 'Pin-Priority: 400' >> /etc/apt/preferences.d/snapcraft-proposed

RUN apt-get update && \
  apt-get dist-upgrade --yes && \
  apt-get install --yes \
  git \
  snapd \
  snapcraft/xenial-proposed \
  && \
  apt-get autoclean --yes && \
  apt-get clean --yes

# Required by click.
ENV LC_ALL C.UTF-8
ENV SNAPCRAFT_SETUP_CORE 1
