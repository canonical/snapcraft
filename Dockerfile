FROM ubuntu:xenial

# Enable multiverse as snapcraft cleanbuild does.
RUN sed -i 's/ universe/ universe multiverse/' /etc/apt/sources.list

RUN apt-get update && \
  apt-get dist-upgrade --yes && \
  apt-get install --yes \
  git \
  snapcraft \
  && \
  apt-get autoclean --yes && \
  apt-get clean --yes

# Required by click.
ENV LC_ALL C.UTF-8
ENV SNAPCRAFT_SETUP_CORE 1
