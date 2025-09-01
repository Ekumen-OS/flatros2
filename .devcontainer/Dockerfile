FROM osrf/ros:jazzy-desktop AS cacher

WORKDIR /workspace/src

COPY iceoryx.repos .
RUN vcs import < iceoryx.repos

COPY flatros2 .

RUN mkdir -p /workspace/cached \
  && find . -name "package.xml" | \
     xargs cp --parents -t /workspace/cached \
  || true

FROM osrf/ros:jazzy-desktop AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    build-essential \
    gcc \ 
    clang \
    curl \
    gdb \
    git \
    linux-tools-common \
    linux-tools-generic \
    python3-pip \
    python3-vcstools \
    tmux \
  && rm -rf /var/lib/apt/lists/*

ARG USER=developer
ARG GROUP=ekumen
ENV HOME=/home/$USER

RUN deluser ubuntu

RUN addgroup --gid 1000 $GROUP \
  && adduser --uid 1000 --ingroup $GROUP --home $HOME --shell /bin/bash --disabled-password --gecos "" $USER \
  && adduser $USER sudo && adduser $USER video \
  && mkdir -p /workspace && chown $USER:$GROUP /workspace \ 
  && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

USER $USER:$GROUP
WORKDIR /workspace

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
RUN echo 'source $HOME/.cargo/env' >> $HOME/.bashrc

COPY --from=cacher /workspace/cached/ src/

RUN sudo apt-get update \
  && sudo apt-get -y dist-upgrade \
  && . /opt/ros/jazzy/setup.sh \
  && rosdep update \
  && rosdep install -i -y --from-path src \
  && sudo rm -rf /var/lib/apt/lists/*
