FROM ubuntu:22.04

# setup timezone
RUN echo 'Asia/Shanghai' > /etc/timezone \
  && ln -s /usr/share/zoneinfo/Asia/Shanghai /etc/localtime \
  && apt-get update \
  && apt-get install -q -y --no-install-recommends tzdata \
  && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# install develop tool
RUN apt-get update && apt-get install -q -y --no-install-recommends \
  gnupg2 libssl-dev wget dirmngr git vim \
  build-essential cmake make \
  gcc-10 g++-10 cpp \
  clang-15 clang++-15 clangd clang-format clang-tidy\
  flex bison lcov bc unzip zip rar unrar \
  doxygen gdb vim emacs uncrustify tree \
  iputils-ping net-tools \
  python3-argcomplete bash-completion \
  python3-pip python3-dev

# install cyber dependencies
RUN apt-get update && apt-get install -q -y --no-install-recommends \
  uuid-dev libncurses5-dev pkg-config
RUN python3 -m pip install protobuf==3.14.0

COPY files/third_part.tar.gz /opt/
RUN cd /opt && tar -zxvf third_part.tar.gz

# add user
ARG REMOTE_USER
ENV USER=$REMOTE_USER
RUN adduser --force-badname --group ${USER}
RUN adduser --shell /bin/bash --system --disabled-password --force-badname ${USER}
RUN echo ${USER} ALL=\(root\) NOPASSWD:ALL >> /etc/sudoers

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN ln -s ${WORKSPACE} /home/${USER}/
