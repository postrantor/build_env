# docker build --build-arg USER_NAME=trantor -f ./Dockerfile -t yocto:v0.1 ./
# docker run -itd --privileged --name yocto.test -v F:\Yocto\meta-ros:/workspaces/ yocto:v0.1

FROM ros2:humble

RUN apt-get update && apt-get install -q -y --no-install-recommends locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN apt-get update && apt-get install -q -y --no-install-recommends \
  sudo curl iputils-ping net-tools \
  chrpath cpio diffstat file gawk zstd lz4
RUN python3 -m pip install kas

# add user
ARG USER_NAME=trantor
RUN adduser --force-badname --group ${USER_NAME}
RUN adduser --shell /bin/bash --system --disabled-password --force-badname ${USER_NAME}
RUN echo ${USER_NAME} ALL=\(root\) NOPASSWD:ALL >> /etc/sudoers

# set bashrc
COPY files/bashrc /home/${USER_NAME}/.bashrc
COPY files/env /home/${USER_NAME}/.env

ARG WORKSPACES=/workspaces
RUN ln -s ${WORKSPACES} /home/${USER_NAME}/
RUN chown -R ${USER_NAME}:${USER_NAME} /home/${USER_NAME}

# Switch to the new user
USER ${USER_NAME}
# Set the home directory of the new user as the default working directory
WORKDIR /home/${USER_NAME}
# RUN sudo chmod 755 -R /home/${USER_NAME}
