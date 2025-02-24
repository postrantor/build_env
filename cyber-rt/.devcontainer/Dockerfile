FROM althack/ros2:humble-dev

# install cyber dependencies
RUN apt update && apt install -q -y --no-install-recommends \
  uuid-dev libncurses5-dev pkg-config
RUN python3 -m pip install protobuf==3.14.0

# add user
ARG REMOTE_USER
ENV USER=$REMOTE_USER
RUN adduser --force-badname --group ${USER}
RUN adduser --shell /bin/bash --system --disabled-password --force-badname ${USER}
RUN echo ${USER} ALL=\(root\) NOPASSWD:ALL >> /etc/sudoers

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN ln -s ${WORKSPACE} /home/${USER}/
