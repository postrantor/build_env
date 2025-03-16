FROM althack/ros2:humble-dev
# FROM dockerhub-crdc.hirain.com/af2/beatles_build:241121

# ** [Optional] Uncomment this section to install additional packages. **
#
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#    && apt-get -y install --no-install-recommends <your-package-list-here> \
#    #
#    # Clean up
#    && apt-get autoremove -y \
#    && apt-get clean -y \
#    && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=dialog

# add user
ARG REMOTE_USER
ENV USER=$REMOTE_USER
RUN adduser --force-badname --group ${USER}
RUN adduser --shell /bin/bash --system --disabled-password --force-badname ${USER}
RUN echo ${USER} ALL=\(root\) NOPASSWD:ALL >> /etc/sudoers

# setup dev env
ENV HOME=/home/${USER}
COPY init_env_1.0.23.sh /home/${USER}/
RUN chmod a+x /home/${USER}/init_env_1.0.23.sh
RUN echo "${USER}" | /home/${USER}/init_env_1.0.23.sh
RUN rm /home/${USER}/init_env_1.0.23.sh

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN ln -s ${WORKSPACE} /home/${USER}/
