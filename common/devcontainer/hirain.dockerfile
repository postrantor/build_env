FROM dockerhub.hirain.com/af2/ros2:rolling

# setup apt source
COPY sources.list /etc/apt/
RUN apt update
RUN rm /etc/apt/sources.list.d/ros2.list
RUN apt install -y gnupg && \
    curl -fsSL http://pub-crdc.hirain.com/key/ros.key | apt-key add - && \
    echo "deb https://mirrors.hirain.com/repository/ROS2-Ubuntu-Ali/ focal main" > /etc/apt/sources.list.d/ros2.list \
    && apt update
# install build essentials
RUN  apt update && apt install -y build-essential \
    gcc-10 g++-10 cpp-10 clang-12 clangd-12 clang-format-12 clang-tidy-12\
    flex bison lcov python3-colcon-lcov-result \
    bc unzip zip rar unrar expect
RUN update-alternatives --install /usr/bin/cc cc /usr/bin/clang-12 100
RUN update-alternatives --install /usr/bin/c++ c++ /usr/bin/clang++-12 100
RUN update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-12 100

# setup pip source
COPY pip.conf /etc/
RUN apt install -y python3-pip python3-autopep8
# handle proxy
ADD certs/crdc-ca.crt /usr/local/share/ca-certificates/
ADD certs/hirain-proxy.crt /usr/local/share/ca-certificates/
RUN update-ca-certificates

# set correct timezone
ENV TZ=Asia/Shanghai
# add user
ARG REMOTE_USER
ENV USER=$REMOTE_USER
RUN adduser --force-badname --group ${USER}
RUN adduser --shell /bin/bash --system --disabled-password --force-badname ${USER}
RUN echo ${USER} ALL=\(root\) NOPASSWD:ALL >> /etc/sudoers
COPY .bashrc /home/${USER}/
# install vscode server
RUN mkdir -p /home/${USER}/.vscode-server/bin/89de5a8d4d6205e5b11647eb6a74844ca23d2573
COPY vss.tar.gz /home/${USER}/
RUN tar --no-same-owner -zxv -f /home/${USER}/vss.tar.gz -C /home/${USER}/.vscode-server/bin/89de5a8d4d6205e5b11647eb6a74844ca23d2573  --strip 1
RUN cd /home/${USER}/.vscode-server/bin && ln -s 89de5a8d4d6205e5b11647eb6a74844ca23d2573 default_version
RUN rm /home/${USER}/vss.tar.gz
COPY extensions/ /home/${USER}/.vscode-server/extensions/
# setup dev env
ENV HOME=/home/${USER}
COPY init_env_1.0.22.sh /home/${USER}/
RUN chmod a+x /home/${USER}/init_env_1.0.22.sh
RUN echo "${USER}" | /home/${USER}/init_env_1.0.22.sh
RUN rm /home/${USER}/init_env_1.0.22.sh
RUN ln -s /usr/bin/python3 /home/${USER}/bin/python
COPY afpt /home/${USER}/bin/afpt
RUN  chmod a+x /home/${USER}/bin/afpt
RUN chown -R ${USER}:${USER} /home/${USER}
# setup rosdep
ADD 20-default.list /etc/ros/rosdep/sources.list.d/
# patch rosdistro
ADD __init__.py /usr/lib/python3/dist-packages/rosdistro/
# install editor and gdb
RUN apt install -y gdb vim emacs uncrustify
# install package for sphinx / reference-system
RUN apt install -y doxygen
RUN pip3 install -U sphinx_rtd_theme exhale sphinx_multiversion \
    psrecord bokeh
# install tools
RUN apt install tree bash-completion

# setup for af
ARG GDBUS_CODEGEN_VERSION=2.99.9
RUN pip3 install http://pvd.hirain.com/gdbus-codegen-glibmm/gdbus_codegen.glibmm-${GDBUS_CODEGEN_VERSION}-py3-none-any.whl
RUN apt install -y libprotobuf-dev protobuf-compiler uuid-dev libsqlite3-dev libglib2.0-dev libglibmm-2.4-dev dbus-x11 python-importlib libfmt-dev

# RUN curl -o /tmp/libglibmm-2.4-1v5_2.58.0-2_amd64.deb http://mirrors.hirain.com/tools/libglibmm-2.4-1v5_2.58.0-2_amd64.deb && \
#     curl -o /tmp/libglibmm-2.4-dev_2.58.0-2_amd64.deb http://mirrors.hirain.com/tools/libglibmm-2.4-dev_2.58.0-2_amd64.deb && \
#     dpkg -i /tmp/libglibmm-2.4-1v5_2.58.0-2_amd64.deb && \
#     dpkg -i /tmp/libglibmm-2.4-dev_2.58.0-2_amd64.deb && \
#     rm -f /tmp/libglibmm-2.4-1v5_2.58.0-2_amd64.deb /tmp/libglibmm-2.4-dev_2.58.0-2_amd64.deb

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN ln -s ${WORKSPACE} /home/${USER}/
# clang-tidy config
COPY clang-tidy /
# patch binder.h
COPY binder.h /usr/include/linux/android/
# patch rospkg
COPY common.py /usr/lib/python3/dist-packages/rospkg/common.py
# gcov
COPY llvm-gcov.sh /usr/bin/
RUN chmod a+x /usr/bin/llvm-gcov.sh
COPY __init__.py.lcov /usr/lib/python3/dist-packages/colcon_lcov_result/verb/__init__.py
# rust
COPY config.toml /home/${USER}/
COPY cargo-cache /home/${USER}/
COPY cargo-bitbake /home/${USER}/

RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi;if [ ! -d ~/.ros ]; then rosdep update; fi;alias ls='ls --color';export PATH=\$PATH:~/bin:~/.local/bin:${WORKSPACE}/install/aidl/bin;cd ${WORKSPACE};cp /clang-tidy ${WORKSPACE}/.clang-tidy" >> /home/${USER}/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/${USER}/.bashrc
