#! /bin/bash
# create-user.bash trantor 1000

USERNAME=$1
USER_UID=$2
USER_GID=$USER_UID

groupadd --gid $USER_GID $USERNAME
useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME

apt update && apt install -y sudo
echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME
chmod 0440 /etc/sudoers.d/$USERNAME
