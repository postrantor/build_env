#! /bin/sh

if [ -d /opt/ros/humble ]; then
  echo "ROS 2 Humble is already installed"
  # exit 0
fi

# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#set-locale
locale # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale # verify settings

# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#setup-sources
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo -E curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#install-ros-2-packages
sudo apt update
sudo apt upgrade
# sudo apt install ros-humble-desktop

# https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#install-development-tools-and-ros-tools
sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

sudo apt install -y \
  python3-flake8-blind-except \
  python3-flake8-builtins \
  python3-flake8-class-newline \
  python3-flake8-comprehensions \
  python3-flake8-deprecated \
  python3-flake8-import-order \
  python3-flake8-quotes \
  python3-pytest-repeat \
  python3-pytest-rerunfailures

## vcs-tools
# https://docs.ros.org/en/humble/Glossary.html#term-VCS
# pip install -U vcstool
