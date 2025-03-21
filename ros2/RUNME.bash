#!/bin/bash

# 帮助函数
function help {
  echo "Usage: $0 [--help]"
  echo ""
  echo "Available functions:"
  echo "  function: \`cd\`"
  echo "    description: 切换到指定目录，e.g. \`cd .....\`。"
  echo "  function: \`cd_ws\`"
  echo "    description: 切换到指定工作空间并加载其环境配置，e.g. \`cd_ws model\`。"
  echo "  function: \`colcon_cd\`"
  echo "    description: 切换到指定包的目录, e.g. \`colcon_cd example_mpc\`。"
  echo "  function: \`colcon_remove\`"
  echo "    description: 移除指定包的构建和安装目录, e.g. \`colcon_remove example_mpc\`。"
  echo "  function: \`colcon_ws\`"
  echo "    description: 在指定工作空间中构建包, e.g. \`colcon_ws model\`。"
  echo "  function: \`format-packages\`"
  echo "    description: 格式化指定包中的代码, e.g. \`format-packages example_mpc\`。"
  echo "  function: \`format-target-folder-by-clang-format\`"
  echo "    description: 使用clang-format格式化目标文件夹中的代码, e.g. \`format-target-folder-by-clang-format ./src/example_mpc\`。"
  echo "  function: \`ignore-colcon-pkg\`"
  echo "    description: 忽略预定义的colcon包，通过创建COLCON_IGNORE文件, e.g. \`ignore-directory interfaces \"interfaces/geometry2\"\`。"
  echo "  function: \`ignore-directory\`"
  echo "    description: 忽略指定目录，通过创建COLCON_IGNORE文件。"
  echo "  function: \`import-quad-src\`"
  echo "    description: 使用vcs导入四足机器人项目的源代码仓库。"
  echo "  function: \`import-vscode-extensions\`"
  echo "    description: 导入VSCode扩展。"
  echo "  function: \`install-dependencies\`"
  echo "    description: 使用rosdep安装指定包的依赖项。"
  echo "  function: \`update-gitconfig\`"
  echo "    description: 更新Git配置并替换用户信息。"
  echo "  function: \`update-rosdep\`"
  echo "    description: 初始化并更新rosdep配置和源。"
}

# 检查是否有 --help 参数
if [[ "$1" == "--help" ]]; then
  help
  exit 0
fi

# check system env
read -p "Enter P7XXTM1_SSH_IP [default: 192.168.31.201]: " P7XXTM1_SSH_IP
export P7XXTM1_SSH_IP=${P7XXTM1_SSH_IP:-192.168.31.201}
read -p "Enter P7XXTM1_PROXY_PORT [default: 7891]: " P7XXTM1_PROXY_PORT
export P7XXTM1_PROXY_PORT=${P7XXTM1_PROXY_PORT:-7891}
export http_proxy=http://${P7XXTM1_SSH_IP}:${P7XXTM1_PROXY_PORT}
export https_proxy=http://${P7XXTM1_SSH_IP}:${P7XXTM1_PROXY_PORT}

# 生成ssh key
ssh-keygen -t rsa -b 4096 -f ${HOME}/.ssh/id_rsa -N ""
cat ${HOME}/.ssh/id_rsa.pub

# 等待用户按下回车键
echo "should add id_rsa.pub to p7xxtm1 ssh keys"
read -p "press [Enter] to continue..."

# Initialize the workspace for the quadruped project
echo "initializing quadruped workspace..."

# 获取当前脚本的绝对路径
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "current script path: ${SCRIPT_PATH}"

# install ros2
bash ${SCRIPT_PATH}/.env/install-ros2-dependence.bash
export ROS_DISTRO=humble
export ROS2_WORKDIR=${SCRIPT_PATH}

# source bash script
source ${ROS2_WORKDIR}/.env/bashrc.bash
source ${ROS2_WORKDIR}/.env/system.bash
source ${ROS2_WORKDIR}/.env/robot.bash

# Initialize the workspace
update-gitconfig
update-bashrc-config

update-rosdep

import-ros2-src
ignore-colcon-pkg
install-dependencies ${ROS2_WORKDIR}

echo "ros2 workspace initialized."
echo "please run the following command to start the ros2 project:"
echo "colcon build"
