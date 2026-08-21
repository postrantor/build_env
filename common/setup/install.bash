# ---
# date: 2026-08-21
# description: 宿主机软件安装入口。现有目标: ROS 2 deb; 后续可在此追加 install-* 子函数。
# ---

# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

# __install_ros_distro
#
# 功能描述：
#   解析 ROS_DISTRO: 已导出则沿用, 否则按 Ubuntu VERSION_CODENAME (jammy->humble, noble->jazzy)。
#
# 参数：
#   (无) 读取 ROS_DISTRO / /etc/os-release。
#
__install_ros_distro() {
  if [[ -n "${ROS_DISTRO:-}" ]]; then
    printf '%s' "${ROS_DISTRO}"
    return 0
  fi
  # shellcheck disable=SC1091
  . /etc/os-release 2>/dev/null || true
  case "${VERSION_CODENAME:-}" in
  jammy) printf '%s' humble ;;
  noble) printf '%s' jazzy ;;
  *) return 1 ;;
  esac
}

# install-ros2
#
# 功能描述：
#   按 Ubuntu 发行版安装 ROS 2 deb (22.04 jammy -> Humble, 24.04 noble -> Jazzy)。
#
# 参数：
#   (无) ROS_DISTRO 已由 system.bash 导出时直接使用; 单独执行本文件时按 VERSION_CODENAME 推断。
#
# 使用示例：
#   install-ros2
#   bash /workspaces/build_env/setup.bash ros2
#
# 注意事项：
#   1. `/opt/ros/${ROS_DISTRO}` 已存在则跳过。
#   2. 需要 sudo 与网络访问 ROS apt 源。
#
install-ros2() {
  local distro
  distro="$(__install_ros_distro)" || {
    echo "install-ros2: ROS_DISTRO unset (need jammy/noble or export ROS_DISTRO)" >&2
    return 1
  }
  export ROS_DISTRO="${distro}"

  if [[ -d "/opt/ros/${ROS_DISTRO}" ]]; then
    echo "ROS 2 (${ROS_DISTRO}) already installed"
    return 0
  fi

  sudo ln -sf /usr/share/zoneinfo/Asia/Shanghai /etc/localtime
  echo "Asia/Shanghai" | sudo tee /etc/timezone

  locale
  sudo -E apt update && sudo -E apt install locales
  sudo -E locale-gen en_US en_US.UTF-8
  sudo -E update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
  locale

  sudo -E apt install software-properties-common -y
  sudo -E add-apt-repository universe -y

  sudo -E apt update && sudo -E apt install curl -y
  export ROS_APT_SOURCE_VERSION
  ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')"
  curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb

  sudo -E apt update && sudo -E apt upgrade -y
  sudo -E apt install -y \
    "ros-${ROS_DISTRO}-desktop-full" \
    "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp"

  sudo -E apt install ros-dev-tools

  sudo -E apt install -y \
    libarchive-dev
}

# setup-install
#
# 功能描述：
#   安装调度器。当前目标: ros2。后续在本文件增加 install-<target> 后在此登记即可。
#
# 参数：
#   $1: target - 安装目标 (ros2)。
#   $@: 透传给对应 install-* 函数。
#
# 使用示例：
#   setup-install ros2
#   bash /workspaces/build_env/setup.bash install ros2
#
setup-install() {
  local target="${1:-}"
  shift || true

  case "${target}" in
  "" | -h | --help | help)
    cat >&2 <<'EOF'
setup-install — install host packages.

Usage: setup-install <target> [args...]
       bash common/setup/install.bash [target] [args...]

Targets:
  ros2    ROS 2 debs (Humble on jammy, Jazzy on noble)

Examples:
  bash /workspaces/build_env/setup.bash ros2
  bash /workspaces/build_env/setup.bash install ros2
  source /workspaces/build_env/setup.bash && install-ros2
EOF
    [[ -z "${target}" ]] && return 1
    return 0
    ;;
  ros2 | install-ros2)
    install-ros2 "$@"
    ;;
  *)
    echo "setup-install: unknown target: ${target}" >&2
    return 1
    ;;
  esac
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  if [[ $# -eq 0 ]]; then
    install-ros2
  else
    setup-install "$@"
  fi
fi
