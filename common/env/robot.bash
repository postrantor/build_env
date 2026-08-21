# ---
# date: 2025-01-12
# update: 2026-01-11 02:56:49
# author: postrantor@gmail.com
# description: robot env entry
# ---

# 1. Environment Directory
# --------------------------
if [[ -z "${PROJECT_CONFIG:-}" ]]; then
  # shellcheck disable=SC1091
  source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/project.bash"
fi
ENV_DIR=${PROJECT_WORKDIR}/.env/

# 2. Shell-Specific Setup
# --------------------------
setup_ros2() {
  # ROS_DISTRO and common env vars are already exported by system.bash.
  # Vendor debs (Clothoids, ompl, ...) live under /opt/ros/ai2rob and chain humble.
  # Source that prefix before workspace install/setup.* so find_package(Clothoids) works.
  if [ -n "$BASH_VERSION" ]; then
    if [ -f /opt/ros/ai2rob/setup.bash ]; then
      source /opt/ros/ai2rob/setup.bash
    else
      source "/opt/ros/${ROS_DISTRO}/setup.bash"
    fi
    if [ -f "${PROJECT_WORKDIR}/install/setup.bash" ]; then
      source "${PROJECT_WORKDIR}/install/setup.bash"
    fi
  elif [ -n "$ZSH_VERSION" ]; then
    if [ -f /opt/ros/ai2rob/setup.zsh ]; then
      source /opt/ros/ai2rob/setup.zsh
    else
      source "/opt/ros/${ROS_DISTRO}/setup.zsh"
    fi
    if [ -f "${PROJECT_WORKDIR}/install/setup.zsh" ]; then
      source "${PROJECT_WORKDIR}/install/setup.zsh"
    fi
  fi
  export MACHINE+=/ros2
  export PATH="${PROJECT_WORKDIR}/install/af_configurator/lib/af_configurator:${PATH}"
}

# 3. RMW Implementation
# --------------------------
setup_rmw() {
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  # export CYCLONEDDS_URI=""
}

# 4. Gazebo Environment Setup
# --------------------------
setup_gazebo() {
  architecture=$(uname -m)
  if [[ "$architecture" == "x86_64" ]]; then
    source /usr/share/gazebo/setup.sh
    # export GAZEBO_MASTER_URI=http://127.0.0.1:11346 # default:11345
    export GAZEBO_PLUGIN_PATH="/opt/ros/${ROS_DISTRO}/lib:${GAZEBO_PLUGIN_PATH}"
    export GAZEBO_PLUGIN_PATH=${PROJECT_WORKDIR}/install/robot_description/lib/robot_description:$GAZEBO_PLUGIN_PATH
    # export GZ_VERSION=fortress
  elif [[ "$architecture" == "aarch64" ]]; then
    echo -e "\033[35mCurrent machine is ARM architecture\033[0m"
  else
    echo "\033[31mUnknown CPU Architecture: $architecture\033[0m"
  fi
}

# 5. Colcon Configuration
# --------------------------
setup_colcon() {
  export _colcon_cd_root=${PROJECT_WORKDIR}  # Set root directory for `colcon_cd`
  source ${ENV_DIR}/colcon_cd.sh          # Source colcon_cd script
  source ${ENV_DIR}/colcon-argcomplete.bash  # Source colcon argument completion

  # Set colcon build arguments and home directory
  export COLCON_DEFAULTS_FILE=${PROJECT_CONFIG}/build-args.yaml  # Default build arguments

  # Colcon configuration directory: cpu / gpu (HOST_* from host_hw_detect in system.bash)
  if [[ "${HOST_NVIDIA_GPU:-0}" -eq 1 ]]; then
    case "${HOST_PLATFORM:-unknown}" in
    arm) export COLCON_HOME=${PROJECT_CONFIG}/gpu/arm ;;
    *) export COLCON_HOME=${PROJECT_CONFIG}/gpu/x86 ;;
    esac
  else
    case "${HOST_PLATFORM:-unknown}" in
    arm) export COLCON_HOME=${PROJECT_CONFIG}/cpu/arm ;;
    *) export COLCON_HOME=${PROJECT_CONFIG}/cpu/x86 ;;
    esac
  fi
}

# 6. Version Control System (VCS) Tool
# --------------------------
setup_vcs() {
  source ${ENV_DIR}/vcs.bash
  # PROJECT_REPO / PROJECT_CORE / PROJECT_DEV 以及 ${PREFIX}_* 由 project.bash 导出
}

# 7. Code Formatting Tool
# --------------------------
setup_formatting() {
  source ${ENV_DIR}/format-code.bash
}

# 8. Project Management
# --------------------------
setup_project_management() {
  source ${ENV_DIR}/manage-project.bash
}

# 9. IDE Management
# --------------------------
setup_ide_management() {
  source ${ENV_DIR}/manage-ide.bash
}

# Main Execution
# --------------------------
setup_ros2
setup_rmw
setup_gazebo
setup_colcon
setup_vcs
setup_formatting
setup_project_management
setup_ide_management
