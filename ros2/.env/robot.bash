# ---
# date: 2025-01-12
# author: postrantor
# description: robot env entry
# ---

# 1. Environment Directory
# --------------------------
ENV_DIR=${ROS2_WORKDIR}/.env/

# 2. Shell-Specific Setup
# --------------------------
setup_shell() {
  if [ -n "$BASH_VERSION" ]; then
    source ${ROS2_WORKDIR}/install/setup.bash
  elif [ -n "$ZSH_VERSION" ]; then
    source ${ROS2_WORKDIR}/install/setup.zsh
  fi
  export PATH=${ROS2_WORKDIR}/install/af_configurator/lib/af_configurator:$PATH
  export PATH=${ROS2_WORKDIR}/install/vcstool/bin:$PATH
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
    export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/${ROS_DISTRO}/lib
    # export GZ_VERSION=fortress
  elif [[ "$architecture" == "aarch64" ]]; then
    echo "Current machine is ARM architecture"
  else
    echo "Unknown CPU Architecture: $architecture"
  fi
}

# 5. Colcon Configuration
# --------------------------
setup_colcon() {
  export _colcon_cd_root=${ROS2_WORKDIR}  # Set root directory for `colcon_cd`
  source ${ENV_DIR}/colcon_cd.sh          # Source colcon_cd script
  source ${ENV_DIR}/colcon-argcomplete.bash  # Source colcon argument completion

  # Set colcon build arguments and home directory
  export COLCON_DEFAULTS_FILE=${ROS2_WORKDIR}/.config/build-args.yaml  # Default build arguments
  export COLCON_HOME=${ROS2_WORKDIR}/.config/  # Colcon configuration directory
}

# 6. Version Control System (VCS) Tool
# --------------------------
setup_vcs() {
  source ${ENV_DIR}/vcs.bash
  export ROS2_REPO=${ROS2_WORKDIR}/.config/ros2.repos
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
setup_shell
setup_rmw
setup_gazebo
setup_colcon
setup_vcs
setup_formatting
setup_project_management
setup_ide_management
