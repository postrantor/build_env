#!/bin/bash

# ---
# date: 2025-01-11
# author: postrantor
# description: manage project
# ---

## @file manage-project.bash
#  @brief Script to manage ROS2-based red robot project.
#
#  This script provides utility functions for managing a ROS2 workspace,
#  including updating rosdep, building packages, importing repositories,
#  installing dependencies, and ignoring specific packages.
#
#  Example usage:
#  @code
#  source /home/trantor/build_env/red/.env/manage-project.bash
#  update-rosdep
#  import-red-src
#  install-dependencies ./src
#  colcon_ws model
#  @endcode

## @function update-rosdep
#  @brief Initialize and update rosdep configuration and sources.
#
#  This function checks if rosdep is initialized and updates it if necessary.
#  It should be run with `sudo` to ensure proper permissions.
#
#  @note This function requires `sudo` privileges.
#  @note https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list
#  Example usage:
#  @code
#  sudo -E bash ${RED_WORKDIR}/.env/update-rosdep.bash
#  @endcode
function update-rosdep() {
  # Check if rosdep is initialized
  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "rosdep is not initialized, initializing..."
    sudo -E rosdep init || {
      echo "Failed to initialize rosdep. Please check your network connection or permissions."
      exit 1
    }
  fi

  # Check if rosdep needs to be updated
  if [ ! -d ~/.ros/rosdep ]; then
    echo "rosdep is not updated, updating..."
    sudo -u $(logname) -E rosdep update || {
      echo "Failed to update rosdep. Please check your network connection or permissions."
      exit 1
    }
  else
    echo "rosdep is already initialized, skipping update."
  fi

  echo "rosdep update completed successfully."
}

## @function cd_ws
#  @brief Change directory to a specific workspace and source its setup file.
#
#  This function navigates to a specified workspace and sources the appropriate
#  setup file (bash or zsh) based on the current shell.
#
#  @param $1 Workspace name (e.g., "model", "control", "unitree").
#
#  Example usage:
#  @code
#  cd_ws model
#  @endcode
function cd_ws() {
  declare -A paths=(
    ["model"]="${RED_WORKDIR}"
    ["control"]="${HOME}/project/control_ws"
    ["unitree"]="${HOME}/project/unitree_ws"
  )

  if [[ -z ${paths[$1]} ]]; then
    echo "Invalid argument. Usage: cd workspace [control|demo|unitree] [custom_args]"
    return 1
  fi

  cd "${paths[$1]}"

  if [ -n "$BASH_VERSION" ]; then
    source "./install/setup.bash"
  elif [ -n "$ZSH_VERSION" ]; then
    source "./install/setup.zsh"
  fi
}

## @function colcon_ws
#  @brief Build a specific workspace using colcon.
#
#  This function changes to the specified workspace and runs `colcon build`
#  with optional arguments.
#
#  @param $1 Workspace name (e.g., "model", "control", "unitree").
#  @param $@ Additional arguments passed to `colcon build`.
#
#  Example usage:
#  @code
#  colcon_ws model --symlink-install
#  @endcode
function colcon_ws() {
  cd_ws $1
  shift # drop before args
  colcon build "$@"
}

## @function colcon_remove
#  @brief Remove build and install directories for specified packages.
#
#  This function removes the build and install directories for one or more
#  packages in the current workspace.
#
#  @param $@ List of packages to remove.
#
#  Example usage:
#  @code
#  colcon_remove package_1 package_2
#  @endcode
function colcon_remove() {
  cd_ws $1

  if [ $# -eq 0 ]; then
    echo "Usage: colcon_remove <package_1> <package_2> ... <package_N>"
    return 1
  fi

  for package in "$@"; do
    install_dir="install/$package"
    build_dir="build/$package"

    if [ -d "$install_dir" ]; then
      echo "Removing $install_dir"
      rm -rf "$install_dir"
    else
      echo "$install_dir does not exist."
    fi

    if [ -d "$build_dir" ]; then
      echo "Removing $build_dir"
      rm -rf "$build_dir"
    else
      echo "$build_dir does not exist."
    fi
  done
}

## @function import-red-src
#  @brief Import red robot source repositories using vcs.
#
#  This function imports the source repositories specified in the `RED_REPO`
#  file into the `./src` directory.
#
#  Example usage:
#  @code
#  import-red-src
#  @endcode
function import-red-src() {
  echo "importing red src..."
  # Create src directory if it doesn't exist
  if ! mkdir -p ./src; then
    echo "Error: Failed to create ./src directory." >&2
    return 1
  fi
  # Import repositories using vcs
  if ! vcs import --retry 3 --force --workers 5 --input "${RED_REPO}" ./src; then
    echo "Error: Failed to import repositories using vcs." >&2
    return 1
  fi
  echo "import completed successfully."
  return 0
}

## @function pull-red-src
#  @brief Import red robot source repositories using vcs.
#
#  This function imports the source repositories specified in the `RED_REPO`
#  file into the `./src` directory.
#
#  Example usage:
#  @code
#  pull-red-src
#  @endcode
function pull-red-src() {
  echo "pulling red src..."
  # pull repositories using vcs
  if ! vcs pull --workers 5 --repos ./src; then
    echo "Error: Failed to pull repositories using vcs." >&2
    return 1
  fi
  echo "pull completed successfully."
  return 0
}

## @function install-dependencies
#  @brief Install dependencies for specified packages using rosdep.
#
#  This function installs dependencies for the specified packages using rosdep.
#  It skips certain keys that are not required or already installed.
#
#  @param $@ List of paths to packages.xml files.
#
#  Example usage:
#  @code
#  install-dependencies ./src
#  @endcode
function install-dependencies() {
  update-rosdep

  rosdep install \
    -y \
    --from-paths "$1" \
    --ignore-src \
    --skip-keys "$@"
}

## @function ignore-directory
#  @brief Ignore a specific directory by creating a COLCON_IGNORE file.
#
#  This function creates a `.colcon-ignore-<key>` file and adds a COLCON_IGNORE
#  file to the specified directory.
#
#  @param $1 Key for the ignore file (e.g., "control").
#  @param $2 Directory pattern to ignore (e.g., "ros2-control/ros2-control").
#
#  Example usage:
#  @code
#  ignore-directory control "ros2-control/ros2-control"
#  @endcode
function ignore-directory() {
  local ignore_file="${RED_WORKDIR}/src/.colcon-ignore-$1"
  local path_pattern="*/$2"

  touch "$ignore_file"
  echo -e "\033[1;31mYou should \`touch ./src/$2/COLCON_IGNORE\`\033[0m"

  for dir in $(find ./ -type d -maxdepth 3 -path "$path_pattern" 2>/dev/null); do
    if [[ -d "$dir" ]]; then
      if ! touch "$dir/COLCON_IGNORE"; then
        echo -e "\033[1;31mWarning: Failed to create COLCON_IGNORE in $dir.\033[0m" >&2
      else
        echo "Created COLCON_IGNORE in $dir"
      fi
    fi
  done
}

## @function ignore-colcon-pkg
#  @brief Ignore specific colcon packages by creating COLCON_IGNORE files.
#
#  This function ignores a predefined list of packages by creating
#  COLCON_IGNORE files in their respective directories.
#
#  Example usage:
#  @code
#  ignore-colcon-pkg
#  @endcode
function ignore-colcon-pkg() {
  declare -A IGNORE_LIST=(
    [reference]="red/beatles/reference-system"
    [protobuf]="red/3part/protobuf/python"
  )

  # Iterate over the ignore list
  for key in "${!IGNORE_LIST[@]}"; do
    ignore-directory "$key" "${IGNORE_LIST[$key]}"
  done
}
