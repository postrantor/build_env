# ---
# date: 2025-01-11
# author: postrantor
# description: manage project
# ---

# go to workspace
function cd_ws() {
  declare -A paths=(
    ["model"]="${QUAD_WORKDIR}"
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

# colcon build target workspace
function colcon_ws() {
  cd_ws $1
  shift # drop before args
  colcon build "$@"
}

function colcon_remove() {
  # Usage example:
  # colcon_remove package_1 package_2
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

# import quad repo
function import-quad-src() {
  echo "importing quad src..."
  # 创建 src 目录，存在时不报错
  if ! mkdir -p ./src; then
    echo "Error: Failed to create ./src directory." >&2
    return 1
  fi
  # 导入仓库
  if ! vcs import --retry 3 --force -w 5 --input "${QUAD_REPO}" ./src; then
    echo "Error: Failed to import repositories using vcs." >&2
    return 1
  fi
  echo "Import completed successfully."
  return 0
}

# install dependencies from packages.xml
function install-dependencies() {
  # sudo rosdep init
  # rosdep update
  rosdep install \
    -y \
    --from-paths "$@" \
    --ignore-src \
    --skip-keys "serial \
              unitree_msgs \
              unitree_sdk \
              quadruped \
              tinynurbs \
              tiny_ekf \
              quadprog \
              matplotlib_cpp \
              qpOASES \
              SuiteSparse \
              message_generation \
              message_runtime \
              catkin \
              xpp_msgs"
}

# ignore target folder
function ignore-directory() {
  local ignore_file="./src/.colcon-ignore-$1"
  local path_pattern="*/$2"

  if [[ ! -e $ignore_file ]]; then
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
  fi
}

# ignore colcon packages
function ignore-colcon-pkg() {
  declare -A IGNORE_LIST=(
    [control]="ros2-control/ros2-control"
    [controllers]="ros2-control/ros2-controllers"
    [parameter]="ros2-control/generate-parameter-library"
    [infrastructure]="ros2-infrastructure"
    [reference]="reference"
    [interfaces]="interfaces/geometry2"
    [simulation]="simulation"
  )

  # 遍历忽略目录
  for key in "${!IGNORE_LIST[@]}"; do
    ignore-directory "$key" "${IGNORE_LIST[$key]}"
  done
}
