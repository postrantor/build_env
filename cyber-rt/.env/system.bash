# add colcon
source ${CYBER_WORKDIR}/.env/colcon_cd.sh
source ${CYBER_WORKDIR}/.env/colcon-argcomplete.bash
export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/

# set colcon::build
export COLCON_DEFAULTS_FILE=${CYBER_WORKDIR}/.config/build-args.yaml # for `colcon build`
export COLCON_HOME=${CYBER_WORKDIR}/.config/                         # for colcon.meta

# add vcs-complete
source ${CYBER_WORKDIR}/.env/vcs.bash
export CYBER_REPO=${CYBER_WORKDIR}/.config/cyber.repos

# use `cd ...` equal `cd ../..`
function cd() {
  if [[ $1 =~ ^\.{2,}$ ]]; then
    local count=${#1}
    local path=""
    for ((i = 1; i < count; i++)); do
      path+="../"
    done
    builtin cd "$path"
  else
    builtin cd "$@"
  fi
}

# 忽略指定目录的函数
function ignore_directory() {
  local ignore_file="./src/.colcon-ignore-$1"
  local path_pattern="*/$2"

  if [[ ! -e $ignore_file ]]; then
    touch "$ignore_file"
    echo -e "\033[1;31mYou should \`touch ./src/$2/COLCON_IGNORE\`\033[0m"

    for dir in $(find ./ -type d -path "$path_pattern" 2>/dev/null); do
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

# 添加patch
function add_patch() {
  local patch_flag="./src/.add-patch-$1"
  local path_pattern="*/$2"
  local patch_file="$3"

  if [[ ! -e $patch_flag ]]; then
    touch "$patch_flag"
    echo -e "\033[1;31mYou should \`add patch ./src/$2\`\033[0m"

    for dir in $(find ./ -type d -path "$path_pattern" 2>/dev/null); do
      cp "./patch/$patch_file" $dir
      cd $dir
      git apply $patch_file
      cd "-"
    done
  fi
}

# import cyber repo
function import-cyber-src {

  echo "importing cyber src..."

  # 创建 src 目录，存在时不报错
  if ! mkdir -p ./src; then
    echo "Error: Failed to create ./src directory." >&2
    return 1
  fi

  # 导入仓库
  if ! vcs import ./src <"$CYBER_REPO"; then
    echo "Error: Failed to import repositories using vcs." >&2
    return 1
  fi

  # 需要忽略的目录列表
  declare -A IGNORE_LIST=(
    [protobuf - python]="protobuf/python"
  )
  for key in "${!IGNORE_LIST[@]}"; do
    ignore_directory "$key" "${IGNORE_LIST[$key]}"
  done

  # 添加patch
  add_patch "fast-rtps" "dds/fast-rtps" "0001-fastrtps-split-patch-for-rtps.patch"
  add_patch "fast-cdr" "dds/fast-cdr" "0001-fastrtps-split-patch-for-cdr.patch"
  add_patch "cyber-rt" "cyber-rt" "0001-cyber-refactor-build.patch"

  echo "Import completed successfully."
  return 0
}
