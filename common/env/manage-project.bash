#!/bin/bash
#
# ---
# date: 2026-04-03 01:48:38
# author: zhiqi.jia <zhiqi.jia@ai2robotics.com>
# description: manage project
# ---


## @file manage-project.bash
#  @brief Script to manage ROS2-based auriga robot project.
#
#  This script provides utility functions for managing a ROS2 workspace,
#  including updating rosdep, building packages, importing repositories,
#  installing dependencies, and ignoring specific packages.
#
#  Example usage:
#  @code
#  source /home/trantor/build_env/project/.env/manage-project.bash
#  update-rosdep
#  import-project-src
#  import-project-src -i
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
#  bash ${PROJECT_WORKDIR}/.env/update-rosdep.bash
#  @endcode
function update-rosdep() {
  # Check if rosdep is initialized
  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "rosdep is not initialized, initializing..."
    rosdep init || {
      echo "failed to initialize rosdep. please check your network connection or permissions."
      exit 1
    }
  fi

  # Check if rosdep needs to be updated
  if [ ! -d ~/.ros/rosdep ]; then
    echo "rosdep is not updated, updating..."
    rosdep update || {
      echo "failed to update rosdep. please check your network connection or permissions."
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
    ["control"]="${HOME}/project/control_ws"
    ["unitree"]="${HOME}/project/unitree_ws"
  )
  [[ -n "${PROJECT_CD_KEY:-}" ]] && paths["${PROJECT_CD_KEY}"]="${PROJECT_WORKDIR}"
  [[ -n "${PROJECT_NAME:-}" ]] && paths["${PROJECT_NAME}"]="${PROJECT_WORKDIR}"
  paths["ai2"]="${AI2_WORKDIR:-${PROJECT_WORKDIR}}"

  if [[ -z ${paths[$1]} ]]; then
    echo "Invalid argument. Usage: cd_ws [${PROJECT_CD_KEY:-ai2}] [control] [unitree]"
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
    echo "usage: colcon_remove <package_1> <package_2> ... <package_N>"
    return 1
  fi

  for package in "$@"; do
    install_dir="install/$package"
    build_dir="build/$package"

    if [ -d "$install_dir" ]; then
      echo "removing $install_dir"
      rm -rf "$install_dir"
    else
      echo "$install_dir does not exist."
    fi

    if [ -d "$build_dir" ]; then
      echo "removing $build_dir"
      rm -rf "$build_dir"
    else
      echo "$build_dir does not exist."
    fi
  done
}

## @function colcon_rebuild
#  @brief Remove build and install directories for specified packages then build.
#
#  This function removes the build and install directories then build
#  packages in the current workspace.
#
#  @param $@ List of packages to remove and build.
#
#  Example usage:
#  @code
#  colcon_rebuild <workspaces> <package_1> [package_2] ... [package_N]
#  @endcode
function colcon_rebuild() {
  if [ $# -lt 2 ]; then
    echo "usage: colcon_rebuild <workspace> <package_1> [package_2] ... [package_N]"
    return 1
  fi

  workspace=$1
  shift  # 移除第一个参数(workspace)

  # 调用 colcon_remove 来清理之前的包
  colcon_remove "$workspace" "$@"

  # 重新构建包
  colcon build --packages-select "$@"
}

## @function import-project-src
#  @brief Ensure ./.ci/manifests exists, optionally checkout a ref, then import ./src via vcs.
#
#  Execution order:
#  1. `cd` to `${PROJECT_WORKDIR}`.
#  2. With `--repos REF`: ensure `./.ci/manifests` exists via `manifests.repos`,
#     `git checkout REF` (branch or tag), then `vcs pull` `./.ci`.
#  3. `vcs import` into `./src` using the `--input` repos file and options
#     below (default input: `${PROJECT_REPO}`; typical follow-up: `${PROJECT_CORE}`).
#
#  Recognised options (any order): `--repos REF`, `--input PATH`, `--skip-existing` /
#  `--no-skip-existing`, `--shallow` / `--no-shallow`, `--recursive` /
#  `--no-recursive`, `-i` / `--interactive`.
#  Other tokens are passed through to the step-3 `vcs import` before `./src`.
#
#  Defaults: input from `${PROJECT_REPO}`; `--skip-existing`, `--shallow`, and
#  `--recursive` off. With `-i`, asks [Y/n] to continue first; then only options not
#  already given on the command line are prompted (defaults shown in brackets).
#
#  Example usage:
#  @code
#  import-project-src
#  import-project-src --repos "${PROJECT_SOURCE_BRANCH}" --input "${PROJECT_CORE}"
#  import-project-src --shallow --recursive --input "${PROJECT_CORE}"
#  import-project-src -i
#  import-project-src -i --recursive
#  @endcode
function import-project-src() {
  case "${1-}" in
  --help | -h)
    cat >&2 <<'EOF'
import-project-src — clone repos into ./src; optionally checkout ./.ci/manifests.

Always runs first:
  vcs import --input ${PROJECT_REPO}

With --repos REF (branch or tag):
  git checkout REF in ./.ci/manifests
  vcs pull --repos ./.ci

Then runs with your options:
  vcs import [OPTIONS...] ./src

Usage: import-project-src [OPTIONS...] [EXTRA_VCS_ARGS...]

  -h, --help              show this help
  -i, --interactive       ask to continue first [Y/n], then prompt for options not given on the command line

  --repos REF             checkout ./.ci/manifests to branch or tag REF, then vcs pull ./.ci
  --input PATH            vcs --input file for ./src (e.g. "${PROJECT_CORE}"); default: ${PROJECT_REPO}
  --skip-existing         enable
  --no-skip-existing      disable (default)
  --shallow / --no-shallow
  --recursive / --no-recursive

  EXTRA_VCS_ARGS          any other vcs import flags (before ./src)

Step 3 always adds: --retry 3 --force --workers 5.

Examples:
  import-project-src
  import-project-src --repos "${PROJECT_SOURCE_BRANCH}" --input "${PROJECT_CORE}"
  import-project-src --shallow --recursive --input "${PROJECT_CORE}"
  import-project-src -i
  import-project-src -i --recursive
EOF
    return 0
    ;;
  esac

  # initialize variables
  local repo_file="${PROJECT_REPO}"
  local repos_ref=""
  local use_skip_existing=0 use_shallow=0 use_recursive=0
  local cli_repos=0 cli_input=0 cli_skip_existing=0 cli_shallow=0 cli_recursive=0
  local interactive=0
  local extra_vcs=()

  # parse arguments
  while (($#)); do
    case "${1}" in
    -i | --interactive)     interactive=1;                             shift   ;;
    --repos)                [[ -n "${2-}" ]] || { echo "error: --repos requires a branch or tag." >&2; return 1; }
                            repos_ref="${2}"; cli_repos=1;                   shift 2 ;;
    --input)                [[ -n "${2-}" ]] || { echo "error: --input requires a file path." >&2; return 1; }
                            repo_file="${2}"; cli_input=1;             shift 2 ;;
    --skip-existing)        use_skip_existing=1; cli_skip_existing=1;  shift   ;;
    --no-skip-existing)     use_skip_existing=0; cli_skip_existing=1;  shift   ;;
    --shallow)              use_shallow=1;       cli_shallow=1;        shift   ;;
    --no-shallow)           use_shallow=0;       cli_shallow=1;        shift   ;;
    --recursive)            use_recursive=1;     cli_recursive=1;      shift   ;;
    --no-recursive)         use_recursive=0;     cli_recursive=1;      shift   ;;
    *)                      extra_vcs+=("$1");                         shift   ;;
    esac
  done

  # interactive mode
  if ((interactive)); then
    echo -e "\033[33mimport-project-src: interactive mode\033[0m" >&2

    local _ans
    read -r -e -p "Continue with import? [Y/n]: " _ans || { echo "error: aborted." >&2; return 1; }
    case "${_ans,,}" in
    n | no)
      echo "import-project-src: aborted." >&2
      return 0
      ;;
    esac

    ((cli_input)) || {
      local _ans; read -r -e -p "vcs --input file path [${repo_file}]: " _ans || { echo "error: aborted." >&2; return 1; }
      [[ -n "$_ans" ]] && repo_file="$_ans"
    }
    ((cli_repos)) || {
      local _ans; read -r -e -p "Checkout ./.ci/manifests branch or tag [--repos]: " _ans || { echo "error: aborted." >&2; return 1; }
      [[ -n "$_ans" ]] && repos_ref="$_ans"
    }
    ((cli_skip_existing)) || {
      local _ans; read -r -e -p "Add --skip-existing? [y/N]: " _ans || { echo "error: aborted." >&2; return 1; }
      [[ "${_ans,,}" =~ ^(y|yes)$ ]] && use_skip_existing=1
    }
    ((cli_shallow)) || {
      local _ans; read -r -e -p "Add --shallow? [y/N]: " _ans || { echo "error: aborted." >&2; return 1; }
      [[ "${_ans,,}" =~ ^(y|yes)$ ]] && use_shallow=1
    }
    ((cli_recursive)) || {
      local _ans; read -r -e -p "Add --recursive? [y/N]: " _ans || { echo "error: aborted." >&2; return 1; }
      [[ "${_ans,,}" =~ ^(y|yes)$ ]] && use_recursive=1
    }
  fi

  # change to workspace directory
  cd ${PROJECT_WORKDIR}

  # ensure .ci/manifests exists
  echo "importing .ci manifests..."
  vcs import --input ${PROJECT_REPO} || {
    echo "error: failed to import repositories using vcs." >&2; return 1
  }

  if [[ -n "${repos_ref}" ]]; then
    local manifest_dir="${PROJECT_WORKDIR}/.ci/manifests"
    echo "checking out ${manifest_dir} to ${repos_ref}..."
    git -C "${manifest_dir}" fetch origin --tags || {
      echo "error: failed to fetch in ${manifest_dir}." >&2; return 1
    }
    git -C "${manifest_dir}" checkout "${repos_ref}" || {
      git -C "${manifest_dir}" checkout -B "${repos_ref}" "origin/${repos_ref}" || {
        echo "error: failed to checkout ${repos_ref} in ${manifest_dir}." >&2; return 1
      }
    }
    vcs pull --workers 5 --repos ${PROJECT_WORKDIR}/.ci || {
      echo "error: failed to pull repositories in ./.ci using vcs." >&2; return 1
    }
  fi

  # validate repo file
  [[ -n "$repo_file" ]] || { echo "error: --input file path is empty (PROJECT_REPO unset?)." >&2; return 1; }
  [[ -f "$repo_file" ]] || { echo "error: repo file not found: ${repo_file}" >&2; return 1; }

  # set vcs import arguments
  local vcs_args=(--retry 3 --force --workers 5 --input "$repo_file")
  ((use_skip_existing)) && vcs_args=(--skip-existing "${vcs_args[@]}")
  ((use_shallow))       && vcs_args+=(--shallow)
  ((use_recursive))     && vcs_args+=(--recursive)

  # import project source repositories
  echo "importing project src (input: ${repo_file})..."
  mkdir -p ${PROJECT_WORKDIR}/src || { echo "error: failed to create ./src directory." >&2; return 1; }
  vcs import "${vcs_args[@]}" "${extra_vcs[@]}" ${PROJECT_WORKDIR}/src || {
    echo "error: failed to import repositories using vcs." >&2; return 1
  }

  echo "import completed done."
  return 0
}

## @function pull-project-src
#  @brief Pull project source and CI manifest repositories using vcs.
#
#  This function pulls `./.ci` (manifests, etc.) first, then `./src`,
#  sequentially via `vcs pull`.
#
#  Example usage:
#  @code
#  pull-project-src
#  @endcode
function pull-project-src() {
  local pull_ci=(./.ci)
  local pull_src=(./src)

  # change to workspace directory
  cd ${PROJECT_WORKDIR}

  echo "pulling .ci manifests (repos files)..."
  vcs pull --workers 5 --repos "${pull_ci[@]}" || {
    echo "error: failed to pull repositories in ./.ci using vcs." >&2
    return 1
  }

  echo "pulling project src..."
  vcs pull --workers 5 --repos "${pull_src[@]}" || {
    echo "error: failed to pull repositories in ./src using vcs." >&2
    return 1
  }

  echo "pull completed done."
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
#  `install-dependencies ./src`
#  @endcode
function install-dependencies() {
  update-rosdep

  rosdep install \
    -y \
    --from-paths "$@" \
    --ignore-src \
    --skip-keys="\
        auriga_sm \
	      rm_ros_interfaces \
	      rm_driver \
	      sr_all_events_go \
	      ros_timer_client \
	      rosidl_interface_packages \
	      xmlrpcpp"
}

## @function __install-deb-paths
#  @brief 安装一组 .deb 路径 (dpkg --force-* + apt-get install -f + dpkg 二次安装)。
#
#  @param $1 dpkg --force 选项 (逗号分隔, 默认 overwrite); 其余参数为 deb 路径。
function __install-deb-paths() {
  local dpkg_force="${1:-overwrite}"
  shift
  local -a deb_paths=("$@")
  if [[ ${#deb_paths[@]} -eq 0 ]]; then
    echo "error: no .deb files to install." >&2
    return 1
  fi
  echo "installing ${#deb_paths[@]} deb(s) with dpkg (--force-${dpkg_force}) ..."
  sudo dpkg -i --force-"${dpkg_force}" "${deb_paths[@]}" || true
  sudo apt-get install -f -y
  sudo dpkg -i --force-"${dpkg_force}" "${deb_paths[@]}"
}

# download-dependencies
#
# 功能描述：
#   从 GitLab binary 仓库下载预编译 vendor .deb 并安装, 或从本地目录遍历 *.deb 后安装。
#   默认模式按 HOST_PLATFORM (arm→aarch|x86→x86) 拉取七套 ros-ai2rob-*-vendor deb 到
#   ${PROJECT_WORKDIR}/dependencies/binary/<aarch|x86> 再 dpkg 安装; --path DIR / 位置参数 DIR
#   则跳过下载, 安装该目录下全部 .deb (sorted)。
#
# 参数：
#   -f | --force - 跳过下载/安装前的 [Y/n] 询问 (默认 Y); 非 TTY 时本就默认 Y。
#   -i | --interactive - 未指定 --path/DIR 时交互选择安装来源 (remote / local / skip),
#                        参考 docker.bash 的 _docker_pick; local 可再输入 deb 目录。
#   -p | --path DIR - 从 DIR 安装全部 *.deb, 不下载远程包。
#   DIR - 同 --path DIR (位置参数)。
#   --redownload - 本地已存在同名 deb 时仍重新下载。
#   --reinstall - dpkg 使用更强 --force (overwrite,confnew,confmiss,depends)。
#   --overwrite - 等同 --redownload 且 --reinstall (完整覆盖安装)。
#
# 环境变量：
#   PROJECT_WORKDIR - 工作空间根; 远程模式必需; 本地交互默认 deb 目录亦依赖此项。
#   HOST_PLATFORM - arm / x86; 决定 binary 子目录与 deb 架构 (arm64 / amd64)。
#
# 使用示例：
#   download-dependencies
#   download-dependencies -i
#   download-dependencies --path ./deb
#   download-dependencies --overwrite --path ./deb
#   download-dependencies -f ./deb
#
# 注意事项：
#   1. 安装阶段需要 sudo; 内部经 __install-deb-paths: dpkg --force-* + apt-get install -f + dpkg 二次安装。
#   2. 各 vendor deb 自同一 /opt/ros/ai2rob merge-install 拆分, setup 等文件会重复;
#      dpkg 默认互斥, 勿用 apt install ./xxx.deb; 本函数默认 --force-overwrite。
#   3. 目标目录下若已存在同名 deb 且未 --redownload, 跳过下载但仍会安装。
#   4. -i 在未给 --path/DIR 且无 TTY 时报错; 远程 deb 列表见函数内 deb_names。
#
function download-dependencies() {
  local force=0
  local interactive=0
  local redownload=0
  local reinstall=0
  local local_dir=""
  local dpkg_force="overwrite"
  case "${1-}" in
  --help | -h)
    cat >&2 <<'EOF'
download-dependencies — fetch prebuilt .deb packages or install from a local directory.

Usage: download-dependencies [OPTIONS...] [--path DIR | DIR]

  -f, --force       proceed without [Y/n] prompt (default Y)
  -i, --interactive prompt for install source when --path/DIR not given
                    (remote GitLab download, local directory, or skip)
  -p, --path DIR    install all *.deb under DIR (skip remote download)
  DIR               same as --path DIR

Install / download overwrite:
  --redownload      re-fetch debs from remote even if cached locally
  --reinstall       dpkg with stronger --force (overwrite,confnew,confmiss,depends)
  --overwrite       --redownload and --reinstall (full overwrite install)

Default mode (no DIR): download vendor debs from GitLab binary repo and install.
Path mode: traverse DIR for *.deb files (sorted) and install only.

Environment (default mode):
  PROJECT_WORKDIR, HOST_PLATFORM

Interactive: -i picks remote, local, or skip (empty default) on TTY; then Y/n before download/install.
Non-TTY: proceeds without prompt (default Y); -i without DIR requires TTY.
EOF
    return 0
    ;;
  esac
  while [[ $# -gt 0 ]]; do
    case "$1" in
    -f | --force)
      force=1
      shift
      ;;
    -i | --interactive)
      interactive=1
      shift
      ;;
    --redownload)
      redownload=1
      shift
      ;;
    --reinstall)
      reinstall=1
      dpkg_force="overwrite,confnew,confmiss,depends"
      shift
      ;;
    --overwrite)
      redownload=1
      reinstall=1
      dpkg_force="overwrite,confnew,confmiss,depends"
      shift
      ;;
    -p | --path)
      if [[ $# -lt 2 ]]; then
        echo "error: --path requires a directory argument (try --help)" >&2
        return 1
      fi
      local_dir=$2
      shift 2
      ;;
    -*)
      echo "error: unknown option: $1 (try --help)" >&2
      return 1
      ;;
    *)
      if [[ -n "$local_dir" ]]; then
        echo "error: unexpected argument: $1 (try --help)" >&2
        return 1
      fi
      local_dir=$1
      shift
      ;;
    esac
  done

  if [[ $interactive -eq 1 && -z "$local_dir" ]]; then
    if [[ ! -t 0 ]]; then
      echo "error: -i requires a TTY when --path/DIR is not given (try --help)" >&2
      return 1
    fi

    echo -e "\033[33mdownload-dependencies: interactive mode\033[0m" >&2
    local mode
    if declare -f _docker_pick &>/dev/null; then
      mode=$(_docker_pick "install source" "" \
        "remote: download vendor debs from GitLab binary repo" \
        "local: install *.deb from a directory" \
        "skip: do not install") || return 1
    else
      echo "available:" >&2
      echo "  [1] remote: download vendor debs from GitLab binary repo" >&2
      echo "  [2] local: install *.deb from a directory" >&2
      echo "  [3] skip: do not install" >&2
      local pick
      read -r -p "install source (number, empty=skip): " pick </dev/tty || true
      case "${pick}" in
      1 | remote*) mode=remote ;;
      2 | local*) mode=local ;;
      "" | 3 | skip*) mode=skip ;;
      *)
        echo "error: invalid install source: ${pick}" >&2
        return 1
        ;;
      esac
    fi

    case "$mode" in
    "" | skip*)
      echo "download-dependencies: skipped." >&2
      return 0
      ;;
    local*)
      local default_dir=""
      if [[ -n "${PROJECT_WORKDIR:-}" ]]; then
        local _plat="${HOST_PLATFORM:-}"
        [[ -z "$_plat" ]] && case "$(uname -m)" in aarch64) _plat=arm;; x86_64) _plat=x86;; esac
        case "$_plat" in
        arm) default_dir="${PROJECT_WORKDIR}/dependencies/binary/aarch" ;;
        x86) default_dir="${PROJECT_WORKDIR}/dependencies/binary/x86" ;;
        esac
      fi
      read -r -e -p "directory containing .deb files [${default_dir}]: " local_dir </dev/tty || true
      local_dir=${local_dir:-$default_dir}
      if [[ -z "$local_dir" ]]; then
        echo "error: directory is required for local install." >&2
        return 1
      fi
      ;;
    esac
  fi

  if [[ -n "$local_dir" ]]; then
    if [[ ! -d "$local_dir" ]]; then
      echo "error: not a directory: ${local_dir}" >&2
      return 1
    fi

    local -a deb_paths=()
    mapfile -t deb_paths < <(find "$local_dir" -maxdepth 1 -type f -name '*.deb' | sort)
    if [[ ${#deb_paths[@]} -eq 0 ]]; then
      echo "error: no .deb files found under ${local_dir}" >&2
      return 1
    fi

    echo "download-dependencies: install from ${local_dir}"
    local path
    for path in "${deb_paths[@]}"; do
      echo "  - $(basename "$path")"
    done

    if [[ $force -eq 0 && -t 0 ]]; then
      local ans
      read -r -p "Install these debs? [Y/n] " ans </dev/tty || true
      case "${ans,,}" in
      n | no)
        echo "download-dependencies: aborted." >&2
        return 1
        ;;
      esac
    fi

    __install-deb-paths "$dpkg_force" "${deb_paths[@]}"
    echo "done: installed ${#deb_paths[@]} package(s) from ${local_dir}"
    return 0
  fi

  if [[ -z "${PROJECT_WORKDIR:-}" ]]; then
    echo "error: PROJECT_WORKDIR is not set." >&2
    return 1
  fi

  local plat="${HOST_PLATFORM:-}"
  [[ -z "$plat" ]] && case "$(uname -m)" in aarch64) plat=arm;; x86_64) plat=x86;; *) plat=unknown;; esac

  declare -A _subdir=([arm]=aarch [x86]=x86)
  declare -A _deb_arch=([arm]=arm64 [x86]=amd64)

  if [[ -z "${_subdir[$plat]+_}" ]]; then
    echo "error: unsupported HOST_PLATFORM=${plat}" >&2
    return 1
  fi

  local sub="${_subdir[$plat]}"
  local darch="${_deb_arch[$plat]}"

  local -a deb_names=(
    "ros-ai2rob-algorithm-vendor_0.1.0-1_${darch}.deb"
    "ros-ai2rob-controls-vendor_0.1.0-1_${darch}.deb"
    "ros-ai2rob-navigation-vendor_0.1.0-1_${darch}.deb"
    "ros-ai2rob-onnxruntime-vendor_1.26.0-1_${darch}.deb"
    "ros-ai2rob-smacc-vendor_0.1.0-1_${darch}.deb"
    "ros-ai2rob-tools-vendor_0.1.0-1_${darch}.deb"
    "ros-ai2rob-vision-vendor_0.1.0-1_${darch}.deb"
  )

  local deps="${PROJECT_WORKDIR}/dependencies"
  local deps_bin="${deps}/binary"
  local plat_dir="${deps_bin}/${sub}"
  local base_url="https://gitlab.ai2rob.com/zhiqi.jia/binary/-/raw/main"

  echo "download-dependencies: ${sub}/ (${darch}) -> ${plat_dir}"
  local name
  for name in "${deb_names[@]}"; do
    echo "  - ${name}"
  done

  if [[ $force -eq 0 && -t 0 ]]; then
    local ans
    read -r -p "Download and install these debs? [Y/n] " ans </dev/tty || true
    case "${ans,,}" in
    n | no)
      echo "download-dependencies: aborted." >&2
      return 1
      ;;
    esac
  fi

  mkdir -p "$plat_dir"
  touch "${deps}/COLCON_IGNORE"
  touch "${deps_bin}/COLCON_IGNORE"

  local name url path
  for name in "${deb_names[@]}"; do
    url="${base_url}/${sub}/${name}"
    path="${plat_dir}/${name}"
    if [[ -e "$path" && $redownload -eq 0 ]]; then
      echo "skipping download: ${path} already exists"
    else
      if [[ -e "$path" ]]; then
        echo "re-downloading (overwrite): ${url} ..."
      else
        echo "downloading ${url} ..."
      fi
      curl -fSL -o "$path" "$url" || { echo "error: download failed for ${name}" >&2; return 1; }
    fi
  done

  local -a deb_paths=()
  for name in "${deb_names[@]}"; do
    deb_paths+=("${plat_dir}/${name}")
  done

  __install-deb-paths "$dpkg_force" "${deb_paths[@]}"

  echo "done: installed ${#deb_names[@]} package(s) from ${plat_dir}"
}

# Private helper for colcon_ignore: touch ${PROJECT_WORKDIR}/src/.colcon-ignore-$1 and
# COLCON_IGNORE under ./src paths matching */$2 (maxdepth 6).
__ignore-directory() {
  local ignore_file="${PROJECT_WORKDIR}/src/.colcon-ignore-$1"
  local path_pattern="*/$2"

  touch "$ignore_file"
  echo -e "\033[1;31mYou should \`touch $2/COLCON_IGNORE\`\033[0m"

  for dir in $(find ./src -type d -maxdepth 6 -path "$path_pattern" 2>/dev/null); do
    if [[ -d "$dir" ]]; then
      if ! touch "$dir/COLCON_IGNORE"; then
        echo -e "\033[1;31mWarning: Failed to create COLCON_IGNORE in $dir.\033[0m" >&2
      else
        echo "Created COLCON_IGNORE in $dir"
      fi
    fi
  done
}

## @function colcon_ignore
#  @brief Ignore specific colcon packages by creating COLCON_IGNORE files.
#
#  This function ignores a predefined list of packages by creating
#  COLCON_IGNORE files in their respective directories.
#
#  Example usage:
#  @code
#  colcon_ignore
#  @endcode
function colcon_ignore() {
  declare -A IGNORE_LIST=(
    [smacc]="smacc2_sm_reference_library" # have gazebo
    [ros2-control-demo-example]="ros2-control-demos" # have gazebo
    [simulation]="src/simulation" # have gazebo
    [opencv-3rdparty]="opencv/3rdparty"
    [opencv_contrib]="opencv_contrib"
    [nav2_system_tests]="nav2_system_tests"
  )

  # Iterate over the ignore list
  for key in "${!IGNORE_LIST[@]}"; do
    __ignore-directory "$key" "${IGNORE_LIST[$key]}"
  done
}
