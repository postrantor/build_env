# ---
# date: 2025-01-12
# update: 2026-04-13 16:54:56
# author: postrantor@gmail.com
# description: robot env entry
# ---


## use `cd ...` equal `cd ../..`
# only for bash
if [ -n "$BASH_VERSION" ]; then
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
fi

## alias rm to confirm deletion
# rm() {
#   command rm "$@" --interactive=always
# }

## alias rsync to show progress
rsync-progress() {
  command rsync \
    --archive \
    --verbose \
    --human-readable \
    --progress \
    --partial \
    --append-verify \
    --info=progress1 \
    "$@"
}

## apt
# `Defaults env_keep += "APT_CONFIG"`
#export APT_CONFIG="${PROJECT_WORKDIR}/.env/apt.conf"

## set machine name
export MACHINE="${HOSTNAME}"
if [ -f /.dockerenv ]; then
  MACHINE="${MACHINE}/docker"
  export MACHINE
fi

## Host CPU family and NVIDIA GPU visibility (for colcon paths, docker run, etc.)
host_hw_detect() {
  export HOST_UNAME_M
  HOST_UNAME_M=$(uname -m)
  case "${HOST_UNAME_M}" in
  aarch64) export HOST_PLATFORM=arm ;;
  x86_64) export HOST_PLATFORM=x86 ;;
  *) export HOST_PLATFORM=unknown ;;
  esac

  local _gpu_count
  _gpu_count=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | grep -c . || true)
  if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null && [[ "${_gpu_count:-0}" -gt 0 ]]; then
    export HOST_NVIDIA_GPU=1
  else
    export HOST_NVIDIA_GPU=0
  fi
}
# Print HOST_PLATFORM and HOST_NVIDIA_GPU (after host_hw_detect)
host_hw_info() {
  case "${HOST_PLATFORM:-unknown}" in
  arm)
    echo -e "\033[35mHOST_PLATFORM=${HOST_PLATFORM}  HOST_NVIDIA_GPU=${HOST_NVIDIA_GPU}\033[0m"
    ;;
  unknown)
    echo -e "\033[31mHOST_PLATFORM=${HOST_PLATFORM}  HOST_NVIDIA_GPU=${HOST_NVIDIA_GPU}\033[0m"
    ;;
  *)
    echo "HOST_PLATFORM=${HOST_PLATFORM}  HOST_NVIDIA_GPU=${HOST_NVIDIA_GPU}"
    ;;
  esac
}
host_hw_detect

## 关闭硬件/软件流控，解决粘连问题
# 可以解绑 ctrl+s
# stty -ixon -ixoff

## ROS 2 distro detection (jammy→humble, noble→jazzy) and common env vars
# Exported here so that common/setup/install.bash, robot.bash, etc. share a single source of truth.
# shellcheck source=/dev/null
. /etc/os-release 2>/dev/null || true
case "${VERSION_CODENAME:-}" in
  jammy) export ROS_DISTRO=humble ;;
  noble) export ROS_DISTRO=jazzy  ;;
  *)     export ROS_DISTRO=""     ;;
esac
export ROS_VERSION=2
export ROS_PYTHON_VERSION=3
export ROS_LOG_DIR="${HOME}/.ros/log/launch"
export RCUTILS_COLORIZED_OUTPUT=1         # 1: coloured log  0: plain (avoids ANSI codes in files)
export RCUTILS_LOGGING_USE_STDOUT=0       # force stderr so log infra can capture correctly
export RCUTILS_LOGGING_BUFFERED_STREAM=1  # ensure logs are flushed to files promptly
export TERM=xterm-256color

## alias to source ~/.bashrc
alias sb="source ~/.bashrc"
