# ---
# date: 2026-04-13
# update: 2026-08-21
# author: zhiqi.jia@ai2robotics.com
# description: docker helpers (container lifecycle, image publishing, daemon.json)
# ---


# _docker_default_container
#
# 功能描述：
#   返回默认容器名, 来自 `PROJECT_CONTAINER` (缺省 `auriga_cpp`)。
#
# 参数：
#   (无)
#
_docker_default_container() {
  printf '%s' "${PROJECT_CONTAINER:-auriga_cpp}"
}

# _docker_default_workspace
#
# 功能描述：
#   返回默认 WORKSPACE 段, 来自 `PROJECT_NAME` (缺省 `ai2-robot`)。
#
# 参数：
#   (无)
#
_docker_default_workspace() {
  printf '%s' "${PROJECT_NAME:-ai2-robot}"
}

# _docker_pick
#
# 功能描述：
#   交互式列表或自由输入选择器。候选项可来自位置参数或 stdin pipe; 用户输入始终从 `/dev/tty` 读取。
#   选中值写到 stdout, 提示与列表写到 stderr。
#
# 参数：
#   $1: prompt - 提示语。
#   $2: default - 空输入时的默认值。
#   $@: items - 可选候选项; 也可从 stdin pipe 读入。
#
# 使用示例：
#   result=$(_docker_pick "prompt" "default" item1 item2 ...)
#   result=$(cmd | _docker_pick "prompt" "default")
#
# 注意事项：
#   1. 数字输入越界时返回 1。
#
_docker_pick() {
  local _prompt=$1 _default=$2
  shift 2
  local -a _items=("$@")
  local _hint _input _i

  # 无 TTY 时从 stdin 读候选项 (支持 pipe); 交互始终走 /dev/tty, 避免与 pipe 冲突
  [[ -t 0 ]] || mapfile -t -O "${#_items[@]}" _items

  if ((${#_items[@]})); then
    echo "available:" >&2
    for _i in "${!_items[@]}"; do
      printf '  [%d] %s\n' "$((_i + 1))" "${_items[_i]}" >&2
    done
    _hint="number, free input, empty = ${_default:-auto}"
  else
    _hint="empty = ${_default:-auto}"
  fi

  read -r -e -p "${_prompt} (${_hint}): " _input </dev/tty

  # 空输入 → 默认值；数字 → 列表索引；否则视为自由输入
  if [[ -z "$_input" ]]; then
    echo "${_default}"
    return 0
  fi

  if [[ "$_input" =~ ^[0-9]+$ ]] && ((${#_items[@]})); then
    if ((_input >= 1 && _input <= ${#_items[@]})); then
      echo "${_items[$((_input - 1))]}"
      return 0
    fi
    echo "_docker_pick: index out of range: $_input (1..${#_items[@]})" >&2
    return 1
  fi

  echo "$_input"
}

# _docker_confirm_pull_image
#
# 功能描述：
#   创建容器前按模式决定是否 `docker pull` 所选镜像。
#
# 参数：
#   $1: image - 镜像引用。
#   $2: pull_mode - `yes` 直接拉取; `no` 跳过; 空则 TTY 询问 (默认 Y), 非 TTY 默认拉取。
#
# 注意事项：
#   1. 选择拉取且 `docker pull` 失败时返回非 0。
#
_docker_confirm_pull_image() {
  local image=$1
  local pull_mode=${2:-}
  [[ -n "$image" ]] || return 0

  # pull_mode: yes/no 跳过询问；未指定时 TTY 交互 (默认 Y)，非 TTY 默认 pull
  case "$pull_mode" in
  no)
    echo "container-create: skipped pull; using local image \"$image\"." >&2
    return 0
    ;;
  yes) ;;
  *)
    if [[ -t 0 ]]; then
      local ans
      read -r -p "Pull \"$image\" to update from registry? [Y/n] " ans </dev/tty || true
      case "${ans,,}" in
      n | no)
        echo "container-create: skipped pull; using local image \"$image\"." >&2
        return 0
        ;;
      esac
    fi
    ;;
  esac

  echo "container-create: pulling \"$image\"..." >&2
  if ! docker pull "$image"; then
    echo "container-create: docker pull failed for \"$image\"" >&2
    return 1
  fi
}

# container-entry
#
# 功能描述：
#   启动已存在的 Docker 容器, 并以指定用户交互式进入 bash (`docker start` + `docker exec`)。
#
# 参数：
#   -i | --interactive - 交互式输入 CONTAINER、USER (与下列位置参数二选一)。
#   $1: container - 容器名称 (默认: auriga_cpp)。
#   $2: user - 容器内用于 exec 的用户名 (默认: `${DEVCONTAINER_USERNAME:-$USER}`)。
#
# 使用示例：
#   container-entry
#   container-entry -i
#   container-entry my_ros_container
#   container-entry my_ros_container ubuntu
#
# 注意事项：
#   1. 需已安装 Docker, 且容器曾成功创建 (名称与 `$1` 一致)。
#   2. 该函数不创建新容器; 新建容器请使用 `container-create` 或自行 `docker run`。
#
container-entry() {
  case "${1-}" in
  --help | -h)
    cat >&2 <<'EOF'
container-entry — start an existing container and attach with bash (docker start; docker exec -it).

Usage: container-entry [-i|--interactive]
       container-entry [CONTAINER] [USER]

  -i, --interactive   prompt for CONTAINER and USER (defaults in brackets)

  CONTAINER   name (default: auriga_cpp)
  USER        exec user, -u (default: $USER; DEVCONTAINER_USERNAME wins if set)

Examples:
  container-entry
  container-entry -i
  container-entry my_ros_container ubuntu

See also: container-create, container-remove (--help on each)
EOF
    return 0
    ;;
  esac

  local container user
  local default_user=${DEVCONTAINER_USERNAME:-$USER}

  # -i 从本地容器列表选取；否则用位置参数
  case "${1-}" in
  -i | --interactive)
    shift
    echo -e "\033[33mcontainer-entry: interactive mode\033[0m" >&2
    container=$(docker ps -a --format '{{.Names}}' 2>/dev/null | sort -u |
      _docker_pick "container name" "$(_docker_default_container)") || return 1
    read -r -e -p "user in container [${default_user}]: " user
    user=${user:-$default_user}
    ;;
  *)
    container=${1:-$(_docker_default_container)}
    user=${2:-$default_user}
    ;;
  esac

  # 已停止的容器也可 start；exec 以指定用户进入 bash
  docker start "$container" >/dev/null &&
    docker exec -it -u "$user" "$container" bash
}

# container-remove
#
# 功能描述：
#   按容器名检查是否存在对应实例; 若存在, 在终端交互确认后执行 `docker stop` 与 `docker rm`。无容器则直接成功返回。
#   可用于释放占用名称、清理旧环境等任意场景, 不限于再次 `docker run`。
#
# 参数：
#   -f | --force - 若容器存在则直接 stop/rm, 不询问 (适合脚本与非交互场景)。
#   -i | --interactive - 交互式输入要删除的容器名 (与下列位置参数二选一)。
#   $1: container_name - 容器名称 (与 `docker ps -a --format '{{.Names}}'` 中一致)。
#
# 使用示例：
#   container-remove -i
#   container-remove -f auriga_cpp
#   container-remove auriga_cpp || return 1
#
# 注意事项：
#   1. 返回 0: 无此名容器, 或用户确认并已删除成功。返回 1: 用户拒绝、非交互环境无法询问、或 `docker rm` 失败。
#   2. 需标准输入为终端 (isatty); 否则在「容器已存在」时返回 1, 避免非交互脚本误删。
#   3. 确认后先 `docker stop` (忽略失败, 兼容已停止状态), 再 `docker rm`。
#
container-remove() {
  case "${1-}" in
  --help | -h)
    cat >&2 <<'EOF'
container-remove — if a container exists, prompt to stop and remove it; if none, succeed.

Usage: container-remove [OPTIONS] [NAME]
       container-remove -i

  -f, --force         stop and remove without prompting if the container exists
  -i, --interactive   prompt for container NAME (default: auriga_cpp)

Exit: 0 = no such container or removed; 1 = declined, non-interactive stdin, or rm failed.

Note: without -f, prompts only when stdin is a TTY (default Y; n/no to abort); otherwise exits 1 if the container exists.

Examples:
  container-remove -i
  container-remove -f auriga_cpp
  container-remove auriga_cpp
  container-remove auriga_cpp || return 1

See also: container-entry, container-create (--help on each)
EOF
    return 0
    ;;
  esac

  local force=0 interactive=0 container_name=""

  # 解析 -f / -i 与容器名
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
    --)
      shift
      break
      ;;
    -*)
      echo "container-remove: unknown option: $1" >&2
      return 1
      ;;
    *)
      if [[ -n "$container_name" ]]; then
        echo "container-remove: unexpected argument: $1" >&2
        return 1
      fi
      container_name=$1
      shift
      ;;
    esac
  done

  if [[ $interactive -eq 1 ]]; then
    echo -e "\033[33mcontainer-remove: interactive mode\033[0m" >&2
    container_name=$(docker ps -a --format '{{.Names}}' 2>/dev/null | sort -u |
      _docker_pick "container to remove" "$(_docker_default_container)") || return 1
  fi

  if [[ -z "$container_name" ]]; then
    echo "container-remove: missing container name (use: container-remove NAME or container-remove -i)" >&2
    return 1
  fi

  # 不存在则静默成功; 存在时 -f 直接删, 否则 TTY 确认 (非 TTY 拒绝)
  if ! docker container inspect "$container_name" &>/dev/null; then
    return 0
  fi

  if [[ $force -eq 1 ]]; then
    docker stop "$container_name" &>/dev/null || true
    if ! docker rm "$container_name"; then
      echo "container-remove: docker rm failed for \"$container_name\"" >&2
      return 1
    fi
    echo "removed container \"$container_name\"." >&2
    return 0
  fi

  if [[ ! -t 0 ]]; then
    echo "container-remove: container \"$container_name\" exists (stdin is not a tty; refusing to prompt)" >&2
    return 1
  fi

  echo -e "\033[33ma container named \"$container_name\" already exists.\033[0m" >&2
  local ans
  read -r -p "stop and remove this container? [Y/n] " ans
  case "${ans,,}" in
  n | no)
    echo "container-remove: aborted (container still exists)." >&2
    return 1
    ;;
  esac

  docker stop "$container_name" &>/dev/null || true
  if ! docker rm "$container_name"; then
    echo "container-remove: docker rm failed for \"$container_name\"" >&2
    return 1
  fi
  echo "removed container \"$container_name\"." >&2
}

# container-create
#
# 功能描述：
#   以 docker run 启动交互式开发容器：挂载 workspace-entrypoint.sh 为 --entrypoint，
#   挂载宿主机 workspaces 至 /workspaces，挂载 entrypoint_additions；
#   根据 HOST_PLATFORM 与 HOST_NVIDIA_GPU 决定是否附加 GPU 相关选项。
#   挂载与入口脚本路径规则见 docs/usage/docker.md §2。
#
# 参数：
#   -f | --force - 所有 Y/n 交互取默认值、不询问: 移除同名容器 Y; pull 默认 Y (--no-pull 改为默认 n)。
#   --pull - 创建前拉取镜像，不询问 (等同 pull 默认 Y)。
#   --no-pull - 跳过镜像拉取，不询问 (等同 pull 默认 n)。
#   --auto-start - 传入容器 AUTO_START=1 (entrypoint 60.entry.user.sh 等)。
#   --robot-type TYPE - 传入容器 ROBOT_TYPE (entry launch robot_type, 默认 bot2)。
#   --robot-id ID - 传入容器 ROBOT_ID (entry launch robot_id, 默认 0156)。
#   --recipe FILE - 传入容器 RECIPE (entry launch recipe, 默认 auriga.sm.real.yaml)。
#   --log-level LEVEL - 传入容器 LOG_LEVEL (entry launch log_level, 默认 info)。
#   -i | --interactive - 交互式输入 NAME、IMAGE、WORKSPACES、WORKSPACE_ENV (与下列位置参数二选一)。
#   $1: container_name - `docker run` 的 `--name` (默认: `${PROJECT_CONTAINER}`)。
#   $2: image - 可选, 完整镜像引用; 省略或空字符串时使用 `ROS2_RELEASE_IMAGE`, 或 `ROS2_REGISTRY` 下的 `ros2-release:${ROS2_RELEASE_VERSION:-v0.9.3}`。
#   $3: workspaces - 宿主机 workspaces 根目录 (默认: `${HOME}/workspaces`)。
#   $4: workspace_env - 传入容器的 WORKSPACE 环境变量, 单层目录名 (默认: `${PROJECT_NAME}`)。
#
# 使用示例：
#   container-create
#   container-create -i
#   container-create -f --no-pull auriga_cpp "" "${HOME}/workspaces" ai2-robot
#   container-create auriga_cpp "docker.example.com/ros:dev" "${HOME}/Development/workspaces" ai2-robot
#   container-create -f --auto-start --robot-id 0156 --robot-type bot2 --recipe auriga.sm.real.yaml auriga_cpp
#
# 注意事项：
#   1. 依赖 shell 中已导出的 `HOST_PLATFORM`、`HOST_NVIDIA_GPU` (通常由登录环境加载)。
#   2. 若容器名已占用: 无 `-f` 时 `container-remove` 交互确认 [Y/n]; `-f` 时默认 Y (直接 stop/rm)。
#   3. 使用 GPU 选项时需宿主机已配置 NVIDIA 容器运行时; 无 GPU 时不传递 NVIDIA 相关 `-e`。
#   4. entrypoint 与 `entrypoint_additions` 依赖宿主机路径存在, 扩展脚本需可执行; 详见 docs/usage/docker.md §2.4–§2.7。
#   5. pull: 无 `-f` 且未指定 `--pull`/`--no-pull` 时 TTY 询问 [Y/n] (默认 Y); `-f` 时默认 Y; `--no-pull` 默认 n。
#   6. jtop (jetson-stats): 若宿主机存在 `/run/jtop.sock` (需主机上 jtop.service 运行), 则挂载进容器;
#      `workspace-entrypoint.sh` 会按 socket 的 GID 创建 jtop 组并将容器用户加入, 无需 `docker run --group-add`。
#      镜像内需安装 jetson-stats。详见 docs/system/config-jtop.md。
#   7. `DEVCONTAINER_USERNAME` 若设置, 覆盖容器内 USERNAME 及 `~/.ssh` 挂载路径所用用户; 否则使用 `$USER`。
#
container-create() {
  case "${1-}" in
  --help | -h)
    cat >&2 <<'EOF'
container-create — run an interactive dev container (privileged, host network, workspace entrypoint).

Usage: container-create [OPTIONS] [NAME] [IMAGE] [WORKSPACES] [WORKSPACE_ENV]
       container-create -i

Options:
  -f, --force         accept default answers for all Y/n prompts (remove existing: Y;
                      pull: Y unless --no-pull); no interactive prompts
  --pull              pull image before run (default Y; no prompt)
  --no-pull           skip image pull (default n; no prompt)
  --auto-start        pass AUTO_START=1 into container (entrypoint 60.entry.user.sh, etc.)
  --robot-type TYPE   pass ROBOT_TYPE=TYPE (entry launch robot_type; default bot2 in 60.entry.user.sh)
  --robot-id ID       pass ROBOT_ID=ID (entry launch robot_id; default 0156 in 60.entry.user.sh)
  --recipe FILE       pass RECIPE=FILE (entry launch recipe; default auriga.sm.real.yaml)
  --log-level LEVEL   pass LOG_LEVEL=LEVEL (entry launch log_level; default info)
  -i, --interactive   prompt for NAME, IMAGE, WORKSPACES, WORKSPACE_ENV (defaults in brackets);
                      IMAGE prompt lists local docker images and accepts a number, full ref, or empty

  NAME           --name (default: auriga_cpp)
  IMAGE          full reference; omit or "" for ROS2_RELEASE_IMAGE, else ROS2_REGISTRY/auriga/ros2-release:$ROS2_RELEASE_VERSION
  WORKSPACES     host dir → /workspaces (default: ${HOME}/workspaces)
  WORKSPACE_ENV  WORKSPACE value, one path segment (default: ai2-robot)

GPU: adds --gpus all on x86 when HOST_NVIDIA_GPU=1, or --runtime nvidia on ARM when HOST_NVIDIA_GPU=1.

Env: DEVCONTAINER_USERNAME overrides container user and ~/.ssh mount path (else $USER).
     ROS2_RELEASE_IMAGE overrides the default image; ROS2_REGISTRY defaults to docker.ai2rob.com:3443;
     ROS2_RELEASE_VERSION defaults to v0.9.3.

Entrypoint and bind-mount paths: see docs/usage/docker.md §2.

jtop: If host has /run/jtop.sock, bind-mounts it; workspace-entrypoint.sh sets jtop group from socket GID. Install jetson-stats in the image; see docs/system/config-jtop.md.

If NAME is taken: prompts [Y/n] unless -f (default Y). Pull: prompts [Y/n] on TTY unless -f, --pull, or --no-pull (-f defaults to pull Y; --no-pull overrides to n).

Examples:
  (recommended) container-create auriga_cpp
  container-create -i
  container-create -f auriga_cpp "" "${HOME}/workspaces" ai2-robot
  container-create -f --no-pull auriga_cpp "" "${HOME}/workspaces" ai2-robot
  ROS2_REGISTRY=docker.postrantor.cn:5000 container-create auriga_cpp
  container-create -f --auto-start --robot-id 0156 --robot-type bot2 --recipe auriga.sm.real.yaml auriga_cpp

See also: container-entry, container-remove (--help on each)
EOF
    return 0
    ;;
  esac

  local force=0 interactive=0 pull_mode=""
  local auto_start=0
  local robot_type="" robot_id="" entry_recipe="" log_level=""

  # 解析 -f / --pull / --no-pull / --auto-start / entry 参数 / -i
  while [[ $# -gt 0 ]]; do
    case "$1" in
    -f | --force)
      force=1
      shift
      ;;
    --auto-start)
      auto_start=1
      shift
      ;;
    --robot-type)
      if [[ -z "${2-}" ]]; then
        echo "container-create: --robot-type requires a value" >&2
        return 1
      fi
      robot_type=$2
      shift 2
      ;;
    --robot-id)
      if [[ -z "${2-}" ]]; then
        echo "container-create: --robot-id requires a value" >&2
        return 1
      fi
      robot_id=$2
      shift 2
      ;;
    --recipe)
      if [[ -z "${2-}" ]]; then
        echo "container-create: --recipe requires a value" >&2
        return 1
      fi
      entry_recipe=$2
      shift 2
      ;;
    --log-level)
      if [[ -z "${2-}" ]]; then
        echo "container-create: --log-level requires a value" >&2
        return 1
      fi
      log_level=$2
      shift 2
      ;;
    --pull)
      if [[ "$pull_mode" == no ]]; then
        echo "container-create: --pull and --no-pull are mutually exclusive" >&2
        return 1
      fi
      pull_mode=yes
      shift
      ;;
    --no-pull)
      if [[ "$pull_mode" == yes ]]; then
        echo "container-create: --pull and --no-pull are mutually exclusive" >&2
        return 1
      fi
      pull_mode=no
      shift
      ;;
    -i | --interactive)
      interactive=1
      shift
      ;;
    --)
      shift
      break
      ;;
    -*)
      echo "container-create: unknown option: $1" >&2
      return 1
      ;;
    *)
      break
      ;;
    esac
  done

  local container_name workspaces workspace_env img
  local cuser=${DEVCONTAINER_USERNAME:-$USER}
  local default_ws="${HOME}/workspaces"

  # 收集 NAME / IMAGE / WORKSPACES / WORKSPACE_ENV
  if [[ $interactive -eq 1 ]]; then
    echo -e "\033[33mcontainer-create: interactive mode\033[0m" >&2
    read -r -e -p "container name [$(_docker_default_container)]: " container_name
    container_name=${container_name:-$(_docker_default_container)}

    img=$(docker images --format '{{.Repository}}:{{.Tag}}' 2>/dev/null |
      grep -v '^<none>:<none>$' | sort -u |
      _docker_pick "image" "") || return 1

    read -r -e -p "workspaces root [${default_ws}]: " workspaces
    workspaces=${workspaces:-$default_ws}
    read -r -e -p "workspace env segment [$(_docker_default_workspace)]: " workspace_env
    workspace_env=${workspace_env:-$(_docker_default_workspace)}
  else
    container_name=${1:-$(_docker_default_container)}
    img=${2:-}
    workspaces=${3:-$default_ws}
    workspace_env=${4:-$(_docker_default_workspace)}
  fi

  # -f: 所有 Y/n 取默认值; pull 默认 Y, --no-pull 覆盖为 n
  if [[ $force -eq 1 && -z "$pull_mode" ]]; then
    pull_mode=yes
  fi

  # 释放同名容器 (-f → 默认 Y, 不询问)
  if [[ $force -eq 1 ]]; then
    container-remove -f "$container_name" || return 1
  else
    container-remove "$container_name" || return 1
  fi

  # IMAGE 为空时使用 multi-platform release 镜像，Docker 按宿主机选择平台
  if [[ -z "$img" ]]; then
    local release_registry=${ROS2_REGISTRY:-docker.ai2rob.com:3443}
    while [[ "$release_registry" == */ ]]; do
      release_registry=${release_registry%/}
    done
    img=${ROS2_RELEASE_IMAGE:-"${release_registry}/auriga/ros2-release:${ROS2_RELEASE_VERSION:-v0.9.3}"}
  fi

  _docker_confirm_pull_image "$img" "$pull_mode" || return 1

  # bind-mount 路径须与 workspace-entrypoint.sh 内 glob 一致，见 docs/usage/docker.md §2.4
  local entrypoint_additions="${workspaces}/build_env/${workspace_env}/.devcontainer/entrypoint_additions"
  local entrypoint_host="${workspaces}/build_env/${workspace_env}/.devcontainer/ai2/workspace-entrypoint.sh"
  local entrypoint_path="/usr/local/bin/workspace-entrypoint.sh"
  local entrypoint_additions_path="/usr/local/bin/entrypoint_additions"

  local -a cmd=(
    docker run -it --privileged --network host --ipc host
    --name "$container_name"
    --entrypoint "$entrypoint_path"
  )

  # GPU：x86 用 --gpus，ARM 用 nvidia runtime
  local plat=${HOST_PLATFORM:-unknown}
  local gpu=${HOST_NVIDIA_GPU:-0}
  if [[ "$plat" == "x86" && "$gpu" == "1" ]]; then
    cmd+=(--gpus all -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all)
  elif [[ "$plat" == "arm" && "$gpu" == "1" ]]; then
    cmd+=(--runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all)
  fi

  # Jetson jtop：宿主机 socket 存在时挂载，entrypoint 内按 GID 建组
  if [[ -S /run/jtop.sock ]]; then
    cmd+=(-v /run/jtop.sock:/run/jtop.sock)
  fi

  # 用户/工作区环境变量与卷挂载 (X11、SSH、workspaces、entrypoint)
  cmd+=(
    -e DISPLAY
    -e USERNAME="$cuser"
    -e HOST_USER_UID="$(id -u)"
    -e HOST_USER_GID="$(id -g)"
    -e WORKSPACE="$workspace_env"
    -v /dev:/dev
    -v /tmp/.X11-unix:/tmp/.X11-unix
    -v "${HOME}/.Xauthority:/tmp/.Xauthority:ro"
    -e XAUTHORITY=/tmp/.Xauthority
    -v "${HOME}/.ssh:/home/${cuser}/.ssh"
    -v "${workspaces}:/workspaces"
    -v "${entrypoint_additions}:${entrypoint_additions_path}:ro"
    -v "${entrypoint_host}:${entrypoint_path}:ro"
  )

  cmd+=(-e UPDATE=false)
  [[ $auto_start -eq 1 ]] && cmd+=(-e AUTO_START=1)
  [[ -n "$robot_type" ]] && cmd+=(-e "ROBOT_TYPE=${robot_type}")
  [[ -n "$robot_id" ]] && cmd+=(-e "ROBOT_ID=${robot_id}")
  [[ -n "$entry_recipe" ]] && cmd+=(-e "RECIPE=${entry_recipe}")
  [[ -n "$log_level" ]] && cmd+=(-e "LOG_LEVEL=${log_level}")

  cmd+=("$img" /bin/bash)

  "${cmd[@]}"
}

# image helpers: 构建并发布带版本的 ROS 2 base/release 镜像。默认值如下; `--config` 可 source 私有覆盖文件。

# _image_error
#
# 功能描述：
#   向 stderr 打印 `image:` 前缀的错误信息。
#
# 参数：
#   $@: message - 错误内容。
#
_image_error() {
  echo "image: $*" >&2
}

# _image_workdir
#
# 功能描述：
#   解析镜像构建所用工作目录 (`PROJECT_WORKDIR` / `AI2_WORKDIR`, 否则回退到本文件上级目录)。
#
# 参数：
#   (无)
#
_image_workdir() {
  if [[ -n "${PROJECT_WORKDIR:-}" ]]; then
    printf '%s\n' "${PROJECT_WORKDIR}"
    return 0
  fi
  if [[ -n "${AI2_WORKDIR:-}" ]]; then
    printf '%s\n' "${AI2_WORKDIR}"
    return 0
  fi

  local env_dir
  env_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  (cd "${env_dir}/.." && pwd)
}

# _image_normalize_registry
#
# 功能描述：
#   去掉 registry 尾部多余 `/`, 空值时回落到默认仓库主机。
#
# 参数：
#   $1: registry - 仓库主机 (可带端口)。
#
_image_normalize_registry() {
  local registry=$1

  while [[ "$registry" == */ ]]; do
    registry=${registry%/}
  done
  : "${registry:=docker.ai2rob.com:3443}"
  printf '%s\n' "$registry"
}

# _image_apply_registry
#
# 功能描述：
#   按 registry 成对设置 `IMAGE_BASE_REPOSITORY` 与 `IMAGE_RELEASE_REPOSITORY`。
#
# 参数：
#   $1: registry - 仓库主机 (可带端口)。
#
_image_apply_registry() {
  local registry
  registry=$(_image_normalize_registry "$1")

  IMAGE_REGISTRY=$registry
  IMAGE_BASE_REPOSITORY="${registry}/auriga/ros2-base"
  IMAGE_RELEASE_REPOSITORY="${registry}/auriga/ros2-release"
}

# _image_load_config
#
# 功能描述：
#   可选 source 配置文件, 并补齐镜像仓库 / CUDA 基础镜像 / Dockerfile 默认值。
#
# 参数：
#   $1: config - 可选配置文件路径; 空则只用内置默认值。
#
_image_load_config() {
  local config=${1:-}

  if [[ -n "$config" ]]; then
    if [[ ! -f "$config" ]]; then
      _image_error "configuration file not found: $config"
      return 1
    fi

    # shellcheck source=/dev/null
    source "$config"
  fi

  : "${IMAGE_REGISTRY:=docker.ai2rob.com:3443}"
  IMAGE_REGISTRY=$(_image_normalize_registry "$IMAGE_REGISTRY")
  : "${IMAGE_BASE_REPOSITORY:=${IMAGE_REGISTRY}/auriga/ros2-base}"
  : "${IMAGE_RELEASE_REPOSITORY:=${IMAGE_REGISTRY}/auriga/ros2-release}"
  : "${IMAGE_BASE_IMAGE_ARM64:=nvcr.io/nvidia/12.6.11-devel:12.6.11-devel-aarch64-ubuntu22.04}"
  : "${IMAGE_BASE_IMAGE_AMD64:=nvidia/cuda:12.6.3-cudnn-devel-ubuntu22.04}"
  : "${IMAGE_TENSORRT_VERSION:=10.7.0.23-1+cuda12.6}"
  : "${IMAGE_DOCKER_CONTEXT:=$(_image_workdir)/.devcontainer/ai2}"
  : "${IMAGE_BASE_DOCKERFILE:=${IMAGE_DOCKER_CONTEXT}/base.dockerfile}"
  : "${IMAGE_RELEASE_DOCKERFILE:=${IMAGE_DOCKER_CONTEXT}/release.dockerfile}"
}

# _image_require_buildx
#
# 功能描述：
#   检查 `docker` 与 `docker buildx` 是否可用。
#
# 参数：
#   (无)
#
_image_require_buildx() {
  if ! command -v docker >/dev/null 2>&1; then
    _image_error "docker is not installed"
    return 1
  fi

  if ! docker buildx version >/dev/null 2>&1; then
    _image_error "docker buildx is required; install docker-buildx-plugin"
    return 1
  fi
}

# _image_validate_version
#
# 功能描述：
#   校验版本字符串是否为 `v<major>.<minor>.<patch|x>` (可带预发布后缀)。
#
# 参数：
#   $1: version - 版本 tag。
#
_image_validate_version() {
  local version=$1

  # Allow numeric patch or floating "x" (e.g. v0.9.3, v0.9.x).
  if [[ ! "$version" =~ ^v[0-9]+\.[0-9]+\.([0-9]+|x)(-[0-9A-Za-z][0-9A-Za-z._-]*)?$ ]]; then
    _image_error "invalid version \"$version\"; expected v<major>.<minor>.<patch|x>"
    return 1
  fi
}

# _image_validate_role
#
# 功能描述：
#   校验角色为 `base` 或 `release`。
#
# 参数：
#   $1: role - 镜像角色。
#
_image_validate_role() {
  case "$1" in
  base | release) ;;
  *)
    _image_error "role must be base or release: $1"
    return 1
    ;;
  esac
}

# _image_validate_reuse_role
#
# 功能描述：
#   校验 `image-reuse` 角色为 `base`、`release` 或 `all`。
#
# 参数：
#   $1: role - 复用角色。
#
_image_validate_reuse_role() {
  case "$1" in
  base | release | all) ;;
  *)
    _image_error "role must be base, release, or all: $1"
    return 1
    ;;
  esac
}

# _image_exists
#
# 功能描述：
#   用 `docker buildx imagetools inspect` 判断镜像引用是否已在仓库中。
#
# 参数：
#   $1: image - 完整镜像引用。
#
_image_exists() {
  docker buildx imagetools inspect "$1" >/dev/null 2>&1
}

# _image_require_existing
#
# 功能描述：
#   要求镜像已存在于仓库, 否则报错返回 1。
#
# 参数：
#   $1: image - 完整镜像引用。
#
_image_require_existing() {
  local image=$1

  if ! _image_exists "$image"; then
    _image_error "required image is missing from registry: $image"
    return 1
  fi
}

# _image_require_new
#
# 功能描述：
#   要求不可变 tag 尚不存在, 已存在则拒绝覆盖。
#
# 参数：
#   $1: image - 完整镜像引用。
#
_image_require_new() {
  local image=$1

  if _image_exists "$image"; then
    _image_error "immutable tag already exists: $image"
    return 1
  fi
}

# _image_set_native_defaults
#
# 功能描述：
#   按本机架构设置 `IMAGE_ARCH`、CUDA 基础镜像与 build 网络默认值。
#
# 参数：
#   (无)
#
_image_set_native_defaults() {
  local host_arch
  host_arch=$(uname -m)

  case "$host_arch" in
  x86_64 | amd64)
    IMAGE_ARCH=x86_64
    IMAGE_CUDA_BASE=${IMAGE_CUDA_BASE:-$IMAGE_BASE_IMAGE_AMD64}
    IMAGE_BUILD_NETWORK=${IMAGE_BUILD_NETWORK:-default}
    ;;
  aarch64 | arm64)
    IMAGE_ARCH=aarch64
    IMAGE_CUDA_BASE=${IMAGE_CUDA_BASE:-$IMAGE_BASE_IMAGE_ARM64}
    IMAGE_BUILD_NETWORK=${IMAGE_BUILD_NETWORK:-host}
    ;;
  *)
    _image_error "unsupported architecture: $host_arch"
    return 1
    ;;
  esac
}

# _image_run_build
#
# 功能描述：
#   执行 `docker buildx build`, 按 `$1` 追加 `--push` 或 `--load`。
#
# 参数：
#   $1: push - `1` 推送, 否则本地 load。
#   $@: cmd - `docker buildx build` 及其参数 (不含 context)。
#
_image_run_build() {
  local push=$1
  shift
  local -a cmd=("$@")

  if [[ "$push" -eq 1 ]]; then
    cmd+=(--push)
  else
    cmd+=(--load)
  fi
  cmd+=("$IMAGE_DOCKER_CONTEXT")

  "${cmd[@]}"
}

# _image_create_base
#
# 功能描述：
#   构建本机架构的 ROS 2 base 子镜像。
#
# 参数：
#   $1: version - 版本 tag。
#   $2: push - `1` 推送, 否则本地 load。
#
_image_create_base() {
  local version=$1
  local push=$2
  local tag="${IMAGE_BASE_REPOSITORY}:${version}-${IMAGE_ARCH}"

  if [[ ! -f "$IMAGE_BASE_DOCKERFILE" ]]; then
    _image_error "base Dockerfile is missing: $IMAGE_BASE_DOCKERFILE"
    return 1
  fi
  _image_require_new "$tag" || return 1

  echo "image-create: building base \"$tag\" (cuda=$IMAGE_CUDA_BASE, network=$IMAGE_BUILD_NETWORK)" >&2
  _image_run_build "$push" \
    docker buildx build \
    --network="$IMAGE_BUILD_NETWORK" \
    --build-arg "AI2_CUDA_BASE=$IMAGE_CUDA_BASE" \
    --build-arg "BASE_IMAGE_ARM64=$IMAGE_BASE_IMAGE_ARM64" \
    --build-arg "BASE_IMAGE_AMD64=$IMAGE_BASE_IMAGE_AMD64" \
    --build-arg "TENSORRT_VERSION=$IMAGE_TENSORRT_VERSION" \
    -f "$IMAGE_BASE_DOCKERFILE" \
    -t "$tag"
}

# _image_create_release
#
# 功能描述：
#   以已发布的 base 为 `--build-arg BASE_IMAGE` 构建本机架构的 release 子镜像。
#
# 参数：
#   $1: version - 版本 tag。
#   $2: push - `1` 推送, 否则本地 load。
#
_image_create_release() {
  local version=$1
  local push=$2
  local base_image="${IMAGE_BASE_REPOSITORY}:${version}"
  local tag="${IMAGE_RELEASE_REPOSITORY}:${version}-${IMAGE_ARCH}"

  if [[ ! -f "$IMAGE_RELEASE_DOCKERFILE" ]]; then
    _image_error "release Dockerfile is missing: $IMAGE_RELEASE_DOCKERFILE"
    return 1
  fi
  _image_require_existing "$base_image" || return 1
  _image_require_new "$tag" || return 1

  echo "image-create: building release \"$tag\" from \"$base_image\" (network=$IMAGE_BUILD_NETWORK)" >&2
  _image_run_build "$push" \
    docker buildx build \
    --pull \
    --network="$IMAGE_BUILD_NETWORK" \
    --build-arg "BASE_IMAGE=$base_image" \
    -f "$IMAGE_RELEASE_DOCKERFILE" \
    -t "$tag"
}

# _image_apply_create_overrides
#
# 功能描述：
#   将 `image-create` 命令行覆盖项写入 `IMAGE_*` 变量。
#
# 参数：
#   $1: registry - 仓库主机; 空则跳过。
#   $2: base_repository - 覆盖 `IMAGE_BASE_REPOSITORY`。
#   $3: release_repository - 覆盖 `IMAGE_RELEASE_REPOSITORY`。
#   $4: cuda_base - 覆盖本机 CUDA 基础镜像。
#   $5: base_image_arm64 - 覆盖 Jetson CUDA 基础镜像。
#   $6: base_image_amd64 - 覆盖 x86 CUDA 基础镜像。
#   $7: tensorrt_version - 覆盖 TensorRT 包版本。
#   $8: build_network - 覆盖 Docker 构建网络。
#
_image_apply_create_overrides() {
  local registry=$1
  local base_repository=$2
  local release_repository=$3
  local cuda_base=$4
  local base_image_arm64=$5
  local base_image_amd64=$6
  local tensorrt_version=$7
  local build_network=$8

  if [[ -n "$registry" ]]; then
    _image_apply_registry "$registry" || return 1
  fi
  [[ -n "$base_repository" ]] && IMAGE_BASE_REPOSITORY=$base_repository
  [[ -n "$release_repository" ]] && IMAGE_RELEASE_REPOSITORY=$release_repository
  [[ -n "$cuda_base" ]] && IMAGE_CUDA_BASE=$cuda_base
  [[ -n "$base_image_arm64" ]] && IMAGE_BASE_IMAGE_ARM64=$base_image_arm64
  [[ -n "$base_image_amd64" ]] && IMAGE_BASE_IMAGE_AMD64=$base_image_amd64
  [[ -n "$tensorrt_version" ]] && IMAGE_TENSORRT_VERSION=$tensorrt_version
  [[ -n "$build_network" ]] && IMAGE_BUILD_NETWORK=$build_network
  return 0
}

# image-create
#
# 功能描述：
#   在本机架构上构建 ROS 2 base 或 release 子镜像 (`docker buildx build`)。
#
# 参数：
#   --config FILE - 可选, source 私有镜像配置。
#   --push - 推送子镜像; 默认本地 `--load`。
#   --registry HOST[:PORT] - 成对切换 base/release 仓库前缀。
#   --base-repository REPO - 覆盖 `IMAGE_BASE_REPOSITORY`。
#   --release-repository REPO - 覆盖 `IMAGE_RELEASE_REPOSITORY`。
#   --cuda-base IMAGE - 覆盖本机 CUDA 基础镜像。
#   --base-image-arm64 IMAGE - 覆盖 Jetson CUDA 基础镜像。
#   --base-image-amd64 IMAGE - 覆盖 x86 CUDA 基础镜像。
#   --tensorrt-version VER - 覆盖 TensorRT 包版本。
#   --network MODE - 覆盖 Docker 构建网络。
#   $1: role - `base` 或 `release`。
#   $2: version - `v<major>.<minor>.<patch|x>`。
#
# 使用示例：
#   image-create base v0.9.3
#   image-create --push release v0.9.x
#
# 注意事项：
#   1. 需要 docker 与 docker buildx。
#   2. 不可变 tag 已存在时拒绝覆盖。
#
image-create() {
  case "${1-}" in
  --help | -h)
    cat >&2 <<'EOF'
image-create — build a native ROS 2 base or release child image.

Usage: image-create [OPTIONS] <base|release> VERSION

VERSION is v<major>.<minor>.<patch> or v<major>.<minor>.x.

Options:
  --config FILE             optionally source private configuration
  --push                    push the child image; default: --load locally
  --registry HOST[:PORT]    switch base/release repositories as a pair
  --base-repository REPO    override IMAGE_BASE_REPOSITORY
  --release-repository REPO override IMAGE_RELEASE_REPOSITORY
  --cuda-base IMAGE         override the native CUDA base image
  --base-image-arm64 IMAGE  override the Jetson CUDA base image
  --base-image-amd64 IMAGE  override the x86 CUDA base image
  --tensorrt-version VER    override the TensorRT package version
  --network MODE            override Docker build network

Examples:
  image-create base v0.9.3 --push
  image-create --registry docker.postrantor.cn:5000 base v0.9.x --push
  image-create --network host base v0.9.3 --push
  image-create --config /tmp/image.conf release v0.9.3 --push
EOF
    return 0
    ;;
  esac

  (
    local config=${AI2_IMAGE_CONFIG:-}
    local push=0 role="" version=""
    local registry="" base_repository="" release_repository="" cuda_base=""
    local base_image_arm64="" base_image_amd64="" tensorrt_version="" build_network=""

    while [[ $# -gt 0 ]]; do
      case "$1" in
      --config)
        [[ -n "${2-}" ]] || {
          _image_error "--config requires a file"
          return 1
        }
        config=$2
        shift 2
        ;;
      --push)
        push=1
        shift
        ;;
      --registry | --base-repository | --release-repository | --cuda-base | --base-image-arm64 | --base-image-amd64 | --tensorrt-version | --network)
        [[ -n "${2-}" ]] || {
          _image_error "$1 requires a value"
          return 1
        }
        case "$1" in
        --registry) registry=$2 ;;
        --base-repository) base_repository=$2 ;;
        --release-repository) release_repository=$2 ;;
        --cuda-base) cuda_base=$2 ;;
        --base-image-arm64) base_image_arm64=$2 ;;
        --base-image-amd64) base_image_amd64=$2 ;;
        --tensorrt-version) tensorrt_version=$2 ;;
        --network) build_network=$2 ;;
        esac
        shift 2
        ;;
      -*)
        _image_error "unknown option: $1"
        return 1
        ;;
      *)
        if [[ -z "$role" ]]; then
          role=$1
        elif [[ -z "$version" ]]; then
          version=$1
        else
          _image_error "unexpected argument: $1"
          return 1
        fi
        shift
        ;;
      esac
    done

    [[ -n "$role" && -n "$version" ]] || {
      _image_error "usage: image-create [OPTIONS] <base|release> VERSION"
      return 1
    }
    _image_load_config "$config" || return 1
    _image_apply_create_overrides \
      "$registry" "$base_repository" "$release_repository" "$cuda_base" \
      "$base_image_arm64" "$base_image_amd64" "$tensorrt_version" "$build_network" || return 1
    _image_validate_version "$version" || return 1
    _image_validate_role "$role" || return 1
    _image_set_native_defaults || return 1
    _image_require_buildx || return 1

    case "$role" in
    base) _image_create_base "$version" "$push" ;;
    release) _image_create_release "$version" "$push" ;;
    *)
      _image_error "role must be base or release: $role"
      return 1
      ;;
    esac
  )
}

# _image_apply_repository_overrides
#
# 功能描述：
#   将 registry / 仓库覆盖项写入 `IMAGE_*` (供 manifest / reuse 使用)。
#
# 参数：
#   $1: registry - 仓库主机; 空则跳过。
#   $2: base_repository - 覆盖 `IMAGE_BASE_REPOSITORY`。
#   $3: release_repository - 覆盖 `IMAGE_RELEASE_REPOSITORY`。
#
_image_apply_repository_overrides() {
  local registry=$1
  local base_repository=$2
  local release_repository=$3

  if [[ -n "$registry" ]]; then
    _image_apply_registry "$registry" || return 1
  fi
  [[ -n "$base_repository" ]] && IMAGE_BASE_REPOSITORY=$base_repository
  [[ -n "$release_repository" ]] && IMAGE_RELEASE_REPOSITORY=$release_repository
  return 0
}

# image-manifest
#
# 功能描述：
#   将已存在的 `VERSION-x86_64` 与 `VERSION-aarch64` 子镜像合并为不可变多架构 tag。
#
# 参数：
#   --config FILE - 可选, source 私有镜像配置。
#   --registry HOST[:PORT] - 成对切换 base/release 仓库前缀。
#   --base-repository REPO - 覆盖 `IMAGE_BASE_REPOSITORY`。
#   --release-repository REPO - 覆盖 `IMAGE_RELEASE_REPOSITORY`。
#   $1: role - `base` 或 `release`。
#   $2: version - `v<major>.<minor>.<patch|x>`。
#
# 使用示例：
#   image-manifest base v0.9.3
#   image-manifest --registry docker.example.com:3443 release v0.9.x
#
# 注意事项：
#   1. 对应的 `VERSION-x86_64` 与 `VERSION-aarch64` tag 必须已存在。
#   2. 目标多架构 tag 已存在时拒绝覆盖。
#
image-manifest() {
  case "${1-}" in
  --help | -h)
    cat >&2 <<'EOF'
image-manifest — publish a multi-platform base or release tag.

Usage: image-manifest [OPTIONS] <base|release> VERSION

Options:
  --config FILE             optionally source private configuration
  --registry HOST[:PORT]    switch base/release repositories as a pair
  --base-repository REPO    override IMAGE_BASE_REPOSITORY
  --release-repository REPO override IMAGE_RELEASE_REPOSITORY

The corresponding VERSION-x86_64 and VERSION-aarch64 tags must already exist.
EOF
    return 0
    ;;
  esac

  (
    local config=${AI2_IMAGE_CONFIG:-}
    local registry="" base_repository="" release_repository="" role="" version=""

    while [[ $# -gt 0 ]]; do
      case "$1" in
      --config | --registry | --base-repository | --release-repository)
        [[ -n "${2-}" ]] || {
          _image_error "$1 requires a value"
          return 1
        }
        case "$1" in
        --config) config=$2 ;;
        --registry) registry=$2 ;;
        --base-repository) base_repository=$2 ;;
        --release-repository) release_repository=$2 ;;
        esac
        shift 2
        ;;
      -*)
        _image_error "unknown option: $1"
        return 1
        ;;
      *)
        if [[ -z "$role" ]]; then
          role=$1
        elif [[ -z "$version" ]]; then
          version=$1
        else
          _image_error "unexpected argument: $1"
          return 1
        fi
        shift
        ;;
      esac
    done

    [[ -n "$role" && -n "$version" ]] || {
      _image_error "usage: image-manifest [OPTIONS] <base|release> VERSION"
      return 1
    }
    _image_load_config "$config" || return 1
    _image_apply_repository_overrides "$registry" "$base_repository" "$release_repository" || return 1
    _image_validate_version "$version" || return 1
    _image_validate_role "$role" || return 1
    _image_require_buildx || return 1

    local repository
    case "$role" in
    base) repository=$IMAGE_BASE_REPOSITORY ;;
    release) repository=$IMAGE_RELEASE_REPOSITORY ;;
    *)
      _image_error "role must be base or release: $role"
      return 1
      ;;
    esac

    local target="${repository}:${version}"
    local image_x86="${repository}:${version}-x86_64"
    local image_arm="${repository}:${version}-aarch64"
    _image_require_new "$target" || return 1
    _image_require_existing "$image_x86" || return 1
    _image_require_existing "$image_arm" || return 1

    echo "image-manifest: publishing $role manifest \"$target\"" >&2
    docker buildx imagetools create \
      --tag "$target" \
      "$image_x86" \
      "$image_arm"
  )
}

# _image_copy_version
#
# 功能描述：
#   把已有多架构 tag 复制为新版本 tag (`docker buildx imagetools create`)。
#
# 参数：
#   $1: repository - 仓库路径。
#   $2: from_version - 源版本。
#   $3: to_version - 目标版本。
#
_image_copy_version() {
  local repository=$1
  local from_version=$2
  local to_version=$3
  local source="${repository}:${from_version}"
  local target="${repository}:${to_version}"

  _image_require_existing "$source" || return 1
  _image_require_new "$target" || return 1

  echo "image-reuse: reusing \"$source\" as \"$target\"" >&2
  docker buildx imagetools create --tag "$target" "$source"
}

# image-reuse
#
# 功能描述：
#   将已有多架构 base / release (或两者) 清单复用到新版本 tag。
#
# 参数：
#   --config FILE - 可选, source 私有镜像配置。
#   --registry HOST[:PORT] - 成对切换 base/release 仓库前缀。
#   --base-repository REPO - 覆盖 `IMAGE_BASE_REPOSITORY`。
#   --release-repository REPO - 覆盖 `IMAGE_RELEASE_REPOSITORY`。
#   $1: from_version - 源版本。
#   $2: to_version - 目标版本。
#   $3: role - `base`、`release` 或 `all` (默认 `all`)。
#
# 使用示例：
#   image-reuse v0.9.2 v0.9.3
#   image-reuse v0.9.3 v0.9.x release
#
# 注意事项：
#   1. 已存在的目标 tag 不会被覆盖。
#
image-reuse() {
  case "${1-}" in
  --help | -h)
    cat >&2 <<'EOF'
image-reuse — reuse an existing multi-platform manifest under a new version.

Usage: image-reuse [OPTIONS] FROM_VERSION TO_VERSION [base|release|all]

Options:
  --config FILE             optionally source private configuration
  --registry HOST[:PORT]    switch base/release repositories as a pair
  --base-repository REPO    override IMAGE_BASE_REPOSITORY
  --release-repository REPO override IMAGE_RELEASE_REPOSITORY

The default role is all. Existing release tags are never overwritten.
EOF
    return 0
    ;;
  esac

  (
    local config=${AI2_IMAGE_CONFIG:-}
    local registry="" base_repository="" release_repository="" from_version="" to_version="" role=all
    local positional_count=0

    while [[ $# -gt 0 ]]; do
      case "$1" in
      --config | --registry | --base-repository | --release-repository)
        [[ -n "${2-}" ]] || {
          _image_error "$1 requires a value"
          return 1
        }
        case "$1" in
        --config) config=$2 ;;
        --registry) registry=$2 ;;
        --base-repository) base_repository=$2 ;;
        --release-repository) release_repository=$2 ;;
        esac
        shift 2
        ;;
      -*)
        _image_error "unknown option: $1"
        return 1
        ;;
      *)
        if [[ "$positional_count" -eq 0 ]]; then
          from_version=$1
        elif [[ "$positional_count" -eq 1 ]]; then
          to_version=$1
        elif [[ "$positional_count" -eq 2 ]]; then
          role=$1
        else
          _image_error "unexpected argument: $1"
          return 1
        fi
        positional_count=$((positional_count + 1))
        shift
        ;;
      esac
    done

    [[ -n "$from_version" && -n "$to_version" ]] || {
      _image_error "usage: image-reuse [OPTIONS] FROM_VERSION TO_VERSION [base|release|all]"
      return 1
    }
    _image_load_config "$config" || return 1
    _image_apply_repository_overrides "$registry" "$base_repository" "$release_repository" || return 1
    _image_validate_version "$from_version" || return 1
    _image_validate_version "$to_version" || return 1
    _image_validate_reuse_role "$role" || return 1
    _image_require_buildx || return 1

    case "$role" in
    base)
      _image_copy_version "$IMAGE_BASE_REPOSITORY" "$from_version" "$to_version"
      ;;
    release)
      _image_copy_version "$IMAGE_RELEASE_REPOSITORY" "$from_version" "$to_version"
      ;;
    all)
      _image_require_existing "${IMAGE_BASE_REPOSITORY}:${from_version}" || return 1
      _image_require_existing "${IMAGE_RELEASE_REPOSITORY}:${from_version}" || return 1
      _image_require_new "${IMAGE_BASE_REPOSITORY}:${to_version}" || return 1
      _image_require_new "${IMAGE_RELEASE_REPOSITORY}:${to_version}" || return 1
      _image_copy_version "$IMAGE_BASE_REPOSITORY" "$from_version" "$to_version" || return 1
      _image_copy_version "$IMAGE_RELEASE_REPOSITORY" "$from_version" "$to_version"
      ;;
    *)
      _image_error "role must be base, release, or all: $role"
      return 1
      ;;
    esac
  )
}

## set docker run command

# 可以配置ssh登录的环境变量来指定是否直接进入容器(默认不进入)
# 建议拷贝到个人账户下的bashrc配置单独的container_name和用户名
#if [ "${ENTRY_CONTAINER:-}" = "true" ]; then
#  echo -e "\033[35mEntering container...\033[0m"
#  container-entry auriga_cpp auriga
#fi

# setup-docker-daemon
#
# 功能描述：
#   写入 `/etc/docker/daemon.json` (data-root / mirrors / insecure-registries; 可选 nvidia runtime)。
#
# 参数：
#   (无)
#
# 使用示例：
#   setup-docker-daemon
#   SETUP_DOCKER_FORCE=1 setup-docker-daemon
#
# 注意事项：
#   1. 已有 `daemon.json` 时默认跳过; `SETUP_DOCKER_FORCE=1` 会备份后覆盖。
#   2. nvidia runtime 仅在 `nvidia-container-runtime` 存在时写入。
#   3. 依赖 `common/setup/common.bash` 的 `__setup_*` 辅助 (由 `setup.bash` 先 source)。
#   4. 生成 JSON 需要 `python3`。
#
setup-docker-daemon() {
  if ! declare -F __setup_warn >/dev/null 2>&1; then
    echo "setup-docker-daemon: source setup.bash first" >&2
    return 1
  fi

  local dest="/etc/docker/daemon.json"
  local tmp
  local data_root="${SETUP_DOCKER_DATA_ROOT}"
  local insecure="${SETUP_DOCKER_INSECURE}"

  __setup_warn "Will write ${dest} (docker data-root / mirrors / insecure-registries)."
  __setup_confirm "configure docker daemon?" || return 0

  if [[ -f "${dest}" && "${SETUP_DOCKER_FORCE:-}" != "1" ]]; then
    __setup_log "${dest} exists; skip (SETUP_DOCKER_FORCE=1 to replace after backup)."
    return 0
  fi

  if [[ -f "${dest}" ]]; then
    __setup_sudo cp -a "${dest}" "${dest}.bak.$(date +%Y%m%d%H%M%S)"
  fi

  tmp="$(mktemp)"
  if command -v python3 >/dev/null 2>&1; then
    SETUP_DOCKER_DATA_ROOT="${data_root}" SETUP_DOCKER_INSECURE="${insecure}" \
      SETUP_HOME_STORE_ROOT="${SETUP_HOME_STORE_ROOT}" python3 - <<'PY' >"${tmp}"
import json
import os
import shutil

cfg = {
    "registry-mirrors": [
        "https://docker.1ms.run",
        "https://docker.xuanyuan.me",
    ],
    "insecure-registries": [
        item.strip()
        for item in os.environ.get("SETUP_DOCKER_INSECURE", "").split(",")
        if item.strip()
    ],
}
store_root = os.environ.get("SETUP_HOME_STORE_ROOT", "/home/data")
data_root = os.environ.get("SETUP_DOCKER_DATA_ROOT", os.path.join(store_root, "docker"))
if os.path.isdir(store_root):
    cfg["data-root"] = data_root
if shutil.which("nvidia-container-runtime"):
    cfg["runtimes"] = {
        "nvidia": {"args": [], "path": "nvidia-container-runtime"}
    }
print(json.dumps(cfg, indent=2))
print()
PY
  else
    __setup_error "python3 is required to generate ${dest}"
    rm -f "${tmp}"
    return 1
  fi

  __setup_sudo mkdir -p /etc/docker
  __setup_sudo install -m 644 -o root -g root "${tmp}" "${dest}"
  rm -f "${tmp}"
  __setup_log "wrote ${dest}"

  if command -v systemctl >/dev/null 2>&1; then
    __setup_confirm "restart docker now?" || return 0
    __setup_sudo systemctl restart docker || return 1
  fi
}
