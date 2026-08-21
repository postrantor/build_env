# ---
# date: 2026-08-21
# description: 宿主机部署公共变量与辅助函数。由 setup.bash source, 不单独执行。
# ---

: "${SETUP_ROOT:=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
: "${SETUP_DIR:=${SETUP_ROOT}/common/setup}"
: "${SETUP_TEMPLATE_DIR:=${SETUP_DIR}/template}"
: "${SETUP_MACHINES_DIR:=${SETUP_DIR}/machines}"

: "${SETUP_USERNAME:=${USER}}"
: "${SETUP_MARKER_BEGIN:=## begin build_env/setup}"
: "${SETUP_MARKER_END:=## end build_env/setup}"

: "${SETUP_HOME_STORE_ROOT:=/home/data}"
: "${SETUP_WORKSPACES:=${HOME}/workspaces}"
: "${SETUP_WORKSPACES_LINK:=/workspaces}"

: "${SETUP_HOSTS_IP:=192.168.66.100}"
: "${SETUP_REPOS_HOST:=repo.postrantor.cn}"
: "${SETUP_NAS_HOST:=aliyun.postrantor.cn}"
: "${SETUP_NAS_USER:=trantor}"
: "${SETUP_NAS_PORT:=1230}"
: "${SETUP_SSH_USER:=${SETUP_USERNAME}}"

: "${SETUP_PROXY_URL:=http://clash.postrantor.com:7891}"
: "${SETUP_DOCKER_DATA_ROOT:=${SETUP_HOME_STORE_ROOT}/docker}"
: "${SETUP_DOCKER_INSECURE:=docker.postrantor.cn:5000,docker.postrantor.com:5000}"

: "${SETUP_GIT_INSTEADOF_FROM:=ssh://git@git.ai2rob.com:8022}"
: "${SETUP_GIT_INSTEADOF_TO:=ssh://repos/~/.repositories/ai2rob}"

export SETUP_ROOT SETUP_DIR SETUP_TEMPLATE_DIR SETUP_MACHINES_DIR
export SETUP_USERNAME SETUP_MARKER_BEGIN SETUP_MARKER_END

# __setup_log
#
# 功能描述：
#   向 stdout 打印一行日志。
#
# 参数：
#   $@: message - 日志内容。
#
__setup_log() {
  printf '%s\n' "$*"
}

# __setup_warn
#
# 功能描述：
#   向 stderr 打印黄色警告。
#
# 参数：
#   $@: message - 警告内容。
#
__setup_warn() {
  echo -e "\033[33m$*\033[0m" >&2
}

# __setup_error
#
# 功能描述：
#   向 stderr 打印 `error:` 前缀的错误信息。
#
# 参数：
#   $@: message - 错误内容。
#
__setup_error() {
  echo "error: $*" >&2
}

# __setup_confirm
#
# 功能描述：
#   交互确认是否继续。`SETUP_NONINTERACTIVE=1` / `SETUP_YES=1` 或非 TTY 时直接返回 0。
#
# 参数：
#   $1: prompt - 提示语 (默认: proceed?)。
#
# 使用示例：
#   __setup_confirm "write /etc/hosts?" || return 0
#
__setup_confirm() {
  local prompt="${1:-proceed?}"
  if [[ "${SETUP_NONINTERACTIVE:-}" == "1" || "${SETUP_YES:-}" == "1" || ! -t 0 ]]; then
    return 0
  fi
  local ans
  read -r -e -p "${prompt} [Y/n] " ans
  case "${ans,,}" in
  n | no)
    echo "skipped."
    return 1
    ;;
  *)
    return 0
    ;;
  esac
}

# __setup_sudo
#
# 功能描述：
#   当前已是 root 则直接执行命令, 否则通过 sudo 执行。
#
# 参数：
#   $@: command - 要执行的命令及其参数。
#
__setup_sudo() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

# __setup_target_home
#
# 功能描述：
#   解析目标用户的家目录; passwd 无记录时回退到 `/home/<user>`。
#
# 参数：
#   $1: user - 用户名 (默认: SETUP_USERNAME)。
#
__setup_target_home() {
  local user="${1:-${SETUP_USERNAME}}"
  local home
  home="$(getent passwd "${user}" 2>/dev/null | cut -d: -f6 || true)"
  if [[ -n "${home}" ]]; then
    printf '%s' "${home}"
    return 0
  fi
  printf '%s' "/home/${user}"
}

# __setup_render
#
# 功能描述：
#   用当前 SETUP_* 变量替换模板中的 `@SETUP_*@` 占位符, 结果写到 stdout。
#
# 参数：
#   $1: src - 模板文件路径。
#
# 注意事项：
#   1. 模板文件必须存在, 否则返回 1。
#
__setup_render() {
  local src="$1"
  local user="${SETUP_USERNAME}"
  local home
  home="$(__setup_target_home "${user}")"
  if [[ ! -f "${src}" ]]; then
    __setup_error "template not found: ${src}"
    return 1
  fi
  sed \
    -e "s|@SETUP_USERNAME@|${user}|g" \
    -e "s|@SETUP_HOME@|${home}|g" \
    -e "s|@SETUP_HOSTS_IP@|${SETUP_HOSTS_IP}|g" \
    -e "s|@SETUP_REPOS_HOST@|${SETUP_REPOS_HOST}|g" \
    -e "s|@SETUP_NAS_HOST@|${SETUP_NAS_HOST}|g" \
    -e "s|@SETUP_NAS_USER@|${SETUP_NAS_USER}|g" \
    -e "s|@SETUP_NAS_PORT@|${SETUP_NAS_PORT}|g" \
    -e "s|@SETUP_SSH_USER@|${SETUP_SSH_USER}|g" \
    -e "s|@SETUP_PROXY_URL@|${SETUP_PROXY_URL}|g" \
    -e "s|@SETUP_HOSTNAME@|${SETUP_HOSTNAME:-$(hostname)}|g" \
    -e "s|@SETUP_PROJECT_WORKDIR@|${SETUP_PROJECT_WORKDIR:-${home}/${SETUP_PROJECT_NAME:-}}|g" \
    -e "s|@SETUP_CONTAINER_NAME@|${SETUP_CONTAINER_NAME:-${user}_cpp}|g" \
    "${src}"
}

# __setup_upsert_marked
#
# 功能描述：
#   在 dest 中用标记块包裹 body; 已有同名标记则替换中间内容。写系统文件时走 sudo。
#
# 参数：
#   $1: dest - 目标文件 (如 /etc/hosts、/etc/fstab)。
#   $2: body - 写入标记块内的文本。
#
# 注意事项：
#   1. 标记由 SETUP_MARKER_BEGIN / SETUP_MARKER_END 定义, 重复执行不会无限追加。
#
__setup_upsert_marked() {
  local dest="$1"
  local body="$2"
  local begin="${SETUP_MARKER_BEGIN}"
  local end="${SETUP_MARKER_END}"
  local tmp existing
  tmp="$(mktemp)"
  if [[ -f "${dest}" ]]; then
    existing="$(__setup_sudo cat "${dest}"; printf x)" || {
      rm -f "${tmp}"
      return 1
    }
    existing="${existing%x}"
  else
    existing=""
  fi

  if printf '%s\n' "${existing}" | grep -qFx "${begin}"; then
    printf '%s\n' "${existing}" | SETUP_MARKED_BEGIN="${begin}" SETUP_MARKED_END="${end}" SETUP_MARKED_BODY="${body}" awk '
      $0 == ENVIRON["SETUP_MARKED_BEGIN"] { print; print ENVIRON["SETUP_MARKED_BODY"]; skip=1; next }
      $0 == ENVIRON["SETUP_MARKED_END"] { skip=0; print; next }
      skip { next }
      { print }
    ' >"${tmp}"
  else
    if [[ -n "${existing}" ]]; then
      printf '%s\n' "${existing}" >"${tmp}"
      [[ "${existing}" == *$'\n' ]] || printf '\n' >>"${tmp}"
    else
      : >"${tmp}"
    fi
    printf '\n%s\n%s\n%s\n' "${begin}" "${body}" "${end}" >>"${tmp}"
  fi

  __setup_sudo cp "${tmp}" "${dest}"
  rm -f "${tmp}"
}

# __setup_install_user_file
#
# 功能描述：
#   把 src 安装到 dest, 并设置属主与权限。目标是当前用户时不走 sudo。
#
# 参数：
#   $1: src - 源文件。
#   $2: dest - 目标路径。
#   $3: user - 属主 (默认: SETUP_USERNAME)。
#   $4: mode - 权限 (默认: 644)。
#
__setup_install_user_file() {
  local src="$1"
  local dest="$2"
  local user="${3:-${SETUP_USERNAME}}"
  local mode="${4:-644}"
  local dir
  dir="$(dirname "${dest}")"
  if [[ "${user}" == "${USER}" && "$(id -u)" -ne 0 ]]; then
    mkdir -p "${dir}"
    cp -- "${src}" "${dest}"
    chmod "${mode}" "${dest}"
  else
    __setup_sudo mkdir -p "${dir}"
    __setup_sudo cp -- "${src}" "${dest}"
    __setup_sudo chmod "${mode}" "${dest}"
    __setup_sudo chown "${user}:${user}" "${dest}"
  fi
}

# __setup_group_exists
#
# 功能描述：
#   判断系统是否存在指定组。
#
# 参数：
#   $1: group - 组名。
#
__setup_group_exists() {
  getent group "$1" >/dev/null 2>&1
}
