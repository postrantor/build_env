# ---
# date: 2026-08-21
# description: 部署 build_env 自身: 按 URL / 版本 clone, 可选写入 BUILD_ENV_ROOT 与 PATH。亦可 curl / ssh | bash。
# ---

_deploy_self_dir=""
if [[ -n "${BASH_SOURCE[0]:-}" ]]; then
  _deploy_self_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" 2>/dev/null && pwd)" || _deploy_self_dir=""
fi
if [[ -n "${_deploy_self_dir}" && -f "${_deploy_self_dir}/common.bash" ]]; then
  export SETUP_DIR="${SETUP_DIR:-${_deploy_self_dir}}"
  export SETUP_ROOT="${SETUP_ROOT:-$(cd "${_deploy_self_dir}/../.." && pwd)}"
  # shellcheck disable=SC1091
  source "${_deploy_self_dir}/common.bash"
else
  __setup_log() { printf '%s\n' "$*"; }
  __setup_warn() { echo -e "\033[33m$*\033[0m" >&2; }
fi
unset _deploy_self_dir

: "${BUILD_ENV_DEFAULT_PATH:=/tmp/build_env}"
: "${BUILD_ENV_PATH_BEGIN:=## begin build_env/path}"
: "${BUILD_ENV_PATH_END:=## end build_env/path}"

# _deploy_self_usage
#
# 功能描述：
#   打印 build_env 自身部署用法到 stdout。
#
# 参数：
#   (无)
#
_deploy_self_usage() {
  cat <<EOF
部署 build_env 元仓库 (clone 到指定路径, 或作为脚本库)。

管道执行 (curl / ssh | bash) 时未给 --url / --version 则先读环境变量,
再不行才在终端询问。给脚本传参须 bash -s --; 不要写 bash --help (那是 bash 自身)。

Usage:
  export build_env_url='ssh://host/path/build_env.git'
  export build_env_version='hexa/dev'
  ssh -T aliyun 'cat ~/deploy.bash' | bash
  ssh -T aliyun 'cat ~/deploy.bash' | bash -s -- --help
  curl https://raw.githubusercontent.com/postrantor/build_env/t/deploy/common/setup/deploy.bash -fsS | bash
  $(basename "$0") --url <git-url> --version <ref> [--path <dir>] [--username <user>]
  $(basename "$0") --help

Options:
  --url URL          源仓库地址 (https / ssh / 本地路径)
  --version REF      分支名或 tag (亦接受 --ref / --branch)
  --path DIR         clone 目标; 省略则为 ${BUILD_ENV_DEFAULT_PATH},
                     并附加 --depth=1 --single-branch (脚本库模式)
  --username USER    HTTPS 用户名; 密码只走交互, 不接受命令行
  -h, --help         显示本帮助

环境变量 (命令行优先; 大写与小写等价):
  BUILD_ENV_URL / build_env_url
  BUILD_ENV_VERSION / build_env_version

说明:
  1. --url / --version 省略时: 环境变量 → 终端询问。管道执行从 /dev/tty 读。
  2. 未给 --path 时视为脚本库: 先删除 ${BUILD_ENV_DEFAULT_PATH} 再浅克隆,
     写入 ~/.bashrc 标记块, 导出 BUILD_ENV_ROOT 并把该目录前置到 PATH,
     以便直接调用 setup.bash / deploy.bash。目标在 /tmp/ 下同样先删再建。
  3. 已给 --path 且不在 /tmp/ 时做常规 clone (不强制浅克隆), 仍写入 BUILD_ENV_ROOT 与 PATH。
  4. 非 /tmp/ 目标已是 Git 仓库则 fetch + checkout 指定版本, 不重复 clone。
  5. HTTPS 未给 --username 时由 git 自己提示; 给了则再询问密码 (不回显)。

Examples:
  build_env_url=git@host:group/build_env.git build_env_version=hexa/dev \\
    ssh -T aliyun 'cat ~/deploy.bash' | bash
  $(basename "$0") --url git@host:group/build_env.git --version hexa/v0.1.0
  $(basename "$0") --url https://host/group/build_env.git --version hexa/dev \\
    --path /workspaces/build_env --username alice
EOF
}

# _deploy_self_error
#
# 功能描述：
#   向 stderr 打印错误并返回 2。
#
# 参数：
#   $1: message - 错误内容。
#
_deploy_self_error() {
  printf 'error: %s\n' "$1" >&2
  return 2
}

# _deploy_self_url_with_user
#
# 功能描述：
#   若 URL 为 http(s) 且未带用户名, 则写入 user@。不把密码写入 URL。
#
# 参数：
#   $1: url - 原始地址。
#   $2: user - 可选用户名。
#
_deploy_self_url_with_user() {
  local url="$1"
  local user="${2:-}"
  if [[ -z "${user}" ]]; then
    printf '%s' "${url}"
    return 0
  fi
  case "${url}" in
  https://*@* | http://*@*)
    printf '%s' "${url}"
    ;;
  https://*)
    printf 'https://%s@%s' "${user}" "${url#https://}"
    ;;
  http://*)
    printf 'http://%s@%s' "${user}" "${url#http://}"
    ;;
  *)
    printf '%s' "${url}"
    ;;
  esac
}

# _deploy_self_from_env
#
# 功能描述：
#   读取 clone 用的 URL / 版本: 先大写 BUILD_ENV_*, 再小写 build_env_*。
#
# 参数：
#   $1: url | version
#
_deploy_self_from_env() {
  case "$1" in
  url)
    if [[ -n "${BUILD_ENV_URL:-}" ]]; then
      printf '%s' "${BUILD_ENV_URL}"
    elif [[ -n "${build_env_url:-}" ]]; then
      printf '%s' "${build_env_url}"
    fi
    ;;
  version)
    if [[ -n "${BUILD_ENV_VERSION:-}" ]]; then
      printf '%s' "${BUILD_ENV_VERSION}"
    elif [[ -n "${build_env_version:-}" ]]; then
      printf '%s' "${build_env_version}"
    fi
    ;;
  *)
    return 2
    ;;
  esac
}

# _deploy_self_read_tty
#
# 功能描述：
#   从终端读一行。curl / ssh | bash 时 stdin 是脚本, 改读 /dev/tty。
#
# 参数：
#   $1: prompt - 提示语。
#   $2: silent - 1 则不回显 (密码)。
#
_deploy_self_read_tty() {
  local prompt="$1"
  local silent="${2:-0}"
  local value=""
  local -a flags=(-r)
  [[ "${silent}" == "1" ]] && flags+=(-s)
  flags+=(-p "${prompt}")
  if [[ -t 0 ]]; then
    read "${flags[@]}" value || true
  elif [[ -r /dev/tty ]]; then
    read "${flags[@]}" value </dev/tty || true
  else
    _deploy_self_error "需要交互终端 (管道执行也要有 /dev/tty)"
    return 2
  fi
  if [[ "${silent}" == "1" ]]; then
    printf '\n' >&2
  fi
  printf '%s' "${value}"
}

# _deploy_self_prompt_password
#
# 功能描述：
#   已指定用户名时, 在终端上无回显读取密码。
#
# 参数：
#   $1: username - Git 用户名。
#
# 注意事项：
#   1. 密码只留在调用方局部变量, 不写日志、不进 argv。
#   2. stdin 不是 TTY 时从 /dev/tty 读。
#
_deploy_self_prompt_password() {
  local username="$1"
  local pass=""
  pass="$(_deploy_self_read_tty "Git password for ${username}: " 1)" || return 2
  if [[ -z "${pass}" ]]; then
    _deploy_self_error "密码为空"
    return 2
  fi
  printf '%s' "${pass}"
}

# _deploy_self_run_git
#
# 功能描述：
#   执行 git 子命令; 若提供了用户名与密码则用一次性 GIT_ASKPASS, 否则允许终端提示。
#
# 参数：
#   $@: git 参数 (不含 git 自身)。
#   环境: DEPLOY_GIT_USERNAME / DEPLOY_GIT_PASSWORD 可选。
#
_deploy_self_run_git() {
  local askpass="" rc=0
  if [[ -n "${DEPLOY_GIT_PASSWORD:-}" ]]; then
    askpass="$(mktemp)"
    chmod 700 "${askpass}"
    cat >"${askpass}" <<'EOF'
#!/bin/sh
case "$1" in
*[Uu]sername*) printf '%s\n' "${DEPLOY_GIT_USERNAME:-}" ;;
*) printf '%s\n' "${DEPLOY_GIT_PASSWORD:-}" ;;
esac
EOF
    GIT_ASKPASS="${askpass}" GIT_TERMINAL_PROMPT=0 git "$@"
    rc=$?
    rm -f "${askpass}"
    return "${rc}"
  fi
  GIT_TERMINAL_PROMPT=1 git "$@"
}

# _deploy_self_upsert_bashrc
#
# 功能描述：
#   在 ~/.bashrc 写入 BUILD_ENV_ROOT 与 PATH 标记块; 已有则替换。不走 sudo。
#
# 参数：
#   $1: dest - clone 后的仓库根路径。
#
_deploy_self_upsert_bashrc() {
  local dest="$1"
  local bashrc="${HOME}/.bashrc"
  local body tmp existing
  body="$(printf 'export BUILD_ENV_ROOT=%q\ncase ":${PATH}:" in\n*":${BUILD_ENV_ROOT}:"*) ;;\n*) export PATH="${BUILD_ENV_ROOT}:${PATH}" ;;\nesac' "${dest}")"

  tmp="$(mktemp)"
  if [[ -f "${bashrc}" ]]; then
    existing="$(cat "${bashrc}"; printf x)"
    existing="${existing%x}"
  else
    existing=""
  fi

  if printf '%s\n' "${existing}" | grep -qFx "${BUILD_ENV_PATH_BEGIN}"; then
    printf '%s\n' "${existing}" | BUILD_ENV_PATH_BEGIN="${BUILD_ENV_PATH_BEGIN}" BUILD_ENV_PATH_END="${BUILD_ENV_PATH_END}" BUILD_ENV_PATH_BODY="${body}" awk '
      $0 == ENVIRON["BUILD_ENV_PATH_BEGIN"] { print; print ENVIRON["BUILD_ENV_PATH_BODY"]; skip=1; next }
      $0 == ENVIRON["BUILD_ENV_PATH_END"] { skip=0; print; next }
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
    printf '\n%s\n%s\n%s\n' "${BUILD_ENV_PATH_BEGIN}" "${body}" "${BUILD_ENV_PATH_END}" >>"${tmp}"
  fi

  cp -- "${tmp}" "${bashrc}"
  rm -f "${tmp}"
  __setup_log "updated ${bashrc} (${BUILD_ENV_PATH_BEGIN})"
}

# _deploy_self_export_path
#
# 功能描述：
#   在当前 shell 导出 BUILD_ENV_ROOT, 并把仓库根前置到 PATH (已存在则跳过)。
#
# 参数：
#   $1: dest - 仓库根路径。
#
_deploy_self_export_path() {
  local dest="$1"
  export BUILD_ENV_ROOT="${dest}"
  case ":${PATH}:" in
  *":${dest}:"*) ;;
  *) export PATH="${dest}:${PATH}" ;;
  esac
}

# _deploy_self_checkout
#
# 功能描述：
#   目标已是 Git 仓库时 fetch 并检出指定版本。
#
# 参数：
#   $1: dest - 仓库根。
#   $2: url - 用于核对 / 更新 origin。
#   $3: version - 分支或 tag。
#   $4: shallow - 1 则 --depth=1。
#
_deploy_self_checkout() {
  local dest="$1"
  local url="$2"
  local version="$3"
  local shallow="$4"
  local -a fetch_args=(fetch --force)

  if [[ "${shallow}" == "1" ]]; then
    fetch_args+=(--depth 1)
  fi
  fetch_args+=(origin "${version}")

  __setup_log "fetch ${version} in ${dest}"
  if ! (
    cd "${dest}" || exit 1
    git remote set-url origin "${url}"
    _deploy_self_run_git "${fetch_args[@]}"
    _deploy_self_run_git checkout --force FETCH_HEAD
  ); then
    _deploy_self_error "无法在 ${dest} 检出 ${version}"
    return 2
  fi
}

# _deploy_self_wipe_tmp_dest
#
# 功能描述：
#   dest 位于 /tmp/ 下时先删除 (脚本库默认 /tmp/build_env), 再走全新 clone。
#   不删除 /tmp 自身。
#
# 参数：
#   $1: dest - 已解析的绝对路径。
#
_deploy_self_wipe_tmp_dest() {
  local dest="$1"
  case "${dest}" in
  /tmp | /tmp/)
    return 0
    ;;
  /tmp/*) ;;
  *)
    return 0
    ;;
  esac
  if [[ -e "${dest}" || -L "${dest}" ]]; then
    __setup_log "清除 ${dest}"
    rm -rf -- "${dest}"
  fi
}

# _deploy_self_clone
#
# 功能描述：
#   将 build_env clone (或更新) 到 dest。
#
# 参数：
#   $1: url - Git 地址 (可已含用户名)。
#   $2: version - 分支或 tag。
#   $3: dest - 目标目录。
#   $4: shallow - 1 则 --depth=1 --single-branch。
#
_deploy_self_clone() {
  local url="$1"
  local version="$2"
  local dest="$3"
  local shallow="$4"
  local -a clone_args=(clone --branch "${version}")

  if [[ "${shallow}" == "1" ]]; then
    clone_args+=(--depth 1 --single-branch)
  fi
  clone_args+=("${url}" "${dest}")

  if [[ -d "${dest}/.git" ]]; then
    _deploy_self_checkout "${dest}" "${url}" "${version}" "${shallow}"
    return $?
  fi
  if [[ -e "${dest}" ]] && [[ -n "$(ls -A "${dest}" 2>/dev/null || true)" ]]; then
    _deploy_self_error "目标已存在且不是 Git 仓库: ${dest}"
    return 2
  fi

  __setup_log "clone ${version} -> ${dest}"
  if ! _deploy_self_run_git "${clone_args[@]}"; then
    _deploy_self_error "git clone 失败"
    return 2
  fi
}

# deploy-build-env
#
# 功能描述：
#   按 --url / --version clone build_env, 写入 BUILD_ENV_ROOT 与 PATH。
#
# 参数：
#   --url URL / --version REF / --path DIR / --username USER
#
# 使用示例：
#   deploy-build-env --url git@host:group/build_env.git --version hexa/v0.1.0
#   bash /workspaces/build_env/deploy.bash --project build_env --url ... --version hexa/dev
#
# 注意事项：
#   1. --url / --version 未给则读 BUILD_ENV_URL / BUILD_ENV_VERSION
#      (或小写 build_env_url / build_env_version), 再不行才终端询问。
#   2. 密码不接受命令行选项。
#   3. 被 source 时会立刻导出 BUILD_ENV_ROOT / PATH; 直接执行则只改 ~/.bashrc, 当前 shell 需再 source。
#   4. 管道执行时 BASH_SOURCE 为空, 仍会入场; 交互从 /dev/tty 读。
#
deploy-build-env() {
  local url="" version="" dest="" username="" 
  local path_set=0 shallow=0

  if [[ "${1:-}" == "clone" ]]; then
    shift
  fi

  while [[ $# -gt 0 ]]; do
    case "$1" in
    -h | --help | help)
      _deploy_self_usage
      return 0
      ;;
    --url)
      [[ -n "${2-}" ]] || {
        _deploy_self_error "--url 需要地址"
        return 2
      }
      url="$2"
      shift 2
      ;;
    --url=*)
      url="${1#--url=}"
      shift
      ;;
    --version | --ref | --branch)
      [[ -n "${2-}" ]] || {
        _deploy_self_error "$1 需要版本 (分支或 tag)"
        return 2
      }
      version="$2"
      shift 2
      ;;
    --version=* | --ref=* | --branch=*)
      version="${1#*=}"
      shift
      ;;
    --path)
      [[ -n "${2-}" ]] || {
        _deploy_self_error "--path 需要目录"
        return 2
      }
      dest="$2"
      path_set=1
      shift 2
      ;;
    --path=*)
      dest="${1#--path=}"
      path_set=1
      shift
      ;;
    --username | --user)
      [[ -n "${2-}" ]] || {
        _deploy_self_error "$1 需要用户名"
        return 2
      }
      username="$2"
      shift 2
      ;;
    --username=* | --user=*)
      username="${1#*=}"
      shift
      ;;
    *)
      _deploy_self_error "未知参数: $1"
      return 2
      ;;
    esac
  done

  if [[ -z "${url}" ]]; then
    url="$(_deploy_self_from_env url)"
    if [[ -n "${url}" ]]; then
      __setup_log "使用环境变量 URL: ${url}"
    else
      url="$(_deploy_self_read_tty "Git URL: ")" || return 2
      [[ -n "${url}" ]] || {
        _deploy_self_error "URL 为空"
        return 2
      }
    fi
  fi
  if [[ -z "${version}" ]]; then
    version="$(_deploy_self_from_env version)"
    if [[ -n "${version}" ]]; then
      __setup_log "使用环境变量 version: ${version}"
    else
      version="$(_deploy_self_read_tty "Version (branch or tag): ")" || return 2
      [[ -n "${version}" ]] || {
        _deploy_self_error "version 为空"
        return 2
      }
    fi
  fi

  if [[ "${path_set}" -eq 0 ]]; then
    dest="${BUILD_ENV_DEFAULT_PATH}"
    shallow=1
    __setup_log "未指定 --path, 脚本库模式: ${dest} (--depth=1 --single-branch)"
  fi
  if [[ "${dest}" != /* ]]; then
    dest="$(pwd)/${dest}"
  fi
  local parent
  parent="$(dirname -- "${dest}")"
  if [[ ! -d "${parent}" ]]; then
    _deploy_self_error "父目录不存在: ${parent}"
    return 2
  fi
  dest="$(cd "${parent}" && pwd)/$(basename -- "${dest}")"
  _deploy_self_wipe_tmp_dest "${dest}"

  local clone_url
  clone_url="$(_deploy_self_url_with_user "${url}" "${username}")"

  unset DEPLOY_GIT_USERNAME DEPLOY_GIT_PASSWORD
  if [[ -n "${username}" ]]; then
    local pass
    pass="$(_deploy_self_prompt_password "${username}")" || return 2
    export DEPLOY_GIT_USERNAME="${username}"
    export DEPLOY_GIT_PASSWORD="${pass}"
  fi

  _deploy_self_clone "${clone_url}" "${version}" "${dest}" "${shallow}" || {
    unset DEPLOY_GIT_PASSWORD
    return 2
  }
  unset DEPLOY_GIT_PASSWORD DEPLOY_GIT_USERNAME

  if [[ ! -f "${dest}/setup.bash" || ! -f "${dest}/deploy.bash" ]]; then
    __setup_warn "检出目录缺少 setup.bash 或 deploy.bash: ${dest}"
  else
    chmod +x "${dest}/setup.bash" "${dest}/deploy.bash" 2>/dev/null || true
  fi

  _deploy_self_export_path "${dest}"
  _deploy_self_upsert_bashrc "${dest}"

  __setup_log "BUILD_ENV_ROOT=${BUILD_ENV_ROOT}"
  __setup_log "当前 shell 若尚未生效: source ~/.bashrc  或  export PATH=\"${dest}:\${PATH}\""
}

# 直接执行、管道执行 (BASH_SOURCE 为空)、或根入口设置 DEPLOY_SELF_INVOKE=1 时入场。
# 被 setup.bash source 时不要消费宿主的位置参数。
if [[ "${DEPLOY_SELF_INVOKE:-}" == "1" ]]; then
  deploy-build-env "$@"
elif [[ "${BASH_SOURCE[0]:-}" == "$0" ]]; then
  deploy-build-env "$@"
elif [[ -z "${BASH_SOURCE[0]:-}" && "${0##*/}" == "bash" ]]; then
  deploy-build-env "$@"
fi
