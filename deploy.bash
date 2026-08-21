#!/usr/bin/env bash
#
# ---
# date: 2026-08-21
# description: 多工程部署统一入口。按 --project 透传到工程 .deploy/ 或 build_env 自身的 common/setup/deploy.bash。
# ---

readonly DEPLOY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DEPLOY_META_PROJECT="build_env"
readonly EXIT_USAGE=2

# _deploy_usage
#
# 功能描述：
#   打印统一入口用法到 stdout。
#
# 参数：
#   (无)
#
_deploy_usage() {
  cat <<EOF
build_env 部署统一入口

按 --project 选择目标, 再把剩余参数原样交给对应脚本。本入口不解释工程脚本的选项。

Usage:
  $(basename "$0") --project <name> [install|dump|hostrc] [args...]
  $(basename "$0") --project ${DEPLOY_META_PROJECT} --url <git-url> --version <ref> [args...]
  $(basename "$0") --list
  $(basename "$0") --help

Options:
  --project NAME   工作区工程目录名, 对应 ${DEPLOY_ROOT}/<name>/.deploy/
                   或保留名 ${DEPLOY_META_PROJECT} (本仓库, 见 common/setup/deploy.bash)
  install          透传到 .deploy/install.bash (工作区工程默认)
  dump             透传到 .deploy/dump.bash
  hostrc           透传到 .deploy/hostrc.env (只提供环境, 须 source)
  --list           列出可部署目标
  -h, --help       显示本帮助; 已指定 --project 时透传给目标脚本

说明:
  1. 只消费第一处 --project 作为工作区工程名; 其后的 --project / -p 原样透传
     (install.bash / dump.bash 自己的项目 id, 如 auriga)。
  2. 工作区工程未写 install|dump|hostrc 时默认 install.bash。
  3. --project ${DEPLOY_META_PROJECT} 不消费 install|dump|hostrc, 全部透传到
     common/setup/deploy.bash。
  4. hostrc.env 只向当前 shell 提供变量与函数, 不改系统文件。须:
       source ${DEPLOY_ROOT}/deploy.bash --project <name> hostrc

Examples:
  $(basename "$0") --project ${DEPLOY_META_PROJECT} --url git@host:group/build_env.git --version hexa/v0.1.0
  $(basename "$0") --project ai2-robot --help
  $(basename "$0") --project ai2-robot --download --version v1.2.x --platform aarch --output ./debs
  source $(basename "$0") --project ai2-robot hostrc
  $(basename "$0") --project ai2-robot dump --date today

Available projects:
$(_deploy_format_projects)
EOF
}

# _deploy_project_name_ok
#
# 功能描述：
#   校验工作区工程目录名, 须为单层路径分量。
#
# 参数：
#   $1: name - 工程目录名。
#
_deploy_project_name_ok() {
  local name="$1"
  [[ "${name}" =~ ^[A-Za-z][A-Za-z0-9._-]*$ ]]
}

# _deploy_is_meta_project
#
# 功能描述：
#   是否为保留名 build_env (本仓库, 不是子目录工程)。
#
# 参数：
#   $1: name - 工程名。
#
_deploy_is_meta_project() {
  [[ "$1" == "${DEPLOY_META_PROJECT}" ]]
}

# _deploy_list_projects
#
# 功能描述：
#   打印可部署目标: 先 build_env, 再仓库根下含 .deploy/ 的工程目录名。
#
# 参数：
#   (无)
#
_deploy_list_projects() {
  local dir name
  printf '%s\n' "${DEPLOY_META_PROJECT}"
  shopt -s nullglob
  for dir in "${DEPLOY_ROOT}"/*/.deploy; do
    [[ -d "${dir}" ]] || continue
    name="$(basename "$(dirname "${dir}")")"
    _deploy_project_name_ok "${name}" || continue
    printf '%s\n' "${name}"
  done
  shopt -u nullglob
}

# _deploy_format_projects
#
# 功能描述：
#   列出工程及其可用脚本, 供帮助与报错使用。
#
# 参数：
#   (无)
#
_deploy_format_projects() {
  local name dir scripts
  local found=0
  while IFS= read -r name; do
    [[ -n "${name}" ]] || continue
    found=1
    if _deploy_is_meta_project "${name}"; then
      printf '  %s  (clone)\n' "${name}"
      continue
    fi
    dir="${DEPLOY_ROOT}/${name}/.deploy"
    scripts=""
    [[ -f "${dir}/install.bash" ]] && scripts+="install "
    [[ -f "${dir}/dump.bash" ]] && scripts+="dump "
    [[ -f "${dir}/hostrc.env" ]] && scripts+="hostrc "
    printf '  %s  (%s)\n' "${name}" "${scripts% }"
  done < <(_deploy_list_projects)
  if [[ "${found}" -eq 0 ]]; then
    printf '  (无)\n'
  fi
}

# _deploy_die
#
# 功能描述：
#   向 stderr 打印错误。被 source 时 return, 直接执行时 exit。
#
# 参数：
#   $1: message - 错误内容。
#   $2: code - 退出码 (默认 2)。
#
_deploy_die() {
  printf 'error: %s\n' "$1" >&2
  if [[ "${_DEPLOY_SOURCED:-0}" == "1" ]]; then
    return "${2:-${EXIT_USAGE}}"
  fi
  exit "${2:-${EXIT_USAGE}}"
}

# _deploy_resolve_script
#
# 功能描述：
#   解析要执行的脚本路径。build_env 固定为 common/setup/deploy.bash。
#
# 参数：
#   $1: project - 工作区工程名或 build_env。
#   $2: command - install / dump / hostrc (build_env 忽略)。
#
_deploy_resolve_script() {
  local project="$1"
  local command="$2"

  if _deploy_is_meta_project "${project}"; then
    local meta="${DEPLOY_ROOT}/common/setup/deploy.bash"
    if [[ ! -f "${meta}" ]]; then
      _deploy_die "找不到 ${meta}" || return $?
    fi
    printf '%s\n' "${meta}"
    return 0
  fi

  local dir="${DEPLOY_ROOT}/${project}/.deploy"
  local script
  if [[ "${command}" == "hostrc" ]]; then
    script="${dir}/hostrc.env"
  else
    script="${dir}/${command}.bash"
  fi

  if [[ ! -d "${dir}" ]]; then
    _deploy_die "工程无 .deploy: ${project}

Available projects:
$(_deploy_format_projects)" || return $?
  fi
  if [[ ! -f "${script}" ]]; then
    _deploy_die "找不到 ${script}

Available projects:
$(_deploy_format_projects)" || return $?
  fi
  printf '%s\n' "${script}"
}

# _deploy_invoke
#
# 功能描述：
#   source 或 exec 目标脚本。被 source 时保留函数与环境变量。
#
# 参数：
#   $1: script - 脚本路径。
#   $@: 透传参数。
#
_deploy_invoke() {
  local script="$1"
  shift
  if [[ "${_DEPLOY_SOURCED:-0}" == "1" ]]; then
    if _deploy_is_meta_project "${_DEPLOY_PROJECT:-}"; then
      export DEPLOY_SELF_INVOKE=1
    fi
    # shellcheck disable=SC1090
    source "${script}" "$@"
    return $?
  fi
  exec bash "${script}" "$@"
}

# deploy
#
# 功能描述：
#   解析 --project 与可选的 install/dump/hostrc, 其余参数透传。
#
# 参数：
#   --project NAME - 工作区工程目录名或 build_env。
#   $1: install|dump|hostrc - 可选; 工作区工程默认 install。
#   $@: 透传给对应脚本。
#
# 使用示例：
#   deploy --project build_env --url git@host:group/build_env.git --version hexa/dev
#   deploy --project ai2-robot --download --version v1.2.x --output ./debs
#   source deploy.bash --project ai2-robot hostrc
#
# 注意事项：
#   1. 只消费第一处 --project; 其后的 --project / -p 原样透传。
#   2. 不消费 -p, 以免抢走 dump.bash 的 updater 项目 id。
#   3. --project build_env 不把 install|dump|hostrc 当调度命令。
#   4. hostrc 只提供环境, 必须 source, 不改系统文件。
#
deploy() {
  local project="" command=""
  local -a forward=()

  while [[ $# -gt 0 ]]; do
    case "$1" in
    --)
      shift
      forward+=("$@")
      break
      ;;
    --project)
      if [[ -n "${project}" ]]; then
        forward+=("$1")
        [[ -n "${2-}" ]] || {
          _deploy_die "$1 需要值" || return $?
        }
        forward+=("$2")
        shift 2
        continue
      fi
      [[ -n "${2-}" ]] || {
        _deploy_die "--project 需要工程名" || return $?
      }
      project="$2"
      shift 2
      ;;
    --project=*)
      if [[ -n "${project}" ]]; then
        forward+=("$1")
        shift
        continue
      fi
      project="${1#--project=}"
      [[ -n "${project}" ]] || {
        _deploy_die "--project 需要工程名" || return $?
      }
      shift
      ;;
    --list)
      if [[ -z "${project}" && -z "${command}" && ${#forward[@]} -eq 0 ]]; then
        _deploy_format_projects
        return 0
      fi
      forward+=("$1")
      shift
      ;;
    -h | --help)
      if [[ -z "${project}" && -z "${command}" && ${#forward[@]} -eq 0 ]]; then
        _deploy_usage
        return 0
      fi
      forward+=("$1")
      shift
      ;;
    install | dump | hostrc)
      if _deploy_is_meta_project "${project}"; then
        forward+=("$1")
        shift
        continue
      fi
      if [[ -z "${command}" && ${#forward[@]} -eq 0 ]]; then
        command="$1"
        shift
      else
        forward+=("$1")
        shift
      fi
      ;;
    *)
      forward+=("$1")
      shift
      ;;
    esac
  done

  if [[ -z "${project}" ]]; then
    _deploy_usage >&2
    _deploy_die "需要 --project <工程名>" || return $?
  fi
  _deploy_project_name_ok "${project}" || {
    _deploy_die "非法工程名: ${project}" || return $?
  }

  if ! _deploy_is_meta_project "${project}"; then
    : "${command:=install}"
  else
    command="clone"
  fi

  if [[ "${command}" == "hostrc" && "${_DEPLOY_SOURCED:-0}" != "1" ]]; then
    _deploy_die "hostrc.env 只提供环境变量, 请: source ${DEPLOY_ROOT}/deploy.bash --project ${project} hostrc" || return $?
  fi

  local script
  script="$(_deploy_resolve_script "${project}" "${command}")" || return $?
  _DEPLOY_PROJECT="${project}"
  _deploy_invoke "${script}" "${forward[@]+"${forward[@]}"}"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  set -Eeuo pipefail
  _DEPLOY_SOURCED=0
  deploy "$@"
elif [[ $# -gt 0 ]]; then
  _DEPLOY_SOURCED=1
  deploy "$@"
fi
