# ---
# date: 2026-08-20
# description: 多工程身份加载。由 RUNME.bash / bashrc 在其它 .env 脚本之前 source。
# ---
#
# 约定:
#   PROJECT_WORKDIR  工作区根(含 RUNME.bash 的目录)
#   PROJECT_NAME     工程目录名
#   PROJECT_PREFIX   工程别名环境变量前缀 (由各工程 .config/project.env 给出)
#   PROJECT_CONFIG   ${PROJECT_WORKDIR}/.config
#
# 各工程在 .config/project.env 中覆盖默认值。
# 加载后会同时导出 ${PREFIX}_WORKDIR / ${PREFIX}_REPO / ${PREFIX}_CORE / ${PREFIX}_DEV。

__project_infer_workdir() {
  if [[ -n "${PROJECT_WORKDIR:-}" ]]; then
    printf '%s' "${PROJECT_WORKDIR}"
    return 0
  fi
  if [[ -n "${AI2_WORKDIR:-}" ]]; then
    printf '%s' "${AI2_WORKDIR}"
    return 0
  fi
  local here
  here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  if [[ "$(basename "${here}")" == "common" ]]; then
    echo "error: PROJECT_WORKDIR is not set (sourced via common/ path)." >&2
    return 1
  fi
  printf '%s' "${here}"
}

__project_export_prefixed() {
  local prefix="$1"
  [[ -n "${prefix}" ]] || return 0
  export "${prefix}_WORKDIR=${PROJECT_WORKDIR}"
  export "${prefix}_REPO=${PROJECT_REPO}"
  export "${prefix}_CORE=${PROJECT_CORE}"
  export "${prefix}_DEV=${PROJECT_DEV}"
}

__project_load() {
  local wd envfile
  wd="$(__project_infer_workdir)" || return 1
  export PROJECT_WORKDIR="${wd}"
  # 以工作区目录名为准, 避免环境中残留的 PROJECT_NAME 串到其它工程。
  export PROJECT_NAME="$(basename "${PROJECT_WORKDIR}")"
  export PROJECT_CONFIG="${PROJECT_WORKDIR}/.config"

  unset PROJECT_PREFIX PROJECT_CONTAINER PROJECT_CORE_REPOS PROJECT_DEV_REPOS
  unset PROJECT_SOURCE_BRANCH PROJECT_TARGET_BRANCH PROJECT_TAG_PREFIX
  unset PROJECT_CM_RELPATH PROJECT_CD_KEY PROJECT_ENTRY_ALIAS

  envfile="${PROJECT_CONFIG}/project.env"
  if [[ -f "${envfile}" ]]; then
    set -a
    # shellcheck disable=SC1090
    source "${envfile}"
    set +a
  fi

  : "${PROJECT_PREFIX:=$(printf '%s' "${PROJECT_NAME}" | tr '[:lower:]-' '[:upper:]_')}"
  : "${PROJECT_CONTAINER:=${PROJECT_NAME}_cpp}"
  : "${PROJECT_CORE_REPOS:=${PROJECT_NAME}-core.repos}"
  : "${PROJECT_DEV_REPOS:=${PROJECT_NAME}-dev.repos}"
  : "${PROJECT_SOURCE_BRANCH:=${PROJECT_NAME}/dev}"
  : "${PROJECT_TARGET_BRANCH:=${PROJECT_NAME}/main}"
  : "${PROJECT_TAG_PREFIX:=${PROJECT_NAME}}"
  : "${PROJECT_CM_RELPATH:=}"
  : "${PROJECT_CD_KEY:=${PROJECT_NAME}}"
  : "${PROJECT_ENTRY_ALIAS:=${PROJECT_CD_KEY}}"

  export PROJECT_PREFIX PROJECT_CONTAINER
  export PROJECT_CORE_REPOS PROJECT_DEV_REPOS
  export PROJECT_SOURCE_BRANCH PROJECT_TARGET_BRANCH PROJECT_TAG_PREFIX
  export PROJECT_CM_RELPATH PROJECT_CD_KEY PROJECT_ENTRY_ALIAS PROJECT_CONFIG

  export PROJECT_REPO="${PROJECT_CONFIG}/manifests.repos"
  export PROJECT_CORE="${PROJECT_WORKDIR}/.ci/manifests/${PROJECT_CORE_REPOS}"
  export PROJECT_DEV="${PROJECT_WORKDIR}/.ci/manifests/${PROJECT_DEV_REPOS}"

  __project_export_prefixed "${PROJECT_PREFIX}"

  # ai2-robot 既有脚本与文档大量使用 AI2_*; 仅在本工程下继续作为主别名。
  if [[ "${PROJECT_PREFIX}" == "AI2" ]]; then
    export AI2_WORKDIR="${PROJECT_WORKDIR}"
    export AI2_REPO="${PROJECT_REPO}"
    export AI2_CORE="${PROJECT_CORE}"
    export AI2_DEV="${PROJECT_DEV}"
  fi
}

__project_load
