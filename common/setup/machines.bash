# ---
# date: 2026-08-21
# description: 扫描 common/setup/machines/<model>/, 注册机型入口 (类似 entrypoint_additions)。
# ---

# 预留子命令, 不可作为机型目录名 (会被 setup-host 内建命令抢先匹配)。
__SETUP_RESERVED_COMMANDS='help user sudoers home-bind home_bind hosts ssh workspaces workspace hostrc bashrc docker docker-daemon daemon desktop git wifi ip-fixed ip_fixed ip network display ros2 install-ros2 install all machine machines list list-machines'

# __setup_machine_name_ok
#
# 功能描述：
#   校验机型目录名: 须匹配 `^[A-Za-z][A-Za-z0-9_-]*$`, 且不以 `_` 开头。
#
# 参数：
#   $1: name - 机型目录名。
#
__setup_machine_name_ok() {
  local name="$1"
  [[ "${name}" =~ ^[A-Za-z][A-Za-z0-9_-]*$ ]] || return 1
  [[ "${name}" != _* ]] || return 1
}

# __setup_machine_names
#
# 功能描述：
#   列出 SETUP_MACHINES_DIR 下已发现的机型目录名 (忽略 `_` 前缀与非目录)。
#
# 参数：
#   (无)
#
__setup_machine_names() {
  local d name old
  old="$(shopt -p nullglob)"
  shopt -s nullglob
  for d in "${SETUP_MACHINES_DIR}"/*/; do
    name="$(basename "${d%/}")"
    __setup_machine_name_ok "${name}" || continue
    printf '%s\n' "${name}"
  done | LC_ALL=C sort -u
  eval "${old}"
}

# __setup_machine_dir
#
# 功能描述：
#   返回机型目录的绝对路径。
#
# 参数：
#   $1: name - 机型目录名。
#
__setup_machine_dir() {
  printf '%s' "${SETUP_MACHINES_DIR}/${1}"
}

# __setup_machine_exists
#
# 功能描述：
#   判断机型名合法且对应目录存在。
#
# 参数：
#   $1: name - 机型目录名。
#
__setup_machine_exists() {
  local name="$1"
  __setup_machine_name_ok "${name}" || return 1
  [[ -d "$(__setup_machine_dir "${name}")" ]]
}

# __setup_machine_label
#
# 功能描述：
#   读取机型展示名: 优先 `label` 文件首行, 否则 env.bash 中 `# label:` 注释。
#
# 参数：
#   $1: name - 机型目录名。
#
__setup_machine_label() {
  local dir label_file envfile
  dir="$(__setup_machine_dir "$1")"
  label_file="${dir}/label"
  envfile="${dir}/env.bash"
  if [[ -f "${label_file}" ]]; then
    head -n 1 "${label_file}"
    return 0
  fi
  if [[ -f "${envfile}" ]]; then
    sed -n 's/^# label:[[:space:]]*//p' "${envfile}" | head -n 1
    return 0
  fi
}

# __setup_machine_is_reserved
#
# 功能描述：
#   判断名称是否与 setup-host 内建子命令冲突。
#
# 参数：
#   $1: cmd - 待检查名称。
#
__setup_machine_is_reserved() {
  local cmd="$1" item
  for item in ${__SETUP_RESERVED_COMMANDS}; do
    [[ "${cmd}" == "${item}" ]] && return 0
  done
  return 1
}

# __setup_machine_default
#
# 功能描述：
#   设置机型默认环境变量。用户预先 export 的值优先; 上一机型注入的默认值可被覆盖。
#
# 参数：
#   $1: var - 变量名。
#   $2: val - 默认值。
#
__setup_machine_default() {
  local var="$1"
  local val="$2"
  local mark="__SETUP_MACHINE_DEFAULT_${var}"
  if [[ "${!mark:-}" == "1" ]]; then
    printf -v "${var}" '%s' "${val}"
    export "${var}"
    return 0
  fi
  if [[ -z "${!var:-}" ]]; then
    printf -v "${var}" '%s' "${val}"
    export "${var}"
    printf -v "${mark}" '1'
    export "${mark}"
  fi
}

# __setup_machine_begin
#
# 功能描述：
#   选定机型: 导出 SETUP_MACHINE / SETUP_MACHINE_DIR, 并 source 该机型 env.bash。
#
# 参数：
#   $1: name - 机型目录名。
#
__setup_machine_begin() {
  local name="$1"
  local dir envfile
  if ! __setup_machine_exists "${name}"; then
    __setup_error "unknown machine: ${name}"
    return 1
  fi
  dir="$(__setup_machine_dir "${name}")"
  export SETUP_MACHINE="${name}"
  export SETUP_MACHINE_DIR="${dir}"
  envfile="${dir}/env.bash"
  if [[ -f "${envfile}" ]]; then
    # shellcheck disable=SC1090
    source "${envfile}"
  fi
  __setup_log "machine: ${name} (${dir})"
}

# __setup_machine_additions
#
# 功能描述：
#   按文件名排序扫描 additions/ 与机型目录下的 `[0-9]*`。
#   文件名含 `.user.` 时以 SETUP_USERNAME 执行; 否则 source (可调用 setup-*)。
#
# 参数：
#   (无) 依赖 SETUP_MACHINE_DIR。
#
# 注意事项：
#   1. 须先调用 __setup_machine_begin。
#
__setup_machine_additions() {
  local dir="${SETUP_MACHINE_DIR:-}"
  local addition base old
  local -a files=()

  if [[ -z "${dir}" || ! -d "${dir}" ]]; then
    __setup_error "SETUP_MACHINE_DIR is not set; call __setup_machine_begin first."
    return 1
  fi

  old="$(shopt -p nullglob)"
  shopt -s nullglob
  if [[ -d "${dir}/additions" ]]; then
    files+=("${dir}/additions/"*)
  fi
  files+=("${dir}/"[0-9]*)
  eval "${old}"

  if ((${#files[@]} == 0)); then
    return 0
  fi

  while IFS= read -r addition; do
    [[ -n "${addition}" ]] || continue
    [[ -f "${addition}" ]] || continue
    [[ "${addition}" == *.bash || "${addition}" == *.sh ]] || continue
    base="$(basename "${addition}")"
    if [[ "${base}" =~ \.user\. ]]; then
      __setup_log "Running machine addition as ${SETUP_USERNAME}: ${addition}"
      if [[ "${SETUP_USERNAME}" == "${USER}" && "$(id -u)" -ne 0 ]]; then
        bash "${addition}" || return 1
      else
        __setup_sudo -u "${SETUP_USERNAME}" bash "${addition}" || return 1
      fi
    else
      __setup_log "Sourcing machine addition: ${addition}"
      # shellcheck disable=SC1090
      source "${addition}" || return 1
    fi
  done < <(printf '%s\n' "${files[@]}" | awk -F/ '{print $NF "\t" $0}' | LC_ALL=C sort | cut -f2-)
}

# __setup_run_machine
#
# 功能描述：
#   默认机型入口: 加载 env, 跑 setup-host all, 再扫描 additions。
#
# 参数：
#   $1: name - 机型目录名。
#   $@: 透传给 setup-host all。
#
__setup_run_machine() {
  local name="$1"
  shift || true
  __setup_machine_begin "${name}" || return 1
  setup-host all "$@" || return 1
  __setup_machine_additions
}

# __setup_define_default_machine_entry
#
# 功能描述：
#   为没有自定义 setup-<name> 的机型生成默认入口函数。
#
# 参数：
#   $1: name - 机型目录名。
#
__setup_define_default_machine_entry() {
  local name="$1"
  eval "setup-${name}() { __setup_run_machine $(printf '%q' "${name}") \"\$@\"; }"
}

# __setup_register_machines
#
# 功能描述：
#   source 各机型 setup.bash (只应定义函数); 若未提供 setup-<name> 则生成默认入口。
#
# 参数：
#   (无)
#
__setup_register_machines() {
  local name dir entry
  while IFS= read -r name; do
    [[ -n "${name}" ]] || continue
    if __setup_machine_is_reserved "${name}"; then
      __setup_warn "skip machine '${name}': name collides with a built-in command"
      continue
    fi
    dir="$(__setup_machine_dir "${name}")"
    entry="${dir}/setup.bash"
    if [[ -f "${entry}" ]]; then
      # shellcheck disable=SC1090
      source "${entry}"
    fi
    if ! declare -F "setup-${name}" >/dev/null 2>&1; then
      __setup_define_default_machine_entry "${name}"
    fi
  done < <(__setup_machine_names)
}

# __setup_list_machines
#
# 功能描述：
#   打印已发现机型及 label, 供 setup-host help 使用。
#
# 参数：
#   (无)
#
__setup_list_machines() {
  local name label
  while IFS= read -r name; do
    [[ -n "${name}" ]] || continue
    label="$(__setup_machine_label "${name}")"
    if [[ -n "${label}" ]]; then
      printf '  %-16s %s\n' "${name}" "${label}"
    else
      printf '  %s\n' "${name}"
    fi
  done < <(__setup_machine_names)
}

__setup_register_machines
