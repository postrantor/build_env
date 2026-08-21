# ---
# date: 2026-08-21
# description: 宿主机步骤 (用户 / hosts / ssh / 桌面) 与 setup-host 调度器。
# ---

# setup-user
#
# 功能描述：
#   确保用户存在并加入 sudo / docker / 硬件组。用户已存在时只补组; 组不存在则跳过。
#   不执行 su, 也不改密码策略以外的 login shell。
#
# 参数：
#   $1: username - 目标用户 (默认: SETUP_USERNAME)。
#
# 使用示例：
#   setup-user
#   setup-user robot
#
# 注意事项：
#   1. 拒绝配置 root。
#   2. visudo / 家目录 bind 见 setup-sudoers / setup-home-bind。
#   3. 非 Jetson 机器没有 gpio 等组时会跳过。
#
setup-user() {
  local user="${1:-${SETUP_USERNAME}}"
  local group groups
  SETUP_USERNAME="${user}"

  if [[ -z "${user}" || "${user}" == "root" ]]; then
    __setup_error "refusing to configure user '${user}'"
    return 1
  fi

  __setup_warn "Will ensure user ${user} exists and is in sudo/docker/hardware groups."
  __setup_confirm "create/update user ${user}?" || return 0

  if ! getent passwd "${user}" >/dev/null 2>&1; then
    if [[ -t 0 && "${SETUP_NONINTERACTIVE:-}" != "1" ]]; then
      __setup_sudo adduser --force-badname "${user}" || return 1
    else
      __setup_sudo adduser --force-badname --disabled-password --gecos "" "${user}" || return 1
      __setup_log "created ${user} without a password; set one with: sudo passwd ${user}"
    fi
  else
    __setup_log "user ${user} already exists."
  fi

  groups="sudo docker audio video render i2c gdm weston-launch gpio ai2rob"
  for group in ${groups}; do
    if __setup_group_exists "${group}"; then
      __setup_sudo usermod -aG "${group}" "${user}" || return 1
    else
      __setup_log "skip missing group: ${group}"
    fi
  done

  __setup_log "user ${user} groups: $(id -nG "${user}" 2>/dev/null || true)"
}

# setup-sudoers
#
# 功能描述：
#   写入 `/etc/sudoers.d/build_env` 的 NOPASSWD 片段。额外用户由 SETUP_SUDOERS_EXTRA 空格分隔。
#
# 参数：
#   $1: username - 主用户 (默认: SETUP_USERNAME)。
#
# 使用示例：
#   setup-sudoers
#   SETUP_SUDOERS_EXTRA=robot setup-sudoers
#
# 注意事项：
#   1. 文件名不含点号, 避免 sudo 忽略。
#   2. 写入前用 visudo -cf 校验生成片段。
#
setup-sudoers() {
  local user="${1:-${SETUP_USERNAME}}"
  local dest="/etc/sudoers.d/build_env"
  local extra="${SETUP_SUDOERS_EXTRA:-}"
  local tmp name
  SETUP_USERNAME="${user}"

  __setup_warn "Will install ${dest} with NOPASSWD for ${user}${extra:+ and ${extra}}."
  __setup_confirm "write ${dest}?" || return 0

  tmp="$(mktemp)"
  {
    printf '%s\n' "${SETUP_MARKER_BEGIN}"
    printf '%s ALL=(ALL) NOPASSWD: ALL\n' "${user}"
    for name in ${extra}; do
      [[ "${name}" == "${user}" ]] && continue
      printf '%s ALL=(ALL) NOPASSWD: ALL\n' "${name}"
    done
    printf '%s\n' "${SETUP_MARKER_END}"
  } >"${tmp}"

  if ! __setup_sudo visudo -cf "${tmp}"; then
    rm -f "${tmp}"
    __setup_error "visudo rejected generated sudoers fragment"
    return 1
  fi

  __setup_sudo install -m 440 -o root -g root "${tmp}" "${dest}"
  rm -f "${tmp}"
  __setup_log "wrote ${dest}"
}

# setup-home-bind
#
# 功能描述：
#   仅当 SETUP_HOME_STORE_ROOT (`/home/data`) 存在时, 把家目录 bind 到 `/home/data/<user>`。
#
# 参数：
#   $1: username - 目标用户 (默认: SETUP_USERNAME)。
#
# 使用示例：
#   setup-home-bind
#
# 注意事项：
#   1. 已是 bind 挂载则跳过。
#   2. 移动家目录有风险, 交互时会再确认一次。
#
setup-home-bind() {
  local user="${1:-${SETUP_USERNAME}}"
  local store_root="${SETUP_HOME_STORE_ROOT}"
  local home store
  SETUP_USERNAME="${user}"
  home="${SETUP_HOME:-$(__setup_target_home "${user}")}"
  store="${SETUP_HOME_STORE:-${store_root}/${user}}"

  if [[ ! -d "${store_root}" ]]; then
    __setup_log "skip home bind: ${store_root} does not exist."
    return 0
  fi

  if [[ "$(findmnt -n -o TARGET --target "${home}" 2>/dev/null || true)" == "${home}" ]] \
    && findmnt -n -o OPTIONS --target "${home}" 2>/dev/null | grep -q bind; then
    __setup_log "${home} is already a bind mount."
    return 0
  fi

  __setup_warn "Will move ${home} -> ${store} (if needed) and bind-mount it via /etc/fstab."
  __setup_confirm "bind ${store} onto ${home}?" || return 0

  if [[ -d "${home}" && ! -d "${store}" ]]; then
    __setup_warn "This relocates the home directory of ${user}."
    __setup_confirm "mv ${home} ${store} now?" || return 0
    __setup_sudo mkdir -p "${store_root}"
    __setup_sudo mv "${home}" "${store}" || return 1
    __setup_sudo mkdir -p "${home}"
  fi

  __setup_sudo mkdir -p "${store}" "${home}"
  if getent passwd "${user}" >/dev/null 2>&1; then
    __setup_sudo chown -R "${user}:${user}" "${store}" "${home}"
  fi

  __setup_upsert_marked /etc/fstab "${store}   ${home}   none   bind   0   0" || return 1
  __setup_sudo mount -a || return 1
  __setup_log "bind-mounted ${store} -> ${home}"
}

# create-user
#
# 功能描述：
#   `setup-user` 的别名, 兼容旧脚本名。
#
# 参数：
#   $@: 透传给 setup-user。
#
# 使用示例：
#   create-user robot
#
create-user() {
  setup-user "$@"
}

# setup-hosts
#
# 功能描述：
#   把 LAN hosts 片段 upsert 进 `/etc/hosts`。替换 IP 用 SETUP_HOSTS_IP (默认 192.168.66.100)。
#
# 参数：
#   (无)
#
# 使用示例：
#   setup-hosts
#   SETUP_HOSTS_IP=192.168.66.100 setup-hosts
#
setup-hosts() {
  local hp="/etc/hosts"
  local body
  local src="${SETUP_TEMPLATE_DIR}/hosts"

  body="$(__setup_render "${src}")" || return 1

  __setup_warn "Will upsert the following into ${hp}:"
  printf '%s\n' "${body}" >&2
  __setup_confirm "update ${hp}?" || return 0

  __setup_upsert_marked "${hp}" "${body}" || return 1
  __setup_log "updated ${hp}"
}

# __setup_ssh_dir
#
# 功能描述：
#   返回目标用户的 `~/.ssh` 路径。
#
# 参数：
#   (无) 使用 SETUP_USERNAME。
#
__setup_ssh_dir() {
  printf '%s' "$(__setup_target_home "${SETUP_USERNAME}")/.ssh"
}

# __setup_ssh_ensure_include
#
# 功能描述：
#   确保 `~/.ssh/config` 含 `Include local.d/*.ssh`; 缺失则插到文件开头。
#
# 参数：
#   $1: ssh_dir - `~/.ssh` 目录。
#   $2: user - 属主。
#
__setup_ssh_ensure_include() {
  local ssh_dir="$1"
  local user="$2"
  local cfg="${ssh_dir}/config"
  local include_line='Include local.d/*.ssh'
  local tmp

  if __setup_sudo test -f "${cfg}" \
    && __setup_sudo grep -qE '^[[:space:]]*Include[[:space:]]+local\.d/' "${cfg}"; then
    return 0
  fi

  tmp="$(mktemp)"
  printf '%s\n\n' "${include_line}" >"${tmp}"
  if __setup_sudo test -f "${cfg}"; then
    __setup_sudo cat "${cfg}" >>"${tmp}"
  fi
  __setup_install_user_file "${tmp}" "${cfg}" "${user}" 600
  rm -f "${tmp}"
}

# __setup_ssh_fix_perms
#
# 功能描述：
#   修正 `~/.ssh` 目录与密钥文件权限 (700 / 600 / 644)。
#
# 参数：
#   $1: ssh_dir - `~/.ssh` 目录。
#
__setup_ssh_fix_perms() {
  local ssh_dir="$1"
  __setup_sudo chmod 700 "${ssh_dir}"
  if [[ -d "${ssh_dir}" ]]; then
    __setup_sudo find "${ssh_dir}" -type f -name '*.pub' -exec chmod 644 {} \;
    __setup_sudo find "${ssh_dir}" -type f ! -name '*.pub' \( -name 'id_*' -o -name '*_key' \) -exec chmod 600 {} \;
    [[ -f "${ssh_dir}/config" ]] && __setup_sudo chmod 600 "${ssh_dir}/config"
    [[ -f "${ssh_dir}/authorized_keys" ]] && __setup_sudo chmod 600 "${ssh_dir}/authorized_keys"
  fi
}

# setup-ssh
#
# 功能描述：
#   准备 `~/.ssh`: 安装 `local.d/build_env.ssh` 并 Include。设置 SETUP_SSH_REPO 才会 clone 密钥仓。
#
# 参数：
#   $1: username - 目标用户 (默认: SETUP_USERNAME)。
#
# 使用示例：
#   setup-ssh
#   SETUP_SSH_REPO=git@example.com:keys.git SETUP_SSH_REPLACE=1 setup-ssh
#
# 注意事项：
#   1. SETUP_SSH_REPLACE=1 才会替换已有 `~/.ssh`。
#
setup-ssh() {
  local user="${1:-${SETUP_USERNAME}}"
  local home ssh_dir tmp rendered
  SETUP_USERNAME="${user}"
  home="$(__setup_target_home "${user}")"
  ssh_dir="$(__setup_ssh_dir)"

  __setup_warn "Will prepare ${ssh_dir} (Include local.d, optional clone)."
  __setup_confirm "configure ${ssh_dir}?" || return 0

  __setup_sudo mkdir -p "${ssh_dir}/local.d"
  __setup_sudo chown -R "${user}:${user}" "${ssh_dir}"

  if [[ -n "${SETUP_SSH_REPO:-}" ]]; then
    if [[ -d "${ssh_dir}" && -n "$(ls -A "${ssh_dir}" 2>/dev/null || true)" && "${SETUP_SSH_REPLACE:-}" != "1" ]]; then
      __setup_log "skip clone: ${ssh_dir} is not empty (set SETUP_SSH_REPLACE=1 to replace)."
    else
      tmp="$(mktemp -d)"
      git clone "${SETUP_SSH_REPO}" "${tmp}/sshd" || {
        rm -rf "${tmp}"
        return 1
      }
      if [[ "${SETUP_SSH_REPLACE:-}" == "1" && -e "${ssh_dir}" ]]; then
        __setup_sudo mv "${ssh_dir}" "${ssh_dir}.bak.$(date +%Y%m%d%H%M%S)"
      fi
      __setup_sudo mkdir -p "${home}"
      __setup_sudo rm -rf "${ssh_dir}"
      __setup_sudo mv "${tmp}/sshd" "${ssh_dir}"
      __setup_sudo chown -R "${user}:${user}" "${ssh_dir}"
      rm -rf "${tmp}"
      __setup_sudo mkdir -p "${ssh_dir}/local.d"
    fi
  fi

  rendered="$(mktemp)"
  __setup_render "${SETUP_TEMPLATE_DIR}/ssh-local.conf" >"${rendered}" || {
    rm -f "${rendered}"
    return 1
  }
  __setup_install_user_file "${rendered}" "${ssh_dir}/local.d/build_env.ssh" "${user}" 600
  rm -f "${rendered}"

  __setup_ssh_ensure_include "${ssh_dir}" "${user}"
  __setup_ssh_fix_perms "${ssh_dir}"
  __setup_log "ssh config ready under ${ssh_dir}"
}

# setup-workspaces
#
# 功能描述：
#   确保 `~/workspaces`、`/workspaces` 链接, 以及可选的 `~/<project>` 工程链接。
#   若当前仓库不在 SETUP_WORKSPACES/build_env, 则把 SETUP_ROOT 链过去 (不重复 clone)。
#
# 参数：
#   $1: project-name - 工程目录名 (也可由 SETUP_PROJECT_NAME 提供)。
#
# 使用示例：
#   setup-workspaces
#   setup-workspaces ai2-robot
#
setup-workspaces() {
  local project="${1:-${SETUP_PROJECT_NAME:-}}"
  local ws="${SETUP_WORKSPACES}"
  local link="${SETUP_WORKSPACES_LINK}"
  local dest="${ws}/build_env"
  local home

  home="$(__setup_target_home "${SETUP_USERNAME}")"
  ws="${SETUP_WORKSPACES:-${home}/workspaces}"
  dest="${ws}/build_env"

  __setup_warn "Will ensure ${ws}, ${link} -> ${ws}, and optional ~/${project:-<project>} link."
  __setup_confirm "create workspace links?" || return 0

  mkdir -p "${ws}"

  if [[ "$(realpath "${SETUP_ROOT}" 2>/dev/null || printf '%s' "${SETUP_ROOT}")" != "$(realpath "${dest}" 2>/dev/null || true)" ]]; then
    if [[ -e "${dest}" ]]; then
      __setup_log "${dest} already exists; leave it in place."
    else
      ln -s "${SETUP_ROOT}" "${dest}"
      __setup_log "linked ${dest} -> ${SETUP_ROOT}"
    fi
  fi

  if [[ -e "${link}" || -L "${link}" ]]; then
    if [[ -L "${link}" ]]; then
      __setup_sudo ln -sfn "${ws}" "${link}"
    else
      __setup_warn "${link} exists and is not a symlink; skip (set SETUP_FORCE=1 after removing it)."
      if [[ "${SETUP_FORCE:-}" == "1" ]]; then
        __setup_sudo ln -sfn "${ws}" "${link}"
      fi
    fi
  else
    __setup_sudo ln -s "${ws}" "${link}"
  fi

  if [[ -n "${project}" ]]; then
    if [[ -d "${dest}/${project}" ]]; then
      ln -sfn "${link}/build_env/${project}" "${home}/${project}"
      __setup_log "linked ${home}/${project} -> ${link}/build_env/${project}"
    else
      __setup_error "project directory not found: ${dest}/${project}"
      return 1
    fi
  fi

  __setup_log "workspaces: ${ws}  link: ${link}"
}

# setup-hostrc
#
# 功能描述：
#   安装 `~/.hostrc` 并确保 `~/.bashrc` source 它。已有文件且未设 SETUP_HOSTRC_REPLACE=1 时只补 bashrc 行。
#
# 参数：
#   $1: username - 目标用户 (默认: SETUP_USERNAME)。
#
# 使用示例：
#   setup-hostrc
#   SETUP_HOSTRC_REPLACE=1 setup-hostrc
#
# 注意事项：
#   1. 占位符可由 SETUP_PROXY_URL / SETUP_HOSTNAME / SETUP_PROJECT_WORKDIR / SETUP_CONTAINER_NAME / SETUP_USERNAME 替换。
#
setup-hostrc() {
  local user="${1:-${SETUP_USERNAME}}"
  local home out template bashrc tmp
  SETUP_USERNAME="${user}"
  home="$(__setup_target_home "${user}")"
  out="${home}/.hostrc"
  bashrc="${home}/.bashrc"
  template="${SETUP_ROOT}/common/env/template/.hostrc"

  if [[ ! -f "${template}" ]]; then
    __setup_error "template not found: ${template}"
    return 1
  fi

  __setup_warn "Will install ${out} and ensure ${bashrc} sources ~/.hostrc."
  __setup_confirm "configure ${out}?" || return 0

  if [[ -f "${out}" && "${SETUP_HOSTRC_REPLACE:-}" != "1" ]]; then
    __setup_log "${out} exists; skip copy (SETUP_HOSTRC_REPLACE=1 to overwrite)."
  else
    tmp="$(mktemp)"
    cp -- "${template}" "${tmp}"
    sed -i \
      -e "s|custom_proxy_url|${SETUP_PROXY_URL}|g" \
      -e "s|custom_hostname|${SETUP_HOSTNAME:-$(hostname)}|g" \
      -e "s|custom_container_name|${SETUP_CONTAINER_NAME:-${user}_cpp}|g" \
      -e "s|custom_user_name|${user}|g" \
      "${tmp}"
    if [[ -n "${SETUP_PROJECT_WORKDIR:-}" ]]; then
      sed -i "s|custom_workdir_path|${SETUP_PROJECT_WORKDIR}|g" "${tmp}"
    elif [[ -n "${SETUP_PROJECT_NAME:-}" ]]; then
      sed -i "s|custom_workdir_path|${home}/${SETUP_PROJECT_NAME}|g" "${tmp}"
    fi
    __setup_install_user_file "${tmp}" "${out}" "${user}" 644
    rm -f "${tmp}"
    __setup_log "wrote ${out}"
  fi

  if [[ -f "${bashrc}" ]] && grep -qF 'source ~/.hostrc' "${bashrc}"; then
    __setup_log "${bashrc} already sources ~/.hostrc"
    return 0
  fi

  if [[ ! -f "${bashrc}" ]]; then
    tmp="$(mktemp)"
    printf 'source ~/.hostrc\n' >"${tmp}"
    __setup_install_user_file "${tmp}" "${bashrc}" "${user}" 644
    rm -f "${tmp}"
    __setup_log "created ${bashrc}"
    return 0
  fi

  if [[ "${user}" == "${USER}" ]]; then
    printf '\nsource ~/.hostrc\n' >>"${bashrc}"
  else
    __setup_sudo bash -c "printf '\\nsource ~/.hostrc\\n' >>'${bashrc}'"
    __setup_sudo chown "${user}:${user}" "${bashrc}"
  fi
  __setup_log "appended source ~/.hostrc to ${bashrc}"
}

# setup-git
#
# 功能描述：
#   写入 git `url.<to>.insteadOf`。默认不改 user.name。只在 SETUP_GIT_MIRROR=1 或交互确认后执行。
#
# 参数：
#   (无) FROM/TO 由 SETUP_GIT_INSTEADOF_FROM / SETUP_GIT_INSTEADOF_TO 覆盖。
#
# 使用示例：
#   setup-git
#   SETUP_GIT_MIRROR=1 setup-git
#
setup-git() {
  local from="${SETUP_GIT_INSTEADOF_FROM}"
  local to="${SETUP_GIT_INSTEADOF_TO}"

  if [[ "${SETUP_GIT_MIRROR:-}" != "1" ]]; then
    __setup_warn "git insteadOf: ${to}  ->  ${from}"
    __setup_confirm "apply git url.insteadOf?" || return 0
  fi

  git config --global "url.${to}.insteadOf" "${from}" || return 1
  __setup_log "git config --global url.${to}.insteadOf ${from}"
}

# setup-desktop
#
# 功能描述：
#   安装 `~/.xsessionrc` (来自 common/desktop/.xsessionrc), 并在有 DISPLAY 时设置 GNOME 纯色背景。
#
# 参数：
#   $1: username - 目标用户 (默认: SETUP_USERNAME)。
#
# 使用示例：
#   setup-desktop
#   SETUP_SKIP_XSESSIONRC=1 setup-desktop
#
# 注意事项：
#   1. 无 DISPLAY / gsettings 时跳过背景。
#   2. SETUP_SKIP_XSESSIONRC=1 跳过 xsessionrc (woosh 默认)。
#
setup-desktop() {
  local user="${1:-${SETUP_USERNAME}}"
  local home dest src
  SETUP_USERNAME="${user}"
  home="$(__setup_target_home "${user}")"
  dest="${home}/.xsessionrc"
  src="${SETUP_ROOT}/common/desktop/.xsessionrc"

  __setup_warn "Will install ${dest} and optionally set a solid GNOME background."
  __setup_confirm "configure desktop session files?" || return 0

  if [[ "${SETUP_SKIP_XSESSIONRC:-}" == "1" ]]; then
    __setup_log "skip xsessionrc (SETUP_SKIP_XSESSIONRC=1)."
  elif [[ -f "${src}" ]]; then
    if [[ -f "${dest}" && "${SETUP_XSESSIONRC_REPLACE:-}" != "1" ]]; then
      __setup_log "${dest} exists; skip (SETUP_XSESSIONRC_REPLACE=1 to overwrite)."
    else
      __setup_install_user_file "${src}" "${dest}" "${user}" 644
      __setup_log "wrote ${dest}"
    fi
  else
    __setup_log "skip xsessionrc: ${src} not found."
  fi

  if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || ! command -v gsettings >/dev/null 2>&1; then
    __setup_log "skip GNOME background: no display or gsettings."
    return 0
  fi

  gsettings set org.gnome.desktop.background picture-uri '' || true
  gsettings set org.gnome.desktop.background picture-uri-dark '' || true
  gsettings set org.gnome.desktop.background color-shading-type 'solid' || true
  gsettings set org.gnome.desktop.background primary-color '#57549b' || true
  __setup_log "set GNOME solid background"
}

# __setup_host_usage
#
# 功能描述：
#   打印 setup-host 帮助 (含子命令与已发现机型)。
#
# 参数：
#   (无)
#
__setup_host_usage() {
  cat <<EOF
setup-host — configure this machine (user, hosts, ssh, docker, network).

Usage:
  source ${SETUP_ROOT}/setup.bash
  setup-host <command> [args...]
  bash ${SETUP_ROOT}/setup.bash <command> [args...]
  bash ${SETUP_ROOT}/setup.bash <machine> [args...]
  bash ${SETUP_ROOT}/setup.bash --machine <machine> <command> [args...]

Commands:
  user [name]         create/update user and groups
  sudoers [name]      NOPASSWD fragment in /etc/sudoers.d/build_env
  home-bind [name]    bind /home/data/<user> onto /home/<user>
  hosts               upsert LAN names into /etc/hosts
  ssh [name]          ~/.ssh/local.d + optional SETUP_SSH_REPO
  workspaces [proj]   ~/workspaces, /workspaces, optional ~/<proj>
  hostrc [name]       ~/.hostrc and source it from ~/.bashrc
  docker              /etc/docker/daemon.json
  desktop [name]      ~/.xsessionrc and GNOME background
  git                 optional url.insteadOf
  wifi [ssid] [dev]   nmcli wifi connect (password via env/prompt)
  ip-fixed [...]      wrap ip-fixed (network.bash)
  display             Orin kiosk (display 用户 / GDM / gsettings)
  ros2                install ROS 2 debs (install-ros2)
  install <target>    setup-install dispatcher (ros2, ...)
  network             wifi (if SSID set) then ip-fixed
  all                 user..desktop (skips git / network unless env set)
  machine <name>      run the named machine entry (env + pipeline + additions)
  machines            list discovered machines under common/setup/machines/
  help                this text

Machines (auto-scanned from ${SETUP_MACHINES_DIR}):
$(__setup_list_machines)

Environment (subset):
  SETUP_USERNAME SETUP_NONINTERACTIVE=1 SETUP_YES=1
  SETUP_HOSTS_IP SETUP_SSH_REPO SETUP_SSH_REPLACE=1
  SETUP_PROJECT_NAME SETUP_PROJECT_WORKDIR SETUP_SUDOERS_EXTRA
  SETUP_WIFI_SSID SETUP_WIFI_PASSWORD SETUP_WIFI_DEVICE
  SETUP_IP_CIDR SETUP_GATEWAY SETUP_DNS
  SETUP_DOCKER_FORCE=1 SETUP_GIT_MIRROR=1 SETUP_SKIP_XSESSIONRC=1

See ${SETUP_ROOT}/docs/usage/setup.md and ${SETUP_ROOT}/docs/usage/machines.md
EOF
}

# __setup_host_invoke_machine
#
# 功能描述：
#   调用已注册的 `setup-<name>` 机型入口。
#
# 参数：
#   $1: name - 机型目录名。
#   $@: 透传给机型入口的其余参数。
#
__setup_host_invoke_machine() {
  local name="$1"
  shift || true
  local fn="setup-${name}"
  if ! declare -F "${fn}" >/dev/null 2>&1; then
    __setup_error "machine entry not registered: ${fn}"
    return 1
  fi
  "${fn}" "$@"
}

# setup-host
#
# 功能描述：
#   宿主机部署调度器: 解析 `--machine`, 分发子命令, 或进入自动发现的机型入口。
#
# 参数：
#   --machine|-m NAME - 先加载该机型 env, 再跑后面的通用子命令。
#   $1: command - 子命令 (user / hosts / all / orin / ...) 或机型名。
#   $@: 透传给对应 setup-* 函数。
#
# 使用示例：
#   setup-host help
#   setup-host all
#   setup-host --machine orin all
#   setup-host orin
#
# 注意事项：
#   1. 由仓库根 setup.bash 在直接执行时调用。
#   2. 机型列表见 SETUP_MACHINES_DIR。
#
setup-host() {
  local cmd
  while [[ $# -gt 0 ]]; do
    case "$1" in
    --machine | -m)
      if [[ $# -lt 2 ]]; then
        __setup_error "missing value for $1"
        return 1
      fi
      __setup_machine_begin "$2" || return 1
      shift 2
      ;;
    --machine=*)
      __setup_machine_begin "${1#*=}" || return 1
      shift
      ;;
    --)
      shift
      break
      ;;
    *)
      break
      ;;
    esac
  done

  cmd="${1:-help}"
  shift || true

  case "${cmd}" in
  -h | --help | help)
    __setup_host_usage
    ;;
  user)
    setup-user "$@"
    ;;
  sudoers)
    setup-sudoers "$@"
    ;;
  home-bind | home_bind)
    setup-home-bind "$@"
    ;;
  hosts)
    setup-hosts "$@"
    ;;
  ssh)
    setup-ssh "$@"
    ;;
  workspaces | workspace)
    setup-workspaces "$@"
    ;;
  hostrc | bashrc)
    setup-hostrc "$@"
    ;;
  docker | docker-daemon | daemon)
    setup-docker-daemon "$@"
    ;;
  desktop)
    setup-desktop "$@"
    ;;
  display)
    setup-display "$@"
    ;;
  ros2 | install-ros2)
    install-ros2 "$@"
    ;;
  install)
    setup-install "$@"
    ;;
  git)
    setup-git "$@"
    ;;
  wifi)
    setup-wifi "$@"
    ;;
  ip-fixed | ip_fixed | ip)
    setup-ip-fixed "$@"
    ;;
  network)
    setup-network "$@"
    ;;
  all)
    setup-user "$@" || return 1
    setup-sudoers "$@" || return 1
    setup-home-bind "$@" || return 1
    setup-hosts || return 1
    setup-ssh "$@" || return 1
    setup-workspaces || return 1
    setup-hostrc "$@" || return 1
    setup-docker-daemon || return 1
    setup-desktop "$@" || return 1
    if [[ "${SETUP_GIT_MIRROR:-}" == "1" ]]; then
      setup-git || return 1
    fi
    if [[ -n "${SETUP_WIFI_SSID:-}" ]]; then
      setup-network || return 1
    fi
    __setup_log "setup-host all: done."
    ;;
  machine)
    if [[ $# -lt 1 ]]; then
      __setup_error "usage: setup-host machine <name>"
      return 1
    fi
    __setup_host_invoke_machine "$@"
    ;;
  machines | list | list-machines)
    __setup_list_machines
    ;;
  *)
    if __setup_machine_exists "${cmd}"; then
      __setup_host_invoke_machine "${cmd}" "$@"
      return
    fi
    __setup_error "unknown command: ${cmd}"
    __setup_host_usage >&2
    return 1
    ;;
  esac
}
