# ---
# date: 2026-06-17
# author: zhiqi.jia@ai2robotics.com
# description: Orin 表情屏 kiosk (display 用户 / GDM / GNOME)。由 orin/setup.bash source。
# ---


# _setup_display_log
#
# 功能描述：
#   向 stderr 打印 `[setup-display]` 前缀日志。
#
# 参数：
#   $@: message - 日志内容。
#
_setup_display_log() {
  echo "[setup-display] $*" >&2
}

# _setup_display_die
#
# 功能描述：
#   向 stderr 打印错误并返回 1。
#
# 参数：
#   $@: message - 错误内容。
#
_setup_display_die() {
  echo "[setup-display] error: $*" >&2
  return 1
}

# _setup_display_setup_root
#
# 功能描述：
#   解析仓库根: 优先 SETUP_ROOT, 否则从本文件向上查找含 setup.bash 的目录。
#
# 参数：
#   (无)
#
_setup_display_setup_root() {
  if [[ -n "${SETUP_ROOT:-}" ]]; then
    printf '%s' "${SETUP_ROOT}"
    return 0
  fi
  local d
  d="$(cd "$(dirname "${BASH_SOURCE[0]:-.}")" && pwd)"
  while [[ "${d}" != / ]]; do
    if [[ -f "${d}/setup.bash" && -d "${d}/common/setup" ]]; then
      printf '%s' "${d}"
      return 0
    fi
    d="$(cd "${d}/.." && pwd)"
  done
  return 1
}

# _setup_display_template_path
#
# 功能描述：
#   返回 `common/env/template/<name>` 的绝对路径。
#
# 参数：
#   $1: name - 模板文件名, 如 `.xsessionrc`、`logo.png`。
#
_setup_display_template_path() {
  local name="$1"
  printf '%s/common/env/template/%s' "$(_setup_display_setup_root)" "${name}"
}

# setup-display
#
# 功能描述：
#   宿主机一键配置 display 表情屏: 创建 display 用户、GDM 自动登录、GNOME gsettings、
#   `.xsessionrc` kiosk 块与会话重载。GDM 与 `.xsessionrc` 块幂等, 重复执行不会写入重复配置。
#
# 参数：
#   (无) 配置完成后会 restart gdm3 以重载 display 图形会话。
#
# 使用示例：
#   source /workspaces/build_env/setup.bash
#   sudo setup-display
#
# 注意事项：
#   1. 需 root 权限 (sudo setup-display)。
#   2. gsettings 需在 display 用户登录后生效; 若 session bus 未就绪, 重登后再次执行。
#   3. 环境变量: DISPLAY_USER / DISPLAY_NUM / GDM_CONF / DISPLAY_XSESSIONRC_TEMPLATE / DISPLAY_LOGO_TEMPLATE。
#
setup-display() {
  case "${1-}" in
  --help | -h)
    cat >&2 <<'EOF'
setup-display — host kiosk setup for the display module (user, GDM, gsettings, .xsessionrc).

Usage: sudo setup-display

Env:
  DISPLAY_USER, DISPLAY_NUM, GDM_CONF
  DISPLAY_XSESSIONRC_TEMPLATE, DISPLAY_LOGO_TEMPLATE

Examples:
  source /workspaces/build_env/setup.bash
  sudo setup-display

See: src/module/display/docs/deploy.md
EOF
    return 0
    ;;
  -*)
    _setup_display_die "unknown option: $1 (try: setup-display --help)"
    return $?
    ;;
  esac

  if [[ $# -gt 0 ]]; then
    _setup_display_die "unexpected argument: $1"
    return $?
  fi

  if [[ "${EUID}" -ne 0 ]]; then
    _setup_display_die "run as root (sudo setup-display)"
    return $?
  fi

  local display_user="${DISPLAY_USER:-display}"
  local display_num="${DISPLAY_NUM:-:0}"
  local gdm_conf="${GDM_CONF:-/etc/gdm3/custom.conf}"
  local xsessionrc="/home/${display_user}/.xsessionrc"
  local xsessionrc_template="${DISPLAY_XSESSIONRC_TEMPLATE:-$(_setup_display_template_path .xsessionrc)}"
  local logo_template="${DISPLAY_LOGO_TEMPLATE:-$(_setup_display_template_path logo.png)}"
  local xsessionrc_marker="# Display module kiosk setup"
  local wallpaper_uri=""

  _setup_display_ensure_user "${display_user}" || return $?

  if _setup_display_gdm_already_ok "${gdm_conf}" "${display_user}"; then
    _setup_display_log "GDM [daemon] already configured for ${display_user}, skip"
  else
    _setup_display_configure_gdm "${gdm_conf}" "${display_user}" || return $?
  fi

  local _xsessionrc_rc=0
  _setup_display_setup_xsessionrc "${display_user}" "${xsessionrc}" "${xsessionrc_template}" "${xsessionrc_marker}" \
    || _xsessionrc_rc=$?
  if [[ "${_xsessionrc_rc}" -eq 1 ]]; then
    return 1
  fi

  _setup_display_remove_desktop_shortcuts "${display_user}"

  wallpaper_uri="$(_setup_display_install_wallpaper "${display_user}" "${logo_template}")" || return $?

  _setup_display_reload_session || return $?

  _setup_display_log "waiting for ${display_user} desktop session"
  _setup_display_wait_for_session_bus "${display_user}" || true

  _setup_display_apply_gsettings "${display_user}" "${display_num}" "${wallpaper_uri}"

  cat >&2 <<EOF

Verify on host:
  loginctl list-sessions
  DISPLAY=${display_num} xset q
  DISPLAY=${display_num} xhost
EOF
  _setup_display_log "done"
}

# _setup_display_ensure_user
#
# 功能描述：
#   创建 display 用户并加入 video/render/audio/input 组。
#
# 参数：
#   $1: user - 用户名。
#
_setup_display_ensure_user() {
  local display_user="$1"
  if ! id "${display_user}" &>/dev/null; then
    _setup_display_log "creating user ${display_user}"
    adduser --disabled-password --gecos "Display kiosk user" "${display_user}" || return 1
  else
    _setup_display_log "user ${display_user} already exists"
  fi
  usermod -aG video,render,audio,input "${display_user}"
}

# _setup_display_gdm_key_ok
#
# 功能描述：
#   检查 GDM 配置项是否已为期望值。
#
# 参数：
#   $1: gdm_conf - GDM 配置文件。
#   $2: key - 键名。
#   $3: value - 期望值。
#
_setup_display_gdm_key_ok() {
  local gdm_conf="$1" key="$2" value="$3"
  grep -qE "^[[:space:]]*${key}=${value}[[:space:]]*$" "${gdm_conf}"
}

# _setup_display_gdm_already_ok
#
# 功能描述：
#   判断 GDM 自动登录三项是否均已配置。
#
# 参数：
#   $1: gdm_conf - GDM 配置文件。
#   $2: user - display 用户名。
#
_setup_display_gdm_already_ok() {
  local gdm_conf="$1" display_user="$2"
  [[ -f "${gdm_conf}" ]] || return 1
  _setup_display_gdm_key_ok "${gdm_conf}" AutomaticLoginEnable true \
    && _setup_display_gdm_key_ok "${gdm_conf}" AutomaticLogin "${display_user}" \
    && _setup_display_gdm_key_ok "${gdm_conf}" WaylandEnable false
}

# _setup_display_configure_gdm
#
# 功能描述：
#   写入 GDM 自动登录与 WaylandEnable=false (幂等)。
#
# 参数：
#   $1: gdm_conf - GDM 配置文件。
#   $2: user - display 用户名。
#
_setup_display_configure_gdm() {
  local gdm_conf="$1" display_user="$2" key
  if [[ ! -f "${gdm_conf}" ]]; then
    _setup_display_die "GDM config not found: ${gdm_conf} (install gdm3 or set GDM_CONF)"
    return $?
  fi

  _setup_display_log "updating GDM autologin in ${gdm_conf}"
  for key in AutomaticLoginEnable AutomaticLogin WaylandEnable; do
    sed -i "/^[[:space:]]*#\\?[[:space:]]*${key}=/d" "${gdm_conf}"
  done

  if ! grep -q '^\[daemon\]' "${gdm_conf}"; then
    printf '\n[daemon]\n' >>"${gdm_conf}"
  fi

  sed -i "/^\[daemon\]/a WaylandEnable=false" "${gdm_conf}"
  sed -i "/^\[daemon\]/a AutomaticLogin=${display_user}" "${gdm_conf}"
  sed -i "/^\[daemon\]/a AutomaticLoginEnable=true" "${gdm_conf}"
}

# _setup_display_setup_xsessionrc
#
# 功能描述：
#   从模板初始化 `.xsessionrc` 并追加 kiosk 块 (幂等)。
#
# 参数：
#   $1: user - display 用户名。
#   $2: xsessionrc - 目标 `.xsessionrc`。
#   $3: template - 模板路径。
#   $4: marker - kiosk 块标记。
#
# 注意事项：
#   1. 返回 0 未变更, 2 已追加 kiosk 块, 1 失败。
#
_setup_display_setup_xsessionrc() {
  local display_user="$1" xsessionrc="$2" xsessionrc_template="$3" xsessionrc_marker="$4"

  if [[ ! -f "${xsessionrc}" ]]; then
    if [[ -f "${xsessionrc_template}" ]]; then
      _setup_display_log "copying ${xsessionrc_template} -> ${xsessionrc}"
      cp "${xsessionrc_template}" "${xsessionrc}" || return 1
    else
      _setup_display_log "template missing (${xsessionrc_template}), creating empty ${xsessionrc}"
      touch "${xsessionrc}" || return 1
    fi
    chown "${display_user}:${display_user}" "${xsessionrc}"
    chmod 644 "${xsessionrc}"
  fi

  if grep -qF "${xsessionrc_marker}" "${xsessionrc}"; then
    _setup_display_log ".xsessionrc kiosk block already present, skip append"
    return 0
  fi

  _setup_display_log "appending kiosk block to ${xsessionrc}"
  tee -a "${xsessionrc}" >/dev/null <<'EOF'

# Display module kiosk setup
if [ -x "/usr/bin/xset" ]; then
  xset s off -dpms s noblank
fi

if [ -x "/usr/bin/xhost" ]; then
  xhost +
  #xhost +SI:localuser:root
  #xhost +SI:localuser:robot
fi
# End
EOF
  chown "${display_user}:${display_user}" "${xsessionrc}"
  return 2
}

# _setup_display_remove_desktop_shortcuts
#
# 功能描述：
#   删除 Jetson 桌面 `.desktop` 快捷方式。
#
# 参数：
#   $1: user - display 用户名。
#
_setup_display_remove_desktop_shortcuts() {
  local display_user="$1"
  if compgen -G "/home/${display_user}/Desktop/*.desktop" >/dev/null; then
    _setup_display_log "removing Jetson desktop shortcuts"
    rm -f "/home/${display_user}/Desktop/"*.desktop
  fi
}

# _setup_display_install_wallpaper
#
# 功能描述：
#   安装 logo 到用户目录并返回 `file://` URI。
#
# 参数：
#   $1: user - display 用户名。
#   $2: logo_src - logo 源文件路径。
#
_setup_display_install_wallpaper() {
  local display_user="$1" logo_src="$2"
  local dest_dir="/home/${display_user}/.local/share/backgrounds"
  local dest_path="${dest_dir}/logo.png"

  if [[ ! -f "${logo_src}" ]]; then
    _setup_display_die "wallpaper template not found: ${logo_src}"
    return $?
  fi

  _setup_display_log "installing wallpaper ${logo_src} -> ${dest_path}"
  mkdir -p "${dest_dir}"
  cp -f "${logo_src}" "${dest_path}"
  chown -R "${display_user}:${display_user}" "/home/${display_user}/.local"
  chmod 644 "${dest_path}"
  printf 'file://%s' "${dest_path}"
}

# _setup_display_wait_for_session_bus
#
# 功能描述：
#   等待 display 用户 session D-Bus 就绪。
#
# 参数：
#   $1: user - display 用户名。
#
# 注意事项：
#   1. 返回 0 就绪, 1 超时 (最多约 120s)。
#
_setup_display_wait_for_session_bus() {
  local display_user="$1" uid bus i
  uid="$(id -u "${display_user}")"
  bus="/run/user/${uid}/bus"
  for ((i = 1; i <= 60; i++)); do
    [[ -S "${bus}" ]] && return 0
    sleep 2
  done
  return 1
}

# _setup_display_apply_gsettings
#
# 功能描述：
#   经 session bus 写入 GNOME 桌面与壁纸 gsettings。
#
# 参数：
#   $1: user - display 用户名。
#   $2: display_num - X11 DISPLAY。
#   $3: wallpaper_uri - 壁纸 `file://` URI。
#
_setup_display_apply_gsettings() {
  local display_user="$1" display_num="$2" wallpaper_uri="$3" uid bus
  uid="$(id -u "${display_user}")"
  bus="/run/user/${uid}/bus"

  if [[ ! -S "${bus}" ]]; then
    _setup_display_log "display session bus not ready (${bus}), skip live gsettings"
    _setup_display_log "after ${display_user} logs in, re-run: sudo setup-display"
    return 0
  fi

  _setup_display_log "applying GNOME gsettings via session bus"
  sudo -u "${display_user}" \
    XDG_RUNTIME_DIR="/run/user/${uid}" \
    DBUS_SESSION_BUS_ADDRESS="unix:path=${bus}" \
    DISPLAY="${display_num}" \
    DISPLAY_WALLPAPER_URI="${wallpaper_uri}" \
    bash <<'EOF'
set -e

set_gsetting() {
  local schema="$1" key="$2" value="$3"
  if gsettings writable "${schema}" "${key}" 2>/dev/null | grep -qx true; then
    gsettings set "${schema}" "${key}" "${value}"
  else
    echo "skip unsupported gsetting: ${schema} ${key}"
  fi
}

set_gsetting org.gnome.desktop.notifications show-banners false
set_gsetting org.gnome.desktop.notifications show-in-lock-screen false

set_gsetting org.gnome.shell.extensions.ding show-home false
set_gsetting org.gnome.shell.extensions.ding show-trash false
set_gsetting org.gnome.shell.extensions.ding show-volumes false
set_gsetting org.gnome.shell.extensions.ding show-network-volumes false
set_gsetting org.gnome.shell.extensions.dash-to-dock dock-fixed false
set_gsetting org.gnome.shell.extensions.dash-to-dock autohide true
set_gsetting org.gnome.shell.extensions.dash-to-dock intellihide true

set_gsetting org.gnome.desktop.session idle-delay 0
set_gsetting org.gnome.desktop.screensaver lock-enabled false
set_gsetting org.gnome.desktop.screensaver ubuntu-lock-on-suspend false
set_gsetting org.gnome.desktop.screensaver idle-activation-enabled false
set_gsetting org.gnome.settings-daemon.plugins.power idle-dim false
set_gsetting org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type nothing
set_gsetting org.gnome.settings-daemon.plugins.power sleep-inactive-ac-timeout 0
set_gsetting org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type nothing
set_gsetting org.gnome.settings-daemon.plugins.power sleep-inactive-battery-timeout 0

if [[ -n "${DISPLAY_WALLPAPER_URI}" ]]; then
  set_gsetting org.gnome.desktop.background picture-uri "${DISPLAY_WALLPAPER_URI}"
  set_gsetting org.gnome.desktop.background picture-uri-dark "${DISPLAY_WALLPAPER_URI}"
  set_gsetting org.gnome.desktop.background picture-options scaled
fi
EOF
}

# _setup_display_reload_session
#
# 功能描述：
#   restart gdm3 使 `.xsessionrc` 等配置生效。
#
# 参数：
#   (无)
#
_setup_display_reload_session() {
  if systemctl is-active --quiet gdm3; then
    _setup_display_log "restarting gdm3"
    systemctl restart gdm3
    return 0
  fi

  _setup_display_die "gdm3 is not active; restart display manager manually"
  return $?
}
