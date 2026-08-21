# ---
# date: 2026-08-21
# description: woosh 机型入口。只定义函数, 由 machines.bash 在加载时 source。
# ---

# setup-woosh
#
# 功能描述：
#   woosh 笔记本入口: 用户 / sudoers / hosts / ssh / workspaces / hostrc / desktop。
#   通常没有 /home/data 与 Jetson 桌面会话, 跳过 home-bind / docker / xsessionrc。
#
# 参数：
#   $@: 透传给 setup-user 等 (通常为 username)。
#
# 使用示例：
#   setup-woosh
#   bash /workspaces/build_env/setup.bash woosh
#
setup-woosh() {
  __setup_machine_begin woosh || return 1
  setup-user "$@" || return 1
  setup-sudoers "$@" || return 1
  setup-hosts || return 1
  setup-ssh "$@" || return 1
  setup-workspaces || return 1
  setup-hostrc "$@" || return 1
  setup-desktop "$@" || return 1
  if [[ "${SETUP_GIT_MIRROR:-}" == "1" ]]; then
    setup-git || return 1
  fi
  if [[ -n "${SETUP_WIFI_SSID:-}" ]]; then
    setup-network || return 1
  fi
  __setup_machine_additions
}
