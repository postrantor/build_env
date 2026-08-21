# ---
# date: 2026-08-21
# description: Orin 机型入口。只定义函数, 由 machines.bash 在加载时 source。
# ---

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/display.bash"

# setup-orin
#
# 功能描述：
#   Orin 机型入口: 通用 all (含 /home/data bind、docker daemon、Jetson xsessionrc) 后配置表情屏 kiosk, 再扫 additions/。
#
# 参数：
#   $@: 透传给 setup-host all (通常为 username)。
#
# 使用示例：
#   setup-orin
#   bash /workspaces/build_env/setup.bash orin
#
setup-orin() {
  __setup_machine_begin orin || return 1
  setup-host all "$@" || return 1
  setup-display || return 1
  __setup_machine_additions
}
