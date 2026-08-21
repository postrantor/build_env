#!/usr/bin/env bash
#
# ---
# date: 2026-08-21
# description: 宿主机部署入口。source 后可调用 setup-* / setup-<机型>; 直接执行则进入 setup-host。
# ---

_setup_entry="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export SETUP_ROOT="${_setup_entry}"
unset _setup_entry

# shellcheck disable=SC1091
source "${SETUP_ROOT}/common/setup/setup.bash"

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  setup-host "$@"
fi
