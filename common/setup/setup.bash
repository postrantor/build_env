# ---
# date: 2026-08-21
# description: 加载宿主机部署函数。由仓库根 setup.bash source。
# ---

_setup_lib="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export SETUP_DIR="${_setup_lib}"
export SETUP_ROOT="${SETUP_ROOT:-$(cd "${_setup_lib}/../.." && pwd)}"

# shellcheck disable=SC1091
source "${_setup_lib}/common.bash"
# shellcheck disable=SC1091
source "${_setup_lib}/docker.bash"
# shellcheck disable=SC1091
source "${_setup_lib}/network.bash"
# shellcheck disable=SC1091
source "${_setup_lib}/install.bash"
# shellcheck disable=SC1091
source "${_setup_lib}/deploy.bash"
# shellcheck disable=SC1091
source "${_setup_lib}/txt.bash"
# shellcheck disable=SC1091
source "${_setup_lib}/media.bash"
# shellcheck disable=SC1091
source "${_setup_lib}/machines.bash"
# shellcheck disable=SC1091
source "${_setup_lib}/host.bash"

unset _setup_lib
