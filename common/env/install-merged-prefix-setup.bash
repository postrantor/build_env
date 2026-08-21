#!/bin/bash
# Install colcon merged-install prefix setup scripts for /opt/ros/ai2rob vendor debs.
#
# Vendor debs rsync isolated colcon package trees into a single prefix. That
# layout matches colcon "merged install", so local_setup must pass --merged-install
# to _local_setup_util_sh.py. setup.* chains the ROS underlay (default humble)
# and then sources this prefix.
#
# Usage: install-merged-prefix-setup.bash <dest_prefix> <colcon_install_dir> [underlay_prefix]

set -euo pipefail

DEST="${1:?dest prefix required}"
COLCON_INSTALL="${2:?colcon install dir required}"
UNDERLAY="${3:-/opt/ros/humble}"

if [ ! -f "${COLCON_INSTALL}/_local_setup_util_sh.py" ]; then
  if [ -f "${UNDERLAY}/_local_setup_util_sh.py" ]; then
    echo "install-merged-prefix-setup: using underlay templates from ${UNDERLAY}" >&2
    COLCON_INSTALL="${UNDERLAY}"
  else
    echo "install-merged-prefix-setup: missing ${2}/_local_setup_util_sh.py (and ${UNDERLAY})" >&2
    exit 1
  fi
fi

install -d "${DEST}"
install -m 644 "${COLCON_INSTALL}/_local_setup_util_sh.py" "${DEST}/"
if [ -f "${COLCON_INSTALL}/_local_setup_util_ps1.py" ]; then
  install -m 644 "${COLCON_INSTALL}/_local_setup_util_ps1.py" "${DEST}/"
fi

for shell in bash sh zsh; do
  src="${COLCON_INSTALL}/local_setup.${shell}"
  if [ ! -f "${src}" ]; then
    echo "install-merged-prefix-setup: missing ${src}" >&2
    exit 1
  fi
  # colcon template ends with `sh bash)"` / `sh zsh)"` / `sh)"`; do not anchor on a
  # trailing `"` or the substitution silently no-ops and merged hooks never run.
  sed 's/_local_setup_util_sh.py" sh\([^)]*\)/_local_setup_util_sh.py" sh\1 --merged-install/' \
    "${src}" > "${DEST}/local_setup.${shell}"
  chmod 644 "${DEST}/local_setup.${shell}"
done

UNDERLAY="${UNDERLAY}" DEST="${DEST}" python3 - <<'PY'
import os
from pathlib import Path

underlay = os.environ["UNDERLAY"]
dest = Path(os.environ["DEST"])

templates = {
    "setup.bash": """# AI2 ROBOTICS vendor prefix setup (merged colcon install)

_colcon_prefix_chain_bash_source_script() {
  if [ -f "$1" ]; then
    if [ -n "$COLCON_TRACE" ]; then
      echo "# . \\"$1\\""
    fi
    . "$1"
  else
    echo "not found: \\"$1\\"" 1>&2
  fi
}

COLCON_CURRENT_PREFIX="__UNDERLAY__"
_colcon_prefix_chain_bash_source_script "$COLCON_CURRENT_PREFIX/local_setup.bash"

COLCON_CURRENT_PREFIX="$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)"
_colcon_prefix_chain_bash_source_script "$COLCON_CURRENT_PREFIX/local_setup.bash"

unset COLCON_CURRENT_PREFIX
unset _colcon_prefix_chain_bash_source_script
""",
    "setup.sh": """# AI2 ROBOTICS vendor prefix setup (merged colcon install)

_colcon_prefix_chain_sh_source_script() {
  if [ -f "$1" ]; then
    if [ -n "$COLCON_TRACE" ]; then
      echo "# . \\"$1\\""
    fi
    . "$1"
  else
    echo "not found: \\"$1\\"" 1>&2
  fi
}

COLCON_CURRENT_PREFIX="__UNDERLAY__"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

COLCON_CURRENT_PREFIX="$(cd "$(dirname "$0")" > /dev/null && pwd)"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

unset COLCON_CURRENT_PREFIX
unset _colcon_prefix_chain_sh_source_script
""",
    "setup.zsh": """# AI2 ROBOTICS vendor prefix setup (merged colcon install)

_colcon_prefix_chain_zsh_source_script() {
  if [ -f "$1" ]; then
    if [ -n "$COLCON_TRACE" ]; then
      echo "# . \\"$1\\""
    fi
    . "$1"
  else
    echo "not found: \\"$1\\"" 1>&2
  fi
}

COLCON_CURRENT_PREFIX="__UNDERLAY__"
_colcon_prefix_chain_zsh_source_script "$COLCON_CURRENT_PREFIX/local_setup.zsh"

COLCON_CURRENT_PREFIX="$(builtin cd -q "`dirname "${(%):-%x}"`" > /dev/null && pwd)"
_colcon_prefix_chain_zsh_source_script "$COLCON_CURRENT_PREFIX/local_setup.zsh"

unset COLCON_CURRENT_PREFIX
unset _colcon_prefix_chain_zsh_source_script
""",
}

for name, content in templates.items():
    (dest / name).write_text(content.replace("__UNDERLAY__", underlay))
    (dest / name).chmod(0o644)
PY

echo "install-merged-prefix-setup: installed prefix setup scripts in ${DEST}"
