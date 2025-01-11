# ---
# date: 2025-01-11
# author: postrantor
# description: manage vscode ide
# ---

function import-vscode-extionsions() {
  cp -r --archive ${QUAD_WORKDIR}/.vscode/extensions/ ${HOME}/.vscode-server/extensions/*
}