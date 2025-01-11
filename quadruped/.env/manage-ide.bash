# ---
# date: 2025-01-11
# author: postrantor
# description: manage vscode ide
# ---

function import-vscode-extionsions() {
  # 遍历 ${QUAD_WORKDIR}/.vscode/extensions/ 下的每个文件夹
  find "${QUAD_WORKDIR}/.vscode/extensions/" -mindepth 1 -maxdepth 1 -type d | while read -r dir; do
    # 复制文件夹
    cp -r --archive "$dir" "${HOME}/.vscode-server/extensions/"
    # 输出一个 . 符号
    echo -n "."
  done
  # 输出换行符，避免后续输出与 . 符号在同一行
  echo

  cp "${QUAD_WORKDIR}/.vscode/extensions/extensions.json" --archive "${HOME}/.vscode-server/extensions/extensions.json"

  rm -rf "${QUAD_WORKDIR}/.vscode/extensions
}
