# ---
# date: 2025-01-11
# author: postrantor
# description: manage vscode ide
# ---

##
# @brief 导入 VS Code 扩展函数
#
# 该函数用于将指定的 VS Code 扩展包（`extensions.tar.gz`）解压到目标目录（`~/.vscode-server`）。
# 适用于在远程开发环境中快速导入 VS Code 扩展。
#
# @details
# 函数会检查扩展包文件是否存在，如果不存在则返回错误。
# 如果目标目录不存在，函数会自动创建该目录。
# 解压完成后，函数会检查解压操作是否成功，并输出相应的提示信息。
#
# @param[in] extensions_tar_gz 扩展包文件路径，默认值为 `${QUAD_WORKDIR}/.vscode/extensions.tar.gz`。
# @param[in] target_dir 目标目录路径，默认值为 `${HOME}/.vscode-server`。
#
# @return 返回值为 0 表示成功，非 0 表示失败。
#
# @note
# - 确保 `${QUAD_WORKDIR}` 和 `${HOME}` 变量已正确定义。
# - 确保当前用户有权限读取扩展包文件并写入目标目录。
# - 扩展包文件必须是有效的 `.tar.gz` 格式。
#
# @see
# - `tar` 命令文档：https://www.gnu.org/software/tar/manual/tar.html
# - VS Code 远程开发文档：https://code.visualstudio.com/docs/remote/remote-overview
#
# @example
# 调用示例：
# ```
# import-vscode-extensions
# ```
##
function import-vscode-extensions() {
  local extensions_tar_gz="${QUAD_WORKDIR}/.vscode/extensions.tar.gz"
  local target_dir="${HOME}/.vscode-server"

  # 检查 extensions.tar.gz 文件是否存在
  if [[ ! -f "$extensions_tar_gz" ]]; then
    echo "error: file $extensions_tar_gz dont exist。"
    return 1
  fi

  # 检查目标目录是否存在，如果不存在则创建
  if [[ ! -d "$target_dir" ]]; then
    echo "target directory $target_dir dont exist, creating..."
    mkdir -p "$target_dir"
  fi

  # 解压文件到目标目录
  echo "tar $extensions_tar_gz to $target_dir ..."
  tar -xzf "$extensions_tar_gz" -C "$target_dir"

  # 检查解压是否成功
  if [[ $? -eq 0 ]]; then
    echo "unpack success."
  else
    echo "error: unpack failed."
    return 1
  fi
}
