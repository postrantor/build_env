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

##
# @brief 更新 Git 配置
#
# 该函数用于读取当前用户的信息（Git 用户名和邮箱），并将其替换到 `.env/config` 文件中的 `[user]` 部分。
# 替换后的内容将追加到 `~/.gitconfig` 文件中。
#
# @details
# 函数会检查当前用户的 Git 配置（用户名和邮箱），如果未设置，则使用系统用户名和主机名作为默认值。
# 替换操作会修改 `.env/config` 文件中的 `[user]` 部分，并将结果追加到 `~/.gitconfig` 文件中。
#
# @return 返回值为 0 表示成功，非 0 表示失败。
#
# @note
# - 确保 `.env/config` 文件存在且格式正确。
# - 确保当前用户有权限读取 `.env/config` 文件并写入 `~/.gitconfig` 文件。
#
# @example
# 调用示例：
# ```
# update-gitconfig
# ```
##
function update-gitconfig() {
  # 获取当前用户的用户名和邮箱
  CURRENT_USER_NAME=$(git config --global user.name)
  CURRENT_USER_EMAIL=$(git config --global user.email)

  # 如果用户没有设置全局的 Git 用户名或邮箱，则使用系统用户名
  if [ -z "$CURRENT_USER_NAME" ]; then
    CURRENT_USER_NAME=$(whoami)
  fi

  if [ -z "$CURRENT_USER_EMAIL" ]; then
    CURRENT_USER_EMAIL="$CURRENT_USER_NAME@$(hostname)"
  fi

  # 读取 .env/config 文件内容并替换 user_name 信息
  CONFIG_CONTENT=$(cat ${QUAD_WORKDIR}/.env/gitconfig)
  CONFIG_CONTENT=$(echo "$CONFIG_CONTENT" | sed "s/user_name/$CURRENT_USER_NAME/g")
  CONFIG_CONTENT=$(echo "$CONFIG_CONTENT" | sed "s/user_name@gmail.com/$CURRENT_USER_EMAIL/g")

  # 将替换后的内容追加到 ~/.gitconfig 中
  echo "$CONFIG_CONTENT" >~/.gitconfig

  echo "git configuration has been updated and appended to ~/.gitconfig."
}

function update-bashrc-config() {
  # 读取 .env/bashrc 文件内容并替换 replace_to_script_path 信息
  CONFIG_CONTENT=$(cat ${QUAD_WORKDIR}/.env/bashrc)
  CONFIG_CONTENT=$(echo "$CONFIG_CONTENT" | sed "s|replace_to_script_path|$QUAD_WORKDIR|g")

  # 将替换后的内容追加到 ~/.bashrc 中
  echo "$CONFIG_CONTENT" >${HOME}/.bashrc
  source ${HOME}/.bashrc

  echo "bashrc configuration has been updated and appended to ~/.bashrc."
}

function update-ssh-config() {
  # 读取 .env/ssh 文件内容并替换 replace_to_script_path 信息
  CONFIG_CONTENT=$(cat ${QUAD_WORKDIR}/.env/sshconfig)
  CONFIG_CONTENT=$(echo "$CONFIG_CONTENT" | sed "s|p7xxtm1_ssh_ip|$P7XXTM1_SSH_IP|g")

  # 将替换后的内容追加到 ~/.ssh 中
  echo "$CONFIG_CONTENT" > ${HOME}/.ssh/config

  echo "ssh configuration has been updated and appended to ~/.ssh/config."
}
