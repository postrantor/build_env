#!/bin/bash
#
# ---
# date: 2025-01-11
# update: 2026-04-19
# author: postrantor
# description: manage vscode ide
# ---


# 交互确认：默认 Y；输入 n / no 则跳过返回 1，由调用方 `|| return 0` 结束当前函数）。
# OTA/无 TTY 时跳过询问 (如 entrypoint 10.updated.user.sh)。
function __manage_ide_confirm() {
  if [[ "${MANAGE_IDE_NONINTERACTIVE:-}" == "1" || ! -t 0 ]]; then
    return 0
  fi
  echo -e "\033[33m$1\033[0m" >&2
  local ans
  read -r -p "$2 [Y/n] " ans
  case "${ans,,}" in
  n | no)
    echo "skipped." >&2
    return 1
    ;;
  *)
    return 0
    ;;
  esac
}

##
# @brief 更新 Cursor：解压 `extensions.tar.gz` 到 `~/.cursor-server`（与 download-dependencies 同路径、同跳过规则）
#
# 本地归档与 `download-dependencies` 一致：`${AI2_WORKDIR}/dependencies/binary/<aarch|x86>/extensions.tar.gz`。
# 若该路径已存在（文件或链接等，与 deb 相同用 `test -e`），则跳过下载；否则从 GitLab raw 拉取后再解压。
#
# @details
# 首先以黄色提示配合 `[Y/n]` 询问是否继续（默认 Y；输入 n/no 则跳过本函数并返回 0）。
# 按 `HOST_PLATFORM`（或 `uname -m`）选择 `aarch` / `x86` 子目录。
# 若目标目录 `${HOME}/.cursor-server` 不存在则自动创建。
#
# @param[in] （隐式）本地归档：`${AI2_WORKDIR}/dependencies/binary/<aarch|x86>/extensions.tar.gz`
# @param[in] （隐式）解压目标：`${HOME}/.cursor-server`
#
# @return 返回值为 0 表示成功，非 0 表示失败。
#
# @note
# - 需已设置 `${AI2_WORKDIR}`；需对 `dependencies/binary` 与 `${HOME}/.cursor-server` 可写。
# - 远程路径与 `manage-project.bash` 中 `download-dependencies` 的 `base_url` 一致：`.../<aarch|x86>/extensions.tar.gz`。
#
# @see
# - `tar` 命令文档：https://www.gnu.org/software/tar/manual/tar.html
#
# @example
# ```
# config-update-cursor
# ```
##
function config-update-cursor() {
  if [[ -z "${PROJECT_WORKDIR:-}" ]]; then
    echo "error: PROJECT_WORKDIR is not set." >&2
    return 1
  fi

  local plat="${HOST_PLATFORM:-}"
  [[ -z "$plat" ]] && case "$(uname -m)" in aarch64) plat=arm;; x86_64) plat=x86;; *) plat=unknown;; esac

  declare -A _subdir=([arm]=aarch [x86]=x86)

  if [[ -z "${_subdir[$plat]+_}" ]]; then
    echo "error: unsupported HOST_PLATFORM=${plat}" >&2
    return 1
  fi

  local sub="${_subdir[$plat]}"
  local archive_name="extensions.tar.gz"
  local base_url="https://gitlab.ai2rob.com/zhiqi.jia/binary/-/raw/main"
  local deps="${PROJECT_WORKDIR}/dependencies"
  local deps_bin="${deps}/binary"
  local plat_dir="${deps_bin}/${sub}"
  local path="${plat_dir}/${archive_name}"
  local target_dir="${HOME}/.cursor-server"

  __manage_ide_confirm \
    "Will unpack ${archive_name} from ${plat_dir}/ to ${target_dir} (skip download if archive already exists)." \
    "Update Cursor extensions" || return 0

  mkdir -p "$plat_dir"

  local url="${base_url}/${sub}/${archive_name}"
  if [[ -e "$path" ]]; then
    echo "skipping download: ${path} already exists"
  else
    echo "downloading ${url} ..."
    curl -fSL -o "$path" "$url" || {
      echo "error: download failed for ${archive_name}" >&2
      return 1
    }
  fi

  if [[ ! -f "$path" ]]; then
    echo "error: not a regular file: ${path}" >&2
    return 1
  fi

  if [[ ! -d "$target_dir" ]]; then
    echo "target directory ${target_dir} does not exist, creating..."
    mkdir -p "$target_dir"
  fi

  echo "unpacking ${path} to ${target_dir} ..."
  tar -xzf "$path" -C "$target_dir" || {
    echo "error: unpack failed." >&2
    return 1
  }

  echo "unpack success."
}

##
# @brief 更新 Git 配置
#
# 该函数基于仓库内模板生成用户全局 Git 配置(`~/.gitconfig`)。
# 通过交互输入 `user.name`，并将 `user.email` 固定为「用户名@ai2robotics.com」。
#
# @details
# 首先以黄色提示配合 `[Y/n]` 询问是否继续（默认 Y；输入 n/no 则跳过本函数并返回 0）。
# 函数会检查模板文件 `${AI2_WORKDIR}/.env/template/.gitconfig` 是否存在；若不存在则返回错误。
# 使用 `read -p` 读取 `user.name`；若为空则返回错误。
# 将模板中的占位符 `user_name`(含 `user_name@ai2robotics.com`)替换为输入的用户名后，写入 `~/.gitconfig`(覆盖已有文件)。
#
# @param[in] user.name 通过 `read -p` 交互输入的 Git 用户名；`user.email` 由规则「用户名@ai2robotics.com」推导。
#
# @return 返回值为 0 表示成功，非 0 表示失败。
#
# @note
# - 确保 `${AI2_WORKDIR}` 已正确定义且模板文件存在。
# - 确保当前用户对 `~/.gitconfig` 有写权限；写入方式为覆盖，而非追加。
# - 模板中 `[user]` 的 `name` 与 `email` 须与占位符约定一致(`user_name` / `user_name@ai2robotics.com`)。
#
# @see
# - Git 配置说明：https://git-scm.com/docs/git-config
#
# @example
# 调用示例：
# ```
# config-update-git
# ```
##
function config-update-git() {
  local template="${PROJECT_WORKDIR}/.env/template/.gitconfig"

  __manage_ide_confirm \
    "Will overwrite ~/.gitconfig from ${template}." \
    "Update Git config" || return 0

  if [ ! -f "$template" ]; then
    echo "error: template not found: $template"
    return 1
  fi

  read -r -p "Git user.name: " CURRENT_USER_NAME
  if [ -z "$CURRENT_USER_NAME" ]; then
    echo "error: user.name cannot be empty."
    return 1
  fi

  # 模板中 name 与 email 均含 user_name；email 为 user_name@ai2robotics.com，一次替换即可
  sed "s|user_name|${CURRENT_USER_NAME}|g" "$template" >~/.gitconfig

  echo "git configuration written to ~/.gitconfig (${CURRENT_USER_NAME}@ai2robotics.com)."
}

##
# @brief 更新用户登录 shell 的 bashrc 配置
#
# 该函数将仓库内提供的 `~/.bashrc` 模板经占位符替换后写入用户主目录，并立即 `source` 使其在当前会话生效。
# 适用于初始化或重置与 ai2-robot 工作区一致的交互式 shell 环境。
#
# @details
# 首先以黄色提示配合 `[Y/n]` 询问是否继续（默认 Y；输入 n/no 则跳过本函数并返回 0）。
# 函数会检查模板文件 `${AI2_WORKDIR}/.env/template/.bashrc` 是否存在；若不存在则返回错误。
# 使用 `od -An -N1 -i /dev/urandom | awk '{print $1 % 100}'` 生成 `ROS_DOMAIN_ID`（0–99），
# 再以 `sed` 将模板中的 `custom_domain_id` 替换为该值后写入 `${HOME}/.bashrc`，随后执行 `source "${HOME}/.bashrc"`。
#
# @param[in] 无显式形参；模板与输出路径由环境变量 `${AI2_WORKDIR}`、`${HOME}` 推导。
#
# @return 返回值为 0 表示成功，非 0 表示失败。
#
# @note
# - 确保 `${AI2_WORKDIR}`、`${HOME}` 已正确定义。
# - 确保对 `${HOME}/.bashrc` 有写权限；操作为覆盖写入。
# - 模板内引用的路径(如 `${HOME}/ai2-robot/.env/bashrc`)须与实际部署一致。
#
# @see
# - Bash 启动文件：https://www.gnu.org/software/bash/manual/html_node/Bash-Startup-Files.html
#
# @example
# 调用示例：
# ```
# config-update-bashrc
# ```
##
function config-update-bashrc() {
  local template="${PROJECT_WORKDIR}/.env/template/.bashrc"

  __manage_ide_confirm \
    "Will overwrite ~/.bashrc from ${template} and source it." \
    "Update ~/.bashrc" || return 0

  if [ ! -f "$template" ]; then
    echo "error: template not found: $template"
    return 1
  fi

  local domain_id
  domain_id=$(od -An -N1 -i /dev/urandom | awk '{print $1 % 100}')

  sed -e "s|custom_domain_id|${domain_id}|g" \
      -e "s|custom_workdir_path|${PROJECT_WORKDIR}|g" \
      "$template" >"${HOME}/.bashrc"
  # shellcheck source=/dev/null
  source "${HOME}/.bashrc"

  echo "bashrc configuration written to ~/.bashrc (ROS_DOMAIN_ID=${domain_id}, PROJECT_WORKDIR=${PROJECT_WORKDIR})."
}

##
# @brief 修正用户主目录下 SSH 相关路径的文件权限
#
# 针对 `~/.ssh` 按常见 OpenSSH 要求收紧权限，避免目录/私钥过宽导致 ssh 拒绝连接或告警。
#
# @details
# 首先以黄色提示配合 `[Y/n]` 询问是否继续（默认 Y；输入 n/no 则跳过本函数并返回 0）。
# 若 `${HOME}/.ssh` 不存在则提示后返回 0。否则：对整棵目录树去掉 group/other 写位、目录 700；
# `config` / `authorized_keys` 为 600；`known_hosts`（及 `.old`）为 644；`*.pub` 为 644；
# 默认私钥 `id_rsa` 显式设为 600；其余私钥类（`id_*` 且非 `.pub`、或 `*_key` 且非 `.pub`）递归设为 600。
#
# @param[in] 无显式形参；路径由 `${HOME}` 推导。
#
# @return 返回值为 0 表示成功或未执行（目录不存在）；确认取消时由调用约定返回 0。
#
# @note
# - 不修改 `~/.ssh` 以外的路径；若需其它家目录权限，应另设专用函数。
# - 非常规文件名私钥可能不会被 `find` 规则覆盖，可手工 `chmod 600`。
#
# @example
# ```
# config-update-ssh
# ```
##
function config-update-ssh() {
  local ssh_dir="${HOME}/.ssh"

  __manage_ide_confirm \
    "Will tighten permissions under ${ssh_dir} (OpenSSH-friendly defaults)." \
    "Fix ~/.ssh permissions" || return 0

  if [[ ! -d "$ssh_dir" ]]; then
    echo "info: ${ssh_dir} does not exist, nothing to do."
    return 0
  fi

  chmod -R go-w "$ssh_dir"
  chmod 700 "$ssh_dir"

  find "$ssh_dir" -type f -name '*.pub' -exec chmod 644 {} \; 2>/dev/null

  [[ -f "${ssh_dir}/config" ]] && chmod 600 "${ssh_dir}/config"
  [[ -f "${ssh_dir}/authorized_keys" ]] && chmod 600 "${ssh_dir}/authorized_keys"
  [[ -f "${ssh_dir}/known_hosts" ]] && chmod 644 "${ssh_dir}/known_hosts"
  [[ -f "${ssh_dir}/known_hosts.old" ]] && chmod 644 "${ssh_dir}/known_hosts.old"

  # Default RSA private key (OpenSSH rejects overly permissive modes).
  [[ -f "${ssh_dir}/id_rsa" ]] && chmod 600 "${ssh_dir}/id_rsa"

  find "$ssh_dir" -type f ! -name '*.pub' \( -name 'id_*' -o -name '*_key' \) -exec chmod 600 {} \; 2>/dev/null

  echo "SSH permissions updated under ${ssh_dir}."
}
