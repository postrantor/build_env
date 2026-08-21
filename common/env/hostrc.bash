# ---
# date: 2026-04-14
# description: 交互式维护 ~/.hostrc(template/.hostrc)、/etc/hosts(template/hosts)；hostrc-add-proxy / hostrc-add-hostname 仅追加对应 export 到 ~/.bashrc；hostrc-update 串联子步骤后配置 ~/.bashrc
#   HOSTRC_BASHRC_MODE — 非交互时可设 append / overwrite / skip(仅当 ~/.bashrc 已存在且尚未含 source ~/.hostrc 时生效)
# ---


# 输出文件与模板路径(PROJECT_WORKDIR 未设时从本脚本所在仓库根推导)
__hostrc_project_workdir() {
  if [[ -n "${PROJECT_WORKDIR:-}" ]]; then
    printf '%s' "$PROJECT_WORKDIR"
    return 0
  fi
  if [[ -n "${AI2_WORKDIR:-}" ]]; then
    printf '%s' "$AI2_WORKDIR"
    return 0
  fi
  local _here
  _here="$(cd "$(dirname "${BASH_SOURCE[0]:-.}")" && pwd)"
  printf '%s' "$(dirname "$_here")"
}

__hostrc_template_path() {
  printf '%s' "$(__hostrc_project_workdir)/.env/template/.hostrc"
}

__hostrc_output_path() {
  printf '%s' "${HOME}/.hostrc"
}

# 若 ~/.hostrc 不存在则从模板复制
__hostrc_ensure_file() {
  local template out
  template="$(__hostrc_template_path)"
  out="$(__hostrc_output_path)"
  if [[ ! -f "$template" ]]; then
    echo "error: template not found: $template" >&2
    return 1
  fi
  if [[ ! -f "$out" ]]; then
    cp -- "$template" "$out" || return 1
    echo "created ${out} from template."
  fi
}

# 交互：仅将 export http_proxy / https_proxy 追加到 ~/.bashrc（不修改 ~/.hostrc）。
hostrc-add-proxy() {
  local default_url="http://clash.postrantor.com:7891"
  local url bashrc="${HOME}/.bashrc"
  read -r -e -p "http/https proxy URL [${default_url}]: " url
  url=${url:-$default_url}
  [[ -n "$url" ]] || {
    echo "error: proxy URL cannot be empty." >&2
    return 1
  }
  {
    printf 'export http_proxy=%s\n' "$url"
    printf 'export https_proxy=%s\n' "$url"
  } >>"$bashrc" || return 1
  echo "appended http_proxy/https_proxy to ${bashrc}"
}

# 交互：http/https 代理 URL(写入与 template 一致的 export http_proxy / https_proxy)
hostrc-update-proxy() {
  __hostrc_ensure_file || return 1
  local out url
  out="$(__hostrc_output_path)"
  local default_url="http://clash.postrantor.com:7891"
  read -r -e -p "http/https proxy URL [${default_url}]: " url
  url=${url:-$default_url}
  if grep -qF 'custom_proxy_url' "$out" 2>/dev/null; then
    sed -i "s|custom_proxy_url|${url}|g" "$out"
  else
    sed -i "s|^export http_proxy=.*|export http_proxy=${url}|" "$out"
    sed -i "s|^export https_proxy=.*|export https_proxy=${url}|" "$out"
  fi
  echo "updated http_proxy/https_proxy in ${out}"
}

# 交互：将 export HOSTNAME 插入 ~/.bashrc 第 2 行（不修改 ~/.hostrc）。
hostrc-add-hostname() {
  local default_name="ai2rob"
  local name bashrc="${HOME}/.bashrc"
  read -r -e -p "HOSTNAME [${default_name}]: " name
  name=${name:-$default_name}
  [[ -n "$name" ]] || {
    echo "error: HOSTNAME cannot be empty." >&2
    return 1
  }
  sed -i "1a export HOSTNAME=${name}" "$bashrc" || return 1
  echo "inserted HOSTNAME into ${bashrc} (line 2)"
}

# 交互：HOSTNAME(template 中 custom_hostname)
hostrc-update-hostname() {
  __hostrc_ensure_file || return 1
  local out name
  local default_name="ai2rob"
  out="$(__hostrc_output_path)"
  read -r -e -p "HOSTNAME [${default_name}]: " name
  name=${name:-$default_name}
  if grep -qF 'custom_hostname' "$out" 2>/dev/null; then
    sed -i "s|custom_hostname|${name}|g" "$out"
  else
    sed -i "s|^export HOSTNAME=.*|export HOSTNAME=${name}|" "$out"
  fi
  echo "updated HOSTNAME in ${out}"
}

# 交互：PROJECT_WORKDIR(template 中 custom_workdir_path)
hostrc-update-workdir() {
  __hostrc_ensure_file || return 1
  local out wd
  out="$(__hostrc_output_path)"
  local suggest
  suggest="$(__hostrc_project_workdir)"
  read -r -e -p "PROJECT_WORKDIR [${suggest}]: " wd
  wd=${wd:-$suggest}
  if [[ -z "$wd" ]]; then
    echo "error: PROJECT_WORKDIR cannot be empty." >&2
    return 1
  fi
  if grep -qF 'custom_workdir_path' "$out" 2>/dev/null; then
    sed -i "s|custom_workdir_path|${wd}|g" "$out"
  else
    sed -i "s|^export PROJECT_WORKDIR=.*|export PROJECT_WORKDIR=${wd}|" "$out"
    sed -i "s|^export AI2_WORKDIR=.*|export PROJECT_WORKDIR=${wd}|" "$out"
  fi
  echo "updated PROJECT_WORKDIR in ${out}"
}

# 交互：ai2rob 别名中的容器名与用户名(template 中 custom_container_name / custom_user_name)
hostrc-update-ai2rob() {
  __hostrc_ensure_file || return 1
  local out cname uname
  out="$(__hostrc_output_path)"
  read -r -e -p "container-entry container name: " cname
  read -r -e -p "container-entry user name: " uname
  if [[ -z "$cname" || -z "$uname" ]]; then
    echo "error: container name and user name cannot be empty." >&2
    return 1
  fi
  if grep -qE 'custom_container_name|custom_user_name' "$out" 2>/dev/null; then
    sed -i "s|custom_container_name|${cname}|g; s|custom_user_name|${uname}|g" "$out"
  else
    sed -i "s|^alias ai2rob=.*|alias ai2rob=\"container-entry ${cname} ${uname}\"|" "$out"
  fi
  echo "updated ai2rob alias in ${out}"
}

# 保证 ~/.bashrc 会加载 ~/.hostrc：已含该行则跳过；无 ~/.bashrc 则新建；
# 已有 ~/.bashrc 且不含该行时，可选追加一行，或将整个 ~/.bashrc 覆盖为仅该行(交互或 HOSTRC_BASHRC_MODE)
__hostrc_ensure_bashrc_sources_hostrc() {
  local bashrc="${HOME}/.bashrc"
  local line='source ~/.hostrc'
  local mode

  if [[ -f "$bashrc" ]] && grep -qF "$line" "$bashrc" 2>/dev/null; then
    echo "~/.bashrc already contains: ${line}"
    return 0
  fi

  if [[ ! -f "$bashrc" ]]; then
    printf '%s\n' "$line" >"$bashrc"
    echo "created ~/.bashrc with: ${line}"
    return 0
  fi

  mode="${HOSTRC_BASHRC_MODE:-}"
  if [[ -z "$mode" ]]; then
    echo -e "\033[33m~/.bashrc exists but does not source ~/.hostrc.\033[0m" >&2
    echo "  [a] append '${line}' at end (default)" >&2
    echo "  [o] overwrite ~/.bashrc with ONLY this line (removes all other content)" >&2
    echo "  [s] skip ~/.bashrc" >&2
    read -r -e -p "Choice [a/o/s] (default a): " mode
  fi

  case "${mode,,}" in
  o | overwrite)
    printf '%s\n' "$line" >"$bashrc"
    echo "overwrote ~/.bashrc with: ${line}"
    ;;
  s | skip | n | no)
    echo "skipped ~/.bashrc"
    ;;
  *)
    printf '\n%s\n' "$line" >>"$bashrc"
    echo "appended to ~/.bashrc: ${line}"
    ;;
  esac
}

# 从模板重置 ~/.hostrc，依次交互替换占位符；结果写入 ~/.hostrc，再按选择追加或覆盖 ~/.bashrc 中的 source ~/.hostrc
hostrc-update() {
  local template out
  template="$(__hostrc_template_path)"
  out="$(__hostrc_output_path)"
  if [[ ! -f "$template" ]]; then
    echo "error: template not found: $template" >&2
    return 1
  fi

  echo -e "\033[33mreset ${out} from template, then proxy / HOSTNAME / PROJECT_WORKDIR / ai2rob prompts.\033[0m" >&2
  cp -- "$template" "$out" || return 1

  hostrc-update-proxy || return 1
  hostrc-update-hostname || return 1
  hostrc-update-workdir || return 1
  hostrc-update-ai2rob || return 1

  echo "host configuration written to ${out}"
  __hostrc_ensure_bashrc_sources_hostrc || return 1
  echo "open a new shell or: source ~/.bashrc"
}

## --- --- ---

__hostrc_hosts_template_path() {
  printf '%s' "$(__hostrc_project_workdir)/.env/template/hosts"
}

# template/hosts 中非空、非 # 注释行是否均已整行出现在 /etc/hosts 中
__hostrc_hosts_lines_all_in_etc() {
  local t="$1" hp="${2:-/etc/hosts}"
  local line
  while IFS= read -r line || [[ -n "$line" ]]; do
    [[ "$line" =~ ^[[:space:]]*# ]] && continue
    [[ -z "${line//[[:space:]]/}" ]] && continue
    grep -qFx "$line" "$hp" 2>/dev/null || return 1
  done <"$t"
  return 0
}

# 将 template/hosts 全文追加到 /etc/hosts(需 sudo；已存在相同条目行则跳过)
hostrc-update-hosts() {
  local t hp
  t="$(__hostrc_hosts_template_path)"
  hp=/etc/hosts
  if [[ ! -f "$t" ]]; then
    echo "error: template not found: $t" >&2
    return 1
  fi
  if [[ ! -f "$hp" ]]; then
    echo "error: missing $hp" >&2
    return 1
  fi

  if __hostrc_hosts_lines_all_in_etc "$t" "$hp"; then
    echo "all non-comment lines from ${t} already present in ${hp}; nothing to do."
    return 0
  fi

  echo -e "\033[33will append the following to ${hp} (requires sudo):\033[0m" >&2
  cat "$t" >&2
  local ans
  read -r -e -p "proceed? [Y/n] " ans
  case "${ans,,}" in
  n | no)
    echo "skipped."
    return 0
    ;;
  esac

  if ! cat "$t" | sudo tee -a "$hp" >/dev/null; then
    echo "error: failed to append to ${hp} (sudo?)" >&2
    return 1
  fi
  echo "appended ${t} to ${hp}"
}
