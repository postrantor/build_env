# ---
# date: 2026-08-21
# description: Wi-Fi 连接与固定 IPv4 (nmcli)。
# ---

# ip-fixed
#
# 功能描述：
#   将指定 NetworkManager 连接配置为固定 IPv4, 并重启该连接使配置生效。
#
# 参数：
#   -i | --interactive - 交互式输入 CONNECTION、DEVICE、ADDRESS、GATEWAY、DNS。
#   -c | --connection - nmcli connection name (默认: 317524135C280015-1f26)。
#   -d | --device - 网卡设备名 (默认: wlp0s20f3)。
#   -a | --address | --ip - IPv4/CIDR (默认: 192.168.90.100/24)。
#   -g | --gateway - 网关地址 (默认: 192.168.90.1)。
#   --dns - DNS 列表, 空格分隔 (默认: 192.168.90.1 223.5.5.5 8.8.8.8)。
#   --no-restart - 只修改配置, 不 down/up 重启连接。
#   $1..$5: CONNECTION DEVICE ADDRESS GATEWAY DNS - 位置参数, 覆盖上述默认值。
#
# 使用示例：
#   ip-fixed
#   ip-fixed -i
#   ip-fixed --connection 317524135C280015-1f26 --device wlp0s20f3 --address 192.168.90.100/24
#   sudo bash common/setup/network.bash one_ai2robot_5G wlp2s0 192.168.66.58/24 192.168.66.1 "192.168.66.1 223.5.5.5"
#
# 注意事项：
#   1. 需要 nmcli。
#   2. 默认连接名/网卡是开发机遗留值, 生产请显式传参或用 SETUP_*。
#
ip-fixed() {
  case "${1-}" in
  --help | -h)
    cat >&2 <<'EOF'
ip-fixed — set a NetworkManager Wi-Fi connection to a static IPv4 address.

Usage: ip-fixed [OPTIONS] [CONNECTION] [DEVICE] [ADDRESS] [GATEWAY] [DNS]
       ip-fixed -i

Options:
  -i, --interactive      prompt for all values
  -c, --connection NAME  nmcli connection name (default: 317524135C280015-1f26)
  -d, --device DEVICE    network device (default: wlp0s20f3)
  -a, --address CIDR     static IPv4/CIDR (default: 192.168.90.100/24)
      --ip CIDR          alias of --address
  -g, --gateway IP       gateway (default: 192.168.90.1)
      --dns DNS_LIST     DNS list, quoted when it contains spaces
      --no-restart       only modify the connection, do not down/up it

Examples:
  ip-fixed
  ip-fixed -i
  ip-fixed --connection 317524135C280015-1f26 --device wlp0s20f3 --address 192.168.90.100/24
  ip-fixed 317524135C280015-1f26 wlp0s20f3 192.168.90.100/24 192.168.90.1 "192.168.90.1 223.5.5.5"
EOF
    return 0
    ;;
  esac

  local interactive=0 restart=1
  local connection_name="317524135C280015-1f26"
  local device_name="wlp0s20f3"
  local ip_address="192.168.90.100/24"
  local gateway="192.168.90.1"
  local dns="192.168.90.1 223.5.5.5 8.8.8.8"
  local -a args=()

  # 解析命令行选项；位置参数按 CONNECTION DEVICE ADDRESS GATEWAY DNS 覆盖默认值
  while [[ $# -gt 0 ]]; do
    case "$1" in
    -i | --interactive)
      interactive=1
      shift
      ;;
    -c | --connection)
      if [[ $# -lt 2 ]]; then
        echo "ip-fixed: missing value for $1" >&2
        return 1
      fi
      connection_name=$2
      shift 2
      ;;
    --connection=*)
      connection_name=${1#*=}
      shift
      ;;
    -d | --device)
      if [[ $# -lt 2 ]]; then
        echo "ip-fixed: missing value for $1" >&2
        return 1
      fi
      device_name=$2
      shift 2
      ;;
    --device=*)
      device_name=${1#*=}
      shift
      ;;
    -a | --address | --ip)
      if [[ $# -lt 2 ]]; then
        echo "ip-fixed: missing value for $1" >&2
        return 1
      fi
      ip_address=$2
      shift 2
      ;;
    --address=* | --ip=*)
      ip_address=${1#*=}
      shift
      ;;
    -g | --gateway)
      if [[ $# -lt 2 ]]; then
        echo "ip-fixed: missing value for $1" >&2
        return 1
      fi
      gateway=$2
      shift 2
      ;;
    --gateway=*)
      gateway=${1#*=}
      shift
      ;;
    --dns)
      if [[ $# -lt 2 ]]; then
        echo "ip-fixed: missing value for $1" >&2
        return 1
      fi
      dns=$2
      shift 2
      ;;
    --dns=*)
      dns=${1#*=}
      shift
      ;;
    --no-restart)
      restart=0
      shift
      ;;
    --)
      shift
      args+=("$@")
      break
      ;;
    -*)
      echo "ip-fixed: unknown option: $1" >&2
      return 1
      ;;
    *)
      args+=("$1")
      shift
      ;;
    esac
  done

  if ((${#args[@]} > 5)); then
    echo "ip-fixed: too many arguments" >&2
    return 1
  fi

  connection_name=${args[0]:-$connection_name}
  device_name=${args[1]:-$device_name}
  ip_address=${args[2]:-$ip_address}
  gateway=${args[3]:-$gateway}
  dns=${args[4]:-$dns}

  if [[ $interactive -eq 1 ]]; then
    echo -e "\033[33mip-fixed: interactive mode\033[0m" >&2
    read -r -e -p "connection name [${connection_name}]: " connection_name_input
    connection_name=${connection_name_input:-$connection_name}
    read -r -e -p "device name [${device_name}]: " device_name_input
    device_name=${device_name_input:-$device_name}
    read -r -e -p "IPv4/CIDR [${ip_address}]: " ip_address_input
    ip_address=${ip_address_input:-$ip_address}
    read -r -e -p "gateway [${gateway}]: " gateway_input
    gateway=${gateway_input:-$gateway}
    read -r -e -p "DNS [${dns}]: " dns_input
    dns=${dns_input:-$dns}
  fi

  if ! command -v nmcli >/dev/null 2>&1; then
    echo "ip-fixed: nmcli not found" >&2
    return 1
  fi

  sudo nmcli connection modify "${connection_name}" \
    connection.interface-name "${device_name}" \
    connection.autoconnect yes \
    ipv4.method manual \
    ipv4.addresses "${ip_address}" \
    ipv4.gateway "${gateway}" \
    ipv4.dns "${dns}" \
    ipv4.ignore-auto-dns yes \
    ipv6.method auto

  if [[ $restart -eq 1 ]]; then
    sudo nmcli connection down "${connection_name}" || true
    sudo nmcli connection up "${connection_name}" || return 1
  fi

  nmcli device show "${device_name}" | grep -E 'IP4.ADDRESS|IP4.GATEWAY|IP4.DNS'
}

# setup-wifi
#
# 功能描述：
#   用 nmcli 连接 Wi-Fi。密码只从 SETUP_WIFI_PASSWORD 或交互输入读取, 不写进仓库。
#
# 参数：
#   $1: ssid - SSID (默认: SETUP_WIFI_SSID)。
#   $2: device - 网卡 (默认: SETUP_WIFI_DEVICE)。
#
# 使用示例：
#   SETUP_WIFI_SSID=office SETUP_WIFI_PASSWORD=secret setup-wifi
#   setup-wifi office wlan0
#
# 注意事项：
#   1. 需要 nmcli。
#   2. 非交互且无密码时失败。
#
setup-wifi() {
  local ssid="${1:-${SETUP_WIFI_SSID:-}}"
  local device="${2:-${SETUP_WIFI_DEVICE:-}}"
  local password="${SETUP_WIFI_PASSWORD:-}"

  if ! command -v nmcli >/dev/null 2>&1; then
    __setup_error "nmcli not found"
    return 1
  fi

  if [[ -z "${ssid}" ]]; then
    if [[ -t 0 && "${SETUP_NONINTERACTIVE:-}" != "1" ]]; then
      read -r -e -p "Wi-Fi SSID: " ssid
    fi
  fi
  if [[ -z "${ssid}" ]]; then
    __setup_error "SSID is required (SETUP_WIFI_SSID or argument)"
    return 1
  fi

  if [[ -z "${password}" && -t 0 && "${SETUP_NONINTERACTIVE:-}" != "1" ]]; then
    read -r -s -e -p "Wi-Fi password: " password
    printf '\n'
  fi
  if [[ -z "${password}" ]]; then
    __setup_error "password is required (SETUP_WIFI_PASSWORD or prompt)"
    return 1
  fi

  __setup_warn "Will connect Wi-Fi SSID ${ssid}${device:+ on ${device}}."
  __setup_confirm "nmcli wifi connect ${ssid}?" || return 0

  __setup_sudo nmcli device wifi rescan || true
  if [[ -n "${device}" ]]; then
    __setup_sudo nmcli device wifi connect "${ssid}" password "${password}" ifname "${device}" || return 1
  else
    __setup_sudo nmcli device wifi connect "${ssid}" password "${password}" || return 1
  fi
  __setup_log "connected ${ssid}"
}

# setup-ip-fixed
#
# 功能描述：
#   封装 ip-fixed。有参数则透传; 无参数时用环境变量或交互 (`ip-fixed -i`)。
#
# 参数：
#   $@: 透传给 ip-fixed; 也可不传而使用 SETUP_WIFI_SSID + SETUP_IP_CIDR。
#
# 使用示例：
#   setup-ip-fixed
#   SETUP_WIFI_SSID=office SETUP_IP_CIDR=192.168.66.58/24 setup-ip-fixed
#
setup-ip-fixed() {
  if [[ $# -gt 0 ]]; then
    ip-fixed "$@"
    return
  fi

  if [[ -n "${SETUP_WIFI_SSID:-}" && -n "${SETUP_IP_CIDR:-}" ]]; then
    local -a ip_args=(--connection "${SETUP_WIFI_SSID}" --address "${SETUP_IP_CIDR}")
    [[ -n "${SETUP_WIFI_DEVICE:-}" ]] && ip_args+=(--device "${SETUP_WIFI_DEVICE}")
    [[ -n "${SETUP_GATEWAY:-}" ]] && ip_args+=(--gateway "${SETUP_GATEWAY}")
    [[ -n "${SETUP_DNS:-}" ]] && ip_args+=(--dns "${SETUP_DNS}")
    ip-fixed "${ip_args[@]}"
    return
  fi

  if [[ -t 0 && "${SETUP_NONINTERACTIVE:-}" != "1" ]]; then
    ip-fixed -i
    return
  fi

  __setup_log "skip ip-fixed: pass arguments or SETUP_WIFI_SSID + SETUP_IP_CIDR."
}

# setup-network
#
# 功能描述：
#   有 SSID 时先连 Wi-Fi, 再尝试固定 IP。均无参数时只提示。
#
# 参数：
#   $@: 透传给 setup-wifi。
#
# 使用示例：
#   SETUP_WIFI_SSID=office setup-network
#
setup-network() {
  if [[ -n "${SETUP_WIFI_SSID:-}" || $# -gt 0 ]]; then
    setup-wifi "$@" || return 1
  fi
  setup-ip-fixed
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  ip-fixed "$@"
fi
