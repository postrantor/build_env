# ---
# date: 2026-08-21
# label: Jetson Orin 机器人
# description: Orin 宿主机默认变量。由 __setup_machine_begin 在执行机型入口时 source。
# ---

__setup_machine_default SETUP_SUDOERS_EXTRA robot
__setup_machine_default SETUP_WIFI_DEVICE wlan0
__setup_machine_default SETUP_SKIP_XSESSIONRC 0
__setup_machine_default SETUP_DOCKER_DATA_ROOT "${SETUP_HOME_STORE_ROOT:-/home/data}/docker"
