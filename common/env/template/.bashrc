## date: 2026-06-14 08:31:05

## --- show current time ---
#TZ=Asia/Shanghai date +"%Z %Y/%m/%d %A"
#TZ=Asia/Shanghai date +"%X"

## --- set hostname ---
export HOSTNAME=custom_hostname

## --- source bashrc ---
#export HOSTNAME=dev.robot # e.g. "robot.2.0131"
# /opt/ros/ai2rob is sourced from ${PROJECT_WORKDIR}/.env/robot.bash (setup_ros2)
source custom_workdir_path/.env/bashrc

## --- override default environment variables ---
export ROS_DOMAIN_ID=custom_domain_id
#export CYCLONEDDS_URI=file://${CONFIG_PATH}/.runtime/cyclonedds/config/bot2/orin.xml
unset CYCLONEDDS_URI
source ${AI2_WORKDIR}/.env/check-environment.bash

## --- set x11 display ---
#export DISPLAY=:10.0

## --- set alpha-bot ---
export ROBOT_TYPE="bot2" # bot1s/bot2
export CHASSIS_TYPE="bot2" # bot1s/bot2
export ARM_TYPE="rm_zpf_73" # rm_65/rm_zpf_73
export EFFECTOR_TYPE="inspire" # changingtek/inspire

## --- set model vender ---
export VENTOR_DOWNLOAD_PATH="${PROJECT_WORKDIR}/dependencies/"

## --- switch to workspace ---
cw
