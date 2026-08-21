# ---
# date: 2025-01-12
# update: 2026-04-08 02:14:37
# author: postrantor@gmail.com
# description: robot env entry
# ---


# set color
PURPLE="\033[35m"
GREEN="\033[32m"
RESET="\033[0m"

echo -e "${PURPLE}Checking ros2 environment...${RESET}"
echo -e "  ${GREEN}- PROJECT_NAME${RESET}    = ${PROJECT_NAME:-<not set>}"
echo -e "  ${GREEN}- PROJECT_WORKDIR${RESET} = ${PROJECT_WORKDIR:-<not set>}"
echo -e "  ${GREEN}- ROS_DOMAIN_ID${RESET}  = ${ROS_DOMAIN_ID:-<not set>}"
echo -e "  ${GREEN}- CYCLONEDDS_URI${RESET} = ${CYCLONEDDS_URI:-<not set>}"
