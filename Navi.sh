#!/bin/bash

# Source ROS2 workspace
source ~/turtlebot3_ws/install/setup.bash

# Màu sắc
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Thư mục log
LOG_DIR=~/turtlebot3_ws/logs
mkdir -p $LOG_DIR

# Chạy 3 lệnh với tag, màu và log riêng
(
    echo -e "${RED}[FAKE_ROBOT] Starting fake_robot.py...${NC}"
    python3 ~/turtlebot3_ws/src/main/fake_robot.py 2>&1 | tee $LOG_DIR/fake_robot.log
) &

(
    echo -e "${GREEN}[NAV2] Starting navigation2...${NC}"
    ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml 2>&1 | tee $LOG_DIR/nav2.log
) &

(
    echo -e "${BLUE}[BRINGUP] Starting turtlebot3_bringup...${NC}"
    ros2 launch turtlebot3_bringup robot.launch.py 2>&1 | tee $LOG_DIR/bringup.log
) &

wait
