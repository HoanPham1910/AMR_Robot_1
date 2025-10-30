#!/bin/bash

# Source ROS2 workspace
source ~/turtlebot3_ws/install/setup.bash

# Hàm in màu
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Chạy 3 lệnh với tag và màu khác nhau
(
    echo -e "${RED}[BRINGUP] Starting turtlebot3_bringup...${NC}"
    ros2 launch turtlebot3_bringup robot.launch.py
) &

(
    echo -e "${GREEN}[CARTOGRAPHER] Starting cartographer...${NC}"
    ros2 launch turtlebot3_cartographer cartographer.launch.py
) &

(
    echo -e "${BLUE}[FAKE_ROBOT] Starting fake_robot.py...${NC}"
    python3 ~/turtlebot3_ws/src/main/fake_robot.py
) &

# Chờ tất cả lệnh xong
wait
