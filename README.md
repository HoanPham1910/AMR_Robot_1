# ROS2 Humble Robot AMR

## Tổng quan
    - Bộ code này triển khai điều khiển và mô phỏng cho robot AMR sử dụng ROS2 Humble. Bao gồm:
    - Điều khiển động cơ Ezi Servo.
    - Mô phỏng robot (fake robot) để test SLAM với Lidar.
    - Xử lý thuật toán SLAM và Navigation dựa trên TurtleBot3.

## Mô tả Code

 - Folder_F0:
    "rplidar_ros": code chạy rplidar được tải từ hãng
    "main" : nơi xử lí tín hiểu điều khiển động cơ - đưa ra các topic /cmd_vel , /joint_states, /odom
    "turtlebot3" : xử lí các thuật toán SLAM , Navigation

 - Folder_F1:
    ------------------ MAIN - Folder ------------------------
    + Library: Thư viện thanh ghi của Driver Ezi Servo
    + OpenCR: Thư viện hỗ trợ nếu chạy bằng MCU
    + Topic_python: Nơi xử lí các topic (xử lí dữ liệu từ các Node đưa về - transfrom)
    + UI: Code giao diện điều khiển động cơ ( theo dõi đồng tốc 2 động cơ ezi_servo)

    ------------------ MAIN - File ------------------------
    + fake_robot.py: Mô phỏng robot nhận lệnh tốc độ, đọc encoder để test SLAM.
    + giaodien.py: Code giao diện điều khiển động cơ.
    + main.py: File chính điều khiển động cơ Ezi Servo trên robot AMR.

     ------------------ turtlebot3 - Folder ------------------------
     + turtlebot3_bringup: Xử lý tín hiệu Lidar, nhận topic từ MAIN.
     + turtlebot3_cartographer: Thuật toán SLAM, điều chỉnh thông số quét map.
     + turtlebot3_description: Mô tả robot (URDF). Chỉnh sửa nếu trục hoặc chiều quay sai.
     + turtlebot3_navigation2: Thuật toán Navigation (ACML) ( điều chỉnh thuật toán navi trong folder này - ACML)
     + turtlebot3_node: Xử lý topic /odom và nhận tín hiệu từ MAIN (chỉ chỉnh sửa khi MAIN viết bằng C++)
     + turtlebot3_teleop: Điều khiển robot bằng teleop.

- Folder_F2:
    ------------------ Topic_python - Folder ------------------------
    + cmd_vel: topic xử lí tín hiệu tốc độ động cơ
    + joint_states: topic xử lí tín hiệu encoder

## Cách hoạt động nhanh
 - Nhập lệnh: "start_SLAM" -> để bắt đầu quét map
 - Nhập lệnh: "Navi" -> bắt đầu điều hướng robot trên map đã quét

## Những lưu ý khi chạy Ros2
 - Phải build lại code khi chỉnh sửa - lệnh:

            cd ~/turtlebot3_ws
            colcon build --symlink-install
            source install/setup.bash

 - Phải cấp quyền cho cổng USB Lidar - lệnh:

            sudo chmod 666 /dev/ttyUSB0

 - Kiểm tra topic ROS2:
    Các topic chính: /cmd_vel, /joint_states, /odom.
    Đảm bảo các topic này được publish/subscribe đúng.