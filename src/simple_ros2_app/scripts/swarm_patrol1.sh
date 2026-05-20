#!/bin/bash

echo "Запуск патрулювання для Дрона 1..."

# Безкінечний цикл (зупиняється через Ctrl+C)
while true; do
    echo "--> Летимо в Точку 1 (15, 0)"
    ros2 service call /uav1/companion_control/start_flight simple_ros2_interfaces/srv/DronePosition "{x: 15.0, y: 0.0, z: 8.0}"
    sleep 15

    echo "--> Летимо в Точку 2 (15, 15)"
    ros2 service call /uav1/companion_control/start_flight simple_ros2_interfaces/srv/DronePosition "{x: 15.0, y: 15.0, z: 8.0}"
    sleep 15

    echo "--> Летимо в Точку 3 (0, 15)"
    ros2 service call /uav1/companion_control/start_flight simple_ros2_interfaces/srv/DronePosition "{x: 0.0, y: 15.0, z: 8.0}"
    sleep 15

    echo "--> Летимо в Точку 4 (0, 0)"
    ros2 service call /uav1/companion_control/start_flight simple_ros2_interfaces/srv/DronePosition "{x: 0.0, y: 0.0, z: 8.0}"
    sleep 15
done