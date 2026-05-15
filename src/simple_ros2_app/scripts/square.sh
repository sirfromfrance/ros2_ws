#!/bin/bash

ros2 service call /uav0/companion_control/start_flight simple_ros2_interfaces/srv/DronePosition "{x: 10.0, y: 0.0, z: 10.0}"
ros2 service call /uav0/companion_control/start_flight simple_ros2_interfaces/srv/DronePosition "{x: 10.0, y: 10.0, z: 10.0}"
ros2 service call /uav0/companion_control/start_flight simple_ros2_interfaces/srv/DronePosition "{x: 0.0, y: 10.0, z: 10.0}"
ros2 service call /uav0/companion_control/start_flight simple_ros2_interfaces/srv/DronePosition "{x: 0.0, y: 0.0, z: 10.0}"

