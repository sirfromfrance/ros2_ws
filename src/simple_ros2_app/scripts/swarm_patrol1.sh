#!/bin/bash
ros2 topic pub --once /swarm/broadcast/position geometry_msgs/msg/PoseStamped "{
  header: {
    stamp: {
      sec: $(date +%s),
      nanosec: 0
    },
    frame_id: 'map'
  },
  pose: {
    position: {x: 10.0, y: 10.0, z: 10.0}
  }
}"