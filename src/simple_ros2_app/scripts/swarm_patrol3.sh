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
    position: {x: -5.0, y: 3.0, z: 15.0} 
  }
}"