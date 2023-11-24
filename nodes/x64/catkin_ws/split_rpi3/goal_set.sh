rostopic pub -1 /rpi3/move_base_simple/goal geometry_msgs/PoseStamped '{ header: { frame_id: "/map" }, pose: { position: { x: 0.9, y: -0.9 }, orientation: { x: 0, y: 0, z: -1, w: 1 } } }'
