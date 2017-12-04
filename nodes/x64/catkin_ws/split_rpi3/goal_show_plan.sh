rosservice call /rpi3/move_base/make_plan '{
  start: {
    header: {
      frame_id: "/map"
    },
    pose: {
      position: {
        x: 0.0, y: 0.0
      },
      orientation: {
        x: 0, y: 0, z: 0, w: 1
      }
    }
  },
  goal: {
    header: {
      frame_id: "/map"
    },
    pose: {
      position: {
        x: 0.24, y: 0.24
      },
      orientation: {
        x: 0, y: 0, z: 0, w: 1
      }
    }
  },
  tolerance: 0.01
}'
echo $?
