rosservice call /compute_trajectory "way_point_list:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: '/base_link'
  poses:
  - position:
      x: 0.350
      y: 0.350
      z: 0.005
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  - position:
      x: 0.500
      y: 0.000
      z: 0.050
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  - position:
      x: 0.350
      y: -0.350
      z: 0.050
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0"