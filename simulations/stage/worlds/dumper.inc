# Create the sensors
define hokuyo ranger
(
  sensor(
    range [0 15.0]
    fov 180.0
    samples 400
  )
  color "black"
  size [ 0.05 0.05 0.1 ]
  #ctrl "lasernoise"
)

define sick ranger
(
  sensor(
    range [0 4.0]
    fov 90
    samples 200
  )
  color "black"
  size [ 0.05 0.05 0.1 ]
  #ctrl "lasernoise"
)

define topurg ranger
(
  sensor(
    range_max 30.0
    fov 270.25
    samples 1081
  )
  # generic model properties
  color "black"
  size [ 0.05 0.05 0.1 ]
)

# Create the dumper robot model
define dumper position
(
  #localization "odom"
  localization_origin [0 0 0 0]
  odom_error [0 0 0 0 0 0]
  size [0.2 0.2 0.45]
  origin [0 0 0 0]
  gui_nose 1
  drive "diff"
  #hokuyo(pose [ 0.0 0.000 0.1 180 ])
  #sick(pose [ 0.0 0.100 0.0 0])
  topurg(pose [ 0.275 0.000 0 0.000 ])
  bitmap "robot.png"
)

