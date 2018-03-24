# Create the digger robot model
define digger position
(
  #localization "odom"
  localization_origin [0 0 0 0]
  odom_error [0 0 0 0 0 0]
  size [0.2 0.2 0.45]
  origin [0 0 0 0]
  gui_nose 1
  drive "diff"
  bitmap "robot.png"
)
