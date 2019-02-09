
define map model
(
  color "black"
  boundary 0
  gui_nose 0
  gui_grid 0
  gui_move 0
  gui_outline 0
  obstacle_return 1
  gripper_return 0
  fiducial_return 0
# laser_return 1
)
# Instantiate a map according to NASA RMC specifications
map
(
  name "mining_field"
  size [3.691 5.756 0.5]
  pose [1.845 2.878 0 0]
  bitmap "mining_field.png"
)

# Open a window centered on the map
window
(
  size [ 371.000 607.000 ]
  center [1.845 2.878]
  scale 100.0
)
