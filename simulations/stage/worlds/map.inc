
define map model
(
  color "gray30"
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
  size [3.78 7.38 0.5]
  pose [1.89 3.69 0 0]
  bitmap "mining_field.png"
)

# Open a window centered on the map
window
(
  size [ 400.000 820.000 ]
  center [1.89 3.69]
  scale 100.0
)
