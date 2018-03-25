# single_marker_tracker.launch usage

This launch file initializes a aruco_ros/single node to track a marker.

It takes in three arguments when included in another launch file:
* node_name: Names the node (use this in conjunction with parameters)
* image_rect_topic: The image topic to subscribe to for a rectified image
* camera_info_topic: The camera infor topic for the camera corresponding to the subscribed image

## Example launch include
`    <include file="/home/ssr/ros_kinetic/rmc_dev_ws/src/2018_robot/launch/components/aruco/single_marker_tracker.launch">
      <arg name="node_name" value="aruco_node_415" />
      <arg name="image_rect_topic" value="image_rect_color"/>
      <arg name="camera_info_topic" value="camera_info"/>
    </include>`

## Corresponding parameter file example
`aruco_node_415:
  marker_size:     0.3667125
  marker_id:       415
  camera_frame:    elp_camera
  marker_frame:    415_test_marker
  reference_frame: ""
  corner_refinement: LINES
  image_is_rectified: True`
