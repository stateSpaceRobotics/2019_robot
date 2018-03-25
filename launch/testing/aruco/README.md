# ARuco Test
These files serve as a test of ARuco functionality. They are also an example of correct usage of the `single_marker_tracker.launch` file.

## aruco_elp_usbcam_test.launch
This is the main launch file. It launches an instance of libuvc_camera to run the ELP camera, an image_proc node to rectify the image, two instances of `single_marker_tracker.launch` with different markers, and an Rviz instance to visualize the results.

## ost.yaml
This is the camera calibration file for the ELP camera used for the test.

## params.yaml
This file defines the parameters used. Each aruco node is configured to track different markers.

## aruco.rviz
This file simply configures Rviz to properly output the results of the two ARuco trackers.
