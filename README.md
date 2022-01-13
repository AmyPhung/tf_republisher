## Usage - Kinect calibration
+ Ensure base tags are visible in Kinect and that config file is set properly
+ Start up Kinect ROS node
```
cpk run -M -f --net=host --   --env=NVIDIA_VISIBLE_DEVICES=all   --env=NVIDIA_DRIVER_CAPABILITIES=all   --env=DISPLAY --runtime=nvidia   --env=QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix   --privileged
```
+ Start up Kinect apriltag detector
```
roslaunch tagslam apriltag_detector_testbed_kinect.launch
```

+ Start up Kinect tagslam
```
roslaunch tagslam tagslam-testbed-kinect.launch
```
+ Run Kinect launch file, make adjustments in rqt window
```
roslaunch tf_republisher calibrate_kinect_tf.launch
```
+ Click `done` when calibration is complete, then close RQT window

## Usage - Fisheye calibration
+ Ensure base tags are visible in Fisheye and that config file is set properly
+ Start up Fisheye ROS node
```
roslaunch camera_launchers kraft_fisheye.launch
```
+ Start up Fisheye apriltag detector
```
roslaunch tagslam apriltag_detector_testbed_fisheye.launch
```

+ Start up Fisheye tagslam
```
roslaunch tagslam tagslam-testbed-fisheye.launch
```
+ Run Fisheye launch file, make adjustments in rqt window
```
roslaunch tf_republisher calibrate_fisheye_tf.launch
```
+ Click `done` when calibration is complete, then close RQT window

## Usage - Testbed operation
+ Start up Kinect
```
cpk run -M -f --net=host --   --env=NVIDIA_VISIBLE_DEVICES=all   --env=NVIDIA_DRIVER_CAPABILITIES=all   --env=DISPLAY --runtime=nvidia   --env=QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix   --privileged
```
+ Start up Fisheye ROS node
```
roslaunch camera_launchers kraft_fisheye.launch
```
+ Start up Fisheye apriltag detector
```
roslaunch tagslam apriltag_detector_testbed_fisheye.launch
```
+ Start up Fisheye tagslam
```
roslaunch tagslam tagslam-testbed-fisheye.launch
```
+ Connect the tf trees using calibrated values
```
roslaunch tf_republisher publish_tf_testbed.launch
```
