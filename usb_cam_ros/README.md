ROS相机标定的方法

1. 打开相机
 ```
 roslaunch usb_cam usb_cam-test.launch
 ```

2. 运行校准程序
 ```
  rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 image:=/usb_cam/image_raw camera:=/usb_cam
 ```
其中 size是角点个数，需要为矩形

square 是每个方格的长度

usb_cam/image_raw是相机拍摄的图像的主题

usb_cam是相机

这个可以使用`rosnode list` && `rosnode info xxx`查看


usb_cam [![Build Status](https://api.travis-ci.org/bosch-ros-pkg/usb_cam.png)](https://travis-ci.org/bosch-ros-pkg/usb_cam)
=======

#### A ROS Driver for V4L USB Cameras
This package is based off of V4L devices specifically instead of just UVC.

For full documentation, see [the ROS wiki](http://ros.org/wiki/usb_cam).

[Doxygen](http://docs.ros.org/indigo/api/usb_cam/html/) files can be found on the ROS wiki.

### License
usb_cam is released with a BSD license. For full terms and conditions, see the [LICENSE](LICENSE) file.

### Authors
See the [AUTHORS](AUTHORS.md) file for a full list of contributors.
