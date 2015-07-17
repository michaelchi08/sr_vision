[![Build Status](https://api.shippable.com/projects/5551c20fedd7f2c052e9809d/badge?branchName=indigo-devel)](https://app.shippable.com/projects/5551c20fedd7f2c052e9809d) [![Documentation Status](https://readthedocs.org/projects/sr-vision/badge/?version=latest)](http://sr-vision.readthedocs.org/) [![Code Health](https://landscape.io/github/shadow-robot/sr_vision/indigo-devel/landscape.svg?style=flat)](https://landscape.io/github/shadow-robot/sr_vision/indigo-devel) [![codecov.io](http://codecov.io/github/shadow-robot/sr_vision/coverage.svg?branch=indigo-devel)](http://codecov.io/github/shadow-robot/sr_vision?branch=indigo-devel)

sr_vision
------------

## Overview
Contains our vision related algorithm (segmentation, tracking, recognition, etc...)

  1. [Tracking](sr_object_tracking/)
This package contains the tracker executable. The launch file starts the video acquisition as well as the tracking node and the visualization one. A color parameter is necessary to process the segmentation.
  2. [Segmentation](sr_object_segmentation/)
Segment the image from the camera in order to find the region of interest, with a color given as parameter.
  3. [Benchmarking](sr_object_benchmarking/)
Benchmarking package for the different segmentation algorithms.
  4. [Visualization](sr_gui_servoing/)
Visualization package : display the image from the camera, let the user select a region of interest, and display the tracking.
  5. [PointCloud utils](sr_point_cloud/)
This package contains a tracker, a point cloud triangulator, a segmentation tool and a tool to transform point clouds.
  6. [Messages](sr_vision_msgs/)
All messages, services and actions for sr_vision.
  7. [Extrinsic camera calibration](sr_extrinsic_calibration/)
Contains nodes for extrinsic camera calibration based on Alvar markers positions in camera and on real robot.

You can find the architecture diagram below for a closer look at how this works.

![Architecture Diagram](doc/sr_vision.png)


## Usage
For the segmentation, different colors are available : red, blue, green, yellow.

### With a Kinect
`roslaunch sr_object_tracking tracking_kinect.launch color:=<color>`

### With an UVC camera
`roslaunch sr_object_tracking tracking.launch color:=<color>`


