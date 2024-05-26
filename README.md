# Stereo-Visual-Odometry

## Video
[![gif.gif](https://i.postimg.cc/HL5kB7Z8/svo2.gif)](https://youtu.be/MRt1zt7aMEY)

[[🔗 Video Link]](https://youtu.be/MRt1zt7aMEY)

## Overview
This ROS2 Stereo Visual Odometry package facilitates the development and testing of stereo visual odometry systems by providing a highly customizable framework. Users can easily interchange feature extractors, trackers, and optimizers thanks to its template-based design. This package leverages popular libraries such as Sophus, g2o, Ceres, and OpenCV, focusing on flexibility rather than raw performance.

## Features
- **Customizable Components**: Easily switch between different feature extractors, trackers, and optimizers using the provided templates.
- **Support for Major Libraries**: Integrates with Sophus, g2o, Ceres, and OpenCV to ensure a versatile and adaptable development environment.
- **Launch File Execution**: Comes with a ROS2 launch file for easy startup and configuration.

### Extractor
- [x] ORB

### Tracker
- [x] Optical flow
- [x] ORB Matcher

### Optimizer
- [x] SolvePnP
- [x] g2o (GaussNewton)

## Dependencies
This package depends on the following external libraries:
- ROS2 Humble
- Sophus
- g2o
- Ceres Solver
- OpenCV 4.x

## Installation
Clone the package from GitHub and install the required dependencies:
```bash
# Clone the package
git clone https://github.com/dawan0111/Stereo-Visual-Odometry.git
cd Stereo-Visual-Odometry
```
