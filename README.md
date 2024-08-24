# Implementation-3DAMCL-ORB-SLAM

**Tutorial-Implementation-3DAMCL-ORB-SLAM-V1.0, August 24th, 2024**

Modify by Zaed Al Musthofa-TelU-Student

==========================================================================

# **1. License**
ORB-SLAM3 is released under [GPLv3 licence](url). For a list all code/library dependencies (associated licences), 
please see [Dependencies.md.](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Dependencies.md)

For a closed-source version of ORB-SLAM3 for commercial purposes, please contact the author: orbslam (at) 
unizar (dot) es.

If you use ORB-SLAM3 in an academic work,please cite:

```
    @article{ORB-SLAM3_TRO,
      title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial
              and Multi-Map {SLAM}},
      author={Campos, Carlos AND Elvira, Richard AND G\`omez, Juan J. AND Montiel,
              Jos\'e M. M. AND Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={37},
      number={6},
      pages={1874-1890},
      year={2021}
}
```
# **2. Prerequisites**
we have tasted the library in ubuntu **16.04** and **18.04**, but it should be easy to compile in others platforms. A powerful
(e.g.i7) will ensure real-time performance and provide more stable and accurate result.

## **C++11 or C++0x Compiler**
we use the new thread and chrono functionalities of C++11.
## Pangolin
We use [pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instruction can be found at: https:[//github.com/stevenlovegrove/Pangolin](https://github.com/stevenlovegrove/Pangolin).
## OpenCV
We use [OpenCV](http://opencv.org.) to manipulate image and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 3.0. Tested with OpenCV 3.2.0 and 4.4.0.**
## Eigen3
Required by g2o (see blow). Dowload and install instruction can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0.**
## DBoW2 and g20 (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizationns. Both modified lurlibraries (which are BSD) are included in the _Thirdparty_ folder.

## Python 
Required to calculate the alignment of the trajectory with ground truth. **Required Numpy module**.
+ (win) http://www.python.org/downloads/windows
+ (deb) `sudo apt install libpython2.7-dev` 
+ (mac) preinstalled with osx

## ROS (optional)
We provide some examples to process input of a monocolar, monocular-inertial, stereo, stereo-inertial or RGB-D camera using ROS. Building these example is optional. These have been tested with ROS Melodic under Ubuntu 18.04.

This repository contains a tutorial on implementing 3DAMCL-ORB-SLAM by Mr.Herusyahputra
