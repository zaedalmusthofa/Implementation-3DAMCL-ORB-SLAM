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

# 3. Building ORB-SLAM3 library and examples 
Clone the repository:
```
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
```
We provide a script `build.sh` to build the _Thirdparty_ libraries and _ORB-SLAM3_. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM3.so** at _lib _ folder and the executables in _Examples _folder.

# 4. Running ORB-SLAM3 with your camera 
Directory `Examples` contain several demo programs and calibration files to run ORB-SLAM3 in all sensor configuration with intel Realsense cameras T265 and D435i. The steps needed to use your own camera are:
 1. Calibrate your camera following `calibration_Tutorial.pdf` and write your calibration file `your_camera.yaml`
 2. Modify one of the provided demos to suit your specific camera model, and build it
 3. Connect the camera to your computer using USB3 or the appropriate interface
 4. Run ORB-SLAM3. For example, for our D435i camera, we would execute:

```
./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/RealSense_D435i.yaml
```

## Version 2 How to run | Usage 
Mono Image 
```
./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml dataset/V1_01_easy ./Examples/Stereo/EuRoC_TimeStamps/V101.txt
```

Stereo Image 
```
./Examples/Stereo-Inertial/stereo_inertial_tum_vi ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/TUM-VI.yaml dataset/dataset-corridor4_512_16/mav0/cam0/data dataset/dataset-corridor4_512_16/mav0/cam1/data ./Examples/Stereo-Inertial/TUM_TimeStamps/dataset-corridor4_512.txt ./Examples/Stereo-Inertial/TUM_IMU/dataset-corridor4_512.txt dataset-corridor4
```
# 5. EuRoC examples 
[EuRoC dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) was record with two pinhole cameras and an inertial sensor. We provide an examples script to launch EuRoC sequence in all senzor configurations.
 1. Dowload a sequence (ASL format) from  https://vision.in.tum.de/data/datasets/visual-inertial-dataset and uncompress it.
 2.  Open the script "tun_vi_examples.sh" in the root of the project. Change **pathDatasetEuRoC** variable to pointto the diractory where the dataset has been uncompressed.
 3.  Execute the following script to process all the sequences with all sensor configurations:
```
 ./euroc_examples
```

## Evaluation 
EuRoC provides ground truth for each sequence in the IMU body reference. As pure visuak executions report trajectories centeres in the left camera, we provide in the "evaluation" folder transformation of the ground truth from dataset.

Execute the following script to process sequences and compute the RMS ATE:
```
./euroc_eval_examples
```
 # 6. TUM-VI Examples 
 [TUM-VI dataset](https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset) was recorded with two fisheye cameras and an inertial sensor.
  1. Dowload a sequence from https://vision.in.tum.de/data/datasets/visual-inertial-dataset and uncompress it.
  2. Open the script "tum_vi_exampls.sh" in the root of the project. Change **pathDatasetTUM_VI** variable to pint to the directory where the dataset has been uncompressed.
  3. Execute the following script to process al sequences with all sensor configuration:
```
./tum_vi_examples
```

## Evaluation 
In TUM-VI ground truth is only avalaible in the room where all sequances start and end. As a result the error measures the drift at the end of the sequance.

Execute the following to process sequences and compute the RMS ATE:
```
./tum_vi_eval_examples
```
# 7. ROS Examples 
### Build the nodes for mono, mono-inertial, stereo, stereo-inertial and RGB-D

tested with ROS AN ubuntu 18.04.
 1. Add the path including Examples/ROS/ORB_SLAM3 to the ROS_PACKAGE_PATH environment variable. Open .bashrc file:
```
gedit ~/.bashrc
```
and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM3:
```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS
```
 2. execute `build_ros.sh` script
```
chmod +x build_ros.sh
./build_ros.sh
```
### Running Monocular Node 
For a monocular input from topic `/camera/image_raw` run node ORB_SLAM3/Mono. You will need to provide the vocabulary file and a settings file. See the monocular examples above.
```
rosrun ORB_SLAM3 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
```
### Running Monocular-Inertial Node 
For a monocular input from topic `/camera/image_raw` and an inertial input from topic `/imu`, run node ORB_SLAM3/Mono_Inertial. Setting the optional third argument to true will apply CLAHE equalization to images (Mainly for TUM-VI dataset).
```
rosrun ORB_SLAM3 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE [EQUALIZATION]	
```
### Running Stereo Node 
For a stereo input from topic `/camera/left/image_raw` and `/camera/right/image_raw` run node ORB_SLAM3/Stereo. You will need to provide the vocabulary file and a settings file. For Pinhole camera model, if you **provide rectification matrices** (see Examples/Stereo/EuRoC.yaml example), the node will recitify the images online, **otherwise images must be pre-rectified**. For FishEye camera model, rectification is not required since system works with original images:
```
rosrun ORB_SLAM3 Stereo PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION
```
### Running Stereo-Inertial Node 
For a stereo input from topics `/camera/left/image_raw` and `/camera/right/image_raw`, and an inertial input from topic `/imu`, run node ORB_SLAM3/Stereo_Inertial. You will need to provide the vocabulary file and a settings file, including rectification matrices if required in a similar way to Stereo case:
```
rosrun ORB_SLAM3 Stereo_Inertial PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION [EQUALIZATION]	
```
### Running RGB-D NODE 
For an RGB-D input from topics `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw`, run node ORB_SLAM3/RGBD. You will need to provide the vocabulary file and a settings file. See the RGB-D example above.
```
rosrun ORB_SLAM3 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
```
Running ROS ecample: Dowlad a rosbag(e.g. V1_02_medium.bag) rom the EuRoC dataset (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Open 3 tabs on the terminal and run the following command at each tab for a Stereo-Inertial configuration:
```
roscore
```

```
rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml true
```

```
rosbag play --pause V1_02_medium.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu
```
Once ORB-SLAM3 has loaded the vocabulary, press space in the rosbag tab.

**Remark:** For rosbags from TUM-VI dataset, some play issue may appear due to chunk size. One possible solution is to rebag them with the default chunk size, for example:
```
rosrun rosbag fastrebag.py dataset-room1_512_16.bag dataset-room1_512_16_small_chunks.bag
```
# 8. Running time analysis
A flag in `include\Config.h` activates time measurements. It is necessary to uncomment the line `#define REGISTER_TIMES` to obtain the time stats of one execution which is shown at the terminal and stored in a text file`(ExecTimeMean.txt)`.

# 9. Calibration 
You can find a tutorial for visual-inertial calibration and a detailed description of the contents of valid configuration files at `Calibration_Tutorial.pdf`

**Reference:** Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, [José M. M. Montiel](https://webdiis.unizar.es/~josemari/), [Juan D. Tardos](https://webdiis.unizar.es/~jdtardos/).

This repository contains a tutorial on implementing 3DAMCL-ORB-SLAM by Mr.Herusyahputra
