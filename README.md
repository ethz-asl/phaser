# PHASER: A Robust and Correspondence-Free Global Pointcloud Registration

## Overview
Point cloud registration using correspondences is inefficient and prone to errors in the many steps of correspondence extraction, description, and matching.
Similarly, the most widespread registration methods work only locally, requiring an initial guess already close to the true solution, something unaffordable in real robotic deployments.
We propose an algorithm for the registration of partially overlapping point clouds that operates at the global level and on the raw data, i.e., no initial guess as well as no candidate matches are required.
We exploit the properties of Fourier analysis to derive a novel registration pipeline based on the cross-correlation of the phases.

## Installation

### Prerequisites

```
  # Standard maplab requirements
  sudo apt-get install -y doxygen liblapack-dev libblas-dev autotools-dev \
     dh-autoreconf libboost-all-dev python-setuptools git g++ cppcheck \
     default-jre libreadline-dev libgtest-dev libglew-dev python-git pylint \
     checkstyle python-termcolor liblog4cplus-dev cimg-dev python-wstool \
     python-catkin-tools libssh2-1-dev libatlas3-base libv4l-dev python-scipy \


   # Ubuntu 18.04 / ROS Melodic.
   sudo apt-get install -y clang-format-6.0 ros-melodic-pcl-conversions \
     libpcl-dev ros-melodic-octomap libvtk6-dev libvtk6-qt-dev libvtk6-java \
     libvtk6-jni libnlopt-dev
```

Within the caktin workspace

```
  wstool init
  wstool merge phaser/dependencies.rosinstall
  wstool update
```

Building the project:

```
  catkin build phaser_ros
```

Optionally one can build an run all unit tests using:

```
  ./phaser_share/run_build_tests
```
However, this might take some minutes to finish.

### Example

The package `phaser_core` provides a simple test driver to run PHASER using two pointclouds stored as `.ply` files.
Additionally, run script for the test driver is provided in the `phaser_share` directory.

The initial alignment of the two pointclouds is as follows:
![PHASER Input Example](./phaser_share/img/input.png "Input source and target pointcloud")

By running
```
./phaser_share/run_phaser_core_driver
```
the registered pointcloud is written to disk:
![PHASER Registered Example](./phaser_share/img/registered.png "Globally registered pointcloud")

## Reference

Our paper is available at  
*Bernreiter, Lukas, Lionel Ott, Juan Nieto, Roland Siegwart, and Cesar Cadena.
"PHASER: A Robust and Correspondence-Free Global Pointcloud Registration."
IEEE Robotics and Automation Letters 6, no. 2 (2021): 855-862.*  [[Link](https://ieeexplore.ieee.org/document/9327458)] [[ArXiv](https://arxiv.org/abs/2102.02767)].

BibTex:
```
@article{bernreiter2021phaser,
  title={PHASER: A Robust and Correspondence-Free Global Pointcloud Registration},
  author={Bernreiter, Lukas and Ott, Lionel and Nieto, Juan and Siegwart, Roland and Cadena, Cesar},
  journal={IEEE Robotics and Automation Letters},
  volume={6},
  number={2},
  pages={855--862},
  year={2021},
  publisher={IEEE}
}
```
