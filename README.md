# Phase Correlation Based Point Cloud Registration


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
  catkin build
```


