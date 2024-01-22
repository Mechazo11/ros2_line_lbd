# ros2_line_lbd

ROS2 library-only port of Dr. Yang`s ROS1 **line_lbd** package from this CubeSLAM project. Useful if you plan on using Dr. Yang's method of 3d object detection using vanishining points to solve for cuboids.


@author: Azmyin Md. Kamal

@job: Ph.D. student, [iCORE Lab](https://icorelab.github.io/), Department of MIE, Louisiana State University, Louisiana, USA

@date: 01/18/2024


## 0. Tested with
1. Boost 1.74 and Boost 1.84
2. Ubuntu 22.04 LTS (Jammy Jellyfish)
3. RO2 Humble Hawksbill (LTS)

## 1. Prerequisitis
* Boost 1.74 or later
* OpenCV >=4.2
* Eigen >=3.3

### 2. Installation
```
cd ~ # move to /home directory
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_test/src
cd ~/ros2_test/src
git clone https://github.com/Mechazo11/ros2_line_lbd.git
cd .. 
colcon build --packages-select ros2_line_lbd
source install/setup.bash
```
### 3. How to use?
* Add in CmakeLists.txt
```
find_package(ros2_line_lbd REQUIRED) # Dr. Yang`s line_lbd library built as a standalone ros2 package
```

* Add dependency in packages.xml
```
<build_depend>ros2_line_lbd</build_depend>
<exec_depend>ros2_line_lbd</exec_depend>
```
To use this library, follow examples in CubeSLAM's [detect_3d_cuboid](https://github.com/shichaoy/cube_slam/tree/master/detect_3d_cuboid) package. Another useage example can be found here [ros2_monocular_object_detection](https://github.com/Mechazo11/ros2_monocular_object_detection)
