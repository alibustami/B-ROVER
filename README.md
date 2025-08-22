# B-ROVER

```
pip install --upgrade pip
pip insatll --upgade setuptools wheel
pip install pre-commit black isort flake8
sudo apt install ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-urdfdom-py

# submodules
git submodule add -b release/1.14 https://github.com/PX4/px4_msgs.git dev_ws/src/px4_msgs
git submodule add https://github.com/YDLIDAR/ydlidar_ros2_driver.git dev_ws/src/ydlidar_ros2_driver && git -C dev_ws/src/ydlidar_ros2_driver fetch --all && git -C dev_ws/src/ydlidar_ros2_driver checkout 91e86db
git submodule add https://github.com/jomidokunMain/ackdrive_px4_mrover.git dev_ws/src/ackdrive_px4_mrover
git submodule update --init --recursive --progress
```
