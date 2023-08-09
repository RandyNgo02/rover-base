# rover-base
 Base Station for Rover

## Set up Orin
Full Desktop Install of [ROS1 Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
Full Desktop Install of ROS2 Foxy

After installing both ROS1 and ROS2, open a new terminal and enter
      ```
      sudo apt install ros-foxy-ros1-bridge
      source /opt/ros/foxy/setup.bash
      cd ~/autonomy/
      colcon build
      ```
## Using the base station
Open a new terminal
      ```
      cd ~/autonomy/
      source install/setup.bash
      ros2 launch launch.py
      ```