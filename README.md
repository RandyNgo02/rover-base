# rover-base
 Base Station for Rover

## Set up Base Station
Full Desktop Install of ROS2 Foxy

After installing ROS2, open a new terminal and enter
        ```
        source /opt/ros/foxy/setup.bash
        cd ~/rover-base/
        colcon build
        ```
To create new packages:
        ```
        ros2 pkg create --build-type ament_python --node-name teleop_node teleop
        ```
## Set up the Orin for a run
Open a new terminal/tab Shell A (ROS 1 only):
        ```
        ssh orin@192.168.0.xxx
        source /opt/ros/noetic/setup.bash
        roscore
        ```
Open a new terminal/tab Shell B (ROS 1 + ROS 2):
        ```
        ssh orin@192.168.0.xxx
        # Source ROS 1 first:
        source /opt/ros/noetic/setup.bash
        # Source ROS 2 next:
        # source /opt/ros/foxy/setup.bash
        source /autonomy/install/setup.bash
        export ROS_MASTER_URI=http://localhost:11311
        ros2 run ros1_bridge dynamic_bridge
        ```
Open a new terminal/tab Shell C (ROS 1 Arduino Bridge):
        ```
        ssh orin@192.168.0.xxx
        # Source ROS 1 first:
        source /opt/ros/noetic/setup.bash
        # Arduino might be a different number than 0 for /dev/ttyACM0
        rosrun rosserial_python serial_node.py /dev/ttyACM0
        ```
Open a new terminal/tab Shell D (ROS 2):
        ```
        ssh orin@192.168.0.xxx
        # Source ROS 2:
        # source /opt/ros/foxy/setup.bash
        source /autonomy/install/setup.bash
        export ROS_MASTER_URI=http://localhost:11311
        ros2 launch launch.py
        ```

## Using the base station
Open a new terminal
        ```
        source install/setup.bash
        ros2 launch launch.py
        ```

