#!/bin/bash
if [ -f install/setup.bash ]; then source install/setup.bash; fi
ros2 launch launch.py
