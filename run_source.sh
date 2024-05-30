#!/bin/bash

source /opt/ros/foxy/setup.bash

source ~/p3.8-env/bin/activate

source ~/SWU_Project/install/local_setup.bash

export ROS_PYTHON_VERSION=3
export PYTHON_EXECUTABLE=/home/ubuntu/p3.8-env/bin/python
export PYTHONPATH=$PYTHONPATH:/home/ubuntu/p3.8-env/lib/python3.8/site-packages

