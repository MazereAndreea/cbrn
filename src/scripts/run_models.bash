# 1. Asigură-te că ROS și Venv sunt active
source /opt/ros/jazzy/setup.bash
source ~/venvs/python_3.12/bin/activate
source ~/cbrn_ws/install/setup.bash

# 2. Rulează scriptul sursă direct (nu pe cel din install)
python3 ~/cbrn_ws/src/cbrn_perception/cbrn_perception/vitpose_detector.py