#!/bin/bash

# 1. Mergi la rădăcina workspace-ului
cd ~/cbrn_ws

source /opt/ros/jazzy/setup.bash

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/cbrn_ws/install/sim_env/share/sim_env/models

# 2. ȘTERGE build-ul și instalarea veche
rm -rf build install log

# 3. COMPILEAZĂ (pentru a copia .sdf-ul modificat)
colcon build

# 4. ACTIVEAZĂ noul build
source install/setup.bash

# 5. RULEAZĂ
ros2 launch sim_env start_sim.launch.py