#!/usr/bin/env python3

import rospy 
from eeg_signal_acquisition.srv import eeg_block_srv
import numpy as np
import sys, os
from os.path import expanduser
home = expanduser("~")
sys.path.insert(2, f"{home}/catkin_ws/src/neurocontroller_database/src")
from utilities import root_dir, load_json, save_json, has_attr, user_protocol_aqd, models_list


# ruta del nodo neurocontroller_database
split_path = root_dir.split("/")[:-1]
path_node = "/".join(split_path)

# ruta de los datos
root_db = f"{path_node}/database"