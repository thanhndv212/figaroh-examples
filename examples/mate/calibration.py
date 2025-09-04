# Copyright [2021-2025] Thanh Nguyen
# Copyright [2022-2023] [CNRS, Toward SAS]

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

# http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import os

# Add the parent directory to Python path to enable proper imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from figaroh.tools.robot import load_robot
from examples.mate.utils.mate_tools import MateCalibration
from pinocchio.visualize import GepettoVisualizer
import time

# load_by_urdf = False, load robot from rospy.get_param(/robot_description)
robot = load_robot("urdf/mate.urdf", package_dirs="models", load_by_urdf=True)

robot_calib = MateCalibration(robot, "config/mate.yaml", del_list=[])
robot_calib.calib_config["known_baseframe"] = False
robot_calib.calib_config["known_tipframe"] = False
# # load data file and determine parameters to be calibrated
robot_calib.load_data_set()

robot_calib.create_param_list()
# print(robot_calib.calib_config['param_name'])
robot_calib.solve()
robot_calib.plot()

# robot.setVisualizer(GepettoVisualizer())
# robot.initViewer(loadModel=True)

# gui = robot.viewer.gui

# gui.setFloatProperty("world/pinocchio/visuals", "Alpha", 1)
# gui.setBackgroundColor1("python-pinocchio", [1.0, 1, 1, 1])
# gui.setBackgroundColor2("python-pinocchio", [1.0, 1, 1, 1])

# for q in robot_calib.q_measured:
#     robot.display(q)
#     time.sleep(0.5)
