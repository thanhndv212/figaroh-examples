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

import os
from os.path import dirname, join, abspath

import pinocchio as pin
from pinocchio.utils import *

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from scipy.optimize import least_squares
import numpy as np

import time
import yaml
from yaml.loader import SafeLoader
import pprint

from figaroh.tools.robot import Robot
from figaroh.calibration.calibration_tools import (
    get_param_from_yaml,
    add_pee_name,
    load_data,
    calculate_base_kinematics_regressor,
    calc_updated_fkm,
    initialize_variables,
)
from figaroh.tools.robot import load_robot

# 1/ Load robot model and create a dictionary containing reserved constants
robot = load_robot(
    "urdf/ur10_robot.urdf",
    package_dirs="models",
    load_by_urdf=True,
)
model = robot.model
data = robot.data

with open("config/ur10_config.yaml", "r") as f:
    config = yaml.load(f, Loader=SafeLoader)
    pprint.pprint(config)
calib_data = config["calibration"]
calib_config = get_param_from_yaml(robot, calib_data)

#############################################################

# 2/ Base parameters calculation
q_rand = []
Rrand_b, R_b, R_e, paramsrand_base, paramsrand_e = calculate_base_kinematics_regressor(
    q_rand, model, data, calib_config
)

# add markers name to calib_config['param_name']
add_pee_name(calib_config)

# total calibrating parameter names
for i, pn in enumerate(calib_config["param_name"]):
    print(i, pn)

#############################################################

# 3/ Data collection/generation
dataSet = "experimental"  # choose data source 'sample' or 'experimental'
if dataSet == "sample":
    # create artificial offsets
    var_sample, nvars_sample = initialize_variables(calib_config, mode=1)

    print("%d var_sample: " % nvars_sample, var_sample)

    # create sample configurations
    q_sample = np.empty((calib_config["NbSample"], model.nq))

    for i in range(calib_config["NbSample"]):
        config = calib_config["q0"]
        config[calib_config["config_idx"]] = pin.randomConfiguration(model)[
            calib_config["config_idx"]
        ]
        q_sample[i, :] = config

    # create simulated data
    PEEm_sample = calc_updated_fkm(model, data, var_sample, q_sample, calib_config)

    q_LM = np.copy(q_sample)
    PEEm_LM = np.copy(PEEm_sample)

elif dataSet == "experimental":
    # load experimental data
    path = abspath("data/calibration.csv")

    PEEm_exp, q_exp = load_data(path, model, calib_config)

    q_LM = np.copy(q_exp)
    PEEm_LM = np.copy(PEEm_exp)

# Remove potential outliers which identified from previous calibration

print(np.shape(PEEm_LM))

print("updated number of samples: ", calib_config["NbSample"])

#############################################################

# 4/ Given a model and configurations (input), end effector positions/locations
# (output), solve an optimization problem to find offset params which are set as variables

# # NON-LINEAR model with Levenberg-Marquardt #################
# minimize the difference between measured coordinates of end-effector
# and its estimated values

coeff = 1e-3  # coefficient that regulates parameters


def cost_func(var, coeff, q, model, data, calib_config, PEEm):
    PEEe = calc_updated_fkm(model, data, var, q, calib_config)
    res_vect = np.append(
        (PEEm - PEEe),
        np.sqrt(coeff) * var[6 : -calib_config["NbMarkers"] * calib_config["calibration_index"]],
    )
    # res_vect = (PEEm - PEEe)
    return res_vect


# initial guess
# mode = 1: random seed [-0.01, 0.01], mode = 0: init guess = 0
var_0, nvars = initialize_variables(calib_config, mode=0)
# Write reference pose of camera in initial guess
var_0[-6:] = np.array(
    [-0.000, -0.118, -0.011, -5.66796100e-01, -5.36932045e-04, 8.36004767e-03]
)
print("initial guess: ", var_0)

# solve
LM_solve = least_squares(
    cost_func,
    var_0,
    method="lm",
    verbose=1,
    args=(coeff, q_LM, model, data, calib_config, PEEm_LM),
)

#############################################################

# 5/ Result analysis
res = LM_solve.x
# PEE estimated by solution
PEEe_sol = calc_updated_fkm(model, data, res, q_LM, calib_config)
# root mean square error
rmse_pos = np.sqrt(
    np.mean((PEEe_sol[: 3 * calib_config["NbSample"]] - PEEm_LM[: 3 * calib_config["NbSample"]]) ** 2)
)
rmse_ori = np.sqrt(
    np.mean((PEEe_sol[3 * calib_config["NbSample"] :] - PEEm_LM[3 * calib_config["NbSample"] :]) ** 2)
)

print("solution: ", res)
print("root mean square error of end-effector position(meters): ", rmse_pos)
print("root mean square error of end-effector orientation(radian): ", rmse_ori)
print("optimality: ", LM_solve.optimality)

# calculate standard deviation of estimated parameter ( Khalil chapter 11)
sigma_ro_sq = (LM_solve.cost**2) / (
    calib_config["NbSample"] * calib_config["calibration_index"] - nvars
)
J = LM_solve.jac
C_param = sigma_ro_sq * np.linalg.pinv(np.dot(J.T, J))
std_dev = []
std_pctg = []
for i in range(nvars):
    std_dev.append(np.sqrt(C_param[i, i]))
    std_pctg.append(abs(np.sqrt(C_param[i, i]) / res[i]))
print("standard deviation: ", std_dev)


#############################################################

# 6/ Plot results

# calculate difference between estimated data and measured data
delta_PEE = PEEe_sol - PEEm_LM
PEE_xyz = delta_PEE.reshape(
    (calib_config["NbMarkers"] * calib_config["calibration_index"], calib_config["NbSample"])
)
PEE_dist = np.zeros((calib_config["NbMarkers"], calib_config["NbSample"]))
for i in range(calib_config["NbMarkers"]):
    for j in range(calib_config["NbSample"]):
        PEE_dist[i, j] = np.sqrt(
            PEE_xyz[i * 3, j] ** 2
            + PEE_xyz[i * 3 + 1, j] ** 2
            + PEE_xyz[i * 3 + 2, j] ** 2
        )

# detect "bad" data (outlierrs) => remove outliers, recalibrate
del_list = []
scatter_size = np.zeros_like(PEE_dist)
for i in range(calib_config["NbMarkers"]):
    for k in range(calib_config["NbSample"]):
        if PEE_dist[i, k] > 0.02:
            del_list.append((i, k))
    scatter_size[i, :] = 20 * PEE_dist[i, :] / np.min(PEE_dist[i, :])
print("indices of samples with >2 cm deviation: ", del_list)

# # 1// Errors between estimated position and measured position of markers

fig1, ax1 = plt.subplots(calib_config["NbMarkers"], 1)
# fig1.suptitle(
# "Relative positional errors between estimated markers and measured markers (m) by samples ")
colors = ["blue", "red", "yellow", "purple"]
if calib_config["NbMarkers"] == 1:
    ax1.bar(np.arange(calib_config["NbSample"]), PEE_dist[i, :])
    ax1.set_xlabel("Sample", fontsize=25)
    ax1.set_ylabel("Error (meter)", fontsize=30)
    ax1.tick_params(axis="both", labelsize=30)
    ax1.grid()
else:
    for i in range(calib_config["NbMarkers"]):
        ax1[i].bar(np.arange(calib_config["NbSample"]), PEE_dist[i, :], color=colors[i])
        ax1[i].set_xlabel("Sample", fontsize=25)
        ax1[i].set_ylabel("Error of marker %s (meter)" % (i + 1), fontsize=25)
        ax1[i].tick_params(axis="both", labelsize=30)
        ax1[i].grid()

# # 2// plot 3D measured poses and estimated
fig2 = plt.figure(2)
fig2.suptitle("Visualization of estimated poses and measured pose in Cartesian")
ax2 = fig2.add_subplot(111, projection="3d")
PEEm_LM2d = PEEm_LM.reshape(
    (calib_config["NbMarkers"] * calib_config["calibration_index"], calib_config["NbSample"])
)
PEEe_sol2d = PEEe_sol.reshape(
    (calib_config["NbMarkers"] * calib_config["calibration_index"], calib_config["NbSample"])
)
for i in range(calib_config["NbMarkers"]):
    ax2.scatter3D(
        PEEm_LM2d[i * 3, :],
        PEEm_LM2d[i * 3 + 1, :],
        PEEm_LM2d[i * 3 + 2, :],
        marker="^",
        color="red",
        label="measured",
    )
    ax2.scatter3D(
        PEEe_sol2d[i * 3, :],
        PEEe_sol2d[i * 3 + 1, :],
        PEEe_sol2d[i * 3 + 2, :],
        marker="o",
        color="green",
        label="estimated",
    )
ax2.set_xlabel("X - front (meter)")
ax2.set_ylabel("Y - side (meter)")
ax2.set_zlabel("Z - height (meter)")
ax2.grid()
ax2.legend()

# 3// visualize relative deviation between measure and estimate
fig3 = plt.figure(3)
ax3 = fig3.add_subplot(111, projection="3d")
for i in range(calib_config["NbMarkers"]):
    ax3.scatter3D(
        PEEm_LM2d[i * 3, :],
        PEEm_LM2d[i * 3 + 1, :],
        PEEm_LM2d[i * 3 + 2, :],
        s=scatter_size[i, :],
        color="blue",
    )
ax3.set_xlabel("X - front (meter)")
ax3.set_ylabel("Y - side (meter)")
ax3.set_zlabel("Z - height (meter)")
ax3.grid()

# 4// joint configurations within range bound
fig4 = plt.figure()
fig4.suptitle("Joint configurations with joint bounds")
ax4 = fig4.add_subplot(111, projection="3d")
lb = ub = []
for j in calib_config["config_idx"]:
    # model.names does not accept index type of numpy int64
    # and model.lowerPositionLimit index lag to model.names by 1
    lb = np.append(lb, model.lowerPositionLimit[j])
    ub = np.append(ub, model.upperPositionLimit[j])
q_actJoint = q_LM[:, calib_config["config_idx"]]
sample_range = np.arange(calib_config["NbSample"])
for i in range(len(calib_config["actJoint_idx"])):
    ax4.scatter3D(q_actJoint[:, i], sample_range, i)
for i in range(len(calib_config["actJoint_idx"])):
    ax4.plot([lb[i], ub[i]], [sample_range[0], sample_range[0]], [i, i])
ax4.set_xlabel("Angle (rad)")
ax4.set_ylabel("Sample")
ax4.set_zlabel("Joint")
ax4.grid()

# identified parameters
if dataSet == "sample":
    plt.figure(5)
    print(len(calib_config["param_name"]), res.shape)
    plt.barh(calib_config["param_name"], res, align="center")
    plt.grid()
elif dataSet == "experimental":
    plt.figure(5)
    plt.barh(calib_config["param_name"][0:6], res[0:6], align="center", color="blue")
    plt.grid()
    plt.figure(6)
    plt.barh(
        calib_config["param_name"][6 : -calib_config["calibration_index"] * calib_config["NbMarkers"]],
        res[6 : -calib_config["calibration_index"] * calib_config["NbMarkers"]],
        align="center",
        color="orange",
    )
    plt.grid()
    plt.figure(7)
    plt.barh(
        calib_config["param_name"][-calib_config["calibration_index"] * calib_config["NbMarkers"] :],
        res[-calib_config["calibration_index"] * calib_config["NbMarkers"] :],
        align="center",
        color="green",
    )
    plt.grid()

## display few configurations
# viz = MeshcatVisualizer(
#     model=robot.model, collision_model=robot.collision_model,
#     visual_model=robot.visual_model, url='classical'
# )
# time.sleep(1)
# for i in range(calib_config['NbSample']):
#     viz.display(q_LM[i, :])
#     time.sleep(1)

plt.show()

############################################################
