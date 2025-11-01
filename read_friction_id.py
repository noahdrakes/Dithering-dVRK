#!/usr/bin/env python3
"""
Joint Effort vs Velocity Analysis from ROS Bag
==============================================

This script reads a ROS bag file recorded from the da Vinci Research Kit (dVRK)
and plots the difference between the measured joint effort and the gravity 
compensation torque, as a function of joint velocity.

This visualization is useful to identify static friction and viscous friction 
regions in each joint, which are key for friction characterization.

Usage:
    python3 plot_effort_vs_velocity.py \
        -a PSM1 \
        -n data.bag \
        -j 0

Arguments:
    -a, --arm          Arm name (PSM1, PSM2, PSM3)
    -n, --file_name    Path to the ROS bag file
    -j, --joint_index  Joint index (0, 1, or 2)
"""

from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import argparse
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

# =========================== ARGUMENT PARSER ==============================
parser = argparse.ArgumentParser()
parser.add_argument('-a', '--arm', type = str, required = True,
                    choices=['PSM1', 'PSM2', 'PSM3'],
                    help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
parser.add_argument('-n', '--file_name', type=str, required=True,
                    help='path to the bag file')
parser.add_argument('-j', '--joint_index', type=int, required=True,
                    choices=[0, 1, 2],
                    help='index of the joint to analyze')

args = parser.parse_args()
file_name   = args.file_name
joint_index = args.joint_index
arm_name    = args.arm

# =========================== INITIALIZATION ===============================
measured_jp = []
measured_jv = []
measured_jf = []
gc_setpoint_jf = []
timestamps = []

bagpath = Path(file_name)
typestore = get_typestore(Stores.LATEST)

# =========================== READ BAG FILE ================================ 
with AnyReader([bagpath], default_typestore=typestore) as reader:
    conns = reader.connections

    # Select the desired ROS topics
    js_conn = [c for c in conns if c.topic == f'/{arm_name}/measured_js'][0]
    gc_conn = [c for c in conns if c.topic == f'/{arm_name}/gravity_compensation/setpoint_js'][0]

    # Read joint state messages
    for conn, timestamp, rawdata in reader.messages(connections=[js_conn]):
        msg = reader.deserialize(rawdata, conn.msgtype)
        measured_jp.append(msg.position)
        measured_jv.append(msg.velocity)
        measured_jf.append(msg.effort)
        timestamps.append(timestamp)

    # Read gravity compensation setpoints
    for conn, _, rawdata in reader.messages(connections=[gc_conn]):
        msg = reader.deserialize(rawdata, conn.msgtype)
        gc_setpoint_jf.append(msg.effort)


# =========================== DATA PROCESSING ============================== #
# Convert lists to numpy arrays
measured_jp    = np.array(measured_jp)
measured_jv    = np.array(measured_jv)
measured_jf    = np.array(measured_jf)
gc_setpoint_jf = np.array(gc_setpoint_jf)
timestamps     = np.array(timestamps)

# Convert timestamps from ns to seconds
timestamps = (timestamps - timestamps[0]) * 1e-9

# Align lengths
min_len        = min(len(measured_jf), len(gc_setpoint_jf))
measured_jv    = measured_jv[:min_len]
measured_jf    = measured_jf[:min_len]
gc_setpoint_jf = gc_setpoint_jf[:min_len]
timestamps     = timestamps[:min_len]

# Compute difference between measured torque and gravity compensation
tau_diff = measured_jf[:, joint_index] - gc_setpoint_jf[:, joint_index]
vel = measured_jv[:, joint_index]

# Trim 20% at start and end
init = int(0.20 * min_len)
end = int(0.80 * min_len)

# =========================== PLOT RESULTS =================================
plt.figure()
plt.plot(vel[init:end], tau_diff[init:end], color='purple', alpha=1, label='filt')
plt.xlabel(f'J{joint_index} velocity [rad/s] or [m/s]')
plt.ylabel(f'J{joint_index} effort [Nm] or [N]\n' + r' $\tau_{measured} - \tau_{gravity}$')
plt.grid()
plt.title('Effort vs Velocity')

plt.show()
