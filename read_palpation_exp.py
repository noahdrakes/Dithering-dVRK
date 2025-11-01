#!/usr/bin/env python3
"""
Force Estimation Validation using ROS Bag Data
==============================================

Compares estimated Cartesian force (from joint torques and Jacobian) against
measured force sensor data. Uses a free-space baseline to remove internal effects.

Usage:
    python read_weights_exp.py \
        -a PSM1 \
        -j 0 \
        -n contact.bag \
        -b baseline.bag \
        -w 3

Arguments:
    -a, --arm                Arm name (PSM1, PSM2, PSM3)
    -j, --joint_index        Joint index (0, 1, or 2)
    -n, --file_name          Path to bag file recorded during contact
    -b, --baseline_file_name Path to baseline bag (no contact)
    -w, --weight             Applied external force [N]
"""

from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import argparse
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from sklearn.metrics import mean_absolute_error
import scipy

# =========================== FILTERING FUNCTION ===========================
def low_pass(data, fs, cutoff, order):
    """Butterworth high pass filtering of a signal"""
    nyq = 0.5 * fs
    f_cut = cutoff / nyq
    b, a = scipy.signal.butter(order, f_cut, btype='low', analog=False)
    return scipy.signal.filtfilt(b, a, data)

# =========================== ARGUMENT PARSER ==============================
parser = argparse.ArgumentParser()
parser.add_argument('-a', '--arm', type = str, required = True,
                    choices=['PSM1', 'PSM2', 'PSM3'],
                    help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
parser.add_argument('-j', '--joint_index', type=int, required=True,
                    choices=[0, 1, 2],
                    help='index of the joint to read')
parser.add_argument('-n', '--file_name', type=str, required=True,
                    help='path to the bag file')
parser.add_argument('-b', '--baseline_file_name', type=str, required=True,
                    help='rosbag record without external forces -> used to compute baseline signal')
parser.add_argument('-w', '--weight', type=float, required=True,
                    help='force (weight) applied in N')

args = parser.parse_args()
joint_index = args.joint_index

# =========================== READ CONTACT BAG ============================ 
measured_jf, jacobian, t_meas = [], [], []
sensor_cf, t_sensor = [], []

bagpath = Path(args.file_name)
typestore = get_typestore(Stores.LATEST)

with AnyReader([bagpath], default_typestore=typestore) as reader:
    conns = reader.connections
    js_conn = [c for c in conns if c.topic == f'/{args.arm}/measured_js'][0]
    jacobian_conn = [c for c in conns if c.topic == f'/{args.arm}/spatial/jacobian'][0]
    fs_conn = [c for c in conns if c.topic == '/measured_cf'][0]

    for conn, t, rawdata in reader.messages(connections=[js_conn]):
        msg = reader.deserialize(rawdata, conn.msgtype)
        measured_jf.append(msg.effort)
        t_meas.append(t)

    for conn, _, rawdata in reader.messages(connections=[jacobian_conn]):
        msg = reader.deserialize(rawdata, conn.msgtype)
        jacobian.append(msg.data)

    for conn, t, rawdata in reader.messages(connections=[fs_conn]):
        msg = reader.deserialize(rawdata, conn.msgtype)
        sensor_cf.append(msg.wrench.force.z)
        t_sensor.append(t)


# =========================== READ BASELINE BAG ===========================
free_jf, t_free = [], []

bagpath = Path(args.baseline_file_name)
typestore = get_typestore(Stores.LATEST)

with AnyReader([bagpath], default_typestore=typestore) as reader:
    conns = reader.connections

    js_conn = [c for c in conns if c.topic == f'/{args.arm}/measured_js'][0]

    for conn, t, rawdata in reader.messages(connections=[js_conn]):
        msg = reader.deserialize(rawdata, conn.msgtype)
        free_jf.append(msg.effort)
        t_free.append(t)


# =========================== DATA PROCESSING =============================
measured_jf = np.array(measured_jf)
free_jf     = np.array(free_jf)
jacobian    = np.array(jacobian).reshape((len(jacobian), 6, 6))
sensor_cf   = np.array(sensor_cf)

t_meas   = (np.array(t_meas) - t_meas[0]) * 1e-9
t_sensor = (np.array(t_sensor) - t_sensor[0]) * 1e-9
t_free   = (np.array(t_free) - t_free[0]) * 1e-9
fs_meas  = 1 / np.mean(np.diff(t_meas))
fs_free  = 1 / np.mean(np.diff(t_free))

# Optionally align time series (if small offset)
delta_t = 0  # [s]
if delta_t > 0:
    aligned_free = np.zeros(int(len(free_jf) + delta_t * fs_free))
    aligned_free[-len(free_jf):] = free_jf
else:
    aligned_free = free_jf[int(abs(delta_t) * fs_free):]


# Trim arrays to common length
min_len = min(len(aligned_free), len(measured_jf), len(jacobian))

measured_jf   = measured_jf[:min_len]
free_jf       = aligned_free[:min_len]
jacobian      = jacobian[:min_len]
t_meas        = t_meas[:min_len]

# Filter low-frequency components
measured_mean = low_pass(measured_jf[:, joint_index], fs_meas, 3, 4)
free_mean = low_pass(free_jf[:, joint_index], fs_free, 3, 4)

# =========================== FORCE ESTIMATION ============================
tau_ext                 = measured_jf - free_jf
tau_ext[:, joint_index] = measured_mean - free_mean

estimated_cf     = np.zeros((min_len, 6))

for i in range(min_len):
    J_inv = np.linalg.inv(jacobian[i, :, :])
    estimated_cf[i, :]     = J_inv.T @ tau_ext[i, :]

# select the direction of application of the force
estimated_cf = estimated_cf[:, 2]


# =========================== VISUALIZATION ===============================
fig, axs = plt.subplots(1, 2)
axs[0].plot(t_meas, measured_jf[:, joint_index], alpha=0.7, label="contact")
axs[0].plot(t_meas, free_jf[:, joint_index], alpha=0.7, label="free space")
axs[0].set_ylabel("Force [N]")
axs[0].set_xlabel("Time [s]")
axs[0].set_title("MEASURED FORCE")
axs[0].legend()
axs[0].grid()
axs[1].plot(t_sensor, np.abs(sensor_cf), color="green", label="sensor")
axs[1].plot(t_meas, estimated_cf, color="purple", label="estimated")
axs[1].set_ylabel("Force [N]")
axs[1].set_xlabel("Time [s]")
axs[1].set_title("EXTERNAL FORCE")
axs[1].legend()
axs[1].grid()
plt.show()


# ================================ MAE ==============================

# Interpolate sensor measurements for comparison
sensor_cf_int = np.interp(t_meas, t_sensor, sensor_cf)

# Define contact interval manually
contact_mask = (t_meas > 8) & (t_meas < 15)
mae = mean_absolute_error(sensor_cf_int[contact_mask], estimated_cf[contact_mask])
print(f"Mean Absolute Error (contact interval): {mae:.3f} N")
