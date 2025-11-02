from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import argparse

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def iir_filter(x):
    y = [0] * len(x)
    for n in range(len(x)):
        y[n] = 0.4 * x[n] + (0.6 * y[n - 1] if n > 0 else 0)
    return y

# ============ PARSER ============
parser = argparse.ArgumentParser()
parser.add_argument('-n', '--file_name', type=str, required=True)
arg = parser.parse_args()


# ============ ROSBAG READING ============
file_name = arg.file_name
split = file_name.split('-')
joint_index = int(split[1])
frequency = float(split[2])
amplitude = float(split[3])

measured_jp = []
measured_jv = []
measured_jf = []
gc_setpoint_jf = []
jacobian = []
timestamps = []

bagpath = Path(file_name)
typestore = get_typestore(Stores.LATEST)

with AnyReader([bagpath], default_typestore=typestore) as reader:
    # collection of all available connections (topic)
    conns = reader.connections
    # selection of the desired ones
    js_conn = [c for c in conns if c.topic == '/PSM1/measured_js'][0]
    gc_conn = [c for c in conns if c.topic == '/PSM1/gravity_compensation/setpoint_js'][0]
    jacobian_conn = [c for c in conns if c.topic == '/PSM1/spatial/jacobian'][0]

    # read messages
    for conn, timestamp, rawdata in reader.messages(connections=[js_conn]):
        msg = reader.deserialize(rawdata, conn.msgtype)
        measured_jp.append(msg.position)
        measured_jv.append(msg.velocity)
        measured_jf.append(msg.effort)
        timestamps.append(timestamp)

    for conn, _, rawdata in reader.messages(connections=[gc_conn]):
        msg = reader.deserialize(rawdata, conn.msgtype)
        gc_setpoint_jf.append(msg.effort)

    for conn, _, rawdata in reader.messages(connections=[jacobian_conn]):
        msg = reader.deserialize(rawdata, conn.msgtype)
        jacobian.append(msg.data)

# from lists to numpy arrays
measured_jp = np.array(measured_jp)
measured_jv = np.array(measured_jv)
measured_jf = np.array(measured_jf)
gc_setpoint_jf = np.array(gc_setpoint_jf)
jacobian = np.array(jacobian).reshape((len(jacobian), 6, 6))
timestamps = np.array(timestamps)
timestamps = (timestamps - timestamps[0]) * 1e-9

min_len = min(len(measured_jf), len(gc_setpoint_jf), len(jacobian))
t = np.linspace(0, min_len, min_len) * 0.001

# plt.figure()
# plt.subplot(311)
# plt.plot(t, measured_jp[:min_len, joint_index], color='purple', alpha=1)
# plt.ylabel('J{} position [rad]'.format(joint_index))
# plt.suptitle('POSITION, VELOCITY AND EFFORT')
# plt.grid()
# plt.subplot(312)
# plt.plot(t, measured_jv[:min_len, joint_index], color='green', alpha=1)
# plt.ylabel('J{} velocity [rad/s]'.format(joint_index))
# plt.grid()
# plt.subplot(313)
# plt.plot(t, measured_jf[:min_len, joint_index], label='effort')
# plt.plot(t, gc_setpoint_jf[:min_len, joint_index], label='gravity')
# plt.ylabel('J{} effort [Nm]'.format(joint_index))
# plt.xlabel('Time [s]')
# plt.legend()
# plt.grid()


########################################################################################################################
################################################ DITHERING SIGNAL MEAN #################################################
########################################################################################################################
fs = 1000
T_dithering = 1/frequency
n_period = 10

window = int(n_period * T_dithering * fs)
measured_jf_mean = np.zeros_like(measured_jf[:, joint_index])       # mean of the signal
measured_jf_up = np.zeros_like(measured_jf_mean)                    # upper envelope
measured_jf_down = np.zeros_like(measured_jf_mean)                  # lower envelope
for i in range(len(measured_jf)-window):
    measured_jf_mean[i+window] = np.mean(measured_jf[i:i+window+1, joint_index])
    measured_jf_up[i+window] = np.mean(measured_jf[i:i+window+1, joint_index][measured_jf[i:i+window+1, joint_index]
                                                                              > np.mean(measured_jf_mean[i:i+window+1])])
    measured_jf_down[i+window] = np.mean(measured_jf[i:i+window+1, joint_index][measured_jf[i:i+window+1, joint_index]
                                                                                < np.mean(measured_jf_mean[i:i+window+1])])

measured_torque = np.mean([measured_jf_up, measured_jf_down], axis=0)   # final mean -> very similar to previous mean

plt.figure()
plt.plot(t, measured_jf[:min_len, joint_index], label='effort', alpha=0.5)
plt.plot(t, measured_jf_mean[:min_len], label='effort mean', alpha=0.5)
plt.plot(t, gc_setpoint_jf[:min_len, joint_index], label='gravity')
plt.plot(t, measured_jf_up[:min_len], label='effort up')
plt.plot(t, measured_jf_down[:min_len], label='effort down')
plt.plot(t, measured_torque[:min_len], label='measured_torque')
plt.ylabel('J{} effort [Nm]'.format(joint_index))
plt.xlabel('Time [s]')
plt.legend()
plt.grid()

# plt.show()

########################################################################################################################
################################################### FORCE ESTIMATION ###################################################
########################################################################################################################

# compute free-space torque from data before the first external force
free_space_torque = np.mean(measured_torque[int(0.2*min_len):int(0.3*min_len)] -
                            gc_setpoint_jf[int(0.2*min_len):int(0.3*min_len), joint_index])
measured_torque = measured_torque[:min_len] - free_space_torque
measured_jf[:min_len, joint_index] = measured_torque        # substitution of the corrected mean in measured_jf

# fig, axs = plt.subplots(6, 1)
# for i in range(6):
#     axs[i].plot(measured_jf[:min_len, i] - gc_setpoint_jf[:min_len, i])
# plt.show()


# last three joints really noisy -> keep just the first three also because the external force is applied before
#                                   the fourth joint

jacobian = jacobian[:min_len, :6, :3]
tau_measured = measured_jf[:min_len, :3]
tau_gravity = gc_setpoint_jf[:min_len, :3]


force = np.zeros((min_len, 6))

for i in range(min_len):
    force[i, :] = np.linalg.pinv(jacobian[i, :, :]).transpose() @ (tau_measured[i, :] - tau_gravity[i, :])

axis = ['$F_x$', '$F_y$', '$F_z$', r'$\tau_x$', r'$\tau_y$', r'$\tau_z$']
fig, axs = plt.subplots(6, 1, sharex=True)
for i in range(6):
    axs[i].plot(t, force[:, i])
    axs[i].set_ylabel(axis[i])

axs[5].set_xlabel('Time')
plt.show()