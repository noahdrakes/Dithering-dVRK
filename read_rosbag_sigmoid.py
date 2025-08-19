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

file_name = arg.file_name
joint_index = int(file_name[1])

measured_jp = []
measured_jv = []
measured_jf = []
gc_setpoint_jf = []
timestamps = []

bagpath = Path(file_name)
typestore = get_typestore(Stores.LATEST)

with AnyReader([bagpath], default_typestore=typestore) as reader:
    # collection of all available connections (topic)
    conns = reader.connections
    # selection of the desired ones
    js_conn = [c for c in conns if c.topic == '/PSM1/measured_js'][0]
    gc_conn = [c for c in conns if c.topic == '/PSM1/gravity_compensation/setpoint_js'][0]

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

# from list to numpy arrays
measured_jp = np.array(measured_jp)
measured_jv = np.array(measured_jv)
measured_jf = np.array(measured_jf)
gc_setpoint_jf = np.array(gc_setpoint_jf)
timestamps = np.array(timestamps)
timestamps = (timestamps - timestamps[0]) * 1e-9

# trim arrays to have the same length
min_len = min(len(measured_jf), len(gc_setpoint_jf))
t = timestamps[:min_len]
vel = measured_jv[:min_len, joint_index]
diff = measured_jf[:min_len, joint_index] - gc_setpoint_jf[:min_len, joint_index]

# PLOT
plt.figure()
plt.subplot(211)
plt.plot(t, measured_jp[:min_len, joint_index]*180/3.141592654, color='purple', alpha=1)
plt.ylabel(f'J{joint_index} position [deg]')
plt.suptitle('POSITION AND VELOCITY FOR CONSTANT ACCELERATION')
plt.grid()
plt.subplot(212)
plt.plot(t, measured_jv[:min_len, joint_index], color='green', alpha=1)
plt.ylabel(f'J{joint_index} velocity [rad/s]')
plt.xlabel('Time [s]')
plt.grid()

plt.figure()
plt.title(r'$\tau_{measured} - \tau_{gravity}$ vs time')
plt.plot(timestamps, diff)
plt.ylabel(f'J{joint_index} effort difference [Nm]')
plt.xlabel('Time [s]')
plt.grid()


plt.figure()
init = int(0.1 * min_len)
end = int(0.9 * min_len)

plt.plot(vel[init:end], diff[init:end], color='skyblue', alpha=1, label='raw')
vel_f = np.array(iir_filter(vel))
diff_f = np.array(iir_filter(diff))
plt.plot(vel_f[init:end], diff_f[init:end], marker='.', color='blue', alpha=1, label='filt')
plt.xlabel(f'J{joint_index} velocity [rad/s]')
plt.ylabel(f'J{joint_index} effort [Nm]')
plt.legend(loc='upper left')
plt.grid()
plt.title('Force vs Velocity')

plt.show()

#
# # ================== SIGMOID FITTING ==================
# def sigmoid_up(q_dot, tau_c_min, tau_c_max, A, B, C):
#     sigmoid_term = (tau_c_max - tau_c_min) / (1 + np.exp(-A * (q_dot + B)))
#     linear_term = C * q_dot
#     return tau_c_min + sigmoid_term + linear_term
#
# def sigmoid_down(q_dot, tau_c_min, tau_c_max, A, B, C):
#     sigmoid_term = (tau_c_max - tau_c_min) / (1 + np.exp(-A * (q_dot - B)))
#     linear_term = C * q_dot
#     return tau_c_min + sigmoid_term + linear_term
#
# start = 1900
# middle = 3450
# end = 5000
# tau_f_max_pos = sigmoid_up(vel_f[start:middle], tau_c_min=-1, tau_c_max=1.2, A=200, B=0.01, C=0.05)
# tau_f_max_neg = sigmoid_up(vel_f[middle:end], tau_c_min=-1, tau_c_max=1.2, A=100, B=0, C=0.05)
# tau_f_max = np.concatenate((tau_f_max_pos, tau_f_max_neg))
#
# tau_f_min_neg = sigmoid_down(vel_f[middle:end], tau_c_min=-1, tau_c_max=1.2, A=200, B=0.01, C=0.05)
# tau_f_min_pos = sigmoid_down(vel_f[start:middle], tau_c_min=-1, tau_c_max=1.2, A=100, B=0, C=0.05)
# tau_f_min = np.concatenate((tau_f_min_pos, tau_f_min_neg))
#
# plt.figure()
# plt.plot(vel, diff, color='skyblue', alpha=0.7, label='raw')
# plt.plot(vel_f, diff_f, color='blue', alpha=0.7, label='filt')
# # plt.plot(vel_f[start:end], tau_f_max, color='magenta', alpha=1, label='tau_f_max')
# # plt.plot(vel_f[start:end], tau_f_min, color='lime', alpha=1, label='tau_f_min')
#
# plt.xlabel(f'J{joint_index} velocity [rad/s]')
# plt.ylabel(f'J{joint_index} effort [Nm]')
# plt.legend()
# plt.grid()
# plt.title('COULOMB FRICTION BANDWIDTH ESTIMATION')
#
# plt.show()
#
