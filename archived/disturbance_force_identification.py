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
joint_index = 0

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

#
# # PLOT
# plt.figure()
# plt.subplot(211)
# plt.plot(t, measured_jp[:min_len, joint_index]*180/3.141592654, color='purple', alpha=1)
# plt.ylabel(f'J{joint_index} position [deg]')
# plt.suptitle('POSITION AND VELOCITY FOR CONSTANT ACCELERATION')
# plt.grid()
# plt.subplot(212)
# plt.plot(t, measured_jv[:min_len, joint_index], color='green', alpha=1)
# plt.ylabel(f'J{joint_index} velocity [rad/s]')
# plt.xlabel('Time [s]')
# plt.grid()
#
# plt.figure()
# plt.title(r'$\tau_{measured} - \tau_{gravity}$ vs time')
# plt.plot(timestamps, diff)
# plt.ylabel(f'J{joint_index} effort difference [Nm]')
# plt.xlabel('Time [s]')
# plt.grid()
#
#
# plt.figure()
init = int(0.2 * min_len)
end = int(0.8 * min_len)
#
# plt.plot(vel[init:end], diff[init:end], color='skyblue', alpha=1, label='raw')
vel_f = np.array(iir_filter(vel))
diff_f = np.array(iir_filter(diff))
# plt.plot(vel_f[init:end], diff_f[init:end], marker='.', color='blue', alpha=1, label='filt')
# plt.xlabel(f'J{joint_index} velocity [rad/s]')
# plt.ylabel(f'J{joint_index} effort [Nm]')
# plt.legend(loc='upper left')
# plt.grid()
# plt.title('Force vs Velocity')
#
# plt.show()

# # ==================== ANIMATED PLOT ====================
# from matplotlib.animation import FuncAnimation
#
# vel_f = vel_f[init:end]
# diff_f = diff_f[init:end]
#
# idx = np.arange(0, len(vel_f), 25)
#
# vel_f = vel_f[idx]
# diff_f = diff_f[idx]
# #
# # print(len(vel_f))
#
# fig, ax = plt.subplots()
# scat = ax.scatter([], [], color='blue', s=5)
# ax.set_xlim(min(vel_f)-0.01, max(vel_f)+0.01)
# ax.set_ylim(min(diff_f)-0.1, max(diff_f)+0.1)
#
# xdata, ydata = [], []
#
# def update(frame):
#     data = np.column_stack((vel_f[:frame + 1], diff_f[:frame + 1]))  # primi frame+1 punti
#     scat.set_offsets(data)
#     return scat,
#
# ani = FuncAnimation(fig, update,
#                     frames=range(len(vel_f)),
#                     init_func=scat.set_offsets(np.empty((0, 2))),
#                     interval=0.0000000001,
#                     blit=True,
#                     repeat=False)
# plt.show()

# ani.save("animated_plot.gif", writer="pillow", fps=10000)  # 2 frame al secondo



# ================== SIGMOID FITTING ==================
def sigmoid_up(q_dot, tau_c_min, tau_c_max, A, B, C):
    sigmoid_term = (tau_c_max - tau_c_min) / (1 + np.exp(-A * (q_dot + B)))
    linear_term = C * q_dot
    return tau_c_min + sigmoid_term + linear_term

def sigmoid_down(q_dot, tau_c_min, tau_c_max, A, B, C):
    sigmoid_term = (tau_c_max - tau_c_min) / (1 + np.exp(-A * (q_dot - B)))
    linear_term = C * q_dot
    return tau_c_min + sigmoid_term + linear_term

tau_c_min = -0.7
tau_c_max = 1.4
# tau_c_mid = np.mean([tau_c_max, tau_c_min])     # tau at zero velocity
tau_c_mid = 0
A = 4000
B = 0.0007
C = 2.1

tau_f_max = sigmoid_up(vel_f[init:end], tau_c_min, tau_c_max, A, B, C)
tau_f_min = sigmoid_down(vel_f[init:end], tau_c_min, tau_c_max, A, B, C)

# ================== GAUSSIAN STD FITTING ==================
std_0 = 0.6
k = 3.5
std = std_0 * (1 + k * abs(vel_f[init:end]))

gaussian_std_sup = sigmoid_up(vel_f[init:end], tau_c_min, tau_c_max, A, B, C) + (std / 2)
gaussian_std_inf = sigmoid_down(vel_f[init:end], tau_c_min, tau_c_max, A, B, C) - (std / 2)

plt.figure()
plt.plot(vel, diff, color='skyblue', alpha=0.5, label='raw')
plt.plot(vel_f, diff_f, color='green', alpha=0.8, marker='.', linestyle='', label='filt')
plt.plot(vel_f[init:end], tau_f_max - tau_c_mid, color='magenta', alpha=1, label='tau_f')
plt.plot(vel_f[init:end], tau_f_min - tau_c_mid, color='magenta', alpha=1)
plt.plot(vel_f[init:end], gaussian_std_sup - tau_c_mid, 'k--', label='gaussian std')
plt.plot(vel_f[init:end], gaussian_std_inf - tau_c_mid, 'k--')

plt.xlabel(f'J{joint_index} velocity [rad/s]')
plt.ylabel(f'J{joint_index} effort [Nm]')
plt.legend()
plt.grid()
plt.title('COULOMB FRICTION BANDWIDTH ESTIMATION')

plt.show()

