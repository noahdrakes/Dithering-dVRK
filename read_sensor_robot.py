from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import argparse
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
import scipy


def signal_mean(signal, n_period, T, fs):
    window = int(n_period * T * fs)
    mean = np.zeros_like(signal)
    for i in range(len(signal)-window):
        mean[i+window] = np.mean(signal[i:i+window+1])

    return mean

def low_pass(data, fs, cutoff, order):
    """Butterworth high pass filtering of a signal"""
    nyq = 0.5 * fs
    f_cut = cutoff / nyq
    b, a = scipy.signal.butter(order, f_cut, btype='low', analog=False)
    return scipy.signal.filtfilt(b, a, data)


parser = argparse.ArgumentParser()
parser.add_argument('-n', '--file_name', type=str, required=True,
                    help='rosbag record with external forces')
parser.add_argument('-b', '--baseline_file_name', type=str, required=True,
                    help='rosbag record without external forces -> used to compute baseline signal')
arg = parser.parse_args()

file_name = arg.file_name
split = file_name.split('-')
joint_index = int(split[1])
frequency = float(split[2])
amplitude = float(split[3])

measured_cf_x = []
t_measured = []
sensor_cf_z = []
t_sensor = []

bagpath = Path(file_name)
typestore = get_typestore(Stores.LATEST)

with AnyReader([bagpath], default_typestore=typestore) as reader:
    # collection of all available connections (topic)
    conns = reader.connections
    # selection of the desired ones
    # js_conn = [c for c in conns if c.topic == '/PSM1/measured_js'][0]
    fs_conn = [c for c in conns if c.topic == '/measured_cf'][0]
    cf_conn = [c for c in conns if c.topic == '/PSM1/spatial/measured_cf'][0]

    for conn, t, rawdata in reader.messages(connections=[cf_conn]):
        msg = reader.deserialize(rawdata, conn.msgtype)
        measured_cf_x.append(msg.wrench.force.x)
        t_measured.append(t)


    for conn, t, rawdata in reader.messages(connections=[fs_conn]):
        msg = reader.deserialize(rawdata, conn.msgtype)
        sensor_cf_z.append(msg.wrench.force.z)
        t_sensor.append(t)

bagpath = Path(arg.baseline_file_name)
typestore = get_typestore(Stores.LATEST)

free_cf_x = []
t_free = []

with AnyReader([bagpath], default_typestore=typestore) as reader:
    # collection of all available connections (topic)
    conns = reader.connections
    # selection of the desired ones
    cf_conn = [c for c in conns if c.topic == '/PSM1/spatial/measured_cf'][0]

    for conn, t, rawdata in reader.messages(connections=[cf_conn]):
        msg = reader.deserialize(rawdata, conn.msgtype)
        free_cf_x.append(msg.wrench.force.x)
        t_free.append(t)


measured_cf_x = np.array(measured_cf_x)
free_cf_x = np.array(free_cf_x)
sensor_cf_z = np.array(sensor_cf_z)

delta_t = 0.45

t_measured = (np.array(t_measured) - t_measured[0]) * 1e-9
t_sensor   = (np.array(t_sensor)   - t_sensor[0])   * 1e-9
t_free     = (np.array(t_free)     - t_free[0])     * 1e-9
fs_measured = 1 / np.mean(np.diff(t_measured))
fs_free     = 1 / np.mean(np.diff(t_free))
T = 1/20.0

new = np.zeros(int(len(free_cf_x) + delta_t * fs_free))
new[-len(free_cf_x):] = free_cf_x

min_len = min(len(new), len(measured_cf_x))
measured_cf_x = measured_cf_x[:min_len]
free_cf_x = new[:min_len]
t_measured = t_measured[:min_len]

# measured_mean = signal_mean(measured_cf_x, 5, T, fs_measured)
# free_mean = signal_mean(free_cf_x, 5, T, fs_free)

measured_mean = low_pass(measured_cf_x, fs_measured, 5, 4)
free_mean = low_pass(free_cf_x, fs_free, 5, 4)

fig, axs = plt.subplots(1, 2)
axs[0].plot(t_measured, measured_cf_x, alpha=0.7, label="contact")
axs[0].plot(t_measured, free_cf_x, alpha=0.7, label="free space")
axs[0].set_ylabel("Force [N]")
axs[0].set_xlabel("Time [s]")
axs[0].set_title("MEASURED FORCE")
axs[0].legend()
axs[0].grid()
axs[1].plot(t_sensor, np.abs(sensor_cf_z), color="green", label="sensor")
axs[1].plot(t_measured, measured_mean - free_mean, color="purple", label="estimated")
axs[1].set_ylabel("Force [N]")
axs[1].set_xlabel("Time [s]")
axs[1].set_title("EXTERNAL FORCE")
axs[1].legend()
axs[1].grid()

plt.show()
