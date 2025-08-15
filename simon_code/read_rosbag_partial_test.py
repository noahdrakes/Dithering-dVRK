#!/usr/bin/env python

import rosbag
import numpy as np
import matplotlib.pyplot as plt


def iir_filter(x):
    y = [0] * len(x)
    for n in range(len(x)):
        y[n] = 0.4 * x[n] + (0.6 * y[n - 1] if n > 0 else 0)
    return y

file_name = 'j0_acquisitions/j0_slow/acc0.0025-40s/j0+80deg.bag'
bag = rosbag.Bag(file_name)
joint_index = int(file_name[1])
measured_jp = []
measured_jv = []
measured_jf = []
timestamps = []

for topic, msg, t in bag.read_messages(topics=['/PSM1/measured_js']):
    measured_jp.append(msg.position)
    measured_jv.append(msg.velocity)
    measured_jf.append(msg.effort)
    timestamps.append(t)

gc_setpoint_jf = []
for topic, msg, t in bag.read_messages(topics=['/PSM1/gravity_compensation/setpoint_js']):
    gc_setpoint_jf.append(msg.effort)

bag.close()

# timestamps = np.array(timestamps)
measured_jp = np.array(measured_jp)
measured_jv = np.array(measured_jv)
measured_jf = np.array(measured_jf)
gc_setpoint_jf = np.array(gc_setpoint_jf)

half_len = len(measured_jp) // 2

plt.figure()
plt.subplot(211)
plt.plot(measured_jp[:half_len, joint_index], color='purple', alpha=1)
plt.ylabel('J{} position [rad]'.format(joint_index))
plt.suptitle('POSITION AND VELOCITY FOR CONSTANT ACCELERATION')
plt.grid()
plt.subplot(212)
plt.plot(measured_jv[:half_len, joint_index], color='green', alpha=1)
plt.ylabel('J{} velocity [rad/s]'.format(joint_index))
plt.xlabel('Time [s]')
plt.grid()


plt.figure()
diff = measured_jf[:half_len-1, joint_index] - gc_setpoint_jf[:half_len-1, joint_index]
plt.scatter(measured_jv[:half_len-1, joint_index], diff, s=10, color='skyblue', alpha=0.7, label='raw')
measured_jv = iir_filter(measured_jv[:half_len-1, joint_index])
diff = iir_filter(diff)
plt.scatter(measured_jv, diff, s=10, color='blue', alpha=0.7, label='filt')
plt.xlabel('J{} velocity [rad/s]'.format(joint_index))
plt.ylabel('J{} effort [Nm]'.format(joint_index))
plt.legend()
plt.grid()
plt.title('COULOMB FRICTION BANDWIDTH ESTIMATION')

plt.figure()
plt.plot(diff)

plt.show()

