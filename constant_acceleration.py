#!/usr/bin/env python

import time
import math
import crtk
import dvrk
import numpy
import sys
import matplotlib.pyplot as plt
from oauthlib.uri_validate import segment

if sys.version_info.major < 3:
    input = raw_input

class ConstantAcceleration:
    def __init__(self, ral, arm_name, period):
        print('-> configuring class for {}'.format(arm_name))
        self.ral = ral
        self.period = period
        self.arm = dvrk.psm(ral = ral,
                            arm_name = arm_name)

    def home(self):
        self.ral.check_connections()

        print('-> starting enable')
        if not self.arm.enable(10):
            sys.exit('  ! failed to enable within 10 seconds')
        print('-> starting home')
        if not self.arm.home(10):
            sys.exit('  ! failed to home within 10 seconds')
        time.sleep(1)

    def start_position(self):
        # go to zero position, make sure 3rd joint is past cannula
        jp, _ = self.arm.setpoint_jp()
        goal = numpy.copy(jp)
        goal.fill(0)
        goal[0] = -0.025
        goal[1] = math.radians(0)
        goal[2] = 0.12
        print('-> move to starting position: {} with jaw closed'.format(list(goal)))

        self.arm.move_jp(goal).wait()
        self.arm.jaw.close().wait()

    def iir_filter(self, x):
        y = [0] * len(x)
        for n in range(len(x)):
            y[n] = 0.4 * x[n] + (0.6 * y[n-1] if n > 0 else 0)
        return y

    def move_joint(self, joint_index, acc):
        input('press Enter to start movement of joint {} with acc = {} rad/s^2...'.format(joint_index, acc))
        jp, _ = self.arm.measured_jp()
        jv, _ = self.arm.measured_jv()
        jf, _ = self.arm.measured_jf()

        q = jp[joint_index]
        v = 0.0
        dt = self.period

        q_hist = [q]
        v_hist = [v]
        jp_hist = [jp[joint_index]]
        jv_hist = [jv[joint_index]]
        tau_hist = [jf[joint_index]]

        a = acc / 2

        segment_duration = 1.0
        steps_per_seg = int(segment_duration / dt)
        total_duration = 40.0
        total_steps = int(total_duration / dt)
        t = numpy.linspace(0, total_duration, total_steps + 1)

        sleep_rate = self.ral.create_rate(1.0 / self.period)

        for step in range(total_steps):
            if step == steps_per_seg:
                a *= 2

            if step % steps_per_seg == 0 and step > 0:
                a = - a

            v += a * dt
            q += v * dt + 0.5 * a * dt ** 2

            v_hist.append(v)
            q_hist.append(q)

            jp, _ = self.arm.measured_jp()
            jv, _ = self.arm.measured_jv()
            jf, _ = self.arm.measured_jf()
            jp_hist.append(jp[joint_index])
            jv_hist.append(jv[joint_index])
            tau_hist.append(jf[joint_index])
            jp.fill(0.0)
            jp[2] = 0.12
            jp[joint_index] = q
            self.arm.servo_jp(jp)

            sleep_rate.sleep()

        plt.figure()
        plt.subplot(211)
        plt.plot(t, q_hist, color='pink', alpha=0.7, label='trajectory')
        plt.plot(t, jp_hist, color='purple', alpha=1, label='measured')
        plt.ylabel('J{} position [rad]'.format(joint_index))
        plt.suptitle('POSITION AND VELOCITY FOR CONSTANT ACCELERATION ({} $rad/s^2$)'.format(acc))
        plt.grid()
        plt.subplot(212)
        plt.plot(t, v_hist, color='lightgreen', alpha=0.7, label='trajectory')
        plt.plot(t, jv_hist, color='green', alpha=0.7, label='trajectory')
        plt.ylabel('J{} velocity [rad/s]'.format(joint_index))
        plt.xlabel('Time [s]')
        plt.grid()


        plt.figure()
        plt.plot(jv_hist, tau_hist, color='skyblue', alpha=0.7, label='raw data')
        jv_hist = self.iir_filter(jv_hist)
        tau_hist = self.iir_filter(tau_hist)
        plt.plot(jv_hist, tau_hist, color='blue', alpha=1, label='filtered data')
        plt.xlabel('J{} velocity [rad/s]'.format(joint_index))
        plt.ylabel('J{} effort [Nm]'.format(joint_index))
        plt.grid()
        plt.legend()
        plt.title('COULOMB FRICTION BANDWIDTH ESTIMATION')

        plt.show()


    def main(self):
        self.home()
        self.start_position()
        self.move_joint(0, 0.1)

if __name__ == '__main__':
    ral = crtk.ral('dvrk_acceleration_test')
    constant_acceleration = ConstantAcceleration(ral, 'PSM1', 0.004)
    ral.spin_and_execute(constant_acceleration.main)