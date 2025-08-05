#!/usr/bin/env python

import time
import math
import crtk
import dvrk
import numpy
import sys

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
        # go to start position, make sure 3rd joint is past cannula
        jp, _ = self.arm.setpoint_jp()
        goal = numpy.copy(jp)
        goal.fill(0)
        set_goal = input('Do you want to set the start position of {}?  [y/n]  -> '.format(self.arm.name()))
        if set_goal == 'y':
            goal[0] = math.radians(float(input('Joint 0 [deg]: ')))
            goal[1] = math.radians(float(input('Joint 1 [deg]: ')))
            goal[2] = input('Joint 2 [m]: ')
        elif set_goal == 'n':
            goal[0] = math.radians(0)
            goal[1] = math.radians(0)
            goal[2] = 0.12
        else:
            sys.exit('  ! failed to set start position')

        print('-> move to starting position: {} with jaw closed'.format(list(goal)))
        self.arm.move_jp(goal).wait()
        self.arm.jaw.close().wait()

    def move_const_acc(self, joint_index, acc):
        input('press Enter to start movement of joint {} with acc = {} rad/s^2...'.format(joint_index, acc))
        jp, _ = self.arm.setpoint_jp()

        q = jp[joint_index]
        v = 0.0
        dt = self.period

        a = acc / 2     # half of the target acceleration for the first segment

        segment_duration = 1.0
        steps_per_seg = int(segment_duration / dt)
        total_duration = 20.0
        total_steps = int(total_duration / dt)

        sleep_rate = self.ral.create_rate(1.0 / self.period)

        for step in range(total_steps):

            if step == steps_per_seg:
                a *= 2      # reset target acceleration after first segment

            if step % steps_per_seg == 0 and step > 0:
                a = - a     # change acceleration sign at every segment
            elif step % (steps_per_seg/2) == 0 and step > 0:
                time.sleep(1)

            v += a * dt
            q += v * dt + 0.5 * a * dt ** 2

            jp, _ = self.arm.setpoint_jp()
            jp.fill(0.0)
            jp[1] = math.radians(-50)
            jp[2] = 0.12
            jp[joint_index] = q
            self.arm.servo_jp(jp)

            sleep_rate.sleep()

    def main(self):
        self.home()
        self.start_position()
        self.move_const_acc(0, 1)

if __name__ == '__main__':
    ral = crtk.ral('dvrk_constant_acceleration_test')
    constant_acceleration = ConstantAcceleration(ral, 'PSM1', 0.005)
    ral.spin_and_execute(constant_acceleration.main)
