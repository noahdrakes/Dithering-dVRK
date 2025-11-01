#!/usr/bin/env python3
"""
constant_acceleration.py
------------------------

This script performs a constant-acceleration motion of one joint of a dVRK arm 
(ECM, MTML, MTMR, PSM1–PSM3) using the CRTK interface.

It allows the user to:
- Home the selected arm.
- Move to a specified or default start position.
- Execute a back-and-forth motion under constant acceleration.

Usage example:
    python arm_constant_acc.py \
    -a PSM1 \
    -j 0 \
    -A 0.5 \
    -d 2.0

Arguments:
    -a, --arm               Arm name [ECM, MTML, MTMR, PSM1, PSM2, PSM3]
    -j, --joint_index       Index of the joint to move [0, 1, 2]
    -A, --acceleration      Constant acceleration value (rad/s² or m/s²)
    -d, --segment_duration  Duration of each acceleration segment [s]
    -p, --period            Servo update period [s] (default: 0.001)
"""

import argparse
import time
import sys
import crtk
import math
import numpy
import PyKDL

class device:
    def __init__(self, ral, arm_name, connection_timeout = 5.0):
        # populate this class with all the ROS topics we need
        self.__ral = ral.create_child(arm_name)
        self.crtk_utils = crtk.utils(self, self.__ral, connection_timeout)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_setpoint_js()
        self.crtk_utils.add_measured_js()
        self.crtk_utils.add_setpoint_cp()
        self.crtk_utils.add_servo_jp()
        self.crtk_utils.add_move_jp()
        self.crtk_utils.add_servo_cp()
        self.crtk_utils.add_move_cp()

    def ral(self):
        return self.__ral


class ConstantAcceleration:

    # configuration
    def __init__(self, ral, arm_name, joint_index, acc, seg_duration, period = 0.001):
        print('> configuring {}'.format(arm_name))
        self.ral          = ral
        self.arm_name     = arm_name
        self.joint_index  = joint_index
        self.acc          = acc
        self.seg_duration = seg_duration
        self.period       = period
        self.arm          = device(ral = ral,
                                   arm_name = arm_name)
        
        time.sleep(0.2)


    # ---------------------- HOMING AND PREPARATION ---------------------- #
    def home(self):

        self.ral.check_connections()

        print('> starting enable')
        if not self.arm.enable(10):
            print('  ! failed to enable within 10 seconds')
            self.ral.shutdown()
        print('> starting home')
        if not self.arm.home(10):
            print('  ! failed to home within 10 seconds')
            self.ral.shutdown()

        # get current joints just to set size, ignore timestamp
        print('> move to starting position')
        self.prepare_cartesian()

        # move and wait
        print('> moving to starting position')
        jp, _ = self.arm.setpoint_jp()
        goal = numpy.copy(jp)
        self.arm.move_jp(goal).wait()

        # try to move again to make sure waiting is working fine, i.e. not blocking
        print('> testing move to current position')
        move_handle = self.arm.move_jp(goal)
        print('  move handle should return immediately')
        move_handle.wait()
        print('< home complete')


    # utility to position tool/camera deep enough before cartesian examples
    def prepare_cartesian(self):

        # make sure the camera is past the cannula and tool vertical
        jp, _ = self.arm.setpoint_jp()
        goal = numpy.copy(jp)

        set_goal = input('Do you want to set the start position of {}?  [y/n]  -> '.format(self.arm_name))
        # you may need to change the starting position according to the amplitude of the motion
        print('Motion amplitude: ±', str(0.5 * self.acc * self.seg_duration * self.seg_duration / 2 / 2))
        
        if set_goal == 'y':
            goal[0] = input('Joint 0 [rad]: ')
            goal[1] = input('Joint 1 [rad]: ')
            goal[2] = input('Joint 2 [m]: ')
        elif set_goal == 'n':
            goal[0] = math.radians(0)
            goal[1] = math.radians(0)
            goal[2] = 0.12
        else:
            sys.exit('  ! failed to set start position')

        self.arm.move_jp(goal).wait()
        print('  < ready for cartesian mode')


    # --------------------- CONSTANT ACC MOTION --------------------
    def move_const_acc(self):

        input('press Enter to start movement of joint {} with acc = {} rad/s^2 (or m/s^2)...'.format(self.joint_index, self.acc))

        jp, _ = self.arm.setpoint_jp()
        jv    = numpy.zeroslike(jp)
        
        q = jp[self.joint_index]
        v = 0.0
        a = self.acc
        dt = self.period

        steps_per_seg = int(self.seg_duration / dt)

        # define how many times drive the joint back and forth
        total_duration = self.seg_duration * 6.0
        total_steps = int(total_duration / dt)

        sleep_rate = self.ral.create_rate(1.0 / self.period)

        for step in range(total_steps):

            if step >= steps_per_seg / 2:
                if step % steps_per_seg == 0:
                    a = - a                             # change acceleration sign at every segment
                elif step % (steps_per_seg/2) == 0:
                    time.sleep(1)                       # wait 1s to be sure to have v=0

                q += v * dt + 0.5 * a * dt ** 2
                v += a * dt

            jp[self.joint_index] = q
            jv[self.joint_index] = v

            self.arm.servo_jp(jp, jv)

            sleep_rate.sleep()


    # --------------------------- MAIN -----------------------------
    def run(self):
        self.home()
        self.move_const_acc()


    def on_shutdown(self):
        print ('>> illustrating user defined shutdown callback')


# ===================================================================
if __name__ == '__main__':
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type = str, required = True,
                        choices=['PSM1', 'PSM2', 'PSM3'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-j', '--joint_index', type=int, required=True,
                        choices=[0, 1, 2],
                        help='index of the joint to move with contant acceleration')
    parser.add_argument('-A', '--acceleration', type=float, required=True,
                        help = 'acceleration value for selected joint')
    parser.add_argument('-d', '--segment_duration', type=float, required=True,
                        help='duration of the segment with constant acceleration')
    parser.add_argument('-p', '--period', type =float, default = 0.001,
                        help = 'period used for loops using servo commands')
    args = parser.parse_args(argv)

    ral = crtk.ral('dvrk_arm_test')
    application = ConstantAcceleration(ral, args.arm, args.joint_index, args.acceleration, args.segment_duration, args.period)
    ral.on_shutdown(application.on_shutdown)
    ral.spin_and_execute(application.run)
