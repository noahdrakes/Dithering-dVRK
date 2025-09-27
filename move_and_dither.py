import argparse
import time
import sys
import crtk
import math

import numpy as np
import scipy
import matplotlib.pyplot as plt
import numpy
import rospy
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Joy
from scipy.signal import get_window, detrend
from collections import deque
import PyKDL

class Device:
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
        self.crtk_utils.add_servo_jf()

    def ral(self):
        return self.__ral


class PositionDithering:

    # configuration
    def __init__(self, ral, arm_name, dith_ampl, dith_freq, period = 0.01):
        print('> configuring dvrk_arm_test for {}'.format(arm_name))
        self.ral = ral
        self.arm_name = arm_name
        self.period = period
        self.frequency = 1.0 / period
        self.joint_index = 0
        self.arm = Device(ral = ral,
                          arm_name = arm_name)

        # dithering configuration
        self.dith_ampl = dith_ampl
        self.dith_freq = dith_freq
        self.dith_off = False
        self.check_spectrum = False

        time.sleep(0.2)


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


    def prepare_cartesian(self):
        # make sure the camera is past the cannula and tool vertical
        jp, ts = self.arm.setpoint_jp()
        goal = numpy.copy(jp)
        if ((self.arm_name.endswith('PSM1')) or (self.arm_name.endswith('PSM2'))
                or (self.arm_name.endswith('PSM3')) or (self.arm_name.endswith('ECM'))):
            print('  > preparing for cartesian motion')

            # set in position joint mode
            goal[0] = math.radians(-8)
            goal[1] = -1
            goal[2] = 0.12
            goal[3] = 0.0
            goal[4] = 0.0
            goal[5] = 0.0

            input('   > press enter to move to start position: {}'.format(goal))

            self.arm.move_jp(goal).wait()
            print('  < ready for cartesian mode')


    def dithering(self):
        start_duration = 5.0
        start_amplitude = numpy.linspace(0, 1, int(start_duration * self.frequency))
        stop_amplitude = numpy.linspace(1, 0, int(1 * self.frequency))

        sleep_rate = self.ral.create_rate(self.frequency)
        t = 0
        t_old = start_duration
        i = 0
        dt = self.period

        jp, _ = self.arm.measured_jp()
        pos = jp[self.joint_index]          # should become the mean of the last period
        # goal_hist = []

        print('> press Enter to start dithering signal for joint {}:'.format(self.joint_index))
        print('  frequency: {} Hz'.format(self.dith_freq))
        input('  amplitude: {} rad'.format(self.dith_ampl))

        while True:

            jp, _ = self.arm.measured_jp()
            # print(jp)

            # safety stop if too high angles are achieved
            if jp[self.joint_index] > math.radians(100) or jp[self.joint_index] < -math.radians(100):
                self.arm.disable()
                print('ROM limit reached')
                self.ral.shutdown()

            # manage the beginning and the end of the dithering signal
            if t <= start_duration:
                if i >= len(start_amplitude): smooth = 1
                else: smooth = start_amplitude[i]
                i += 1
            elif self.dith_off:
                self.check_spectrum = False
                if i >= len(stop_amplitude):
                    self.dith_off = False
                    break
                else: smooth = stop_amplitude[i]
                i += 1
            else:
                self.check_spectrum = True
                smooth = 1
                i = 0

            # arm movement
            if t >= 10.0:
                pos += 0.0000025

            sine = numpy.sin(2.0 * math.pi * self.dith_freq * t) * smooth

            goal = np.copy(jp)
            goal[self.joint_index] = sine * self.dith_ampl + pos
            # goal_hist.append(goal[self.joint_index])
            # print(goal)
            self.arm.servo_jp(goal)

            if t > 125.0: self.dith_off = True

            t += dt
            sleep_rate.sleep()

        # return goal_hist


    def run(self):
        self.home()
        self.dithering()


    def on_shutdown(self):
        print ('>> illustrating user defined shutdown callback')


if __name__ == '__main__':
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type = str, required = True,
                        choices=['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-p', '--period', type =float, default = 0.01,
                        help = 'period used for loops using servo commands')
    parser.add_argument('-A', '--dithering_amplitude', type=float,
                        help='amplitude of the dithering command')
    parser.add_argument('-f', '--dithering_frequency', type=float,
                        help='frequency of the dithering command')
    args = parser.parse_args(argv)

    ral = crtk.ral('dvrk_arm_test')
    application = PositionDithering(ral, args.arm, args.dithering_amplitude, args.dithering_frequency, args.period)
    ral.on_shutdown(application.on_shutdown)
    ral.spin_and_execute(application.run)