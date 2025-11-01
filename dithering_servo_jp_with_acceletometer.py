#!/usr/bin/env python3
"""
Position Dithering Control for dVRK Arms
=========================================

This script applies a joint-level dithering signal to a dVRK arm through the 
CRTK interface. Dithering consists of a small oscillatory torque applied 
to reduce static friction and improve motion smoothness or force estimation.

This version operates in position control mode (`servo_jp`).

It supports optional online amplitude tuning using accelerometer feedback.

Usage:
    python dithering_servo_jp_with_accelerometer.py \
        -a PSM1 \
        -j 2 \
        -A 0.005 \
        -f 10 \
        -p 0.001

Arguments:
    -a, --arm                   Arm name (PSM1, PSM2, PSM3)
    -j, --joint_index           Index of the joint to apply dithering
    -A, --dithering_amplitude   Dithering amplitude (rad or m)
    -f, --dithering_frequency   Dithering frequency (Hz)
    -p, --period                Control period [s] (default: 0.01)

Safety:
    Ensure the robot is in a safe configuration and test mode 
    before enabling this script. Dithering generates continuous oscillatory motion.
"""


import argparse
import time
import sys
import crtk
import math
import numpy
import rospy
from geometry_msgs.msg import Vector3Stamped
from collections import deque
import PyKDL

# =========================== DEVICE WRAPPER ==============================
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


# =========================== POSITION DITHERING ==========================
class PositionDithering:

    # configuration
    def __init__(self, ral, arm_name, dith_ampl, dith_freq, joint_index, period = 0.01):
        print('> configuring dvrk_arm_test for {}'.format(arm_name))
        self.ral         = ral
        self.arm_name    = arm_name
        self.period      = period
        self.frequency   = 1.0 / period
        self.joint_index = joint_index
        self.arm         = Device(ral = ral,
                                  arm_name = arm_name)

        # dithering configuration
        self.dith_ampl = dith_ampl
        self.dith_freq = dith_freq
        self.dith_off = False

        # joint limits for safety
        self.JOINT_LIMITS = [
            (math.radians(-100.0), math.radians(100.0)),  # joint 0
            (math.radians(-60.0), math.radians(60.0)),  # joint 1
            (0.050, 0.250),                             # joint 2
        ]

        # accelerometer data subscriber
        self.acc_data       = deque(maxlen=2000)
        self.acc_sub        = self.ral.subscriber("/accelerometer/data", Vector3Stamped, self.acc_callback)
        self.check_spectrum = False
        self.acc_freq       = 498.0
        self.acc_counter    = 0

        time.sleep(0.2)


    # ---------------------- ACCELEROMETER CALLBACK ---------------------- #
    def acc_callback(self, msg):

        self.acc_data.append([msg.vector.x, msg.vector.y, msg.vector.z])
    
        if self.check_spectrum:
            self.acc_counter += 1
            if self.acc_counter == 2000:
                self.acc_counter = 0
                data = numpy.array(self.acc_data)
                x = self.spectrum(data[:, 0], self.acc_freq)
                y = self.spectrum(data[:, 1], self.acc_freq)
                z = self.spectrum(data[:, 2], self.acc_freq)
                total_spectrum = numpy.sqrt(x**2 + y**2 + z**2)
                print(f"> spectral amplitude at {self.dith_freq} Hz: {total_spectrum:.4f}")

                # dithering amplitude adjustment
                if total_spectrum < 0.005:
                    self.dith_ampl += 0.1
                    print(f'  < dithering amplitude increased: {self.dith_ampl:.2f} Nm (or N)')
                elif total_spectrum > 0.010:
                    self.dith_ampl -= 0.1
                    print(f'  < dithering amplitude decreased: {self.dith_ampl:.2f} Nm (or N)')

                print('---------------------------------')

    
    # ---------------------- SPECTRAL ANALYSIS --------------------------- #
    def analyze_spectrum(self, freq, ampl, target_f):
        '''returns the maximum value of the spectrum in a neighborhood of the dithering frequency'''
        idx_l = numpy.argmin(numpy.abs(freq - (target_f - 1.0)))
        idx_h = numpy.argmin(numpy.abs(freq - (target_f + 1.0)))
        return numpy.max(ampl[idx_l:idx_h])
    
    
    def spectrum(self, signal, fs):
        '''computes the spectrum for dithering amplitude tuning'''
        N = len(signal)
        fft_vals = numpy.fft.fft(signal)
        fft_vals = fft_vals[:N // 2]
        freqs = numpy.fft.fftfreq(N, 1 / fs)[:N // 2]
        amplitudes = (2.0 / N) * numpy.abs(fft_vals)
        value = self.analyze_spectrum(freqs, amplitudes, self.dith_freq)
        return value


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


    def prepare_cartesian(self):
        # make sure the camera is past the cannula and tool vertical
        jp, ts = self.arm.setpoint_jp()
        goal = numpy.copy(jp)
        if ((self.arm_name.endswith('PSM1')) or (self.arm_name.endswith('PSM2'))
                or (self.arm_name.endswith('PSM3')) or (self.arm_name.endswith('ECM'))):
            print('  > preparing for cartesian motion')

            # set start position
            goal[0] = math.radians(0.0)
            goal[1] = 0.0
            goal[2] = 0.12
            goal[3] = math.radians(0.0)
            goal[4] = math.radians(0.0)
            goal[5] = math.radians(0.0)

            input('   > press enter to move to start position: {}'.format(goal))

            self.arm.move_jp(goal).wait()
            print('  < ready for cartesian mode')


    # ---------------------- SAFETY CHECK ------------------------------- #
    def check_joint_limits(self, joint_values, limits):

        joint_values = joint_values[:3]

        for i, (val, (low, high)) in enumerate(zip(joint_values, limits)):
            if not (low <= val <= high):
                print(joint_values, limits)
                self.arm.disable()
                print('! ROM limit reached')
                self.ral.shutdown()


    # ---------------------- DITHERING LOOP ----------------------------- #
    def dithering(self):
        start_duration  = 5.0
        stop_duration   = 1.0
        start_amplitude = numpy.linspace(0, 1, int(start_duration * self.frequency))
        stop_amplitude  = numpy.linspace(1, 0, int(stop_duration * self.frequency))

        sleep_rate = self.ral.create_rate(self.frequency)
        t = 0
        i = 0
        dt = self.period

        jp, _ = self.arm.measured_jp()

        # set nominal position and velocity
        q_ref = jp[self.joint_index]
        q_dot_ref = 0

        print('> press Enter to start dithering signal for joint {}:'.format(self.joint_index))
        print('  frequency: {} Hz'.format(self.dith_freq))
        print('  amplitude: {} rad (or m)'.format(self.dith_ampl))
        
        while not self.ral.is_shutdown():

            jp, _ = self.arm.measured_jp()

            # safety stop if too high angles are achieved
            self.check_joint_limits(jp, self.JOINT_LIMITS)

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
                self.check_spectrum = True  # change to False to disable online-tuning of dithering amplitude
                smooth = 1
                i = 0

            # to move the arm while dithering is on, update nominal position and velocity
            if t >= 10.0:
                q_ref -= 0.0000025    # velocity = 0.0000025 rad/ms --> 0.0025 rad/s
                q_dot_ref = -0.0025

            # preparing servo_js command
            sine = numpy.sin(2.0 * math.pi * self.dith_freq * t) * smooth
            cosine = numpy.cos(2.0 * math.pi * self.dith_freq * t) * smooth

            jp_setpoint = np.copy(jp)
            jp_setpoint[self.joint_index] = sine * self.dith_ampl + q_ref
            
            jv_setpoint = np.zeros_like(jp_setpoint)
            jv_setpoint[self.joint_index] = self.dith_ampl * 2.0 * math.pi * self.dith_freq * cosine + q_dot_ref
            
            self.arm.servo_jp(jp_setpoint, jv_setpoint)

            # dithering disable condition
            if t > 10.0: 
                self.dith_off = True

            t += dt
            sleep_rate.sleep()


    # ---------------------- EXECUTION ENTRY ----------------------------- #
    def run(self):
        self.home()
        self.dithering()


    def on_shutdown(self):
        print ('>> illustrating user defined shutdown callback')


# ===========================================================================
if __name__ == '__main__':
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type = str, required = True,
                        choices=['PSM1', 'PSM2', 'PSM3'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-p', '--period', type =float, default = 0.01,
                        help = 'period used for loops using servo commands')
    parser.add_argument('-A', '--dithering_amplitude', type=float,
                        help='amplitude of the dithering command')
    parser.add_argument('-f', '--dithering_frequency', type=float,
                        help='frequency of the dithering command')
    parser.add_argument('-j', '--joint_index', type=int,
                        choices=[0, 1, 2],
                        help='joint you want to dither')
    args = parser.parse_args(argv)

    ral = crtk.ral('dvrk_arm_test')
    application = PositionDithering(ral, args.arm, args.dithering_amplitude, args.dithering_frequency, args.joint_index, args.period)
    ral.on_shutdown(application.on_shutdown)
    ral.spin_and_execute(application.run)
