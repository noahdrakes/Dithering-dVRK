import argparse
import time
import sys
import crtk
import math
import scipy
import matplotlib.pyplot as plt
import numpy
import rospy
from geometry_msgs.msg import Vector3Stamped
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
        self.joint_index = 0
        self.arm = Device(ral = ral,
                          arm_name = arm_name)

        # dithering configuration
        self.dith_ampl = dith_ampl
        self.dith_freq = dith_freq
        self.dith_on = False

        # accelerometer data subscriber
        self.acc_data = deque(maxlen=2000)
        self.acc_sub = self.ral.subscriber("/accelerometer/data", Vector3Stamped, self.acc_callback)

        time.sleep(0.2)


    def acc_callback(self, msg):
        self.acc_data.append([msg.vector.x, msg.vector.y, msg.vector.z])


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
            goal[0] = math.radians(60)
            goal[1] = -1
            goal[2] = 0.12
            goal[3] = 0.0
            goal[4] = 0.0
            goal[5] = 0.0

            input('   > press enter to move to start position: {}'.format(goal))

            self.arm.move_jp(goal).wait()
            print('  < ready for cartesian mode')

    def spectrum(self, signal, fs):
        # convert signal (deque) to numpy array
        signal = numpy.array(signal)

        # signle axis acceleration
        signal_1axis = numpy.sqrt(numpy.sum(signal ** 2, axis=1))
        N = len(signal_1axis)

        # spectrum computation
        signal_det = detrend(signal_1axis, type='constant')
        window = get_window('hann', N)
        signal_w = signal_det * window
        fft_vals = numpy.fft.rfft(signal_w)
        freqs = numpy.fft.rfftfreq(N, d=1 / fs)
        U = (window ** 2).sum() / N
        amp = (numpy.abs(fft_vals) / (numpy.sqrt(U) * N)) * numpy.sqrt(2)

        return freqs, amp

    def adjust_amplitude(self):
        freqs, ampl = self.spectrum(self.acc_data, 500)

        # spectral amplitude at the dithering frequency
        if abs(freqs[ampl.argmax()] - self.dith_freq) < 0.5:
            spectral_freq = freqs[ampl.argmax()]
            spectral_ampl = ampl.max()
        else:
            idx = numpy.argmin(numpy.abs(freqs - self.dith_freq))
            spectral_freq = freqs[idx]
            spectral_ampl = ampl[idx]

        print(f'  > computed spectral amplitude at {spectral_freq:.2f} Hz: {spectral_ampl:.4f} g')

        # adjusting dithering amplitude according to spectral amplitude only when dithering is on
        if self.dith_on:
            if spectral_ampl < 0.010:
                self.dith_ampl += 0.0001
                print('    < dithering amplitude increased: {} Nm'.format(self.dith_ampl))
            elif spectral_ampl > 0.015:
                self.dith_ampl -= 0.0001
                print('    < dithering amplitude decreased: {} Nm'.format(self.dith_ampl))


    def dithering_signal(self, duration):

        # duration = 30  # seconds
        samples = int(duration / self.period)
        t = numpy.linspace(0, duration, samples)
        sine_wave = numpy.sin(2.0 * math.pi * self.dith_freq * t)

        # ramp to smooth the beginning and the end of the dithering signal
        transition = numpy.linspace(0, 1, int(5 * self.dith_freq))

        j = 0

        for i in range(len(transition)):
            while sine_wave[j] >= 0:
                sine_wave[j] *= transition[i]
                j += 1

            while sine_wave[j] < 0:
                sine_wave[j] *= transition[i]
                j += 1

        j = len(sine_wave) - 1
        for i in range(len(transition)):
            while sine_wave[j] >= 0:
                sine_wave[j] *= transition[i]
                j -= 1

            while sine_wave[j] < 0:
                sine_wave[j] *= transition[i]
                j -= 1

        return sine_wave


    def move_and_dither(self, pos):
        pos = math.radians(pos)
        # move to desired positon
        jp, _ = self.arm.measured_jp()
        jp[self.joint_index] = pos
        input('> press Enter to move to {}'.format(jp))
        self.arm.move_jp(jp).wait()
        print('  < arm moved')

        # start dithering signal
        print('> press Enter to start dithering signal for joint {}:'.format(self.joint_index))
        print('  frequency: {} Hz'.format(self.dith_freq))
        input('  amplitude: {} Nm'.format(self.dith_ampl))

        sine_wave = self.dithering_signal(60)
        frequency = 1.0 / self.period
        sleep_rate = self.ral.create_rate(frequency)
        self.dith_on = True
        # goal_hist = []

        for i in range(len(sine_wave)):

            # dithering amplitude adjustment
            if 15 * int(frequency) < i <= len(sine_wave) - 5 * int(frequency) and i % int(5 * frequency) == 0:
                print('> amplitude check at sample {}'.format(i))
                self.adjust_amplitude()

            # safety stop if too high angles are achieved
            jp, _ = self.arm.measured_jp()

            if jp[self.joint_index] > math.radians(100) or jp[self.joint_index] < -math.radians(100):
                self.arm.disable()
                print('ROM limit reached')
                self.ral.shutdown()

            goal = numpy.copy(jp)
            goal[self.joint_index] = sine_wave[i] * self.dith_ampl + pos
            # goal_hist.append(goal[self.joint_index])
            self.arm.servo_jp(goal)
            sleep_rate.sleep()

        # return goal_hist



    def run(self):
        self.home()
        self.move_and_dither(pos=90)


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