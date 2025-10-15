import argparse
import time
import sys
import crtk
import math
import matplotlib.pyplot as plt
import numpy
import rospy
from geometry_msgs.msg import Vector3Stamped
import scipy
from collections import deque


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
        self.crtk_utils.add_servo_js()
        self.crtk_utils.add_move_jp()
        self.crtk_utils.add_servo_cp()
        self.crtk_utils.add_move_cp()
        self.crtk_utils.add_servo_jf()

    def ral(self):
        return self.__ral


class PositionDithering:

    # configuration
    def __init__(self, ral, arm_name, dith_ampl, dith_freq, joint_index, period = 0.01):
        print('> configuring dvrk_arm_test for {}'.format(arm_name))
        self.ral = ral
        self.arm_name = arm_name
        self.period = period
        self.frequency = 1.0 / period
        self.joint_index = joint_index
        self.arm = Device(ral = ral,
                          arm_name = arm_name)

        # dithering configuration
        self.dith_ampl = dith_ampl
        self.dith_freq = dith_freq
        self.dith_off = False


        # Joint limits
        self.JOINT_LIMITS = [
            (math.radians(-20.0), math.radians(20.0)),  # Joint 1
            (math.radians(-10.0), math.radians(10.0)),  # Joint 2
            (0.050, 0.200),  # Joint 3
            # (-1.57, 1.57),  # Joint 4
            # (-3.14, 3.14),  # Joint 5
            # (-1.0, 1.0)  # Joint 6
        ]

        # accelerometer data subscriber
        self.acc_data = deque(maxlen=2000)
        self.acc_sub = self.ral.subscriber("/accelerometer/data", Vector3Stamped, self.acc_callback)
        self.check_spectrum = False
        self.acc_freq = 498.0
        self.acc_counter = 0

        time.sleep(0.2)

    def acc_callback(self, msg):
        self.acc_data.append([msg.vector.x, msg.vector.y, msg.vector.z])
    
        if self.check_spectrum:
            self.acc_counter += 1
            if self.acc_counter == 2000:
                self.acc_counter = 0
                data = numpy.array(self.acc_data)
                # print('X: ', end='')
                x = self.spectrum(data[:, 0], self.acc_freq)
                # print('Y: ', end='')
                y = self.spectrum(data[:, 1], self.acc_freq)
                # print('Z: ', end='')
                z = self.spectrum(data[:, 2], self.acc_freq)
                total_spectrum = numpy.sqrt(x**2 + y**2 + z**2)
                print(f"> spectral amplitude at {self.dith_freq} Hz: {total_spectrum:.4f}")

                if total_spectrum < 0.005:
                    self.dith_ampl += 0.1
                    print(f'  < dithering amplitude increased: {self.dith_ampl:.2f} Nm')
                elif total_spectrum > 0.010:
                    self.dith_ampl -= 0.1
                    print(f'  < dithering amplitude decreased: {self.dith_ampl:.2f} Nm')

                print('---------------------------------')

    def band_pass(self, data, fs, low_cut, high_cut, order):
        nyq = 0.5 * fs
        low = low_cut / nyq
        high = high_cut / nyq
        b, a = scipy.signal.butter(order, [low, high], btype='band', analog=False)
        return scipy.signal.filtfilt(b, a, data)
    
    def analyze_spectrum(self, freq, ampl, target_f):
        idx_l = numpy.argmin(numpy.abs(freq - (target_f - 1.0)))
        idx_h = numpy.argmin(numpy.abs(freq - (target_f + 1.0)))
        return numpy.max(ampl[idx_l:idx_h])
    
    def spectrum(self, signal, fs):
        signal_filt = self.band_pass(signal, fs, self.dith_freq - 5.0, self.dith_freq + 5.0, 4)
        N = len(signal_filt)
        fft_vals = numpy.fft.fft(signal_filt)
        fft_vals = fft_vals[:N // 2]
        freqs = numpy.fft.fftfreq(N, 1 / fs)[:N // 2]
        amplitudes = (2.0 / N) * numpy.abs(fft_vals)
        value = self.analyze_spectrum(freqs, amplitudes, self.dith_freq)
        return value



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
            goal[0] = math.radians(0.0)
            goal[1] = 0.0
            goal[2] = 0.12
            goal[3] = math.radians(0.0)
            goal[4] = math.radians(0.0)
            goal[5] = math.radians(0.0)

            input('   > press enter to move to start position: {}'.format(goal))

            self.arm.move_jp(goal).wait()
            print('  < ready for cartesian mode')


    def check_joint_limits(self, joint_values, limits):

        joint_values = joint_values[:3]

        for i, (val, (low, high)) in enumerate(zip(joint_values, limits)):
            if not (low <= val <= high):
                print(joint_values, limits)
                self.arm.disable()
                print('ROM limit reached')
                self.ral.shutdown()


    def dithering(self):
        start_duration = 5.0
        stop_duration = 1.0
        start_amplitude = numpy.linspace(0, 1, int(start_duration * self.frequency))
        stop_amplitude = numpy.linspace(1, 0, int(stop_duration * self.frequency))

        sleep_rate = self.ral.create_rate(self.frequency)
        t = 0
        i = 0
        dt = self.period

        jp_measured, _ = self.arm.measured_jp()
        pos = jp_measured[self.joint_index]
        vel = 0.0


        print('> press Enter to start dithering signal for joint {}:'.format(self.joint_index))
        print('  frequency: {} Hz'.format(self.dith_freq))
        input('  amplitude: {} Nm'.format(self.dith_ampl))

        while not self.ral.is_shutdown():
            jp_measured, _ = self.arm.measured_jp()

            # safety stop if too high angles are achieved
            self.check_joint_limits(jp_measured, self.JOINT_LIMITS)


            # manage the beginning and the end of the dithering signal
            if t <= start_duration:
                if i >= len(start_amplitude):
                    smooth = 1
                else:
                    smooth = start_amplitude[i]
                i += 1
            elif self.dith_off:
                self.check_spectrum = False
                if i >= len(stop_amplitude):
                    self.dith_off = False
                    break
                else:
                    smooth = stop_amplitude[i]
                i += 1
            else:
                self.check_spectrum = True
                smooth = 1
                i = 0

            # preparing servo js commands
            jp_setpoint = numpy.copy(jp_measured)
            jp_setpoint[self.joint_index] = pos

            jv_setpoint = numpy.zeros_like(jp_setpoint)
            jv_setpoint[self.joint_index] = vel

            jf_setpoint = numpy.zeros_like(jp_setpoint)
            jf_setpoint[self.joint_index] = numpy.sin(2.0 * math.pi * self.dith_freq * t) * smooth * self.dith_ampl


            self.arm.servo_js(jp_setpoint, jv_setpoint, jf_setpoint)


            if t > 40.0:
                self.dith_off = True

            t += dt
            sleep_rate.sleep()



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
    parser.add_argument('-j', '--joint_index', type=int,
                        help='joint you want to dither')
    args = parser.parse_args(argv)

    ral = crtk.ral('dvrk_arm_test')
    application = PositionDithering(ral, args.arm, args.dithering_amplitude, args.dithering_frequency, args.joint_index, args.period)
    ral.on_shutdown(application.on_shutdown)
    ral.spin_and_execute(application.run)


