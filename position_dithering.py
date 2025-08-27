
import argparse
import time
import sys
import crtk
import math
import scipy
import matplotlib.pyplot as plt
import numpy
import rospy
from sensor_msgs.msg import JointState
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
        self.crtk_utils.add_servo_jf()

    def ral(self):
        return self.__ral

# example of application using arm.py
class example_application:

    # configuration
    def __init__(self, ral, arm_name, dith_ampl, dith_freq, period = 0.01):
        print('> configuring dvrk_arm_test for {}'.format(arm_name))
        self.ral = ral
        self.arm_name = arm_name
        self.dith_ampl = dith_ampl
        self.dith_freq = dith_freq
        self.period = period
        self.jp_init = None
        self.arm = device(ral = ral,
                          arm_name = arm_name)
        time.sleep(0.2)


    # homing example
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


    def run_servo_jp(self):

        joint_index = 0
        freq = self.dith_freq
        ampl = self.dith_ampl

        print('> dithering signal for joint {}:'.format(joint_index))
        print('  frequency: {} Hz\n  amplitude: {} Nm'.format(freq, ampl))
        input('press Enter to start dithering...')

        duration = 45  # seconds
        samples = int(duration / self.period)
        t = numpy.linspace(0, duration, samples)
        # square_wave = scipy.signal.square(2 * math.pi * freq * t)
        sine_wave = numpy.sin(2.0 * math.pi * freq * t)

        # goal_hist = []

        # ramp to smooth the transition
        transition = numpy.linspace(0, 1, int(5 * freq))

        j = 0
        # for i in range(len(transition)):
        #     while square_wave[j] == 1:
        #         square_wave[j] *= transition[i]
        #         j += 1
        #
        #     while square_wave[j] == -1:
        #         square_wave[j] *= transition[i]
        #         j += 1

        for i in range(len(transition)):
            while sine_wave[j] >= 0:
                sine_wave[j] *= transition[i]
                j += 1

            while sine_wave[j] < 0:
                sine_wave[j] *= transition[i]
                j += 1

        sleep_rate = self.ral.create_rate(1.0 / self.period)

        for i in range(samples):

            # safety stop if too high angles are achieved
            jp, _ = self.arm.measured_jp()

            if self.jp_init is None:
                self.jp_init = jp[joint_index]

            if jp[joint_index]>math.radians(100) or jp[joint_index]<-math.radians(100):
                self.arm.disable()
                print('ROM limit reached')
                self.ral.shutdown()

            goal = numpy.copy(jp)
            goal[joint_index] = sine_wave[i] * ampl + self.jp_init
            # goal_hist.append(goal[joint_index])
            self.arm.servo_jp(goal)
            sleep_rate.sleep()

        # plt.figure()
        # plt.plot(goal_hist)
        # plt.show()


    def prepare_cartesian(self):
        # make sure the camera is past the cannula and tool vertical
        jp, ts = self.arm.setpoint_jp()
        goal = numpy.copy(jp)
        if ((self.arm_name.endswith('PSM1')) or (self.arm_name.endswith('PSM2'))
                or (self.arm_name.endswith('PSM3')) or (self.arm_name.endswith('ECM'))):
            print('  > preparing for cartesian motion')

            # set in position joint mode
            goal[0] = math.radians(90)
            goal[1] = -1
            goal[2] = 0.12
            goal[3] = 0.0
            goal[4] = 0.0
            goal[5] = 0.0

            input('   > press enter to move to start position: {}'.format(goal))

            self.arm.move_jp(goal).wait()
            print('  < ready for cartesian mode')

    def stop(self):
        time.sleep(1)
        measured_jp, _ = self.arm.setpoint_jp()
        self.arm.move_jp(measured_jp).wait()

    # main method
    def run(self):
        self.home()
        self.run_servo_jp()
        # self.stop()

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
    application = example_application(ral, args.arm, args.dithering_amplitude, args.dithering_frequency, args.period)
    ral.on_shutdown(application.on_shutdown)
    ral.spin_and_execute(application.run)
