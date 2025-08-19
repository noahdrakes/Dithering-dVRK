
import argparse
import time
import sys
import crtk
import math

import matplotlib.pyplot as plt
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

# example of application using arm.py
class example_application:

    # configuration
    def __init__(self, ral, arm_name, acc, seg_duration, period = 0.01):
        print('> configuring dvrk_arm_test for {}'.format(arm_name))
        self.ral = ral
        self.arm_name = arm_name
        self.acc = acc
        self.seg_duration = seg_duration
        self.period = period
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

    # direct joint control example
    def run_servo_jp(self):

        joint_index = 0
        input('press Enter to start movement of joint {} with acc = {} rad/s^2...'.format(joint_index, self.acc))

        jp_init, _ = self.arm.setpoint_jp()
        jv, _ = self.arm.setpoint_jv()
        jv.fill(0.0)
        # q = jp_init[joint_index]
        q_hist = []
        v_hist = []
        v = 0.0
        a = self.acc
        dt = self.period

        segment_duration = self.seg_duration
        steps_per_seg = int(segment_duration / dt)
        total_duration = segment_duration * 6.0
        total_steps = int(total_duration / dt)

        sleep_rate = self.ral.create_rate(1.0 / self.period)

        for step in range(total_steps):

            if step >= steps_per_seg / 2:
                if step % steps_per_seg == 0:
                    a = - a                             # change acceleration sign at every segment
                elif step % (steps_per_seg/2) == 0:
                    time.sleep(1)                       # wait 1s to be sure to have v=0

                jp_init[joint_index] += v * dt + 0.5 * a * dt ** 2
                v += a * dt

            # jp = jp_init
            # jv.fill(0.0)
            # jp[joint_index] = q
            jv[joint_index] = v
            q_hist.append(jp_init[joint_index])
            v_hist.append(v)
            self.arm.servo_jp(jp_init)

            sleep_rate.sleep()

        #
        # plt.figure()
        # plt.plot(q_hist)
        # plt.show()


    # utility to position tool/camera deep enough before cartesian examples
    def prepare_cartesian(self):
        # make sure the camera is past the cannula and tool vertical
        jp, ts = self.arm.setpoint_jp()
        goal = numpy.copy(jp)
        set_goal = input('Do you want to set the start position of {}?  [y/n]  -> '.format(self.arm_name))
        print('Suggestion: ', str(0.5 * self.acc * self.seg_duration * self.seg_duration / 2 / 2))
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

    # main method
    def run(self):
        self.home()
        self.run_servo_jp()

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
    parser.add_argument('-A', '--acceleration', type=float, required=True,
                        help = 'acceleration value for selected joint')
    parser.add_argument('-d', '--segment_duration', type=float, required=True,
                        help='duration of the segment with constant acceleration')
    parser.add_argument('-p', '--period', type =float, default = 0.01,
                        help = 'period used for loops using servo commands')
    args = parser.parse_args(argv)

    ral = crtk.ral('dvrk_arm_test')
    application = example_application(ral, args.arm, args.acceleration, args.segment_duration, args.period)
    ral.on_shutdown(application.on_shutdown)
    ral.spin_and_execute(application.run)
