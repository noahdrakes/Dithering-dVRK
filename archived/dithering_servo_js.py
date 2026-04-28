import argparse
import time
import sys
import crtk
import math
import numpy
import rclpy

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
            (math.radians(-10.0), math.radians(10.0)),  # Joint 1
            (math.radians(-10.0), math.radians(10.0)),  # Joint 2
            (0.050, 0.200),  # Joint 3
            # (-1.57, 1.57),  # Joint 4
            # (-3.14, 3.14),  # Joint 5
            # (-1.0, 1.0)  # Joint 6
        ]

        # self.gc = numpy.zeros(6)
        # self.gc_sub = self.ral.subscriber(f'/{arm_name}/gravity_compensation/setpoint_js', JointState, self.gravity_callback)

        time.sleep(0.2)

    # def gravity_callback(self, msg):
    #     self.gc = numpy.array(msg.effort)

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
            goal[1] = math.radians(0.0)
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

        jf_setpoint = numpy.zeros(6)

        pos_hist = []
        vel_hist = []
        eff_hist = []
        command_hist = []
        gc_hist = []

        print('> press Enter to start dithering signal for joint {}:'.format(self.joint_index))
        print('  frequency: {} Hz'.format(self.dith_freq))
        input('  amplitude: {} Nm'.format(self.dith_ampl))

        while not self.ral.is_shutdown():
            jp_measured, _ = self.arm.measured_jp()

            # safety stop if too high angles are achieved
            self.check_joint_limits(jp_measured, self.JOINT_LIMITS)

            jf_measured, _ = self.arm.measured_jf()

            # try subtraction
            # eff_hist.append(jf_measured[self.joint_index])
            # command_hist.append(jf_setpoint[self.joint_index])
            # gc_hist.append(self.gc[self.joint_index])


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

            # if t >= 10.0:
            #     pos -= 0.0000025    # velocity = 0.0000025 rad/ms --> 0.0025 rad/s
            #     vel = - 0.0025

            # ROS 2 CRTK exposes joint position/velocity and effort commands on
            # separate topics (servo_jp and servo_jf), not combined servo_js.
            self.arm.servo_jp(jp_setpoint, jv_setpoint)
            self.arm.servo_jf(jf_setpoint)

            # pos_hist.append(jp_setpoint[self.joint_index])
            # vel_hist.append(jv_setpoint[self.joint_index])
            # eff_hist.append(jf_setpoint[self.joint_index])

            if t > 20.0:
                self.dith_off = True

            t += dt
            sleep_rate.sleep()

        # eff_hist = numpy.array(eff_hist)
        # command_hist = numpy.array(command_hist)
        #
        # plt.figure()
        # plt.plot(eff_hist)
        # plt.plot(eff_hist - command_hist - gc_hist)
        # plt.grid()
        # plt.ylabel('measured_js - dithering - gravity [Nm]')
        # plt.xlabel('sample [-]')
        # # plt.plot(command_hist)
        # # fig, axs = plt.subplots(3, 1, sharex=True)
        # # axs[0].plot(pos_hist, color='purple')
        # # axs[1].plot(vel_hist, color='green')
        # # axs[2].plot(eff_hist)
        # plt.show()


    def run(self):
        self.home()
        self.dithering()


    def on_shutdown(self):
        print ('>> illustrating user defined shutdown callback')


if __name__ == '__main__':
    # initialize ROS 2 and strip ROS args before argparse processing
    rclpy.init(args = sys.argv[1:])  # skip argv[0], script name
    argv = rclpy.utilities.remove_ros_args(sys.argv[1:])

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
    try:
        ral.spin_and_execute(application.run)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
