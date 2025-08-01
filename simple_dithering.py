#!/usr/bin/env python

import argparse
import crtk
import dvrk
import math
import numpy
import PyKDL
import sys
import matplotlib.pyplot as plt
import scipy

if sys.version_info.major < 3:
    input = raw_input


class DitheringSignal:
    def __init__(self, ral, arm_name, period):
        print('-> configuring class for {}'.format(arm_name))
        self.ral = ral
        self.period = period
        self.arm = dvrk.psm(ral = ral,
                            arm_name = arm_name)

    # homing example
    def home(self):
        self.ral.check_connections()

        print('-> starting enable')
        if not self.arm.enable(10):
            sys.exit('  ! failed to enable within 10 seconds')
        print('-> starting home')
        if not self.arm.home(10):
            sys.exit('  ! failed to home within 10 seconds')

    def start_position(self):
        # go to zero position, make sure 3rd joint is past cannula
        print('-> move to starting position: [0, 0, 0.12, 0, 0, 0] with jaw closed')
        jp, _ = self.arm.setpoint_jp()
        goal = numpy.copy(jp)
        goal.fill(0)
        goal[2] = 0.12

        self.arm.move_jp(goal).wait()
        self.arm.jaw.close().wait()

    def dithering(self, joint_index, freq, ampl):
        input('press Enter to start dithering...')
        print('-> dithering signal started for joint {}:'.format(joint_index))
        print('   frequency: {} Hz\n   amplitude: {} Nm'.format(freq, ampl))

        duration = 15  # seconds
        samples = int(duration / self.period)
        t = numpy.linspace(0, duration, samples)
        square_wave = scipy.signal.square(2 * math.pi * freq * t)
        # sine = math.sin(2.0 * math.pi * freq * t)
        fs = int(1 / self.period)

        # ramp to smooth the transition
        ramp = numpy.linspace(0, 1, 2*freq)
        j = 0
        for i in range(2*freq):
            while square_wave[j] == 1:
                square_wave[j] *= ramp[i]
                j += 1

            while square_wave[j] == -1:
                square_wave[j] *= ramp[i]
                j += 1

        sleep_rate = self.ral.create_rate(1.0 / self.period)

        center = numpy.zeros(samples)
        mean = []
        up_level = []
        low_level = []
        goal_hist = []
        j_pos = []

        for i in range(samples):
            measured_jf, _ = self.arm.measured_jf()
            measured_jp, _ = self.arm.measured_jp()
            center[i] = measured_jf[joint_index]
            j_pos.append(measured_jp[joint_index])

            if i >= fs:
                mean.append(numpy.mean(center[i - fs:i]))
                up_level.append(numpy.mean(center[i - fs:i][center[i - fs:i] > mean[-1]]))
                low_level.append(numpy.mean(center[i - fs:i][center[i - fs:i] < mean[-1]]))

            goal = numpy.copy(measured_jf)
            goal.fill(0.0)
            goal[joint_index] = square_wave[i] * ampl
            goal_hist.append(goal[joint_index])
            self.arm.servo_jf(goal)
            sleep_rate.sleep()
        
        plt.figure()
        plt.subplot(212)
        plt.plot(t, center, label='measured jf')
        plt.plot(t[fs:], mean, label='mean jf')
        plt.plot(t[fs:], up_level, label='up jf')
        plt.plot(t[fs:], low_level, label='low jf')
        plt.legend()
        plt.grid()
        plt.ylabel('Joint {} effort [Nm]'.format(joint_index))
        plt.subplot(211)
        plt.plot(t, goal_hist, color='pink', label='dithering signal')
        plt.legend()
        plt.grid()
        plt.xlabel('Time [s]')
        plt.ylabel('Dithering signal [Nm]')

        # plt.figure()
        # plt.plot(t, j_pos)
        # plt.grid()
        # plt.xlabel('Time [s]')
        # plt.ylabel('Joint {} [rad]')

        plt.show()

    def stop(self):
        final_jp, _ = self.arm.measured_jp()
        print('-> dithering signal stopped')
        self.arm.move_jp(final_jp).wait()


    # main method
    def run(self):
        self.home()
        self.start_position()
        self.dithering(0, 10, 0.3)
        self.stop()

if __name__ == '__main__':
    ral = crtk.ral('dvrk_dithering_test')
    dithering = DitheringSignal(ral, 'PSM1', 0.01)
    ral.spin_and_execute(dithering.run)