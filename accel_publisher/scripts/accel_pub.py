#!/usr/bin/env python3
import serial
import time
import struct

import rospy
from geometry_msgs.msg import Vector3Stamped

# global variables for time management
t0_sensor = None
t0_ros = None

# variables for serial communication
ARDUINO_PORT = '/dev/ttyACM0'
ARDUINO_BAUDRATE = 230400
DATA_NUM = 3    # 3-axis accelerometer
FLOAT_SIZE = 4  # floats byte number
DATA_SIZE = (DATA_NUM * FLOAT_SIZE) + 4
fmt = f'={DATA_NUM}fL'


def convert_timestamp(micros):
    """Converts arduino micros() in rospy.Time."""
    global t0_sensor, t0_ros
    if t0_sensor is None:
        t0_sensor = micros
        t0_ros = rospy.Time.now()
    # compute time passed from the beginning of the acquisition in seconds
    dt = (micros - t0_sensor) * 1e-6
    return t0_ros + rospy.Duration.from_sec(dt)


def main():
    rospy.init_node("accelerometer_publisher", anonymous=True)
    pub = rospy.Publisher("/accelerometer/data", Vector3Stamped, queue_size=10)
    # rate = rospy.Rate(100)  # force the frequency to 100 Hz

    # input("> Press Enter to start acquisition...")
    arduino.write(b'S')
    print("Reading data from serial port... Press CTRL+C to stop.")


    while not rospy.is_shutdown():
        try:
            sync = arduino.read(1)
            if sync != b'\xAA':
                continue

            raw = arduino.read(DATA_SIZE)

            if len(raw) == DATA_SIZE:
                # unpack sensor data
                acc_x, acc_y, acc_z, micros = struct.unpack(fmt, raw)
                # create ROS message
                msg = Vector3Stamped()
                msg.header.stamp = convert_timestamp(micros)
                msg.header.frame_id = "accelerometer_link"
                msg.vector.x = acc_x
                msg.vector.y = acc_y
                msg.vector.z = acc_z
                pub.publish(msg)
                # rate.sleep()

        except Exception as e:
            print(f"\n[ACC] Error: {e}")


if __name__ == "__main__":
    try:
        arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=0.01)
        time.sleep(2)
        arduino.reset_input_buffer()
        time.sleep(0.5)
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        if arduino.is_open:
            arduino.close()
            print("\nSerial closed.")
