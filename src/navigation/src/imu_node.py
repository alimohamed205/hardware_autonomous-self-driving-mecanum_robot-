#!/usr/bin/env python3

import time
import smbus
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_multiply, quaternion_about_axis, quaternion_from_euler
from mpu_6050_driver.registers import PWR_MGMT_1, ACCEL_XOUT_H, ACCEL_YOUT_H, ACCEL_ZOUT_H, TEMP_H,\
    GYRO_XOUT_H, GYRO_YOUT_H, GYRO_ZOUT_H

ADDR = None
bus = None
IMU_FRAME = None

# Calibration values for your IMU, adjust as needed
ACCEL_X_OFFSET = 0
ACCEL_Y_OFFSET = 0
ACCEL_Z_OFFSET = 0
GYRO_X_OFFSET = 0
GYRO_Y_OFFSET = 0
GYRO_Z_OFFSET = 0

def read_word(adr):
    high = bus.read_byte_data(ADDR, adr)
    low = bus.read_byte_data(ADDR, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def publish_imu(timer_event):
    global last_time, last_orientation
    
    imu_msg = Imu()
    imu_msg.header.frame_id = IMU_FRAME

    # Read the acceleration values
    accel_x = (read_word_2c(ACCEL_XOUT_H) - ACCEL_X_OFFSET) / 16384.0
    accel_y = (read_word_2c(ACCEL_YOUT_H) - ACCEL_Y_OFFSET) / 16384.0
    accel_z = (read_word_2c(ACCEL_ZOUT_H) - ACCEL_Z_OFFSET) / 16384.0

    if np.isnan(accel_x) or np.isnan(accel_y) or np.isnan(accel_z):
        rospy.logwarn("IMU acceleration contains NaN values")
        return

    # Calculate the gravity vector and normalize
    accel_normalized = np.array([accel_x, accel_y, accel_z])
    accel_normalized /= np.linalg.norm(accel_normalized)

    # Read the gyro values
    gyro_x = (read_word_2c(GYRO_XOUT_H) - GYRO_X_OFFSET) / 131.0
    gyro_y = (read_word_2c(GYRO_YOUT_H) - GYRO_Y_OFFSET) / 131.0
    gyro_z = (read_word_2c(GYRO_ZOUT_H) - GYRO_Z_OFFSET) / 131.0

    if np.isnan(gyro_x) or np.isnan(gyro_y) or np.isnan(gyro_z):
        rospy.logwarn("IMU gyro contains NaN values")
        return

    # Calculate time difference since last update
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()
    last_time = current_time

    # Calculate orientation quaternion using gyro integration
    orientation_change = quaternion_about_axis(dt * gyro_z, [0, 0, 1])  # Using only yaw for simplicity
    current_orientation = quaternion_multiply(last_orientation, orientation_change)
    last_orientation = current_orientation

    # Load up the IMU message
    imu_msg.orientation.x = current_orientation[0]
    imu_msg.orientation.y = current_orientation[1]
    imu_msg.orientation.z = current_orientation[2]
    imu_msg.orientation.w = current_orientation[3]

    imu_msg.angular_velocity.x = gyro_x
    imu_msg.angular_velocity.y = gyro_y
    imu_msg.angular_velocity.z = gyro_z

    imu_msg.linear_acceleration.x = accel_x
    imu_msg.linear_acceleration.y = accel_y
    imu_msg.linear_acceleration.z = accel_z

    imu_msg.header.stamp = rospy.Time.now()

    imu_pub.publish(imu_msg)

if __name__ == '__main__':
    rospy.init_node('imu_node')

    bus = smbus.SMBus(rospy.get_param('~bus', 1))
    ADDR = rospy.get_param('~device_address', 0x68)
    if isinstance(ADDR, str):
        ADDR = int(ADDR, 16)

    IMU_FRAME = rospy.get_param('~imu_frame', 'base_link')

    bus.write_byte_data(ADDR, PWR_MGMT_1, 0)

    imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
    
    # Initialize orientation variables
    last_orientation = quaternion_from_euler(0, 0, 0)  # Initial orientation is identity quaternion
    last_time = rospy.Time.now()

    imu_timer = rospy.Timer(rospy.Duration(0.02), publish_imu)
    rospy.spin()
