#!/usr/bin/env python
# license removed for brevity
import serial
import rospy
from sensor_msgs.msg import Imu
import math
import tf
from median_filter import MedianFilter
from common_algos.constants import *
import traceback

um6_imu = Imu()

velocity_mf = MedianFilter(max_queue_size=5, min_value=0.0005, max_value=2.05)

def get_angle(section, coef=1.00424):
    data = ord(section[3]) + ord(section[4]) * 256 * 256 + ord(section[5]) * 256
    data = data - (data >> 23) * 1024 * 1024 * 16
    data = data * coef / 100
    return data

def get_velocity(section):
    data = ord(section[0]) + ord(section[1]) * 256 * 256 + ord(section[2]) * 256
    data = data - (data >> 23) * 1024 * 1024 * 16
    data = data * 2500.0 / 8388608
    return data

def check(section):
    sum = 0
    for i in range(6):
        sum = sum + ord(section[i])
    sum = sum % 256
    if sum == ord(section[6]):
        return True
    return False

def callback(data):
    global um6_imu
    um6_imu = data
    pass

def filter_velocity(data, min_value, max_value):
    if abs(data) < min_value:
        data = 0.
    elif abs(data) > max_value:
        # TODO
        data = None
    return data

def publish_costly_imu(debug, coef):
    if debug:
        pub_raw = rospy.Publisher(COSTLY_IMU_RAW_TOPIC, Imu, queue_size=10)
    global anguler_mf
    pub = rospy.Publisher(COSTLY_IMU_TOPIC, Imu, queue_size=10)
    rospy.sleep(0.2)
    #rospy.Subscriber("/imu/data", Imu, callback)
    ser = serial.Serial('/dev/costly_imu', 115200, timeout=0.5)
    #ser = serial.Serial('/dev/ttyUSB4', 115200, timeout=0.5)
    rate = rospy.Rate(115200) # 100hz
    #rate = rospy.Rate(10) # 100hz
    while not rospy.is_shutdown():
        try:
            head = ser.read(1)
            if ord(head[0]) == 0xdd:
                package = ser.read(7)
                if check(package) == False:
                    continue
                angle1 = (get_angle(package, coef) * math.pi / 180.0 + math.pi) % (2 * math.pi) - math.pi 
                velocity1 = get_velocity(package)  * math.pi / 180.0
                #angle1 = get_angle(package)
                angle = anguler_mf.filter(angle1, velocity1)
                #velocity = velocity_mf.filter(velocity1)
                velocity = filter_velocity(velocity1, 0.0005, 2.05)
                #rospy.loginfo("angle1: %s, angle:%s, velocity1: %s, velocity: %s", angle1, angle, velocity1, velocity)
                if angle is not None and velocity is not None:
                    q = tf.transformations.quaternion_from_euler(0, 0, -angle)
                    imu = Imu()
                    imu.header.frame_id = "base_link"
                    imu.header.stamp = rospy.Time.now()
                    imu.orientation.x = q[0]
                    imu.orientation.y = q[1]
                    imu.orientation.z = q[2]
                    imu.orientation.w = q[3]
                    # TODO reset covariance
                    imu.orientation_covariance = [1e-05, 0., 0., 0., 1e-5, 0., 0., 0., 1e-06]
                    imu.angular_velocity_covariance = [1e-05, 0., 0., 0., 1e-5, 0., 0., 0., 1e-06]
                    imu.angular_velocity.z = -velocity
                    pub.publish(imu)
                if debug:
                    q = tf.transformations.quaternion_from_euler(0, 0, -angle1)
                    imu = Imu()
                    imu.header.frame_id = "costly_imu_link"
                    imu.header.stamp = rospy.Time.now()
                    imu.orientation.x = q[0]
                    imu.orientation.y = q[1]
                    imu.orientation.z = q[2]
                    imu.orientation.w = q[3]
                    # TODO reset covariance
                    imu.orientation_covariance = [1e-05, 0., 0., 0., 1e-05, 0., 0., 0., 1e-06]
                    imu.angular_velocity_covariance = [1e-05, 0., 0., 0., 1e-5, 0., 0., 0., 1e-06]
                    imu.angular_velocity.z = -velocity1
                    pub_raw.publish(imu)                   
        except Exception as e:
            rospy.logerr("failed to publish imu data:%s", e.message)
            ser = serial.Serial('/dev/costlyimu', 115200, timeout=0.5)
        rate.sleep()

if __name__ == '__main__': 
    rospy.init_node('costly_imu_node') 
    angular_window = int(rospy.get_param("~angular_window", 15))
    coef = float(rospy.get_param("~coef", 1.00424))
    rospy.loginfo("angular_window: %s, coef:%.5f", angular_window, coef)
    debug = 1
    anguler_mf = MedianFilter(max_queue_size=angular_window, min_value=0.00001, max_value=3.15, max_update_interval=0.3, max_outlier_count=30)
    while not rospy.is_shutdown():
        try:
            publish_costly_imu(debug, coef)
        except Exception as e:
            rospy.logerr(traceback.format_exc())
            pass
