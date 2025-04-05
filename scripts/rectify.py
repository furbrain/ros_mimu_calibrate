from itertools import chain

import numpy as np

import rospy
from mag_cal import Sensor, Axes
from sensor_msgs.msg import MagneticField, Imu
from mimu_calibrate.utils import vector_as_list, list_as_vector

def sensor_from_param(param: str):
    dct = rospy.get_param(param)
    dct['field_avg'] = None
    dct['field_std'] = None
    return Sensor.from_dict(dct)

def flatten(lst: list):
    return list(chain.from_iterable(lst))


class RectifyMagNode:
    def __init__(self):
        self.pub = rospy.Publisher("magnetic_rect", MagneticField, queue_size=5)
        self.mag_sub = rospy.Subscriber("magnetic_raw", MagneticField, self.mag_callback, queue_size=5)
        self.sensor = sensor_from_param("calibration/magnetic/sensor")
        self.mag_cov = flatten(rospy.get_param("calibration/magnetic/cov", [0.0]*9))
        self.time_offset = rospy.Duration(rospy.get_param("calibration/time_offset",0.0))

    def mag_callback(self, point: MagneticField):
        header = point.header
        header.stamp += self.time_offset
        rect_point = self.sensor.apply(np.array(vector_as_list(point.magnetic_field)))
        self.pub.publish(header=header,
                         magnetic_field = list_as_vector(rect_point),
                         magnetic_field_covariance = self.mag_cov)

class RectifyImuNode:
    def __init__(self):
        self.pub = rospy.Publisher("imu_rect", Imu, queue_size=5)
        self.mag_sub = rospy.Subscriber("imu_raw", Imu, self.mag_callback, queue_size=5)
        self.time_offset = rospy.Duration(rospy.get_param("calibration/time_offset",0.0))
        self.accel_sensor = sensor_from_param("calibration/accel/sensor")
        self.accel_cov = flatten(rospy.get_param("calibration/accel/cov", [0.0]*9))
        self.gyro_bias = np.array(rospy.get_param("calibration/gyro/bias", [0.0]*3))
        self.gyro_cov = flatten(rospy.get_param("calibration/gyro/cov", [0.0]*9))
        self.gyro_axes = Axes(rospy.get_param("calibration/gyro/axes","+X+Y+Z"))

    def mag_callback(self, point: Imu):
        header = point.header
        header.stamp += self.time_offset
        rect_point = self.accel_sensor.apply(np.array(vector_as_list(point.linear_acceleration)))
        rect_gyro = (np.array(vector_as_list(point.angular_velocity)) - self.gyro_bias)
        rect_gyro = self.gyro_axes.fix_axes(rect_gyro)
        self.pub.publish(header=header,
                         linear_acceleration = list_as_vector(rect_point),
                         linear_acceleration_covariance = self.accel_cov,
                         angular_velocity = list_as_vector(rect_gyro),
                         angular_velocity_covariance = self.gyro_cov)

if __name__=="__main__":
    rospy.init_node("mimu_rect")
    RectifyMagNode()
    RectifyImuNode()
    rospy.spin()
