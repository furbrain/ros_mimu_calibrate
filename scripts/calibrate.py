import numpy as np

import rospy
from mag_cal import Sensor
from scipy.spatial import KDTree
from sensor_msgs.msg import PointCloud2, Imu, MagneticField
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from mimu_calibrate.utils import create_cloud_xyz32, vector_as_list

ROLLING_BUF_SIZE = 25


class CalibrateNode:
    def __init__(self):
        self.cal_pub = rospy.Publisher("calibrated_magnetic_points", PointCloud2, queue_size=5)
        self.subs = [
            rospy.Subscriber("all_magnetic_points", PointCloud2, self.all_mag_cb, queue_size=5),
            rospy.Subscriber("paired_magnetic_points", PointCloud2, self.paired_mag_cb, queue_size=5),
            rospy.Subscriber("paired_accel_points", PointCloud2, self.paired_accel_cb, queue_size=5),
        ]
        self.cal_svr = rospy.Service("calibrate", Trigger, self.handle_calibrate)
        self.all_mag_cloud = np.empty((0,3))
        self.paired_mag_cloud = np.empty((0,3))
        self.paired_accel_cloud = np.empty((0,3))

    def all_mag_cb(self, mag_data: PointCloud2):
        self.all_mag_cloud= np.frombuffer(mag_data.data, dtype=np.float32).reshape((-1,3))

    def paired_mag_cb(self, mag_data: PointCloud2):
        self.paired_mag_cloud= np.frombuffer(mag_data.data, dtype=np.float32).reshape((-1,3))

    def paired_accel_cb(self, mag_data: PointCloud2):
        self.paired_accel_cloud= np.frombuffer(mag_data.data, dtype=np.float32).reshape((-1,3))

    @staticmethod
    def log_gravs(data, desc:str):
        strengths = np.linalg.norm(data, axis=1)
        mean = np.mean(strengths)
        std = np.std(strengths)
        rospy.loginfo(f"{desc} Strengths: {strengths}")
        rospy.loginfo(f"{desc} Mean: {mean}")
        rospy.loginfo(f"{desc} std: {std}")

    def handle_calibrate(self, req: TriggerRequest):
        print("handle_calibrate called")
        mag_sensor = Sensor(axes=rospy.get_param("~mag_axes", "+X+Y+Z"))
        mag_result = mag_sensor.fit_ellipsoid(self.all_mag_cloud) # convert back to Tesla from microtesla
        accel_sensor = Sensor(axes=rospy.get_param("~accel_axes", "+X+Y+Z"))
        gyro_axes = rospy.get_param("~gyro_axes", "+X+Y+Z")
        accel_results = accel_sensor.fit_ellipsoid(self.paired_accel_cloud)
        # scale sensor back to tesla units
        gain = np.mean(np.linalg.norm(mag_sensor.transform, axis=0))
        mag_sensor.transform /= gain
        self.log_gravs(self.paired_accel_cloud,"Raw")
        gain = np.mean(np.linalg.norm(accel_sensor.transform, axis=0))
        accel_sensor.transform /= gain
        self.log_gravs(accel_sensor.apply(self.paired_accel_cloud),"Calibrated")
        gravity_strengths = np.linalg.norm(accel_sensor.apply(self.paired_accel_cloud), axis=1)
        accel_sensor.transform *= 9.81 / np.mean(gravity_strengths)
        self.log_gravs(accel_sensor.apply(self.paired_accel_cloud),"standardised")
        calibrated_points = mag_sensor.apply(self.all_mag_cloud)
        self.cal_pub.publish(create_cloud_xyz32(calibrated_points * 1e6))
        mag_dct = mag_sensor.as_dict()
        del mag_dct['field_avg']
        del mag_dct['field_std']
        rospy.set_param("calibration/magnetic/sensor", mag_dct)
        accel_dct = accel_sensor.as_dict()
        del accel_dct['field_avg']
        del accel_dct['field_std']
        rospy.set_param("calibration/accel/sensor", accel_dct)
        rospy.set_param("calibration/gyro/axes", gyro_axes)
        return TriggerResponse(True, "Calibration completed")

if __name__ == '__main__':
    rospy.init_node("calibrater")
    CalibrateNode()
    rospy.spin()
