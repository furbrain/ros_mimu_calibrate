import numpy as np

import rospy
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from sensor_msgs.msg import Imu, MagneticField

from mimu_calibrate.utils import vector_as_list


class VarianceCalc:
    def __init__(self):
        self.recording = False
        self.mag_data = []
        self.accel_data = []
        self.gyro_data = []

        self.imu_sub = rospy.Subscriber("imu_raw", Imu, self.imu_callback)
        self.mag_sub = rospy.Subscriber("magnetic_raw", MagneticField, self.mag_callback)

        self.start_recording_srv = rospy.Service("start_recording", Trigger, self.start_recording)
        self.get_variance_srv = rospy.Service("get_variance", Trigger, self.get_variance)

    def imu_callback(self, imu_data: Imu):
        if self.recording:
            self.accel_data.append(vector_as_list(imu_data.linear_acceleration))
            self.gyro_data.append(vector_as_list(imu_data.angular_velocity))

    def mag_callback(self, mag_data: MagneticField):
        if self.recording:
            self.mag_data.append(vector_as_list(mag_data.magnetic_field))

    def start_recording(self, request: TriggerRequest):
        self.recording = True
        self.mag_data = []
        self.accel_data = []
        self.gyro_data = []
        return TriggerResponse(True, "Recording started")

    def get_variance(self, request: TriggerRequest):
        self.recording = False
        mag_cov = np.cov(self.mag_data, rowvar=False)
        accel_cov = np.cov(self.accel_data, rowvar=False)
        gyro_cov = np.cov(self.gyro_data, rowvar=False)
        gyro_bias = np.mean(self.gyro_data, axis=0)
        rospy.set_param("calibration/magnetic/cov", mag_cov.tolist())
        rospy.set_param("calibration/accel/cov", accel_cov.tolist())
        rospy.set_param("calibration/gyro/cov", gyro_cov.tolist())
        rospy.set_param("calibration/gyro/bias", gyro_bias.tolist())
        return TriggerResponse(True, "Variance calculated")

if __name__=="__main__":
    rospy.init_node("variance")
    calc=VarianceCalc()
    rospy.spin()