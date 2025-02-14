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
        self.all_mag_points = rospy.Subscriber("all_magnetic_points", PointCloud2, self.mag_callback, queue_size=5)
        self.cal_svr = rospy.Service("calibrate", Trigger, self.handle_calibrate)
        self.mag_cloud = np.empty((0,3))

    def imu_callback(self,imu_data: Imu):
        pass

    def mag_callback(self, mag_data: PointCloud2):
        self.mag_cloud= np.frombuffer(mag_data.data, dtype=np.float32).reshape((-1,3))

    def handle_calibrate(self, req: TriggerRequest):
        print("handle_calibrate called")
        sensor = Sensor(axes=rospy.get_param("~mag_axes", "+X+Y+Z"))
        result = sensor.fit_ellipsoid(self.mag_cloud) # convert back to Tesla from microtesla
        # scale sensor back to tesla units
        gain = np.mean(np.linalg.norm(sensor.transform, axis=0))
        sensor.transform /= gain
        print(result)
        print(sensor.transform)
        print(sensor.centre)
        calibrated_points = sensor.apply(self.mag_cloud)
        self.cal_pub.publish(create_cloud_xyz32(calibrated_points * 1e6))
        dct = sensor.as_dict()
        del dct['field_avg']
        del dct['field_std']
        print(dct)
        rospy.set_param("mag_cal", dct)
        return TriggerResponse(True, "Calibration completed")

if __name__ == '__main__':
    rospy.init_node("calibrate")
    CalibrateNode()
    rospy.spin()
