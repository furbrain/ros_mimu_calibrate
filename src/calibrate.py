import ctypes
import struct

import rospy
import numpy as np

import rospy
from geometry_msgs.msg import Vector3
from scipy.spatial import KDTree
from sensor_msgs.msg import PointCloud2, PointField, Imu, MagneticField
from std_msgs.msg import Header
from mimu_calibrate.srv import CalibrateRequest, CalibrateResponse, Calibrate

ROLLING_BUF_SIZE = 25


def create_cloud_xyz32(points: np.ndarray):
    """
    Create a L{sensor_msgs.msg.PointCloud2} message with 3 float32 fields (x, y, z).

    @param points: The point cloud points.
    @type  points: np.ndarray
    @return: The point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """
    header = Header(frame_id="map")
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]
    data = points.astype(np.float32).tobytes()
    return PointCloud2(header=header,
                       height=1,
                       width=points.shape[0],
                       is_dense=False,
                       is_bigendian=False,
                       fields=fields,
                       point_step=12,
                       row_step=len(data),
                       data=data)


class CalibrateNode:
    def __init__(self):
        self.pub = rospy.Publisher("magnetic_points", PointCloud2, queue_size=5)
        self.imu_sub = rospy.Subscriber("/imu/imu_raw", Imu, self.imu_callback, queue_size=5)
        self.mag_sub = rospy.Subscriber("/imu/magnetic_raw", MagneticField, self.mag_callback, queue_size=5)
        self.cal_svr = rospy.Service("calibrate", Calibrate, self.handle_calibrate)
        self.mag_points = np.empty((0,3), dtype=np.float32)
        self.mag_kdtree: KDTree = None
        self.grav_points = np.empty((0,3))
        self.mag_grav_points = np.empty((0,3))
        self.rolling_mag_data = np.full((ROLLING_BUF_SIZE,3), np.nan)
        self.rolling_grav_data = np.full((ROLLING_BUF_SIZE,3), np.nan)
        self.rolling_gyro_data = np.full((ROLLING_BUF_SIZE,3), np.nan)
        self.rolling_index = 0

    @staticmethod
    def as_list(d: Vector3):
        return [d.x,d.y,d.z]

    def imu_callback(self,imu_data: Imu):
        pass

    def add_mag_point(self, point: np.ndarray):
        self.mag_points = np.append(self.mag_points,np.expand_dims(point,0), axis=0)
        print(self.mag_points)
        self.mag_kdtree = KDTree(self.mag_points)

    def mag_callback(self, mag_data: MagneticField):
        point = np.array(self.as_list(mag_data.magnetic_field), dtype=np.float32)*1e6
        if self.mag_kdtree is None:
            self.add_mag_point(point)
        else:
            d, i = self.mag_kdtree.query(point, distance_upper_bound=5)
            if d > 5:
                self.add_mag_point(point)
        print(self.mag_points.shape[0])
        #self.mag_points.append(self.as_list(mag_data.magnetic_field))
        cloud = create_cloud_xyz32(self.mag_points)
        #print(cloud)
        self.pub.publish(cloud)

    def handle_calibrate(self, req: CalibrateRequest):
        print("handle_calibrate called")
        return CalibrateResponse(True)

if __name__ == '__main__':
    rospy.init_node("imu_calibrate")
    CalibrateNode()
    rospy.spin()
