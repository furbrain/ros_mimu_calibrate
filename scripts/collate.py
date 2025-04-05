from collections import deque
from typing import Optional

import numpy as np

import rospy
from scipy.spatial import KDTree
from sensor_msgs.msg import PointCloud2, Imu, MagneticField
from mimu_calibrate.utils import create_cloud_xyz32, vector_as_list
import message_filters

class MovementDetector:
    NUM_READINGS = 15
    MAX_READINGS = np.array([6*1e-6]*3 + [1.0]*3 + [0.1]*3)
    def __init__(self):
        self.index = 0
        self.matrix = np.zeros((self.NUM_READINGS, 9))
        self.full = False

    def add(self, values):
        self.matrix[self.index] = values
        self.index += 1
        std = np.std(self.matrix, axis=0)
        rospy.loginfo(f"std: {std}")
        if self.index >= self.NUM_READINGS:
            self.full = True
            self.index = 0
        return (std > self.MAX_READINGS).any()

class UniqueCloud:
    def __init__(self, proximity: float, pub_name: str):
        self.proximity = proximity
        self.points = []
        self.tree = None
        self.pub = rospy.Publisher(pub_name, PointCloud2, queue_size=5)

    def add_point(self, point):
        self.points.append(point)
        if self.proximity != 0:
            self.tree = KDTree(self.points)

    def novel(self, point):
        if self.proximity==0 or self.tree is None:
            return True
        d, i = self.tree.query(point, distance_upper_bound=self.proximity)
        return d > self.proximity

    def publish(self, stamp:Optional[rospy.Time] = None):
        cloud = create_cloud_xyz32(np.array(self.points))
        self.pub.publish(cloud)

class CollateNode:
    SAMPLE_INTERVAL = rospy.Duration.from_sec(0.1)
    def __init__(self):
        self.proximity = rospy.get_param("~proximity", 5e-6)
        self.mag_sub = rospy.Subscriber("magnetic_raw", MagneticField, self.mag_callback, queue_size=5)
        self.paired_sub = message_filters.ApproximateTimeSynchronizer(
            (message_filters.Subscriber("imu_raw", Imu),
            message_filters.Subscriber("magnetic_raw", MagneticField)),
            queue_size=10, slop=0.1)
        self.paired_sub.registerCallback(self.paired_callback)
        self.all_mag_cloud = UniqueCloud(proximity=self.proximity, pub_name="~all_magnetic_points")
        self.paired_mag_cloud = UniqueCloud(proximity=self.proximity, pub_name="~paired_magnetic_points")
        self.paired_accel_cloud = UniqueCloud(proximity=1, pub_name="~paired_accel_points")
        self.last_ts = rospy.Time()
        self.movement_detector = MovementDetector()

    def mag_callback(self, mag_data: MagneticField):
        point = vector_as_list(mag_data.magnetic_field)
        if self.all_mag_cloud.novel(point):
            self.all_mag_cloud.add_point(point)
        self.all_mag_cloud.publish()

    def paired_callback(self, imu_data: Imu, magnetic_data: MagneticField):
        stamp = imu_data.header.stamp
        rospy.loginfo(f"got paired data: {stamp}")
        if self.last_ts + self.SAMPLE_INTERVAL < stamp:
            rospy.loginfo("new data")
            if self.last_ts + self.SAMPLE_INTERVAL * 2 < stamp:
                self.last_ts = stamp
            else:
                self.last_ts += self.SAMPLE_INTERVAL
            mag = vector_as_list(magnetic_data.magnetic_field)
            accel = vector_as_list(imu_data.linear_acceleration)
            gyro = vector_as_list(imu_data.angular_velocity)
            movement = self.movement_detector.add(mag + accel + gyro)
            if not movement:
                if self.paired_mag_cloud.novel(mag) and self.paired_accel_cloud.novel(accel):
                    self.paired_mag_cloud.add_point(mag)
                    self.paired_accel_cloud.add_point(accel)
            self.paired_mag_cloud.publish(stamp=stamp)
            self.paired_accel_cloud.publish(stamp=stamp)


if __name__ == '__main__':
    rospy.init_node("collate")
    CollateNode()
    rospy.spin()
