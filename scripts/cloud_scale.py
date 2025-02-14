import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2


class CloudScale:
    def __init__(self):
        self.scale = rospy.get_param("~scale",1e6)
        self.sub = rospy.Subscriber("in", PointCloud2, self.callback, queue_size=5)
        self.pub = rospy.Publisher("out", PointCloud2, queue_size=5)

    def callback(self, cloud:PointCloud2):
        nums = np.frombuffer(cloud.data, dtype=np.float32)
        nums = nums*self.scale
        cloud.data = nums.tobytes()
        self.pub.publish(cloud)

if __name__ == '__main__':
    rospy.init_node("cloud_scale")
    CloudScale()
    rospy.spin()
