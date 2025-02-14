import numpy as np

import rospy
from mag_cal import Sensor
from sensor_msgs.msg import MagneticField
from mimu_calibrate.utils import vector_as_list, list_as_vector

class RectifyMagNode:
    def __init__(self):
        self.pub = rospy.Publisher("magnetic_rect", MagneticField, queue_size=5)
        self.mag_sub = rospy.Subscriber("magnetic_raw", MagneticField, self.mag_callback, queue_size=5)
        cal_dct = rospy.get_param("mag_cal",None)
        if cal_dct is None:
            raise ValueError("param not found")
        cal_dct['field_avg'] = None
        cal_dct['field_std'] = None
        self.sensor = Sensor.from_dict(cal_dct)

    def mag_callback(self, point: MagneticField):
        rect_point = self.sensor.apply(np.array(vector_as_list(point.magnetic_field)))
        self.pub.publish(magnetic_field = list_as_vector(rect_point))

if __name__=="__main__":
    rospy.init_node("imu_rect")
    RectifyMagNode()
    rospy.spin()
