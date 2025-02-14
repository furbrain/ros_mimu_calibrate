from typing import Optional

import numpy as np
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import PointField, PointCloud2
from std_msgs.msg import Header


def create_cloud_xyz32(points: np.ndarray,
                       frame_id: Optional[str]=None,
                       stamp: Optional[rospy.Time]=None):
    """
    Create a L{sensor_msgs.msg.PointCloud2} message with 3 float32 fields (x, y, z).

    @param points: The point cloud points.
    @type  points: np.ndarray
    @return: The point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """
    if frame_id is None:
        frame_id = "map"
    if stamp is None:
        stamp = rospy.Time.now()
    header = Header(frame_id="map", stamp=stamp)
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


def vector_as_list(d: Vector3):
    return [d.x,d.y,d.z]


def list_as_vector(arr: np.array):
    return Vector3(x=arr[0],y=arr[1],z=arr[2])
