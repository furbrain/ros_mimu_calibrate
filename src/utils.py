import numpy as np
from sensor_msgs.msg import PointField, PointCloud2
from std_msgs.msg import Header


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
