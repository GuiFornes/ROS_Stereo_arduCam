import open3d
import rospy
import numpy as np
from ctypes import *
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2

FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
                [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]
# Bit operations
BIT_MOVE_16 = 2 ** 16
BIT_MOVE_8 = 2 ** 8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000) >> 16, (rgb_uint32 & 0x0000ff00) >> 8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)


# Conversion d'un pointcloud open3d en structure pointcloud2 ROS
def convert_open3d_to_ros(open3d_cloud, frame_id="r_robot_filtered"):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    points = np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors:  # XYZ seulement
        fields = FIELDS_XYZ
        cloud_data = points
    else:  # XYZ + RGB, donc avec couleur
        print("here")
        fields = FIELDS_XYZRGB
        colors = np.floor(np.asarray(open3d_cloud.colors) * 255)
        colors = colors[:, 0] * BIT_MOVE_16 + colors[:, 1] * BIT_MOVE_8 + colors[:, 2]
        print("colors", colors)
        cloud_data = np.c_[points, colors]
    print("here 2")
    out = pc2.create_cloud(header, fields, cloud_data)
    print("here 3")
    return out


# Conversion d'un pointcloud2 ROS en pointcloud open3d
def convert_ros_to_open3d(ros_cloud):
    field_names = [field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names=field_names))
    open3d_cloud = open3d.geometry.PointCloud()
    if len(cloud_data) == 0:
        print("Conversion d'un nuage de point vide!")
        return None
    if "rgb" in field_names:
        idx_rgb_in_field = 3  # x, y, z, rgb
        xyz = [(x, y, z) for x, y, z, rgb in cloud_data]
        if type(cloud_data[0][idx_rgb_in_field]) == float:
            rgb = [convert_rgbFloat_to_tuple(rgb) for x, y, z, rgb in cloud_data]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x, y, z, rgb in cloud_data]
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb) / 255.0)
    else:
        xyz = [(x, y, z) for x, y, z in cloud_data]  # get xyz
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
    return open3d_cloud
