from id_common.tf_message_filter import transform_conemap
from id_common import GT_MAP_FRAME_ID, cone_map_utils
import geometry_msgs.msg
from std_msgs.msg import Header
import sys
import math
import rclpy.time

if len(sys.argv) != 5:
    print("Usage: python3 translate.py <path to conemap> <x> <y> <theta>")

dummy_header = Header(frame_id=GT_MAP_FRAME_ID, stamp=rclpy.time.Time().to_msg())
cm = cone_map_utils.load_from_file(sys.argv[1], header=dummy_header)


def get_quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """
    Convert an Euler angle to a quaternion.

    Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    return [qx, qy, qz, qw]


x, y, z, w = get_quaternion_from_euler(0, 0, -float(sys.argv[4]))


tf = geometry_msgs.msg.TransformStamped(
    header=dummy_header,
    child_frame_id=GT_MAP_FRAME_ID,
    transform=geometry_msgs.msg.Transform(
        translation=geometry_msgs.msg.Vector3(x=-float(sys.argv[2]), y=-float(sys.argv[3]), z=0.0),
        rotation=geometry_msgs.msg.Quaternion(x=x, y=y, z=z, w=w)
    )
)

cm2 = transform_conemap(cm, tf)

im, _, _ = cone_map_utils.as_occupancy_grid(cm2)
im.show()

print(cone_map_utils.as_csv(cm2))
