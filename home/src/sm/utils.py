from math import acos, asin, cos, sin


PI = 3.1415926535897931


def get_sorted_poses(poses):
    angel = get_angle_by_orientation(poses[0].orientation)
    print("[get_sorted_poses] the angel ", angel)

    if 0 <= angel < 45 or 315 <= angel < 360 or 135 <= angel < 225:
        return sorted(poses, key=lambda pose: pose.position.x)
    elif 45 <= angel < 135 or 225 <= angel < 315:
        return sorted(poses, key=lambda pose: pose.position.y)
    else:
        return poses


def get_angle_by_orientation(orientation):
    theta = acos(orientation.w) * 2
    if asin(orientation.z) < 0:
        theta += PI

    return int((theta/PI) * 180)
