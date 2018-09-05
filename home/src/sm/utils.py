from math import acos, asin, cos, sin

import dynamic_reconfigure.client


PI = 3.1415926535897931


def get_sorted_poses(poses):
    angel = get_angle_by_orientation(poses[0].orientation)
    print("[get_sorted_poses] the angel ", angel)

    if 0 <= angel < 45 or 315 <= angel < 360 or 135 <= angel < 225:
        return sorted(poses, key=lambda pose: pose.position.y)
    elif 45 <= angel < 135 or 225 <= angel < 315:
        return sorted(poses, key=lambda pose: pose.position.x)
    else:
        return poses


def get_angle_by_orientation(orientation):
    theta = acos(orientation.w) * 2
    if asin(orientation.z) < 0:
        theta += PI

    return int((theta/PI) * 180)

def change_costmap_params(robot_radius=0.18, inflation_radius=0.3):
    client = dynamic_reconfigure.client.Client("move_base/global_costmap", timeout=10)
    client.update_configuration({"robot_radius": robot_radius})
    client = dynamic_reconfigure.client.Client("move_base/global_costmap/inflation_layer", timeout=10)
    client.update_configuration({"inflation_radius": inflation_radius})
    client = dynamic_reconfigure.client.Client("move_base/local_costmap", timeout=10)
    client.update_configuration({"robot_radius": robot_radius})
    client = dynamic_reconfigure.client.Client("move_base/local_costmap/inflation_layer", timeout=10)
    client.update_configuration({"inflation_radius": inflation_radius})

def decrease_costmap():
    # Config.change_costmap_params(0.1, 0.1)
    pass

def increase_costmap():
    # Config.change_costmap_params(0.18, 0.3)
    pass
