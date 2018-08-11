# -*- coding: utf-8 -*-
import pytest

from geometry_msgs.msg import Pose, Point, Quaternion

from ..utils import get_sorted_poses


def get_poses(q, points):
    return [Pose(p, q) for p in points]


@pytest.mark.parametrize("points", [
    (Point(1, 1.1, 0),
     Point(2, 1.5, 0),
     Point(0.5, 1.3, 0),
     ),
])
def test_sort_pose(points):
    # 机器人向前，按照y方向排序
    poses = get_poses(Quaternion(0, 0, -0.0557494534077, 0.998444789883), points)
    poses = get_sorted_poses(poses)
    assert poses[0].position == points[0]
    assert poses[1].position == points[2]
    assert poses[2].position == points[1]

    poses = get_poses(Quaternion(0, 0, 0, 1), points)
    poses = get_sorted_poses(poses)
    assert poses[0].position == points[0]
    assert poses[1].position == points[2]
    assert poses[2].position == points[1]

    # 按照x方向排序
    poses = get_poses(Quaternion(0, 0, -0.7057494534077, 0.708444789883), points)
    poses = get_sorted_poses(poses)
    assert poses[0].position == points[2]
    assert poses[1].position == points[0]
    assert poses[2].position == points[1]
