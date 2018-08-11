# -*- coding: utf-8 -*-
import pytest

from geometry_msgs.msg import Pose, Point, Quaternion

from ..utils import pose_valid, poses_valid

q = Quaternion(0, 0, 0, 1)


@pytest.mark.parametrize("poses", [
    (Pose(Point(18.929, 1.1, 0), q),
     Pose(Point(0.5, 1.3, 0), q),
     Pose(Point(1, 21, 0), q),
     Pose(Point(21, 1.5, 0), q),
     Pose(Point(0.5, float("nan"), 0), q),
     ),
])
def test_pose_valid(poses):
    assert pose_valid(poses[0])
    assert pose_valid(poses[1])
    assert not pose_valid(poses[2])
    assert not pose_valid(poses[3])
    assert not pose_valid(poses[4])


@pytest.mark.parametrize("poses_lst", [
    ([[Pose(Point(1, 1.1, 0), q),
       Pose(Point(18.9, 1.5, 0), q),
       Pose(Point(0.5, 1.3, 0), q),
       Pose(Point(0.5, 1, 0), q)],
      [Pose(Point(1, 1.1, 0), q),
       Pose(Point(19, 1.5, 0), q),
       Pose(Point(0.5, 1.3, 0), q),
       Pose(Point(0.5, float("nan"), 0), q)],
      ]
     ),
])
def test_poses_valid(poses_lst):
    assert poses_valid(poses_lst[0])
    assert not poses_valid(poses_lst[1])
