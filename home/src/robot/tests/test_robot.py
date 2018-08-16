# -*- coding: utf-8 -*-
import pytest

from geometry_msgs.msg import Pose, Point, Quaternion

from ..utils import pose_valid, poses_valid, filter_pose_out_of_map

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


@pytest.mark.parametrize("poses_lst", [
    ([
        [Pose(Point(1, 0, 0), q),
         Pose(Point(3.5, -1.3, 0), q),
         Pose(Point(9.1, 4.8, 0), q),
         Pose(Point(0.5, 10.8, 0), q)],
        [Pose(Point(1, 1.1, 0), q),
         Pose(Point(19, 1.5, 0), q),
         Pose(Point(0.5, 1.3, 0), q),
         Pose(Point(0.5, 1, 0), q)],
        [Pose(Point(-0.1, 0, 0), q),
         Pose(Point(3.5, -1.3, 0), q),
         Pose(Point(9.4, 4.8, 0), q),
         Pose(Point(0.5, 10.8, 0), q)],
        [Pose(Point(1, -2.6, 0), q),
         Pose(Point(3.5, 12, 0), q),
         Pose(Point(12, 4.8, 0), q),
         Pose(Point(0.5, 10.8, 0), q)],
        [Pose(Point(0, -2.6, 0), q),
         Pose(Point(13.5, 8.5, 0), q),
         Pose(Point(12, 4.8, 0), q),
         Pose(Point(0.5, 11.8, 0), q)],
    ]
    ),
])
def test_filter_pose_out_of_map(poses_lst):
    xmin = 0.347
    ymin = -1.43
    xmax = 9.25
    ymax = 10.9
    points = (xmin, ymin, xmax, ymax)
    assert len(filter_pose_out_of_map(poses_lst[0], *points)) == 4
    assert len(filter_pose_out_of_map(poses_lst[1], *points)) == 3
    assert len(filter_pose_out_of_map(poses_lst[2], *points)) == 2
    assert len(filter_pose_out_of_map(poses_lst[3], *points)) == 1
    assert len(filter_pose_out_of_map(poses_lst[4], *points)) == 0
