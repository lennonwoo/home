# -*- coding: utf-8 -*-
from ..base import RobotPart


class Memory(RobotPart):
    def __init__(self, robot):
        RobotPart.__init__(self, robot)

        self._job_list = []

    def add_job(self, job):
        self._job_list.append(job)

    def get_job(self):
        return self._job_list



