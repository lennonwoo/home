# -*- coding: utf-8 -*-
from ..base import RobotPart


class Memory(RobotPart):
    def __init__(self, robot):
        RobotPart.__init__(self, robot)

        self._job_list = []

    def add_job(self, job):
        self._job_list.append(job)

    def delete_last_job(self):
        if len(self._job_list) > 0:
            self._job_list.pop()

    def get_jobs(self):
        return self._job_list



