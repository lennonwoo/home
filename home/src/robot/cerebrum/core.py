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
        else:
            return None

    def get_last_job(self):
        if len(self._job_list) > 0:
            return self._job_list[-1]
        else:
            return None

    def get_jobs(self):
        return self._job_list

    def exist_name(self, name):
        for job in self._job_list:
            if name == job.people_name:
                return True
        else:
            return False

    def get_job_by_name(self, name):
        for job in self._job_list:
            if name == job.people_name:
                return job
