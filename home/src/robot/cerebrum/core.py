# -*- coding: utf-8 -*-
import threading
from Queue import LifoQueue

from bs4 import BeautifulSoup


class Job:
    def __init__(self,
                 people_img=None,
                 people_name=None,
                 obj_location=None,
                 obj_name=None,
                 raw_text=None,
                 ):
        self.people_img = people_img
        self.people_name = people_name
        self.obj_location = obj_location
        self.obj_name = obj_name
        self.raw_text = raw_text

    @staticmethod
    def job_parser(result):
        soup = BeautifulSoup(result,  "lxml")
        people_img = None
        people_name = soup.guestname.string.encode('utf-8')
        if soup.livingroom:
            obj_location = soup.livingroom.string.encode('utf-8')
        elif soup.dingingroom:
            obj_location = soup.dingingroom.string.encode('utf-8')
        elif soup.kitchen:
            obj_location = soup.kitchen.string.encode('utf-8')
        elif soup.bedroom:
            obj_location = soup.bedroom.string.encode('utf-8')
        else:
            obj_location = None
        obj_name = soup.item.string.encode('utf-8')
        raw_text = soup.rawtext.string.encode('utf-8')
        return Job(people_img, people_name,
                   obj_location, obj_name,
                   raw_text)

    def __repr__(self):
        return """
        people_img: %s
        people_name: %s
        obj_location: %s
        obj_name: %s
        raw_text: %s
        """ % (self.people_img, self.people_name,
               self.obj_location, self.obj_name,
               self.raw_text)


# class UpdateVariable:
#     def __init__(self):
#         self.event = threading.Event()
#
#     def update(self):
#         self.event.set()
#
#     def wait_update(self):
#         self.event.clear()
#         self.event.wait()


class Memory:
    def __init__(self):
        self._job_list = []
        # self._heard_list = []
        # # 用任务分派的queue来缓存图片真的ok吗
        # self._watched_queue = LifoQueue(maxsize=30)
        # self._face_queue = LifoQueue(maxsize=30)
        # self._people_queue = LifoQueue(maxsize=30)
        # self._people_list = None
        # self.last_face = None
        # self.last_job = None

    def add_job(self, job):
        print('add job')
        self._job_list.append(job)

    def get_job(self):
        print('get job')
        return self._job_list[-1]

    # def add_heard(self, msg):
    #     self._heard_list.append(msg)
    #
    # def add_face(self, img):
    #     self._face_queue.put(img)
    #
    # def add_peoples(self, raw_img, people_list):
    #     self._people_queue.put(raw_img)
    #     self._people_list = people_list
    #
    # def add_watched(self, img):
    #     self._watched_queue.put(img)
    #
    # def wait_face(self):
    #     print("wait for face")
    #     self.face_updated.wait_update()
    #     self.last_face = self._face_queue.get()
    #     print("wait for face done")
    #
    # def wait_job(self):
    #     print("wait for job")
    #     self.job_updated.wait_update()
    #     self.last_job = self._job_list[-1]
    #     print("wait for job done")
    #
    # def get_last_job(self):
    #     self.last_job.people_img = self.last_face
    #     return self.last_job


if __name__ == '__main__':
    result = """
<?xml version='1.0' encoding='utf-8' standalone='yes' ?>
<nlp>
  <version>1.1</version>
  <rawtext>我是拉文到厨房冰箱拿可乐给我</rawtext>
  <confidence>53</confidence>
  <engine>local</engine>
  <result>
    <focus>intro|guestname|go|kitchen|kd|grasp|item|give</focus>
    <confidence>59|62|66|41|44|63|92|55</confidence>
    <object>
      <intro id="401">我是</intro>
      <guestname id="4000">拉文</guestname>
      <go id="101">到</go>
      <kitchen id="1300">厨房</kitchen>
      <kd id="1301">冰箱</kd>
      <grasp id="201">拿</grasp>
      <item id="2003">可乐</item>
      <give id="302">给我</give>
    </object>
  </result>
</nlp>
    """
    job = Job.job_parser(result)
    print(job)
