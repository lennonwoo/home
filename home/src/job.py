# -*- coding: utf-8 -*-
import time

import cv2

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

    def set_face(self, img):
        self.people_img = img

    def debug(self):
        print(self)
        cv2.imshow("debug", self.people_img)
        while cv2.waitKey(30) != 27:
            time.sleep(0.1)

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


if __name__ == '__main__':
    msg = """
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
    job = job_parser(msg)
