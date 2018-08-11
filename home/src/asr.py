# -*- coding: utf-8 -*-
import time

import cv2

from bs4 import BeautifulSoup


class AsrJobBase:
    def __init__(self,
                 people_faces=None,
                 people_name=None,
                 obj_name=None,
                 raw_text=None,
                 ):
        self.people_faces = people_faces
        self.people_name = people_name
        self.obj_name = obj_name
        self.raw_text = raw_text

    def add_faces(self, faces):
        self.people_faces = faces

    def get_faces(self):
        return self.people_faces

    def debug(self):
        print(self)
        for face in self.get_faces():
            cv2.imshow("debug", face)
            while cv2.waitKey(30) != 27:
                time.sleep(0.1)


class AsrJobComplicative(AsrJobBase):
    def __init__(self,
                 people_faces=None,
                 people_name=None,
                 obj_location=None,
                 obj_name=None,
                 raw_text=None,
                 ):
        AsrJobBase.__init__(self, people_faces, people_name, obj_name, raw_text)
        self.obj_location = obj_location

    def __repr__(self):
        return """
        people_img: %s
        people_name: %s
        obj_location: %s
        obj_name: %s
        raw_text: %s
        """ % (self.people_faces, self.people_name,
               self.obj_location, self.obj_name,
               self.raw_text)

    @staticmethod
    def parser(result):
        soup = BeautifulSoup(result,  "lxml")
        people_faces = None
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
        return AsrJobComplicative(people_faces, people_name,
                                  obj_location, obj_name,
                                  raw_text)


class AsrJobNameObj(AsrJobBase):
    """ The demo
    <?xml version='1.0' encoding='utf-8' standalone='yes' ?><nlp>
      <version>1.1</version>
      <rawtext>我是拉文我要可乐</rawtext>
      <confidence>52</confidence>
      <engine>local</engine>
      <result>
        <focus>intro|guestname|take|item</focus>
        <confidence>65|71|44|48</confidence>
        <object>
          <intro id="65535">我是</intro>
          <guestname id="65535">拉文</guestname>
          <take id="65535">我要</take>
          <item id="65535">可乐</item>
        </object>
      </result>
    </nlp>
    """
    def __init__(self,
                 people_faces=None,
                 people_name=None,
                 obj_name=None,
                 raw_text=None,
                 ):
        AsrJobBase.__init__(self, people_faces, people_name, obj_name, raw_text)

    def __repr__(self):
        return """
        people_img: %s
        people_name: %s
        obj_name: %s
        raw_text: %s
        """ % (self.people_faces, self.people_name,
               self.obj_name, self.raw_text)

    @staticmethod
    def parser(result):
        soup = BeautifulSoup(result,  "lxml")
        people_faces = None

        # check the confidence
        confidence_list = [int(i) for i in soup.result.confidence.string.encode('utf-8').split('|')]
        cl = confidence_list
        if cl[0] < 5 or cl[1] < 15 or cl[2] < 5 or cl[3] < 15:
            print("confidence too low, return None", cl)
            return None

        people_name = soup.guestname.string.encode('utf-8')
        obj_name = soup.item.string.encode('utf-8')
        raw_text = soup.rawtext.string.encode('utf-8')
        print("confidence right, return Job", confidence_list)
        return AsrJobNameObj(people_faces, people_name,
                             obj_name, raw_text)


class AsrConfirm:
    def __init__(self, confirm_words, confirmed):
        self.confirm_words = confirm_words
        self.confirmed = confirmed

    def __repr__(self):
        return """
        confirm_words: %s
        confirmed: %s
        """ % (self.confirm_words, self.confirmed)

    @staticmethod
    def parser(result):
        soup = BeautifulSoup(result,  "lxml")
        if soup.yes is not None:
            return AsrConfirm(soup.yes.string.encode('utf-8'), True)
        elif soup.no is not None:
            return AsrConfirm(soup.no.string.encode('utf-8'), False)
        else:
            return AsrConfirm("error words", False)


if __name__ == '__main__':
    msg = """
<?xml version='1.0' encoding='utf-8' standalone='yes' ?>
<nlp>
  <version>1.1</version>
  <rawtext>是</rawtext>
  <result>
    <object>
      <yes id="401">是</yes>
      <no id="4000">拉文</no>
    </object>
  </result>
</nlp>
    """
    soup = BeautifulSoup(msg,  "lxml")
    if soup.yes is not None:
        print soup.yes.string
    elif soup.no is not None:
        print soup.no
