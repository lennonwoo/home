from bs4 import BeautifulSoup
from geometry_msgs.msg import Pose, Point, Quaternion


def get_nav_pose(file_path):
    lst, dic = list(), dict()
    with open(file_path) as f:
        soup = BeautifulSoup(f, 'lxml')
        for pose in soup.findAll("pose"):
            x, y, z = (float(i) for i in pose.position.text.split(' '))
            point = Point(x, y, z)

            x, y, z, w = (float(i) for i in pose.orientation.text.split(' '))
            quaternion = Quaternion(x, y, z, w)

            lst.append(Pose(point, quaternion))
            dic[pose.nav_name.text] = Pose(point, quaternion)

    return lst, dic
