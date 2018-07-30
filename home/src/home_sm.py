#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys

import rospy
import smach
import smach_ros

from sm import WaitDoorOpen, MeetGuest, LookingGoods, FindPeople, GuestRecognition
from robot import Robot


NAV_POS_LIST_FILE = "/home/lennon/home_ws/.cache/poses.xml"
SELF_INTRO_STR = "你们好啊,我是浙工大机器人,请你们开始介绍自己吧"
GUEST_NUM = 2


def main():
    rospy.init_node('home_state_machine')

    robot = Robot(poses_file_path=NAV_POS_LIST_FILE)

    @smach.cb_interface(outcomes=['finished'])
    def intro_self():
        if robot.speak(SELF_INTRO_STR):
            return 'finished'

    @smach.cb_interface(outcomes=['arrived', 'retry'])
    def nav_to_dining_room():
        return robot.nav_by_place_name('dining room')

    @smach.cb_interface(outcomes=['arrived', 'retry'])
    def leaving():
        return robot.nav_by_place_name('leaving')

    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        # smach.StateMachine.add('WaitDoorOpen', WaitDoorOpen(),
        #                        transitions={'waiting': 'WaitDoorOpen',
        #                                     'door_opened': 'NavToDiningRoom',})

        # smach.StateMachine.add('WaitDoorOpen', WaitDoorOpen(),
        #                        transitions={'waiting': 'WaitDoorOpen',
        #                                     'door_opened': 'MeetGuest',})
        #
        # smach.StateMachine.add('NavToDiningRoom', smach.CBState(nav_to_dining_room),
        #                        transitions={'arrived': 'SelfIntro',
        #                                     'retry': 'NavToDiningRoom',})
        #
        # smach.StateMachine.add('SelfIntro', smach.CBState(intro_self),
        #                        transitions={'finished': 'MeetGuest',})

        smach.StateMachine.add('MeetGuest', MeetGuest(robot, GUEST_NUM, enable_debug=True),
                                transitions={'finished': 'LookingGoods',})

        smach.StateMachine.add('LookingGoods', LookingGoods(robot),
                               transitions={'finished': 'FindPeople',})

        smach.StateMachine.add('FindPeople', FindPeople(robot),
                               transitions={'finished': 'GuestRecognition',})

        smach.StateMachine.add('GuestRecognition', GuestRecognition(robot),
                               transitions={'finished': 'Leaving',})

        smach.StateMachine.add('Leaving', smach.CBState(leaving),
                               transitions={'arrived': 'DONE',
                                            'retry': 'Leaving',})

    try:
        sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
        sis.start()
        sm.execute()
        rospy.spin()
        sis.stop()
    except KeyboardInterrupt:
        rospy.signal_shutdown('All done.')


if __name__ == '__main__':
    sys.exit(main())
