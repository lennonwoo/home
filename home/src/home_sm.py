#!/usr/bin/env python
import sys

import rospy
import smach
import smach_ros

from robot import Robot
from config import Config
from sm import WaitDoorOpen, MeetGuest, LookingGoods, FindPeople


def main():
    rospy.init_node('home_state_machine')

    robot = Robot(Config)

    @smach.cb_interface(outcomes=['arrived', 'retry'])
    def enter_door(userdata):
        return 'arrived' if robot.nav_by_place_name('enter_door') else 'retry'

    @smach.cb_interface(outcomes=['finished'])
    def self_intro(userdata):
        robot.speak_self_intro()
        return 'finished'

    @smach.cb_interface(outcomes=['arrived', 'retry'])
    def leaving(userdata):
        return 'arrived' if robot.nav_by_place_name('leave') else 'retry'

    @smach.cb_interface(outcomes=['finished'])
    def debug(userdata):
        robot.debug()
        return 'finished'

    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add('WaitDoorOpen', WaitDoorOpen(robot),
                               transitions={'waiting': 'WaitDoorOpen',
                                            'door_opened': 'EnterDoor',})

        smach.StateMachine.add('EnterDoor', smach.CBState(enter_door),
                               transitions={'arrived': 'SelfIntro',
                                            'retry': 'EnterDoor',})
        #
        smach.StateMachine.add('SelfIntro', smach.CBState(self_intro),
                               transitions={'finished': 'MeetGuest',})

        smach.StateMachine.add('MeetGuest', MeetGuest(robot, Config.people_num),
                               transitions={'finished': 'LookingGoods',
                                            # 'retry': 'MeetGuest',
                               })

        smach.StateMachine.add('LookingGoods', LookingGoods(robot),
                               transitions={'finished': 'FindPeople',})

        smach.StateMachine.add('FindPeople', FindPeople(robot),
                               transitions={'finished': 'Leaving',
                                            'retry': 'FindPeople',})

        smach.StateMachine.add('Leaving', smach.CBState(leaving),
                               transitions={'arrived': 'Debug',
                                            'retry': 'Leaving',})

        smach.StateMachine.add('Debug', smach.CBState(debug),
                               transitions={'finished': 'DONE',})

    try:
        if Config.debug:
            sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
            sis.start()

        sm.execute()
        rospy.spin()

        if Config.debug:
            sis.stop()
    except KeyboardInterrupt:
        rospy.signal_shutdown('All done.')


if __name__ == '__main__':
    sys.exit(main())
