#!/usr/bin/env python
import sys

import rospy
import smach
import smach_ros

from config import Config
from sm import WaitDoorOpen, MeetGuest, LookingGoods, FindPeople
from robot import Robot


def main():
    rospy.init_node('home_state_machine')

    robot = Robot(Config)

    @smach.cb_interface(outcomes=['finished'])
    def intro_self(userdata):
        robot.speak_with_wav(Config.self_intro_wav)
        return 'finished'

    @smach.cb_interface(outcomes=['arrived', 'retry'])
    def nav_to_meet_guest(userdata):
        return 'arrived' if robot.nav_by_place_name('meet_guest') else 'retry'

    @smach.cb_interface(outcomes=['arrived', 'retry'])
    def leaving(userdata):
        return 'arrived' if robot.nav_by_place_name('leave') else 'retry'

    @smach.cb_interface(outcomes=['finished'])
    def debug(userdata):
        robot.debug()
        return 'finished'

    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add('WaitDoorOpen', WaitDoorOpen(),
                               transitions={'waiting': 'WaitDoorOpen',
                                            'door_opened': 'NavToDiningRoom',})

        smach.StateMachine.add('NavToDiningRoom', smach.CBState(nav_to_meet_guest),
                               transitions={'arrived': 'SelfIntro',
                                            'retry': 'NavToDiningRoom',})
        #
        smach.StateMachine.add('SelfIntro', smach.CBState(intro_self),
                               transitions={'finished': 'MeetGuest',})

        smach.StateMachine.add('MeetGuest', MeetGuest(robot, Config.people_num),
                               transitions={'finished': 'LookingGoods',})

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
        sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
        sis.start()
        sm.execute()
        rospy.spin()
        sis.stop()
    except KeyboardInterrupt:
        rospy.signal_shutdown('All done.')


if __name__ == '__main__':
    sys.exit(main())