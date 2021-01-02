#!/usr/bin/env python

import rospy
from exploration.molten_core import molten_core, State

if __name__ == '__main__':

    rospy.loginfo('Initialising core...')
    rospy.init_node('explorer', anonymous=True)
    molten_core = molten_core()
    rospy.loginfo('Done!')

    rospy.loginfo('Control loop is running.')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        next_state = molten_core.mainMove()
        if next_state == State.Done:
            rospy.loginfo('Explorer has completed!')
            break
        rate.sleep()
