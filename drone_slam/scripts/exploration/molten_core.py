#!/usr/bin/env python

import rospy
import numpy as np
from enum import Enum
from drone_data import drone_data, Actions, Planner, movement
from ypstruct import structure

"""state_machine = structure()
State.Init = 0
State.WaitingForData = 1
State.Takeoff = 2
State.ExploreStart = 3
State.FullRadialScan = 4
State.FindGoal = 5
State.TravelToGoal = 6
State.ExplorationFinish = 7
State.Land = 8
State.Done = 9"""

class State(Enum):
    Init = 0
    WaitingForData = 1
    Takeoff = 2
    ExploreStart = 3
    FullRadialScan = 4
    FindGoal = 5
    TravelToGoal = 6

    ExplorationFinish = 777
    Land = 888
    Done = 999




class molten_core:
    def __init__(self):
        self.state = State.Init

        self.configuration = {
        'hover_height': 1.0,
        'exclude_from_hover':[State.Init, State.Takeoff,
        State.Land, State.Done],
        'blacklisted_radius':2.0
        }

        self.data = drone_data()
        self.movement = movement(core=self, drone_data=self.data)
        self.actions = Actions(core=self, drone_data=self.data, movement=self.movement)
        self.planner = Planner(core=self, drone_data=self.data, movement=self.movement)


        # Temporary variables
        self.temp = {}
        self.last_goal = None

    def ASM(self, state):
        if state == State.Init:
            self.planner.cancel_all_goals()
            rospy.loginfo('Hector has been initialized')

            return State.WaitingForData

        elif state == State.WaitingForData:
            rospy.loginfo('We are waiting for data...')
            self.data.incoming_data()
            rospy.loginfo('Hector is taking off!!')

            return State.Takeoff

        elif state == State.Takeoff:
            return State.ExploreStart if self.actions.launching() else None

        elif state == State.ExploreStart:
            rospy.loginfo('Exploration algorithm is starting')
            rospy.loginfo('Proceeding to radial scan')
            return State.FullRadialScan
        
        elif state == State.FullRadialScan:
            start_signal = self.temp.get('start_signal')

            if start_signal is None:
                start_signal = rospy.Time.now()
                self.temp['start_signal'] = start_signal

            done = self.actions.radial_scan(start=start_signal)

            if done:
                self.temp['start_signal'] = None
		rospy.loginfo('Radial scan complete!')
                return State.FindGoal
            else:
                return None
        
        elif state == State.FindGoal:
            rospy.loginfo('Found a goal!')

            goal = self.planner.find_goal()
            rospy.loginfo(goal)

            if goal is None:
		rospy.loginfo('No more goals...we are done')
                return State.ExplorationFinish
            if goal == self.last_goal:
		rospy.loginfo('Previous goal is the same as next goal, so we are done')
                return State.ExplorationFinish

            self.last_goal = goal
            x, y, yaw = goal
            self.planner.publish_goal(x, y, yaw)
	    rospy.loginfo('Traveling to the goal')
            return State.TravelToGoal

        elif state == State.TravelToGoal:

            if self.planner.travel_to_goal():
                self.planner.cancel_all_goals()
                return State.FullRadialScan
            else:
                return None

        elif state ==State.ExplorationFinish:
            rospy.loginfo('MISSION ACCOMPLISHED')

            self.planner.cancel_all_goals()
            return State.Land
        
        elif state == State.Land:
            rospy.loginfo('LANDING')

            return State.Done if self.actions.land() else None

        else:
            return None
    
    def mainMove(self):
        self.data.update_last_frame

        current_state = self.state
        self.movement.start_step(current_state)
        next_state = self.ASM(current_state)

        if next_state is not None:
            self.state = next_state
        
        self.movement.finish()
        return next_state
