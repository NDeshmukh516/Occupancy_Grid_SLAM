#!/usr/bin/env python

import rospy
import math
import tf2_ros
import ros_numpy
import numpy as np
from sensor_msgs.msg import Range
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from tf.transformations import euler_from_quaternion
from ypstruct import structure
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

def in_bounds(x, y):
    return 0 <= x < y

def cell_entropy(x):
    if x < 0:
        return np.log(2)
    return -x * np.log(x) - (1 - x) * np.log(1 - x)

class drone_data:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sonar_height = None
        self.last_frame = None

        self.last_map_res = None
        self.last_map_raw = None
        self.last_map_extents = None
        self.last_map = None

        self.last_global_costmap = None
        self.last_global_costmap_raw = None
        self.last_global_costmap_extents = None

        self.last_updated_costmap = None
        self.last_updated_costmap_extents = None


        rospy.Subscriber('sonar_height', Range, self.sonar_callback)
        rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.global_costmap_callback)
        rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.global_costmap_update_callback)

    def incoming_data(self):

        while True:
            self.update_last_frame()

            missing_data = []
            if self.last_frame is None:
                missing_data.append('last_frame')
            if self.last_map is None:
                missing_data.append('last_map')
            if self.last_global_costmap is None:
                missing_data.append('last_global_costmap')

            if len(missing_data) == 0:
                break

            rospy.loginfo('Waiting for the following data: {}.'.format(', '.join(missing_data)))
            rospy.sleep(2)
        

    def update_last_frame(self):
        try:
            tfmsg = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
            trans = tfmsg.transform.translation
            orient = tfmsg.transform.rotation
            yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])[2]
            self.last_frame = (trans.x, trans.y, yaw)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False
        return True

    def sonar_callback(self, data):
        self.sonar_height = data.range

    def map_callback(self, data):
        self.last_map_raw = data
        self.last_map = ros_numpy.occupancy_grid.occupancygrid_to_numpy(data).data
        res = data.info.resolution
        left = data.info.origin.position.x
        right = left + data.info.width * res
        bottom = data.info.origin.position.y
        top = bottom + data.info.height * res
        self.last_map_extents = (left, right, bottom, top)

    def global_costmap_callback(self, data):
        self.last_global_costmap_raw = data

        self.last_global_costmap = ros_numpy.occupancy_grid.occupancygrid_to_numpy(data)

        res = data.info.resolution
        left = data.info.origin.position.x
        right = left + data.info.width * res
        bottom = data.info.origin.position.y
        top = bottom + data.info.height * res
        self.last_global_costmap_extents = (left, right, bottom, top)


    def global_costmap_update_callback(self, data):
        if self.last_map_raw is None or self.last_global_costmap is None:
            return

        self.last_updated_costmap = self.occupancy_grid_update_to_numpy(data)

        self.last_global_costmap[data.y:data.y + data.height, data.x:data.x + data.width] = self.last_updated_costmap
        res = self.last_map_raw.info.resolution
        left = self.last_global_costmap_extents[0] + data.x*res
        right = left + data.width*res
        bottom = self.last_global_costmap_extents[2] + data.y*res
        top = bottom + data.height*res
        self.last_updated_costmap_extents = (left, right, bottom, top)

    @staticmethod
    def occupancy_grid_update_to_numpy(msg):
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.height, msg.width)
        return np.ma.array(data, mask=data == -1, fill_value=-1)

class Actions:

    def __init__(self, core, drone_data, movement):
        self.core = core
        self.data = drone_data
        self.movement = movement

        self.previous_height = None
        self.counter = 0

    def launching(self):
        sonar_height = self.data.sonar_height

        if sonar_height is None:
            return False
    
        diff = self.core.configuration['hover_height'] - sonar_height

        if diff > 0.03:
            vel = np.sign(diff)*0.1
            self.movement.request_velocity(zLin = vel)
            return False

        self.movement.request_velocity(zLin = 0)
        return True

    def radial_scan(self, start):
        speed = 0.5
        rotation_time = 2* math.pi/speed
        current_time = rospy.Time.now()
        duration = current_time - start

        if duration < rospy.Duration(rotation_time):
            self.movement.request_velocity(zAng = speed)
            return False
        
        else:
            self.movement.request_velocity(zAng = 0)
            return True
    
    def land(self):
        self.movement.request_velocity(zLin = -0.1)
        sonar_height = self.data.sonar_height

        if sonar_height is None:
            return False
        
        if self.previous_height is None:
            self.previous_height = sonar_height
            self.counter = 0
            return False
        
        if abs(self.previous_height - sonar_height) < 0.005:
            self.counter += 1

        if self.counter > 30:
            # Reset Variables
            self.previous_height = None
            self.counter = 0
            return True

        self.previous_height = sonar_height
        return False

class movement:

    def __init__(self, core, drone_data):
        self.state = None
        self.core = core
        self.store = drone_data

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.current_vel_msg = None

    def hover(self):
        sonar_height = self.store.sonar_height
        if sonar_height is not None:
            hover_height = self.core.configuration['hover_height']
            difference = hover_height - sonar_height
            if abs(difference) > 0.01:
                self.request_velocity(zLin=difference)

    def start_step(self, current_state):

        # Initialise a new velocity message for the current step
        self.current_vel_msg = Twist()
        self.current_vel_msg.linear.x = 0
        self.current_vel_msg.linear.y = 0
        self.current_vel_msg.linear.z = 0
        self.current_vel_msg.angular.x = 0
        self.current_vel_msg.angular.y = 0
        self.current_vel_msg.angular.z = 0

        self.state = current_state

    def finish(self):

        # Adjust altitude of the drone if necessary
        if self.state not in self.core.configuration['exclude_from_hover']:
            self.hover()

        self.velocity_publisher.publish(self.current_vel_msg)
        self.state = None

    def request_velocity(self, xLin=None, yLin=None, zLin=None, xAng=None, yAng=None, zAng=None):
        if self.current_vel_msg is None:
            return None

        if xLin is not None:
            self.current_vel_msg.linear.x = xLin
        if yLin is not None:
            self.current_vel_msg.linear.y = yLin
        if zLin is not None:
            self.current_vel_msg.linear.z = zLin
        if xAng is not None:
            self.current_vel_msg.angular.x = xAng
        if yAng is not None:
            self.current_vel_msg.angular.y = yAng
        if zAng is not None:
            self.current_vel_msg.angular.z = zAng

        return self.current_vel_msg
    
class Planner:

    def __init__(self, core, drone_data, movement):
        self.core = core
        self.store = drone_data
        self.movement = movement

        rospy.Subscriber('/planned_cmd_vel', Twist, self.planned_cmd_vel_callback)

        rospy.loginfo('Initialising a SimpleActionClient for move_base and waiting for connection...')
        self.sa_client = SimpleActionClient('move_base', MoveBaseAction)
        self.sa_client.wait_for_server()
        rospy.loginfo('Connected to move_base action server!')

        self.last_goal = None
        self.last_planned_cmd_vel_msg = None
        self.blacklisted_goals = []

    def planned_cmd_vel_callback(self, twist_msg):
        self.last_planned_cmd_vel_msg = twist_msg

    def find_goal(self):
        locations_indices = self.find_locations_indices()

        if len(locations_indices) < 1:
            return None

        # Make sure locations are at least one meter apart and NOT in blacklisted regions
        possible_locations = []
        min_gap = 1.0
        for loc_a, loc_b in locations_indices:
            too_close = False
            x, y = self.indices_to_meters(loc_a, loc_b)

            # Check that the current location is at least `min_gap` away from all other candidate locations
            for pos_a, pos_b in possible_locations:
                pos_x, pos_y = self.indices_to_meters(pos_a, pos_b)
                distance = math.sqrt((x - pos_x) ** 2 +  (y - pos_y) ** 2)
                if distance > min_gap:
                    too_close = True
                    break
            if too_close:
                continue

            # Check that the current location is not in blacklisted regions
            blacklisted = False
            for blacklst_x, blacklst_y, _ in self.blacklisted_goals:
                distance = math.sqrt((x - blacklst_x) ** 2 +  (y - blacklst_y) ** 2)
                if distance <= self.core.configuration['blacklisted_goal_radius']:
                    blacklisted = True
                    break
            if blacklisted:
                continue

            # The current location is OK, add it to the list of candidates
            possible_locations.append((loc_a, loc_b))

        # Iterate through all locations and find the one that gives maximises information gain
        max_gain = -1
        best_location = None
        for loc_a, loc_b in possible_locations:
            gain = self.goal_gain(loc_a, loc_b)
            if gain > max_gain:
                # Check if the goal with best gain is possible to observer safely
                goal_safe = self.find_safe_space_to_observe(loc_a, loc_b)
                if goal_safe is None:
                    continue
                max_gain = gain
                best_location = goal_safe

        # Find a safe place to observe the best location
        if best_location is not None:
            goal_safe_a, goal_safe_b = best_location
            goal_x, goal_y = self.indices_to_meters(goal_safe_a, goal_safe_b)

            pos_x, pos_y, yaw = self.store.last_frame
            angle_to_goal = math.atan2(goal_y - pos_y, goal_x - pos_x)

            return round(goal_x, 3), round(goal_y, 3), round(angle_to_goal, 3)

        # Didn't find anything, return none
        return None

    def goal_gain(self, loc_a, loc_b):

        info = self.store.last_map_raw.info
        resolution = info.resolution

        gain_radius = 3.0
        radius = int(gain_radius / resolution)

        m, n = self.store.last_map.shape
        gain_sum = 0
        total = 0
        for a in range(loc_a - radius, loc_a + radius + 1):
            if not in_bounds(a, m):
                continue
            for b in range(loc_b - radius, loc_b + radius + 1):
                if not in_bounds(b, n):
                    continue
                distance = math.sqrt((a - loc_a) ** 2 + (b - loc_b) ** 2)
                if distance > radius:
                    continue
                gain_sum += self.cell_gain(a, b)
                total += 1
        return gain_sum / total

    def cell_gain(self, x, y):
        p_true = 0.9

        p = self.store.last_map[x, y]
        if p < 0:
            return 0.5983
        elif p == 0 or p >= 1:
            return 4.259045e-6
        p_o = p_true * p + (1 - p_true) * (1 - p)
        p_n = p_true * (1 - p) + (1 - p_true) * p
        EH = -p_true * p * np.log(p_true * p / p_o) - p_true * (1 - p) * np.log(p_true * (1 - p) / p_n)
        return cell_entropy(p) - EH

    def find_safe_space_to_observe(self, loc_a, loc_b):

        costmap = self.store.last_global_costmap
        m, n = costmap.shape

        candidates = [
            (3, 3),
            (3, 0),
            (3, -3),
            (0, -3),
            (-3, -3),
            (-3, 0),
            (-3, 3),
            (0, 3),
        ]

        best_score = -1
        best_loc_a = None
        best_loc_b = None

        for candidate_loc_a, candidate_loc_b in candidates:
            new_loc_a = loc_a + candidate_loc_a
            new_loc_b = loc_b + candidate_loc_b

            if not in_bounds(new_loc_a, m) \
                    or not in_bounds(new_loc_b, n) \
                    or costmap.mask[new_loc_a, new_loc_b] \
                    or costmap[new_loc_a, new_loc_b] != 0:
                continue

            score = 0
            for a in range(new_loc_a - 2, new_loc_a + 3):
                for b in range(new_loc_b - 2, new_loc_b + 3):
                    if not in_bounds(a, m) or not in_bounds(b, n):
                        continue

                    if not costmap.mask[a, b] and costmap[a, b] == 0:
                        score += 1

            if score > best_score:
                best_score = score
                best_loc_a = new_loc_a
                best_loc_b = new_loc_b

        if best_loc_a is not None and best_loc_b is not None:
            return best_loc_a, best_loc_b

        return None

    def find_locations_indices(self):
        costmap = self.store.last_global_costmap
        rospy.loginfo(costmap)
        if costmap is None:
            rospy.logwarn('No costmap is available - cannot find locations as indices, doing nothing.')
            return []

        m, n = costmap.shape
        safe_space_indices = np.where(costmap == 0)
        location_index_list = []

        candidates = [
            (1, -1),
            (1, 0),
            (1, 1),
            (0, 1),
            (-1, 1),
            (-1, 0),
            (-1, -1),
            (-1, 0),
        ]

        # We define a location as a cell that has at at least three free ngainhbours and and at least three unknown
        # ngainhbours. Remaining ngainhbours should either be free on unknown.

        for (loc_a, loc_b) in zip(*safe_space_indices):

            # Sanity check - skip masked inputs if they are accidentally included
            if costmap.mask[loc_a, loc_b]:
                continue

            free = 0
            known_occupied = 0
            for off_loc_a, off_loc_b in candidates:
                new_loc_a, new_loc_b = loc_a + off_loc_a, loc_b + off_loc_b

                if 0 <= new_loc_a < m and 0 <= new_loc_b < n:
                    value = costmap[new_loc_a, new_loc_b]
                    masked = costmap.mask[new_loc_a, new_loc_b]
                    if value == 0:
                        free += 1
                    elif not masked:
                        known_occupied += 1
                        break
                else:
                    # We're add the edge of the costmap, can consider this as an unknown
                    pass

            if known_occupied == 0 and 3 <= free <= 5:
                location_index_list.append((loc_a, loc_b))

        return location_index_list

    def travel_to_goal(self):
        goal_status = self.sa_client.get_state()

        if goal_status == GoalStatus.PENDING:
            return False

        elif goal_status == GoalStatus.ACTIVE:
            if self.last_planned_cmd_vel_msg is None:
                return False

            msg = self.last_planned_cmd_vel_msg
            self.movement.request_velocity(
                xLin=msg.linear.x,
                yLin=msg.linear.y,
                zLin=msg.linear.z,
                xAng=msg.angular.x,
                yAng=msg.angular.y,
                zAng=msg.angular.z
            )
            return False

        elif goal_status in [GoalStatus.RECALLED, GoalStatus.PREEMPTED]:
            rospy.logwarn('Goal was recalled or preempted.')
            return True

        elif goal_status in [GoalStatus.REJECTED, GoalStatus.ABORTED]:
            rospy.logwarn('Goal was rejected or aborted - this goal was blacklisted.')
            self.blacklisted_goals.append(self.last_goal)
            self.last_goal = None
            return True

        elif goal_status == GoalStatus.SUCCEEDED:
            rospy.loginfo('Goal succeeded.')
            self.last_goal = None
            return True

        elif goal_status == GoalStatus.LOST:
            rospy.loginfo('Goal was lost!')
            self.last_goal = None
            return True

        rospy.logwarn('Unrecognised goal status was returned! Assuming goal reaching behaviour terminated.')
        return True

    def publish_goal(self, x_coord, y_coord, yaw):
        rospy.loginfo(" requesting to move to {}".format((x_coord, y_coord, yaw)))

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x_coord
        goal.target_pose.pose.position.y = y_coord
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.last_goal = (x_coord, y_coord, yaw)
        self.sa_client.send_goal(goal)

    def cancel_all_goals(self):
        self.sa_client.cancel_all_goals()

    def indices_to_meters(self, loc_a, loc_b):
        info = self.store.last_map_raw.info
        resolution = info.resolution
        x = info.origin.position.x + loc_b * resolution + resolution / 2
        y = info.origin.position.y + loc_a * resolution + resolution / 2
        return x, y

    def meters_to_indices(self, x, y):
        info = self.store.last_map_raw.info
        resolution = info.resolution
        loc_b = int(round((x - info.origin.position.x) / resolution))
        loc_a = int(round((y - info.origin.position.y) / resolution))
        return loc_a, loc_b
