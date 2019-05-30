#!/usr/bin/env python
import rospy
import sys
import tf
from enum import Enum
import numpy as np
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
from math import cos, sin, asin, tan, atan2, pi, sqrt, copysign
# msgs and srv for working with the set_model_service
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from copy import deepcopy
from sensor_msgs.msg import Range
from std_srvs.srv import Empty

# a handy tool to convert orientations
from tf.transformations import euler_from_quaternion, quaternion_from_euler
class Thymio_State(Enum):
    SEARCHING = 0
    PICK = 1
    UNLOAD = 2
    OBSTACLE = 3
    DONE = 4
    SLEEP = 5
class MarkerErrors(Exception):
    pass
class PID:
    def __init__(self):
        self.error_sum = 0
        self.previous_error = None
    def calculate_pid_step(self, error):
        K_p = 2
        K_i = 1
        K_d = 0.5
        dt = 0.01
        derivative = (error - self.previous_error) / dt if self.previous_error else 0
        if self.previous_error == 0:
            derivative = 0
        self.previous_error = error
        self.error_sum += error * dt

        return K_p * error + K_d * derivative + K_i * self.error_sum
class BasicThymio:

    def __init__(self, thymio_name):
        """init"""
        self.thymio_name = thymio_name
        rospy.init_node('thymioid', anonymous=True)

        # Publish to the topic '/thymioX/cmd_vel'.
        self.velocity_publisher = rospy.Publisher(self.thymio_name + '/cmd_vel',
                                                  Twist, queue_size=10)

        self.tf = tf.TransformListener()
        self.state = None
        self.odom_frame = '{}/odom'.format(self.thymio_name)
        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.'{}/base_link'.format(self.thymio_name)
        self.pose_subscriber = rospy.Subscriber(self.odom_frame,
                                                Odometry, self.update_state)
        # self.pose_publisher = rospy.Publisher('{}/cmd_vel'.format(self.thymio_name), Twist, queue_size=10)

        self.base_frame = '{}/base_link'.format(self.thymio_name)
        # self.tf.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(1.0))

        # Subscirbe to the ar_pose_markers topic
        # this helps us keep stored the data of each sensor
        self.sensors = {}
        self.current_pose = Pose()
        self.current_twist = Twist()
        self.vel_msg = Twist()
        self.starting_pose = None

        """Moves the migthy thymio"""
        self.vel_msg.linear.x = 0.00  # m/s
        self.vel_msg.angular.z = 0.0  # rad/s
        self.vel_msg.linear.y = 0.0  # m/s
        self.vel_msg.angular.y = 0.0  # rad/s
        self.vel_msg.linear.z = 0.00  # m/s
        self.vel_msg.angular.x = 0.0  # rad/s
        # stops the robot
        self.is_finished = False
        self.move_forward = False
        # publish at this rate
        self.rate = rospy.Rate(1.0)
        # flags for seeing if the robot is turning, touched the wall with fron sensors, and touched the wall with rear sensors
        self.has_touched = False
        self.yaw = 0.0
        self.starting_angle = None
        self.rotating_angle = None
        self.marker_seen = False
        # variable to keep if seen the marker
        self.markers_to_search = 2
        self.seen_marker = [0] * self.markers_to_search
        self.last_seen_marker = None
        self.receive_more_data = True
        self.time = None
        # PID variables
        self.pid = PID()
        # list of sensors, found by running the command 'rostopic list <robot_name>/proximity' in our case <robot_name> corresponds to thymio10
        # retrieving the list of topics, return value is an array of arrays of the type [[topic1, type1]...[topicN, typeN]],
        # so topic1 will be /<robotname>/proximity/center_left
        # list_of_topics = rospy.get_published_topics(namespace='/{}/proximity'.format(self.thymio_name))
        #
        # for sensor_id, sensor_name in enumerate(list_of_topics):
        #     # subscribe to sensors
        #     rospy.Subscriber(sensor_name[0], Range, self.callback, (sensor_id, sensor_name[0]))
        self.marker_frame = '{}/ar_pose_marker'.format(self.thymio_name)
        try:
            self.marker_subscriber = rospy.Subscriber(self.marker_frame, AlvarMarkers, self.update_markers)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('basic Thymio')
            pass
    def get_sensor_data(self, sensor_name):
        try:
            return self.sensors["/{}/proximity/{}".format(self.thymio_name, sensor_name)]
        except KeyError as e:
            return {'range': 0.0}

    def check_sensor_has_collided(self, curr_data):
        if curr_data.max_range > curr_data.range:
            print('collided')
            self.has_touched = True
            self.vel_msg.linear.x = 0.0
            self.velocity_publisher.publish(self.vel_msg)

    def fix_position(self, curr_data, sensor_name, step):
        right_sensor_data = self.get_sensor_data(sensor_name)
        if right_sensor_data.range == float('Inf'):
            right_sensor_data.range = right_sensor_data.max_range
        error_difference = curr_data.range - right_sensor_data.range
        pid_step = self.pid.calculate_pid_step(abs(error_difference))
        print(error_difference, pid_step)
        if abs(error_difference) == float('Inf') or abs(pid_step) == float('Inf'):
            print('inf')
            self.vel_msg.angular.z = 0
            self.is_finished = True
            self.velocity_publisher.publish(self.vel_msg)
            return
        if error_difference < 0:
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = pid_step * step
            self.velocity_publisher.publish(self.vel_msg)
        elif error_difference > 0:
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = -pid_step * step
            self.velocity_publisher.publish(self.vel_msg)
        if abs(error_difference) < 0.0001:
            self.vel_msg.angular.z = 0
            self.is_finished = True
            self.velocity_publisher.publish(self.vel_msg)


    def get_distance(self, goal_x, goal_y):
        '''
            given an x and y calculate the euclidian distance and returns it
        '''
        distance = sqrt(
            pow((goal_x - self.current_pose.position.x), 2) + pow((goal_y - self.current_pose.position.y), 2))
        return distance

    def callback(self, data, sensor_info):
        sensor_topic_name = sensor_info[1]
        curr_data = data
        if data.range == float('Inf'):
            data.range = data.max_range
        if self.is_finished and not self.move_forward:
            # if self.starting_angle == None:
            #     self.starting_angle = self.current_pose.orientation.z
            #     self.desired_angle = self.starting_angle + pi if self.starting_angle >= 0 else self.starting_angle - pi
            # # rotate by 180 degrees, formula is : degrees * pi / 180, but since degrees = 180 we have left with pi
            # angle = self.desired_angle - self.yaw
            # if (angle > pi):
            #     angle -= 2 * pi
            # if (angle <= -pi):
            #     angle += 2 * pi
            angular_vel = 0.08 if self.state != Thymio_State.PICK else 0
            self.move_forward = True
            self.pid = PID()
            # seen_all = self.marker_seen[9] and self.marker_seen[13]
            # print(seen_all, self.state == Thymio_State.SEARCHING, self.state == Thymio_State.UNLOAD)
            # if self.state == Thymio_State.SEARCHING:
            #     self.state = Thymio_State.UNLOAD
            #     self.has_touched = False
            #     print('changed state to unload')
            # elif not seen_all and self.state == Thymio_State.UNLOAD:
            #     self.state = Thymio_State.SEARCHING
            #     print('not seen all markers', seen_all, 'changing state to searching')
            # elif seen_all and self.state == Thymio_State.UNLOAD:
            #     angular_vel = 0
            #     print ('seen all')
            self.vel_msg.angular.z = angular_vel
            self.velocity_publisher.publish(self.vel_msg)
            # if abs(angular_vel) < 0.01:
            #     self.vel_msg.angular.z = 0.0
            #     self.move_forward = True
            #     self.velocity_publisher.publish(self.vel_msg)
        # elif self.is_finished and self.move_forward:
        #     if not self.starting_pose:
        #         self.starting_pose = self.current_pose
        #     pose = self.current_pose
        #     distance = self.get_distance(pose.position.x, pose.position.y)
        #     if distance >= 2:
        #         self.vel_msg.linear.x = 0.0
        #         self.velocity_publisher.publish(self.vel_msg)
        #         return
        #     else:
        #         self.vel_msg.linear.x = 0.1
        #         self.velocity_publisher.publish(self.vel_msg)
        elif not self.is_finished:
            # If it's not stored in our dictionary store it
            # initial robot pose data
            if sensor_topic_name not in self.sensors.keys():
                self.sensors[sensor_topic_name] = data


            # previous stored sensor data
            prev_data = self.sensors[sensor_topic_name]
            # current new pose data
            self.sensors[sensor_topic_name] = data

            is_left = sensor_topic_name == "/{}/proximity/{}".format(self.thymio_name, 'left')
            is_right = sensor_topic_name == "/{}/proximity/{}".format(self.thymio_name, 'right')
            if not self.has_touched and is_left:
                self.check_sensor_has_collided(curr_data)

            elif not self.has_touched and is_right:
                self.check_sensor_has_collided(curr_data)

            elif self.has_touched and is_left:

                self.fix_position(curr_data, 'right', 1)

            elif self.has_touched and is_right:
                self.fix_position(curr_data, 'left', -1)
        # if sensor_data[1] == '/thymio10/proximity/center' :
        # print "s_name: {}, fov: {}".format(sensor_data[1],self.sensors[sensor_data[1]].field_of_view)

    def thymio_state_service_request(self, position, orientation):
        """Request the service (set thymio state values) exposed by
        the simulated thymio. A teleportation tool, by default in gazebo world frame.
        Be aware, this does not mean a reset (e.g. odometry values)."""
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            model_state = ModelState()
            model_state.model_name = self.thymio_name
            model_state.reference_frame = self.odom_frame  # the frame for the pose information
            model_state.pose.position.x = position[0]
            model_state.pose.position.y = position[1]
            model_state.pose.position.z = position[2]
            qto = quaternion_from_euler(orientation[0], orientation[0], orientation[0], axes='sxyz')
            model_state.pose.orientation.x = qto[0]
            model_state.pose.orientation.y = qto[1]
            model_state.pose.orientation.z = qto[2]
            model_state.pose.orientation.w = qto[3]
            # a Twist can also be set but not recomended to do it in a service
            gms = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            response = gms(model_state)
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def update_state(self, data):
        """A new Odometry message has arrived. See Odometry msg definition."""
        # Note: Odmetry message also provides covariance
        self.current_pose = data.pose.pose
        self.current_twist = data.twist.twist
        quat = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(quat)
        self.yaw = yaw
        # rospy.loginfo("State from Odom: (%.5f, %.5f, %.5f) " % (self.current_pose.position.x, self.current_pose.position.y, yaw))

    def basic_move(self):
        print('called basic move')
        while not rospy.is_shutdown():
            # print('in shutdown')
            # Publishing thymo vel_msg
            # print(self.vel_msg)
            self.velocity_publisher.publish(self.vel_msg)
            # .. at the desired rate.
            self.rate.sleep()

        # Stop thymio. With is_shutdown condition we do not reach this point.
        # vel_msg.linear.x = 0.
        # vel_msg.angular.z = 0.
        # self.velocity_publisher.publish(vel_msg)

        # waiting until shutdown flag (e.g. ctrl+c)
        # rospy.spin()

    def update_markers(self, data):
        if self.state == Thymio_State.DONE:
            return
        try:
            mark = None
            # Looking through markers, if I have not seen one before, search for a marker or take the data of
            # an existing marker that you have already found before
            # This is just to stop the robot in case there are more marker to search but can't find none
            if self.state == Thymio_State.SEARCHING:
                now = rospy.Time.now()
                all_seen = not (0 in self.seen_marker)
                if self.time and now.secs - self.time.secs > 60 and not all_seen:
                    rospy.loginfo('There were more marker to see but saw none of them. Stopping!...')
                    self.vel_msg.angular.z = 0.00
                    self.velocity_publisher.publish(self.vel_msg)
                    self.state = Thymio_State.DONE
            elif self.state == Thymio_State.UNLOAD:
                rospy.loginfo('checking time')
                now = rospy.Time.now()
                print(self.time, now)
                if self.time and now.secs - self.time.secs > 60:
                    rospy.loginfo('I can\'t find the trash can. I lost it!. Stopping!...')
                    self.vel_msg.angular.z = 0.00
                    self.velocity_publisher.publish(self.vel_msg)
                    self.state = Thymio_State.DONE
            if not self.last_seen_marker:
                for marker in data.markers:
                    index = marker.id % self.markers_to_search
                    if self.state == Thymio_State.SEARCHING and marker.id > 8 and not self.seen_marker[index]:
                        mark = marker
                        break
                    elif self.state == Thymio_State.UNLOAD and marker.id == 4:
                        mark = marker
                        break
            else:
                lm_seen = self.last_seen_marker
                for marker in data.markers:
                    if marker.id == lm_seen.id:
                        mark = marker
                        break
            # If I saw before a marker (self.marker_seen) but it's not in the data.markers it means that I have lost it
            if not mark and not self.marker_seen:
                return
            elif not mark and self.marker_seen:
                raise IndexError

            # This is to project the marker to the array of marker seen, change self.markers_to_search if you want
            # to search for more than 2 markers (2 is not ideal because if i see a marker id with id 13, and then mark
            # with id 15, they will be considered as seen, the more markers to search the better)
            mark_index = mark.id % self.markers_to_search
            # This is the state where I'm searching for gibberish
            if self.state == Thymio_State.SEARCHING and mark.id > 8 and not self.seen_marker[mark_index]:
                rospy.loginfo("Trash found going towards it")
                # mark that I have seen a marker
                self.marker_seen = True
                # set that I have seen that mark
                self.seen_marker[mark_index] = 1
                # Keep track of the data of last seen marker
                self.last_seen_marker = deepcopy(mark)
                # move towards the marker
                self.move_to_goal(mark.pose.pose)
                # Reset the timer
                self.time = None
            elif self.state == Thymio_State.UNLOAD and mark.id == 4:
                rospy.loginfo("Trashcan Found!, moving towards it")
                # mark that I have seen a marker
                self.marker_seen = True
                # copy the data
                self.last_seen_marker = deepcopy(mark)
                # move towards it
                self.move_to_goal(mark.pose.pose)
                self.time = None
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        except IndexError:
            # Here enters when he looses a marker from the camera, so that we have to adjust it's velocity
            # What that means we have to go backwards, and in the opposite angular velocity
            if self.marker_seen:
                # If we have seen a marker, we have to reset it's flag
                if self.state == Thymio_State.SEARCHING:
                    mark_id = self.last_seen_marker.id
                    mark_index = mark_id % self.markers_to_search
                    self.seen_marker[mark_index] = 0
                # going in the opposite direction
                self.vel_msg.linear.x /= -5
                self.vel_msg.angular.z /= -10
                rospy.loginfo("Marker Lost!")
                # publish the velocity
                self.velocity_publisher.publish(self.vel_msg)
                # reset flags
                self.marker_seen = False
                return
            return

    def move_to_goal(self, coordinates):
        '''
            Move to coordinates (x,y), listener is a tf type object that is needed for looking up the transform of the 2 turtles

        '''

        # creating goal pose object with given data
        goal_pose = (Pose()).position
        
        goal_pose.x = coordinates.position.x
        goal_pose.y = coordinates.position.y
        goal_distance = 0.10
        # Since we have already the pose transformed w.r.t the base link, we need to keep truck just of the x and y
        # y is for adjusting the angle between the robot and the marker
        # x is for the distance from the marker
        # basically if it's positive rotate right
        if goal_pose.y > 0.05:  # 22.5 degrees so that the marker is in the center (rotate left)
            velocity = 0.2 * goal_pose.y
            self.vel_msg.angular.z = max(0.08, min(0.2, velocity))
        # If it's negative rotate left
        elif goal_pose.y < -0.05:  # 22.5 degrees so that the marker is in the center (rotate left)
            velocity = 0.2 * goal_pose.y
            self.vel_msg.angular.z = min(-0.08, max(-0.2, velocity))
        # stop
        else:
            self.vel_msg.angular.z = 0
        # We check how distant is the robot from the marker
        diff = goal_pose.x - goal_distance
        # if it's far from the goal
        if abs(diff) > 0.08:
            # we divided by 100 to move it slowly so that when it's moving slowly it will not lose the marker due the blur effect
            lin_vel = diff/100
            # if it's negative it means that we overshooted and went too far
            if lin_vel < 0:
                # need tp go back asap
                lin_vel = lin_vel * 0.5
            self.vel_msg.linear.x = copysign(max(0.05, min(0.05, abs(lin_vel))), lin_vel)
        # stop
        else:
            self.vel_msg.linear.x = 0
        self.velocity_publisher.publish(self.vel_msg)
        is_stopped = not self.vel_msg.linear.x and not self.vel_msg.angular.z
        # if the robot has stopped e.g linear and angular velocity are zero
        if is_stopped:
            rospy.loginfo('Reached desired goal.')
            # reset the flag, since wee need to see a new marker
            self.marker_seen = False
            # Scan through th array of seen markers, if there is a 0 in there it means that there are more marker to see
            all_seen = not (0 in self.seen_marker)
            # if there are no more marker to see just stop
            if self.state == Thymio_State.UNLOAD and all_seen:
                rospy.loginfo('No more markers to see, shutting down.....')
                self.vel_msg.angular.z = 0.00
                self.velocity_publisher.publish(self.vel_msg)
                self.state = Thymio_State.DONE
            else:
                # If I'm at the trash can just stop for a moment before continuing (this is to clear
                # the array of the vel_publisher, which means give him some time to publish all other messages)
                if self.state == Thymio_State.UNLOAD:
                    self.state = Thymio_State.SLEEP
                    rospy.loginfo('Unloading the trash give me 5 seconds')
                    rospy.sleep(5)
                    rospy.loginfo('Trash unloaded')
                    self.state = Thymio_State.UNLOAD
                # switch states
                rospy.loginfo('Switching states...')
                if self.state == Thymio_State.SEARCHING:
                    rospy.loginfo('Looking for trashcan')
                    if not self.time:
                        self.time = rospy.Time.now()
                elif self.state == Thymio_State.UNLOAD:
                    rospy.loginfo('Looking for more markers')
                    # I have unloaded the trash, now i have to keep track of the time elapsed
                    if not self.time:
                        self.time = rospy.Time.now()
                self.state = Thymio_State.UNLOAD if self.state == Thymio_State.SEARCHING else Thymio_State.SEARCHING
                self.vel_msg.angular.z = 0.09
                self.velocity_publisher.publish(self.vel_msg)
            self.last_seen_marker = None

def usage():
    return "Wrong number of parameters. myT.py [thymio_name]"


if __name__ == '__main__':
    if len(sys.argv) >= 2:
        thymio_name = sys.argv[1]
        print "Now working with robot: %s" % thymio_name
    else:
        print usage()
        sys.exit(1)
    try:
        thymio = BasicThymio(thymio_name)
        rospy.loginfo('Give me 10 sec to set everything up')
        rospy.sleep(10.)
        thymio.state = Thymio_State.SEARCHING
        # Teleport the robot to a certain pose. If pose is different to the
        # origin of the world, you must account for a transformation between
        # odom and gazebo world frames.
        # NOTE: The goal of this step is *only* to show the available
        # tools. The launch file process should take care of initializing
        # the simulation and spawning the respective models
        # thymio.thymio_state_service_request([0., 0., 0.], [0., 0., 0.])
        # rospy.sleep(1.)
        rospy.loginfo('Good to go, let me search')
        while not rospy.is_shutdown():
            continue
        thymio.vel_msg.linear.x = 0
        thymio.vel_msg.angular.z = 0
        thymio.velocity_publisher.publish(thymio.vel_msg)
    except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
    except:
        pass