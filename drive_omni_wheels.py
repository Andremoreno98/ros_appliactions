#!/usr/bin/env python3
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2
import numpy as np
import math


class DriveOmniWheelsPoint(EventState):
    '''
    Driving state for an Omni wheel robot. This state allows the robot to drive
    to a specific position without rotating the robot during the trajectory.

    -- drive_to_x   float   Goal Position on the X axis
    -- drive_to_y   float   Goal Position on the Y axis

    <= failed               If behavior is unable to ready on time
    <= done                 Dingo reached the Goal Position

    '''

    def __init__(self, drive_to_x, drive_to_y):
        super(DriveOmniWheelsPoint, self).__init__(outcomes=['failed', 'done'])
        self._start_time = None
        self.data = None
        self.pose_in_x = drive_to_x
        self.pose_in_y = drive_to_y
        self.goal = Point()
        self.goal.x = self.pose_in_x
        self.goal.y = self.pose_in_y
        self.list_odo_x = []
        self.list_odo_y = []
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_theta = 0.0
        self.final_pose = None

        self.vel_topic ='/cmd_vel'

        # create publisher to set the corresponding vx, vy on the dingo velocity topic
        self.pub = ProxyPublisher({self.vel_topic: Twist})
        # create subscriber to get the current position in real time
        self.sub_odo = rospy.Subscriber("/odometry/filtered", Odometry, self.newOdom)

    def execute(self, userdata):
        """
        vx and vy represent vectors of Dingo's velocity.
        Once a pose is defined in the ROS Flexbi, the speed vy is calculated for the
        robot to drive in the direction of the final pose.
        While the robot has not reached the position, it continues to publish speeds
        """

        if not self.cmd_pub:
            return 'failed'

        if (self.intial_pose is not None):
            global start_x
            global start_y

            start_x = self.list_odo_x[0]
            #goal_x = start_x
            start_y = self.list_odo_y[0]
            dif_x = self.goal.x - start_x
            dif_y = self.goal.y - start_y

            vx = 0.5
            vy = (dif_y / dif_x) * vx
            res_vel = np.sqrt((vx) ** 2 + (vy) ** 2)
            # measure distance travelled
            elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
            distance_travelled = elapsed_time * res_vel
            distance_to_drive = np.sqrt((dif_x) ** 2 + (dif_y) ** 2)

            if (abs(distance_travelled - distance_to_drive) > 0.1): #decreasing the difference adjust the accuracy of the last position
                self.cmd_pub.linear.x = vx
                self.cmd_pub.linear.y = vy
                self.pub.publish(self.vel_topic, self.cmd_pub)
            else:
                print("Dingo reached the Goal Position")
                #print("last_pose_x", self.initial_x)
                #print("last_pose_y", self.initial_y)
                #print("distance_to_drive", distance_to_drive)
                return 'done'


    def on_enter(self, userdata):
        """
        This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        It is primarily used to start actions which are associated with this state.
        """
        Logger.loginfo("Drive FWD STARTED!")
        # set robot speed here
        self.cmd_pub = Twist()
        self._start_time = rospy.Time.now()

    def on_exit(self, userdata):
        """
        This method is called when an outcome is returned and another state gets active.
        After Dingo reaches the first goal the speed are set equal to zero
        """

        self.cmd_pub.linear.x = 0.0
        self.cmd_pub.linear.y = 0.0
        self.pub.publish(self.vel_topic, self.cmd_pub)

        #clear odo list before exit
        self.list_odo_x.clear()
        self.list_odo_y.clear()
        Logger.loginfo("Drive FWD ENDED!")

    def on_start(self):
        """ This method is called when the behavior is started"""

        Logger.loginfo("Drive FWD READY!")

    def on_stop(self):
        """This method is called whenever the behavior stops execution, also if it is cancelled"""
        Logger.loginfo("Drive FWD STOPPED!")

    def newOdom(self, msg):
        """Callback function to subscribe current position in real time"""
        self.intial_pose = msg.pose.pose.position
        self.initial_x = msg.pose.pose.position.x
        self.initial_y = msg.pose.pose.position.y
        self.list_odo_x.append(self.initial_x)
        self.list_odo_y.append(self.initial_y)

        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.initial_theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
