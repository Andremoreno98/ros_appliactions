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

class DriveOmniSecondPoint(EventState):
    '''
    Driving state for a ground robot. The state allows to drive the robot
    horizontally from the current position (x,y) to the position (x,-y)

    <= failed               If behavior is unable to ready on time
    <= done                 Dingo reached the Goal Position

    '''

    def __init__(self):
        super(DriveOmniSecondPoint, self).__init__(outcomes=['failed', 'done'])
        self._start_time = None
        self.data = None
        self.list_odo_x = []
        self.list_odo_y = []
        self.goal = Point()
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_theta = 0.0

        self.vel_topic = '/cmd_vel'

        # create publisher to set the corresponding vx, vy on the dingo velocity topic
        self.pub = ProxyPublisher({self.vel_topic: Twist})

    def execute(self, userdata):

        """ The current position (x,y) is subscribed and the distance to the position (x,-y)
            is calculated. The VY velocity is published in the Topic until Dingo has reached
            the position (x,-y)"""

        if not self.cmd_pub:
            return 'failed'

        if self.initial_x != 0.0 and self.initial_y != 0.0:
            global start_x
            global start_y

            start_x = self.list_odo_x[0]
            goal_x = start_x
            start_y = math.ceil(self.list_odo_y[0] + 0.1)
            goal_y = start_y - (2 * start_y)

            vx = 0.0
            vy = -0.5

            # measure distance travelled
            elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
            distance_travelled = elapsed_time * abs(vy)
            distance_to_drive = np.sqrt((goal_x - start_x) ** 2 + (goal_y - start_y) ** 2)

            if (distance_to_drive - abs(distance_travelled)) > 0.02: #decreasing the difference adjust the accuracy of the last position
                self.cmd_pub.linear.x = vx
                self.cmd_pub.linear.y = vy
                self.pub.publish(self.vel_topic, self.cmd_pub)
            else:
                print("Dingo reached the Goal Position")
                #print("start_x", start_x)
                #print("start_y", start_y)
                #print("goal_y", goal_y)
                #print("self.list_odo_y[0]", self.list_odo_y[0])
                #print("last_pose_x", self.initial_x)
                #print("last_pose_y", self.initial_y)
                #print("distance_to_drive", distance_to_drive)

                return 'done'


    def on_enter(self, userdata):
        """This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        It is primarily used to start actions which are associated with this state"""

        Logger.loginfo("Drive FWD STARTED!")
        # set robot speed here
        self.cmd_pub = Twist()
        # create subscriber to get the current position in real time
        self.sub_odo_second = rospy.Subscriber("/odometry/filtered", Odometry, self.newOdom)
        self._start_time = rospy.Time.now()


    def on_exit(self, userdata):
        """This method is called when an outcome is returned and another state gets active.
        After Dingo reaches the goal the speed are set equal to zero"""

        self.cmd_pub.linear.x = 0.0
        self.cmd_pub.linear.y = 0.0
        self.pub.publish(self.vel_topic, self.cmd_pub)
        # clear odo list before exit
        self.list_odo_x.clear()
        self.list_odo_y.clear()

        Logger.loginfo("Drive FWD ENDED!")


    def on_start(self):
        """ This method is called when the behavior is started"""

        Logger.loginfo("Drive FWD READY!")

    def on_stop(self):
        """This method is called whenever the behavior stops execution, also if it is cancelled"""

    def newOdom(self, msg):
        """Callback function to subscribe current position in real time"""

        self.intial_pose = msg.pose.pose.position
        self.initial_x = msg.pose.pose.position.x
        self.initial_y = msg.pose.pose.position.y

        self.list_odo_x.append(self.initial_x)
        self.list_odo_y.append(self.initial_y)


        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.initial_theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
