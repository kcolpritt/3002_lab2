import rospy, tf, copy, math

from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String



class Robot:

    def __init__(self):

        """
            This constructor sets up class variables and pubs/subs
        """

        self._current = Pose() # initlize correctly
        self._odom_list = tf.TransformListener()
        rospy.Timer(rospy.Duration(.1), self.timerCallback)
        self._vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        #rospy.Subscriber('YOUR_STRING_HERE', ..., self.navToPose, queue_size=1) # handle nav goal events


    def navToPose(self,goal):
        """
            This is a callback function. It should exract data from goal, drive in a striaght line to reach the goal and
            then spin to match the goal orientation.
        """

    #self._odom_list.waitForTransform('YOUR_STRING_HERE', 'YOUR_STRING_HERE', rospy.Time(0), rospy.Duration(1.0))
    #transGoal = self._odom_list.transformPose('YOUR_STRING_HERE', goal) # transform the nav goal from the global coordinate system to the robot's coordinate system

    def executeTrajectory(self):
        self.driveStraight(.2, .6)
        rospy.sleep(.1)
        self.rotate(-1.57)
        rospy.sleep(.1)
        self.driveStraight(.2,.45)
        rospy.sleep(.1)
        self.rotate(0)
        rospy.sleep(.1)
        """
            See lab manual for the dance the robot has to excute
        """

    def driveStraight(self, speed, distance):
      """
        This method should populate a ??? message type and publish it to ??? in order to move the robot
      """

      isThere = False
      origin = copy.deepcopy(self._current) #hint:  use this
      while not isThere and not rospy.is_shutdown():
          currentPosition = copy.deepcopy(self._current)

          currentDistance = math.sqrt(((currentPosition.position.x - origin.position.x)**2) + ((currentPosition.position.y - origin.position.y)**2))

          if (currentDistance >= distance):
              isThere = True
              self.publishTwist(0,0)
          else:
              self.publishTwist(speed,0)
              rospy.sleep(0.1)
              print currentDistance
              print 'not there'
              #print currentPosition

    def spinWheels(self, v_left, v_right, time):
        diameter = 0.16 # based on wheel track from https://yujinrobot.github.io/kobuki/doxygen/enAppendixKobukiParameters.html
        twist_msg = Twist();
        stop_msg = Twist();
        R = (diameter/2) * (v_left + v_right) / (v_right - v_left)
        w = (v_right - v_left)/diameter
        linearVel = w * R
        twist_msg.linear.x = linearVel
        twist_msg.angular.z = w

        driveStartTime = rospy.Time.now().secs
        print linearVel
        print w
        print driveStartTime
        while (rospy.Time.now().secs - driveStartTime <= time and not rospy.is_shutdown()):
            self.publishTwist(linearVel, w)
        self.publishTwist(0,0)
        """
            This method should use differential drive kinematics to compute V and omega (linear x = V, angular z = omega).
            It should then create a ??? message type, and publish it to ??? in order to move the robot
        """


    def rotate(self,angle):
        """
            This method should populate a ??? message type and publish it to ??? in order to spin the robot
        """


        origin = copy.deepcopy(self._current)
        q = [origin.orientation.x,
        origin.orientation.y,
        origin.orientation.z,
        origin.orientation.w] # quaternion nonsense
        (roll, pitch, yaw) = euler_from_quaternion(q)
        initialYaw = yaw
        atAngle = False
        highRange = .01 + angle
        lowRange = angle - .01
        while not atAngle and not rospy.is_shutdown():
            origin = copy.deepcopy(self._current)
            q = [origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w] # quaternion nonsense
            (roll, pitch, yaw) = euler_from_quaternion(q)
            currentYaw = yaw
            if currentYaw <= angle:
                if currentYaw >= highRange or currentYaw <= lowRange:
                    self.publishTwist(0,0.25)
                    print 'not there'
                    print currentYaw
                    currentYaw = yaw
                else:
                    self.publishTwist(0,0)
                    print 'im there'
                    atAngle = True
            else:
                if currentYaw >= highRange or currentYaw <= lowRange:
                    print'not there'
                    self.publishTwist(0, -0.25)
                    currentYaw = yaw
                else:
                    self.publishTwist(0,0)
                    print 'im there'
                    atAngle = True



    def timerCallback(self,evprent):
        """
            This is a callback that runs every 0.1s.
            Updates this instance of Robot's internal position variable (self._current)
        """
	# wait for and get the transform between two frames
        self._odom_list.waitForTransform('/odom', '/base_footprint', rospy.Time(0), rospy.Duration(1.0))
        (position, orientation) = self._odom_list.lookupTransform('/odom','/base_footprint', rospy.Time(0))
	     # save the current position and orientation
        self._current.position.x = position[0]
        self._current.position.y = position[1]
        self._current.orientation.x = orientation[0]
        self._current.orientation.y = orientation[1]
        self._current.orientation.z = orientation[2]
        self._current.orientation.w = orientation[3]

	        # create a quaternion
        q = [self._current.orientation.x,
             self._current.orientation.y,
             self._current.orientation.z,
             self._current.orientation.w]

	       # convert the quaternion to roll pitch yaw
        (roll, pitch, yaw) = euler_from_quaternion(q)
        self.yaw = yaw

# helper functions
    def planTraj(self, b, t):


        """
            Bonus Question:  compute the coefs for a cubic polynomial (hint:  you did this in 3001)
        """

    def publishTwist(self, linearVelocity, angularVelocity):
        msg = Twist()
        msg.linear.x = linearVelocity
        msg.angular.z = angularVelocity
        self._vel_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('drive_base')

    turtle = Robot()
    #turtle.spinWheels(30,40,10000)
    #turtle.driveStraight(.5,2)
    #turtle.rotate(1)
    turtle.executeTrajectory()
    #test function calls here

    while  not rospy.is_shutdown():
        pass
