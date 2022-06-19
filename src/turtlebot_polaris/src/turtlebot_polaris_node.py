#!/usr/bin/env python

import rospy
import tf
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from turtlebot_polaris.srv import PlaceTurtlebot

'''
This project is developed for Xihelm interview. This is an implementation of "continuously orientates a
turtlebot to Polaris (the north star) from anywhere on Earth" on Python and ROS.
'''

class Orientate_polaris():

    def __init__(self):
        """
        Initialize the parameters

        """
        self.angular_speed = 1.0 # default anglar speed of robot 1 rad/sec 
        self.bearing = 0

        # true north coordinate value
        self.north_latitude = 90
        self.north_longitude = 0
        

    def imu_callback(self, msg):
        """
        IMU callback

        Args:
            msg: sensor_msgs/Imu.msg
        """
        quaternion = [msg.orientation.x, msg.orientation.y,
                      msg.orientation.z, msg.orientation.w]
        rob_yaw = tf.transformations.euler_from_quaternion(quaternion)[2]

        goal = 0.5 * math.pi - self.bearing # assuming the defult orentation of turtlebot is due East
        self.move(rob_yaw, goal)


    def move(self, cur_yaw, goal_yaw):
        """
        Rotate the turtlebot to the north

        Args
        ----------
        cur_yaw: robot current yaw read from IMU
        goal_yaw: the target yaw
        """
        rate = rospy.Rate(100)
        msg = Twist()

        if cur_yaw < goal_yaw - 0.1 or cur_yaw > goal_yaw + 0.1: # set tolerance to 0.1, when current poisiton reach target +/- 0.1 stop.
            # angular.z positive is turn left and negative turn right
            msg.angular.z = self.angular_speed if -0.5 * math.pi < cur_yaw < 0.5 * math.pi else -self.angular_speed
        else:
            msg.angular.z = 0
        
        self.pub.publish(msg)
        rate.sleep()



    def get_bearing(self, robot_poistion, true_north):
        """
        Calculates the bearing between two points.

        Args
        ----------
        robot_poistion: latitude and longitude of robot GPS position
        true_north: latitude and longitude of north

        Returns
        -------
        Bearing: rad
            Bearing in rad between the robot poistion and true north.
        """

        robot_lat = math.radians(robot_poistion[0])
        robot_lng = math.radians(robot_poistion[1])
        true_north_lat = math.radians(true_north[0])
        true_north_lng = math.radians(true_north[1])

        d_lng = true_north_lng - robot_lng

        if abs(d_lng) > math.pi:
            if d_lng > 0.0:
                d_lng = -(2.0 * math.pi - d_lng)
            else:
                d_lng = (2.0 * math.pi + d_lng)

        tan_start = math.tan(robot_lat / 2.0 + math.pi / 4.0)
        tan_end = math.tan(true_north_lat / 2.0 + math.pi / 4.0)
        d_phi = math.log(tan_end / tan_start)    
        bearing = math.degrees(math.atan2(d_lng, d_phi))
        return math.radians(bearing)

    def run(self):
        """
        Start the program
        """

        rospy.loginfo("Please run place_turtlebot_server to place robot in anywhere of earth")  
        rospy.wait_for_service('place_turtlebot')
        
        self.place_turtlebot = rospy.ServiceProxy('place_turtlebot', PlaceTurtlebot)
        turtlebot_position = self.place_turtlebot()

        self.bearing = self.get_bearing([turtlebot_position.latitude, turtlebot_position.longitude],[self.north_latitude,self.north_longitude])

        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.sub_imu = rospy.Subscriber('/mobile_base/sensors/imu_data', Imu, self.imu_callback)

if __name__ == '__main__':

    rospy.init_node('turtlebot_polaris_node', anonymous=True)

    turtlebot_polaris = Orientate_polaris()
    turtlebot_polaris.run()

    rospy.spin()