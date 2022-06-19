#!/usr/bin/env python

import rospy
from turtlebot_polaris.srv import PlaceTurtlebot, PlaceTurtlebotResponse

def place_turtlebot(req):
    latitude = input("Please enter the latitude where you want to place turtlebot: (-90 to 90)\n")
    longitude = input("Please enter the longitude where you want to place turtlebot: (-180 to 180)\n")
    return PlaceTurtlebotResponse(latitude, longitude)


def place_turtlebot_server():
    rospy.init_node("place_turtlebot_server")
    s = rospy.Service("place_turtlebot", PlaceTurtlebot, place_turtlebot)
    rospy.spin()


if __name__ == "__main__":
    place_turtlebot_server()