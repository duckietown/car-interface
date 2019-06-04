#!/usr/bin/env python

import rospy
from car_interface import CarInterface


def on_shutdown():
  rospy.loginfo("[%s] Shutting down." % rospy.get_name())

def main():
  # Initialize the node with rospy
  rospy.init_node('car_interface_node', anonymous=False)
  # Create the DaguCar object
  iface = CarInterface()
  # Setup proper shutdown behavior
  rospy.on_shutdown(on_shutdown)
  # Keep it spinning to keep the node alive
  rospy.spin()

if __name__ == '__main__':
  main()
