#!/usr/bin/env python
import os
import rospy
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
from duckietown_msgs.srv import SetValueRequest, SetValueResponse, SetValue
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty


class CarInterface(object):

  def __init__(self):
    # get node name and vehicle name
    self.node_name = rospy.get_name()
    self.veh_name = os.environ['VEHICLE_NAME']

    # Set local variable by reading parameters
    self.gain = self.setup_parameter("~gain", 0.6)
    self.trim = self.setup_parameter("~trim", 0.0)
    self.baseline = self.setup_parameter("~baseline", 0.1)
    self.radius = self.setup_parameter("~radius", 0.0318)
    self.k = self.setup_parameter("~k", 27.0)
    self.limit = self.setup_parameter("~limit", 1.0)
    self.limit_max = 1.0
    self.limit_min = 0.0

    self.v_max = 999.0     # TODO: Calculate v_max !
    self.omega_max = 999.0     # TODO: Calculate v_max !

    # Prepare services
    self.srv_set_gain = rospy.Service("~set_gain", SetValue, self.cb_srv_set_gain)
    self.srv_set_trim = rospy.Service("~set_trim", SetValue, self.cb_srv_set_trim)
    self.srv_set_baseline = rospy.Service("~set_baseline", SetValue, self.cb_srv_set_baseline)
    self.srv_set_radius = rospy.Service("~set_radius", SetValue, self.cb_srv_set_radius)
    self.srv_set_k = rospy.Service("~set_k", SetValue, self.cb_srv_set_k)
    self.srv_set_limit = rospy.Service("~set_limit", SetValue, self.cb_srv_set_limit)
    self.srv_save = rospy.Service("~save_calibration", Empty, self.cb_srv_save_calibration)

    # Setup the publisher and subscriber
    self.sub_car_cmd = rospy.Subscriber("~car_cmd", Twist2DStamped, self.cb_car_cmd)
    self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)

  def save_calibration(self):
    return

  def load_calibration(self):
    return

  def cb_srv_save_calibration(self, req):
    self.save_calibration()
    return EmptyResponse()

  def cb_srv_set_gain(self, req):
    self.gain = req.value
    self.set_param('~gain', req.value)
    return SetValueResponse()

  def cb_srv_set_trim(self, req):
    self.trim = req.value
    self.set_param('~trim', req.value)
    return SetValueResponse()

  def cb_srv_set_baseline(self, req):
    self.baseline = req.value
    self.set_param('~baseline', req.value)
    return SetValueResponse()

  def cb_srv_set_radius(self, req):
    self.radius = req.value
    self.set_param('~radius', req.value)
    return SetValueResponse()

  def cb_srv_set_k(self, req):
    self.k = req.value
    self.set_param('~k', req.value)
    return SetValueResponse()

  def cb_srv_set_limit(self, req):
    self.limit = self.set_limit(req.value)
    self.set_param('~limit', self.limit)
    return SetValueResponse()

  def set_limit(self, value):
    if value > self.limit_max:
      rospy.logwarn("[%s] limit (%s) larger than max at %s" % (self.node_name, value, self.limit_max))
      limit = self.limit_max
    elif value < self.limit_min:
      rospy.logwarn("[%s] limit (%s) smaller than allowable min at %s" % (self.node_name, value, self.limit_min))
      limit = self.limit_min
    else:
      limit = value
    return limit

  def cb_car_cmd(self, msg_car_cmd):
    # assuming same motor constants k for both motors
    k_r = self.k
    k_l = self.k

    # adjusting k by gain and trim
    k_r_inv = (self.gain + self.trim) / k_r
    k_l_inv = (self.gain - self.trim) / k_l

    omega_r = (msg_car_cmd.v + 0.5 * msg_car_cmd.omega * self.baseline) / self.radius
    omega_l = (msg_car_cmd.v - 0.5 * msg_car_cmd.omega * self.baseline) / self.radius

    # conversion from motor rotation rate to duty cycle
    u_r = omega_r * k_r_inv
    u_l = omega_l * k_l_inv

    # limiting output to limit, which is 1.0 for the duckiebot
    u_r_limited = max(min(u_r, self.limit), -self.limit)
    u_l_limited = max(min(u_l, self.limit), -self.limit)

    # wrap the wheel commands in a message and publish
    msg_wheels_cmd = WheelsCmdStamped()
    msg_wheels_cmd.header.stamp = msg_car_cmd.header.stamp
    msg_wheels_cmd.vel_right = u_r_limited
    msg_wheels_cmd.vel_left = u_l_limited
    self.pub_wheels_cmd.publish(msg_wheels_cmd)

  def set_param(self, param_name, param_value):
    # write to parameter server
    rospy.set_param(param_name, param_value)
    rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, param_value))

  def setup_parameter(self, param_name, default_value):
    value = rospy.get_param(param_name, default_value)
    # Write to parameter server for transparency
    self.set_param(param_name, value)
    return value
