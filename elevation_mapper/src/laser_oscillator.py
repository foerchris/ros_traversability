#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import roslib; roslib.load_manifest('elevation_mapper')
import rospy

import math
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Imu

from dynamixel_msgs.msg import JointState

import tf

# Create the Simple Action Server
class DynamixelServer():

    def __init__(self, name, min_angle, max_angle):
        
        #Publisher für Servoposition Neigeeinheit (Drehwinkel)
        self.position_publisher_tilt = rospy.Publisher('/GETjag/laser_scan_front_pitch_controller/command', Float64, queue_size=1)
        
        #Publisher für Serovposition Rolleinheit (Drehwinkel)
        self.position_publisher_roll = rospy.Publisher('/GETjag/laser_scan_front_roll_controller/command', Float64, queue_size=1)
        
        #Subscriber zum empfangen der aktuellen IMU Daten
        rospy.Subscriber('/GETjag/imu/data', Imu, self.imu_callback)

        #TransformBroadcaster
        self.bc = tf.TransformBroadcaster()
        
        self.t = 0
        self.angle_amplitude = math.radians((max_angle - min_angle) / 2)
        self.angle_offset = math.radians((max_angle + min_angle) / 2)
        self.angle_step = 0.05
        
        # Move servo to init angle
        self.position_publisher_tilt.publish(Float64(0))
        self.position_publisher_roll.publish(Float64(0))
        
        self.angle_hr = 0
        self.angle_ht = 0


    # IMU callback
    def imu_callback (self, data):
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.angle_hr = -roll
        self.angle_ht = -pitch
    
    def update_laser_position(self):
        self.position_publisher_tilt.publish (Float64(self.angle_ht - (math.sin(self.t) * self.angle_amplitude + self.angle_offset)))
        self.position_publisher_roll.publish (Float64(self.angle_hr))        
        self.t += self.angle_step

if __name__ == '__main__':
    rospy.init_node('orientation_correction')
    try:
        dyn = DynamixelServer (rospy.get_name(), -20, 10)
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            dyn.update_laser_position()
            r.sleep()
    except rospy.ROSInterruptException: pass
