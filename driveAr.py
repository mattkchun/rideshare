#/usr/bin/env python

import rospy
import time
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Joy, Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String


DRIVE_TOPIC = "/drive"
SCAN_TOPIC = "/scan"
AR_TOPIC = "/ar_pose_marker"

class ARDrive(object):
    def __init__(self):
        rospy.init_node("ar")
        #initialize publishers and subscribers
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size = 1)
        self.scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.driveCallback)
        self.ar_sub = rospy.Subscriber(AR_TOPIC, AlvarMarkers, self.arCallback)
        self.sound_pub = rospy.Publisher("state", String, queue_size=2)
        
        #initialize cmd object
        self.cmd = AckermannDriveStamped()
        self.cmd.drive.speed = 0
        self.cmd.drive.steering_angle = 0
    
        self.states = [2,5]
        self.last_states = [2,5]
        self.values = [0.2,-0.2,0,-1,1,0]

    def driveCallback(self, data):
        '''LIDAR callback, sets drive commands'''
        #TODO: Set drive commands according to the current state
        self.cmd.drive.speed = self.values[self.states[0]]
        self.cmd.drive.steering_angle = self.values[self.states[1]]
        rospy.loginfo("states: {}".format(self.states))

    def arCallback(self, tags):
        '''Callback when an AR tag is detected'''
        #TODO: Write your state changes here
        for tag in tags.markers:
            if tag.id <= 2:
                self.states[0] = tag.id
            else:
                self.states[1] = tag.id
        if self.states[0] != self.last_states[0]:
            print(self.states[0])
            self.sound_pub.publish(str(self.states[0]))
            self.last_states[0] = self.states[0]
        elif self.states[1] != self.last_states[1]:
            print(self.states[1])
            self.sound_pub.publish(str(self.states[1]))
            self.last_states[1] = self.states[1]

def main():
    try:
        ic = ARDrive()
        rospy.Rate(100)
        while not rospy.is_shutdown():
            ic.drive_pub.publish(ic.cmd)
    except rospy.ROSInterruptException:
        exit()

if __name__ == "__main__":
    main()



    

