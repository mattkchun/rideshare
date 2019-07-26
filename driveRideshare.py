#/usr/bin/env python

import rospy
import time
import math
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Joy, Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped


DRIVE_TOPIC = "/drive"
SCAN_TOPIC = "/scan"
AR_TOPIC = "/ar_pose_marker"
POS_TOPiC = "pf/viz/inferred_pose"

class ARDrive(object):
    def __init__(self):
        rospy.init_node("ar")
        #initialize publishers and subscribers
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size = 1)
        self.scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.driveCallback)
        self.ar_sub = rospy.Subscriber(AR_TOPIC, AlvarMarkers, self.arCallback)
        self.pos_sub = rospy.Subscriber(POS_TOPiC, PoseStamped, self.posCallback)
        self.sound_pub = rospy.Publisher("state", String, queue_size=2)
        
        #initialize cmd object
        self.cmd = AckermannDriveStamped()
        self.cmd.drive.speed = 0
        self.cmd.drive.steering_angle = 0
    
        self.locations = {"starting": (0.5,0.72), "pickup": (-0.8,2.35), "1": (-0.6,-0.9), "2": (-2.15,-1.7), "5": (-3.15,-0.2), "center": (-1.74,0.26)}
        self.person_order = [0,0,0]
        self.states = ["scanning", "picking_up", "dropping_off", "stopped"]
        #self.states = [2,5]
        self.state = self.states[1]
        self.dropped_off = 0
        self.target_house = 9
        self.values = [0.2,-0.2,0,-1,1,0]
        self.current_angle = 0
        self.change_angle = 0
        self.pos = None
        self.reached_middle = False

    def driveCallback(self, data):
        '''LIDAR callback, sets drive commands'''
        #TODO: Set drive commands according to the current state
        print(self.person_order)
        print(self.state)
        if self.state == "picking_up":
            if self.dropped_off>0 and self.person_order[self.dropped_off-1] == 5 and self.reached_middle == False:
                target_angle = self.get_angle((self.pos.x,self.pos.y),(self.locations["center"]))
                self.change_angle = (target_angle - self.current_angle)/(math.pi/2)
                self.cmd.drive.speed = 0.5
                self.cmd.drive.steering_angle = self.change_angle
                if self.get_diff((self.pos.x,self.pos.y),(self.locations["center"]))<0.5:
                    self.reached_middle = True
            else:
                target_angle = self.get_angle((self.pos.x,self.pos.y),(self.locations["pickup"]))
                self.change_angle = (target_angle - self.current_angle)/(math.pi/2)
                self.cmd.drive.speed = 0.5
                self.cmd.drive.steering_angle = self.change_angle
                if self.get_diff((self.pos.x,self.pos.y),(self.locations["pickup"]))<0.5:
                    self.state = "dropping_off"
                    if self.person_order[self.dropped_off-1] == 5:
                        self.reached_middle = False
                    if self.person_order[2] != 0:
                        self.state = "stopped"
        elif self.state == "dropping_off":
            self.drop_passenger()
        elif self.state == "stopped":
            self.cmd.drive.speed = 0
            self.cmd.drive.steering_angle = 0
            return
        rospy.loginfo("change: {}".format(self.change_angle))
#        self.cmd.drive.speed = 0.5
#        self.cmd.drive.steering_angle = self.change_angle
        #rospy.loginfo("states: {}".format(self.states))

    def arCallback(self, tags):
        '''Callback when an AR tag is detected'''
        #TODO: Write your state changes here
        if len(tags.markers)>0:
            if tags.markers[0].id not in self.person_order and self.person_order[2] == 0:
                self.person_order[self.dropped_off] = tags.markers[0].id

#        for tag in tags.markers:
#            if tag.id <= 2:
#                self.states[0] = tag.id
#            else:
#                self.states[1] = tag.id
#        if self.states[0] != self.last_states[0]:
#            print(self.states[0])
#            self.sound_pub.publish(str(self.states[0]))
#            self.last_states[0] = self.states[0]
#        elif self.states[1] != self.last_states[1]:
#            print(self.states[1])
#            self.sound_pub.publish(str(self.states[1]))
#            self.last_states[1] = self.states[1]
                        
    def quatToAng3D(self,quat):
        euler = euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
        return euler

    def posCallback(self, pose):
        self.pos = pose.pose.position
        quat = pose.pose.orientation
        euler = self.quatToAng3D(quat)
        rospy.loginfo("pos: {}".format(self.pos))
        #rospy.loginfo("euler: {} yaw: {}".format(euler, euler[2]))
        self.current_angle = euler[2]

    def get_angle(self, point1, point2):
        x_diff=point2[0]-point1[0]
        y_diff=point2[1]-point1[1]
        angle = math.atan2(y_diff,x_diff)
        return angle
    
    def get_diff(self, point1, point2):
        x_diff=point2[0]-point1[0]
        y_diff=point2[1]-point1[1]
        total_diff = math.hypot(x_diff,y_diff)
        return abs(total_diff)

    def drop_passenger(self):
        print(self.reached_middle)
        passenger = self.person_order[self.dropped_off]
        if passenger == 5 and self.reached_middle == False:
            target_angle = self.get_angle((self.pos.x,self.pos.y),(self.locations["center"]))
            self.change_angle = (target_angle - self.current_angle)
            if self.change_angle>0:
                self.change_angle = math.pi - self.change_angle
            else:
                self.change_angle = -math.pi - self.change_angle
            self.change_angle /= -math.pi
            self.cmd.drive.speed = -0.5
            self.cmd.drive.steering_angle = -self.change_angle
            if self.get_diff((self.pos.x,self.pos.y),(self.locations["center"]))<1.0:
                self.reached_middle = True
        else:
            target_angle = self.get_angle((self.pos.x,self.pos.y),(self.locations[str(passenger)]))
            self.change_angle = (target_angle - self.current_angle)
            if self.change_angle>0:
                self.change_angle = math.pi - self.change_angle
            else:
                self.change_angle = -math.pi - self.change_angle
            self.change_angle /= -math.pi
            #            self.change_angle += math.pi
            #            if self.change_angle>math.pi:
            #                self.change_angle = self.change_angle - (2*math.pi)
            self.cmd.drive.speed = -0.5
            self.cmd.drive.steering_angle = -self.change_angle
            if self.get_diff((self.pos.x,self.pos.y),(self.locations[str(passenger)]))<0.5:
                self.state = "picking_up"
                self.dropped_off += 1
                if passenger == 5:
                    self.reached_middle = False

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



    

