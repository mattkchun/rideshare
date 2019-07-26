#/usr/bin/env python

#import libraries and color segmentation code
import rospy
import cv2
import time
import numpy as np
from zed import Zed_converter
from cv_bridge import CvBridge, CvBridgeError
from color_segmentation import cd_color_segmentation
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Joy, Image

#initalize global variables
DRIVE_TOPIC = "/drive"
IMAGE_TOPIC = "/zed/zed_node/color_seg_output"

AUTONOMOUS_MODE = True

class driveStop(object):
    """class that will help the robot drive and stop at certain conditions"""
    def __init__(self):
        """initalize the node"""
        rospy.init_node("driveStop")
        self.pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size = 1)
        self.image_pub = rospy.Publisher(IMAGE_TOPIC, Image, queue_size = 2)
        rospy.Subscriber("scan", LaserScan, self.driveStop_car_callback)
        
        """initialize the box dimensions"""
        self.empty_box = ((0,0),(0,0))
        self.flag_boxR = self.empty_box
        self.flag_boxB = self.empty_box
        self.flag_boxY = self.empty_box
        self.certainty = 0
        
        self.flag_center = (0,0)
        self.flag_size = 0
        
        """driving stuff"""
        self.cmd = AckermannDriveStamped()
        self.cmd.drive.speed = 0
        self.cmd.drive.steering_angle = 0
        
        """get the camera data from the class Zed_converter in Zed"""
        self.camera_data = Zed_converter(False, save_image = False)
        self.imagePush = None
        self.imagePushB = None
        self.imagePushY = None
        
        self.bridge = CvBridge()
        self.min_value=0
        self.last_x = 0
        self.state = 0
        self.follow_color = "red"
        self.find_color = "yellow"
        self.timer = 0
        self.dir=0
        self.stopped = False
    
    #    def size_calc(self):
    #        """ calculate the x and y size of the box in pixels"""
    #        pix_width = self.flag_box[1][0] - self.flag_box[0][0]
    #        pix_height = self.flag_box[1][1] - self.flag_box[0][1]
    #        #print("point", self.flag_box[0][0], ":", self.flag_box[0][1])
    #        #print("point", self.flag_box[1][0], ":", self.flag_box[1][1])
    #        self.box_size = pix_width*pix_height
    
    def size_calc(self, box):
        """ calculate the x and y size of the box in pixels"""
        #rospy.loginfo("box_size: {}".format(self.box_size))
        width = box[1][0] - box[0][0]
        height = box[1][1] - box[0][1]
        #        self.box_size = width*height
        self.box_x = (box[0][0]+box[1][0])/2
        #rospy.loginfo("x: {} , y: {}".format(self.box_x, box[0][1]))
        if self.box_x == 0:
            self.box_x = self.last_x
        else:
            self.last_x = self.box_x
        self.box_left = box[0][0]
        
        offset = 0.05 + (width/2000) #adjust for left camera offset, adjust more when closer to cone
        
        self.new_angle = offset -(self.box_x-336)/500.0 #1000.0 336
    #print("driving angle: ", self.new_angle)
    #rospy.loginfo("a: {} w: {}".format(self.new_angle, width))
    
    def driveStop_car_callback(self,data):
        """laser scan callback function"""
        #checks if the image is valid first
        while self.camera_data.cv_image is None:
            time.sleep(0.5)
            print("sleeping")
        
        #applies the current filter to the image and stores the image in imagePush
        self.flag_boxR, self.imagePush = cd_color_segmentation(self.camera_data.cv_image, color="r")
        self.flag_boxB, self.imagePushB = cd_color_segmentation(self.camera_data.cv_image, color="b")
        self.flag_boxY, self.imagePushY = cd_color_segmentation(self.camera_data.cv_image, color="y")
        
        #finds the size of the box
        #        self.size_calc(self.flag_boxR)
        #        self.size_calc(self.flag_boxB)
        #        self.size_calc(self.flag_boxY)
        
        #outputs the image to the IMAGE_TOPIC
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.imagePush, "bgr8"))
        except CvBridgeError as e:
            print("Error bridging Zed image", e)
        
        print(self.flag_boxR, self.flag_boxB, self.flag_boxY)
        self.check_stop()
        if self.stopped:
            self.cmd.drive.steering_angle = 1
            self.cmd.drive.speed = 0
            return
        if self.timer <=0:
            if self.flag_boxY != self.empty_box and self.follow_color != "y":
                self.timer = 30
                self.follow_color = "y"
            elif self.flag_boxB != self.empty_box and self.follow_color != "b" and self.flag_boxY == self.empty_box:
                self.timer = 60
                self.follow_color = "b"
            elif self.flag_boxR != self.empty_box and self.follow_color != "r" and self.flag_boxY == self.empty_box and self.flag_boxB == self.empty_box:
                self.timer = 30
                self.follow_color = "r"
        else:
            self.timer -= 1
        
        rospy.loginfo("following: {}".format(self.follow_color))
        
        if self.follow_color == "y":
            self.size_calc(self.flag_boxY)
        elif self.follow_color == "b":
            self.size_calc(self.flag_boxB)
        elif self.follow_color == "r":
            self.size_calc(self.flag_boxR)
        else:
            self.size_calc(self.empty_box)

        self.drive_on_path()
    #        if self.timer<=0:
    #            if self.state == 0:
    #                self.follow_color = "red"
    #                self.find_color = "yellow"
    #                self.dir = -0.2
    #                self.size_calc(self.flag_boxR)
    #                self.switch_color(self.flag_boxY)
    #            elif self.state == 1:
    #                self.follow_color = "yellow"
    #                self.find_color = "red"
    #                self.dir = 0.1
    #                self.size_calc(self.flag_boxY)
    #                self.switch_color(self.flag_boxR)
    #            elif self.state == 2:
    #                self.follow_color = "red"
    #                self.find_color = "yellow"
    #                self.dir = -0.05
    #                self.size_calc(self.flag_boxR)
    #                self.switch_color(self.flag_boxY)
    #            elif self.state == 3:
    #                self.follow_color = "yellow"
    #                self.find_color = "red"
    #                self.dir = -0.1
    #                self.size_calc(self.flag_boxY)
    #                self.switch_color(self.flag_boxR)
    #            elif self.state == 4:
    #                self.follow_color = "red"
    #                self.find_color = "blue"
    #                self.dir = -1
    #                self.size_calc(self.flag_boxR)
    #                self.switch_color(self.flag_boxB)
    #            elif self.state == 5:
    #                self.follow_color = "blue"
    #                self.find_color = "blue"
    #                self.size_calc(self.flag_boxB)
    #                self.switch_color(self.flag_boxB)
    #            elif self.state == 6:
    #                self.cmd.drive.steering_angle=0
    #                self.cmd.drive.speed = 0
    #                pass
    #            else:
    #                self.cmd.drive.steering_angle = self.dir
    #                self.timer -= 1
    #        rospy.loginfo("state: {} follow: {} find: {}".format(self.state, self.follow_color, self.find_color))
    #        self.drive_on_path()
    #        if AUTONOMOUS_MODE:
    #            self.drive()
    #        else:
    #            pass
    
    def drive_on_path(self):
        """write driving commands here! You don't need to publish the drive command,
            that is being taken care of in the main() function"""
        
        #        if self.timer <=0:
        #            self.cmd.drive.steering_angle=self.new_angle
        self.cmd.drive.steering_angle=self.new_angle
        self.cmd.drive.speed = 5

#    def switch_color(self, targetBox):
#        if self.state == 5:
#            #            if self.flag_boxR == self.empty_box and self.flag_boxB == self.empty_box and self.flag_boxY == self.empty_box:
#            #                if self.certainty>30:
#            #                    self.state+=1
#            #                else:
#            #                    self.certainty += 1
#            pass
#
#        if targetBox != self.empty_box:
#            if self.certainty>3:
#                self.certainty = 0
#                self.state += 1
#                self.timer = 40
#                if self.state == 2:
#                    self.timer = 60
#                elif self.state == 3:
#                    self.timer = 80
#            else:
#                self.certainty += 1
#        else:
#            self.certainty = 0
#
#        def drive(self):
#            pass
    def check_stop(self):
        if self.flag_boxR == self.empty_box and self.flag_boxB == self.empty_box and self.flag_boxY == self.empty_box:
            if self.certainty>40:
                self.stopped = True
            else:
                self.certainty += 1
        else:
            self.certainty = 0
        

def main():
    global AUTONOMOUS_MODE
    try:
        ic = driveStop()
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if AUTONOMOUS_MODE:
                ic.pub.publish(ic.cmd)

    except rospy.ROSInterruptException:
        exit()


if __name__ == "__main__":
    main()


