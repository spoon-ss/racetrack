#!/usr/bin/env python
​
import rospy, cv2, cv_bridge, numpy
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from math import pi
​
# return the center of each contour
def get_contours_centers(contours):
    centers = []
    for contour in contours:
        center = get_center(contour)
        if(center != None):
            centers.append(center)
    return centers
​
# return the center of contour
def get_center(contour):
    M = cv2.moments(contour)
    if(M['m00'] != 0):
        return (int(M['m10']/M['m00']), int(M['m01']/M['m00']))
    else:
        return None
​
# partition left and right
def partition(points, left_x, left_y, right_x, right_y):
    left_points = []
    right_points = []
    for point in points:
        if(point[0] < left_x and point[1] > left_y):
            left_points.append(point)
        elif(point[0] > right_x and point[1] > right_y):
            right_points.append(point)
    return left_points, right_points
    
# return the point in image that we desire the robot to go
def turn_point(left_points, right_points, default_point, default_left, default_right):
    if(len(left_points) != 0 and len(right_points) != 0):
        return default_point
    elif(len(left_points) != 0):
        return default_right
    elif(len(right_points) != 0):
        return default_left
    else:
        return default_point
​
# avoid left or right, return the desired angular speed
def left_or_right(left, right):
    angular_add = 0.3
    if min(left) >= min(right):
        # need to turn left, control speed limit at pi / 2
        return angular_add
    else:
        # need to turn right, control speed limit at - pi / 2
        return - angular_add
​
class Follower:
​
    def __init__(self, camera_topic='/camera/rgb/image_raw', cmd_topic='cmd_vel'):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.image_callback)
        self.laser_sub = rospy.Subscriber('scan', LaserScan, self.laser_callback)
        self.cmd_vel_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        self.twist = Twist()
        self.logcount = 0
        self.lostcount = 0
        self.left_ranges = [4]
        self.right_ranges = [4]
        self.p_constant = 0.4
        self.timer = None
​
    def laser_callback(self, msg):
        ranges = np.array(msg.ranges)
        # I calculated to consider the 54 degrees infront of the robot
        # first clean laser data
        for i in range(360):
            if ranges[i] == float('inf') or ranges[i] < msg.range_min:
                ranges[i] = 4
        self.left_ranges = ranges[0:10]
        self.right_ranges = ranges[350:360]
​
    def image_callback(self, msg):
​
        # get image from camera
        image = self.bridge.imgmsg_to_cv2(msg)
        height, width, _ = image.shape
        middle_y = height / 2
        middle_x = width / 2
​
        # filter out everything that's not white and blue
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([ 0, 0, 200])
        upper_white = numpy.array([ 0, 70, 255])
        mask = cv2.inRange(hsv,  lower_white, upper_white)
​
        lower_blue = numpy.array([120, 150, 150])
        upper_blue = numpy.array([150, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
​
        # clear all but a square near the top of the image
        h, w, d = image.shape
        search_top = 2 * h /4
        search_bot = search_top + 4 * h /5
        search_left = w / 5
        search_right = 4 * w / 5
​
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        mask[search_top: search_bot, 0: search_left] = 0
        mask[search_top: search_bot, search_right:w] = 0
​
        blue_mask[0:search_top, 0:w] = 0
        blue_mask[search_bot:h, 0:w] = 0
​
        cv2.imshow("white_band", mask)
        cv2.imshow('blue_band', blue_mask)
​
        # find contours on the blue mask and get center points of all contours
        (_, contours, _) = cv2.findContours(blue_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
        centers = get_contours_centers(contours)
​
        left_points, right_points = partition(centers, middle_x - 50, middle_y + 100, middle_x + 50, middle_y + 100)
        print(left_points)
        print(right_points)
​
        # get turn points
        blue_turn_x, blue_turn_y = turn_point(left_points,right_points, (middle_x, middle_y), (0, middle_y), (2 * middle_x, middle_y))
        cv2.circle(image, (blue_turn_x, blue_turn_y), 30, (120,120,120), -1)
        cv2.circle(image, (middle_x, middle_y), 20, (0,0,255), -1)
        
        # move according to turn points
        cv2.imshow("image", image)
        
        self.move(blue_turn_x, blue_turn_y, middle_x, middle_y)
        cv2.waitKey(3)
    
    # move by pid control
    def move(self, cx, cy, middle_x, middle_y):
        if self.timer != None and (rospy.Time.now() - self.timer).secs >= 4:
            self.p_constant = 0.4
        diff_y = cy - middle_y
        diff_x = cx - middle_x
        if(diff_x > 0):
            self.twist.angular.z = float(abs(diff_x)) / middle_x * -self.p_constant
            self.twist.linear.x = 2
            print("< 0", self.twist)
        else:
            self.twist.angular.z = float(abs(diff_x)) / middle_x * self.p_constant
            self.twist.linear.x = 2
            print("> 0", self.twist)
        if min(self.left_ranges) <= 2 or min(self.right_ranges) <= 2:
            if self.timer == None:
                self.timer = rospy.Time.now()
                self.p_constant = 0.8
            print("avoid obstacle")
            self.twist.angular.z = self.twist.angular.z + left_or_right(self.left_ranges, self.right_ranges)
            self.twist.linear.x = 0.5
        print((middle_x - abs(diff_x)) / middle_x)
​
        self.cmd_vel_pub.publish(self.twist)
        
rospy.init_node('follower')
follower = Follower()
rospy.spin()