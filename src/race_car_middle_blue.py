#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
def get_contours_centers(contours):
    centers = []
    for contour in contours:
        center = get_center(contour)
        if(center != None):
            centers.append(center)
    return centers
def get_center(contour):
    M = cv2.moments(contour)
    if(M['m00'] != 0):
        return (int(M['m10']/M['m00']), int(M['m01']/M['m00']))
    else:
        return None

def partition(points, left_x, left_y, right_x, right_y):
    left_points = []
    right_points = []
    for point in points:
        if(point[0] < left_x and point[1] > left_y):
            left_points.append(point)
        elif(point[0] > right_x and point[1] > right_y):
            right_points.append(point)
    return left_points, right_points
    
def turn_point(left_points, right_points, default_point, default_left, default_right):
    if(len(left_points) != 0 and len(right_points) != 0):
        return default_point
    elif(len(left_points) != 0):
        return default_right
    elif(len(right_points) != 0):
        return default_left
    else:
        return default_point
class Follower:
    def __init__(self, camera_topic='/camera/rgb/image_raw', cmd_topic='cmd_vel'):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        self.twist = Twist()
        self.logcount = 0
        self.lostcount = 0
    def image_callback(self, msg):

         # get image from camera
        image = self.bridge.imgmsg_to_cv2(msg)
        height, width, _ = image.shape
        middle_y = height / 2
        middle_x = width / 2

        # filter out everything that's not yellow
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([ 0, 0, 200])
        upper_white = numpy.array([ 0, 70, 255])
        mask = cv2.inRange(hsv,  lower_white, upper_white)
        #masked = cv2.bitwise_and(image, image, mask=mask)

        lower_blue = numpy.array([120, 150, 150])
        upper_blue = numpy.array([150, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # clear all but a 20 pixel band near the top of the image
        h, w, d = image.shape
        search_top = 2 * h /4
        search_bot = search_top + 4 * h /5
        search_left = w / 5
        search_right = 4 * w / 5
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        mask[search_top: search_bot, 0: search_left] = 0
        mask[search_top: search_bot, search_right:w] = 0

        blue_mask[0:search_top, 0:w] = 0
        blue_mask[search_bot:h, 0:w] = 0


        cv2.imshow("white_band", mask)
        cv2.imshow('blue_band', blue_mask)


        (_, contours, _) = cv2.findContours(blue_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
        centers = get_contours_centers(contours)
        left_points, right_points = partition(centers, middle_x - 50, middle_y + 100, middle_x + 50, middle_y + 100)
        print(left_points)
        print(right_points)
        blue_turn_x, blue_turn_y = turn_point(left_points,right_points, (middle_x, middle_y), (0, middle_y), (2 * middle_x, middle_y))
        cv2.circle(image, (blue_turn_x, blue_turn_y), 30, (120,120,120), -1)
        cv2.circle(image, (middle_x, middle_y), 20, (0,0,255), -1)
       
        cv2.imshow("image", image)
        self.move(blue_turn_x, blue_turn_y, middle_x, middle_y)
        cv2.waitKey(3)
    

    def move(self, cx, cy, middle_x, middle_y):
        
        diff_y = cy - middle_y
        diff_x = cx - middle_x
        if(diff_x > 0):
            self.twist.angular.z = float(abs(diff_x)) / middle_x * -0.4
            print("< 0", self.twist)
        else:
            self.twist.angular.z = float(abs(diff_x)) / middle_x * 0.4
            print("> 0", self.twist)
        print((middle_x - abs(diff_x)) / middle_x)
        self.twist.linear.x = 2

        self.cmd_vel_pub.publish(self.twist)
        
rospy.init_node('follower')
follower = Follower()
rospy.spin()