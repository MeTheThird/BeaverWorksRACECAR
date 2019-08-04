#!/usr/bin/env python2
from math import *
import numpy as np
import rospy
import cv2
import time

from sensor_msgs.msg import LaserScan, Image
from ackermann_msgs.msg import AckermannDriveStamped
from color_segmentation import cd_color_segmentation
from newZed import Zed_converter
from ImageMatch import matchLeftRightTemplate
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool
from ar_track_alvar_msgs.msg import AlvarMarkers


'''
TODO:
delay for highway state switch -- DONE

left wall follower -- DONE

line follower at graveyard -- try both mine and Nalin's
    switch to pot at AR tag #6

slow down right wall follow at tag #8

right wall at roundabout -- stay through TA section

switch to pot at the end of the section

left wall follow at Lockheed garden

mb Kelley construction left wall <<shortcut>> OR straight up pot

decrease speed??

implement 'idle' state reset with joystick (low priori)

CONSIDER --->>>>>> doing a right wall follower all the way to the end after the end of the second
time we go on the highway
'''

lidarPoints = 1081

SPEED = 2

LOW_DESIRED_DISTANCE = 0.5
HIGH_DESIRED_DISTANCE = 1.0

AR_DIST_THRESHOLD = 1.5
HIGHWAY_TIME_THRESHOLD = 6.0

#potStuff
kmax = 2
carCharge = 1.0
jetCharge = 500
wallCharge = 1.1

# POT_TURN_STRENGTH = 50

steeringFactor = .00085 #0.009
speedFactor = 1.2 #1.3

#63,83 220,255 0, 80
# GREEN = np.array([[63, 120, 0], [83, 180, 255]])
#GREEN = np.array([[37, 240, 30], [48, 255, 176]])
#GREEN = np.array([[63, 220, 0], [83, 255, 80]])
GREEN_LIGHT = np.array([[63, 220, 0], [83, 255, 150]])
RED_BRICKS = np.array([[153, 102, 102], [255, 204, 204]])
RED_LINE = np.array([[0, 30, 130], [10, 255, 255]])
THRESHOLD_AREA = 10

class Racecar:
    SCAN_TOPIC = '/scan'
    DRIVE_TOPIC = '/drive'
    POSE_TOPIC = '/pf/viz/inferred_pose'
    SAFETY_TOPIC = '/pot_safe'
    AR_TOPIC = "/ar_pose_marker"

    state = 'pot'
    greenFrames = []
    tagRead = False
    timeStarted = 0
    IMAGE_TOPIC = '/zed/zed_node/rectImage'

    def __init__(self):
        self.data = None
        self.SIDE = 0
        # x (steering)
        self.iHat = []
        self.left_right_list = []
        # y (speed)
        self.jHat = []
        self.starttime = time.time()
        # [speed, angle]

        self.greenFrames = []

        self.cmd = AckermannDriveStamped()
        self.cmd.drive.speed = 0

        self.pos = 0
        self.orientation = None

        self.img_pub = rospy.Publisher(self.IMAGE_TOPIC, Image, queue_size = 1)
        self.sub_scan = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, callback=self.scan_callback)
        self.pub_drive = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.POSE_TOPIC, PoseStamped, self.poseCallback)
        self.safety_pub = rospy.Publisher(self.SAFETY_TOPIC, Bool, queue_size = 1)
        self.ar_sub = rospy.Subscriber(self.AR_TOPIC, AlvarMarkers, self.arCallback)

        self.camera_data = Zed_converter(False, save_image = False)
        self.bridge = CvBridge()
        self.img = None
        self.image_pub = rospy.Publisher("/zed/zed_node/output", Image, queue_size = 1)



    def scan_callback(self, data):
        print(self.state)
        '''
        Current states:
        idle
        potential (might require to test constant values)
        sign (in progress?)
        bricks (work to do)
        carWash (work to do)
        bridge (working but could be improved)
        highway (might work not tested - requires to test constant values)
        boulet (might work not tested - requires to test constant values)
        graveyard (need localization to work)
        '''


        '''try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.img, "bgr8"))
        except CvBridgeError:
            print("failed to publish image")'''

        # print(self.state)
        self.data = data
        self.ranges = np.array(self.data.ranges)

        if self.state == 'idle':
            self.safety_pub.publish(Bool(True))
            going = self.greenLightGo()
            print('greenFrames = ' + str(self.greenFrames))
            if going:
                self.state = 'pot'
            elif len(self.greenFrames) > 5:
                self.greenFrames.pop(0)

        elif self.state == 'pot':
            self.safety_pub.publish(Bool(True))
            self.cmd.drive.speed = SPEED
            self.cmd.drive.steering_angle = steeringFactor * self.pot()[0]

            if self.timeStarted > 0 and time.time() - self.timeStarted > HIGHWAY_TIME_THRESHOLD:
                self.state = 'highway'
                self.timeStarted = -1

        elif self.state == 'sign':
            self.safety_pub.publish(Bool(True))
            turnDir = self.oneWayTurn()
            """ self.cmd.drive.speed = SPEED
            if turnDir == 'left' and not self.turnStarted:
                self.cmd.drive.steering_angle = 1
                # self.state = 'leftWallFollower'
                self.turnStarted = True
                self.timeStarted = time.time()
            elif turnDir == 'right' and not self.turnStarted:
                self.cmd.drive.steering_angle = -1
                # self.state = 'rightWallFollower'
                self.turnStarted = True
                self.timeStarted = time.time()
            elif time.time() - self.timeStarted > 1.0:
                self.state = 'pot' """

        elif self.state == 'bricks':
            self.safety_pub.publish(Bool(False))
            self.cmd.drive.speed = SPEED
            self.cmd.drive.steering_angle = self.bricksTurnAngle()

        # elif self.state == "carWash":
        #     self.safety_pub.publish(Bool(False))
        #     self.cmd.drive.speed = SPEED
        #     self.cmd.drive.steering_angle = self.carWashTurnAngle()

        elif self.state == "bridge":
            self.safety_pub.publish(Bool(False))
            # potential for now but further change is possible
            self.cmd.drive.speed = SPEED
            self.cmd.drive.steering_angle = steeringFactor * self.pot()[0]

        elif self.state == "highway":
            self.safety_pub.publish(Bool(True))
            # if time.time() - self.timeStarted > HIGHWAY_TIME_THRESHOLD:
            self.cmd.drive.speed = 1000
            self.cmd.drive.steering_angle = self.rightWall(HIGH_DESIRED_DISTANCE)

        # elif self.state == "boulet":
        #     self.safety_pub.publish(Bool(True))
        #     self.cmd.drive.speed = SPEED
        #     self.cmd.drive.steering_angle = self.leftWall(LOW_DESIRED_DISTANCE)

        elif self.state == "graveyard":
            self.safety_pub.publish(Bool(True))
            self.state = 'pot'

        elif self.state == "construction":
            self.safety_pub.publish(Bool(True))
            self.cmd.drive.speed = SPEED
            self.cmd.drive.steering_angle = self.leftWall(LOW_DESIRED_DISTANCE)

        elif self.state == "leftWall":
            self.safety_pub.publish(Bool(True))
            self.cmd.drive.speed = SPEED
            self.cmd.drive.steering_angle = self.leftWall(LOW_DESIRED_DISTANCE)

        elif self.state == "rightWall":
            self.safety_pub.publish(Bool(True))
            self.cmd.drive.speed = SPEED
            self.cmd.drive.steering_angle = self.rightWall(LOW_DESIRED_DISTANCE)


        # THIS IS AN EXAMPLE STATE FOR USING pot2point IN OUR CODE -- we will obvs need to collect
        # points and stuff later

        # elif self.state == 'pot2pointLOCATION':
        #     self.safety_pub.publish(Bool(True))
        #     self.cmd.drive.speed = SPEED
        #     potVect = self.pot()
        #     potVect[0] = (potVect[0]*(1-POINT_TURN_STRENGTH) + self.point2point(INSERT_LOCATION_HERE)*POINT_TURN_STRENGTH)
        #     self.cmd.drive.steering_angle = steeringFactor * potVect[0]


        # EXMAPLE OF NORMAL POINT TO POINT

        elif self.state == 'deliver2point':
            self.safety_pub.publish(Bool(False))
            self.cmd.drive.speed = SPEED

            # USE ONE OF THE BOTTOM TWO
            # self.cmd.drive.steering_angle = self.point2point(INSERT_LOCATION_HERE)
            point = (0.55, -1.32)
            # print('current pose: ' + str(self.pos))
            print('raw angle: ' + str(self.point2point(point)))
            # print('constant modified angle: ' + str(steeringFactor * self.point2point(point)))
            print('distBetween: ' + str(self.distBetween(point)))
            self.cmd.drive.steering_angle = self.point2point(point)

        self.pub_drive.publish(self.cmd)


    def poseCallback(self, msg):
        self.pos = msg.pose.position
        self.orientation = self.quatToAng3D(msg.pose.orientation)
        # print(self.pos)

        # THIS IS AN EXMAPLE OF USING pot2point and switching
        # if self.distBetween(INSERT_LOCATION_HERE) < DIST_THRESHOLD:
        #     self.state = 'pot2pointLOCATION'
        # elif self.distBetween(POT_SWITCH_LOCATION) < DIST_THRESHOLD:
        #     self.state = 'pot'

    def arCallback(self, tags):
        if len(tags.markers) == 0:
            return

        tag = tags.markers[0]
        distTo = self.distToARTag(tag)

        if tag.id == 1 and distTo < AR_DIST_THRESHOLD:
            self.state = 'pot'
            if self.timeStarted < 0:
                self.tagRead = True
                self.timeStarted = time.time()

        elif tag.id == 3 and distTo < AR_DIST_THRESHOLD:
             self.state = 'leftWall'

        elif tag.id == 4 and distTo < AR_DIST_THRESHOLD:
            self.state = 'pot'

        elif tag.id == 5 and distTo < AR_DIST_THRESHOLD:
            self.state = 'graveyard'

        elif tag.id == 6 and distTo < AR_DIST_THRESHOLD:
            self.state = 'pot'

        elif tag.id == 7 and distTo < AR_DIST_THRESHOLD:
            self.state = 'pot'
            if self.timeStarted < 0:
                self.tagRead = True
                self.timeStarted = time.time()

        elif tag.id == 8 and distTo < AR_DIST_THRESHOLD:
            self.state = 'rightWall'

        elif tag.id == 9 and distTo < AR_DIST_THRESHOLD:
            self.state = 'bridge'

        elif tag.id == 10 and distTo < AR_DIST_THRESHOLD:
            self.state = 'rightWall'

        # consider changing ALL of the ones below to the 'rightWall' state
        elif tag.id == 11 and distTo < AR_DIST_THRESHOLD:
            self.state = 'pot'

        elif tag.id == 12 and distTo < AR_DIST_THRESHOLD:
            self.state = 'pot'

        elif tag.id == 14 and distTo < AR_DIST_THRESHOLD:
            self.state = 'construction'

        elif tag.id == 16 and distTo < AR_DIST_THRESHOLD:
            self.state = 'rightWall'

        elif tag.id == 17 and distTo < AR_DIST_THRESHOLD:
            self.state = 'pot'


    def pot(self):
        self.iHat = np.array([0]*1081)
        self.jHat = np.array([0]*1081)
        for i, item in enumerate(self.ranges):
            angle = float(i)/lidarPoints * (3*pi/2) - (pi/4)
            k = kmax - exp(abs((pi/2)-angle))*0.2
            # k = kmax
            maga = -k * (carCharge * wallCharge)/(item)
            self.iHat[i] = -maga * cos(angle)
            self.jHat[i] = maga * sin(angle)

        return [np.sum(self.iHat), np.sum(self.jHat) + (k * jetCharge)]


    def point2point(self, destination):
        ANGLE_CONSTANT = 0.1

        deltaX = destination[0] - self.pos.x
        deltaY = destination[1] - self.pos.y

        goalVectAngle = fabs(atan2(deltaY, deltaX))
        yaw = self.orientation[2] / 2.0

        turn_angle = 0
        if deltaX > 0 and deltaY > 0:
            print('deltaX > 0 and deltaY > 0')
            if yaw > 0:
                print('yaw > 0')
                if yaw > goalVectAngle:
                    print('yaw > goalVectAngle')
                    turn_angle = -1 * fabs(yaw - goalVectAngle)
                else:
                    print('yaw <= goalVectAngle')
                    turn_angle = fabs(yaw - goalVectAngle)
            else:
                print('yaw < 0')
                # if fabs(yaw) + goalVectAngle < 2*np.pi - (fabs(yaw) + goalVectAngle):
                turn_angle = fabs(yaw) + goalVectAngle
                # else:
                    # turn_angle = -1*(2*np.pi - (fabs(yaw) + goalVectAngle))

        elif deltaX < 0 and deltaY > 0:
            print('deltaX < 0 and deltaY > 0')
            if yaw > 0:
                if np.pi - yaw - goalVectAngle > 0:
                    turn_angle = np.pi - yaw - goalVectAngle
                else:
                    turn_angle = -1 * (yaw + goalVectAngle - np.pi)
            else:
                turn_angle = np.pi - fabs(yaw) + goalVectAngle

        elif deltaX > 0 and deltaY < 0:
            print('deltaX > 0 and deltaY < 0')
            if yaw > 0:
                print('yaw > 0')
                turn_angle = -1 * (yaw + goalVectAngle)
            else:
                print('yaw < 0')
                if fabs(yaw) > goalVectAngle:
                    print('fabs(yaw) > goalVectAngle')
                    turn_angle = fabs(yaw - goalVectAngle)
                else:
                    print('fabs(yaw) <= goalVectAngle')
                    turn_angle = -1 * fabs(yaw - goalVectAngle)
                # ik this overrides the prev lines but i wanna test it out :)
                turn_angle = fabs(yaw - goalVectAngle)

        else:
            print('deltaX < 0 and deltaY < 0')
            if yaw > 0:
                turn_angle = np.pi - yaw + goalVectAngle
            else:
                if np.pi - fabs(yaw) - goalVectAngle > 0:
                    turn_angle = -1 * (np.pi - fabs(yaw) - goalVectAngle)
                else:
                    turn_angle = fabs(yaw) - np.pi + goalVectAngle

        return turn_angle


    def greenLightGo(self):
        while self.camera_data.cv_image is None:
            print('sleeping')
            self.cmd.drive.speed = 0

        flag_box, self.img, _ = cd_color_segmentation(self.camera_data.cv_image, GREEN_LIGHT, (0, self.camera_data.cv_image.shape[1]), (0, int(1.0/3 * self.camera_data.cv_image.shape[0])))

        if (flag_box[1][0] - flag_box[0][0]) * (flag_box[1][1] - flag_box[0][1]) > THRESHOLD_AREA:
            print('GREEN YEET')
        else:
            print('no green sad boiz')

        if (flag_box[1][0] - flag_box[0][0]) * (flag_box[1][1] - flag_box[0][1]) > THRESHOLD_AREA:
            self.greenFrames.append(True)
        else:
            self.cmd.drive.speed = 0
            self.greenFrames.append(False)

        if len(self.greenFrames) >= 5:
            green = True
            for bool in self.greenFrames:
                if not bool:
                    green = False
            return green

        return False


    def oneWayTurn(self):
        while self.camera_data.cv_image is None:
            print('sleeping')
            self.cmd.drive.speed = 0

        if len(self.left_right_list) == 10:
            print(self.left_right_list)
            print("got ten images")
            self.turnStarted = True
            direction = max(set(self.left_right_list), key= self.left_right_list.count)
            if direction == "left" and not self.turnStarted:
                print("left\n\n\n\n\n----------------------------------------------------")
                return 'left'
            elif direction == "right" and not self.turnStarted:
                print("right\n\n\n\n\n----------------------------------------------------")
                return 'right'
        else:
            leftRightVal = matchLeftRightTemplate(self.camera_data.cv_image)
            self.img = leftRightVal[2]
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.img, "bgr8"))
            print("getting 10 images")
            if leftRightVal[0] > leftRightVal[1]:
                #self.left_right_list.append("left")
                print("left")
            else:
                #self.left_right_list.append("right")
                print("right")


    def bricksTurnAngle(self):
        while self.camera_data.cv_image is None:
            time.sleep(0.5)
            print("sleeping")

        ANGLE_CONSTANT = 0.1
        # applies the current filter to the image and stores the image in imagePush
        self.flag_box, self.imagePush, contours = cd_color_segmentation(self.camera_data.cv_image, RED_BRICKS, (0, self.camera_data.cv_image.shape[1]), (int(1.0/2 * self.camera_data.cv_image.shape[0]), self.camera_data.cv_image.shape[0]))

        CENTER = self.imagePush.shape[1] / 2

        xSum = 0
        for contour in contours[int(4.0/5 * len(contours)):]:
            x1, y1, x2, y2 = cv2.boundingRect(contour)
            xSum += x1 + x1 + x2

        xAvg = xSum / int(2.0/5 * len(contours))

        error = xAvg - CENTER

        # it might have to be -1 * error * ANGLE_CONSTANT * np.pi/180
        # even though that makes no logical sense cuz that's what I had for the cone parking thing
        return error * ANGLE_CONSTANT * np.pi/180


    def rightWall(self, desired_distance):
        # if lidar data has not been received, do nothing
        if self.data == None:
            return 0

        tempAngle = 0
        KP = .14
        KD = .04
        length = len(self.ranges)

        #Prevents circles
        if np.min(self.ranges[230:310]) < self.ranges[length/2]:
            temp = KP*(desired_distance - np.min(self.ranges[230:310]))
        else:
            temp = -KP*(desired_distance - np.min(self.ranges[230:310]))

        # print(temp)
        tempAngle = temp + KD*(3.14/4/(pow(self.ranges[length/6], 2) + pow(self.ranges[length/3], 2) - 2*3.14/4*self.ranges[length/6]*self.ranges[length/3])*self.ranges[length/6]-3.14/4)
        # print(tempAngle)

        return tempAngle


    def leftWall(self, desired_distance):
        # if lidar data has not been received, do nothing
        if self.data == None:
            return 0

        tempAngle = 0
        KP = .14
        KD = .04
        length = len(self.ranges)

        #Prevents circles
        if np.min(self.ranges[770:850]) < self.ranges[length/2]:
            temp = -KP*(desired_distance - np.min(self.ranges[770:850]))
        else:
            temp = KP*(desired_distance - np.min(self.ranges[770:850]))

        tempAngle = temp - KD*(3.14/4/(pow(self.ranges[length*5/6], 2) + pow(self.ranges[length*2/3], 2) - 2*3.14/4*self.ranges[length*5/6]*self.ranges[length*2/3])*self.ranges[length*5/6]-3.14/4)

        return tempAngle


    def distBetween(self, point2):
        deltaX = point2[0] - self.pos.x
        deltaY = point2[1] - self.pos.y

        return sqrt(deltaX**2 + deltaY**2)


    def distToARTag(self, tag):
        return sqrt(tag.pose.pose.position.x**2 + tag.pose.pose.position.y**2)


    def quatToAng3D(self, quat):
        euler = euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
        return euler


    def stop(self):
        self.drive(0,0)


    def scan(self):
        return self.scan


rospy.init_node('controller')
rate = rospy.Rate(60)
rc = Racecar()

while not rospy.is_shutdown():
    # TODO implement controller logic here
    #print("spinning")
    rospy.spin()
    #print("done")
