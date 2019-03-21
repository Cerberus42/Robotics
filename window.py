# -*- coding: utf-8 -*-
"""
Created on Fri Feb 15 2019
@author: student
"""
import rospy
import cv2
import numpy
import actionlib
import time
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#from actionlib_msgs.msg import GoalStatus.msg
class find_poles:

    def __init__(self):
        
        #setup
        cv2.startWindowThread()
        self.thresh_img = numpy.zeros(0)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image, self.processImage)
        self.pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size = 2)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.twist = Twist()
        self.Red = True
        self.Green = True
        self.Blue = True
        self.Yellow = True

        self.value = self.GetFind()
        self.threshold = 20000 
        self.bigBool = True

      #  self.twist = Twist

       # self.rotate()


        self.lasersub = rospy.Subscriber("/scan", LaserScan, self.procScan)
        self.laserdata = LaserScan()
        self.scanthresh = 5 #amount of ranges
        self.frontie = 100 #set to 100 to avoid false entries on startup
        self.leftie = 100
        self.rightie = 100

        self.x = [0.0, 0.0, 1.0, -4.0, -4.0]
        self.y = [0.0, 4.0, -4.5, -0.3, 3.0]
        self.curnode = 0
       #process the image from the robot



    def processImage(self, data):
        cv2.namedWindow("Thresh Window", 1)
        cv2.namedWindow("Grayscale", 1)
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
        
        
        height = len(cv_image[:,1]  )
        width = len(cv_image[1,:]  )
        
         #define colours in the array
        yellowLow = numpy.array((0,101,101))
        yellowHigh = numpy.array((4,102,102))
        
        blueLow = numpy.array((101,0,0))
        blueHigh = numpy.array((255,0,0))
        
        
        redLow = numpy.array((0,0,101))        
        redHigh = numpy.array((4,4,255))
        
        greenLow = numpy.array((0,0,0))
        greenHigh = numpy.array((6,255,6))
        
        blue_thresh = numpy.zeros([height, width])
        yellow_thresh = numpy.zeros([height, width])
        red_thresh = numpy.zeros([height, width])        
        green_thresh = numpy.zeros([height, width])
        if self.Blue == True:        
            #thresh blue - good
            blue_thresh = cv2.inRange(cv_image,blueLow,blueHigh)
            
        if self.Yellow == True:        
            #thresh yellow - good              
            yellow_thresh = cv2.inRange(cv_image,yellowLow,yellowHigh)        
        if self.Red == True:                
            #thresh red - good                   
            red_thresh = cv2.inRange(cv_image,redLow,redHigh)                                    
        if self.Green == True:
            #green thresh
            green_thresh = cv2.inRange(cv_image, greenLow,greenHigh)
                                 
        
        self.thresh_img = blue_thresh + yellow_thresh + red_thresh + green_thresh
        self
        ret,self.thresh_img = cv2.threshold(self.thresh_img, 0,255, cv2.THRESH_BINARY)
        #create grayscale and display images side by side
        grayScale = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)

        cv2.imshow("Thresh Window", self.thresh_img)
        cv2.imshow("Grayscale", grayScale)
        cv2.waitKey(1)

        # move boi
        if self.bigBool:
            self.movement()
            self.bigBool = False

    def GetImage(self):
        return self.thresh_img
    
    def SetFind(self, color, value):
        if color == "Blue":
            self.Blue = value
        if color == "Green":
            self.Green = value
        if color == "Red":
            self.Red = value
        if color == "Yellow":
            self.Yellow = value

        #sleep(0.5) #wait for changes to take effect


    def GetFind(self):
        values = []
        values.append(self.Blue)
        values.append(self.Green)
        values.append(self.Red)
        values.append(self.Yellow)

        return values

    def Coloursearch(self):
        count = (self.GetImage() > 253).sum()
       
        if count > 4000:
            if self.value[0] == True:
                self.twist.linear.x = 1
                self.SetFind("Blue", False)
                count = (self.GetImage() > 253).sum()
                if count < self.threshold:
                    print "Blue"
                    #break
                else:
                    self.SetFind("Blue", True)
            if self.value[1] == True: #check green
                self.SetFind("Green", False)
                count = (self.GetImage() > 253).sum()
                if count < self.threshold:
                    print "Found Green"
                        #self.__values[1] = False
                    #break
                else:
                    self.SetFind("Green", True)


            if self.value[2] == True: #check red
                self.SetFind("Red", False)
                count = (self.GetImage() > 253).sum()
                if count < self.threshold:
                    print "Found Red"
                    #self.__values[2] = False
                    #break
                else:
                    self.SetFind("Red", True)

            if self.value[3] == True: #check yellow
                self.SetFind("Yellow", False)
                count = (self.GetImage() > 253).sum()
                if count < self.threshold:
                    print "Found Yello"
                    #self.__values[3] = False
                    #break
                else:
                    self.SetFind("Yello", True)
        self.movement()
    def procScan(self, data):
        self.laserData = data

        #calculte scan data
        dataLen = len(data.ranges)
        self.frontie = sum(data.ranges[dataLen/2:dataLen/2 + self.scanthresh])/self.scanthresh
        self.leftie = data.ranges[dataLen / 8]
        self.rightie = data.ranges[(dataLen / 2) + (dataLen / 8)]

        #print data.ranges[0]
        #print "Front: " + str(self.frontie)
        #print "Left: "  + str(self.leftie)
        #print "Right: "  + str(self.rightie)
        #print "----------------------"
        
    def search(self):
        if numpy.isnan(self.distance):
            self.rotate()
        else:
            if self.distance < 1:
                self.twist.linear.x = 0 # stops forward motion
                self.twist.angular.z = 0#stops previous rotation
                self.rotate()
            else:
                self.twist.linear.x = 1# move forward
                self.twist.angular.z = 0 # stops rotation so the robot can move in a straight line

 #   def rotate(self):
#            self.twist.linear.x = 0

         #   if self.distance < 1:
        #        if self.right == False:
       #             self.twist.angular.z = random.uniform(0,4)
      #          else:
     #               self.twist.angular.z = -(random.uniform(0,4))
    #        else:
   #             self.twist.angular.z = 0
  #          if numpy.isnan(image_converter.distance):
 #               self.twist.angular.z = 0.5
#            self.cmd_vel_pub.publish(self.twist)



    def movement(self):
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = self.y[self.curnode]
        goal.target_pose.pose.position.y = self.x[self.curnode]
        
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)  
        seconds = (0)

        print "Moving"
        for i in range(seconds):
            print(str(seconds - i) + "remaining")
            time.sleep(1)
        self.Coloursearch()

    def moveToObject(self,objectArea):
        if objectArea < 500:
            self.twist.linear.x = 0.7
            return False
        else:
            self.twist.linear.x=0
            time.sleep(3)
        return True        

class countdown:
        
        def __init__(self):
            for i in range(seconds):
                print(str(seconds - i) + "remaining")
                time.sleep(1)
        pass
find_poles()
rospy.init_node('find_poles','countdown', anonymous=True)
rospy.spin()
