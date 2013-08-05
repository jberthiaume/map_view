#!/usr/bin/env python

import wx
import os, time
import rospy                                                #@UnresolvedImport
import numpy as np
import threading as t
from datetime import datetime
from TimerThread import TimerThread
from std_msgs.msg import String                             #@UnresolvedImport
from std_msgs.msg import UInt32                             #@UnresolvedImport
from std_msgs.msg import Int32MultiArray                    #@UnresolvedImport
from nav_msgs.msg import OccupancyGrid                      #@UnresolvedImport
from nav_msgs.msg import GridCells                          #@UnresolvedImport
from geometry_msgs.msg import PoseWithCovarianceStamped     #@UnresolvedImport
from geometry_msgs.msg import Twist                         #@UnresolvedImport
import Image
import ImageOps

from move_base_msgs.msg import MoveBaseActionGoal           #@UnresolvedImport
from move_base_msgs.msg import MoveBaseActionResult         #@UnresolvedImport      

FILENAME = "map.png"
MAX_COUNT = 3

class ROSNode():
    
    def __init__(self, parent): 
        # pos : geometry_msgs/Point 
        #     float x
        #     float y
        #     float z       
        self.origin_pos = None
        self.pose_pos = None
        
        # orient : geometry_msgs/Quaternion: 
        #     float x
        #     float y
        #     float z
        #     float w 
        self.origin_orient = None        
        self.pose_orient = None 
        
        # obstacles : geometry_msgs/Point[]:
        #     float x
        #     float y
        #     float z
        self.obstacles_1 = None
        self.obstacles_2 = None
                 
        self.image_width = 1000
        self.resolution = None
        self.refresh = False
        self.image = None
        self.filename = FILENAME 
        self.count = 0
        
        self.parent = parent
        self.pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped)
        self.goal_pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal)              
    
    def Publish2DPoseEstimate(self, point, orient):
        pwc = PoseWithCovarianceStamped()
        pwc.pose.pose.position.x = point[0]
        pwc.pose.pose.position.y = point[1]
        pwc.pose.pose.orientation.z = orient[2]
        pwc.pose.pose.orientation.w = orient[3]
        self.pose_pub.publish(pwc)   
        
    def Publish2DNavGoal(self, point, orient):
        ag = MoveBaseActionGoal()
        ag.goal.target_pose.header.frame_id = "/map"
        ag.goal.target_pose.pose.position.x = point[0]
        ag.goal.target_pose.pose.position.y = point[1]
        ag.goal.target_pose.pose.orientation.z = orient[2]
        ag.goal.target_pose.pose.orientation.w = orient[3]
        self.goal_pub.publish(ag)     
    
    def Listen(self):
        self.refresh = False
        rospy.init_node('map_view', anonymous=False)
        rospy.Subscriber("map", OccupancyGrid, self.MapCB)
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.PoseCB)
        rospy.Subscriber("node_traveller/dest", UInt32, self.DestCB)
        rospy.Subscriber("node_traveller/route", Int32MultiArray, self.RouteCB)
        rospy.Subscriber("move_base/result", MoveBaseActionResult, self.StatusCB)
        rospy.Subscriber("move_base_node/local_costmap/obstacles", GridCells, self.ObsCB)
#         rospy.Subscriber("move_base_node/local_costmap/inflated_obstacles", GridCells, self.ObsCB2)
        
        if __name__ == '__main__':
            rospy.spin()       

#---------------------------------------------------------------------------------------------#    
#    Callback function for the "/node_traveller/dest" topic.                                  #
#    -> Highlights the robot's current destination.                                           #
#---------------------------------------------------------------------------------------------#                
    def DestCB(self, data):
        dest = int(data.data) 
        wx.CallAfter(self.mframe.HighlightDestination, dest)      

#---------------------------------------------------------------------------------------------#    
#    Callback function for the "/amcl_pose" topic.                                            #
#    -> Moves the graphical robot representation to a new pose.                               #
#---------------------------------------------------------------------------------------------#    
    def PoseCB(self, data):
        try:
            self.pose_pos = data.pose.pose.position
            self.pose_orient = data.pose.pose.orientation                
            destination = (self.pose_pos.x, self.pose_pos.y)
            orient = self.pose_orient
            wx.CallAfter(self.mframe.MoveRobotTo, destination, orient, True)
            
        except wx.PyDeadObjectError:
            print "EXIT"       

#---------------------------------------------------------------------------------------------#    
#    Callback function for the "move_base/result" topic.                                      #
#    -> Send a signal to the map viewer if the status is 'Destination Reached'                #
#---------------------------------------------------------------------------------------------#        
    def StatusCB(self, data):
        status = int(data.status.status)
        if status == 3:
            wx.CallAfter(self.mframe.OnReachDestination)

#---------------------------------------------------------------------------------------------#    
#    Callback function for the "node_traveller/route" topic.                                  #
#    -> Draws the route in the map viewer.                                                    #
#---------------------------------------------------------------------------------------------#
    def RouteCB(self, data):
        route = data.data
        wx.CallAfter(self.mframe.DrawRoute, route, False)
        if route[0] != -1:
            self.parent.mp.ep.btn_rte.Enable(True)
        else:
            self.parent.mp.ep.btn_rte.Enable(False)
        wx.Yield()

#---------------------------------------------------------------------------------------------#    
#    Callback functions for the "move_base/local_costmap/*obstacles" topics.                  #
#    -> Draws the obstacles in the map viewer (currently unused)                              #
#---------------------------------------------------------------------------------------------#        
    def ObsCB(self, data):
        self.obstacles_1 = data.cells        
        
        if self.count % MAX_COUNT == 0:
#             wx.CallAfter(self.mframe.DrawObstacles, self.obstacles_2, 'inf')
            wx.CallAfter(self.mframe.DrawObstacles, self.obstacles_1, 'obs')
        self.count = self.count+1
        
    def ObsCB2(self, data):
        self.obstacles_2 = data.cells
            
#---------------------------------------------------------------------------------------------#
#    Callback function for the "/map" topic.                                                  #
#    Turns the OccupancyGrid map data into an image.                                          #
#---------------------------------------------------------------------------------------------#   
    def MapCB(self, data):
        self.parent.mp.btn_rf.Enable(True)
          
        array_length = len(data.data)
        self.image_width = int(np.sqrt(array_length))
        self.resolution = self.Truncate(data.info.resolution, 5)
        self.origin_pos = data.info.origin.position
        self.origin_orient = data.info.origin.orientation        

        color_data = self.TranslateToRGB(data.data)             
        img=Image.new('RGB', (self.image_width,self.image_width))
        img.putdata(color_data)        
        # Rotate and flip image (otherwise it won't match the real map)
        img = img.rotate(180)
        img_mirror = ImageOps.mirror(img)
        
        if not self.refresh:
            try:
                os.remove(self.filename)
            except OSError:
                pass        
            img_mirror.save(self.filename)
            if self.mframe.modes['verbose']:         
                print "Map file created. (%s)" % self.filename
            self.refresh = True

        # Creates the wx.Image to be passed to the ZoomPanel
        self.image = self.PilImageToWxImage(img_mirror)
        self.image_data = data.data
        wx.CallAfter(self.mframe.SetMapMetadata, self.image_width,
                     self.resolution,self.origin_pos)

#---------------------------------------------------------------------------------------------#    
#    Creates a wx.Image object from the data in a PIL Image                                   #
#---------------------------------------------------------------------------------------------#
    def PilImageToWxImage(self, pil_img):
        wx_img = wx.EmptyImage( self.image_width, self.image_width )
        wx_img.SetData( pil_img.convert( 'RGB' ).tostring() )
        return wx_img        
    
#---------------------------------------------------------------------------------------------#    
#    Translates an OccupancyGrid into pixel colors for the map                                #
#    To work correctly, dark grey RGB should be >50 and light grey RGB should be >150         #
#---------------------------------------------------------------------------------------------#
    def TranslateToRGB(self, input_array): 
        output_array=[]
        
        for element in input_array:            
            if element==-1:
                output_array.append((100,100,92)) #unknown (dark grey)
            elif element==0:
                output_array.append((230,230,230)) #known (light grey)
            elif element<=100:
                output_array.append((0,0,0))   #blocked (black)
            else:
                pass #unexpected value
                    
        return output_array
    
    def Truncate(self, f, n):
        return ('%.*f' % (n + 1, f))[:-1]  
            
#---------------------------------------------------------------------------------------------#    
#    Returns the filename used by this listener when exporting map files.                     #
#---------------------------------------------------------------------------------------------#    
    def GetDefaultFilename(self):
        return self.filename    
    def SetAttributes(self):        
        self.mframe = self.parent.mp.mframe 
    
    
if __name__ == '__main__':
    ros = ROSNode(None)
    ros.Listen()
