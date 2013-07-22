#!/usr/bin/env python

import wx
import os, time
import rospy                                                #@UnresolvedImport
import numpy as np
from datetime import datetime
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

class ROSNode():
    
    def __init__(self, parent): 
        # pos -> geometry_msgs/Point: 
        #     float64 x
        #     float64 y
        #     float64 z       
        self.origin_pos = None
        self.pose_pos = None
        
        # orient -> geometry_msgs/Quaternion: 
        #     float64 x
        #     float64 y
        #     float64 z
        #     float64 w 
        self.origin_orient = None        
        self.pose_orient = None 
        
        self.vel_linear = None
        self.vel_angular = None
        self.obstacles = []
                 
        self.image_width = 1000
        self.resolution = None
        self.refresh = False
        self.ok = True
        self.image = None
        self.filename = FILENAME       
        
        self.parent = parent
#         self.tt = TimerThread(self)
#         self.tt.start()
        self.tour_pub = rospy.Publisher('tour', String)
        self.pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped)
        self.goal_pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal)
        
    def PublishTour(self):  
        msg = "Tour %s" % rospy.get_time()
        self.tour_pub.publish(String(msg))        
    
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
        rospy.Subscriber("cmd_vel", Twist, self.VelocityCB)
        rospy.Subscriber("tour", String, self.TourCB)
        rospy.Subscriber("node_traveller/dest", UInt32, self.DestCB)
        rospy.Subscriber("node_traveller/route", Int32MultiArray, self.RouteCB)
        rospy.Subscriber("move_base/result", MoveBaseActionResult, self.StatusCB)
#         rospy.Subscriber("move_base/goal", MoveBaseActionGoal, self.GoalCB)
        
        if __name__ == '__main__':
            rospy.spin()       

#---------------------------------------------------------------------------------------------#    
#    Callback function for the "/node_traveller/dest" topic                                   #
#---------------------------------------------------------------------------------------------#                
    def DestCB(self, data):
        dest = int(data.data)         
#         if dest == -1:
#             self.mframe.ClearHighlighting()
#         else:
        print "Heading to node %s" % dest
        self.mframe.HighlightDestination(dest)             

#---------------------------------------------------------------------------------------------#    
#    Callback function for the "/amcl_pose" topic                                             #
#---------------------------------------------------------------------------------------------#    
    def PoseCB(self, data):
        self.pose_pos = data.pose.pose.position
        self.pose_orient = data.pose.pose.orientation                
        destination = (self.pose_pos.x, self.pose_pos.y)
        orient = self.pose_orient
               
        self.mframe.MoveRobotTo(destination, orient, True)
        
#---------------------------------------------------------------------------------------------#    
#    Callback function for the "/cmd_vel" topic                                               #
#---------------------------------------------------------------------------------------------#    
    def VelocityCB(self, data):
        self.vel_linear = (data.linear.x, data.linear.y)
        self.vel_angular = (data.angular.z)
        
    def StatusCB(self, data):
        status = int(data.status.status)
        if status == 3:
            print "Reached destination."
#             self.mframe.OnReachDestination()
#         pass

    def RouteCB(self, data):
#         self.mframe.ShowRoute(data.data)
        pass
        
#     def ObsCB(self, data):
#         pass
#         if self.ok:
#             print "Received obstacles"
#             if ( (data.cell_width+0.01 < 0.05) or (data.cell_width-0.01 > 0.05) ):
#                 print "Warning: obstacle cell size different than expected"
#             self.obstacles = data.cells
# #             self.mframe.DrawObstacles(self.obstacles)
#             self.ok = False
        
    def TourCB(self, data):
        print "Received tour message"
        print data
            
#---------------------------------------------------------------------------------------------#    
#    Turns the OccupancyGrid data received from "/map" into an image file.                    #
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
        self.mframe.SetMapMetadata(self.image_width,self.resolution,self.origin_pos) 

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
