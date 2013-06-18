#!/usr/bin/env python

import wx
import os, time
import rospy
import numpy as np
from datetime import datetime
from nav_msgs.msg import OccupancyGrid as og
from geometry_msgs.msg import PoseWithCovarianceStamped as pwc
from geometry_msgs.msg import Twist as t
# from move_base_msgs.msg import MoveBaseActionGoal    ##incompatible with catkin##
import Image
import ImageOps

FILENAME = "map.png"
RESOLUTION = 0.05

class listener():
    
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
                 
        self.image_width = 1000
        self.refresh = False
        self.image = None
        self.filename = FILENAME       
        
        self.parent = parent

    
    def Listen(self):
        self.refresh = False
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("map", og, self.MapCB)
        rospy.Subscriber("amcl_pose", pwc, self.PoseCB)
        rospy.Subscriber("cmd_vel", t, self.VelocityCB)
        
        if __name__ == '__main__':
            rospy.spin()  
                
#---------------------------------------------------------------------------------------------#    
#    Callback function for the "/map" topic                                                   #
#---------------------------------------------------------------------------------------------#       
    def MapCB(self, data):
        if self.refresh==False:
            self.ProcessMapCB(data)
            self.refresh=True
        #             if __name__ == '__main__':              
        #                 print "Ending process."
        #                 pid = os.getpid()
        #                 os.kill(pid,1)


#---------------------------------------------------------------------------------------------#    
#    Callback function for the "/move_base/goal" topic                                        #
#---------------------------------------------------------------------------------------------#                
    def GoalCB(self, data):
        pass               

#---------------------------------------------------------------------------------------------#    
#    Callback function for the "/amcl_pose" topic                                             #
#---------------------------------------------------------------------------------------------#    
    def PoseCB(self, data):
        self.pose_pos = data.pose.pose.position
        self.pose_orient = data.pose.pose.orientation 
           
#         print data.pose.pose.position
#         print ""
                
        destination = self.ConvertToPixels((self.pose_pos.x, self.pose_pos.y))
        orient = self.pose_orient
        
#         try:        
        self.zp.MoveRobotTo(destination, orient)
#         except AttributeError:
#             print "Unexpected error while issuing move command"
        
#---------------------------------------------------------------------------------------------#    
#    Callback function for the "/cmd_vel" topic                                             #
#---------------------------------------------------------------------------------------------#    
    def VelocityCB(self, data):
        self.vel_linear = (data.linear.x, data.linear.y)
        self.vel_angular = (data.angular.z)
            
#---------------------------------------------------------------------------------------------#    
#    Turns the OccupancyGrid data received from "/map" into an image file.                    #
#---------------------------------------------------------------------------------------------#   
    def ProcessMapCB(self, data):  
        array_length = len(data.data)
        self.image_width = int(np.sqrt(array_length))
        
        self.origin_pos = data.info.origin.position
        self.origin_orient = data.info.origin.orientation        
        
#         if __name__ == '__main__':            
#             unknown=0
#             print "\nData received:\n%s" % str(data.info.origin)            
#             print "\nMap array size:%s " % str(array_length)               
#             for element in data.data:
#                 if element != -1:
#                     unknown+=1                          
#             print "Known elements: %s" % unknown    
#             print "Unknown elements: %s" % (array_length-unknown)    
        
#         self.FindEdges(data.data)
        color_data = self.TranslateToRGB(data.data) 
            
        img=Image.new('RGB', (self.image_width,self.image_width))
        img.putdata(color_data)
        
        # Rotate and flip image (otherwise it won't match the real map)
        img = img.rotate(180)
        img_mirror = ImageOps.mirror(img)
        
#         try:
#             os.remove(self.filename)
#         except OSError:
#             pass        
#         img_mirror.save(self.filename)         
#         print "Map file created. (%s)" % self.filename

        # Creates the wx.Image to be passed to the ZoomPanel
        self.image = self.PilImageToWxImage(img_mirror) 

#---------------------------------------------------------------------------------------------#    
#    Creates a wx.Image object from the data in a PIL Image                                   #
#---------------------------------------------------------------------------------------------#
    def PilImageToWxImage(self, pil_img):
        wx_img = wx.EmptyImage( self.image_width, self.image_width )
        wx_img.SetData( pil_img.convert( 'RGB' ).tostring() )
        return wx_img        
    
#---------------------------------------------------------------------------------------------#    
#    Translates an OccupancyGrid into pixel colors for the map                                #
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
    
#---------------------------------------------------------------------------------------------#    
#    (incomplete)                                                                             #
#---------------------------------------------------------------------------------------------#    
    def FindEdges(self, input_array):     
        top    = None
        bot    = None
        left   = None
        right  = None
        
        w      = self.image_width
        mid    = w/2        
        found_vert = 0
        found_horz = 0
        
        for i in range(w):
            if found_vert is 0:
                if input_array[ (w*i)+mid ] >= 0:
                    print "Found bottom edge at row " + str(i)
                    bot = w
                    found_vert = 1
            elif found_vert is 1:
                if input_array[ (w*i)+mid ] < 0:
                    print "Found top edge at row " + str(i)
                    top = w
                    found_vert = 2
                          
            if found_horz is 0:
                if input_array[ (w*mid)+i ] >= 0:
                    print "Found left edge at column " + str(i)
                    left = w
                    found_horz = 1
            elif found_horz is 1:
                if input_array[ (w*mid)+i ] < 0:
                    print "Found right edge at column " + str(i)
                    right = w
                    found_horz = 2  
                                      
            if found_vert is 2 and found_horz is 2:
                break        
    
#---------------------------------------------------------------------------------------------#    
#    Converts metric coordinates to pixel coordinates.                                        #
#---------------------------------------------------------------------------------------------#    
    def ConvertToPixels(self, m_coords):
        x = (m_coords[0] / RESOLUTION)
        y = (m_coords[1] / RESOLUTION)
        return (x,y)
        
#---------------------------------------------------------------------------------------------#    
#    Returns the filename used by this listener when exporting map files.                     #
#---------------------------------------------------------------------------------------------#    
    def GetDefaultFilename(self):
        return self.filename     
    
    def SetAttributes(self):        
        self.zp = self.parent.mp.zp  
    
    
if __name__ == '__main__':
    ls = listener(None)
    ls.Listen()
