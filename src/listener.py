#!/usr/bin/env python

import os, time
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid as og
from geometry_msgs.msg import PoseWithCovarianceStamped as pwc
# from move_base_msgs.msg import MoveBaseActionGoal    ##incompatible with catkin##
import Image
import ImageOps

# TODO: self-ize everything to get rid of global calls
    
FILENAME = "map.png"

class listener():
    
    def __init__(self, position, orientation): 
        # pos -> geometry_msgs/Point: 
        #     float64 x
        #     float64 y
        #     float64 z       
        self.pos = position
        
        # orient -> geometry_msgs/Quaternion: 
        #     float64 x
        #     float64 y
        #     float64 z
        #     float64 w 
        self.orient = orientation 
          
        self.image_width = 1000
        self.refresh = False
        self.filename = FILENAME   
    
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
        print "callback recv'd: Goal ID %s" % str(data.goal_id) 
               

#---------------------------------------------------------------------------------------------#    
#    Callback function for the "/amcl_pose" topic                                             #
#---------------------------------------------------------------------------------------------#    
    def PoseCB(self, data):
        print "pose: %s" % str(data.pose.pose)
    
    def Listen(self):
        self.refresh = False
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("map", og, self.MapCB)
        rospy.Subscriber("amcl_pose", pwc, self.PoseCB)
        
        if __name__ == '__main__':
            rospy.spin()       
        
    
#---------------------------------------------------------------------------------------------#    
#    Turns the OccupancyGrid data received from "/map" into an image file.                    #
#---------------------------------------------------------------------------------------------#   
    def ProcessMapCB(self, data):  
        unknown=0
        array_length = len(data.data)
        self.image_width = int(np.sqrt(array_length))
        
        self.pos = data.info.origin.position
        self.orient = data.info.origin.orientation        
        
        if __name__ == '__main__':
            print "\nData received:\n%s" % str(data.info.origin)            
            print "\nMap array size:%s " % str(array_length)               
            for element in data.data:
                if element != -1:
                    unknown+=1                          
            print "Known elements: %s" % unknown    
            print "Unknown elements: %s" % (array_length-unknown)    
        
        color_data = self.TranslateToRGB(data.data) 
            
        img=Image.new('RGB', (self.image_width,self.image_width))
        img.putdata(color_data)
        
        # Rotate and flip image (otherwise it won't match the real map)
        img = img.rotate(180)
        img_mirror = ImageOps.mirror(img)
        
        try:
            os.remove(self.filename)
        except OSError:
            pass
        
        img_mirror.save(self.filename)    
        
        print "Map file created. (%s)" % self.filename
        
    
#---------------------------------------------------------------------------------------------#    
#    Translates an OccupancyGrid into pixel colors for the map                                #
#---------------------------------------------------------------------------------------------#
    def TranslateToRGB(self, input_array): 
        output_array=[]
        
        for element in input_array:            
            if element==-1:
                output_array.append((100,100,100)) #unknown (dark grey)
            elif element==0:
                output_array.append((205,205,205)) #known (light grey)
            elif element<=100:
                output_array.append((0,0,0))   #blocked (black)
            else:
                pass #unexpected value
                    
        return output_array
        
#---------------------------------------------------------------------------------------------#    
#    Returns the filename used by this listener when exporting map files.                     #
#---------------------------------------------------------------------------------------------#    
    def GetDefaultFilename(self):
        return self.filename   
    
    
if __name__ == '__main__':
    ls = listener(None, None)
    ls.Listen()
