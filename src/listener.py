#!/usr/bin/env python

import os, time
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
#import scipy.misc.pilutil as smp
import Image
import ImageOps

# TODO: self-ize everything to get rid of global calls
    
refresh=False
filename = "map.png"

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
    
    def callback(self, data):
        global refresh
        if refresh==False:
            self.ProcessCallback(data)
            refresh=True
            
            # Kill this listener
    #         pid = os.getpid()
    #         os.kill(pid,1)
            #rospy.loginfo(rospy.get_name() + "\nMESSAGE:\n%s" % str(data.info))
        
        
    def listen(self):
        global refresh
        refresh = False
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("map", OccupancyGrid, self.callback)
#         rospy.spin()
        
    
    ''' Turns the OccupancyGrid pixel data into a map image'''    
    def ProcessCallback(self, data):  
        unknown=0
        array_length = len(data.data)
        self.image_width = int(np.sqrt(array_length))
        
        self.pos = data.info.origin.position
        self.orient = data.info.origin.orientation        
        
    ###   DEBUG    ### 
#         print "\nData received:\n%s" % str(data.info.origin)
    #   
    #     print "\nMap array size:%s " % str(array_length)  
    #    
    #     for element in data.data:
    #         if element != -1:
    #             unknown+=1             
    #    
    #     print "Known elements: %s" % unknown    
    #     print "Unknown elements: %s" % (array_length-unknown)    
        
        color_data = self.TranslateToRGB(data.data) 
            
        img=Image.new('RGB', (self.image_width,self.image_width))
        img.putdata(color_data)
        
        # Rotate and flip image (otherwise it won't match the real map)
        img = img.rotate(180)
        img_mirror = ImageOps.mirror(img)
    
        try:
            os.remove(filename)
        except OSError:
            pass
        
        img_mirror.save(filename)    
        
        print "Map file created... %s" % filename
        
    
    ''' Translates an OccupancyGrid list into RGB values '''
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
        
    
    
    def GetDefaultFilename(self):
        return filename   
    
    
if __name__ == '__main__':
    ls = listener(None, None)
    ls.listen()
