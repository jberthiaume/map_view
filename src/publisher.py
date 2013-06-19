#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class publisher():
    
    def __init__(self, parent):
        self.parent = parent

    def PublishTour(self):
        pub = rospy.Publisher('tour', String)      
        str = "Tour %s" % rospy.get_time()
        pub.publish(String(str))
        rospy.sleep(1.0)
        pub.publish(String(str))
    
    
if __name__ == '__main__':
    pub = publisher(None)
    try:
        pub.PublishTour()
    except rospy.ROSInterruptException:
        pass