'''
Created on Jun 5, 2013

@author: jon
'''

RESOLUTION = 0.05   # Meters per pixel

class Edge():
    
    def __init__(self, id_number, node1, node2, length):        
        self.id = id_number
        self.node1 = node1
        self.node2 = node2
        self.length = length
        self.graphic = -1   # Used to link to the graphical object which represents the edge
        
    def GetId(self):
        return self.id
    
    def SetId(self, new_id):
        self.id = new_id

    def GetGraphicIndex(self):
        return self.graphic
    
    def SetGraphicIndex(self, new_object):
        self.graphic = new_object
        
    def GetMetricLength(self):
        return self.length * RESOLUTION