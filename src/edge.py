'''
Created on Jun 5, 2013

@author: jon
'''

#TODO: fixme    
RESOLUTION = 0.05

class Edge():
    
    def __init__(self, id_number, node1, node2, length):        
        self.id = id_number
        self.node1 = node1
        self.node2 = node2
        self.length = length
        self.graphic = -1   # Used to link to the graphical object which represents the edge
        
    def GetMetricLength(self):
        return self.length * RESOLUTION