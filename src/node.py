'''
Created on Jun 4, 2013

@author: jon
'''
class Node():
    
    def __init__(self, id_number, coords):        
        self.id = id_number
        self.prev_id = -1       # Used to save the old node id when renumbering is done
        self.graphic = -1       # Used to link to the graphical object which represents the node
        self.coords = coords        
        self.m_coords = None