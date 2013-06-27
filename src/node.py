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
        
    def GetId(self):
        return self.id
    
    def SetId(self, new_id):
        self.id = new_id
        
    def GetCoords(self):
        return self.coords
    
    def SetCoords(self, new_coords):
        self.coords = new_coords
        
    def GetGraphicIndex(self):
        return self.graphic
    
    def SetGraphicIndex(self, new_object):
        self.graphic = new_object