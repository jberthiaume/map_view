#!/usr/bin/env python 

'''
Created on May 30, 2013

@author: jon
'''

import wx 
import pickle
import math
import time
import edge as ED
import node as N
import random as R
import numpy as NP
import listener as LS
import publisher as PB
import NavCanvas, FloatCanvas
from wx.lib.floatcanvas.Utilities import BBox
from datetime import datetime

RESOLUTION = 0.05

#----- Global colors -----#
NODE_FILL           = (240,240,240)
NODE_BORDER         = (119,41,83)
EDGE_COLOR          = (119,41,83)
ROBOT_FILL_1        = (100,100,100)
ROBOT_FILL_2        = (180,0,0)
ROBOT_BORDER        = (50,0,0)
HIGHLIGHT_COLOR     = (255,106,54)
TEXT_COLOR          = (119,41,83)

#----- Global dimensions for graphical objects -----#
NODE_DIAM           = 9
NODE_BORDER_WIDTH   = 2
EDGE_WIDTH          = 5
ROBOT_DIAM          = 9
ROBOT_BORDER_WIDTH  = 2
FONT_SIZE_1         = 4     # for one/two-digit numbers
FONT_SIZE_2         = 3     # for three-digit numbers

class ZoomPanel(wx.Frame): 

    def __init__(self, *args, **kwargs): 
        wx.Frame.__init__(self, *args, **kwargs) 
        self.CreateStatusBar() 
        
        # Initialize data structures   
        self.export = False
        self.verbose = False
        self.redraw = True
        self.autoedges = True 
        self.leftdown = False       
        
        self.origin = None
        self.robot = None
        self.image_width = 2000
        self.last_sel_node = "" 
        self.gg_const = (100,6,30,5,80)
        self.current_map = []
        
        self.nodelist = []
        self.edgelist = []
        self.graphics_nodes = []
        self.graphics_edges = []
        self.graphics_text = []
        
        self.sel_nodes = []
        self.sel_edges = []  
        
#         sys.stdout = open('log.txt', 'w')       
        
        # Connection matrix data structure
        # See "GenerateConnectionMatrix()" for more info
        self.conn_matrix = NP.empty(shape=(150,150))
        self.conn_matrix[:] = -1  
        
        try:
            self.ls = self.GetParent().ls
        except AttributeError:
            print "Warning: GUI listener object not found"
            self.ls = LS.listener() 
        try:
            self.pb = self.GetParent().pb
        except AttributeError:
            print "Warning: GUI publisher object not found"
            self.pb = PB.publisher() 
            
        # Add the Canvas 
        Canvas = NavCanvas.NavCanvas(self, 
                                     ProjectionFun = None, 
                                     BackgroundColor = "DARK GREY", 
                                     ).Canvas 
        Canvas.MaxScale=4 
        self.Canvas = Canvas
        
        # Bind canvas mouse events
        self.Bind(wx.EVT_CLOSE, self.OnClose)
        self.Bind(wx.EVT_CHAR_HOOK, self.OnKeyPress)
        
        FloatCanvas.EVT_MOTION(self.Canvas, self.OnMove )        
        self.Canvas.Bind(FloatCanvas.EVT_LEFT_DOWN, self.OnLeftDown)
        self.Canvas.Bind(FloatCanvas.EVT_LEFT_UP, self.OnLeftUp)
        self.Canvas.Bind(FloatCanvas.EVT_RIGHT_DOWN, self.OnRightDown)
        
        id_sel_all = wx.NewId()
        id_desel_all = wx.NewId()
        id_sel_nodes = wx.NewId()
        id_sel_edges = wx.NewId()
        id_create_edges = wx.NewId()
        id_connect = wx.NewId()
        id_del = wx.NewId()
        
        wx.EVT_MENU(self, id_sel_all, self.SelectAll) 
        wx.EVT_MENU(self, id_desel_all, self.DeselectAll) 
        wx.EVT_MENU(self, id_sel_nodes, self.SelectNodes)
        wx.EVT_MENU(self, id_sel_edges, self.SelectEdges)
        wx.EVT_MENU(self, id_create_edges, self.CreateEdges)
        wx.EVT_MENU(self, id_connect, self.OnConnectNeighbors)
        wx.EVT_MENU(self, id_del, self.DeleteSelection)
        
        # Accelerator table for hotkeys
        self.accel_tbl = wx.AcceleratorTable([(wx.ACCEL_CTRL, ord('A'), id_sel_all),
                                              (wx.ACCEL_CTRL, ord('D'), id_desel_all),
                                              (wx.ACCEL_CTRL|wx.ACCEL_SHIFT, ord('N'), id_sel_nodes),
                                              (wx.ACCEL_CTRL|wx.ACCEL_SHIFT, ord('E'), id_sel_edges),
                                              (wx.ACCEL_CTRL, ord('E'), id_create_edges),
                                              (wx.ACCEL_CTRL, ord('K'), id_connect),
                                              (wx.ACCEL_NORMAL, wx.WXK_DELETE, id_del)
                                             ])
        self.SetAcceleratorTable(self.accel_tbl)  
        self.Layout()
        
    
    def OnClose(self, event):
        self.GetParent().OnShowHideMap(event)            

#---------------------------------------------------------------------------------------------#    
#    Writes the current cursor coordinates to the status bar at the bottom                    #
#---------------------------------------------------------------------------------------------#
    def OnMove(self, event): 
        self.SetStatusText("%i, %i"%tuple(event.Coords))
        
    
#---------------------------------------------------------------------------------------------#    
#    Mouse click handler: left button                                                         #
#---------------------------------------------------------------------------------------------# 
    def OnLeftDown(self, event):
        current_mode = self.Canvas.GetMode()        
        if current_mode=='GUIMouse':
            self.CreateNode(event.Coords)            
        elif current_mode=='GUIMove':
            pass               
    
    def OnLeftUp(self, event):
        current_mode = self.Canvas.GetMode()        
        if current_mode=='GUIMove':
            while self.Canvas.SelBoxStart is None:
                time.sleep(0.1)
            
            start   = self.Canvas.SelBoxStart  
            end     = self.Canvas.SelBoxEnd 
            x_range = (start[0], end[0])
            y_range = (start[1], end[1])
            self.SelectBox(x_range, y_range)
        else:
            pass   
    
#---------------------------------------------------------------------------------------------#    
#    Mouse click handler: right button (opening the menu)                                     #
#---------------------------------------------------------------------------------------------#
    def OnRightDown(self, event):
        rc_menu = wx.Menu()
        
        rc_menu.Append(11, "(%s, %s)" % (int(event.Coords[0]), int(event.Coords[1])))
        rc_menu.Enable(11, False)
        
        rc_menu.AppendSeparator()
        
        rc_menu.Append(21, '&Create edges\tCtrl+E')        
        wx.EVT_MENU(self,21,self.CreateEdges)
        if len(self.sel_nodes) < 2:
            rc_menu.Enable(21, False)
            
        rc_menu.Append(22, 'Connect Neighbors\tCtrl+K')        
        wx.EVT_MENU(self,22,self.OnConnectNeighbors)
        if len(self.sel_nodes) < 1:
            rc_menu.Enable(22, False)
        
        rc_menu.Append(23, '&Delete Selection\tDel')        
        wx.EVT_MENU(self,23,self.DeleteSelection)
        if len(self.sel_nodes)<1 and len(self.sel_edges)<1:
            rc_menu.Enable(23, False)
        
        rc_menu.AppendSeparator()
        
        # Select submenu
        rc_submenu1 = wx.Menu()
        rc_submenu1.Append(311, '&All\tCtrl+A')        
        wx.EVT_MENU(self,311,self.SelectAll)
        rc_submenu1.Append(312, '&Nodes\tCtrl+Shift+N')
        wx.EVT_MENU(self,312,self.SelectNodes)
        rc_submenu1.Append(313, '&Edges\tCtrl+Shift+E') 
        wx.EVT_MENU(self,313,self.SelectEdges)  
                     
        rc_submenu1.AppendSeparator()    
                    
        rc_submenu1.Append(315, 'Deselect &All\tCtrl+D')
        wx.EVT_MENU(self,315,self.DeselectAll)        
        rc_menu.AppendMenu(31, '&Select..', rc_submenu1)
        
        self.PopupMenu( rc_menu, (event.GetPosition()[0]+10, event.GetPosition()[1]+30) )
        rc_menu.Destroy()  
        
      
    def OnKeyPress(self, event):
        rd = self.redraw
        self.redraw = False
        selection = list(set(self.sel_nodes))
        edges_to_redraw = []
        self.DeselectAll(None)
        
        step = 5
        diam = NODE_DIAM
        ew = EDGE_WIDTH
        lw = NODE_BORDER_WIDTH
        lc = NODE_BORDER    
        fc = HIGHLIGHT_COLOR
        
        for item in selection:
            ID = item.Name
            node = self.nodelist[int(ID)]            
            
            if event.GetKeyCode() == wx.WXK_UP:
                xy = node.coords[0], node.coords[1]+step
                dxy = 0,5
            elif event.GetKeyCode() == wx.WXK_DOWN:
                xy = node.coords[0], node.coords[1]-step
                dxy = 0,-5
            elif event.GetKeyCode() == wx.WXK_LEFT:
                xy = node.coords[0]-step, node.coords[1]
                dxy = -5,0
            elif event.GetKeyCode() == wx.WXK_RIGHT:
                xy = node.coords[0]+step, node.coords[1]
                dxy = 5,0
                
            if int(ID) < 100:  
                fs = FONT_SIZE_1
            else:
                fs = FONT_SIZE_2            
            node.coords = xy
            node.m_coords = node.MetricCoords(xy)
            
            for idx,edge_id in enumerate(self.conn_matrix[int(ID)]):
                if idx > len(self.nodelist):
                    break
                if edge_id != -1:
                    edges_to_redraw.append( int(edge_id) )                    
            
            self.graphics_nodes[int(ID)].Move(dxy)
            self.graphics_text[ int(ID)].Move(dxy)
            
            if self.verbose is True:
                print "Moved node %s to location %s" % ( str(node.id), str(xy) )
            self.SelectOneNode(self.graphics_nodes[int(ID)],False)
        
        for edge_id in edges_to_redraw: 
            n1 = self.nodelist[ int(self.edgelist[edge_id].node1) ]  
            n2 = self.nodelist[ int(self.edgelist[edge_id].node2) ]    
            self.Canvas.RemoveObject(self.graphics_edges[edge_id])
            
            e = self.Canvas.AddLine((n1.coords, n2.coords), LineWidth=ew, LineColor=EDGE_COLOR)
            e.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickEdge)
            self.graphics_edges[edge_id] = e           
            e.Name = str(edge_id)
        
        self.redraw = rd    
        self.Canvas.Draw(True)            
        
              
#---------------------------------------------------------------------------------------------#    
#    Adds a representation of the robot to the canvas. A grey robot means that its pose info  #
#    is not accurate. When accurate pose info is received, the robot graphic turns red.       #
#---------------------------------------------------------------------------------------------#    
    def AddRobot(self, coords, orient):
        # If no coords are specified, just put the robot at the origin
        if coords == -1:
            xy = (0,0)
            zw = (0,1)
        else:
            xy = coords
            zw = orient            
        diam = ROBOT_DIAM
        lw = ROBOT_BORDER_WIDTH
        lc = ROBOT_BORDER    
        fc = ROBOT_FILL_1  
        
        r = self.Canvas.AddCircle(xy, diam, LineWidth = lw,
                                  LineColor = lc, FillColor = fc, InForeground = True)
        r.Coords = xy
        self.robot = r
        r.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickRobot)
        
        theta = self.ToDegrees( self.Angle(zw) )                
        a = self.Canvas.AddArrowLine( (xy, (xy[0]+diam, xy[1]) ), LineWidth = lw,
                                 LineColor = lc, ArrowHeadSize=10, InForeground = True)
        a.Coords = xy
        a.Theta  = theta
        self.arrow = a
        a.Visible = False
        a.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickRobot)
        
        self.Timer = wx.PyTimer(self.ShowFrame)
        self.FrameDelay = 16        ## 16ms per frame = framerate of 60 FPS
        self.Canvas.Draw(True)
#         return r


#---------------------------------------------------------------------------------------------#    
#    Sets a destination for the robot graphic and determines the distance to travel per frame #
#---------------------------------------------------------------------------------------------#  
    def MoveRobotTo(self, dest, orient):
        self.destination = dest        
        self.dest_theta = self.ToDegrees( self.Angle(orient) )         
        
        r = self.robot
        r.Coords = ( int(dest[0]), int(dest[1]) )
        a = self.arrow
        a.Coords = r.Coords 
        
#         print "rXY  " + str(r.XY)
#         print "dest " + str(dest)
#         print "deg dest " + str(self.dest_theta)
#         print "rad dest " + str(self.Angle(orient) )         
        
        self.NumTimeSteps = 24  
        
        self.dx = (dest[0]-r.XY[0]) / self.NumTimeSteps
        self.dy = (dest[1]-r.XY[1]) / self.NumTimeSteps
        self.dt = (self.dest_theta - a.Theta) / self.NumTimeSteps        
        
        self.arrow_drawn = False
        if self.dt > 6 or self.dt <-6:
            self.workaround = True
        else:
            self.workaround = False
        
        self.arrow_drawn = False
        self.TimeStep = 1
        self.Timer.Start(self.FrameDelay) 
        
             
#---------------------------------------------------------------------------------------------#    
#    Moves the robot graphic forward 1 frame until the frame limit (NumTimeSteps) is reached. #
#---------------------------------------------------------------------------------------------#        
    def ShowFrame(self):
        r = self.robot
        a = self.arrow
        dest = self.destination
        dest_theta = self.dest_theta       
        
        x,y = r.XY
        theta = a.Theta           
#         if ((x+self.dx >= dest[0] and self.dx > 0 or
#              x+self.dx <= dest[0] and self.dx < 0 ) and
#            ( y+self.dy >= dest[1] and self.dy > 0 or
#              y+self.dy <= dest[1] and self.dy < 0 ) and
#             ( theta+self.dt >= dest_theta and self.dt > 0 or
#               theta+self.dt <= dest_theta and self.dt < 0) 
#             ):            
#             print "stopped"
#             self.TimeStep = self.NumTimeSteps
        
        if  self.TimeStep < self.NumTimeSteps:
            xy1 = (x+self.dx, y+self.dy) 
                
            r.Move( (self.dx,self.dy) )            
            try:
                self.Canvas.RemoveObject(self.arrow)
            except ValueError:
                pass
            
            if self.workaround is True and self.arrow_drawn is False:
                new_theta = dest_theta
                new_theta_rad = self.ToRadians(new_theta)
                xy2 = ( x + (ROBOT_DIAM * math.sin(new_theta_rad)), 
                        y + ROBOT_DIAM * math.cos(new_theta_rad) )                
#                 print "Drew arrow from %s to %s. Angle: %s" % (str(dest), str(xy2), self.theta)
                  
                a = self.Canvas.AddArrowLine((dest,xy2), LineWidth = ROBOT_BORDER_WIDTH,
                                     LineColor = ROBOT_BORDER, ArrowHeadSize=15, InForeground = True)
                a.Coords = r.XY
                a.Theta  = new_theta
                self.arrow = a
                a.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickRobot)
                self.arrow_drawn = True
             
            elif self.workaround is False:    
                new_theta = theta + self.dt             
                new_theta_rad = self.ToRadians(new_theta)
                xy2 = ( x+(ROBOT_DIAM * math.cos(new_theta_rad)), y+(ROBOT_DIAM * math.sin(new_theta_rad)) )
                 
#                 print "rad " + str(new_theta_rad)
#                 print "deg " + str(new_theta)
                 
                a = self.Canvas.AddArrowLine((xy1,xy2), LineWidth = ROBOT_BORDER_WIDTH,
                                        LineColor = ROBOT_BORDER, ArrowHeadSize=10, InForeground = True)
                a.Coords = xy1
                a.Theta  = new_theta   ##fixme
                a.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickRobot)
                self.arrow = a
                
                self.TimeStep += 1
                self.Canvas.Draw(True)
                wx.GetApp().Yield(True)
        
        else:
#             print "stopped (timer)"            
            self.robot.SetFillColor(ROBOT_FILL_2)
            self.arrow.Visible = True
            self.Timer.Stop()
            self.Canvas.Draw(True)
            
    
    def OnClickRobot(self, obj):
        if self.verbose is True:
            print "Robot location: (%s, %s)" % (obj.Coords[0], obj.Coords[1])

#--------------------------------------------------------------------------------------------#    
#     Creates a single node at the given coordinates                                         #
#--------------------------------------------------------------------------------------------#    
    def CreateNode(self, coords): 
        # coords are in float, but we need int values for pixels
        node_coords = [int(coords[0]), int(coords[1])]   
        node = N.Node(len(self.nodelist), node_coords)
        
        ID = str(len(self.nodelist))
        xy = node_coords[0], node_coords[1]
        diam = NODE_DIAM
        lw = NODE_BORDER_WIDTH
        lc = NODE_BORDER    
        fc = NODE_FILL
        if int(ID) < 100:  
            fs = FONT_SIZE_1
        else:
            fs = FONT_SIZE_2   
        
        collision = self.DetectCollision(node)
        if collision < 0:  
                   
            # Draw the node on the canvas
            c = self.Canvas.AddCircle(xy, diam, LineWidth=lw, LineColor=lc, FillColor=fc,
                                      InForeground = True)
            c.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickNode)    # Make the node 'clickable'
            self.graphics_nodes.append(c)
            c.Name = ID
            c.Coords = node_coords    
            
            t = self.Canvas.AddScaledText(ID, xy, Size=fs, Position="cc", 
                                    Color=TEXT_COLOR, Weight=wx.BOLD, InForeground = True)
            self.graphics_text.append(t)        
            
            # Assign the Circle to its node
            node.SetGraphicIndex( int(c.Name) )        
            self.nodelist.append(node) 
            try:
                # Tell the connection matrix that this node now exists
                self.conn_matrix[int(ID)][int(ID)] = 0  
                if self.verbose is True:
                    print "Created Node #%s at (%s, %s)" % (ID, node_coords[0], node_coords[1])
                    print "\tMetric = (%s, %s)" % (node.m_coords[0], node.m_coords[1])
                if self.autoedges is True:
                    self.ConnectNeighbors(node.id, self.gg_const[1], self.gg_const[4], True)
                
            except IndexError:
                # Out of space in the connection matrix - expand it by 50 nodes
                curr_len = len(self.conn_matrix[0])
                self.conn_matrix.resize((curr_len+50, curr_len+50))
                
                # Tell the connection matrix that this node now exists
                self.conn_matrix[int(ID)][int(ID)] = 0 
                if self.verbose is True:
                    print "Created Node #%s at (%s, %s)" % (ID, node_coords[0], node_coords[1])
                    print "\tMetric = (%s, %s)" % (node.m_coords[0], node.m_coords[1])
                if self.autoedges is True:
                    self.ConnectNeighbors(node.id, self.gg_const[1], self.gg_const[4], True)
            
            if self.redraw is True:
                self.Canvas.Draw(True)
            self.GetParent().SetSaveStatus(False) 
            return True 
                  
        else:
            if self.verbose is True:
                print "Could not create node at (%s, %s). (Collision with node %s)" \
                        % (node_coords[0], node_coords[1], str(collision))
            return False
        

#--------------------------------------------------------------------------------------------#    
#    Creates edges between all selected nodes. Edges are be created in the order that the    #
#    nodes were selected.                                                                    #
#--------------------------------------------------------------------------------------------#         
    def CreateEdges(self, event):
        if len(self.sel_nodes) >= 2:
                        
            max_x = 0
            max_y = 0
            min_x = 4000
            min_y = 4000
            
            # Add the coordinates of the nodes to be connected to a list of points
            points = []
            for i in range(len(self.sel_nodes)):
                
                # Taking note of the maximum and minimum coordinates for later
#                 if self.sel_nodes[i].Coords[0] > max_x:
#                     max_x = self.sel_nodes[i].Coords[0] + (NODE_DIAM/2)
#                 if self.sel_nodes[i].Coords[0] < min_x:
#                     min_x = self.sel_nodes[i].Coords[0] - (NODE_DIAM/2)
#                 if self.sel_nodes[i].Coords[1] > max_y:
#                     max_y = self.sel_nodes[i].Coords[1] + (NODE_DIAM/2)
#                 if self.sel_nodes[i].Coords[1] < min_y:
#                     min_y = self.sel_nodes[i].Coords[1] - (NODE_DIAM/2)
#                                     
#                 points.append(self.sel_nodes[i].Coords)   
                points.append( self.nodelist[ int(self.sel_nodes[i].Name) ].coords)  
                
            # Create the edges 
            lw = EDGE_WIDTH          
            cl = EDGE_COLOR              
            for j in range(len(points)-1): 
                
                try:
                    self.GenerateConnectionMatrix()
                    node1 = self.sel_nodes[j]
                    node2 = self.sel_nodes[j+1]

#                      TODO: integrate location checking in normal edge creation
#                     xxxxx = self.CheckEdgeLocation(self.ls.image_data, 
#                                            self.nodelist[int(node1.Name)].coords, 
#                                            self.nodelist[int(node2.Name)].coords, 1)
                    
                    # Only create the edge if no edge exists between the selected points
                    if int(self.conn_matrix[int(node1.Name)][int(node2.Name)]) < 0:
                        edge = ED.Edge(len(self.edgelist), node1.Name, node2.Name, 
                                      self.Distance(node1.Coords, node2.Coords))
                        self.edgelist.append(edge)
                               
                        e = self.Canvas.AddLine([points[j], points[j+1]], 
                                                LineWidth = lw, LineColor = cl)
                        e.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickEdge)
                        self.graphics_edges.append(e)
                        
                        e.Name = str(edge.id)
                        
                        edge.SetGraphicIndex( int(e.Name) )
                        if self.verbose is True:
                            print "Created edge #%s between nodes %s and %s" % (str(edge.id), 
                                                                                str(edge.node1),
                                                                                str(edge.node2))
                            print "\t(%s, %s) -> (%s, %s)" % (str(points[j][0]),
                                                                str(points[j][1]),
                                                                str(points[j+1][0]),
                                                                str(points[j+1][1]))
                    else:
                        if self.verbose is True:
                            print int(self.conn_matrix[int(node1.Name)][int(node2.Name)])
                            print "Did not create edge between nodes %s and %s\n\t(already exists)" \
                            % (node1.Name,node2.Name)
                except IndexError:
                    pass
#             if self.redraw is True:
#                 self.RefreshNodes(min_x, max_x, min_y, max_y)
            
            self.GenerateConnectionMatrix()   
            if self.redraw is True:
                self.Canvas.Draw(True)
            self.DeselectAll(event)
            self.GetParent().SetSaveStatus(False)

#--------------------------------------------------------------------------------------------#    
#    (no longer useful)                                                                      #
#--------------------------------------------------------------------------------------------#             
    def RefreshNodes(self, min_x, max_x, min_y, max_y):
        for node in self.nodelist:
            if (node.coords[0] <= max_x and node.coords[0] >= min_x and
                node.coords[1] <= max_y and node.coords[1] >= min_y):
                
                ID = str(node.id)
                xy = node.coords[0], node.coords[1]
                diam = NODE_DIAM
                lw = NODE_BORDER_WIDTH
                lc = NODE_BORDER    
                fc = NODE_FILL 
                if int(ID) < 100:  
                    fs = FONT_SIZE_1
                else:
                    fs = FONT_SIZE_2          
                                    
                # Draw the node on the canvas
                c = self.Canvas.AddCircle(xy, diam, LineWidth=lw, LineColor=lc, FillColor=fc)
                c.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickNode)
                self.Canvas.RemoveObject( self.graphics_nodes[int(ID)] )
                self.graphics_nodes[int(ID)] = c                     
                c.Name = ID
                c.Coords = node.coords                  
                
                # Make the old text invisible and replace it in the data structure
                t = self.Canvas.AddScaledText(ID, xy, Size=fs, Position="cc",Color=TEXT_COLOR, 
                                              Weight=wx.BOLD, InForeground = True)
                self.Canvas.RemoveObject( self.graphics_text[int(ID)] )
                self.graphics_text[int(ID)] = t
                

#--------------------------------------------------------------------------------------------#
#     coords = node's coords                                                                 #
#     dw = minimum distance to wall                                                          #
#     dn = minimum distance between nodes                                                    #
#--------------------------------------------------------------------------------------------#            
    def CheckNodeLocation(self, image_data, coords, dw, dn):
        w = self.image_width
        x = int(coords[0])
        y = int(coords[1])        

        for i in range(-dw, dw, 1):
            if image_data[ (w*(y-dw)) + (x+i) ] != 0:
                return False
            if image_data[ (w*(y+dw)) + (x+i) ] != 0:
                return False
            if image_data[ (w*(y+i)) + (x-dw) ] != 0:
                return False
            if image_data[ (w*(y+i)) + (x+dw) ] != 0:
                return False
            
        for node2 in self.nodelist:
            if self.Distance(coords, node2.coords) < dn:
#                 self.Canvas.AddArc((coords[0],coords[1]+dn), (coords[0],coords[1]+18),
#                                    coords, LineColor=ROBOT_FILL_2, LineWidth=2)
#                 print "Collision with node %s" % node2.id
                return False        
        return True
        
    
    def CheckEdgeLocation(self, image_data, coords1, coords2, increment):
        w  = self.image_width
        x1 = int(coords1[0])
        y1 = int(coords1[1])
        x2 = int(coords2[0])
        y2 = int(coords2[1])
        
        dx    = x2-x1
        dy    = y2-y1
        theta = math.atan2(dy,dx)
        ln    = self.Distance(coords1,coords2)
        kx    = math.cos(theta)
        ky    = math.sin(theta)
        
        pos = 0
        done = False
        last = False
        points = []
        while not done:
            x = int( x1+(pos*kx) )
            y = int( y1+(pos*ky) )
            points.append((x,y))
            
            if image_data[ (w*y)+x ] != 0:
#                 self.Canvas.AddPointSet(points, ROBOT_FILL_2, 2)
#                 self.Canvas.Draw(True)
                return False
            
            if not last and pos>ln:
                last = True
                pos = ln
            elif last:
                done = True
                
            pos += increment
#         self.Canvas.AddPointSet(points, ROBOT_FILL_2, 2)        
#         self.Canvas.Draw(True)
        return True

#--------------------------------------------------------------------------------------------#    
#     Automated graph creation algorithm (random)                                            #
#     n = number of nodes to created                                                         #
#     k = number of neighbors to analyze for each node                                       #
#     d = minimum distance between nodes                                                     #
#     e = maximum edge length                                                                #
#--------------------------------------------------------------------------------------------#                    
    def GenerateGraph(self, n, k, d, w, e):
        wx.BeginBusyCursor()
        
        # Save current states
        vb = self.verbose
        ae = self.autoedges
        rd = self.redraw
        
        self.verbose = False
        self.autoedges = False
        self.redraw = False
        
        self.SelectAll(None)
        self.DeleteSelection(None)        
#         self.ExportConnectionMatrix("conns.txt", 20, 500)
        
        data = self.ls.image_data
        lim = self.FindImageLimit(data,4)        
        b = lim[0]
        t = lim[1]
        l = lim[2]
        r = lim[3]
                
        nodes_created = 0
        while nodes_created < n:
            x = int( l + ((r-l)*R.random()) )
            y = int( b + ((t-b)*R.random()) )
            
            if self.CheckNodeLocation(data, (x,y), w, d) is True:
                if self.CreateNode( (x,y) ) is True:
                    self.ConnectNeighbors(nodes_created, k, e, False)
                    nodes_created += 1
        
#         self.RefreshNodes(0, self.image_width, 0, self.image_width)                   
        self.Canvas.Draw(True) 
        
        # Restore saved states
        self.verbose = vb  
        self.autoedges = ae
        self.redraw = rd
        wx.EndBusyCursor()
      
    def GetNodeDistances(self, node1, k):
        distances = []
        for node2 in self.nodelist:
            d = self.Distance(node1.coords, node2.coords)
            distances.append((d, node2.id))
        
        distances.sort(key=lambda tup: tup[0])
        return distances[0:k+1]
    
    def OnConnectNeighbors(self, event):
        for node in self.sel_nodes:
            self.ConnectNeighbors( int(node.Name), self.gg_const[1], self.gg_const[4], True)
        self.DeselectAll(event)
    
#--------------------------------------------------------------------------------------------#
#     k = number of neighbors to analyze for each node                                       #
#     m = maximum edge length                                                                #
#--------------------------------------------------------------------------------------------#    
    def ConnectNeighbors(self, input_node, k, e, refresh):  
        rd = self.redraw
        self.redraw = False
        
        data = self.ls.image_data
        node1 = self.nodelist[ input_node ]
        distances = self.GetNodeDistances(node1,k)
        
        for entry in distances:                      
            
            if entry[1] == input_node:
                continue    # Skip the node if it's the same one we're at
            
            if entry[0] > e:
                break       # Stop if the remaining edges are too far away
            
            node2 = self.nodelist[ entry[1] ]
            if self.CheckEdgeLocation(data,node1.coords, 
                                      node2.coords,4) is True:    
                self.DeselectAll(None)                     
                self.SelectOneNode(self.graphics_nodes[node1.id], False)
                self.SelectOneNode(self.graphics_nodes[node2.id], False)
                self.CreateEdges(None)
        
#         if refresh is True:
#             self.RefreshNodes( node1.coords[0]-e, node1.coords[0]+e,
#                                node1.coords[1]-e, node1.coords[1]+e )
        self.redraw = rd

#--------------------------------------------------------------------------------------------#    
#      Deletes all selected nodes and edges                                                  #
#--------------------------------------------------------------------------------------------#           
    def DeleteSelection(self, event):
        rd = self.redraw
        vb = self.verbose
        self.redraw = False
        self.verbose = False  
              
        for node in self.sel_nodes:
            self.RemoveNode(node)
        for edge in self.sel_edges:
            self.RemoveEdge(edge)
        
        self.RenumberNodes()  
        self.RenumberEdges()
        self.GenerateConnectionMatrix()        
        
        self.DeselectAll(event=None)
        self.Canvas.Draw(True)
        self.GetParent().SetSaveStatus(False)
        
        self.redraw = rd
        self.verbose = vb

#--------------------------------------------------------------------------------------------#    
#     Deletes a node.                                                                        #
#     This function should not be called directly -> use DeleteSelection() instead           #
#--------------------------------------------------------------------------------------------#        
    def RemoveNode(self, node):
        ID = int(node.Name)
        node_obj = self.nodelist[ID] # Pointer to the 'node.Node' data structure

        self.Canvas.RemoveObject(self.graphics_nodes[ ID ])
        self.graphics_text [ ID ].Visible = False
        self.Canvas.RemoveObject( self.graphics_text[ ID ] )
        
        for i in range(len(self.conn_matrix[ID])):
            e = int(self.conn_matrix[ID][i])
            if e >= 0 and i != ID:
                self.RemoveEdge( self.graphics_edges[e] )
        
        self.nodelist[ID] = None
        self.graphics_nodes[ID] = None
        self.graphics_text[ID] = None
        
        if self.verbose is True:
            print "Removed node #" + str(ID)
        if self.redraw is True:
            self.Canvas.Draw(True)
                
#--------------------------------------------------------------------------------------------#    
#     Deletes an edge.                                                                       #
#     This function should not be called directly -> use DeleteSelection() instead           #
#--------------------------------------------------------------------------------------------#           
    def RemoveEdge(self, edge):   
        try:     
            ID = int(edge.Name)
            edge_obj = self.edgelist[ID]  # Pointer to the 'edge.Edge' data structure
            
#             self.graphics_edges[ edge_obj.graphic ].Visible = False   
            self.Canvas.RemoveObject(self.graphics_edges[ID])                   
            self.edgelist[ID] = None
            self.graphics_edges[ID] = None            
            
            if self.verbose is True:
                print "Removed edge #" + str(ID)
            if self.redraw is True:
                self.Canvas.Draw(True)
            
        except AttributeError:
            # Case where the edge has already been removed: do nothing
            pass
        
#--------------------------------------------------------------------------------------------#    
#     Renumbers the nodes after a deletion                                                   #
#     This function should not be called directly -> use DeleteSelection() instead           #
#--------------------------------------------------------------------------------------------#            
    def RenumberNodes(self):
        nodes = []  # temporary variables
        graphics = []
        text = []
        
        for i in range(len(self.nodelist)):             
            if self.nodelist[i] is not None:
                self.nodelist[i].prev_id = i        # Save the old id for later use
                nodes.append(self.nodelist[i])
                graphics.append(self.graphics_nodes[i])
                text.append(self.graphics_text[i])
                
        for j in range(len(nodes)):
            if nodes[j].id != j:
                 
                nodes[j].id = j
                nodes[j].graphic = j
                graphics[j].Name = str(j)
                if j < 100:  
                    fs = FONT_SIZE_1
                else:
                    fs = FONT_SIZE_2
                
                # Make the old text invisible and replace it in the data structure                
                text[j].Visible = False
                xy = (nodes[j].coords[0], nodes[j].coords[1])
                t = self.Canvas.AddScaledText(str(j), xy, Size=fs, Position="cc", 
                                              Color=TEXT_COLOR, Weight=wx.BOLD, InForeground = True)
                text[j] = t
                
                for edge in self.edgelist:
                    if edge is not None:
                        if int(edge.node1) == nodes[j].prev_id:
                            edge.node1 = j
                        elif int(edge.node2) == nodes[j].prev_id:
                            edge.node2 = j
                    
                
        self.nodelist = nodes
        self.graphics_nodes= graphics
        self.graphics_text = text
        
#--------------------------------------------------------------------------------------------#    
#     Renumbers the edges after a deletion                                                   #
#     This function should not be called directly -> use DeleteSelection() instead           #
#--------------------------------------------------------------------------------------------#        
    def RenumberEdges(self):
        edges = [] # temporary variables
        graphics = []
        
        for i in range(len(self.edgelist)):             
            if self.edgelist[i] is not None:
                edges.append(self.edgelist[i])
                graphics.append(self.graphics_edges[i])
                 
        for j in range(len(edges)):
            if edges[j].id != j:                
                edges[j].id = j           
                edges[j].graphic = j                     
                graphics[j].Name = str(j) 
                
        self.edgelist = edges
        self.graphics_edges = graphics
    

#--------------------------------------------------------------------------------------------#    
#     Generates the connection matrix data structure.                                        #
#--------------------------------------------------------------------------------------------#     
#                                                                                            #
#     - If node A exists, then conn_matrix[A][A] is 0                                        #
#                                                                                            #
#     - If node A doesn't exist, then conn_matrix[A][A] is -1                                #
#                                                                                            #
#     - If an edge exists between node A and node B, then conn_matrix[A][B] and              #
#       conn_matrix[B][A] both contain the ID of the edge which connects them.               #
#                                                                                            #
#     - If no edge exists between node A and node B, then conn_matrix[A][B] and              #
#       conn_matrix[B][A] are both -1.                                                       #
#                                                                                            #
#--------------------------------------------------------------------------------------------#       
    def GenerateConnectionMatrix(self):        
        Shape = self.conn_matrix.shape
        conn_mtx = NP.empty(shape=Shape)        
        conn_mtx[:] = -1 
        
        # put the edge data into the matrix
        for edge in self.edgelist:
            conn_mtx[ int(edge.node1) ][ int(edge.node2) ] = edge.id
            conn_mtx[ int(edge.node2) ][ int(edge.node1) ] = edge.id
        
        # put zeros on the diagonal at locations where nodes exist
        for i in range(len(self.nodelist)):
            conn_mtx[i][i] = 0
            
        self.conn_matrix = conn_mtx
#         return conn_mtx
        
    
#--------------------------------------------------------------------------------------------#    
#     For debugging purposes. Writes the connection matrix to a text file.                   #
#--------------------------------------------------------------------------------------------#    
    def ExportConnectionMatrix(self, filename, edgeitems, linewidth):         
        NP.set_printoptions(edgeitems=edgeitems, linewidth=linewidth)  
              
        if self.export is False:
            conn_file = open(filename, "w")
            self.export = True
        else:
            conn_file = open(filename, "a")        
        
        L = len(self.nodelist)   
        export_mtx = self.conn_matrix[0:L,0:L]       
            
        conn_file.write(str( export_mtx ))
        conn_file.write("\n\n")
        conn_file.close()            
            

#--------------------------------------------------------------------------------------------#    
#     Returns -1 if there is no collision with another node. If there is a collision,        #
#     returns the ID of the first node there was a collision with.                           #
#--------------------------------------------------------------------------------------------#     
    
    def DetectCollision(self, new_node): 
        for existing_node in self.nodelist:
            try:
                if self.Distance(new_node.coords, existing_node.coords) <= NODE_DIAM:
                    return existing_node.id
            except AttributeError:
                pass
        return -1  
    
#     def SetOrigin(self, origin):
#         self.origin = origin
    
#--------------------------------------------------------------------------------------------#    
#     Basic distance formula. Returns the distance between two points.                       #
#--------------------------------------------------------------------------------------------# 
    def Distance(self, p1, p2): 
        x1 = float(p1[0])
        x2 = float(p2[0])
        y1 = float(p1[1])
        y2 = float(p2[1])
        dist = math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )  
        return dist  
    
    def Midpoint(self, p1, p2):
        x1 = float(p1[0])
        x2 = float(p2[0])
        y1 = float(p1[1])
        y2 = float(p2[1])
        mpx = (x1+x2)/2
        mpy = (y1+y2)/2
        return mpx,mpy 
    
#---------------------------------------------------------------------------------------------#    
#    Returns an angle (in radians) from an input orientation quaternion                       #
#---------------------------------------------------------------------------------------------#    
    def Angle(self, orient):
        try:
            z = orient.z
            w = orient.w
        except AttributeError:
            z = 0.0
            w = 1.0        
        w=w     #placeholder
        theta = 2*math.asin(z)    
        return theta % (2*math.pi)
    
    
#---------------------------------------------------------------------------------------------#    
#    Converts a degree angle to radians and transforms it into the robot's coordinate plane   #
#---------------------------------------------------------------------------------------------#
#     FloatCanvas Coordinate Plane         Robot Coordinate Plane                             #
#                                                                                             #
#                  0                               pi/2                                       #
#                  :                                 :                                        #
#                  :                                 :                                        #
#         270 -----+----- 90      --->       pi -----+----- 0                                 #
#                  :                                 :                                        #
#                  :                                 :                                        #
#                 180                              3pi/2                                      #
#                                                                                             #
#---------------------------------------------------------------------------------------------# 
    def ToRadians(self, angle):
        return ( ((-angle+90) * math.pi) / 180 ) % (2*math.pi)

#---------------------------------------------------------------------------------------------#    
#    Converts a radian angle to degrees and transforms it into the FC coordinate plane        #
#---------------------------------------------------------------------------------------------#  
#     FloatCanvas Coordinate Plane         Robot Coordinate Plane                             #
#                                                                                             #
#                  0                               pi/2                                       #
#                  :                                 :                                        #
#                  :                                 :                                        #
#         270 -----+----- 90      <---       pi -----+----- 0                                 #
#                  :                                 :                                        #
#                  :                                 :                                        #
#                 180                              3pi/2                                      #
#                                                                                             #
#---------------------------------------------------------------------------------------------#   
    def ToDegrees(self, angle):
        return ( 90 - (angle*(180/math.pi)) ) % 360   

    
#--------------------------------------------------------------------------------------------#    
#     Selects a single node. If desel is True, deselects everything else.                    #
#--------------------------------------------------------------------------------------------#   
    def SelectOneNode(self, obj, desel):
        if desel is True:      
            self.DeselectAll(event=None)
            if self.verbose is True:
                print "Selected Node #" + obj.Name      
        self.sel_nodes.append(obj)         
        self.last_sel_node = obj.Name
        obj.SetFillColor(HIGHLIGHT_COLOR)
        if self.redraw is True:
            self.Canvas.Draw(True)
        
#--------------------------------------------------------------------------------------------#    
#     Selects a single edge. If desel is True, deselects everything else.                    #
#--------------------------------------------------------------------------------------------#   
    def SelectOneEdge(self, obj, desel):
        if desel is True:     
            self.DeselectAll(event=None)
            if self.verbose is True:
                print "Selected Edge #" + obj.Name     
        self.sel_edges.append(obj)
        obj.SetLineColor(HIGHLIGHT_COLOR)
        if self.redraw is True:
            self.Canvas.Draw(True) 
            
#--------------------------------------------------------------------------------------------#    
#     Selects all nodes and deselects everything else.                                       #
#--------------------------------------------------------------------------------------------#        
    def SelectNodes(self, event):
        self.DeselectAll(event)
        
        # Set highlighted colour
        for node in self.nodelist:
            self.graphics_nodes[ node.graphic ].SetFillColor(HIGHLIGHT_COLOR)
            self.sel_nodes.append(self.graphics_nodes[node.graphic])
        if self.verbose is True:
                print "Selected all nodes"
        if self.redraw is True:    
            self.Canvas.Draw(True) 
        
#--------------------------------------------------------------------------------------------#    
#     Selects all edges and deselects everything else.                                       #
#--------------------------------------------------------------------------------------------#    
    def SelectEdges(self, event):
        self.DeselectAll(event)
        
        # Set highlighted colour
        for edge in self.edgelist:
            self.graphics_edges[ edge.graphic ].SetLineColor(HIGHLIGHT_COLOR)
            self.sel_edges.append(self.graphics_edges[edge.graphic])
        if self.verbose is True: 
            print "Selected all edges"
        if self.redraw is True:    
            self.Canvas.Draw(True)
            
    def SelectBox(self, x_range, y_range):
        self.DeselectAll(None)
        
        for node in self.nodelist:
            if( (x_range[0] <= node.coords[0] <= x_range[1] or
                 x_range[1] <= node.coords[0] <= x_range[0]) and
                (y_range[0] <= node.coords[1] <= y_range[1] or
                 y_range[1] <= node.coords[1] <= y_range[0]) ):
                self.SelectOneNode(self.graphics_nodes[node.id], False)
        for edge in self.edgelist:
            n1 = self.nodelist[ int(edge.node1) ]
            n2 = self.nodelist[ int(edge.node2) ]
            mp = self.Midpoint(n1.coords, n2.coords)
            if( (x_range[0] <= mp[0] <= x_range[1] or
                 x_range[1] <= mp[0] <= x_range[0]) and
                (y_range[0] <= mp[1] <= y_range[1] or
                 y_range[1] <= mp[1] <= y_range[0]) ):
                self.SelectOneEdge(self.graphics_edges[edge.id], False)
        
#--------------------------------------------------------------------------------------------#    
#     Select/deselect everything                                                             #
#--------------------------------------------------------------------------------------------#             
    def SelectAll(self, event):                       
        self.DeselectAll(event)   
           
        for node in self.nodelist:                
            if event is not None:   
                self.graphics_nodes[ node.graphic ].SetFillColor(HIGHLIGHT_COLOR)
            self.sel_nodes.append(self.graphics_nodes[node.graphic])
        for edge in self.edgelist:
            if event is not None:   
                self.graphics_edges[ edge.graphic ].SetLineColor(HIGHLIGHT_COLOR)
            self.sel_edges.append(self.graphics_edges[edge.graphic])
        
        if self.verbose is True:            
            print "Selected all nodes and edges"
        if self.redraw is True:    
            self.Canvas.Draw(True) 
            
    def DeselectAll(self, event):
        for obj in self.sel_nodes:
            obj.SetFillColor(NODE_FILL)
        for obj in self.sel_edges:
            obj.SetLineColor(EDGE_COLOR) 
        if self.redraw is True:            
            self.Canvas.Draw(True)                
        self.sel_nodes = []
        self.sel_edges = [] 
        self.last_sel_node = ""   

#--------------------------------------------------------------------------------------------#    
#     Event when a node is left-clicked.                                                     #
#     Adds the node to the selection list and highlights it on the canvas                    #
#--------------------------------------------------------------------------------------------# 
    def OnClickNode(self, obj):
        if obj.Name != self.last_sel_node:  
            coords = self.nodelist[int(obj.Name)].coords   
            if self.verbose is True:        
                print "Selected Node #%s  (%s, %s)" % (obj.Name, coords[0], coords[1])     
            self.sel_nodes.append(obj)       
            obj.SetFillColor(HIGHLIGHT_COLOR)    
            self.last_sel_node = obj.Name
            self.Canvas.Draw(True)
        
#--------------------------------------------------------------------------------------------#    
#     Event when an edge is left-clicked.                                                    #
#     Adds the node to the selection list and highlights it on the canvas                    #
#--------------------------------------------------------------------------------------------#            
    def OnClickEdge(self, obj):  
        if self.verbose is True:       
            print "Selected Edge #" + obj.Name     
        self.sel_edges.append(obj)        
        obj.SetLineColor(HIGHLIGHT_COLOR) 
        self.Canvas.Draw(True) 

#--------------------------------------------------------------------------------------------#    
#     "Setter" functions for file paths and data structures                                  #
#--------------------------------------------------------------------------------------------#            
    def SetCurrentMapPath(self, map_file):
        self.current_map = map_file
    
    def SetNodeList(self, new_list):
        self.nodelist = []
        self.nodelist = new_list
    
    def SetEdgeList(self, new_list):
        self.edgelist=[]
        self.edgelist = new_list

#--------------------------------------------------------------------------------------------#    
#     Pickles the NodeList and EdgeList data structures and saves them on the file system    #
#--------------------------------------------------------------------------------------------#        
    def ExportGraph(self, f):
        g = [self.nodelist, self.edgelist]
        pickle.dump(g,f)
        self.SetNodeList([])
        self.SetEdgeList([])

#--------------------------------------------------------------------------------------------#    
#     Unpickles an existing graph file from the file system.                                 #
#     NodeList is in slot 0, EdgeList is in slot 1.                                          #
#--------------------------------------------------------------------------------------------#       
    def ImportGraph(self, f):
        g = pickle.load(f)
        self.SetNodeList( g[0] )   
        self.SetEdgeList( g[1] )     

#--------------------------------------------------------------------------------------------#    
#     Iterates through an imported node list and creates the nodes.                          #
#     This function should not be called on its own, but rather as a part of SetImage()      #
#--------------------------------------------------------------------------------------------#                   
    def LoadNodes(self):
        tmp_nodelist = self.nodelist
        self.nodelist = []           
        self.graphics_nodes = []
        self.graphics_text = []
             
        for node in tmp_nodelist:
            self.CreateNode((node.coords[0],node.coords[1]))
            
    
#--------------------------------------------------------------------------------------------#    
#     Iterates through an imported edge list and creates the edges.                          #
#     This function should not be called on its own, but rather as a part of SetImage()      #
#--------------------------------------------------------------------------------------------#       
    def LoadEdges(self):
        tmp_edgelist = self.edgelist
        self.edgelist = []        
        self.graphics_edges = []
            
        
        for edge in tmp_edgelist:
            self.sel_nodes.append( self.graphics_nodes[ int(edge.node1) ] )
            self.sel_nodes.append( self.graphics_nodes[ int(edge.node2) ] )
            self.CreateEdges(event=None)     
        
#--------------------------------------------------------------------------------------------#    
#    Finds and returns the extremities of the graph (the topmost point, the leftmost point,  #
#    etc).                                                                                   #
#    The 'granularity' argument defines how extensively the search is performed: Higher      #
#    values for this argument will execute the subroutine faster, but with less accuracy     #                                                        #
#--------------------------------------------------------------------------------------------#
    def FindImageLimit(self, image_data, granularity):
#         st = datetime.now()       
        w      = self.image_width
        d      = granularity     #interval between scans
        
        foundB = False
        foundT = False
        foundL = False
        foundR = False
        
        top   = w
        bot   = 0
        left  = 0
        right = w
        
        for i in range(w/(2*d)):
            for j in range(w/d):
                if image_data[ (w*i*d)+(j*d) ] >= 0 and foundB is False:
                    bot = i*d
                    foundB = True
                    
                if image_data[ w-((w*i*d)+(j*d)) ] >= 0 and foundT is False:
                    top = w-(i*d)
                    foundT = True
                    
                if image_data[ (w*j*d)+(i*d) ] >= 0 and foundL is False:
                    left = i*d
                    foundL = True
                    
                if image_data[ w-((w*j*d)+(i*d)) ] >= 0 and foundR is False:
                    right = w-(i*d)
                    foundR = True
                    
                if (foundB is True and foundT is True and
                    foundL is True and foundR is True):
                    break
            if (foundB is True and foundT is True and
                foundL is True and foundR is True):
                break                     
      
        et = datetime.now()
        
#         if self.verbose is True:
#             print "Top edge at row %s" % (str(top))
#             print "Bottom edge at row %s" % (str(bot)) 
#             print "Left edge at column %s" % (str(left))   
#             print "Right edge at column %s" % (str(right))
#              
#             print "Total time: %s" % str(et-st)
                
        return bot,top,left,right

#--------------------------------------------------------------------------------------------#    
#     Zooms to a given location with a given floating-point magnification.                   #
#--------------------------------------------------------------------------------------------#        
    def Zoom(self, location, magnification): 
        iw = self.image_width/2
        x = location[0]
        y = location[1]
        m = magnification
        
        tl = ( x-(iw/m), y+(iw/m) )
        br = ( x+(iw/m), y-(iw/m) )
               
        self.Canvas.ZoomToBB(BBox.fromPoints(NP.r_[tl,br]))
        
#--------------------------------------------------------------------------------------------#    
#     Zooms to fit the currently displayed map                                               #
#--------------------------------------------------------------------------------------------#        
    def ZoomToFit(self):
        data_ready = False
        while data_ready is False: 
            try:
                lim = self.FindImageLimit(self.ls.image_data, 10) 
                data_ready = True
            except AttributeError:
                # Sleep until the image data can be obtained
                time.sleep(0.5)
        
        tl = (lim[1],lim[2])
        br = (lim[0],lim[3])
                      
        self.Canvas.ZoomToBB(BBox.fromPoints(NP.r_[tl,br]))

#--------------------------------------------------------------------------------------------#    
#     Saves the canvas as an image. (unused)                                                 #
#--------------------------------------------------------------------------------------------#
    def Binding(self, event): 
        print "Writing a png file:" 
        self.Canvas.SaveAsImage("zzzz.png") 
        print "Writing a jpeg file:" 
        self.Canvas.SaveAsImage("zzzz.jpg",wx.BITMAP_TYPE_JPEG) 
        
    
#--------------------------------------------------------------------------------------------#    
#     Clears the canvas                                                                      #
#--------------------------------------------------------------------------------------------#   
    def Clear(self):
        self.Canvas.InitAll()        
    
#--------------------------------------------------------------------------------------------#    
#     Sets the image to display on the canvas. If the map has an associated graph file,      #
#     the corresponding nodes and edges are loaded and drawn onto the canvas.                #
#--------------------------------------------------------------------------------------------#
    def SetImage(self, image_obj):
        # Save current states
        vb = self.verbose
        ae = self.autoedges
        rd = self.redraw
        
        self.verbose = False
        self.autoedges = False
        self.redraw = False     
        self.Clear()    
                 
        try:
            # Creates the image from a .png file (used when loading a .png map file)
            image = wx.Image(image_obj, wx.BITMAP_TYPE_ANY)
            image_file = image_obj
            rf_needed = True
            
        except TypeError:
            # Creates the image directly from a wx.Image object (used when refreshing a live map)
            image = image_obj
            image_file = self.ls.GetDefaultFilename()
            rf_needed = False
           
        img = self.Canvas.AddScaledBitmap( image, 
                                      (0,0), 
                                      Height=image.GetHeight(), 
                                      Position = 'bl')  
        self.image_width = image.GetHeight()         
            
        self.LoadNodes()
        self.LoadEdges()
        self.GenerateConnectionMatrix()
                
        self.AddRobot(-1,-1)
#         if rf_needed is True:
#             self.RefreshNodes(0, self.image_width, 0, self.image_width)
    
        self.SetCurrentMapPath(image_file)
        self.Show() 
        self.Layout()
        
        self.ZoomToFit()
        self.verbose = vb
        self.autoedges = ae
        self.redraw = rd

if __name__ == '__main__':
    app = wx.App(False) 
    F = ZoomPanel(None, title="Map Viewer", size=(700,700) ) 
    app.MainLoop() 
