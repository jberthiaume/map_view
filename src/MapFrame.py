#!/usr/bin/env python 

'''
Created on May 30, 2013

@author: jon
'''

import wx 
import sys
import pickle
import math
import time
import Image
import numpy as np
import random as rand
import GraphStructs as gs
import NavCanvas, FloatCanvas
from wx.lib.floatcanvas.Utilities import BBox
from datetime import datetime

#----- Global colors -----#
NODE_FILL           = (240,240,240)
NODE_BORDER         = (119,41,83)
EDGE_COLOR          = (119,41,83)
ROBOT_FILL_1        = (100,100,100)
ROBOT_FILL_2        = (180,0,0)
ROBOT_BORDER        = (50,0,0)
HIGHLIGHT_COLOR     = (255,106,54)
DESTINATION_COLOR   = (50,200,50)
TEXT_COLOR          = (119,41,83)

#----- Global dimensions for graphical objects -----#
NODE_DIAM           = 9
NODE_BORDER_WIDTH   = 2
EDGE_WIDTH          = 5
ROBOT_DIAM          = 9
ROBOT_BORDER_WIDTH  = 2
FONT_SIZE_1         = 4     # for one/two-digit numbers
FONT_SIZE_2         = 3     # for three-digit numbers

class MapFrame(wx.Frame): 

    def __init__(self, *args, **kwargs): 
        wx.Frame.__init__(self, *args, **kwargs) 
        self.CreateStatusBar()
        
        # Initialize data structures   
        self.export = False
        self.verbose = True
        self.redraw = True
        self.autoedges = True 
        self.edgespacing = True
        self.cleargraph = True
        self.leftdown = False       
        
        self.resolution = None
        self.origin = None
        self.robot = None
        self.image_width = None
        self.gg_const = (100,6,30,5,80)
        self.current_map = []
        
        self.nodelist = []
        self.edgelist = []
        self.graphics_nodes = []
        self.graphics_edges = []
        self.graphics_text = []
        
        self.sel_nodes = []
        self.sel_edges = []  
        self.curr_dest = None      
        
        # Connection matrix data structure
        # See GenerateConnectionMatrix()
        self.conn_matrix = np.empty(shape=(150,150))
        self.conn_matrix[:] = -1          
       
        self.mp = self.GetParent()
        self.ros = self.mp.ros
            
        # Add the Canvas
        self.NavCanvas = NavCanvas.NavCanvas(self, 
                                     ProjectionFun = None, 
                                     BackgroundColor = "DARK GREY", 
                                     )
        self.Canvas = self.NavCanvas.Canvas
        
        # Menu bar                 
        self.file_menu = wx.Menu()
        self.file_menu.Append(1001, '&Open Map\tCtrl+O')
        self.file_menu.Append(1002, '&Save Map\tCtrl+S')        
        self.file_menu.Append(1003, 'Save &As...\tCtrl+Shift+S')
        self.file_menu.AppendSeparator()      
        self.file_menu.Append(1008, 'Close Map\tCtrl+W')
        self.file_menu.Append(1009, 'Q&uit\tCtrl+Q')
                
        menu_bar = wx.MenuBar()
        menu_bar.Append(self.file_menu, '&File')
        self.SetMenuBar(menu_bar)
         
        # Menu event binders
        wx.EVT_MENU(self,1001,self.OnOpen)
        wx.EVT_MENU(self,1002,self.OnSave)        
        wx.EVT_MENU(self,1003,self.OnSaveAs)
        wx.EVT_MENU(self,1008,self.OnCloseMap)
        wx.EVT_MENU(self,1009,self.OnExit)
        
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
        
#---------------------------------------------------------------------------------------------#    
#    Hides the window instead of closing it when the X button is pressed                      #
#---------------------------------------------------------------------------------------------#    
    def OnClose(self, event):
        self.mp.OnShowHideMap(event)            

#---------------------------------------------------------------------------------------------#    
#    Writes the current cursor coordinates to the status bar at the bottom                    #
#---------------------------------------------------------------------------------------------#
    def OnMove(self, event): 
        self.SetStatusText("%i, %i"%tuple(event.Coords))
        
#---------------------------------------------------------------------------------------------#    
#    Event handlers passed down to the main panel                                             #
#---------------------------------------------------------------------------------------------#        
    def OnOpen(self, event):
        self.mp.OnOpen(event)
         
    def OnSave(self, event):
        self.mp.OnSave(event) 
                
    def OnSaveAs(self, event):
        self.mp.OnSaveAs(event)         
        
    def OnSettings(self, event):
        self.mp.OnSettings(event)  
        
    def OnCloseMap(self, event):
        self.mp.OnCloseMap(event)       
        
    def OnExit(self, event):
        self.mp.OnExit(event)        
    
#---------------------------------------------------------------------------------------------#    
#    Mouse click handler: left button                                                         #
#---------------------------------------------------------------------------------------------# 
    def OnLeftDown(self, event):
        current_mode = self.Canvas.GetMode()        
        if current_mode=='GUIMouse':
            self.CreateNode(event.Coords)            
        elif current_mode=='GUISelect':
            pass               
    
    def OnLeftUp(self, event):
        current_mode = self.Canvas.GetMode()        
        if current_mode=='GUISelect':
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
        
#---------------------------------------------------------------------------------------------#    
#    Event handler for moving nodes around with the arrow keys                                #
#---------------------------------------------------------------------------------------------#      
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
            else:
                return
                
            if int(ID) < 100:  
                fs = FONT_SIZE_1
            else:
                fs = FONT_SIZE_2            
            node.coords = xy
            node.m_coords = self.PixelsToMeters(xy)
            
            # Flag edges for redrawing if they are connected to a node which will move
            for idx,edge_id in enumerate(self.conn_matrix[int(ID)]):
                if idx > len(self.nodelist):
                    break
                if edge_id != -1 and idx != int(ID):
                    edges_to_redraw.append( int(edge_id) )                    
            
            self.graphics_nodes[int(ID)].Move(dxy)
            self.graphics_text[ int(ID)].Move(dxy)
            
            if self.verbose is True:
                print "Moved node %s to location %s" % ( str(node.id), str(xy) )
            self.SelectOneNode(self.graphics_nodes[int(ID)],False)
        
        # Redraw edges to correspond to the new coordinates of their endpoints
        for edge_id in edges_to_redraw: 
            n1 = self.nodelist[ int(self.edgelist[edge_id].node1) ]  
            n2 = self.nodelist[ int(self.edgelist[edge_id].node2) ]    
            self.Canvas.RemoveObject(self.graphics_edges[edge_id])
            
            e = self.Canvas.AddLine((n1.coords, n2.coords), LineWidth=ew, LineColor=EDGE_COLOR)
            e.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickEdge)
            e.Bind(FloatCanvas.EVT_FC_ENTER_OBJECT, self.OnMouseEnterEdge)
            e.Bind(FloatCanvas.EVT_FC_LEAVE_OBJECT, self.OnMouseLeaveEdge)
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


#---------------------------------------------------------------------------------------------#    
#    Sets a destination for the robot graphic and starts the animation timer                  #
#---------------------------------------------------------------------------------------------#  
    def MoveRobotTo(self, dest, orient, metric):        
        if metric is True:
            self.destination = self.MetersToPixels(dest)
            dest = self.destination
        else:
            self.destination = dest        
        self.dest_theta = self.ToDegrees( self.Angle(orient) )         
        
        r = self.robot
        r.Coords = ( int(dest[0]), int(dest[1]) )
        a = self.arrow
        a.Coords = r.Coords         
        
        distance = self.Distance(dest, r.XY)
        if distance < 150:
            self.NumTimeSteps = 24  
        else:
            self.NumTimeSteps = 60 
        
        self.dx = (dest[0]-r.XY[0]) / self.NumTimeSteps
        self.dy = (dest[1]-r.XY[1]) / self.NumTimeSteps
        self.dt = (self.dest_theta - a.Theta) / self.NumTimeSteps  
        
#         print "dt: %s" % self.dt      
        
        self.arrow_drawn = False
        if self.dt > 6 or self.dt <-6:
            self.workaround = True
        else:
            self.workaround = False
        
        self.arrow_drawn = False
        self.TimeStep = 1
        self.Timer.Start(self.FrameDelay) 
        
             
#---------------------------------------------------------------------------------------------#    
#    Moves the animation forward 1 frame until the frame limit (NumTimeSteps) is reached.     #
#---------------------------------------------------------------------------------------------#        
    def ShowFrame(self):
        r = self.robot
        a = self.arrow
        dest = self.destination
        dest_theta = self.dest_theta       
        
        x,y = r.XY
        theta = a.Theta 
        
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
                                  
                a = self.Canvas.AddArrowLine((xy1,xy2), LineWidth = ROBOT_BORDER_WIDTH,
                                        LineColor = ROBOT_BORDER, ArrowHeadSize=10, InForeground = True)
                a.Coords = xy1
                a.Theta  = new_theta
                a.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickRobot)
                self.arrow = a
                
                self.TimeStep += 1
                self.Canvas.Draw(True)
                wx.GetApp().Yield(True)
        
        else:         
            self.robot.SetFillColor(ROBOT_FILL_2)
            self.arrow.Visible = True
            self.Timer.Stop()
            self.Canvas.Draw(True)
            
#---------------------------------------------------------------------------------------------#    
#    Event handler when the user clicks on the robot graphic.                                 #
#---------------------------------------------------------------------------------------------#    
    def OnClickRobot(self, obj):
        if self.verbose is True:
            print "Robot location:" 
            print "\t Pixels: (%s, %s)" % (int(obj.Coords[0]), int(obj.Coords[1]))
            m_Coords = self.PixelsToMeters(obj.Coords)
            print "\t Metric: (%s, %s)" % (m_Coords[0], m_Coords[1])
            
            
    def HighlightDestination(self, dest):
        if self.curr_dest is not None: 
            self.graphics_nodes[ self.curr_dest ].SetFillColor(NODE_FILL) 
        self.graphics_nodes[ dest ].SetFillColor(DESTINATION_COLOR)      
        self.curr_dest = dest
        self.Canvas.Draw(True)

#--------------------------------------------------------------------------------------------#    
#     Creates a single node at the given coordinates                                         #
#--------------------------------------------------------------------------------------------#    
    def CreateNode(self, coords): 
        # coords are in float, but we need int values for pixels
        node_coords = [int(coords[0]), int(coords[1])]   
        node = gs.Node(len(self.nodelist), node_coords)
        node.m_coords = self.PixelsToMeters(node.coords)
        
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
            c.Bind(FloatCanvas.EVT_FC_ENTER_OBJECT, self.OnMouseEnterNode)
            c.Bind(FloatCanvas.EVT_FC_LEAVE_OBJECT, self.OnMouseLeaveNode)
            self.graphics_nodes.append(c)
            c.Name = ID
            c.Coords = node_coords    
            
            t = self.Canvas.AddScaledText(ID, xy, Size=fs, Position="cc", 
                                    Color=TEXT_COLOR, Weight=wx.BOLD, InForeground = True)
            self.graphics_text.append(t)        
            
            # Assign the Circle to its node
            node.graphic =  int(c.Name)        
            self.nodelist.append(node) 
            try:
                # Tell the connection matrix that this node now exists
                self.conn_matrix[int(ID)][int(ID)] = 0  
                if self.verbose is True:
                    print "Created node %s at (%s, %s)" % (ID, node_coords[0], node_coords[1])
                    print "\tMetric: (%s, %s)" % (node.m_coords[0], node.m_coords[1])
                if self.autoedges is True:
                    self.ConnectNeighbors(node.id, self.gg_const[1], self.gg_const[4], True)
                
            except IndexError:
                # Out of space in the connection matrix - expand it by 50 nodes
                curr_len = len(self.conn_matrix[0])
                self.conn_matrix.resize((curr_len+50, curr_len+50))
                
                # Tell the connection matrix that this node now exists
                self.conn_matrix[int(ID)][int(ID)] = 0 
                if self.verbose is True:
                    print "Created node %s at (%s, %s)" % (ID, node_coords[0], node_coords[1])
                    print "\tMetric = (%s, %s)" % (node.m_coords[0], node.m_coords[1])
                if self.autoedges is True:
                    self.ConnectNeighbors(node.id, self.gg_const[1], self.gg_const[4], True)
            
            if self.redraw is True:
                self.Canvas.Draw(True)
            self.mp.SetSaveStatus(False) 
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
            
            # Add the coordinates of the nodes to be connected to a list of points
            points = []
            for i in range(len(self.sel_nodes)): 
                points.append( self.nodelist[ int(self.sel_nodes[i].Name) ].coords)  
                
            # Create the edges 
            lw = EDGE_WIDTH          
            cl = EDGE_COLOR              
            for j in range(len(points)-1): 
                
                try:
                    self.GenerateConnectionMatrix()
                    node1 = self.sel_nodes[j]
                    node2 = self.sel_nodes[j+1]
                    
                    # Only create the edge if no edge exists between the selected points
                    if int(self.conn_matrix[int(node1.Name)][int(node2.Name)]) < 0:
                        edge = gs.Edge(len(self.edgelist), node1.Name, node2.Name, 
                                      self.Distance(node1.Coords, node2.Coords))
                        edge.m_length = edge.length*self.resolution
                        self.edgelist.append(edge)
                               
                        e = self.Canvas.AddLine([points[j], points[j+1]], 
                                                LineWidth = lw, LineColor = cl)
                        e.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickEdge)
                        e.Bind(FloatCanvas.EVT_FC_ENTER_OBJECT, self.OnMouseEnterEdge)
                        e.Bind(FloatCanvas.EVT_FC_LEAVE_OBJECT, self.OnMouseLeaveEdge)
                        self.graphics_edges.append(e)
                        
                        e.Name = str(edge.id)
                        
                        edge.graphic = int(e.Name)
                        if self.verbose is True:
                            print "Created edge %s between nodes %s and %s" % (str(edge.id), 
                                                                                str(edge.node1),
                                                                                str(edge.node2))
                    else:
                        if self.verbose is True:
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
            self.mp.SetSaveStatus(False)

#--------------------------------------------------------------------------------------------#    
#    -deprecated-                                                                            #
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
#     Returns True if a point is suitable for creation of a new node, False otherwise.       #
#                                                                                            #
#     coords: The point in question                                                          #
#     w: minimum allowable distance between nodes and obstacles                              #
#     d: minimum allowable distance between any two nodes                                    #
#--------------------------------------------------------------------------------------------#            
    def CheckNodeLocation(self, image_data, coords, w, d):
        iw = self.image_width
        x = int(coords[0])
        y = int(coords[1])        
        
        if self.image_data_format is 'int':
            for i in range(-w, w, 1):
                if image_data[ (iw*(y-w)) + (x+i) ] != 0:
                    return False
                if image_data[ (iw*(y+w)) + (x+i) ] != 0:
                    return False
                if image_data[ (iw*(y+i)) + (x-w) ] != 0:
                    return False
                if image_data[ (iw*(y+i)) + (x+w) ] != 0:
                    return False
        elif self.image_data_format is 'byte':
            for i in range(-w, w, 1):
                if image_data[ (iw*(y-w)) + (x+i) ] != chr(230):
                    return False
                if image_data[ (iw*(y+w)) + (x+i) ] != chr(230):
                    return False
                if image_data[ (iw*(y+i)) + (x-w) ] != chr(230):
                    return False
                if image_data[ (iw*(y+i)) + (x+w) ] != chr(230):
                    return False
            
        for node2 in self.nodelist:
            if self.Distance(coords, node2.coords) < d:
#                 self.Canvas.AddArc((coords[0],coords[1]+d), (coords[0],coords[1]+18),
#                                    coords, LineColor=ROBOT_FILL_2, LineWidth=2)
#                 print "Collision with node %s" % node2.id
                return False        
        return True
        
#--------------------------------------------------------------------------------------------#
#     Returns True if an edge can be created between two given points, False otherwise.      #
#                                                                                            #
#     image_data: The pixel data of the current map                                          #
#     coords1/2: The endpoints of the potential edge (order doesn't matter)                  #
#     increment: Increasing this value will take less processing time, but can yield         #
#                inaccurate results. Default is 1.                                           #
#--------------------------------------------------------------------------------------------#     
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
        
        x1_1 = x1 + (( (NODE_DIAM) /2.0)*math.cos(theta+(math.pi/2)))
        y1_1 = y1 + (( (NODE_DIAM) /2.0)*math.sin(theta+(math.pi/2)))        
        x1_2 = x1 + (( (NODE_DIAM) /2.0)*math.cos(theta-(math.pi/2)))
        y1_2 = y1 + (( (NODE_DIAM) /2.0)*math.sin(theta-(math.pi/2)))
        
        pos = 0
        done = False
        last = False
        
        if self.edgespacing is True:
            while not done:
                x1 = int( x1_1+(pos*kx) )
                y1 = int( y1_1+(pos*ky) )
                x2 = int( x1_2+(pos*kx) )
                y2 = int( y1_2+(pos*ky) )                
                
                if self.image_data_format is 'int':
                    if image_data[ (w*y1)+x1 ] > 0 or image_data[ (w*y2)+x2 ] > 0:
                        return False                
                if self.image_data_format is 'byte':
                    if (image_data[ (w*y1)+x1 ] != chr(230) or image_data[ (w*y2)+x2 ] != chr(230)):
                        return False
                
                if not last and pos>ln:
                    last = True
                    pos = ln
                elif last:
                    done = True                    
                pos += increment
                
        else:
            while not done:
                x = int( x1+(pos*kx) )
                y = int( y1+(pos*ky) )     
                
                if self.image_data_format is 'int':
                    if image_data[ (w*y)+x ] != 0:
                        return False                
                if self.image_data_format is 'byte':
                    if image_data[ (w*y)+x ] != chr(230):
                        return False
                
                if not last and pos>ln:
                    last = True
                    pos = ln
                elif last:
                    done = True                    
                pos += increment
        return True

#--------------------------------------------------------------------------------------------#    
#     Generates a random graph on the current map (probabilistic roadmap)                    #                         #
#                                                                                            #
#     n: number of nodes to created                                                          #
#     k: number of neighbors to analyze for each node                                        #
#     d: minimum distance between nodes                                                      #
#     w: minimum distance from nodes to obstacles                                            #
#     e: maximum edge length                                                                 #
#--------------------------------------------------------------------------------------------#                    
    def GenerateGraph(self, n, k, d, w, e):
        wx.BeginBusyCursor()
        st = datetime.now()
        
        # Save current states
        vb = self.verbose
        ae = self.autoedges
        rd = self.redraw
        
        self.verbose = False
        self.autoedges = False
        self.redraw = False
        
        if self.cleargraph is True:
            self.SelectAll(None)
            self.DeleteSelection(None)        
        
        data = self.image_data
        lim = self.FindImageLimit(data,4)        
        b = lim[0]
        t = lim[1]
        l = lim[2]
        r = lim[3]
                
        nodes_created = 0
        while nodes_created < n:
            x = int( l + ((r-l)*rand.random()) )
            y = int( b + ((t-b)*rand.random()) )
            
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
        et = datetime.now()
        if self.verbose is True:
            print "Total time to generate graph: %s" % str(et-st)

#--------------------------------------------------------------------------------------------#
#     Find the distances from a given node 'node1' to all other nodes in the graph.          # 
#                                                                                            #
#     k: Number of nodes to return (function returns the 'k' closest nodes)                  #
#     e: Maximum search radius                                                               #                                                               #
#--------------------------------------------------------------------------------------------#     
    def GetNodeDistances(self, node1, k, e):
        distances = []
        for node2 in self.nodelist:
            d = self.Distance(node1.coords, node2.coords)
            if d < e:
                distances.append((d, node2.id))
        
        distances.sort(key=lambda tup: tup[0])
        return distances[0:k+1]

#--------------------------------------------------------------------------------------------#
#     Event handler for Connect Nodes command. Passes arguments to ConnectNeighbors(..)      #
#--------------------------------------------------------------------------------------------#    
    def OnConnectNeighbors(self, event):
        for node in self.sel_nodes:
            self.ConnectNeighbors( int(node.Name), self.gg_const[1], self.gg_const[4], True)
        self.DeselectAll(event)
    
#--------------------------------------------------------------------------------------------#
#     Tries to connect edges from a given 'input_node' to its closest neighbors              #
#                                                                                            #
#     k: The number of neighbors to analyze                                                  #
#     e: Maximum edge length (furthest distance to search for neighbors)                     #
#--------------------------------------------------------------------------------------------#    
    def ConnectNeighbors(self, input_node, k, e, refresh):  
        rd = self.redraw
        self.redraw = False
        
        data = self.image_data
        node1 = self.nodelist[ input_node ]
        distances = self.GetNodeDistances(node1, k, e)
        
        for entry in distances:                      
            
            if entry[1] == input_node:
                continue    # Skip the node if it's the same one we're at
            
            if entry[0] > e:
                break       # Stop if the remaining edges are too far away
            
            node2 = self.nodelist[ entry[1] ]
            if self.CheckEdgeLocation(data,node1.coords, 
                                      node2.coords,1) is True:    
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
        self.mp.SetSaveStatus(False)
        
        self.redraw = rd
        self.verbose = vb

#--------------------------------------------------------------------------------------------#    
#     Deletes a node.                                                                        #
#     This function should not be called directly -> use DeleteSelection() instead           #
#--------------------------------------------------------------------------------------------#        
    def RemoveNode(self, node):
        ID = int(node.Name)

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
        conn_mtx = np.empty(shape=Shape)        
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
        np.set_printoptions(edgeitems=edgeitems, linewidth=linewidth)  
              
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
    
    #---MOVE ME---#
    def Publish2DPoseEstimate(self, start_pt, end_pt):
        x1 = start_pt[0]
        y1 = start_pt[1]
        x2 = end_pt[0]
        y2 = end_pt[1]
        
        pose = self.PixelsToMeters(start_pt)
        
        theta = math.atan2(y2-y1, x2-x1)
        print "theta :: %s rad /  %s deg" % (theta, theta*(180/math.pi))
        z = math.sin(theta/2.0)
        w = math.cos(theta/2.0)
        
        orient = (0,0,z,w)        
        self.ros.Publish2DPoseEstimate(pose, orient)
        
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

#--------------------------------------------------------------------------------------------#    
#     Basic midpoint formula. Returns the center of two points.                              #
#--------------------------------------------------------------------------------------------#     
    def Midpoint(self, p1, p2):
        x1 = float(p1[0])
        x2 = float(p2[0])
        y1 = float(p1[1])
        y2 = float(p2[1])
        mpx = (x1+x2)/2
        mpy = (y1+y2)/2
        return mpx,mpy 
    
#---------------------------------------------------------------------------------------------#    
#    Returns an angle (in radians) from a given orientation quaternion                        #
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
    
    def PixelsToMeters(self, xy_p):
        x = (xy_p[0] * self.resolution) + self.origin.x
        y = (xy_p[1] * self.resolution) + self.origin.y
        return (x,y)  
    
    def MetersToPixels(self, xy_m):
        x = (xy_m[0] - self.origin.x) / self.resolution
        y = (xy_m[1] - self.origin.y) / self.resolution
        return (x,y)
    
#--------------------------------------------------------------------------------------------#    
#     Selects a single node. If desel is True, deselects everything else.                    #
#--------------------------------------------------------------------------------------------#   
    def SelectOneNode(self, obj, desel):
        if desel is True:      
            self.DeselectAll(event=None)
            if self.verbose is True:
                print "Selected Node #" + obj.Name      
        self.sel_nodes.append(obj)   
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

#--------------------------------------------------------------------------------------------#
#     Selects all nodes/edges located within a given X and Y range. This is used with the    #
#     'box selection' tool on the NavCanvas.                                                   #
#--------------------------------------------------------------------------------------------#           
    def SelectBox(self, x_range, y_range):
        rd = self.redraw
        self.redraw = False
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
        self.redraw = rd
        self.Canvas.Draw(True)
        
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

#--------------------------------------------------------------------------------------------#    
#     Event when a node is left-clicked.                                                     #
#     Adds the node to the selection list and highlights it on the canvas                    #
#--------------------------------------------------------------------------------------------# 
    def OnClickNode(self, obj):
        if obj in self.sel_nodes:
            coords = self.nodelist[int(obj.Name)].coords  
            if self.verbose is True:        
                print "Deselected Node %s  (%s, %s)" % (obj.Name, coords[0], coords[1])
            self.sel_nodes.remove(obj)
            obj.SetFillColor(NODE_FILL)
        else:  
            coords = self.nodelist[int(obj.Name)].coords   
            m_coords = self.nodelist[int(obj.Name)].m_coords 
            if self.verbose is True:        
                print "Selected Node %s" % (obj.Name)   
                print "\tPixel Location:   (%s, %s)" % (coords[0]  , coords[1]  )
                print "\tMetric Location:  (%s, %s)" % (m_coords[0], m_coords[1])
            self.sel_nodes.append(obj)       
            obj.SetFillColor(HIGHLIGHT_COLOR) 
        self.Canvas.Draw(True)
    
    def OnMouseEnterNode(self, obj):
        self.Canvas.SetCursor(self.Canvas.GUIMode.Cursors.PointerHandCursor)         
    def OnMouseLeaveNode(self, obj):
        self.Canvas.SetCursor(self.Canvas.GUIMode.Cursors.ArrowCursor)   
        
#--------------------------------------------------------------------------------------------#    
#     Event when an edge is left-clicked.                                                    #
#     Adds the node to the selection list and highlights it on the canvas                    #
#--------------------------------------------------------------------------------------------#            
    def OnClickEdge(self, obj): 
        if obj in self.sel_edges:  
            if self.verbose is True:        
                print "Deselected Edge " + obj.Name
            self.sel_edges.remove(obj)
            obj.SetLineColor(EDGE_COLOR)
        else: 
            if self.verbose is True:       
                print "Selected Edge " + obj.Name     
            self.sel_edges.append(obj)        
            obj.SetLineColor(HIGHLIGHT_COLOR) 
        self.Canvas.Draw(True) 
        
    def OnMouseEnterEdge(self, obj):
        self.Canvas.SetCursor(self.Canvas.GUIMode.Cursors.PointerHandCursor)         
    def OnMouseLeaveEdge(self, obj):
        self.Canvas.SetCursor(self.Canvas.GUIMode.Cursors.ArrowCursor)

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
        metadata = [self.image_width, self.resolution, self.origin]
        g = [self.nodelist, self.edgelist, metadata]
        pickle.dump(g,f)

#--------------------------------------------------------------------------------------------#    
#     Unpickles an existing graph file from the file system.                                 #
#     NodeList is in slot 0, EdgeList is in slot 1, Metadata is in slot 2.                   #
#--------------------------------------------------------------------------------------------#       
    def ImportGraph(self, f):
        g = pickle.load(f)
        try:
            self.SetNodeList( g[0] )   
            self.SetEdgeList( g[1] ) 
            self.SetMapMetadata( *g[2] )
        except IndexError:
            ori = gs.Origin((0,0))
            self.SetMapMetaData(None, 0.05, ori)

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
#     Sets global variables for metadata obtained from the ROS listener                      #
#--------------------------------------------------------------------------------------------#     
    def SetMapMetadata(self, width, res, origin):
        self.image_width = width
        self.resolution = float(res)
        self.origin = origin
                
#--------------------------------------------------------------------------------------------#    
#    Finds and returns the extremities of the graph (the topmost point, the leftmost point,  #
#    etc).                                                                                   #
#    The 'granularity' argument defines how extensively the search is performed: Higher      #
#    values for this argument will execute the function faster, but with less accuracy       #                                                        #
#--------------------------------------------------------------------------------------------#
    def FindImageLimit(self, image_data, granularity):
        st = datetime.now()       
        w      = self.image_width
        d      = granularity     #interval between scans
        top   = w
        bot   = 0
        left  = 0
        right = w
        
        foundB = False
        foundT = False
        foundL = False
        foundR = False         
               
        if self.image_data_format is 'int':
            for i in range(w/(2*d)):
                for j in range(w/d):
                    try:
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
                    except IndexError:
                        break
                if (foundB is True and foundT is True and
                    foundL is True and foundR is True):
                    break     
                
        #badcodingpraticelol        
        elif self.image_data_format is 'byte':
            for i in range(w/(2*d)):
                for j in range(w/d):
                    try:
                        if image_data[ (w*i*d)+(j*d) ] != chr(100) and foundB is False:
                            bot = i*d
                            foundB = True
                            
                        if image_data[ w-((w*i*d)+(j*d)) ] != chr(100) and foundT is False:
                            top = w-(i*d)
                            foundT = True
                            
                        if image_data[ (w*j*d)+(i*d) ] != chr(100) and foundL is False:
                            left = i*d
                            foundL = True
                            
                        if image_data[ w-((w*j*d)+(i*d)) ] != chr(100) and foundR is False:
                            right = w-(i*d)
                            foundR = True
                            
                        if (foundB is True and foundT is True and
                            foundL is True and foundR is True):
                            break                
                    except IndexError:
                        break
                if (foundB is True and foundT is True and
                    foundL is True and foundR is True):
                    break                   
      
        et = datetime.now()
        
        if self.verbose is True:
            print "Top edge of map at row %s" % (str(top))
            print "Bottom edge of map at row %s" % (str(bot)) 
            print "Left edge of map at column %s" % (str(left))   
            print "Right edge of map at column %s" % (str(right))              
            print "Total time to scan map data: %s" % str(et-st)
                
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
               
        self.Canvas.ZoomToBB(BBox.fromPoints(np.r_[tl,br]))
        
#--------------------------------------------------------------------------------------------#    
#     Zooms to fit the currently displayed map                                               #
#--------------------------------------------------------------------------------------------#        
    def ZoomToFit(self):
        data_ready = False
        while data_ready is False: 
            try:
                lim = self.FindImageLimit(self.image_data, 10) 
                data_ready = True
            except AttributeError:
                # Sleep until the image data can be obtained
                time.sleep(0.5)
        
        tl = (lim[1],lim[2])
        br = (lim[0],lim[3])
                      
        self.Canvas.ZoomToBB(BBox.fromPoints(np.r_[tl,br]))

#--------------------------------------------------------------------------------------------#
#     Displays a dialog box for showing 'loading' messages                                   #
#--------------------------------------------------------------------------------------------#        
    def SetBusyDialog(self, msg):  
        self.bdlg = wx.BusyInfo(msg, parent=self)          
    def KillBusyDialog(self):
        del self.bdlg
        
#---------------------------------------------------------------------------------------------#    
#    Creates a wx.Image object from the data in a PIL Image                                   #
#---------------------------------------------------------------------------------------------#
    def PilImageToWxImage(self, pil_img):
        wx_img = wx.EmptyImage( pil_img.size[0], pil_img.size[1] )
        wx_img.SetData( pil_img.convert( 'RGB' ).tostring() )
        return wx_img      
    
    def LogOutput(self, enabled):      
        if enabled is True: 
            sys.stdout = open('stdout.log', 'a')
        else:
            sys.stdout = sys.__stdout__
            
    def ClearGraph(self):
        self.SelectAll(None)
        self.DeleteSelection(None)
    
#--------------------------------------------------------------------------------------------#    
#     Clears the entire canvas                                                               #
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
        st = datetime.now()    
                 
        try:
            # Creates the image from a .png file (used when loading a .png map file)
            self.image_data = []
            image_file = image_obj
            
            # Load as a PIL image so that we can "flip" the data (otherwise it's wrong)
            pil_img = Image.open(image_obj)
            pil_img_flip = pil_img.transpose(Image.FLIP_TOP_BOTTOM)
            
            image = self.PilImageToWxImage(pil_img)
            image_flip = self.PilImageToWxImage(pil_img_flip)            
            image_file = image_obj
            self.image_data = image_flip.GetData()[0::3]
            self.image_data_format = "byte"
            
        except AttributeError:
            # Creates the image directly from a wx.Image object (used when refreshing a live map)
            image = image_obj
            image_file = self.ros.GetDefaultFilename()
            self.image_data = self.ros.image_data
            self.image_data_format = "int"    
        
        if self.image_width is None:
            self.image_width = image.GetHeight() # Case where metadata was not set
        img = self.Canvas.AddScaledBitmap( image, 
                                      (0,0), 
                                      Height=image.GetHeight(), 
                                      Position = 'bl')    
        self.LoadNodes()
        self.LoadEdges()
        self.GenerateConnectionMatrix()
                
        self.AddRobot(-1,-1)    
        self.SetCurrentMapPath(image_file)
        self.Show() 
        self.Layout()        
        self.ZoomToFit()
        
        et = datetime.now()
        self.verbose = vb
        self.autoedges = ae
        self.redraw = rd
        if self.verbose is True:
            print "Time taken to set image: %s" % str(et-st)

if __name__ == '__main__':
    app = wx.App(False) 
    F = ZoomPanel(None, title="Map Viewer", size=(700,700) ) 
    app.MainLoop() 
