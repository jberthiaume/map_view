#!/usr/bin/env python 

'''
Created on May 30, 2013

@author: jon
'''

import wx 
import pickle
import math
import time
import Image
import numpy as np
import random as rand
import threading as t
import GraphStructs as gs
import NavCanvas, FloatCanvas
from wx.lib.floatcanvas.Utilities import BBox
from TimerThread import TimerThread
from datetime import datetime

#----- Global colors -----#
NODE_FILL           = (240,240,240)
NODE_BORDER         = (119,41,83)
EDGE_COLOR          = (119,41,83)
ROBOT_FILL_1        = (100,100,100)
ROBOT_FILL_2        = (180,0,0)
ROBOT_BORDER        = (50,0,0)
ERROR_COLOR         = (255,45,45)
SELECT_COLOR        = (255,106,54)
HIGHLIGHT_COLOR     = (255,106,54)
DESTINATION_COLOR   = (30,230,40)
OBSTACLE_COLOR_1    = (255,106,20)
OBSTACLE_COLOR_2    = (240,210,20)
ROUTE_COLOR         = (20,165,0)
TEXT_COLOR          = (119,41,83)

#----- Global dimensions for graphical objects -----#
NODE_DIAM           = 9
NODE_BORDER_WIDTH   = 2
EDGE_WIDTH          = 5
ROBOT_DIAM          = 9
ROBOT_BORDER_WIDTH  = 2
FONT_SIZE_1         = 4     # for one/two-digit numbers
FONT_SIZE_2         = 3     # for three-digit numbers
FONT_SIZE_3         = 6     # large font

class MapFrame(wx.Frame): 

    def __init__(self, *args, **kwargs): 
        wx.Frame.__init__(self, *args, **kwargs) 
        self.CreateStatusBar()
        
        # Initialize data structures                
        self.modes = {
                      'export':False, 
                      'auto_erase':True, 
                      'verbose':True, 
                      'redraw':True, 
                      'auto_edges':True, 
                      'spaced_edges':True,  
                      'unknown_edges':True, 
                      'clear_graph':True, 
                      'auto_intersections':True, 
                      'manual_edges':False,
                      'pose_est':False, 
                      'load':False, 
                      'running':False, 
                      'obstacles':True,
                      }
        self.saved_modes = {}
        
        self.resolution = None
        self.origin = None
        self.robot = None
        self.image_width = None
        self.gg_const = self.GetParent().gg_const
        self.current_map = []
        
        self.nodelist = []
        self.edgelist = []
        self.highlights = []
        self.graphics_obs = []
        self.graphics_nodes = []
        self.graphics_edges = []
        self.graphics_text = []
        self.graphics_route = []
        
        self.sel_nodes = []
        self.sel_edges = [] 
        self.route = []
        self.obstacles_1 = None
        self.obstacles_2 = None
        self.arrows = None
        self.pe_graphic = None
        self.ng_graphic = None
        self.curr_dest = None   
        self.curr_edge = None
        self.started_edge = False    
        
        # Connection matrix data structure
        # See GenerateConnectionMatrix()
        self.conn_matrix = np.empty(shape=(150,150))
        self.conn_matrix[:] = -1   
       
        self.mp = self.GetParent()
        self.ros = self.mp.ros
#         self.q = Queue()
#         self.qt = qt.QueueThread(self)
#         self.qt.daemon = True
#         self.qt.start()
            
        # Add the Canvas
        self.NavCanvas = NavCanvas.NavCanvas(self, 
                                     ProjectionFun = None, 
                                     BackgroundColor = "DARK GREY", 
                                     )
        self.Canvas = self.NavCanvas.Canvas
#         self.Canvas.Debug = True

        
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
        id_close = wx.NewId()
        id_quit = wx.NewId()
        
        wx.EVT_MENU(self, id_sel_all, self.SelectAll) 
        wx.EVT_MENU(self, id_desel_all, self.DeselectAll) 
        wx.EVT_MENU(self, id_sel_nodes, self.SelectNodes)
        wx.EVT_MENU(self, id_sel_edges, self.SelectEdges)
        wx.EVT_MENU(self, id_create_edges, self.CreateEdges)
        wx.EVT_MENU(self, id_connect, self.OnConnectNeighbors)
        wx.EVT_MENU(self, id_del, self.DeleteSelection)
        wx.EVT_MENU(self, id_close, self.OnCloseMap)
        wx.EVT_MENU(self, id_quit, self.OnExit)
        
        # Accelerator table for hotkeys
        self.accel_tbl = wx.AcceleratorTable([(wx.ACCEL_CTRL, ord('A'), id_sel_all),
                                              (wx.ACCEL_CTRL, ord('D'), id_desel_all),
                                              (wx.ACCEL_CTRL|wx.ACCEL_SHIFT, ord('N'), id_sel_nodes),
                                              (wx.ACCEL_CTRL|wx.ACCEL_SHIFT, ord('E'), id_sel_edges),
                                              (wx.ACCEL_CTRL, ord('E'), id_create_edges),
                                              (wx.ACCEL_CTRL, ord('K'), id_connect),
                                              (wx.ACCEL_NORMAL, wx.WXK_DELETE, id_del),
                                              (wx.ACCEL_CTRL, ord('W'), id_close),
                                              (wx.ACCEL_CTRL, ord('Q'), id_quit)
                                             ])
        self.SetAcceleratorTable(self.accel_tbl)  
        self.Layout()
        
#---------------------------------------------------------------------------------------------#    
#    Hides the window instead of closing it when the X button is pressed                      #
#---------------------------------------------------------------------------------------------#    
    def OnClose(self, event):
        self.Hide()            

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
#    Mouse click handlers: left button                                                        #
#---------------------------------------------------------------------------------------------# 
    def OnLeftDown(self, event):
        current_mode = self.Canvas.GetMode()        
        if current_mode=='GUIMouse':
            self.CreateNode(event.Coords)
        elif current_mode == 'GUIEdges':
            if self.started_edge:
                self.CreateNode(event.Coords)
                self.started_edge = False           
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
#    Mouse click handlers: right button (opening the menu)                                    #
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
        current_mode = self.Canvas.GetMode()  
        if current_mode == 'GUIEdges':
            return
        
        self.SetModes('KeyPress', {
                        'redraw':False, 
                      })
        selection = list(set(self.sel_nodes))
        edges_to_redraw = []
        self.DeselectAll(None)
        
        step = 5
        ew = EDGE_WIDTH
        for item in selection:
            ID = item.Name
            node = self.nodelist[int(ID)]            
            
            if event.GetKeyCode() == wx.WXK_UP:
                xy = node.coords[0], node.coords[1]+step
                dxy = 0,step
            elif event.GetKeyCode() == wx.WXK_DOWN:
                xy = node.coords[0], node.coords[1]-step
                dxy = 0,-step
            elif event.GetKeyCode() == wx.WXK_LEFT:
                xy = node.coords[0]-step, node.coords[1]
                dxy = -step,0
            elif event.GetKeyCode() == wx.WXK_RIGHT:
                xy = node.coords[0]+step, node.coords[1]
                dxy = step,0
            else:
                self.RestoreModes('KeyPress')  
                self.Canvas.Draw(True)  
                return    
                 
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
            
            if self.modes['verbose']:
                print "Moved node %s to location %s" % ( str(node.id), str(xy) )
            self.SelectOneNode(self.graphics_nodes[int(ID)],False)
    
        # Redraw edges to correspond to the new coordinates of their endpoints 
        for edge_id in edges_to_redraw: 
            n1 = self.nodelist[ int(self.edgelist[edge_id].node1) ]  
            n2 = self.nodelist[ int(self.edgelist[edge_id].node2) ]
            
            self.Canvas.RemoveObject( self.graphics_edges[edge_id] )         
            e = self.Canvas.AddLine((n1.coords, n2.coords), LineWidth=ew, LineColor=EDGE_COLOR)
            e.Name = str(edge_id)
            self.graphics_edges[edge_id] = e 
            self.BindEvents( e, 'edge')
        
        self.Canvas.Draw(True)   
        self.RestoreModes('KeyPress')       
     
    def BindEvents(self, obj, obj_type):
        if obj_type == 'edge':
            obj.Bind( FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickEdge) 
            obj.Bind( FloatCanvas.EVT_FC_ENTER_OBJECT, self.OnMouseEnterEdge) 
            obj.Bind( FloatCanvas.EVT_FC_LEAVE_OBJECT, self.OnMouseLeaveEdge)
              
#---------------------------------------------------------------------------------------------#    
#    Adds a representation of the robot to the canvas. A grey robot means that its pose info  #
#    is not accurate. When accurate pose info is received, the robot graphic turns red.       #
#---------------------------------------------------------------------------------------------#    
    def AddRobot(self, coords, orient):
        # If no coords are specified, just put the robot at the origin
        if coords == -1:
            xy = (0,0)
            zw = 0   
            fc = ROBOT_FILL_1
        else:
            xy = coords
            zw = orient     
            fc = ROBOT_FILL_2          
        diam = ROBOT_DIAM
        lw = ROBOT_BORDER_WIDTH
        lc = ROBOT_BORDER   
        
        r = self.Canvas.AddCircle(xy, diam, LineWidth = lw,
            LineColor = lc, FillColor = fc, InForeground = True)
        r.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickRobot)
        r.Coords = xy
        self.robot = r
        
#             theta = self.ToDegrees( self.Angle(zw) )
        theta = zw                
        a = self.Canvas.AddArrowLine( (xy, (xy[0]+diam, xy[1]) ), LineWidth = lw,
            LineColor = lc, ArrowHeadSize=10, InForeground = True)   
        a.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickRobot)         
        self.Canvas.Draw(True)
        a.Coords = xy
        a.Theta  = theta
        self.arrow = a
        
        self.Timer = wx.PyTimer(self.ShowFrame)
        self.FrameDelay = 16        


#---------------------------------------------------------------------------------------------#    
#    Sets a destination for the robot graphic and starts the animation timer                  #
#---------------------------------------------------------------------------------------------#  
    def MoveRobotTo(self, dest, orient, metric):
        if self.robot is None:
            return
         
        try:    
            if metric:
                self.destination = self.MetersToPixels(dest)
                dest = self.destination
            else:
                self.destination = dest 
        except AttributeError:
            print "Could not move robot graphic (initialization error)"   
            return    
        self.dest_theta = self.ToDegrees( self.Angle(orient) )         
        
        r = self.robot
        r.Coords = ( int(dest[0]), int(dest[1]) )
        a = self.arrow
        a.Coords = r.Coords         
        
        distance = self.Distance(dest, r.XY)
        if distance < 150:
            self.NumTimeSteps = 4  
        else:
            self.NumTimeSteps = 4 
        
        self.dx = (dest[0]-r.XY[0]) / self.NumTimeSteps
        self.dy = (dest[1]-r.XY[1]) / self.NumTimeSteps
        self.dt = (self.dest_theta - a.Theta) / self.NumTimeSteps
        
        self.arrow_drawn = False
        if (self.dest_theta-a.Theta > 180 or self.dest_theta-a.Theta <-180):
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
        
        
        if  self.TimeStep < self.NumTimeSteps: 
            r.Move( (self.dx,self.dy) ) 
            x,y = r.XY
            theta = a.Theta           
            try:                  
                self.Canvas.RemoveObject(self.arrow)
            except ValueError:
                pass
            
            if self.workaround:
                new_theta = theta
                new_theta_rad = self.ToRadians(new_theta)
                xy2 = ( x + (ROBOT_DIAM * math.cos(new_theta_rad)), 
                        y + (ROBOT_DIAM * math.sin(new_theta_rad)) )                
                
                a = self.Canvas.AddArrowLine((r.XY,xy2), LineWidth = ROBOT_BORDER_WIDTH,
                        LineColor = ROBOT_BORDER, ArrowHeadSize=15, InForeground = True)
                a.Coords = r.XY
                a.Theta  = new_theta
                self.arrow = a
                self.arrow_drawn = True
             
            elif not self.workaround:    
                new_theta = theta + self.dt             
                new_theta_rad = self.ToRadians(new_theta)
                xy2 = ( x+(ROBOT_DIAM * math.cos(new_theta_rad)), y+(ROBOT_DIAM * math.sin(new_theta_rad)) )
                
                a = self.Canvas.AddArrowLine((r.XY,xy2), LineWidth = ROBOT_BORDER_WIDTH,
                        LineColor = ROBOT_BORDER, ArrowHeadSize=10, InForeground = True)
                a.Coords = r.XY
                a.Theta  = new_theta
                self.arrow = a
            
        
            self.Canvas.Draw(True)
            wx.GetApp().Yield(True)
            self.TimeStep += 1
        
        else: 
            # Last TimeStep: adjust position for rounding errors along the way
            error_xy = dest - r.XY 
            r.Move( (error_xy[0], error_xy[1]) )
            
            try:
                self.Canvas.RemoveObject(self.arrow)
            except ValueError:
                pass
            x,y = r.XY
            new_theta = dest_theta
            new_theta_rad = self.ToRadians(new_theta)
            xy2 = ( x + (ROBOT_DIAM * math.cos(new_theta_rad)), 
                    y + (ROBOT_DIAM * math.sin(new_theta_rad)) )  
            a = self.Canvas.AddArrowLine((r.XY,xy2), LineWidth = ROBOT_BORDER_WIDTH,
                             LineColor = ROBOT_BORDER, ArrowHeadSize=15, InForeground = True)
            a.Coords = r.XY
            a.Theta  = new_theta
            self.arrow = a            
                     
            self.robot.SetFillColor(ROBOT_FILL_2)
        
            self.Timer.Stop()
            self.Canvas.Draw(True) 
            
#---------------------------------------------------------------------------------------------#    
#    Sends 2D Pose Estimate data to the ROS node to be published                              #
#---------------------------------------------------------------------------------------------# 
    def Publish2DPoseEstimate(self, start_pt, end_pt, graphic_obj):
        x1 = start_pt[0]
        y1 = start_pt[1]
        x2 = end_pt[0]
        y2 = end_pt[1]
        self.pe_graphic = graphic_obj
        
        pose = self.PixelsToMeters(start_pt)
        
        theta = math.atan2(y2-y1, x2-x1)
        z = math.sin(theta/2.0)
        w = math.cos(theta/2.0)        
        orient = (0,0,z,w)        
        
        if self.modes['verbose']:
            x = self.Truncate(pose[0], 4)
            y = self.Truncate(pose[1], 4)
            print "Created 2D pose estimate at point (%s, %s)" % (x, y)
        self.SetModes('PoseEst', {'pose_est':True})
        self.ros.Publish2DPoseEstimate(pose, orient)
        
#---------------------------------------------------------------------------------------------#    
#    Sends 2D NavGoal data to the ROS node to be published                                    #
#---------------------------------------------------------------------------------------------# 
    def Publish2DNavGoal(self, start_pt, end_pt, graphic_obj):
        x1 = start_pt[0]
        y1 = start_pt[1]
        x2 = end_pt[0]
        y2 = end_pt[1]
        self.ng_graphic = graphic_obj
        
        pose = self.PixelsToMeters(start_pt)
        
        theta = math.atan2(y2-y1, x2-x1)
        z = math.sin(theta/2.0)
        w = math.cos(theta/2.0)        
        orient = (0,0,z,w)        
        
        if self.modes['verbose']:
            x = self.Truncate(pose[0], 4)
            y = self.Truncate(pose[1], 4)
            print "Created 2D nav goal at point (%s, %s)" % (x, y)
        self.ros.Publish2DNavGoal(pose, orient)
        
#---------------------------------------------------------------------------------------------#    
#    Draws obstacles received from the robot's sensors. Unstable, currently unused.           #
#---------------------------------------------------------------------------------------------#        
    def DrawObstacles(self, points, mode):
        if points is None:
            return
        
        if mode == 'inf': 
            fc = OBSTACLE_COLOR_2
            if self.obstacles_2 is not None: 
                try: 
                    self.Canvas.RemoveObject(self.obstacles_2)                 
                except ValueError:
                    self.obstacles_2 = None   
        else:
            fc = OBSTACLE_COLOR_1  
            if self.obstacles_1 is not None: 
                try:
                    self.Canvas.RemoveObject(self.obstacles_1) 
                except ValueError:
                    self.obstacles_1 = None   
                            
        obs = FloatCanvas.Group()
        try:
            for point in points:      
                    x = int( self.MetersToPixels((point.x,0))[0] )
                    y = int( self.MetersToPixels((point.y,0))[0] )                   
                    d = 4
                    p = FloatCanvas.Circle((x,y), d, FillColor=fc, LineColor=fc)
                    p.Coords = (x,y)
                    obs.AddObject(p)                    
            
            self.Canvas.AddObject(obs)  
            if mode == 'inf':
                self.obstacles_2 = obs 
            else:
                self.obstacles_1 = obs 
            self.ShowObstacles( self.modes['obstacles'] )
        except AttributeError:
            pass
            
    def ShowObstacles(self, boolean):
        try:
            self.obstacles_1.Visible = boolean
            self.obstacles_2.Visible = boolean
            self.Canvas.Draw(True)
        except (ValueError, AttributeError):
            pass
#---------------------------------------------------------------------------------------------#    
#    Event handler when the user clicks on the robot graphic.                                 #
#---------------------------------------------------------------------------------------------#    
    def OnClickRobot(self, obj):
        if self.modes['verbose']:
            print "Robot location:" 
            print "\t Pixels: (%s, %s)" % (int(obj.Coords[0]), int(obj.Coords[1]))
            m_Coords = self.PixelsToMeters(obj.Coords)
            print "\t Metric: (%s, %s)" % (m_Coords[0], m_Coords[1])
                        
#---------------------------------------------------------------------------------------------#    
#    Marks the robot's current goal node                                                      #
#---------------------------------------------------------------------------------------------#             
    def HighlightDestination(self, dest):    
        if self.curr_dest is not None and dest != self.curr_dest: 
            self.Canvas.RemoveObject(self.graphics_route[0])
            self.graphics_route.pop(0) 
            
            n1 = self.nodelist[self.curr_dest]
            n2 = self.nodelist[dest]
            e = int(self.conn_matrix[n1.id][n2.id])
            lw = EDGE_WIDTH
            lc = HIGHLIGHT_COLOR   
            l = self.Canvas.AddLine( (n1.coords,n2.coords), LineWidth=lw, LineColor=lc)    
            self.Canvas.Draw(True)
            self.highlights.append(l)
            self.curr_edge = e 
            if self.modes['verbose']: 
                print "Heading from %s to %s (edge %s)" % (self.curr_dest, dest, e)
        else:
            try:
                self.Canvas.RemoveObject(self.pe_graphic)
                self.pe_graphic = None
            except (ValueError, AttributeError):
                pass   
        self.curr_dest = dest     
            
#---------------------------------------------------------------------------------------------#    
#    Change the color of the last edge traveled to show that it has been visited.             #
#---------------------------------------------------------------------------------------------#
    def OnReachDestination(self): 
        try:                  
            self.Canvas.RemoveObject(self.ng_graphic)
            self.ng_graphic = None
        except (ValueError, AttributeError):
            pass         
                    
        if self.curr_edge is not None:
            edge = self.edgelist[self.curr_edge]
            coords1 = self.nodelist[int(edge.node1)].coords
            coords2 = self.nodelist[int(edge.node2)].coords
            lw = EDGE_WIDTH
            lc = DESTINATION_COLOR 
              
            l = self.Canvas.AddLine( (coords1,coords2), LineWidth=lw, LineColor=lc)
            self.highlights.append(l)               
            
        self.Canvas.Draw(True)

#---------------------------------------------------------------------------------------------#    
#    Displays the route created by node_traveller as a set of arrows. The color of each arrow #
#    depends on how close it is to the start or the end of the route. Red = Start, Blue = End #
#---------------------------------------------------------------------------------------------#        
    def DrawRoute(self, route, show):
        if route[0] == -1:
            print "End of tour."
            try:
                self.robot.Visible = True
                self.arrow.Visible = True
                
                self.route = []  
                self.curr_dest = None 
                self.Enable(True)
                try:
                    self.RestoreModes('Route') 
                except KeyError:
                    pass      
                      
            except IndexError:
                pass
            return
        
        if self.graphics_route != []:
            self.OnClear()
                
        self.route = route
        tmp_edges = []   
        
        sat = 255
        desat = 0     
        color = (desat, sat/2, sat)
        color_flt = (desat, sat/2, sat)
        
        steps = len(route)-2
        incr = min((2.25*sat)/steps, 30)
        phase = 0        
        self.Enable(False)
        
        for idx,node in reversed(list(enumerate(route))):            
            if (idx-1) >= 0:
                try:
                    n1_id = int(node)
                    n2_id = int(route[idx-1])
                    node1 = self.nodelist[n1_id]
                    node2 = self.nodelist[n2_id]
                except IndexError:
                    print n1_id, n2_id
            else:
                break      
            
            edge_id = int( self.conn_matrix[n1_id][n2_id] )
            
            x1 = node1.coords[0]
            y1 = node1.coords[1]
            x2 = node2.coords[0]
            y2 = node2.coords[1]  
            
            dx    = x2-x1
            dy    = y2-y1
            theta = math.atan2(dy,dx)
            kx    = math.cos(theta)
            ky    = math.sin(theta)
            
            xS = self.Round( x1+(( (NODE_DIAM+3)/2.0) *kx) )
            yS = self.Round( y1+(( (NODE_DIAM+3)/2.0) *ky) )
            xD = self.Round( x2-(( (NODE_DIAM+3)/2.0) *kx) )
            yD = self.Round( y2-(( (NODE_DIAM+3)/2.0) *ky) )            
            pS = (xS,yS)
            pD = (xD,yD)

            l = self.Canvas.AddArrowLine( (pD,pS), LineWidth = EDGE_WIDTH, LineColor = color,
                                     ArrowHeadSize=10, InForeground=True)
            self.graphics_route.insert(0,l)  
            tmp_edges.append(edge_id)
                        
            if phase == 0:
                color_flt =  ( color_flt[0], color_flt[1]-(2*incr), color_flt[2] )
                color =  ( color_flt[0], self.Round(color_flt[1]), color_flt[2] )
                
                if color[1] <= desat:
                    color_flt = (color_flt[0], desat, color_flt[2])
                    color = (color[0], desat, color[2])
                    phase = phase+1
                      
            if phase == 1:
                color_flt =  ( (color_flt[0]+incr), color_flt[1], color_flt[2] )
                color = ( self.Round(color_flt[0]), color_flt[1], color_flt[2] )
                
                if color[0] >= sat:
                    color_flt = (sat, color_flt[1], color_flt[2])
                    color = (sat, color[1], color[2])
                    phase = phase+1
            
            if phase == 2:        
                color_flt =  ( color_flt[0], color_flt[1], (color_flt[2]-incr) )
                color =  ( color_flt[0], color_flt[1], self.Round(color_flt[2]) )
                
                if color[2] <= desat:
                    color_flt = (color_flt[0], color_flt[1], desat)
                    color = (color[0], color[1], desat)
                    phase = phase+1
                    

        if not show:
            for gr in self.graphics_route:
                gr.Visible = True
        
        self.robot.Visible = False
        self.arrow.Visible = False
        self.Canvas.Draw(True)
        self.RefreshNodes()
        self.SetModes('Route', {'running':True})

#---------------------------------------------------------------------------------------------#    
#    Functions to display or hide the route created in DrawRoute()                            #
#---------------------------------------------------------------------------------------------#            
    def ShowRoute(self):        
#         for edge in self.graphics_edges:
#             edge.Visible = False    
        for gr in self.graphics_route:
            gr.Visible = True
        self.Canvas.Draw(True)
                
    def HideRoute(self):
        for edge in self.graphics_edges:
            edge.Visible = True    
        for gr in self.graphics_route:
            gr.Visible = False
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
        if collision < 0 or self.modes['load']:
                   
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
            self.nodelist.append(node)
            
            try:
                # Tell the connection matrix that this node now exists
                self.conn_matrix[int(ID)][int(ID)] = 0  
                if self.modes['verbose']:
                    print "Created node %s at (%s, %s) / metric: (%s, %s)" % \
                    (ID, node_coords[0], node_coords[1],
                     node.m_coords[0], node.m_coords[1])                      
                
                if self.modes['auto_erase']:    
                    self.DeselectAll(None)
                    md = self.MinDistanceToEdge(node)   
                    for entry in md:
                        edge = self.edgelist[ entry[0] ]
                        if self.modes['verbose']:
                            st = ("Auto-deleted edge %s between nodes "
                                  "%s and %s (too close to node %s)")
                            print st % (edge.id, edge.node1, edge.node2, node.id)
                        self.SelectOneEdge(self.graphics_edges[entry[0]], False)
                    self.DeleteSelection(None)                 
                
                if self.modes['auto_edges']:                    
                    self.ConnectNeighbors(node.id, self.gg_const['k'], self.gg_const['e'], True)
                     
                
            except IndexError:
                # Out of space in the connection matrix - expand it by 50 nodes
                curr_len = len(self.conn_matrix[0])
                self.conn_matrix.resize((curr_len+50, curr_len+50))
                
                # Tell the connection matrix that this node now exists
                self.conn_matrix[int(ID)][int(ID)] = 0 
                if self.modes['verbose']:
                    print "Created node %s at (%s, %s) / Metric = (%s, %s)" % (ID, 
                                                           node_coords[0], node_coords[1],
                                                           node.m_coords[0], node.m_coords[1])
                if self.modes['auto_edges']:
                    self.ConnectNeighbors(node.id, self.gg_const['k'], self.gg_const['e'], True)
            
            if self.modes['redraw']:
                self.Canvas.Draw(True)
            self.mp.SetSaveStatus(False) 
            return 1 
                  
        else:
            if self.modes['verbose']:
                print "Could not create node at (%s, %s). (Too close to node %s)" \
                        % (node_coords[0], node_coords[1], str(collision))
                      
            
            if self.modes['redraw']:
                a = self.Canvas.AddArc((node_coords[0]+(NODE_DIAM/2), node_coords[1]),
                    (node_coords[0]+(NODE_DIAM/2), node_coords[1]), node_coords,                                   
                    LineColor=ERROR_COLOR, LineWidth=NODE_BORDER_WIDTH+1, InForeground=True)
                self.Canvas.Draw(True)
                wx.Yield()
                  
                time.sleep(0.5)
                
                self.Canvas.RemoveObject(a)
                self.Canvas.Draw(True)
            return -collision
        

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
            if self.conn_matrix[0][0] == -1:
                self.GenerateConnectionMatrix() 
                        
            for j in range(len(points)-1): 
                
                try:
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
                        
                        if self.modes['auto_erase']:
                            md = self.MinDistanceToNode(edge) 
                        else:
                            md = None
                              
                        if md is not None:
                            if self.modes['verbose']:
                                st = ("Did not create edge between nodes "
                                      "%s and %s (too close to node %s)")
                                print st % (node1.Name, node2.Name, md[0])
                            
                            if self.modes['redraw']:
                                l = self.Canvas.AddLine((node1.Coords, node2.Coords),
                                    LineWidth=lw, LineColor=ERROR_COLOR, InForeground=True)
                                self.Canvas.Draw(True)
                                wx.Yield()
                                
                                if self.modes['redraw']:  
                                    time.sleep(0.5)
                                else:
                                    time.sleep(0.1)
    
                                self.Canvas.RemoveObject(l)
                                self.Canvas.Draw(True)    
                                
                            self.SelectOneEdge(self.graphics_edges[edge.id], True)
                            self.DeleteSelection(None)
                        else:
                            self.AddConnectionEntry(edge)                          
                            if self.modes['auto_intersections']:
                                self.ConvertIntersections(edge)                                                        
                            if self.modes['verbose']:
                                print "Created edge %s between nodes %s and %s" % (str(edge.id), 
                                                                                str(edge.node1),
                                                                                str(edge.node2))                             
                    else:
                        if self.modes['verbose']:
                            print "Did not create edge between nodes %s and %s (already exists)" \
                            % (node1.Name,node2.Name)
                            
                except IndexError:
                    pass

            if self.modes['redraw']:
                self.Canvas.Draw(True)
            self.DeselectAll(event)
            self.mp.SetSaveStatus(False)

#--------------------------------------------------------------------------------------------#    
#    -deprecated-                                                                            #
#--------------------------------------------------------------------------------------------#             
    def RefreshNodes(self):     
        old_robot, old_arrow = self.robot, self.arrow
        self.AddRobot(self.robot.Coords, self.arrow.Theta)
        self.Canvas.RemoveObject(old_robot)
        self.Canvas.RemoveObject(old_arrow)
        if self.modes['verbose']:
            print "Refreshed robot graphic."
             
#             for node in self.nodelist:               
#                 ID = str(node.id)
#                 xy = node.coords[0], node.coords[1]
#                 diam = NODE_DIAM
#                 lw = NODE_BORDER_WIDTH
#                 fc = NODE_FILL 
#                 
#                 if color_str == 'dest':
#                     lc = DESTINATION_COLOR  
#                 else:
#                     lc = NODE_BORDER
#                 if int(ID) < 100:  
#                     fs = FONT_SIZE_1
#                 else:
#                     fs = FONT_SIZE_2          
#                                     
#                 # Draw the node on the canvas
#                 c = self.Canvas.AddCircle(xy, diam, LineWidth=lw, LineColor=lc, FillColor=fc,
#                                           InForeground = True)
#                 c.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickNode)
#                 self.Canvas.RemoveObject( self.graphics_nodes[int(ID)] )
#                 self.graphics_nodes[int(ID)] = c                     
#                 c.Name = ID
#                 c.Coords = node.coords                  
#                 
#                 t = self.Canvas.AddScaledText(ID, xy, Size=fs, Position="cc",Color=lc, 
#                                               Weight=wx.BOLD, InForeground = True)
#                 self.Canvas.RemoveObject( self.graphics_text[int(ID)] )
#                 self.graphics_text[int(ID)] = t
                

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
        
        if self.modes['spaced_edges']:
            while not done:
                x1 = int( x1_1+(pos*kx) )
                y1 = int( y1_1+(pos*ky) )
                x2 = int( x1_2+(pos*kx) )
                y2 = int( y1_2+(pos*ky) )                 
                data1 = image_data[ (w*y1)+x1 ]
                data2 = image_data[ (w*y2)+x2 ]               
                
                if self.image_data_format is 'int':
                    if not self.modes['unknown_edges']:
                        if data1 != 0 or data2 != 0:
                            return False      
                    else:
                        if data1 > 0 or data2 > 0:
                            return False 
                                 
                if self.image_data_format is 'byte':                    
                    if not self.modes['unknown_edges']:
                        if ord(data1) < 150 or ord(data2) < 150:
                            return False
                    else:
                        if ord(data1) < 50 or ord(data2) < 50:
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
                data = image_data[ (w*y)+x ]    
                
                if self.image_data_format is 'int':
                    if not self.modes['unknown_edges']:
                        if data != 0:
                            return False      
                    else:
                        if data > 0:
                            return False 
                                       
                if self.image_data_format is 'byte':
                    if not self.modes['unknown_edges']:
                        if ord(data) < 150:
                            return False
                    else:
                        if ord(data) < 50:
                            return False
                
                if not last and pos>ln:
                    last = True
                    pos = ln
                elif last:
                    done = True                    
                pos += increment
        return True

    def Perp(self, a) :
        b = np.empty_like(a)
        b[0] = -a[1]
        b[1] = a[0]
        return b

#--------------------------------------------------------------------------------------------#    
#      Returns the location of intersections along a given edge 'e1'                         #
#--------------------------------------------------------------------------------------------#     
    def FindIntersections(self, e1) :
        e1_x1 = float(self.nodelist[ int(e1.node1) ].coords[0])
        e1_y1 = float(self.nodelist[ int(e1.node1) ].coords[1])
        e1_x2 = float(self.nodelist[ int(e1.node2) ].coords[0])
        e1_y2 = float(self.nodelist[ int(e1.node2) ].coords[1]) 
        a1 = np.array([e1_x1, e1_y1])
        a2 = np.array([e1_x2, e1_y2])        
        intersections = {}
        e1_interval = ( min(e1_x1, e1_x2), max(e1_x1, e1_x2))
        
        for e2 in self.edgelist:
            if e2 == e1:
                continue
            if (e1.node1 == e2.node1 or e1.node1 == e2.node2 or 
                e1.node2 == e2.node2 or e1.node2 == e2.node1):
                continue
            
            e2_x1 = float(self.nodelist[ int(e2.node1) ].coords[0])
            e2_y1 = float(self.nodelist[ int(e2.node1) ].coords[1])            
            e2_x2 = float(self.nodelist[ int(e2.node2) ].coords[0])
            e2_y2 = float(self.nodelist[ int(e2.node2) ].coords[1])            
            e2_interval = ( min(e2_x1, e2_x2), max(e2_x1, e2_x2))
            
            b1 = np.array([e2_x1, e2_y1])
            b2 = np.array([e2_x2, e2_y2])
            
            da = a2-a1
            db = b2-b1
            dp = a1-b1
            dap = self.Perp(da)
            denom = np.dot( dap, db )
            num = np.dot( dap, dp )
            if denom == 0:
                # The edges are parallel (no intersect)
                continue
            
            result = (num / denom)*db + b1
            if (result[0]>e1_interval[0] and result[0]<e1_interval[1] and
                result[0]>e2_interval[0] and result[0]<e2_interval[1]):
                intersections[e2.id] = result     
                break 
        return intersections
    
#--------------------------------------------------------------------------------------------#    
#      Converts edge intersections into new nodes, if possible                               #
#--------------------------------------------------------------------------------------------#     
    def ConvertIntersections(self, e1):
        self.SetModes('ConvertIntersections', {
                        'redraw':False, 
#                         'verbose':False, 
                        'auto_edges':False
#                         'auto_intersections':False
                        })
        
        nodes = []
        nodes.append( int(e1.node1) )
        nodes.append( int(e1.node2) )     
        intersections = self.FindIntersections(e1)
        
        ok_to_proceed = False
        while not ok_to_proceed:
            if len(intersections) > 0:
                for key,val in intersections.iteritems():
                    e2 = self.edgelist[key]
                    nodes.append( int(e2.node1) )
                    nodes.append( int(e2.node2) )
                    x = int(val[0])
                    y = int(val[1])           
                    
                    self.SetModes('ConvertIntersections2', {
                                    'auto_erase':False, 
                                    })
                    result = self.CreateNode((x,y))
                    self.RestoreModes('ConvertIntersections2')
                    if result <= 0:
                        # if the edge intersection is too close to a node, get rid of the edge.
                        if int(e1.node1) == -result or int(e1.node2) == -result:
                            self.SelectOneEdge(self.graphics_edges[e2.id], True)
                            self.DeleteSelection(None)
                            if self.modes['verbose']:
                                st = ("Auto-deleted edge %s between nodes %s and %s "
                                      "(too close to node %s)")
                                print  st % (e2.id, e2.node1, e2.node2, -result)
                                
                            if self.modes['redraw']:
                                l = self.Canvas.AddLine(
                                    (self.nodelist[int(e2.node1)].coords, 
                                     self.nodelist[int(e2.node2)].coords),
                                     LineWidth=EDGE_WIDTH, LineColor=ERROR_COLOR, 
                                     InForeground=True)
                                self.Canvas.Draw(True)
                                wx.Yield() 
                                time.sleep(0.5)
                                self.Canvas.RemoveObject(l)
                                self.Canvas.Draw(True) 
                                    
                            intersections = self.FindIntersections(e1)
                        else:
                            self.SelectOneEdge(self.graphics_edges[e1.id], True)
                            self.DeleteSelection(None)
                            ok_to_proceed = True
                        break
                    
                    new_node = len(self.nodelist)-1   # the new node
                    self.SelectOneEdge(self.graphics_edges[e1.id], True)
                    self.SelectOneEdge(self.graphics_edges[e2.id], False)
                    self.DeleteSelection(None)
                    
                    for existing_node in nodes:
                        self.SelectOneNode(self.graphics_nodes[new_node], True)
                        self.SelectOneNode(self.graphics_nodes[existing_node], False)
                        self.CreateEdges(None)
                        
#                     self.ConnectNeighbors(new_node, self.gg_const['k'], self.gg_const['e'], True)
                    ok_to_proceed = True
            else:
                ok_to_proceed = True
        
        # Restore saved states        
        self.DeselectAll(None) 
        self.RestoreModes('ConvertIntersections')
        
#--------------------------------------------------------------------------------------------#    
#      Given an edge, returns None if it is not too close to any nodes. If it is too close   #
#      to a node, the Node ID and distance to the node are returned.                         #
#--------------------------------------------------------------------------------------------#     
    def MinDistanceToNode(self, edge):
        min_distance = -1, max(self.gg_const['d']/2.0, 5)
        ex1 = float(self.nodelist[ int(edge.node1) ].coords[0])   
        ey1 = float(self.nodelist[ int(edge.node1) ].coords[1]) 
        ex2 = float(self.nodelist[ int(edge.node2) ].coords[0]) 
        ey2 = float(self.nodelist[ int(edge.node2) ].coords[1])        
        ln = self.Distance2((ex1, ey1), (ex2,ey2))   
        
        for node in self.nodelist:
            if node.id == int(edge.node1) or node.id == int(edge.node2):
                continue
            
            nx = float(node.coords[0])
            ny = float(node.coords[1])
            t = ( (nx-ex1)*(ex2-ex1) + (ny-ey1)*(ey2-ey1) ) / ln
            if t<0:
                dist = math.sqrt( self.Distance2(node.coords, (ex1, ey1)) )
                if dist < min_distance[1]:
                    min_distance = node.id, dist
                    break
            elif t>1:
                dist = math.sqrt( self.Distance2(node.coords, (ex2, ey2)) )
                if dist < min_distance[1]:
                    min_distance = node.id, dist
                    break
            else:
                dist = math.sqrt( self.Distance2(node.coords, ( ex1+t*(ex2-ex1), ey1+t*(ey2-ey1) )) )
                if dist < min_distance[1]:
                    min_distance = node.id, dist
                    break
        if min_distance[0] != -1:
            return min_distance
        else:
            return None
        
#--------------------------------------------------------------------------------------------#    
#      Given a node, returns any edges which are too close                                   #
#--------------------------------------------------------------------------------------------#    
    def MinDistanceToEdge(self, node):
        thresh = max(self.gg_const['d']/2.0, 5)
        min_distance = []
        nx = float(node.coords[0])
        ny = float(node.coords[1])           
        
        for edge in self.edgelist:
            if node.id == int(edge.node1) or node.id == int(edge.node2):
                continue
            
            ex1 = float(self.nodelist[ int(edge.node1) ].coords[0])   
            ey1 = float(self.nodelist[ int(edge.node1) ].coords[1]) 
            ex2 = float(self.nodelist[ int(edge.node2) ].coords[0]) 
            ey2 = float(self.nodelist[ int(edge.node2) ].coords[1])
            ln = self.Distance2((ex1, ey1), (ex2,ey2))
            
            t = ( (nx-ex1)*(ex2-ex1) + (ny-ey1)*(ey2-ey1) ) / ln
            if t<0:
                dist = math.sqrt( self.Distance2(node.coords, (ex1, ey1)) )
                if dist < thresh:
                    min_distance.append( (edge.id, dist) )
                continue
            elif t>1:
                dist = math.sqrt( self.Distance2(node.coords, (ex2, ey2)) )
                if dist < thresh:
                    min_distance.append( (edge.id, dist) )
                continue
            else:
                dist = math.sqrt( self.Distance2(node.coords, ( ex1+t*(ex2-ex1), ey1+t*(ey2-ey1) )) )
                if dist < thresh:
                    min_distance.append( (edge.id, dist) )
                continue
        return min_distance
            

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
        self.ttime = datetime(100,1,1,0,0,0) 
        self.SetModes('GenerateGraph', {
                        'auto_edges':False, 
                        'redraw':False, 
#                         'verbose':False
                        })
        
        if self.modes['clear_graph']:
            self.SelectAll(None)
            self.DeleteSelection(None)        
        
        data = self.image_data
        lim = self.FindImageLimit(data,4)        
        b = lim[0]
        t = lim[1]
        l = lim[2]
        r = lim[3]
                
        while len(self.nodelist) < n:
            x = int( l + ((r-l)*rand.random()) )
            y = int( b + ((t-b)*rand.random()) )
            
            if self.CheckNodeLocation(data, (x,y), w, d):
                if int(self.CreateNode( (x,y) )) == 1: # Node successfully created                  
                    self.ConnectNeighbors(len(self.nodelist)-1, k, e, False)
                       
        self.Canvas.Draw(True) 
        
        # Restore saved states
        wx.EndBusyCursor()
        et = datetime.now()
        self.RestoreModes('GenerateGraph')
        if self.modes['verbose']:
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
            self.ConnectNeighbors( int(node.Name), self.gg_const['k'], self.gg_const['e'], True)
        self.DeselectAll(event)
    
#--------------------------------------------------------------------------------------------#
#     Tries to connect edges from a given 'input_node' to its closest neighbors              #
#                                                                                            #
#     k: The number of neighbors to analyze                                                  #
#     e: Maximum edge length (furthest distance to search for neighbors)                     #
#--------------------------------------------------------------------------------------------#    
    def ConnectNeighbors(self, input_node, k, e, refresh): 
        if self.modes['verbose']:
            print "Connecting neighbors for node %s" % input_node
        self.SetModes('ConnectNeighbors', {
                        'redraw':False,
                        'auto_edges':False
                        })
        
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
                                      node2.coords,1):    
                self.DeselectAll(None)                     
                self.SelectOneNode(self.graphics_nodes[node1.id], False)
                self.SelectOneNode(self.graphics_nodes[node2.id], False)
                self.CreateEdges(None)
                
        self.RestoreModes('ConnectNeighbors')

#--------------------------------------------------------------------------------------------#    
#      Deletes all selected nodes and edges                                                  #
#--------------------------------------------------------------------------------------------#           
    def DeleteSelection(self, event):
        self.SetModes('DeleteSelection', {
                        'redraw':False, 
                        'verbose':False
                        })
        
        #Mode check to prevent errors if we're in the edge creation tool
        current_mode = self.Canvas.GetMode()
        if current_mode == 'GUIEdges':
            if self.Canvas.GUIMode.start_node is not None:
                self.started_edge = False
                self.Canvas.GUIMode.start_node = None
                self.Canvas.GUIMode.start_coords = None
                self.Canvas.GUIMode.EraseCurrentEdge()
                self.Canvas.Draw(True)
              
        for node in self.sel_nodes:
            self.RemoveNode(node)
        for edge in self.sel_edges:
            self.RemoveEdge(edge)
        
        self.RenumberNodes()  
        self.RenumberEdges()
        self.GenerateConnectionMatrix()        
        
        self.DeselectAll(event=None)
        self.mp.SetSaveStatus(False)
        self.RestoreModes('DeleteSelection')
        
        if self.modes['redraw']:
            self.Canvas.Draw(True)

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
        
        if self.modes['verbose']:
            print "Removed node #" + str(ID)
        if self.modes['redraw']:
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
            if self.modes['redraw']:
                self.Canvas.Draw(True)          
            
            if self.modes['verbose']:
                print "Removed edge #" + str(ID)
            
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
                graphics[j].Name = str(j)
                if j < 100:  
                    fs = FONT_SIZE_1
                else:
                    fs = FONT_SIZE_2
                
                # Make the old text invisible and replace it in the data structure
                xy = (nodes[j].coords[0], nodes[j].coords[1])            
                self.Canvas.RemoveObject(text[j])                
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

#---------------------------------------------------------------------------------------------#    
#    Adds a single edge into the connection matrix.                                           #
#---------------------------------------------------------------------------------------------#
    def AddConnectionEntry(self, edge):
        self.conn_matrix[ int(edge.node1) ][ int(edge.node2) ] = edge.id
        self.conn_matrix[ int(edge.node2) ][ int(edge.node1) ] = edge.id
        
#--------------------------------------------------------------------------------------------#    
#     For debugging purposes. Writes the connection matrix to a text file.                   #
#--------------------------------------------------------------------------------------------#    
    def ExportConnectionMatrix(self, filename, edgeitems, linewidth, string):         
        np.set_printoptions(edgeitems=edgeitems, linewidth=linewidth)  
              
        if not self.modes['export']:
            conn_file = open(filename, "w")
            self.SetModes('Export', {'export':True})
        else:
            conn_file = open(filename, "a")        
        
        L = len(self.nodelist)   
        export_mtx = self.conn_matrix[0:L,0:L]       
        
        conn_file.write("%s:\n" % string)    
        conn_file.write(str( export_mtx ))
        conn_file.write("\n\n")
        conn_file.close()            
            

#--------------------------------------------------------------------------------------------#    
#     Returns -1 if there is no collision with another node. If there is a collision,        #
#     returns the ID of the closest node there was a collision with.                         #
#--------------------------------------------------------------------------------------------#    
    def DetectCollision(self, new_node):
        if self.modes['manual_edges']:
            min_dist = max(self.gg_const['d']/2.0, 5)
        else:
            min_dist = max(self.gg_const['d'], 5)
        
        collisions = {} 
        for existing_node in self.nodelist:
            try:
                dist = self.Distance(new_node.coords, existing_node.coords)
                if dist <= min_dist:
                    collisions[existing_node.id] = dist
            except AttributeError:
                pass
        if len(collisions) == 0:
            return -1  
        else:
            closest = min(collisions, key=collisions.get)
            return closest
        
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
#     Same as self.Distance(), but without the sqrt                                          #
#--------------------------------------------------------------------------------------------# 
    def Distance2(self, p1, p2): 
        x1 = float(p1[0])
        x2 = float(p2[0])
        y1 = float(p1[1])
        y2 = float(p2[1])
        dist = ( (x2 - x1)**2 + (y2 - y1)**2 )  
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

#--------------------------------------------------------------------------------------------#    
#     Functions to convert between pixel coordinates and real-world metric coordinates       #
#--------------------------------------------------------------------------------------------#    
    def PixelsToMeters(self, xy_p):
        x = (xy_p[0] * self.resolution) + self.origin.x
        y = (xy_p[1] * self.resolution) + self.origin.y
        return (x,y)    
    def MetersToPixels(self, xy_m):
        x = (xy_m[0] - self.origin.x) / self.resolution
        y = (xy_m[1] - self.origin.y) / self.resolution
        return (x,y)

#--------------------------------------------------------------------------------------------#    
#     Truncates a floating point number 'f' to 'n' decimal places                            #
#--------------------------------------------------------------------------------------------#    
    def Truncate(self, f, n):
        return ('%.*f' % (n + 1, f))[:-1]     

#---------------------------------------------------------------------------------------------#    
#     Rounds a floating point number to the nearest integer                                   #
#---------------------------------------------------------------------------------------------#
    def Round(self, flt):
        if flt % 1 >= 0.5:
            return int(flt)+1
        else:
            return int(flt)
    
#--------------------------------------------------------------------------------------------#    
#     Selects a single node. If desel is True, deselects everything else.                    #
#--------------------------------------------------------------------------------------------#   
    def SelectOneNode(self, obj, desel):
        if desel:      
            self.DeselectAll(event=None)      
        self.sel_nodes.append(obj) 
        obj.SetFillColor(SELECT_COLOR)
        if self.modes['redraw']:
            self.Canvas.Draw(True)
        
#--------------------------------------------------------------------------------------------#    
#     Selects a single edge. If desel is True, deselects everything else.                    #
#--------------------------------------------------------------------------------------------#   
    def SelectOneEdge(self, obj, desel):
        if desel:     
            self.DeselectAll(event=None)    
        self.sel_edges.append(obj)
        obj.SetLineColor(SELECT_COLOR)
        if self.modes['redraw']:
            self.Canvas.Draw(True) 
            
#--------------------------------------------------------------------------------------------#    
#     Selects all nodes and deselects everything else.                                       #
#--------------------------------------------------------------------------------------------#        
    def SelectNodes(self, event):
        self.DeselectAll(event)
        
        # Set highlighted colour
        for node in self.nodelist:
            self.graphics_nodes[ node.id ].SetFillColor(SELECT_COLOR)
            self.sel_nodes.append(self.graphics_nodes[node.id])                
        if self.modes['redraw']:    
            self.Canvas.Draw(True) 
                
        if self.modes['verbose']:
                print "Selected all nodes"
        
#--------------------------------------------------------------------------------------------#    
#     Selects all edges and deselects everything else.                                       #
#--------------------------------------------------------------------------------------------#    
    def SelectEdges(self, event):
        self.DeselectAll(event)
            
        # Set highlighted colour
        for edge in self.edgelist:
            self.graphics_edges[ edge.id ].SetLineColor(SELECT_COLOR)
            self.sel_edges.append(self.graphics_edges[edge.id])
        if self.modes['redraw']:    
            self.Canvas.Draw(True)
                
        if self.modes['verbose']: 
            print "Selected all edges"


#--------------------------------------------------------------------------------------------#
#     Selects all nodes/edges located within a given X and Y range. This is used with the    #
#     'box selection' tool on the NavCanvas.                                                   #
#--------------------------------------------------------------------------------------------#           
    def SelectBox(self, x_range, y_range):
        self.SetModes('SelectBox', {
                        'redraw':False
                        })
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
        
        self.RestoreModes('SelectBox')
        self.Canvas.Draw(True)
        
#--------------------------------------------------------------------------------------------#    
#     Select/deselect all nodes and edges                                                    #
#--------------------------------------------------------------------------------------------#             
    def SelectAll(self, event):                    
        self.DeselectAll(event)   
          
        for node in self.nodelist:                
            if event is not None:   
                self.graphics_nodes[ node.id ].SetFillColor(SELECT_COLOR)
            self.sel_nodes.append(self.graphics_nodes[node.id])
        for edge in self.edgelist:
            if event is not None:   
                self.graphics_edges[ edge.id ].SetLineColor(SELECT_COLOR)
            self.sel_edges.append(self.graphics_edges[edge.id])            
        if self.modes['redraw']:    
            self.Canvas.Draw(True) 
        
        if self.modes['verbose']:            
            print "Selected all nodes and edges"
            
    def DeselectAll(self, event):
        for obj in self.sel_nodes:
            obj.SetFillColor(NODE_FILL)
        for obj in self.sel_edges:
            obj.SetLineColor(EDGE_COLOR) 
        if self.modes['redraw']:            
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
            if self.modes['verbose']:        
                print "Deselected Node %s  (%s, %s)" % (obj.Name, coords[0], coords[1])
            self.sel_nodes.remove(obj)
            obj.SetFillColor(NODE_FILL)
            self.Canvas.Draw(True)
        else:  
            coords = self.nodelist[int(obj.Name)].coords   
            m_coords = self.nodelist[int(obj.Name)].m_coords 
            if self.modes['verbose']:        
                print "Selected Node %s" % (obj.Name)   
                print "\tPixel Location:   (%s, %s)" % (coords[0]  , coords[1]  )
                print "\tMetric Location:  (%s, %s)" % (m_coords[0], m_coords[1])
            self.sel_nodes.append(obj)   
            obj.SetFillColor(SELECT_COLOR) 
            self.Canvas.Draw(True)
        
        current_mode = self.Canvas.GetMode()  
        if current_mode == 'GUIEdges':
            if not self.started_edge:
                self.Canvas.GUIMode.SetStartNode(obj.Name, self.nodelist[int(obj.Name)].coords)
                self.started_edge = True
            else:
                self.Canvas.GUIMode.SetEndNode(obj.Name)
                self.started_edge = False

#--------------------------------------------------------------------------------------------#    
#     Event handlers to change the mouse cursor when hovering over objects                   #
#--------------------------------------------------------------------------------------------#    
    def OnMouseEnterNode(self, obj):
        if self.modes['running']:
            return
        
        current_mode = self.Canvas.GetMode() 
        if current_mode == 'GUIEdges':
#             self.q.put( (self.Canvas.GUIMode.LockEdge,
#                          'e_node', obj.Name, self.nodelist[int(obj.Name)].coords) )
            self.Canvas.GUIMode.LockEdge('e_node', obj.Name, self.nodelist[int(obj.Name)].coords)  
        self.Canvas.GUIMode.SwitchCursor('enter')  
#         self.q.put( (self.Canvas.GUIMode.SwitchCursor, 'enter') )
            
    def OnMouseLeaveNode(self, obj):  
        if self.modes['running']:
            return
        
        current_mode = self.Canvas.GetMode() 
        if current_mode == 'GUIEdges':
#             self.q.put( (self.Canvas.GUIMode.LockEdge,
#                          'l_node', obj.Name, self.nodelist[int(obj.Name)].coords) )
            self.Canvas.GUIMode.LockEdge('l_node', obj.Name, self.nodelist[int(obj.Name)].coords) 
        self.Canvas.GUIMode.SwitchCursor('leave')  
#         self.q.put( (self.Canvas.GUIMode.SwitchCursor, 'leave') )
               
    def OnMouseEnterEdge(self, obj):
        if self.modes['running']:
            return
        
        current_mode = self.Canvas.GetMode() 
        if current_mode == 'GUIEdges':
#             self.q.put( (self.Canvas.GUIMode.LockEdge,
#                          'e_edge', obj.Name, self.nodelist[int(obj.Name)].coords) )
            self.Canvas.GUIMode.LockEdge('e_edge', -1, -1)
        self.Canvas.GUIMode.SwitchCursor('enter')  
#         self.q.put( (self.Canvas.GUIMode.SwitchCursor, 'enter') )
                  
    def OnMouseLeaveEdge(self, obj):
        if self.modes['running']:
            return
        
        current_mode = self.Canvas.GetMode() 
        if current_mode == 'GUIEdges':
#             self.q.put( (self.Canvas.GUIMode.LockEdge,
#                          'l_edge', obj.Name, self.nodelist[int(obj.Name)].coords) )
            self.Canvas.GUIMode.LockEdge('l_edge', -1, -1) 
        self.Canvas.GUIMode.SwitchCursor('leave') 
#         self.q.put( (self.Canvas.GUIMode.SwitchCursor, 'leave') )
        
#--------------------------------------------------------------------------------------------#    
#     Event when an edge is left-clicked.                                                    #
#     Adds the node to the selection list and highlights it on the canvas                    #
#--------------------------------------------------------------------------------------------#            
    def OnClickEdge(self, obj): 
        if obj in self.sel_edges:  
            if self.modes['verbose']:        
                print "Deselected Edge " + obj.Name
            self.sel_edges.remove(obj)
            obj.SetLineColor(EDGE_COLOR)
            self.Canvas.Draw(True) 
        else: 
            if self.modes['verbose']:       
                print "Selected Edge " + obj.Name     
            self.sel_edges.append(obj)   
            obj.SetLineColor(SELECT_COLOR) 
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
        metadata = [self.image_width, self.resolution, self.origin]
        g = [self.nodelist, self.edgelist, metadata]
        pickle.dump(g,f)

#--------------------------------------------------------------------------------------------#    
#     Unpickles an existing graph file from the file system.                                 #
#     NodeList is in slot 0, EdgeList is in slot 1, Metadata is in slot 2.                   #
#--------------------------------------------------------------------------------------------#       
    def ImportGraph(self, f):
        if f is not None:
            g = pickle.load(f)
            self.SetNodeList( g[0] )   
            self.SetEdgeList( g[1] ) 
            self.SetMapMetadata( *g[2] )
        else:
            ori = gs.Origin((0,0))
            self.SetMapMetadata(None, 0.05, ori)

#--------------------------------------------------------------------------------------------#    
#     Iterates through an imported node list and creates the nodes.                          #
#     This function should not be called on its own, but rather as a part of SetImage()      #
#--------------------------------------------------------------------------------------------#                   
    def LoadNodes(self):
        self.SetModes('LoadNodes', {
                        'auto_erase':False, 
                        'auto_intersections':False, 
                        'auto_edges':False,
                        'load':True
                        })
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
            try:
                self.sel_nodes.append( self.graphics_nodes[ int(edge.node1) ] )
                self.sel_nodes.append( self.graphics_nodes[ int(edge.node2) ] )
                self.CreateEdges(event=None)   
            except IndexError:
                print edge.node1, edge.node2, len(self.graphics_nodes)           
        
        self.RestoreModes('LoadNodes')
              
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
            for i in range(w/d):
                for j in range(w/d):
                    try:
                        if image_data[ (w*i*d)+(j*d) ] >= 0 and not foundB:
                            bot = i*d
                            foundB = True
                            
                        if image_data[ w-((w*i*d)+(j*d)) ] >= 0 and not foundT:
                            top = w-(i*d)
                            foundT = True
                            
                        if image_data[ (w*j*d)+(i*d) ] >= 0 and not foundL:
                            left = i*d
                            foundL = True
                            
                        if image_data[ w-((w*j*d)+(i*d)) ] >= 0 and not foundR:
                            right = w-(i*d)
                            foundR = True
                            
                        if (foundB and foundT and
                            foundL and foundR):
                            break                
                    except IndexError:
                        break
                if (foundB and foundT and
                    foundL and foundR):
                    break     
                      
        elif self.image_data_format is 'byte':
            for i in range(w/d):
                for j in range(w/d):
                    try:
                        if image_data[ (w*i*d)+(j*d) ] != chr(100) and not foundB:
                            bot = i*d
                            foundB = True
                            
                        if image_data[ w**2-((w*i*d)+(j*d)) ] != chr(100) and not foundT:
                            top = w-(i*d)
                            foundT = True
                            
                        if image_data[ (w*j*d)+(i*d) ] != chr(100) and not foundL:
                            left = i*d
                            foundL = True
                            
                        if image_data[ w**2-((w*j*d)+(i*d)) ] != chr(100) and not foundR:
                            right = w-(i*d)
                            foundR = True
                            
                        if (foundB and foundT and
                            foundL and foundR):
                            break                
                    except IndexError:
                        break
                if (foundB and foundT and
                    foundL and foundR):
                    break       
        et = datetime.now()        
        if self.modes['verbose']:
            print "Top edge of map at row %s" % (str(top))
            print "Bottom edge of map at row %s" % (str(bot)) 
            print "Left edge of map at column %s" % (str(left))   
            print "Right edge of map at column %s" % (str(right))              
            print "Scanned map data. Time taken: %s" % str(et-st)     
                       
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
#     Zooms to the edges of the currently displayed map                                      #
#--------------------------------------------------------------------------------------------#        
    def ZoomToFit(self):
        if self.update_imglimits:
            data_ready = False
            while not data_ready: 
                try:
                    self.img_limits = self.FindImageLimit(self.image_data, 10) 
                    self.update_imglimits = False
                    data_ready = True
                except AttributeError:
                    # Sleep until the image data can be obtained
                    time.sleep(0.5)
            
        tl = (self.img_limits[2],self.img_limits[1])
        br = (self.img_limits[3],self.img_limits[0]) 
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
          
#---------------------------------------------------------------------------------------------#    
#    Saves the current modes and changes them.                                                #
#                                                                                             #
#    key: The current modes are saved as a dictionary entry. This is the key to that entry.   #
#    state_dict: Input dictionary containing the modes to be changes and their new values.    #
#---------------------------------------------------------------------------------------------#    
    def SetModes(self, key, state_dict):
        self.saved_modes[key] = (self.modes.copy())  
        for key,val in state_dict.iteritems():
            self.modes[key] = val

#---------------------------------------------------------------------------------------------#    
#    Restores the saved modes located at index 'key'                                          #
#---------------------------------------------------------------------------------------------#            
    def RestoreModes(self, key):
        self.modes = self.saved_modes[key].copy()
    
#--------------------------------------------------------------------------------------------#    
#     Clears the entire canvas (including the map)                                           #
#--------------------------------------------------------------------------------------------#   
    def Clear(self):
        self.Canvas.InitAll()  

#---------------------------------------------------------------------------------------------#    
#    Erases route graphics if they exist. If not, erases all nodes and edges from the map.    #
#---------------------------------------------------------------------------------------------#        
    def OnClear(self):
        if self.highlights != [] or self.graphics_route != []:
            for obj in self.highlights:
                self.Canvas.RemoveObject(obj)
            self.highlights = []
            
            for obj in self.graphics_route:
                self.Canvas.RemoveObject(obj)
            self.graphics_route = []
            
            for obj in self.graphics_edges:
                obj.Visible = True                
            self.Canvas.Draw(True)
#                 self.RefreshNodes('normal')
            self.mp.ep.btn_rte.SetLabel('Hide Route')
            self.mp.ep.btn_rte.Enable(False)
#             self.mp.ep.btn_stop.Enable(False)
        else:
            self.ClearGraph()

#---------------------------------------------------------------------------------------------#    
#    Erases all nodes and edges from the map                                                  #
#---------------------------------------------------------------------------------------------#                   
    def ClearGraph(self):
        self.SetModes('ClearGraph', {                        
                        'verbose':False, 
                        }) 
        self.SelectAll(None)
        self.DeleteSelection(None)  
        self.RestoreModes('ClearGraph')    

#---------------------------------------------------------------------------------------------#    
#    Saves the content of the canvas as a .png image                                          #
#---------------------------------------------------------------------------------------------#    
    def SaveCanvasImage(self, filename):
        # For some reason FloatCanvas doesn't save foreground objects
        # So, we need to draw some temporary nodes on the background
        st = datetime.now()
        temp_obj = []
        
        for node in self.nodelist:
            xy = node.coords[0], node.coords[1]
            diam = NODE_DIAM
            lw = NODE_BORDER_WIDTH
            lc = NODE_BORDER    
            fc = NODE_FILL
            if node.id < 100:  
                fs = FONT_SIZE_1
            else:
                fs = FONT_SIZE_2     
                    
            c = self.Canvas.AddCircle(xy, diam, LineWidth=lw, LineColor=lc, FillColor=fc,
                                      InForeground = False)        
            t = self.Canvas.AddScaledText(str(node.id), xy, Size=fs, Position="cc", 
                                    Color=TEXT_COLOR, Weight=wx.BOLD, InForeground = False)
            temp_obj.append(c)    
            temp_obj.append(t)
        
        self.Canvas.Draw(True)
        self.Canvas.SaveAsImage(filename)
        self.Canvas.RemoveObjects(temp_obj) # Get rid of the temporary nodes
        self.Canvas.Draw(True)
        
        et = datetime.now()
        print "Saved canvas image. Time taken: %s" % (et-st)

#---------------------------------------------------------------------------------------------#    
#    (-Debug-)                                                                                #
#---------------------------------------------------------------------------------------------#           
    def Test(self):  
#         self.DrawObstacles(self.ros.obstacles)
#         print t.activeCount()
        route = [29,30,3,48,50,4,0,46,49,29,30,3,48,50,4,0,46,49,29,30,3,48,50,4,0,46,49,29,
                 62,56,1,55,56,62,30,3,48,50,25,35,11,27,59,2,47,37,28,57,5,40,24]
        tt = TimerThread(self, 1)
        tt.start()
    
        self.DrawRoute(route, True)
        self.GetParent().ep.btn_rte.Enable(True)
#         self.GetParent().ep.btn_stop.Enable(True)
                   
        for n in route:
            self.HighlightDestination(n)
            time.sleep(0.1)
            wx.Yield()
            self.OnReachDestination()
            self.OnReachDestination()
            time.sleep(0.1)
            wx.Yield()
#         
        self.DrawRoute([-1], True)
         
        tt.stopped = True
        tt.join()

#     def DrawTest(self):
#         print "MF %s" % t.current_thread()

    
#--------------------------------------------------------------------------------------------#    
#     Sets the image to display on the canvas. If the map has an associated graph file,      #
#     the corresponding nodes and edges are loaded and drawn onto the canvas.                #
#--------------------------------------------------------------------------------------------#
    def SetImage(self, image_obj):
        self.SetModes('SetImage', {                        
                        'verbose':False, 
                        'redraw':False, 
                        'auto_edges':False
                        }) 
        self.Clear()
        st = datetime.now()    
                 
        try:
            # Creates the image from a file (used when loading a .png map file)
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
        
        self.image_width = image.GetHeight() # Case where metadata was not set
        self.update_imglimits = True
        
        self.img = self.Canvas.AddScaledBitmap( image, 
                                  (0,0), 
                                  Height=image.GetHeight(), 
                                  Position = 'bl')    
        self.LoadNodes()
        self.LoadEdges()
        self.GenerateConnectionMatrix()
         
        if self.robot is None:        
            self.AddRobot(-1,-1) 
        else:
            self.AddRobot(self.robot.Coords, self.arrow.Theta)  
             
        self.SetCurrentMapPath(image_file)
        self.Show() 
        self.Layout()        
        self.ZoomToFit()
        
        et = datetime.now()
        self.RestoreModes('SetImage')

        if self.modes['verbose']:
            print "Set map image. Time taken: %s" % str(et-st)
