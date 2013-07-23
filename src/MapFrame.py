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
from datetime import datetime

#----- Global colors -----#
NODE_FILL           = (240,240,240)
NODE_BORDER         = (119,41,83)
EDGE_COLOR          = (119,41,83)
ROBOT_FILL_1        = (100,100,100)
ROBOT_FILL_2        = (180,0,0)
ROBOT_BORDER        = (50,0,0)
SELECT_COLOR        = (255,106,54)
HIGHLIGHT_COLOR     = (220,20,220)
DESTINATION_COLOR   = (30,165,40)
OBSTACLE_COLOR      = (240,30,30)
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
        self.modes = {'export':False, 'auto_erase':True, 'verbose':True, 'redraw':True, 
                       'auto_edges':True, 'spaced_edges':True,  'unknown_edges':True, 
                       'clear_graph':True, 'auto_intersections':True, 'manual_edges':False,
                       'pose_est':False, 'route_shown':False}
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
        self.graphics_nodes = []
        self.graphics_edges = []
        self.graphics_text = []
        self.graphics_route = []
#         self.graphics_route_h = []
        
        self.canvas_lock = t.RLock()
        
        self.sel_nodes = []
        self.sel_edges = [] 
        self.route = []
#         self.route_collisions = defaultdict(list)
        self.pe_graphic = None
        self.ng_graphic = None
        self.curr_dest = None   
        self.curr_edge = None
        self.started_edge = False    
        
        # Connection matrix data structure
        # See GenerateConnectionMatrix()
        self.conn_matrix = np.empty(shape=(150,150))
        self.conn_matrix[:] = -1   
        self.tt = datetime(1000,1,1,0,0,0)       
       
        self.mp = self.GetParent()
        self.ros = self.mp.ros
            
        # Add the Canvas
        self.NavCanvas = NavCanvas.NavCanvas(self, 
                                     ProjectionFun = None, 
                                     BackgroundColor = "DARK GREY", 
                                     )
        self.Canvas = self.NavCanvas.Canvas

        
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
#    Mouse click handler: left button                                                         #
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
        with self.canvas_lock:
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
                self.Canvas.RemoveObject(self.graphics_edges[edge_id])
                
                e = self.Canvas.AddLine((n1.coords, n2.coords), LineWidth=ew, LineColor=EDGE_COLOR)
                e.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickEdge)
                e.Bind(FloatCanvas.EVT_FC_ENTER_OBJECT, self.OnMouseEnterEdge)
                e.Bind(FloatCanvas.EVT_FC_LEAVE_OBJECT, self.OnMouseLeaveEdge)
                self.graphics_edges[edge_id] = e           
                e.Name = str(edge_id)
        
        self.RestoreModes('KeyPress')  
        self.Canvas.Draw(True)            
        
              
#---------------------------------------------------------------------------------------------#    
#    Adds a representation of the robot to the canvas. A grey robot means that its pose info  #
#    is not accurate. When accurate pose info is received, the robot graphic turns red.       #
#---------------------------------------------------------------------------------------------#    
    def AddRobot(self, coords, orient):
        with self.canvas_lock:
            # If no coords are specified, just put the robot at the origin
            if coords == -1:
                xy = (0,0)
                zw = 0
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
            
#             theta = self.ToDegrees( self.Angle(zw) )
            theta = zw                
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
        with self.canvas_lock:   
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
                self.NumTimeSteps = 12  
            else:
                self.NumTimeSteps = 48 
            
            self.dx = (dest[0]-r.XY[0]) / self.NumTimeSteps
            self.dy = (dest[1]-r.XY[1]) / self.NumTimeSteps
            self.dt = (self.dest_theta - a.Theta) / self.NumTimeSteps  
            
    #         print "Current t: %s  |  Destination t: %s  |  dt: %s" % (a.Theta, self.dest_theta, self.dt)
    #         print "Total diff t: %s" % (self.dest_theta - a.Theta)
    #         print "dt = %s" % (self.dt)
            
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
        with self.canvas_lock:
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
    #                 print "Drew arrow from %s to %s. Angle: %s" % (r.XY, str(xy2), new_theta)
                      
                    a = self.Canvas.AddArrowLine((r.XY,xy2), LineWidth = ROBOT_BORDER_WIDTH,
                                         LineColor = ROBOT_BORDER, ArrowHeadSize=15, InForeground = True)
                    a.Coords = r.XY
                    a.Theta  = new_theta
                    self.arrow = a
                    a.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickRobot)
                    self.arrow_drawn = True
                 
                elif not self.workaround:    
                    new_theta = theta + self.dt             
                    new_theta_rad = self.ToRadians(new_theta)
                    xy2 = ( x+(ROBOT_DIAM * math.cos(new_theta_rad)), y+(ROBOT_DIAM * math.sin(new_theta_rad)) )
                                      
                    a = self.Canvas.AddArrowLine((r.XY,xy2), LineWidth = ROBOT_BORDER_WIDTH,
                                            LineColor = ROBOT_BORDER, ArrowHeadSize=10, InForeground = True)
                    a.Coords = r.XY
                    a.Theta  = new_theta
                    a.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickRobot)
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
                a.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickRobot)            
                          
                try:
                    self.Canvas.RemoveObject(self.pe_graphic)
                    self.pe_graphic = None
                except (ValueError, AttributeError):
                    pass        
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
        
    def DrawObstacles(self, points):
        pass
#         print len(points)
#         try:
#             for point in points:
#                 x = int( self.MetersToPixels((point.x,0))[0] )
#                 y = int( self.MetersToPixels((point.y,0))[0] )               
#                 
#                 fc = OBSTACLE_COLOR
#                 lc = ROBOT_BORDER
#                 d = 5
#                 lw = 1
# #                 p = self.Canvas.AddRectangle((x-(d/2), y-(d/2)), (d,d), LineWidth = lw, 
# #                                           LineColor = lc, FillColor = fc)
#                 p = self.Canvas.AddCircle((x,y), d, FillColor=fc, LineColor=fc)
#                 p.Coords = (x,y)
#                 self.obstacles.append(p)
#             self.Canvas.Draw(True)
#         except AttributeError:
#             pass
            
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
#         rm_obj = False
#         while not rm_obj:
#             try:                        
#                 try:
#                     print "Removing 2D nav goal graphic"
#                     self.Canvas.RemoveObject(self.ng_graphic)
#                     self.ng_graphic = None
#                 except (ValueError, AttributeError):
#                     pass    
#                 rm_obj = True
#             except OSError as e:
#                 if e.errno == 11:
#                     print "error 11"
#                     time.sleep(0.5)
#                 else:
#                     print e.errno               
             
#             color_set = False
#             while not color_set:
#                 if self.curr_dest is not None: 
#                     print "Resetting node color"
#                     self.graphics_nodes[ self.curr_dest ].SetFillColor(NODE_FILL)
#                     self.curr_dest = None
#                 color_set = True
#              
#             print "Set heading to node %s" % dest                    
#             color_set = False
#             while not color_set:
#                 self.graphics_nodes[ dest ].SetFillColor(DESTINATION_COLOR)
#                 color_set = True
#             self.curr_dest = dest
#             self.Canvas.Draw(True)

#         print "Set heading to node %s" % dest 
#         if self.curr_dest is None:
#             self.curr_dest = dest
#             return
#                    
        with self.canvas_lock:
            if self.curr_dest is not None:
                print "Heading from %s to %s" % (self.curr_dest, dest)
                n1 = self.nodelist[self.curr_dest]
                n2 = self.nodelist[dest]
                e = int(self.conn_matrix[n1.id][n2.id])
                lw = EDGE_WIDTH
                lc = HIGHLIGHT_COLOR    
                l = self.Canvas.AddLine( (n1.coords,n2.coords), LineWidth=lw, LineColor=lc)
                self.highlights.append(l)
                self.curr_edge = e                
                
                self.Canvas.RemoveObject(self.graphics_route[0])
                self.graphics_route.pop(0) 
                self.Canvas.Draw(True)
            self.curr_dest = dest     
            

    def OnReachDestination(self):  
        with self.canvas_lock:
            try:
                self.Canvas.RemoveObject(self.ng_graphic)
                self.ng_graphic = None
            except (ValueError, AttributeError):
                pass         
                  
        with self.canvas_lock:    
            if self.curr_edge is not None:
                edge = self.edgelist[self.curr_edge]
                coords1 = self.nodelist[int(edge.node1)].coords
                coords2 = self.nodelist[int(edge.node2)].coords
                lw = EDGE_WIDTH
                lc = DESTINATION_COLOR    
                l = self.Canvas.AddLine( (coords1,coords2), LineWidth=lw, LineColor=lc)
                self.highlights.append(l)                
                
#                 c = self.route_collisions[edge.id]               
#                 if c != []:
#                     c[0].Visible = True
#                     c.pop(0)
#                     self.graphics_route_h.pop(0)                   
                
            self.Canvas.Draw(True)
                    
    def ClearHighlighting(self):
        if self.modes['verbose']:
            print "Tour complete. Reset highlighted edges."
        with self.canvas_lock:
            for edge in self.graphics_edges:
                edge.Visible = True
                edge.SetFillColor(EDGE_COLOR)
            for obj in self.graphics_route:
                self.Canvas.RemoveObject(obj)
            self.Canvas.Draw(True)  
        
    def DrawRoute(self, route, show):
        self.route = route
        
        with self.canvas_lock:
            tmp_edges = []          
            
            color = (240,0,0)
            steps = len(route)
            incr = min(720.0/steps, 30)
            phase = 0
            for idx,node in reversed(list(enumerate(route))):            
                try:
                    n1_id = int(node)
                    n2_id = int(route[idx-1])
                    node1 = self.nodelist[n1_id]
                    node2 = self.nodelist[n2_id]
                except IndexError:
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
                
                xD = self.Round( x2-((NODE_DIAM/2.0)*kx) )
                yD = self.Round( y2-((NODE_DIAM/2.0)*ky) )            
                p1 = (x1,y1)
                p2 = (xD,yD)
                
                l = self.Canvas.AddArrowLine( (p2,p1), LineWidth = EDGE_WIDTH, LineColor = color,
                                             ArrowHeadSize=15, InForeground=True)
                self.graphics_route.insert(0, l) 
                
#                 if edge_id in tmp_edges:
#                     self.route_collisions[edge_id].append(l)
#                     self.graphics_route_h.append(l)
#                     l.Visible = False
                tmp_edges.append(edge_id)
                
                if color[1]+incr < 240 and phase == 0:
                    color =  ( color[0], int(color[1]+incr), color[2] )
                    continue
                elif color[0]-incr > 0 and phase == 1:
                    color =  ( int(color[0]-incr), color[1], color[2] )
                    continue
                elif color[2]+(2*incr) < 240 and phase == 2:
                    color =  ( color[0], color[1], int(color[2]+(2*incr)) )
                    continue
                elif color[1]-(2*incr) > 0 and phase == 3:
                    color =  ( color[0], int(color[1]-(2*incr)), color[2] )
                    continue
                else:
                    phase = phase+1                 
           
            if not show:
                for gr in self.graphics_route:
                    gr.Visible = False
            self.RefreshNodes('normal')
            self.Canvas.Draw(True)
            
    def ShowRoute(self):
        with self.canvas_lock:            
#             self.SetModes('ShowRoute', {'route_shown':True})
            for edge in self.graphics_edges:
                edge.Visible = False  
#             for hl in self.highlights:
#                 hl.Visible = False
            for gr in self.graphics_route:
                gr.Visible = True
#             for gr in self.graphics_route_h:
#                 gr.Visible = False
            self.Canvas.Draw(True)
                
    def HideRoute(self):
        with self.canvas_lock:
            for edge in self.graphics_edges:
                edge.Visible = True  
#             for hl in self.highlights:
#                 hl.Visible = True
            for gr in self.graphics_route:
                gr.Visible = False
            self.Canvas.Draw(True)
#             self.RestoreModes('ShowRoute')

#--------------------------------------------------------------------------------------------#    
#     Creates a single node at the given coordinates                                         #
#--------------------------------------------------------------------------------------------#    
    def CreateNode(self, coords): 
        with self.canvas_lock:
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
                self.nodelist.append(node)
    #             lock = t.RLock()
    #             self.node_locks.append(lock)
                
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
                return -collision
        

#--------------------------------------------------------------------------------------------#    
#    Creates edges between all selected nodes. Edges are be created in the order that the    #
#    nodes were selected.                                                                    #
#--------------------------------------------------------------------------------------------#         
    def CreateEdges(self, event):
        with self.canvas_lock:
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
    def RefreshNodes(self, color_str):
        with self.canvas_lock:
            for node in self.nodelist:                    
                ID = str(node.id)
                xy = node.coords[0], node.coords[1]
                diam = NODE_DIAM
                lw = NODE_BORDER_WIDTH
                fc = NODE_FILL 
                
                if color_str == 'dest':
                    lc = DESTINATION_COLOR  
                else:
                    lc = NODE_BORDER
                if int(ID) < 100:  
                    fs = FONT_SIZE_1
                else:
                    fs = FONT_SIZE_2          
                                    
                # Draw the node on the canvas
                c = self.Canvas.AddCircle(xy, diam, LineWidth=lw, LineColor=lc, FillColor=fc,
                                          InForeground = True)
                c.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickNode)
                self.Canvas.RemoveObject( self.graphics_nodes[int(ID)] )
                self.graphics_nodes[int(ID)] = c                     
                c.Name = ID
                c.Coords = node.coords                  
                
                t = self.Canvas.AddScaledText(ID, xy, Size=fs, Position="cc",Color=lc, 
                                              Weight=wx.BOLD, InForeground = True)
                self.Canvas.RemoveObject( self.graphics_text[int(ID)] )
                self.graphics_text[int(ID)] = t
                
            old_robot, old_arrow = self.robot, self.arrow
            self.AddRobot(self.robot.Coords, self.arrow.Theta)
            self.Canvas.RemoveObject(old_robot)
            self.Canvas.RemoveObject(old_arrow)
                

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
        with self.canvas_lock:    
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
        with self.canvas_lock:
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
        with self.canvas_lock:
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
        with self.canvas_lock:
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
    #                 print "edge %s distance to node %s: %s (t0 = %s)" % (edge.id, node.id, dist, t)
                    if dist < min_distance[1]:
                        min_distance = node.id, dist
                        break
                elif t>1:
                    dist = math.sqrt( self.Distance2(node.coords, (ex2, ey2)) )
    #                 print "edge %s distance to node %s: %s (t1 = %s)" % (edge.id, node.id, dist, t)
                    if dist < min_distance[1]:
                        min_distance = node.id, dist
                        break
                else:
                    dist = math.sqrt( self.Distance2(node.coords, ( ex1+t*(ex2-ex1), ey1+t*(ey2-ey1) )) )
    #                 print "edge %s distance to node %s: %s (t2 = %s)" % (edge.id, node.id, dist, t)
                    if dist < min_distance[1]:
                        min_distance = node.id, dist
                        break
    #         print "edge %s min distance is %s to node %s" % (edge.id, min_distance[1], min_distance[0])
            if min_distance[0] != -1:
                return min_distance
            else:
                return None
        
#--------------------------------------------------------------------------------------------#    
#      Given a node, returns any edges which are too close                                   #
#--------------------------------------------------------------------------------------------#    
    def MinDistanceToEdge(self, node):
        with self.canvas_lock:
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
    #                 print "node %s distance to edge %s: %s (t0 = %s)" % (node.id, edge.id, dist, t)
                    if dist < thresh:
                        min_distance.append( (edge.id, dist) )
                    continue
                elif t>1:
                    dist = math.sqrt( self.Distance2(node.coords, (ex2, ey2)) )
    #                 print "node %s distance to edge %s: %s (t1 = %s)" % (node.id, edge.id, dist, t)
                    if dist < thresh:
                        min_distance.append( (edge.id, dist) )
                    continue
                else:
                    dist = math.sqrt( self.Distance2(node.coords, ( ex1+t*(ex2-ex1), ey1+t*(ey2-ey1) )) )
    #                 print "node %s distance to edge %s: %s (t2 = %s)" % (node.id, edge.id, dist, t)
                    if dist < thresh:
                        min_distance.append( (edge.id, dist) )
                    continue
    #         print "node %s min distance is %s to edge %s" % (node.id, min_distance[1], min_distance[0])
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
        with self.canvas_lock:
            wx.BeginBusyCursor()
            st = datetime.now() 
            self.tt = datetime(100,1,1,0,0,0) 
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
            print "Total gcm time: %s" % self.tt

#--------------------------------------------------------------------------------------------#
#     Find the distances from a given node 'node1' to all other nodes in the graph.          # 
#                                                                                            #
#     k: Number of nodes to return (function returns the 'k' closest nodes)                  #
#     e: Maximum search radius                                                               #                                                               #
#--------------------------------------------------------------------------------------------#     
    def GetNodeDistances(self, node1, k, e):
        with self.canvas_lock:
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
        with self.canvas_lock:
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
        with self.canvas_lock:
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
        with self.canvas_lock:
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
        with self.canvas_lock: 
            try:     
                ID = int(edge.Name)
                self.Canvas.RemoveObject(self.graphics_edges[ID])                   
                self.edgelist[ID] = None
                self.graphics_edges[ID] = None            
                
                if self.modes['verbose']:
                    print "Removed edge #" + str(ID)
                if self.modes['redraw']:
                    self.Canvas.Draw(True)
                
            except AttributeError:
                # Case where the edge has already been removed: do nothing
                pass
        
#--------------------------------------------------------------------------------------------#    
#     Renumbers the nodes after a deletion                                                   #
#     This function should not be called directly -> use DeleteSelection() instead           #
#--------------------------------------------------------------------------------------------#            
    def RenumberNodes(self):
        with self.canvas_lock:
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
        with self.canvas_lock:
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
        with self.canvas_lock:
            st = datetime.now()
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
            et = datetime.now()
            self.tt = self.tt + (et-st)
    #         print "Generated connection matrix. Time taken: %s" % (et-st)
    #         return conn_mtx     

    def AddConnectionEntry(self, edge):
        st = datetime.now()
        self.conn_matrix[ int(edge.node1) ][ int(edge.node2) ] = edge.id
        self.conn_matrix[ int(edge.node2) ][ int(edge.node1) ] = edge.id
        et = datetime.now()
        self.tt = self.tt + (et-st)
        
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
        with self.canvas_lock:
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

    def Round(self, flt):
        if flt % 1 >= 0.5:
            return int(flt)+1
        else:
            return int(flt)
    
#--------------------------------------------------------------------------------------------#    
#     Selects a single node. If desel is True, deselects everything else.                    #
#--------------------------------------------------------------------------------------------#   
    def SelectOneNode(self, obj, desel):
        with self.canvas_lock:
            if desel:      
                self.DeselectAll(event=None)
    #             if self.modes['verbose']:
    #                 print "Selected Node #" + obj.Name      
            self.sel_nodes.append(obj)   
            obj.SetFillColor(SELECT_COLOR)
            if self.modes['redraw']:
                self.Canvas.Draw(True)
        
#--------------------------------------------------------------------------------------------#    
#     Selects a single edge. If desel is True, deselects everything else.                    #
#--------------------------------------------------------------------------------------------#   
    def SelectOneEdge(self, obj, desel):
        with self.canvas_lock:
            if desel:     
                self.DeselectAll(event=None)
    #             if self.modes['verbose']:
    #                 print "Selected Edge #" + obj.Name     
            self.sel_edges.append(obj)
            obj.SetLineColor(SELECT_COLOR)
            if self.modes['redraw']:
                self.Canvas.Draw(True) 
            
#--------------------------------------------------------------------------------------------#    
#     Selects all nodes and deselects everything else.                                       #
#--------------------------------------------------------------------------------------------#        
    def SelectNodes(self, event):
        with self.canvas_lock:
            self.DeselectAll(event)
            
            # Set highlighted colour
            for node in self.nodelist:
                self.graphics_nodes[ node.id ].SetFillColor(SELECT_COLOR)
                self.sel_nodes.append(self.graphics_nodes[node.id])
            if self.modes['verbose']:
                    print "Selected all nodes"
            if self.modes['redraw']:    
                self.Canvas.Draw(True) 
        
#--------------------------------------------------------------------------------------------#    
#     Selects all edges and deselects everything else.                                       #
#--------------------------------------------------------------------------------------------#    
    def SelectEdges(self, event):
        with self.canvas_lock:
            self.DeselectAll(event)
            
            # Set highlighted colour
            for edge in self.edgelist:
                self.graphics_edges[ edge.id ].SetLineColor(SELECT_COLOR)
                self.sel_edges.append(self.graphics_edges[edge.id])
            if self.modes['verbose']: 
                print "Selected all edges"
            if self.modes['redraw']:    
                self.Canvas.Draw(True)

#--------------------------------------------------------------------------------------------#
#     Selects all nodes/edges located within a given X and Y range. This is used with the    #
#     'box selection' tool on the NavCanvas.                                                   #
#--------------------------------------------------------------------------------------------#           
    def SelectBox(self, x_range, y_range):
        with self.canvas_lock:
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
            self.Canvas.Draw(True)
            self.RestoreModes('SelectBox')
        
#--------------------------------------------------------------------------------------------#    
#     Select/deselect everything                                                             #
#--------------------------------------------------------------------------------------------#             
    def SelectAll(self, event):  
        with self.canvas_lock:                     
            self.DeselectAll(event)   
               
            for node in self.nodelist:                
                if event is not None:   
                    self.graphics_nodes[ node.id ].SetFillColor(SELECT_COLOR)
                self.sel_nodes.append(self.graphics_nodes[node.id])
            for edge in self.edgelist:
                if event is not None:   
                    self.graphics_edges[ edge.id ].SetLineColor(SELECT_COLOR)
                self.sel_edges.append(self.graphics_edges[edge.id])
            
            if self.modes['verbose']:            
                print "Selected all nodes and edges"
            if self.modes['redraw']:    
                self.Canvas.Draw(True) 
            
    def DeselectAll(self, event):
        with self.canvas_lock:
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
        with self.canvas_lock:
            if obj in self.sel_nodes:
                coords = self.nodelist[int(obj.Name)].coords  
                if self.modes['verbose']:        
                    print "Deselected Node %s  (%s, %s)" % (obj.Name, coords[0], coords[1])
                self.sel_nodes.remove(obj)
                obj.SetFillColor(NODE_FILL)
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
        with self.canvas_lock:
            current_mode = self.Canvas.GetMode() 
            if current_mode == 'GUIEdges':
                self.Canvas.GUIMode.LockEdge('e_node', obj.Name, self.nodelist[int(obj.Name)].coords)  
    #             print "enter node"          
    #         else:
            self.Canvas.GUIMode.SwitchCursor('enter')  
            
    def OnMouseLeaveNode(self, obj):  
        with self.canvas_lock:      
            current_mode = self.Canvas.GetMode() 
            if current_mode == 'GUIEdges':
                self.Canvas.GUIMode.LockEdge('l_node', obj.Name, self.nodelist[int(obj.Name)].coords) 
    #             print "leave node"           
    #         else:
            self.Canvas.GUIMode.SwitchCursor('leave')  
               
    def OnMouseEnterEdge(self, obj):
        with self.canvas_lock:
            current_mode = self.Canvas.GetMode() 
            if current_mode == 'GUIEdges':
                self.Canvas.GUIMode.LockEdge('e_edge', -1, -1)
    #             print "enter edge"
    #         else:
            self.Canvas.GUIMode.SwitchCursor('enter')  
                  
    def OnMouseLeaveEdge(self, obj):
        with self.canvas_lock:
            current_mode = self.Canvas.GetMode() 
            if current_mode == 'GUIEdges':
                self.Canvas.GUIMode.LockEdge('l_edge', -1, -1) 
    #             print "leave edge"
    #         else:
            self.Canvas.GUIMode.SwitchCursor('leave') 
        
#--------------------------------------------------------------------------------------------#    
#     Event when an edge is left-clicked.                                                    #
#     Adds the node to the selection list and highlights it on the canvas                    #
#--------------------------------------------------------------------------------------------#            
    def OnClickEdge(self, obj): 
        with self.canvas_lock:
            if obj in self.sel_edges:  
                if self.modes['verbose']:        
                    print "Deselected Edge " + obj.Name
                self.sel_edges.remove(obj)
                obj.SetLineColor(EDGE_COLOR)
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
                        'auto_edges':False
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
                
        #badcodingpraticelol        
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
        with self.canvas_lock:
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
        with self.canvas_lock:
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
          
    
    def SetModes(self, key, state_dict):
        self.saved_modes[key] = (self.modes.copy())  
        for key,val in state_dict.iteritems():
            self.modes[key] = val
            
    def RestoreModes(self, key):
        self.modes = self.saved_modes[key].copy()
    
#--------------------------------------------------------------------------------------------#    
#     Clears the entire canvas                                                               #
#--------------------------------------------------------------------------------------------#   
    def Clear(self):
        self.Canvas.InitAll()  
        
    def OnClear(self):
        if self.highlights != []:
            with self.canvas_lock:
                for obj in self.highlights:
                    self.Canvas.RemoveObject(obj)
                self.highlights = []
                for obj in self.graphics_route:
                    self.Canvas.RemoveObject(obj)
                self.graphics_route = []
                for obj in self.graphics_edges:
                    obj.Visible = True
                self.RefreshNodes('normal')
                self.mp.ep.btn_rte.SetLabel('Show Route')
                self.mp.ep.btn_rte.Enable(False)
                self.Canvas.Draw(True)
        else:
            self.ClearGraph()
                   
    def ClearGraph(self):
        self.SetModes('ClearGraph', {                        
                        'verbose':False, 
                        }) 
        self.SelectAll(None)
        self.DeleteSelection(None)  
        self.RestoreModes('ClearGraph')    
    
    def SaveCanvasImage(self, filename):
        with self.canvas_lock:
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
            
    def Test(self):
        route = [0,1,2,6,8,9,8,6,5,2,3,4,3,1,5,7,1,0,7]
        self.DrawRoute(route, False)
        self.GetParent().ep.btn_rte.Enable(True)
                
        with self.canvas_lock:
            for n in route:
                self.HighlightDestination(n)
                self.Canvas.Draw(True)
                time.sleep(1)
                self.OnReachDestination()
                self.OnReachDestination()
                time.sleep(1)
                self.Canvas.Draw(True)
                wx.Yield()
    
#--------------------------------------------------------------------------------------------#    
#     Sets the image to display on the canvas. If the map has an associated graph file,      #
#     the corresponding nodes and edges are loaded and drawn onto the canvas.                #
#--------------------------------------------------------------------------------------------#
    def SetImage(self, image_obj):
        with self.canvas_lock:
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
    #         self.DrawObstacles(self.ros.obstacles)
                 
            self.SetCurrentMapPath(image_file)
            self.Show() 
            self.Layout()        
            self.ZoomToFit()
            
            et = datetime.now()
            self.RestoreModes('SetImage')
    
            if self.modes['verbose']:
                print "Set map image. Time taken: %s" % str(et-st)
