#!/usr/bin/env python

'''
Created on May 30, 2013
'''

import wx
import os, time, shutil
import math
import listener as ls
import publisher as pb
from zoompanel import ZoomPanel

APP_SIZE        = (240,452)
APP_SIZE_EXP    = (240,502)
BUTTON_COLOR    = (119,41,83)
BUTTON_SIZE     = (180,30)
BG_COLOR        = (205,205,205)
H_SPACER_WIDTH  = 20
V_SPACER_SMALL  = 10
V_SPACER_LARGE  = 15
SIZER_BORDER    = 10

#TODO: check variables on save (new nodes start at ID 0 even if nodes were loaded??)

#TODO: opening a file while nodes still on canvas = some node numbers invisible
#      (doesn't seem to break functionality) 

#TODO: automatically generate map

#TODO: ability to move nodes (arrow keys? dragging?)

#TODO: function to convert (x,y) into image_data array index

class MainFrame(wx.Frame):
    def __init__(self, parent, title):
        wx.Frame.__init__(self, parent, title=title, size=APP_SIZE_EXP,
                        style=wx.FRAME_SHAPED
                        )
        self.resolution = wx.GetDisplaySize()
        self.leftDown = False                                 
        self.font = wx.Font(pointSize=14, family=wx.FONTFAMILY_DEFAULT, 
                       style=wx.FONTSTYLE_NORMAL, weight=wx.FONTWEIGHT_NORMAL, 
                       faceName="lucida sans")    
        
        self.ls = ls.listener(self)
        self.pb = pb.publisher(self) 
#         self.tt = TimerThread(self)       
        self.mp = MainPanel(self)  
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.mp, 1, wx.EXPAND)
        
        self.ls.SetAttributes()
#         self.tt.start()
          
        self.Bind(wx.EVT_MOTION, self.OnMouse)
        self.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        self.Bind(wx.EVT_LEFT_UP, self.OnLeftUp) 
        
        self.SetPosition((0,0))        
        self.Layout()  
        self.mp.ep.size = self.mp.ep.GetSize()

#---------------------------------------------------------------------------------------------#    
#    Mouse capturing functions to allow dragging of the control panel around the screen.      #
#---------------------------------------------------------------------------------------------#        
    def OnMouse(self, event):
        if event.Dragging() and self.leftDown:
            pos = self.ClientToScreen(event.GetPosition()) 
            fp = (pos.x - self.delta.x, pos.y - self.delta.y) 
            self.parent_frame.Move(fp)  
            
    def OnLeftDown(self, event): 
        self.CaptureMouse() 
        self.leftDown = True 
        pos = self.ClientToScreen(event.GetPosition()) 
        origin = self.parent_frame.GetPosition() 
        dx = pos.x - origin.x 
        dy = pos.y - origin.y 
        self.delta = wx.Point(dx, dy)  
        
    def OnLeftUp(self, event): 
        try:
            self.ReleaseMouse() 
            self.leftDown = False
        except wx._core.PyAssertionError:
            pass  
        
        
        
class MainPanel(wx.Panel):
    def __init__(self, parent):        
        wx.Panel.__init__(self, parent=parent)        
        self.bg = self.DrawBG(APP_SIZE_EXP)
        
        self.leftDown = False
        self.saved = True
        self.buttons = []
        self.contents = []
        
        # Set parent frame value
        self.parent_frame = parent 
        while self.parent_frame.GetParent() is not None: 
            self.parent_frame = self.parent_frame.GetParent()
        
        self.ls = self.parent_frame.ls
        self.pb = self.parent_frame.pb
#         self.tt = self.parent_frame.tt
        
        # Create the sizers 
        self.sizer_main = wx.BoxSizer(wx.HORIZONTAL)         
        self.sizer_menu = wx.BoxSizer(wx.VERTICAL)      
        self.sizer_display = wx.BoxSizer(wx.VERTICAL)
        
        # The map viewer panel
        zp_size = self.parent_frame.resolution[1]-60     
        self.zp=ZoomPanel(self, title="Map View",
                                  size=((zp_size,zp_size)), 
#                                   style=wx.FRAME_SHAPED
                                  )  
        self.zp.Hide()
        self.zp.SetPosition((320,0))  
        
        self.sizer_menu.AddSpacer(V_SPACER_SMALL)  
                        
        # Refresh map button
        hbox00 = wx.BoxSizer(wx.HORIZONTAL)     
        hbox00.AddSpacer(H_SPACER_WIDTH)
        self.btn_rf = wx.Button(self, label="Update Map", size=BUTTON_SIZE)        
        self.btn_rf.Bind(wx.EVT_BUTTON, self.OnRefreshMap)
        self.buttons.append(self.btn_rf) 
        hbox00.Add(self.btn_rf)        
        self.sizer_menu.Add(hbox00,0,wx.TOP|wx.LEFT|wx.RIGHT,SIZER_BORDER) 

        # Show/Hide map viewer button
        hbox03 = wx.BoxSizer(wx.HORIZONTAL)            
        hbox03.AddSpacer(H_SPACER_WIDTH)
        self.btn_map = wx.Button(self, label="Show Map", size=BUTTON_SIZE)        
        self.btn_map.Bind(wx.EVT_BUTTON, self.OnShowHideMap)   
        self.btn_map.Enable(False)   
        self.buttons.append(self.btn_map)   
        hbox03.Add(self.btn_map)        
        self.sizer_menu.Add(hbox03,0,
                            wx.TOP|wx.LEFT|wx.RIGHT
                            ,SIZER_BORDER)  
               
#         # Explore button
#         hbox06 = wx.BoxSizer(wx.HORIZONTAL)     
#         hbox06.AddSpacer(H_SPACER_WIDTH)
#         self.btn_exp = wx.Button(self, label="Explore...", size=BUTTON_SIZE)        
#         self.btn_exp.Bind(wx.EVT_BUTTON, self.OnExplore)
#         self.btn_exp.Enable(False) 
#         self.buttons.append(self.btn_exp) 
#         hbox06.Add(self.btn_exp)        
#         self.sizer_menu.Add(hbox06,0,wx.TOP|wx.LEFT|wx.RIGHT,SIZER_BORDER)
        
        self.sizer_menu.AddSpacer(V_SPACER_LARGE)
        
        # Explore panel
        hbox09 = wx.BoxSizer(wx.HORIZONTAL)     
        hbox09.AddSpacer(H_SPACER_WIDTH)       
        self.ep = ExplorePanel(self)
#         self.ep.Hide()
        hbox09.Add(self.ep)   
        self.sizer_menu.Add(hbox09,0,wx.LEFT|wx.RIGHT,SIZER_BORDER)
        
        self.sizer_menu.AddSpacer(V_SPACER_LARGE) 
        self.sizer_menu.AddSpacer(V_SPACER_LARGE) 

        # Open button
        hbox10 = wx.BoxSizer(wx.HORIZONTAL)            
        hbox10.AddSpacer(H_SPACER_WIDTH)
        self.btn_open = wx.Button(self, label="Open", size=BUTTON_SIZE)   
        self.buttons.append(self.btn_open)        
        self.btn_open.Bind(wx.EVT_BUTTON, self.OnOpen)  
        hbox10.Add(self.btn_open)           
        self.sizer_menu.Add(hbox10,0,
                            wx.TOP|wx.LEFT|wx.RIGHT
                            ,SIZER_BORDER)  
        # Save button
        hbox13 = wx.BoxSizer(wx.HORIZONTAL)            
        hbox13.AddSpacer(H_SPACER_WIDTH)
        self.btn_sv = wx.Button(self, label="Save", size=BUTTON_SIZE) 
        self.btn_sv.Bind(wx.EVT_BUTTON, self.OnSave)        
        self.btn_sv.Enable(False)     
        self.buttons.append(self.btn_sv)   
        hbox13.Add(self.btn_sv)           
        self.sizer_menu.Add(hbox13,0,
                            wx.TOP|wx.LEFT|wx.RIGHT
                            ,SIZER_BORDER)        
        # Save As button
        hbox16 = wx.BoxSizer(wx.HORIZONTAL)            
        hbox16.AddSpacer(H_SPACER_WIDTH)
        self.btn_svas = wx.Button(self, label="Save As", size=BUTTON_SIZE) 
        self.btn_svas.Bind(wx.EVT_BUTTON, self.OnSaveAs)        
        self.btn_svas.Enable(False)     
        self.buttons.append(self.btn_svas)   
        hbox16.Add(self.btn_svas)           
        self.sizer_menu.Add(hbox16,0,
                            wx.TOP|wx.LEFT|wx.RIGHT
                            ,SIZER_BORDER)  
        
        self.sizer_menu.AddSpacer(V_SPACER_LARGE)
              
        # Exit button
        hbox20 = wx.BoxSizer(wx.HORIZONTAL)             
        hbox20.AddSpacer(H_SPACER_WIDTH)
        btn_exit = wx.Button(self, label="Exit", size=BUTTON_SIZE)  
        self.buttons.append(btn_exit)      
        hbox20.Add(btn_exit)           
        btn_exit.Bind(wx.EVT_BUTTON, self.OnExit)
        self.sizer_menu.Add(hbox20,0,wx.TOP|wx.LEFT|wx.RIGHT,SIZER_BORDER)   
                      
        self.PaintButtons( (255,255,255),BUTTON_COLOR )  
                
        # Mouse capturing events
        self.bg.Bind(wx.EVT_MOTION, self.OnMouse)
        self.bg.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        self.bg.Bind(wx.EVT_LEFT_UP, self.OnLeftUp)        
        self.Bind(wx.EVT_MOTION, self.OnMouse)
        self.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        self.Bind(wx.EVT_LEFT_UP, self.OnLeftUp)   
        
        self.sizer_main.Add(self.sizer_menu,0,wx.ALIGN_RIGHT)
        self.sizer_main.Add(self.sizer_display,0,wx.ALIGN_LEFT)
        self.SetSizer(self.sizer_main)
        
        self.Layout()

#---------------------------------------------------------------------------------------------#    
#    Draws the background for the control panel                                               #
#---------------------------------------------------------------------------------------------#       
    def DrawBG(self, size):
        x = size[0]
        y = size[1]
        bmp = wx.EmptyBitmap(x,y)
        dc = wx.MemoryDC()
        dc.SelectObject(bmp)
        
        solidbrush = wx.Brush(BUTTON_COLOR, wx.SOLID)
        dc.SetBrush(solidbrush)        
        dc.DrawRectangle(-1, -1, x+2, y+2)
        solidbrush = wx.Brush(BG_COLOR, wx.SOLID)
        dc.SetBrush(solidbrush)        
        dc.DrawRectangle(3, 3, x-6, y-6)
        
        return wx.StaticBitmap(self, -1, bmp, (0, 0)) 

#---------------------------------------------------------------------------------------------#    
#    Sets the foreground and background color of all buttons on the control panel             #
#---------------------------------------------------------------------------------------------#        
    def PaintButtons(self, foreground, background):
        try:
            for btn in self.buttons:
                btn.SetForegroundColour(foreground)
                btn.SetBackgroundColour(background)
        except IOError:
            print "Invalid argument: expected (R,G,B) value"
                
#---------------------------------------------------------------------------------------------#    
#    Starts a listener process which listens on the "/map" topic. Once the listener has       #
#    exported the map file, it is passed to ZoomPanel, which sets the image in the viewer     #
#---------------------------------------------------------------------------------------------#                
    def OnRefreshMap(self, event):
        
        if self.saved is False:
            
            dlg = wx.MessageDialog(self,
            "The current map is unsaved.\nWould you like to save it?", 
            "Notice", wx.YES_NO)
            
            if dlg.ShowModal() == wx.ID_YES:
                # User has chosen to save the map
                self.OnSaveAs(event)
                dlg.Destroy()
            else:                
                # User has chosen not to save
                dlg.Destroy()
                self.SetSaveStatus(True)
        
        if len(self.zp.nodelist) > 0:
            dlg = wx.MessageDialog(self,
            "Do you want to keep the current\nnodes and edges on the updated map?", 
            "Map", wx.YES_NO)
                        
            if dlg.ShowModal() == wx.ID_NO:
                self.zp.SetNodeList([])
                self.zp.SetEdgeList([])
            dlg.Destroy()
                
        
        wx.BeginBusyCursor()    
        # Start listening for a map
        self.ls.Listen()        
        print "Creating map..."  
                
        try:
            map_file = self.ls.GetDefaultFilename()
            self.parent_frame.SetTitle("%s" % map_file)
            
            # Update some statuses
            for b in self.buttons:
                if b.Enabled == False:
                    b.Enable(True)
            self.SetSaveStatus(False)            
            
            # Show the image panel                  
            self.zp.origin = (self.ls.origin_pos, self.ls.origin_orient)
#             self.zp.SetImage(map_file)
            while self.ls.image is None:
                time.sleep(0.5)
                
            self.zp.SetImage(self.ls.image)
            self.zp.Show()   
#             self.tt.paused = False
            
        except IndexError:
            # Image not found in directory
            pass      
        
        self.btn_map.SetLabel("Hide Map")         
        self.Layout()                
        wx.EndBusyCursor()
       
#---------------------------------------------------------------------------------------------#    
#    Shows or hides the map, depending on the map's current state.                            #
#---------------------------------------------------------------------------------------------#        
    def OnShowHideMap(self, event):
        if self.btn_map.GetLabel()[0]=='S':
            self.zp.Show()
            self.btn_map.SetLabel("Hide Map")
        else:
            self.zp.Hide()
            self.btn_map.SetLabel("Show Map")

#---------------------------------------------------------------------------------------------#    
#    Shows the "explore" panel, which allows the user to find specific nodes and edges.       #
#---------------------------------------------------------------------------------------------#            
    def OnExplore(self, event):
        if self.ep.IsShown():
            self.ep.Hide()
            self.parent_frame.SetSize(APP_SIZE)            
            self.bg = self.DrawBG(APP_SIZE)
            self.Show()
            self.Layout()
        else:
            self.ep.txt.Clear()            
            self.parent_frame.SetSize(APP_SIZE_EXP)
            self.bg = self.DrawBG(APP_SIZE_EXP)
            for b in self.buttons:
                b.Show()
            self.ep.Show()
            self.Layout()
            

#---------------------------------------------------------------------------------------------#    
#    Shows a file dialog allowing the user to select a map file (.png format)                 #
#---------------------------------------------------------------------------------------------#    
    def OnOpen(self, event): 
          
        if self.saved is False:
            dlg = wx.MessageDialog(self,
            "The current map is unsaved.\nWould you like to save it before opening a new one?", 
            "Notice", wx.YES_NO)
            
            if dlg.ShowModal() == wx.ID_YES:
                # User has chosen to save the map
                self.OnSaveAs(event)
                dlg.Destroy()
            else:                
                # User has chosen not to save
                dlg.Destroy()
                self.SetSaveStatus(True)
                self.OnOpen(event)
                      
        else:
            # Open a file dialog for the user to select a file            
            filters = 'Image files (*.png)|*.png'
            dlg = wx.FileDialog(self, message="Open Map File", defaultDir=os.getcwd(), 
                                defaultFile="", wildcard=filters, style=wx.FD_OPEN)
            
            if dlg.ShowModal() == wx.ID_OK:   
                self.zp.SetNodeList([])
                self.zp.SetEdgeList([])
                
                # Set the viewer image to the selected file
                filename = dlg.GetPath()  
                self.parent_frame.SetTitle("%s" % dlg.GetFilename()) 
                
                # Import the node data. For this to work, the node file must have the same
                # name as the map file, but with the extension ".graph"
                try:
                    graph_filename = "%sgraph" % filename.rstrip("png")
                    graph_file = open(graph_filename, "r")
                    self.zp.ImportGraph(graph_file)
                    graph_file.close()
                except IOError:
                    self.zp.SetNodeList([])
                    self.zp.SetEdgeList([])
                
                self.ls.Listen()
                self.zp.SetImage(filename)  
                self.zp.Show()             
                
                for b in self.buttons:
                    if b.Enabled == False:
                        b.Enable(True)                 
                self.SetSaveStatus(True) 
                self.btn_map.SetLabel("Hide Map")
                            
            dlg.Destroy()
     
            
#---------------------------------------------------------------------------------------------#    
#    Saves the current map, overwriting the old version.                                      #
#---------------------------------------------------------------------------------------------#   
    def OnSave(self, event):    
        current_map = self.zp.current_map 
        if current_map is [] or os.path.basename(current_map)==self.ls.GetDefaultFilename():
            self.OnSaveAs(event)  
        else:
            shutil.move(current_map,current_map)
            
            # The graph filename must be the same as the map filename (except the extension)
            graph_filename = "%sgraph" % current_map.rstrip("png")
            graph_file = open(graph_filename, "w")
            self.zp.ExportGraph(graph_file)
            graph_file.close()
            
            print "Saved: %s" % current_map
            self.SetSaveStatus(True) 
    
    
#---------------------------------------------------------------------------------------------#    
#    Opens a file dialog and lets the user save a map. The map file is stored as a *.png      #
#    image, and the graph data is stored as a *.graph file with the same name as the map.     #
#---------------------------------------------------------------------------------------------# 
    def OnSaveAs(self, event):
        filters = 'Image files (*.png)|*.png'
        dlg = wx.FileDialog(self, message="Save Map File", defaultDir=os.getcwd(), 
                            defaultFile="", wildcard=filters, style=wx.FD_SAVE|
                            wx.FD_OVERWRITE_PROMPT)
        
        if dlg.ShowModal() == wx.ID_OK:   
            # Save the file to the path given by the user         
            current_map = self.zp.current_map
            filename = dlg.GetPath()
            
            try:
                shutil.copy(current_map,filename) 
            except shutil.Error:
                shutil.move(filename, filename)   
            
            # The graph filename must be the same as the map filename (except the extension)
            graph_filename = "%sgraph" % filename.rstrip("png")
            graph_file = open(graph_filename, "w")
            self.zp.ExportGraph(graph_file)
            graph_file.close()
            
            print "Saved: %s" % filename            
            self.SetSaveStatus(True) 
            self.parent_frame.SetTitle("%s" % dlg.GetFilename())
                        
        dlg.Destroy()
        
#---------------------------------------------------------------------------------------------#    
#    Accessor function for the current save state (Saved/Unsaved)                            #
#---------------------------------------------------------------------------------------------#         
    def SetSaveStatus(self, bool_save):
        self.saved = bool_save

#---------------------------------------------------------------------------------------------#    
#    Exits the application. If the current map is unsaved, user is asked to save first.       #
#---------------------------------------------------------------------------------------------#    
    def OnExit(self, event):
        if self.saved is False:
            dlg = wx.MessageDialog(self,
            "The current map is unsaved.\nWould you like to save it before exiting?", 
            "Warning", wx.YES_NO)
            
            if dlg.ShowModal() == wx.ID_YES:
                # User has chosen to save the map
                self.OnSaveAs(event)
            dlg.Destroy()        
        
#         self.tt.stopped = True 
#         self.tt.join()       
        self.zp.Close()
        self.parent_frame.Close()
                

#---------------------------------------------------------------------------------------------#    
#    Mouse capturing functions to allow dragging of the control panel around the screen.      #
#---------------------------------------------------------------------------------------------#        
    def OnMouse(self, event):
        if event.Dragging() and self.leftDown:
            pos = self.ClientToScreen(event.GetPosition()) 
            fp = (pos.x - self.delta.x, pos.y - self.delta.y) 
            self.parent_frame.Move(fp)  
            
    def OnLeftDown(self, event): 
        self.CaptureMouse() 
        self.leftDown = True 
        pos = self.ClientToScreen(event.GetPosition()) 
        origin = self.parent_frame.GetPosition() 
        dx = pos.x - origin.x 
        dy = pos.y - origin.y 
        self.delta = wx.Point(dx, dy)  
        
    def OnLeftUp(self, event): 
        try:
            self.ReleaseMouse() 
            self.leftDown = False
        except wx._core.PyAssertionError:
            pass
        
        
class ExplorePanel(wx.Panel):
    def __init__(self, parent):        
        wx.Panel.__init__(self, parent=parent)
        self.zp = self.GetParent().zp     
        
        # Set the background colour
        bmp = wx.EmptyBitmap(500, 500)
        dc = wx.MemoryDC()
        dc.SelectObject(bmp)
        solidbrush = wx.Brush(BG_COLOR, wx.SOLID)
        dc.SetBrush(solidbrush)
        dc.DrawRectangle(0, 0, 500, 500)
        self.bg = wx.StaticBitmap(self, -1, bmp, (-2, -2))
        
        # Set parent frame value
        self.parent_frame = parent 
        while self.parent_frame.GetParent() is not None: 
            self.parent_frame = self.parent_frame.GetParent()
        
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.hbox00 = wx.BoxSizer(wx.HORIZONTAL)
        self.hbox01 = wx.BoxSizer(wx.HORIZONTAL)
        
        # Generate Graph button
        vbox00 = wx.BoxSizer(wx.VERTICAL)   
        self.btn_gg = wx.Button(self, label="Generate Graph", size=BUTTON_SIZE)        
        self.btn_gg.Bind(wx.EVT_BUTTON, self.OnGenerateGraph)
        self.btn_gg.Enable(False)
        self.GetParent().buttons.append(self.btn_gg) 
        vbox00.Add(self.btn_gg)        
        self.sizer.Add(vbox00,1,wx.TOP,10)
        
        #TODO: thirds txtbox for dist between nodes
        
        # Textbox0
        vbox03 = wx.BoxSizer(wx.VERTICAL)     
        vbox03.AddSpacer(5)
        self.txt0 = wx.TextCtrl(self, size=(80,30), style=wx.NO_BORDER|wx.TE_CENTER)
        self.txt0.SetMaxLength(3)    #Maximum of 3 characters
        self.txt0.SetFont(self.parent_frame.font)
        self.txt0.SetValue('n')
        self.txt0.SetForegroundColour((255,131,79))
        self.txt0.SetBackgroundColour((85,85,80))
        self.txt0.Bind(wx.EVT_SET_FOCUS, self.OnTxtFocus)
        vbox03.Add(self.txt0)
        self.hbox00.Add(vbox03,1,wx.LEFT|wx.BOTTOM|wx.RIGHT,5)
        
        # Textbox1
        vbox06 = wx.BoxSizer(wx.VERTICAL)     
        vbox06.AddSpacer(5)
        self.txt1 = wx.TextCtrl(self, size=(80,30), style=wx.NO_BORDER|wx.TE_CENTER)
        self.txt1.SetMaxLength(3)    #Maximum of 3 characters
        self.txt1.SetFont(self.parent_frame.font)        
        self.txt1.SetValue('k')
        self.txt1.SetForegroundColour((255,131,79))
        self.txt1.SetBackgroundColour((85,85,80))
        self.txt1.Bind(wx.EVT_SET_FOCUS, self.OnTxtFocus)
        vbox06.Add(self.txt1)
        self.hbox00.Add(vbox06,1,wx.LEFT|wx.BOTTOM,5)
         
#         # Textbox2
#         vbox09 = wx.BoxSizer(wx.VERTICAL)     
#         vbox09.AddSpacer(5)
#         self.txt2 = wx.TextCtrl(self, size=(50,30), style=wx.NO_BORDER|wx.TE_CENTER)
#         self.txt2.SetMaxLength(3)    #Maximum of 3 characters
#         self.txt2.SetFont(self.parent_frame.font)
#         self.txt2.SetForegroundColour((255,131,79))
#         self.txt2.SetBackgroundColour((85,85,80))
#         vbox09.Add(self.txt2)
#         self.hbox00.Add(vbox09,1,wx.TOP|wx.LEFT,10)
        
        self.sizer.Add(self.hbox00)
        
        # Node/Edge radio buttons
        vbox03 = wx.BoxSizer(wx.VERTICAL)
        self.radio_node = wx.RadioButton(self, -1, "Node", style=wx.RB_GROUP) 
        self.radio_edge = wx.RadioButton(self, -1, "Edge")
        vbox03.Add(self.radio_node) 
        vbox03.Add(self.radio_edge) 
        self.hbox01.Add(vbox03,0,wx.TOP,5)    
        self.radio_node.Bind(wx.EVT_RADIOBUTTON, self.OnSelectNode)
        self.radio_edge.Bind(wx.EVT_RADIOBUTTON, self.OnSelectEdge)
        
        # Textbox
        vbox06 = wx.BoxSizer(wx.VERTICAL)     
        vbox06.AddSpacer(5)
        self.txt = wx.TextCtrl(self, size=(50,30), style=wx.NO_BORDER|wx.TE_CENTER)
        self.txt.SetMaxLength(3)    #Maximum of 3 characters
        self.txt.SetFont(self.parent_frame.font)
        self.txt.SetForegroundColour((255,131,79))
        self.txt.SetBackgroundColour((85,85,80))
        vbox06.Add(self.txt)
        self.hbox01.Add(vbox06,1,wx.TOP|wx.LEFT,10)
         
        # Go button
        vbox09 = wx.BoxSizer(wx.VERTICAL)     
        vbox09.AddSpacer(5)
        self.btn_go = wx.Button(self, label="Go", size=(50,30))        
        self.btn_go.Bind(wx.EVT_BUTTON, self.OnGotoNode)
        self.btn_go.Enable(False)
        self.GetParent().buttons.append(self.btn_go) 
        vbox09.Add(self.btn_go)        
        self.hbox01.Add(vbox09,1,wx.TOP|wx.LEFT,10)        
        
        self.sizer.Add(self.hbox01)
        
        # Tour button
        vbox10 = wx.BoxSizer(wx.VERTICAL)   
        self.btn_tour = wx.Button(self, label="Do Tour", size=BUTTON_SIZE)        
        self.btn_tour.Bind(wx.EVT_BUTTON, self.OnTour)
        self.btn_tour.Enable(False)
        self.GetParent().buttons.append(self.btn_tour) 
        vbox10.Add(self.btn_tour)        
        self.sizer.Add(vbox10,1,wx.TOP,10)        
        
        
        self.SetSizer(self.sizer)  
        self.Layout()        
        
#---------------------------------------------------------------------------------------------#    
#    Draws the background for the 'explore' panel  (unused for now)                           #
#---------------------------------------------------------------------------------------------#       
    def DrawBG(self, size):
        x = size[0]
        y = size[1]
        bmp = wx.EmptyBitmap(x,y)
        dc = wx.MemoryDC()
        dc.SelectObject(bmp)
        
        solidbrush = wx.Brush(BUTTON_COLOR, wx.SOLID)
        dc.SetBrush(solidbrush)        
        dc.DrawRectangle(-1, -1, x+2, y+2)
        solidbrush = wx.Brush(BG_COLOR, wx.SOLID)
        dc.SetBrush(solidbrush)        
        dc.DrawRectangle(3, 3, x-6, y-6)
        
        return wx.StaticBitmap(self, -1, bmp, (0, 0))
    
    def OnTxtFocus(self, event):
        event.GetEventObject().Clear()

#---------------------------------------------------------------------------------------------#    
#    Functions for the radio buttons. Determines if we're looking for nodes or edges.         #
#---------------------------------------------------------------------------------------------#         
    def OnSelectNode(self, event):
        self.btn_go.Bind(wx.EVT_BUTTON, None)
        self.btn_go.Bind(wx.EVT_BUTTON, self.OnGotoNode)        
    def OnSelectEdge(self, event):
        self.btn_go.Bind(wx.EVT_BUTTON, None)
        self.btn_go.Bind(wx.EVT_BUTTON, self.OnGotoEdge)
        
#---------------------------------------------------------------------------------------------#    
#    Selects a node and zooms in on it.                                                       #
#    Accepts an integer X from user input. If X is positive, zooms in on Node #X. If X is     #
#    negative, zooms in on Node #(NumTotalNodes - X)                                          #
#---------------------------------------------------------------------------------------------#    
    def OnGotoNode(self, event):
        txt = self.txt.GetValue()           
        try:
            ID = int(txt)            
            try:
                magnification = self.zp.image_width / 250
                node = self.zp.nodelist[ID]                
                self.zp.SelectOneNode(self.zp.graphics_nodes[ID], True)                
                self.zp.Zoom(node.coords, magnification)
            except IndexError:
                dlg = wx.MessageDialog(self,
                "Node %s does not exist." % str(ID), "Error", wx.ICON_ERROR)
                dlg.ShowModal() 
                dlg.Destroy()
            
        except ValueError:
            dlg = wx.MessageDialog(self,
                "Please enter an integer value", "Error", wx.ICON_ERROR)
            dlg.ShowModal() 
            dlg.Destroy()
            
#---------------------------------------------------------------------------------------------#    
#    Selects an edge and zooms in on it.                                                      #
#    Accepts an integer X from user input. If X is positive, zooms in on Edge #X. If X is     #
#    negative, zooms in on Edge #(NumTotalEdges - X)                                          #
#---------------------------------------------------------------------------------------------#     
    def OnGotoEdge(self, event):
        txt = self.txt.GetValue()            
        try:
            ID = int(txt)            
            try:
                magnification = self.zp.image_width / 250                
                edge = self.zp.edgelist[ID] 
                self.zp.SelectOneEdge(self.zp.graphics_edges[ID], True)
                end1 = self.zp.nodelist[int(edge.node1)].coords
                end2 = self.zp.nodelist[int(edge.node2)].coords
                
                x = int( (math.fabs( end1[0]+end2[0])) /2 )   
                y = int( (math.fabs( end1[1]+end2[1])) /2 )                  
                self.zp.Zoom((x,y), magnification)
            except IndexError:
                dlg = wx.MessageDialog(self,
                "Edge %s does not exist." % str(ID), "Error", wx.ICON_ERROR)
                dlg.ShowModal() 
                dlg.Destroy()
            
        except ValueError:
            dlg = wx.MessageDialog(self,
                "Please enter a positive integer value", "Error", wx.ICON_ERROR)
            dlg.ShowModal() 
            dlg.Destroy()
            
#---------------------------------------------------------------------------------------------#    
#                                                                                             #
#---------------------------------------------------------------------------------------------#             
    def OnGenerateGraph(self, event): 
        try:      
            n = int(self.txt0.GetValue())
        except:
            n = 50 #default
        try:      
            k = int(self.txt1.GetValue())
        except:
            k = 5 #default
        try:      
            d = int(self.txt2.GetValue())
        except:
            d = 25 #default
        self.zp.GenerateGraph(n,k,d)        
                   
#---------------------------------------------------------------------------------------------#    
#    TODO: Publish some stuff                                                                 #
#---------------------------------------------------------------------------------------------#             
    def OnTour(self, event):                       
#         self.parent_Frame.tt.paused = True
        self.parent_frame.pb.PublishTour()
    
if __name__ == '__main__':
    app = wx.App(False)
    wx.Log_SetActiveTarget(wx.LogStderr())
    frame = MainFrame(None, "Map Viewer")
    frame.Show()
    app.MainLoop()