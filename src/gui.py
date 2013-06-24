#!/usr/bin/env python

'''
Created on May 30, 2013
'''

import wx
import os, time, shutil
import math
import listener as ls
import publisher as pb
from datetime import datetime
from zoompanel import ZoomPanel

APP_SIZE        = (240,462)
APP_SIZE_EXP    = (240,492)
BUTTON_COLOR    = (119,41,83)
BUTTON_SIZE     = (180,30)
TXT_FG_COLOR    = (255,131,79)
TXT_BG_COLOR    = (85,85,80)
BG_COLOR        = (205,205,205)
H_SPACER_WIDTH  = 20
V_SPACER_SMALL  = 10
V_SPACER_LARGE  = 15
SIZER_BORDER    = 10

#TODO: check variables on save (new nodes start at ID 0 even if nodes were loaded??)
#      (not spotted since 14/6/13, assume bug is fixed for now)

#TODO: opening a file while nodes still on canvas = some node numbers invisible
#      (doesn't seem to break functionality) 

#TODO: ability to move nodes (arrow keys? click/drag?)

#TODO: function to convert (x,y) into image_data array index

#TODO: figure out something to indicate map is loading @ OnOpen

#TODO: doc comments for new functions

class MainFrame(wx.Frame):
    def __init__(self, parent, title):
        wx.Frame.__init__(self, parent, title=title, size=APP_SIZE_EXP,
                        style=wx.FRAME_SHAPED
                        )
        self.resolution = wx.GetDisplaySize()
        self.verbose = False
        self.leftDown = False                                 
        self.font = wx.Font(pointSize=14, family=wx.FONTFAMILY_DEFAULT, 
                       style=wx.FONTSTYLE_NORMAL, weight=wx.FONTWEIGHT_NORMAL, 
                       faceName="lucida sans")    
        
        self.ls = ls.listener(self)
        self.pb = pb.publisher(self) 
#         self.tt = TimerThread(self)       
        self.mp = MainPanel(self)
        self.sp = SettingsPanel(self)  
#         self.sizer = wx.BoxSizer(wx.VERTICAL)
#         self.sizer.Add(self.mp, 1, wx.EXPAND)
        
        self.mp.Show()
        self.sp.Hide()
        self.ls.SetAttributes()
#         self.tt.start()

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.mp,1,wx.EXPAND)
        self.sizer.Add(self.sp,1,wx.EXPAND)
        self.SetSizer(self.sizer)
          
        self.Bind(wx.EVT_MOTION, self.OnMouse)
        self.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        self.Bind(wx.EVT_LEFT_UP, self.OnLeftUp) 
        
        self.SetPosition((0,0)) 
        self.Layout() 
        self.mp.ep.size = self.mp.ep.GetSize()
        
    def SuppressOutput(self, boolean):
        if boolean is True:
            self.verbose = False
            self.mp.zp.verbose = False
        else:
            self.verbose = True
            self.mp.zp.verbose = True

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
        
        self.gg_const = (70,6,35,5,80)
        
        # Set parent frame value
        self.parent_frame = parent 
        while self.parent_frame.GetParent() is not None: 
            self.parent_frame = self.parent_frame.GetParent()
        
        self.ls = self.parent_frame.ls
        self.pb = self.parent_frame.pb
#         self.tt = self.parent_frame.tt
        self.verbose = self.parent_frame.verbose
        
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
                            wx.LEFT|wx.RIGHT
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
                
        # Settings
        hbox19 = wx.BoxSizer(wx.HORIZONTAL)                      
        hbox19.AddSpacer(H_SPACER_WIDTH)
        self.btn_set = wx.Button(self, label="Settings", size=BUTTON_SIZE)        
        self.btn_set.Bind(wx.EVT_BUTTON, self.OnSettings)
        self.buttons.append(self.btn_set) 
        hbox19.Add(self.btn_set)        
        self.sizer_menu.Add(hbox19,0,wx.TOP|wx.LEFT|wx.RIGHT,SIZER_BORDER)
              
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
        if self.verbose is True:        
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
                st = datetime.now()
                wx.BeginBusyCursor()
#                 self.zp.Hide() 
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
#                 self.zp.Show()  
                wx.EndBusyCursor()          
                
                for b in self.buttons:
                    if b.Enabled == False:
                        b.Enable(True)                 
                self.SetSaveStatus(True) 
                self.btn_map.SetLabel("Hide Map")
                
                et = datetime.now()
                if self.verbose is True:
                    print "Total open time: %s" % (et-st)
                            
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
            
            if self.verbose is True:
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
            
            if self.verbose is True:
                print "Saved: %s" % filename            
            self.SetSaveStatus(True) 
            self.parent_frame.SetTitle("%s" % dlg.GetFilename())
                        
        dlg.Destroy()
        
#---------------------------------------------------------------------------------------------#    
#    Accessor function for the current save state (Saved/Unsaved)                            #
#---------------------------------------------------------------------------------------------#         
    def SetSaveStatus(self, bool_save):
        self.saved = bool_save

    
    def OnSettings(self, event):
        self.Hide()
        self.parent_frame.sp.Show()
        self.parent_frame.Layout()
        self.Layout()

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
       
class SettingsPanel(wx.Panel):
    def __init__(self, parent):        
        wx.Panel.__init__(self, parent=parent)
        self.bg = self.DrawBG(APP_SIZE_EXP)
        
        self.leftDown = False     
        self.parent_frame = parent 
        while self.parent_frame.GetParent() is not None: 
            self.parent_frame = self.parent_frame.GetParent()
        
        self.labels = []
        self.txtbxs = []
        self.ls = self.parent_frame.ls
        self.pb = self.parent_frame.pb
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        
        title_font = self.parent_frame.font
        title_font.SetPointSize(12)
        title_font.SetWeight(wx.BOLD)
        
        # Title Label 1
        vbox00 = wx.BoxSizer(wx.VERTICAL)   
        self.lbl_gg = wx.StaticText(self, label="Graph Generation Settings", 
                                    size=(243,30), style=wx.CENTER)
        
        self.lbl_gg.SetFont(title_font)
        self.labels.append(self.lbl_gg)
        vbox00.Add(self.lbl_gg)        
        self.sizer.Add(vbox00,1,wx.TOP|wx.LEFT,15)
        
        # LabelN
        self.hbox00 = wx.BoxSizer(wx.HORIZONTAL)
        self.hbox00.AddSpacer(10)
        vbox03 = wx.BoxSizer(wx.VERTICAL)     
        vbox03.AddSpacer(5)
        self.lbl_n = wx.StaticText(self, label="Number of nodes to generate",
                                   size=(130,35))
#         self.labels.append(self.lbl_n)
        vbox03.Add(self.lbl_n)
        self.hbox00.Add(vbox03,1,wx.LEFT|wx.TOP|wx.BOTTOM|wx.RIGHT,10)
        
        # TextboxN
        vbox06 = wx.BoxSizer(wx.VERTICAL)     
        vbox06.AddSpacer(10)
        self.txt_n = wx.TextCtrl(self, size=(50,30), style=wx.NO_BORDER|wx.TE_CENTER)
        self.txt_n.SetMaxLength(3)    #Maximum of 3 characters
        self.txt_n.SetFont(self.parent_frame.font)  
        self.txt_n.SetValue( str(self.GetParent().mp.gg_const[0]) )
        self.txt_n.Bind(wx.EVT_SET_FOCUS, self.OnTxtFocus)
        self.txtbxs.append(self.txt_n)
        vbox06.Add(self.txt_n)
        self.hbox00.Add(vbox06,1,wx.LEFT|wx.TOP|wx.BOTTOM|wx.RIGHT,5)
        self.sizer.Add(self.hbox00)
        
        # LabelK
        self.hbox02 = wx.BoxSizer(wx.HORIZONTAL)
        self.hbox02.AddSpacer(10)
        vbox03 = wx.BoxSizer(wx.VERTICAL)     
        vbox03.AddSpacer(5)
        self.lbl_k = wx.StaticText(self, label="Number of neighbors to scan",
                                   size=(130,35))
#         self.labels.append(self.lbl_k)
        vbox03.Add(self.lbl_k)
        self.hbox02.Add(vbox03,1,wx.LEFT|wx.BOTTOM|wx.RIGHT,10)
        
        # TextboxK
        vbox06 = wx.BoxSizer(wx.VERTICAL)     
        vbox06.AddSpacer(5)
        self.txt_k = wx.TextCtrl(self, size=(50,30), style=wx.NO_BORDER|wx.TE_CENTER)
        self.txt_k.SetMaxLength(3)    #Maximum of 3 characters
        self.txt_k.SetFont(self.parent_frame.font)  
        self.txt_k.SetValue( str(self.GetParent().mp.gg_const[1]) )
        self.txt_k.Bind(wx.EVT_SET_FOCUS, self.OnTxtFocus)
        self.txtbxs.append(self.txt_k)
        vbox06.Add(self.txt_k)
        self.hbox02.Add(vbox06,1,wx.LEFT|wx.BOTTOM|wx.RIGHT,5)
        self.sizer.Add(self.hbox02)
        
        # LabelD
        self.hbox04 = wx.BoxSizer(wx.HORIZONTAL)
        self.hbox04.AddSpacer(10)
        vbox03 = wx.BoxSizer(wx.VERTICAL)     
        vbox03.AddSpacer(5)
        self.lbl_d = wx.StaticText(self, label="Minimum distance between nodes (px)",
                                   size=(135,35))
#         self.labels.append(self.lbl_d)
        vbox03.Add(self.lbl_d)
        self.hbox04.Add(vbox03,1,wx.LEFT|wx.BOTTOM|wx.RIGHT,10)
        
        # TextboxD
        vbox06 = wx.BoxSizer(wx.VERTICAL)     
        vbox06.AddSpacer(5)
        self.txt_d = wx.TextCtrl(self, size=(50,30), style=wx.NO_BORDER|wx.TE_CENTER)
        self.txt_d.SetMaxLength(3)    #Maximum of 3 characters
        self.txt_d.SetFont(self.parent_frame.font)  
        self.txt_d.SetValue( str(self.GetParent().mp.gg_const[2]) )
        self.txt_d.Bind(wx.EVT_SET_FOCUS, self.OnTxtFocus)
        self.txtbxs.append(self.txt_d)
        vbox06.Add(self.txt_d)
        self.hbox04.Add(vbox06,1,wx.BOTTOM|wx.RIGHT,5) 
        self.sizer.Add(self.hbox04) 
        
        # LabelW
        self.hbox05 = wx.BoxSizer(wx.HORIZONTAL)
        self.hbox05.AddSpacer(10)
        vbox03 = wx.BoxSizer(wx.VERTICAL)     
        vbox03.AddSpacer(5)
        self.lbl_w = wx.StaticText(self, label="Minimum distance to wall (px)",
                                   size=(130,35))
#         self.labels.append(self.lbl_w)
        vbox03.Add(self.lbl_w)
        self.hbox05.Add(vbox03,1,wx.LEFT|wx.BOTTOM|wx.RIGHT,10)
        
        # TextboxW
        vbox06 = wx.BoxSizer(wx.VERTICAL)     
        vbox06.AddSpacer(5)
        self.txt_w = wx.TextCtrl(self, size=(50,30), style=wx.NO_BORDER|wx.TE_CENTER)
        self.txt_w.SetMaxLength(3)    #Maximum of 3 characters
        self.txt_w.SetFont(self.parent_frame.font)  
        self.txt_w.SetValue( str(self.GetParent().mp.gg_const[3]) )        
        self.txtbxs.append(self.txt_w)
        self.txt_w.Bind(wx.EVT_SET_FOCUS, self.OnTxtFocus)
        vbox06.Add(self.txt_w)
        self.hbox05.Add(vbox06,1,wx.LEFT|wx.BOTTOM|wx.RIGHT,5) 
        self.sizer.Add(self.hbox05)
        
        # LabelE
        self.hbox06 = wx.BoxSizer(wx.HORIZONTAL)
        self.hbox06.AddSpacer(10)
        vbox03 = wx.BoxSizer(wx.VERTICAL)     
        vbox03.AddSpacer(5)
        self.lbl_e = wx.StaticText(self, label="Maximum scan radius (px)",
                                   size=(130,35))
#         self.labels.append(self.lbl_e)
        vbox03.Add(self.lbl_e)
        self.hbox06.Add(vbox03,1,wx.LEFT|wx.BOTTOM|wx.RIGHT,10)
        
        # TextboxE
        vbox06 = wx.BoxSizer(wx.VERTICAL)     
        vbox06.AddSpacer(5)
        self.txt_e = wx.TextCtrl(self, size=(50,30), style=wx.NO_BORDER|wx.TE_CENTER)
        self.txt_e.SetMaxLength(3)    #Maximum of 3 characters
        self.txt_e.SetFont(self.parent_frame.font)  
        self.txt_e.SetValue( str(self.GetParent().mp.gg_const[4]) )
        self.txt_e.Bind(wx.EVT_SET_FOCUS, self.OnTxtFocus)
        self.txtbxs.append(self.txt_e)
        vbox06.Add(self.txt_e)
        self.hbox06.Add(vbox06,1,wx.LEFT|wx.BOTTOM|wx.RIGHT,5)   
        self.sizer.Add(self.hbox06)
        
        self.sizer.AddSpacer(15)  
        
        # Title Label 2
        vbox10 = wx.BoxSizer(wx.VERTICAL)   
        self.lbl_gg = wx.StaticText(self, label="Other Settings", 
                                    size=(243,30), style=wx.CENTER)
        
        self.lbl_gg.SetFont(title_font)
        self.labels.append(self.lbl_gg)
        vbox10.Add(self.lbl_gg)        
        self.sizer.Add(vbox10,1,wx.TOP|wx.LEFT,15) 
        
        # Edge Creation Checkbox
        vbox13 = wx.BoxSizer(wx.VERTICAL)
        self.chk_ec = wx.CheckBox(self, label="Automatically connect nodes")
        self.chk_ec.SetValue(True)
        vbox13.Add(self.chk_ec)
        self.sizer.Add(vbox13,1,wx.TOP|wx.LEFT,10)
        
        # Console Output Checkbox
        vbox13 = wx.BoxSizer(wx.VERTICAL)
        self.chk_co = wx.CheckBox(self, label="Enable console output")
        self.chk_co.SetValue(False)
        vbox13.Add(self.chk_co)
        self.sizer.Add(vbox13,1,wx.LEFT|wx.TOP,10)
        
        self.sizer.AddSpacer(50) 
        
        # Ok button
        hbox30 = wx.BoxSizer(wx.HORIZONTAL)             
        hbox30.AddSpacer(H_SPACER_WIDTH)
        btn_ok = wx.Button(self, label="Ok", size=BUTTON_SIZE)  
        self.parent_frame.mp.buttons.append(btn_ok)      
        hbox30.Add(btn_ok)           
        btn_ok.Bind(wx.EVT_BUTTON, self.OnOk)
        self.sizer.Add(hbox30,0,wx.TOP|wx.LEFT|wx.RIGHT|wx.BOTTOM,SIZER_BORDER)  
                
        # Mouse capturing events
        self.bg.Bind(wx.EVT_MOTION, self.OnMouse)
        self.bg.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        self.bg.Bind(wx.EVT_LEFT_UP, self.OnLeftUp)        
        self.Bind(wx.EVT_MOTION, self.OnMouse)
        self.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        self.Bind(wx.EVT_LEFT_UP, self.OnLeftUp)   
        
        self.SetSizer(self.sizer) 
        self.parent_frame.mp.PaintButtons ( (255,255,255),BUTTON_COLOR ) 
        self.PaintLabels(BUTTON_COLOR, BG_COLOR)  
        self.PaintTextboxes(TXT_FG_COLOR, TXT_BG_COLOR)  
        self.Layout()
        
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
#    Sets the foreground and background color of all labels on the settings panel             #
#---------------------------------------------------------------------------------------------#        
    def PaintLabels(self, foreground, background):
        try:
            for lbl in self.labels:
                lbl.SetForegroundColour(foreground)
                lbl.SetBackgroundColour(background)
        except IOError:
            print "Invalid argument: expected (R,G,B) value"
            
#---------------------------------------------------------------------------------------------#    
#    Sets the foreground and background color of all textboxes on the settings panel          #
#---------------------------------------------------------------------------------------------#        
    def PaintTextboxes(self, foreground, background):
        try:
            for txt in self.txtbxs:
                txt.SetForegroundColour(foreground)
                txt.SetBackgroundColour(background)
        except IOError:
            print "Invalid argument: expected (R,G,B) value"
            
    
    def OnTxtFocus(self, event):
        event.GetEventObject().Clear()            
    
#TODO: warnings for "strange" values?
#---------------------------------------------------------------------------------------------#    
#    Saves the settings                                                                       #
#---------------------------------------------------------------------------------------------#
    def OnOk(self, event):        
        if self.chk_ec.GetValue() is True:
            self.parent_frame.mp.zp.autoedges = True
        else:
            self.parent_frame.mp.zp.autoedges = False
            
        if self.chk_co.GetValue() is True:
            self.parent_frame.SuppressOutput(False)
        else:
            self.parent_frame.SuppressOutput(True)
        
        try:
            n = int( self.txt_n.GetValue() )
            k = int( self.txt_k.GetValue() )
            d = int( self.txt_d.GetValue() )
            w = int( self.txt_w.GetValue() )
            e = int( self.txt_e.GetValue() )
        except ValueError:
            dlg = wx.MessageDialog(self,
                "Invalid value", "Error", wx.ICON_ERROR)
            dlg.ShowModal() 
            dlg.Destroy()
            return
            
        if n<0 or k<0 or d<0 or w<0 or e<0:
            dlg = wx.MessageDialog(self,
            "Positive integers only", "Error", wx.ICON_ERROR)
            dlg.ShowModal() 
            dlg.Destroy()
            return       
         
        self.parent_frame.mp.gg_const = (n,k,d,w,e)
        self.parent_frame.mp.zp.gg_const = (n,k,d,w,e)      
        self.Hide()
        self.parent_frame.mp.Show()
        self.parent_frame.Layout()      
        
        
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
        
#         # TextboxN
#         vbox03 = wx.BoxSizer(wx.VERTICAL)     
#         vbox03.AddSpacer(5)
#         self.txt_n = wx.TextCtrl(self, size=(50,30), style=wx.NO_BORDER|wx.TE_CENTER)
#         self.txt_n.SetMaxLength(3)    #Maximum of 3 characters
#         self.txt_n.SetFont(self.parent_frame.font)
#         self.txt_n.SetValue('n')
#         self.txt_n.SetForegroundColour((255,131,79))
#         self.txt_n.SetBackgroundColour((85,85,80))
#         self.txt_n.Bind(wx.EVT_SET_FOCUS, self.OnTxtFocus)
#         vbox03.Add(self.txt_n)
#         self.hbox00.Add(vbox03,1,wx.LEFT|wx.BOTTOM|wx.RIGHT,5)
#         
#         # TextboxK
#         vbox06 = wx.BoxSizer(wx.VERTICAL)     
#         vbox06.AddSpacer(5)
#         self.txt_k = wx.TextCtrl(self, size=(50,30), style=wx.NO_BORDER|wx.TE_CENTER)
#         self.txt_k.SetMaxLength(3)    #Maximum of 3 characters
#         self.txt_k.SetFont(self.parent_frame.font)        
#         self.txt_k.SetValue('k')
#         self.txt_k.SetForegroundColour((255,131,79))
#         self.txt_k.SetBackgroundColour((85,85,80))
#         self.txt_k.Bind(wx.EVT_SET_FOCUS, self.OnTxtFocus)
#         vbox06.Add(self.txt_k)
#         self.hbox00.Add(vbox06,1,wx.LEFT|wx.BOTTOM|wx.RIGHT,5)
#          
#         # TextboxD
#         vbox09 = wx.BoxSizer(wx.VERTICAL)     
#         vbox09.AddSpacer(5)
#         self.txt_d = wx.TextCtrl(self, size=(50,30), style=wx.NO_BORDER|wx.TE_CENTER)
#         self.txt_d.SetMaxLength(3)    #Maximum of 3 characters
#         self.txt_d.SetFont(self.parent_frame.font)       
#         self.txt_d.SetValue('d')
#         self.txt_d.SetForegroundColour((255,131,79))
#         self.txt_d.SetBackgroundColour((85,85,80))
#         self.txt_d.Bind(wx.EVT_SET_FOCUS, self.OnTxtFocus)
#         vbox09.Add(self.txt_d)
#         self.hbox00.Add(vbox09,1,wx.LEFT|wx.BOTTOM,5)
#         
#         self.sizer.Add(self.hbox00)
        
        # Tour button
        vbox10 = wx.BoxSizer(wx.VERTICAL)   
        self.btn_tour = wx.Button(self, label="Do Tour", size=BUTTON_SIZE)        
        self.btn_tour.Bind(wx.EVT_BUTTON, self.OnTour)
        self.btn_tour.Enable(False)
        self.GetParent().buttons.append(self.btn_tour) 
        vbox10.Add(self.btn_tour)        
        self.sizer.Add(vbox10,1,wx.TOP,10) 
        
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
        n=self.GetParent().gg_const[0]
        k=self.GetParent().gg_const[1]
        d=self.GetParent().gg_const[2]
        w=self.GetParent().gg_const[3]
        e=self.GetParent().gg_const[4]
        
        self.zp.GenerateGraph(n,k,d,w,e)        
                   
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