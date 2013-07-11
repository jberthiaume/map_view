#!/usr/bin/env python

'''
Created on May 30, 2013
'''

import wx
import os, time, shutil
import math
import ROSNode
from datetime import datetime
from MapFrame import MapFrame

APP_SIZE        = (240,392)
APP_SIZE_EXP    = (240,620)
BUTTON_COLOR    = (119,41,83)
BUTTON_SIZE     = (180,30)
BUTTON_SIZE_SM  = (85,30)
# TXT_FG_COLOR    = (255,131,79)
TXT_FG_COLOR    = (221,72,20)
TXT_BG_COLOR    = (185,185,180)
BG_COLOR        = (205,205,205)
H_SPACER_WIDTH  = 20
V_SPACER_SMALL  = 10
V_SPACER_LARGE  = 15
SIZER_BORDER    = 10

#TOOD: BUG: redraw state gets stuck on False randomly (hard to reproduce)

#TODO: make edge tool icon

#TODO: move gg/tour/find to mapframe

class MainFrame(wx.Frame):
    def __init__(self, parent, title):
        wx.Frame.__init__(self, parent, title=title, size=APP_SIZE,
                        style=wx.FRAME_SHAPED
                        )
        
        self.screensize = wx.GetDisplaySize()
        self.verbose = False
        self.leftDown = False                                 
        self.font = wx.Font(pointSize=14, family=wx.FONTFAMILY_DEFAULT, 
                       style=wx.FONTSTYLE_NORMAL, weight=wx.FONTWEIGHT_NORMAL, 
                       faceName="lucida sans")    
        
        self.ros = ROSNode.ROSNode(self)  
        self.mp = MainPanel(self)
        self.sp = SettingsPanel(self)  
        
        self.mp.Show()
        self.sp.Hide()
        self.ros.SetAttributes()

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.mp,1,wx.EXPAND)
        self.sizer.Add(self.sp,1,wx.EXPAND)
        self.SetSizer(self.sizer)     
          
        self.Bind(wx.EVT_MOTION, self.OnMouse)
        self.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        self.Bind(wx.EVT_LEFT_UP, self.OnLeftUp) 
        self.Bind(wx.EVT_SIZE, self.OnResize)
        
        self.SetPosition((0,0))  
        self.Layout() 
        self.mp.ep.size = self.mp.ep.GetSize()
        self.ros.Listen()       
        
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
        
    def OnResize(self, event):
        pass

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
        
        
class MainPanel(wx.Panel):
    def __init__(self, parent):        
        wx.Panel.__init__(self, parent=parent)        
        self.bg = self.DrawBG(APP_SIZE)
        
        self.leftDown = False
        self.saved = True
        self.buttons = []
        self.btn_disabled = []
        self.contents = []
        
        self.gg_const = (100,5,20,5,80)
        
        # Set parent frame value
        self.parent_frame = parent 
        while self.parent_frame.GetParent() is not None: 
            self.parent_frame = self.parent_frame.GetParent()
        
        self.ros = self.parent_frame.ros
        self.verbose = self.parent_frame.verbose
        
        # Create the sizers 
        self.sizer_main = wx.BoxSizer(wx.HORIZONTAL)         
        self.sizer_menu = wx.BoxSizer(wx.VERTICAL)      
        self.sizer_display = wx.BoxSizer(wx.VERTICAL)
        
        # The map viewer panel
        zp_size = self.parent_frame.screensize[1]-60     
        self.zp=MapFrame(self, title="Map View",
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
        self.btn_rf.Enable(False)        
        self.btn_rf.Bind(wx.EVT_BUTTON, self.OnRefreshMap)
        self.buttons.append(self.btn_rf) 
        hbox00.Add(self.btn_rf)        
        self.sizer_menu.Add(hbox00,0,wx.TOP|wx.LEFT|wx.RIGHT,SIZER_BORDER) 
        
        # Open button
        hbox10 = wx.BoxSizer(wx.HORIZONTAL)            
        hbox10.AddSpacer(H_SPACER_WIDTH)
        self.btn_open = wx.Button(self, label="Open Map File", size=BUTTON_SIZE)   
        self.buttons.append(self.btn_open)        
        self.btn_open.Bind(wx.EVT_BUTTON, self.OnOpen)  
        hbox10.Add(self.btn_open)           
        self.sizer_menu.Add(hbox10,0,
                            wx.TOP|wx.LEFT|wx.RIGHT,SIZER_BORDER) 
        
        # Save button
        hbox13 = wx.BoxSizer(wx.HORIZONTAL)            
        hbox13.AddSpacer(H_SPACER_WIDTH)
        self.btn_sv = wx.Button(self, label="Save", size=BUTTON_SIZE_SM) 
        self.btn_sv.Bind(wx.EVT_BUTTON, self.OnSave)        
        self.btn_disabled.append(self.btn_sv)     
        self.buttons.append(self.btn_sv)   
        hbox13.Add(self.btn_sv,0,wx.RIGHT, SIZER_BORDER)      
                
        # Save As button
        self.btn_svas = wx.Button(self, label="Save As", size=BUTTON_SIZE_SM) 
        self.btn_svas.Bind(wx.EVT_BUTTON, self.OnSaveAs)        
        self.btn_disabled.append(self.btn_svas)     
        self.buttons.append(self.btn_svas)   
        hbox13.Add(self.btn_svas)           
        self.sizer_menu.Add(hbox13,0,
                            wx.TOP|wx.LEFT|wx.RIGHT
                            ,SIZER_BORDER) 

        # Show/Hide map viewer button
#         hbox03 = wx.BoxSizer(wx.HORIZONTAL)            
#         hbox03.AddSpacer(H_SPACER_WIDTH)
        self.btn_map = wx.Button(self, label="Show Map", size=BUTTON_SIZE)        
        self.btn_map.Bind(wx.EVT_BUTTON, self.OnShowHideMap)   
        self.btn_disabled.append(self.btn_map)   
        self.buttons.append(self.btn_map)   
#         hbox03.Add(self.btn_map)        
#         self.sizer_menu.Add(hbox03,0,
#                             wx.TOP|wx.LEFT|wx.RIGHT
#                             ,SIZER_BORDER)  
        self.btn_map.Hide()
               
#         # Explore button
#         hbox06 = wx.BoxSizer(wx.HORIZONTAL)     
#         hbox06.AddSpacer(H_SPACER_WIDTH)
#         self.btn_exp = wx.Button(self, label="Explore...", size=BUTTON_SIZE)        
#         self.btn_exp.Bind(wx.EVT_BUTTON, self.OnExplore)
#         self.btn_disabled.append(self.btn_exp) 
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

 
#         
#         self.sizer_menu.AddSpacer(V_SPACER_LARGE)        
                 
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
        self.EnableButtons(self.btn_disabled, False)  
                
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
            
            
    def EnableButtons(self, btn_list, boolean):
        for btn in btn_list:
            btn.Enable(boolean)
            
    def EnableMenuOptions(self, key_list, boolean):
        for key in key_list:
            self.parent_frame.file_menu.Enable(key, boolean)
            self.zp.file_menu.Enable(key, boolean)
                
#---------------------------------------------------------------------------------------------#    
#    Starts a listener process which listens on the "/map" topic. Once the listener has       #
#    exported the map file, it is passed to ZoomPanel, which sets the image in the viewer     #
#---------------------------------------------------------------------------------------------#                
    def OnRefreshMap(self, event):
        self.ros.refresh = False
        
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
                self.zp.ClearGraph()
            dlg.Destroy()
        
        self.zp.Hide()       
        wx.Yield() 
        msg = "Retrieving map..."
        self.zp.SetBusyDialog(msg)  
        wx.Yield() 
        if self.verbose is True:        
            print "Retrieving data from /map topic..."  
                
        try:
            map_file = self.ros.GetDefaultFilename()
            self.parent_frame.SetTitle("%s" % map_file)
            
            # Update some statuses
            self.EnableButtons(self.btn_disabled, True)
            self.SetSaveStatus(False)     
            
            while self.ros.image is None:
                time.sleep(0.5)                  
            self.zp.SetImage(self.ros.image)
            
        except IndexError:
            # Image not found
            pass        
        
        self.zp.Show()   
        self.zp.KillBusyDialog()
        wx.EndBusyCursor()
        self.btn_map.SetLabel("Hide Map")         
        self.Layout()           
       
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
        pass
#         if self.ep.IsShown():
#             self.ep.Hide()
#             self.parent_frame.SetSize(APP_SIZE)            
#             self.bg = self.DrawBG(APP_SIZE)
#             self.Show()
#             self.Layout()
#         else:
#             self.ep.txt.Clear()            
#             self.parent_frame.SetSize(APP_SIZE)
#             self.bg = self.DrawBG(APP_SIZE)
#             for b in self.buttons:
#                 b.Show()
#             self.ep.Show()
#             self.Layout()
            

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
                msg = "Loading map..."
                self.zp.SetBusyDialog(msg)
                wx.BeginBusyCursor()
                wx.Yield()
#                 self.zp.NavCanvas.Hide()
                self.zp.Hide()  
                wx.Yield() 
                self.zp.ClearGraph()
                
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
                    self.zp.ClearGraph()
                    self.zp.ImportGraph(None)
                                  
                self.zp.SetImage(filename)  
                wx.EndBusyCursor()  
                self.zp.KillBusyDialog()  
                self.EnableButtons(self.btn_disabled, True)              
                self.SetSaveStatus(True) 
                self.btn_map.SetLabel("Hide Map")
                
                et = datetime.now()
                self.zp.Show()
#                 self.zp.NavCanvas.Show()
                if self.verbose is True:
                    print "Loaded map in %s seconds" % (et-st)
                            
            dlg.Destroy()
     
            
#---------------------------------------------------------------------------------------------#    
#    Saves the current map, overwriting the old version.                                      #
#---------------------------------------------------------------------------------------------#   
    def OnSave(self, event):    
        current_map = self.zp.current_map 
        if current_map is [] or os.path.basename(current_map)==self.ros.GetDefaultFilename():
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
            self.zp.current_map = filename
            
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
        self.parent_frame.SetMinSize(APP_SIZE_EXP)
        self.parent_frame.SetSize(APP_SIZE_EXP)    
        self.parent_frame.sp.Show()
        self.parent_frame.sp.Layout()
        self.parent_frame.Layout()
        self.Layout()        
    
    def OnCloseMap(self, event):  
        if self.saved is False:
            dlg = wx.MessageDialog(self,
            "The current map is unsaved.\nWould you like to save it before closing?", 
            "Warning", wx.YES|wx.NO|wx.CANCEL)
            
            result = dlg.ShowModal()
            if result == wx.ID_YES:
                self.OnSaveAs(event)
            elif result == wx.ID_CANCEL:
                dlg.Destroy()
                return
            dlg.Destroy()
        
        self.zp.ClearGraph()      
        self.zp.Clear()
        self.zp.Hide()
        self.btn_map.SetLabel("Show Map")
        self.EnableButtons(self.btn_disabled, False)
        self.SetSaveStatus(True)

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
        
        self.lbl_titles = []
        self.lbl_text = []
        self.txtbxs = []
        self.ros = self.parent_frame.ros
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        
        title_font = self.parent_frame.font
        title_font.SetPointSize(12)
        title_font.SetWeight(wx.BOLD)
        
        # Title Label 1  
        self.lbl_gg = wx.StaticText(self, label="Graph Generation Settings", 
                                    size=(243,30), pos=(20,10), style=wx.CENTER)
        
        self.lbl_gg.SetFont(title_font)
        self.lbl_titles.append(self.lbl_gg)
        
        # LabelN
        self.lbl_n1 = wx.StaticText(self, label="Generate",
                                   size=(70,30), pos=(20,45))
        self.lbl_text.append(self.lbl_n1)
        
        self.txt_n = wx.TextCtrl(self, size=(32,25), pos=(89,42),
                                 style=wx.NO_BORDER|wx.TE_CENTER)
        self.txt_n.SetMaxLength(3)    #Maximum of 3 characters
        self.txt_n.SetFont(self.parent_frame.font)  
        self.txt_n.SetValue( str(self.GetParent().mp.gg_const[0]) )
        self.txt_n.Bind(wx.EVT_SET_FOCUS, self.OnTxtFocus)
        

        self.lbl_n2 = wx.StaticText(self, label="nodes",
                                   size=(-1,30), pos=(128,45))
        self.lbl_text.append(self.lbl_n2)
        
        
        # LabelD
        self.lbl_d1 = wx.StaticText(self, label="There must be at least",                                                
                                   size=(150,20), pos=(20,80))
        self.lbl_text.append(self.lbl_d1)

        self.txt_d = wx.TextCtrl(self, size=(32,25), pos=(171,77), 
                                 style=wx.NO_BORDER|wx.TE_CENTER)
        self.txt_d.SetMaxLength(3)    #Maximum of 3 characters
        self.txt_d.SetFont(self.parent_frame.font)  
        self.txt_d.SetValue( str(self.GetParent().mp.gg_const[2]) )
        self.txt_d.Bind(wx.EVT_SET_FOCUS, self.OnTxtFocus)

        self.lbl_d2 = wx.StaticText(self, label="px",
                                   size=(20,20), pos=(205,80))
        self.lbl_text.append(self.lbl_d2)

        self.lbl_d3 = wx.StaticText(self, label="of space between nodes",
                                   size=(220,20), pos=(20,100))
        self.lbl_text.append(self.lbl_d3)
        
        
        # LabelW
        self.lbl_w1 = wx.StaticText(self, label="There must be at least",
                                   size=(150,20), pos=(20,135))
        self.lbl_text.append(self.lbl_w1)

        self.txt_w = wx.TextCtrl(self, size=(32,25), pos=(171,132),
                                 style=wx.NO_BORDER|wx.TE_CENTER)
        self.txt_w.SetMaxLength(3)    #Maximum of 3 characters
        self.txt_w.SetFont(self.parent_frame.font)  
        self.txt_w.SetValue( str(self.GetParent().mp.gg_const[3]) )   
        self.txt_w.Bind(wx.EVT_SET_FOCUS, self.OnTxtFocus)

        self.lbl_w2 = wx.StaticText(self, label="px",
                                   size=(20,20), pos=(205,135))
        self.lbl_text.append(self.lbl_w2)

        self.lbl_w3 = wx.StaticText(self, label="between nodes and obstacles",
                                   size=(220,20), pos=(20,155))
        self.lbl_text.append(self.lbl_w3)   

     
        # LabelK
        self.lbl_k1 = wx.StaticText(self, label="Scan",
                                   size=(32,30), pos=(20,190))
        self.lbl_text.append(self.lbl_k1)

        self.txt_k = wx.TextCtrl(self, size=(32,25), pos=(55,187),
                                 style=wx.NO_BORDER|wx.TE_CENTER)
        self.txt_k.SetMaxLength(2)    #Maximum of 2 characters
        self.txt_k.SetFont(self.parent_frame.font)  
        self.txt_k.SetValue( str(self.GetParent().mp.gg_const[1]) )
        self.txt_k.Bind(wx.EVT_SET_FOCUS, self.OnTxtFocus)

        self.lbl_k2 = wx.StaticText(self, label="neighbors per node",
                                   size=(-1,30), pos=(90,190))
        self.lbl_text.append(self.lbl_k2)
        
        
        # LabelE
        self.lbl_e1 = wx.StaticText(self, label="Search no further than",
                                   size=(158,20), pos=(20,225))
        self.lbl_text.append(self.lbl_e1)

        self.txt_e = wx.TextCtrl(self, size=(32,25), pos=(171,222),
                                 style=wx.NO_BORDER|wx.TE_CENTER)
        self.txt_e.SetMaxLength(3)    #Maximum of 3 characters
        self.txt_e.SetFont(self.parent_frame.font)  
        self.txt_e.SetValue( str(self.GetParent().mp.gg_const[4]) )
        self.txt_e.Bind(wx.EVT_SET_FOCUS, self.OnTxtFocus)

        self.lbl_e2 = wx.StaticText(self, label="px",
                                   size=(20,20), pos=(205,225))
        self.lbl_text.append(self.lbl_e2)

        self.lbl_e3 = wx.StaticText(self, label="away for neighboring nodes",
                                   size=(220,20), pos=(20,245))
        self.lbl_text.append(self.lbl_e3)
        
        self.txtbxs.append(self.txt_n)        
        self.txtbxs.append(self.txt_k)
        self.txtbxs.append(self.txt_d)        
        self.txtbxs.append(self.txt_w)
        self.txtbxs.append(self.txt_e)
        
        # Edges Checkbox
        self.chk_edge = wx.CheckBox(self, label="Allow edge generation\nwithin 5 px of obstacles",
                                    pos=(20,280))
        self.chk_edge.SetValue(False)
        
        # Unknown Edges Checkbox
        self.chk_uk = wx.CheckBox(self, label="Allow edge generation\nthrough unknown map areas",
                                    pos=(20,325))
        self.chk_uk.SetValue(True)
        
        # Clear Checkbox
        self.chk_clr = wx.CheckBox(self, label="Clear nodes and edges\nwhen generating new graph",
                                    pos=(20,370))
        self.chk_clr.SetValue(True)
        
        
        # Title Label 2 
        self.lbl_st = wx.StaticText(self, label="Other Settings", 
                                    size=(240,30), pos=(10,425), style=wx.CENTER)
        
        self.lbl_st.SetFont(title_font)
        self.lbl_titles.append(self.lbl_st) 
        
        # Edge Creation Checkbox
        self.chk_ec = wx.CheckBox(self, label="Automatically connect nodes",
                                  pos=(10,453))
        self.chk_ec.SetValue(True)
        
        # Intersection Checkbox
        self.chk_int = wx.CheckBox(self, label="Automatically create nodes\nat edge intersections",
                                  pos=(10,485))
        self.chk_int.SetValue(True)
        
        # Console Output Checkbox
        self.chk_co = wx.CheckBox(self, label="Enable console output",
                                  pos=(10,530))
        self.chk_co.SetValue(True)
                
        # Ok button
        btn_ok = wx.Button(self, label="Accept", size=(95,30), 
                           pos=(20, APP_SIZE_EXP[1]-45))  
        self.parent_frame.mp.buttons.append(btn_ok) 
        btn_ok.Bind(wx.EVT_BUTTON, self.OnOk) 
        
        # Cancel button
        btn_cnc = wx.Button(self, label="Cancel", size=(95,30), 
                           pos=(125, APP_SIZE_EXP[1]-45))  
        self.parent_frame.mp.buttons.append(btn_cnc) 
        btn_cnc.Bind(wx.EVT_BUTTON, self.OnCancel) 
                
        # Mouse capturing events
        self.bg.Bind(wx.EVT_MOTION, self.OnMouse)
        self.bg.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        self.bg.Bind(wx.EVT_LEFT_UP, self.OnLeftUp)        
        self.Bind(wx.EVT_MOTION, self.OnMouse)
        self.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        self.Bind(wx.EVT_LEFT_UP, self.OnLeftUp)   
        
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
        
        dc.DrawLine(20, 70,220, 70)
        dc.DrawLine(20,125,220,125)
        dc.DrawLine(20,180,220,180)
        dc.DrawLine(20,215,220,215)
        dc.DrawLine(20,270,220,270)
        
        return wx.StaticBitmap(self, -1, bmp, (0, 0))
    
#---------------------------------------------------------------------------------------------#    
#    Sets the foreground and background color of all labels on the settings panel             #
#---------------------------------------------------------------------------------------------#        
    def PaintLabels(self, foreground, background):
        try:
            for lbl in self.lbl_titles:
                lbl.SetForegroundColour(foreground)
                lbl.SetBackgroundColour(background)
            for lbl in self.lbl_text:
                pass    #placeholder
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
                txt.Bind(wx.EVT_SET_FOCUS, self.OnTxtFocus)
                txt.Bind(wx.EVT_KILL_FOCUS, self.OnTxtLoseFocus)
        except IOError:
            print "Invalid argument: expected (R,G,B) value"
            
    
    def OnTxtFocus(self, event):
        event.GetEventObject().Clear()   
        
    def OnTxtLoseFocus(self, event):
        txt = event.GetEventObject()
        if txt.GetValue() == "":
            txt.SetValue(self.GetDefaultValue(txt)) 
            
    def GetDefaultValue(self, txt):
        for i in [i for i,x in enumerate(self.txtbxs) if x == txt]:  
            return str(self.GetParent().mp.gg_const[i])   

#---------------------------------------------------------------------------------------------#    
#    Saves the settings                                                                       #
#---------------------------------------------------------------------------------------------#
    def OnOk(self, event):
        self.parent_frame.mp.zp.spaced_edges = not self.chk_edge.GetValue()
        self.parent_frame.mp.zp.clear_graph = self.chk_clr.GetValue()            
        self.parent_frame.mp.zp.auto_edges = self.chk_ec.GetValue()   
        self.parent_frame.mp.zp.auto_intersections = self.chk_int.GetValue()  
        self.parent_frame.mp.zp.unknown_edges = self.chk_uk.GetValue()    
        self.parent_frame.SuppressOutput(not self.chk_co.GetValue())
        
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
        self.parent_frame.SetMinSize(APP_SIZE)
        self.parent_frame.SetSize(APP_SIZE)   
        self.parent_frame.mp.Show()
        self.parent_frame.mp.Layout()
        self.parent_frame.Layout() 
        
    
    def OnCancel(self, event):
        self.Hide()
        for txt in self.txtbxs:
            txt.SetValue(self.GetDefaultValue(txt))               
        self.parent_frame.SetMinSize(APP_SIZE)
        self.parent_frame.SetSize(APP_SIZE)
        self.parent_frame.mp.Show()        
        self.parent_frame.mp.Layout()
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
        self.GetParent().btn_disabled.append(self.btn_gg)
        self.GetParent().buttons.append(self.btn_gg) 
        vbox00.Add(self.btn_gg)        
        self.sizer.Add(vbox00,1,wx.TOP,10)
        
        # Tour button
        vbox10 = wx.BoxSizer(wx.VERTICAL)   
        self.btn_tour = wx.Button(self, label="Do Tour", size=BUTTON_SIZE)        
        self.btn_tour.Bind(wx.EVT_BUTTON, self.OnTour)
        self.GetParent().btn_disabled.append(self.btn_tour)
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
        vbox06.AddSpacer(7)
        self.txt = wx.TextCtrl(self, size=(45,30), style=wx.NO_BORDER|wx.TE_CENTER)
        self.txt.SetMaxLength(3)    #Maximum of 3 characters
        self.txt.SetFont(self.parent_frame.font)
        self.txt.SetForegroundColour((255,131,79))
        self.txt.SetBackgroundColour((85,85,80))
        vbox06.Add(self.txt)
        self.hbox01.Add(vbox06,1,wx.TOP|wx.LEFT,8)
         
        # Go button
        vbox09 = wx.BoxSizer(wx.VERTICAL)     
        vbox09.AddSpacer(9)
        self.btn_go = wx.Button(self, label="Find", size=(53,32))        
        self.btn_go.Bind(wx.EVT_BUTTON, self.OnGotoNode)
        self.GetParent().btn_disabled.append(self.btn_go)
        self.GetParent().buttons.append(self.btn_go) 
        vbox09.Add(self.btn_go)        
        self.hbox01.Add(vbox09,1,wx.TOP|wx.LEFT,5)        
        
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
#    Publish some stuff                                                                       #
#---------------------------------------------------------------------------------------------#             
    def OnTour(self, event):                       
#         self.parent_Frame.tt.paused = True
        self.parent_frame.ros.PublishTour()
    
if __name__ == '__main__':
    app = wx.App(False)
    
    wx.Log_SetActiveTarget(wx.LogStderr())
    frame = MainFrame(None, "Map Viewer")
    frame.Show()
    app.MainLoop()