#!/usr/bin/env python

'''
Created on May 30, 2013
'''

import wx
import os, time, shutil
import math
import signal
import ROSNode
import Resources
import subprocess
from datetime import datetime
from MapFrame import MapFrame

APP_SIZE        = (240,425)
APP_SIZE_EXP    = (240,652)
BUTTON_COLOR    = (119,41,83)
BUTTON_SIZE     = (180,30)
BUTTON_SIZE_SM  = (85,30)
TXT_FG_COLOR    = (221,72,20)
TXT_BG_COLOR    = (185,185,180)
BG_COLOR        = (230,230,230)
H_SPACER_WIDTH  = 20
V_SPACER_SMALL  = 10
V_SPACER_LARGE  = 15
SIZER_BORDER    = 10

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
        
#         signal.signal(signal.SIGINT, self.sigint_hdlr)
#         signal.signal(signal.SIGTERM, self.sigterm_hdlr)
        
        self.SetPosition((0,0))  
        self.Layout() 
        self.mp.ep.size = self.mp.ep.GetSize()
        self.ros.Listen()       
    
    # Placeholder function in case I want to reroute stdout at some point   
    def SuppressOutput(self, boolean):
        if boolean:
            self.verbose = False
        else:
            self.verbose = True

#---------------------------------------------------------------------------------------------#    
#    Mouse capturing functions to allow dragging of the control panel around the screen.      #
#---------------------------------------------------------------------------------------------#        
    def OnMouse(self, event):
        if event.Dragging() and self.leftDown:
            pos = self.ClientToScreen(event.GetPosition()) 
            fp = (pos.x - self.delta.x, pos.y - self.delta.y) 
            self.pf.Move(fp)  
            
    def OnLeftDown(self, event): 
        self.CaptureMouse() 
        self.leftDown = True 
        pos = self.ClientToScreen(event.GetPosition()) 
        origin = self.pf.GetPosition() 
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
        self.gg_const = {'n':100, 'k':5, 'd':20, 'w':8, 'e':80}
        
        # Set parent frame value
        self.pf = parent 
        while self.pf.Parent is not None: 
            self.pf = self.pf.Parent
        
        self.ros = self.pf.ros
        self.verbose = self.pf.verbose
        
        # Create the sizers 
        self.sizer_main = wx.BoxSizer(wx.HORIZONTAL)         
        self.sizer_menu = wx.BoxSizer(wx.VERTICAL)      
        self.sizer_display = wx.BoxSizer(wx.VERTICAL)
        
        # The map viewer panel
        mframe_size = self.pf.screensize[1]-60     
        self.mframe=MapFrame(self, title="Map View",
                                  size=((mframe_size,mframe_size)), 
#                                   style=wx.FRAME_SHAPED
                                  )  
        self.mframe.Hide()
        self.mframe.SetPosition((320,0))  
        
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
        self.btn_exit = wx.Button(self, label="Exit", size=BUTTON_SIZE)  
        self.buttons.append(self.btn_exit)      
        hbox20.Add(self.btn_exit)           
        self.btn_exit.Bind(wx.EVT_BUTTON, self.OnExit)
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
            self.pf.file_menu.Enable(key, boolean)
            self.mframe.file_menu.Enable(key, boolean)
                
#---------------------------------------------------------------------------------------------#    
#    Starts a listener process which listens on the "/map" topic. Once the listener has       #
#    exported the map file, it is passed to ZoomPanel, which sets the image in the viewer     #
#---------------------------------------------------------------------------------------------#                
    def OnRefreshMap(self, event):
        self.ros.refresh = False
        
        if not self.saved:
            
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
        
        if len(self.mframe.nodelist) > 0:
            dlg = wx.MessageDialog(self,
            "Do you want to keep the current\nnodes and edges on the updated map?", 
            "Map", wx.YES_NO)
                        
            if dlg.ShowModal() == wx.ID_NO:
                self.mframe.ClearGraph()
            dlg.Destroy()
        
        self.mframe.Hide()       
        wx.Yield() 
        msg = "Retrieving map..."
        self.mframe.SetBusyDialog(msg)  
        wx.Yield() 
        if self.verbose:
            print "Retrieving data from /map topic..."  
                
        try:
            map_file = self.ros.GetDefaultFilename()
            self.mframe.SetTitle("Map Viewer    |    %s" % map_file)
            
            # Update some statuses
            self.EnableButtons(self.btn_disabled, True)
            self.SetSaveStatus(False)     
            
            while self.ros.image is None:
                time.sleep(0.5) 
#             wait_time = 0
#             while self.ros.obstacles == [] and wait_time < 6:
#                 print "sleeping 0.5"
#                 time.sleep(0.5)
#                 wait_time += 1                 
            self.mframe.SetImage(self.ros.image)
            
        except IndexError:
            # Image not found
            pass        
        
        self.mframe.Show()   
        self.mframe.KillBusyDialog()
        wx.EndBusyCursor()
        self.ros.refresh2 = False        
        self.Layout()             

#---------------------------------------------------------------------------------------------#    
#    Shows a file dialog allowing the user to select a map file (.png format)                 #
#---------------------------------------------------------------------------------------------#    
    def OnOpen(self, event): 
          
        if not self.saved:
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
                self.mframe.SetBusyDialog(msg)
                wx.BeginBusyCursor()
                wx.Yield()
#                 self.mframe.NavCanvas.Hide()
                self.mframe.Hide()  
                wx.Yield() 
                self.mframe.ClearGraph()
                
                # Set the viewer image to the selected file
                filename = dlg.GetPath()  
                self.mframe.SetTitle("Map Viewer    |    %s" % dlg.GetFilename()) 
                
                # Import the node data. For this to work, the node file must have the same
                # name as the map file, but with the extension ".graph"
                try:
                    graph_filename = "%sgraph" % filename.rstrip("png")
                    graph_file = open(graph_filename, "r")
                    self.mframe.ImportGraph(graph_file)
                    graph_file.close()
                except IOError:
                    self.mframe.ClearGraph()
                    self.mframe.ImportGraph(None)
                                  
                self.mframe.SetImage(filename)  
                wx.EndBusyCursor()  
                self.mframe.KillBusyDialog()  
                self.EnableButtons(self.btn_disabled, True)              
                self.SetSaveStatus(True)
                
                et = datetime.now()
                self.mframe.Show()
#                 self.mframe.NavCanvas.Show()
#                 if self.verbose:
                print "Loaded map %s. Time taken: %s" % (filename, (et-st))
                            
            dlg.Destroy()
     
            
#---------------------------------------------------------------------------------------------#    
#    Saves the current map, overwriting the old version.                                      #
#---------------------------------------------------------------------------------------------#   
    def OnSave(self, event):   
        st = datetime.now() 
        current_map = self.mframe.current_map 
        if current_map is [] or os.path.basename(current_map)==self.ros.GetDefaultFilename():
            self.OnSaveAs(event)  
        else:
            shutil.move(current_map,current_map)
            
            # The graph filename must be the same as the map filename (except the extension)
            graph_filename = "%sgraph" % current_map.rstrip("png")
            graph_file = open(graph_filename, "w")
            self.mframe.ExportGraph(graph_file)
            graph_file.close()
            
#             if self.verbose:
            et = datetime.now()
            print "Saved map %s. Time taken: %s" % (current_map, (et-st))
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
            st = datetime.now()   
            # Save the file to the path given by the user         
            current_map = self.mframe.current_map
            filename = dlg.GetPath()
            
            try:
                shutil.copy(current_map,filename)
            except shutil.Error:
                shutil.move(filename, filename)
            
            # The graph filename must be the same as the map filename (except the extension)
            graph_filename = "%sgraph" % filename.rstrip("png")
            graph_file = open(graph_filename, "w")
            self.mframe.ExportGraph(graph_file)
            graph_file.close()            
            self.mframe.current_map = filename
            
#             if self.verbose:  
            et = datetime.now()
            print "Saved map %s. Time taken: %s" % (filename, (et-st))         
            self.SetSaveStatus(True) 
            self.mframe.SetTitle("Map Viewer    |    %s" % dlg.GetFilename())
                        
        dlg.Destroy()
        
#---------------------------------------------------------------------------------------------#    
#    Accessor function for the current save state (Saved/Unsaved)                            #
#---------------------------------------------------------------------------------------------#         
    def SetSaveStatus(self, bool_save):
        self.saved = bool_save
    
    def OnSettings(self, event):
        self.Hide()    
        self.pf.SetMinSize(APP_SIZE_EXP)
        self.pf.SetSize(APP_SIZE_EXP)    
        self.pf.sp.Show()
        self.pf.sp.Layout()
        self.pf.Layout()
        self.Layout()        
    
    def OnCloseMap(self, event):  
        if not self.saved:
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
        
        self.mframe.ClearGraph()      
        self.mframe.Clear()
        self.mframe.Hide()
        self.EnableButtons(self.btn_disabled, False)
        self.SetSaveStatus(True)

#---------------------------------------------------------------------------------------------#    
#    Exits the application. If the current map is unsaved, user is asked to save first.       #
#---------------------------------------------------------------------------------------------#    
    def OnExit(self, event):
        if not self.saved:
            dlg = wx.MessageDialog(self,
            "The current map is unsaved.\nWould you like to save it before exiting?", 
            "Warning", wx.CANCEL|wx.YES|wx.NO)
            
            result = dlg.ShowModal()
            if result == wx.ID_YES:
                self.OnSaveAs(event)
            elif result == wx.ID_CANCEL:
                dlg.Destroy()
                return
            dlg.Destroy()   
            
        self.mframe.Close()
        self.pf.Close()        
                

#---------------------------------------------------------------------------------------------#    
#    Mouse capturing functions to allow dragging of the control panel around the screen.      #
#---------------------------------------------------------------------------------------------#        
    def OnMouse(self, event):
        if event.Dragging() and self.leftDown:
            pos = self.ClientToScreen(event.GetPosition()) 
            fp = (pos.x - self.delta.x, pos.y - self.delta.y) 
            self.pf.Move(fp)  
            
    def OnLeftDown(self, event): 
        self.CaptureMouse() 
        self.leftDown = True 
        pos = self.ClientToScreen(event.GetPosition()) 
        origin = self.pf.GetPosition() 
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
        self.pf = parent 
        while self.pf.Parent is not None: 
            self.pf = self.pf.Parent
        
        self.lbl_titles = []
        self.lbl_text = []
        self.txtbxs = {}
        self.ros = self.pf.ros
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        
        title_font = self.pf.font
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
        self.txt_n.SetFont(self.pf.font)  
        self.txt_n.SetValue( str(self.Parent.mp.gg_const['n']) )
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
        self.txt_d.SetFont(self.pf.font)  
        self.txt_d.SetValue( str(self.Parent.mp.gg_const['d']) )
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
        self.txt_w.SetFont(self.pf.font)  
        self.txt_w.SetValue( str(self.Parent.mp.gg_const['w']) )   
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
        self.txt_k.SetFont(self.pf.font)  
        self.txt_k.SetValue( str(self.Parent.mp.gg_const['k']) )
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
        self.txt_e.SetFont(self.pf.font)  
        self.txt_e.SetValue( str(self.Parent.mp.gg_const['e']) )
        self.txt_e.Bind(wx.EVT_SET_FOCUS, self.OnTxtFocus)

        self.lbl_e2 = wx.StaticText(self, label="px",
                                   size=(20,20), pos=(205,225))
        self.lbl_text.append(self.lbl_e2)

        self.lbl_e3 = wx.StaticText(self, label="away for neighboring nodes",
                                   size=(220,20), pos=(20,245))
        self.lbl_text.append(self.lbl_e3)
        
        self.txtbxs['n'] = self.txt_n        
        self.txtbxs['k'] = self.txt_k
        self.txtbxs['d'] = self.txt_d       
        self.txtbxs['w'] = self.txt_w
        self.txtbxs['e'] = self.txt_e
        
#         # Edges Checkbox
#         self.chk_edge = wx.CheckBox(self, label="Allow edge generation within\n5 px of obstacles",
#                                     pos=(10,280))
#         self.chk_edge.SetValue(False)
        
        # Unknown Edges Checkbox
        self.chk_uk = wx.CheckBox(self, label="Allow edge generation\nthrough unknown map areas",
                                    pos=(10,290))
        self.chk_uk.SetValue(True)
        
        # Clear Checkbox
        self.chk_clr = wx.CheckBox(self, label="Clear nodes and edges when\ngenerating new graph",
                                    pos=(10,335))
        self.chk_clr.SetValue(True)
        
        
        # Title Label 2 
        self.lbl_st = wx.StaticText(self, label="Other Settings", 
                                    size=(240,30), pos=(10,405), style=wx.CENTER)
        
        self.lbl_st.SetFont(title_font)
        self.lbl_titles.append(self.lbl_st) 
        
        # Obstacles Checkbox
        self.chk_obs = wx.CheckBox(self, label="Show obstacles on map",
                                  pos=(10,438))
        self.chk_obs.SetValue(True)
        
        # Edge Creation Checkbox
        self.chk_ec = wx.CheckBox(self, label="Automatically connect nodes",
                                  pos=(10,470))
        self.chk_ec.SetValue(True)
        
        # Intersection Checkbox
        self.chk_int = wx.CheckBox(self, label="Automatically convert edge\nintersections into nodes",
                                  pos=(10,502))
        self.chk_int.SetValue(True)
        
        # Console Output Checkbox
        self.chk_co = wx.CheckBox(self, label="Enable console output",
                                  pos=(10,547))
        self.chk_co.SetValue(True)
                
        # Ok button
        btn_ok = wx.Button(self, label="Accept", size=(95,30), 
                           pos=(20, APP_SIZE_EXP[1]-45))  
        self.pf.mp.buttons.append(btn_ok) 
        btn_ok.Bind(wx.EVT_BUTTON, self.OnOk) 
        
        # Cancel button
        btn_cnc = wx.Button(self, label="Cancel", size=(95,30), 
                           pos=(125, APP_SIZE_EXP[1]-45))  
        self.pf.mp.buttons.append(btn_cnc) 
        btn_cnc.Bind(wx.EVT_BUTTON, self.OnCancel) 
                
        # Mouse capturing events
        self.bg.Bind(wx.EVT_MOTION, self.OnMouse)
        self.bg.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        self.bg.Bind(wx.EVT_LEFT_UP, self.OnLeftUp)        
        self.Bind(wx.EVT_MOTION, self.OnMouse)
        self.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        self.Bind(wx.EVT_LEFT_UP, self.OnLeftUp)   
        
        self.pf.mp.PaintButtons ( (255,255,255),BUTTON_COLOR ) 
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
        dc.DrawLine(20,216,220,216)
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
            for key,txt in self.txtbxs.iteritems():
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
        for key,item in self.txtbxs.iteritems():
            if item == txt:
                return str(self.Parent.mp.gg_const[key])
                
#         for i in [i for i,x in enumerate(self.txtbxs) if x == txt]:  
#             return str(self.Parent.mp.gg_const[i])   

#---------------------------------------------------------------------------------------------#    
#    Saves the settings                                                                       #
#---------------------------------------------------------------------------------------------#
    def OnOk(self, event):
        self.pf.mp.mframe.SetModes('Settings', {
                                 'spaced_edges': (not self.chk_edge.GetValue()),  
                                 'clear_graph': self.chk_clr.GetValue(),
                                 'auto_edges': self.chk_ec.GetValue(),
                                 'auto_intersections': self.chk_int.GetValue(),
                                 'unknown_edges': self.chk_uk.GetValue(),  
                                 'verbose': (self.chk_co.GetValue()),
                                 'obstacles': self.chk_obs.GetValue()                               
                               })  
#         self.pf.SuppressOutput(not self.chk_co.GetValue())
        
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
        
        wx.CallAfter(self.pf.mp.mframe.ShowObstacles, self.chk_obs.GetValue() )
         
        self.pf.mp.gg_const = {'n':n, 'k':k, 'd':d, 'w':w, 'e':e}
        self.pf.mp.mframe.gg_const = {'n':n, 'k':k, 'd':d, 'w':w, 'e':e}      
        self.Hide()
        self.pf.SetMinSize(APP_SIZE)
        self.pf.SetSize(APP_SIZE)   
        self.pf.mp.Show()
        self.pf.mp.Layout()
        self.pf.Layout()
        
    
    def OnCancel(self, event):
        self.Hide()
        for key,txt in self.txtbxs.iteritems():
            txt.SetValue(self.GetDefaultValue(txt))               
        self.pf.SetMinSize(APP_SIZE)
        self.pf.SetSize(APP_SIZE)
        self.pf.mp.Show()        
        self.pf.mp.Layout()
        self.pf.Layout()
        
        
#---------------------------------------------------------------------------------------------#    
#    Mouse capturing functions to allow dragging of the control panel around the screen.      #
#---------------------------------------------------------------------------------------------#        
    def OnMouse(self, event):
        if event.Dragging() and self.leftDown:
            pos = self.ClientToScreen(event.GetPosition()) 
            fp = (pos.x - self.delta.x, pos.y - self.delta.y) 
            self.pf.Move(fp)  
            
    def OnLeftDown(self, event): 
        self.CaptureMouse() 
        self.leftDown = True 
        pos = self.ClientToScreen(event.GetPosition()) 
        origin = self.pf.GetPosition() 
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
        self.mframe = self.Parent.mframe     
        
        # Set the background colour
        bmp = wx.EmptyBitmap(500, 500)
        dc = wx.MemoryDC()
        dc.SelectObject(bmp)
        solidbrush = wx.Brush(BG_COLOR, wx.SOLID)
        dc.SetBrush(solidbrush)
        dc.DrawRectangle(0, 0, 500, 500)
        self.bg = wx.StaticBitmap(self, -1, bmp, (-2, -2))
        
        # Set parent frame value
        self.pf = parent 
        while self.pf.Parent is not None: 
            self.pf = self.pf.Parent
        
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.hbox00 = wx.BoxSizer(wx.HORIZONTAL)
        self.hbox01 = wx.BoxSizer(wx.HORIZONTAL)
        
        # Generate Graph button
        vbox00 = wx.BoxSizer(wx.VERTICAL)   
        self.btn_gg = wx.Button(self, label="Generate Graph", size=BUTTON_SIZE)        
        self.btn_gg.Bind(wx.EVT_BUTTON, self.OnGenerateGraph)
        self.Parent.btn_disabled.append(self.btn_gg)
        self.Parent.buttons.append(self.btn_gg) 
        vbox00.Add(self.btn_gg)        
        self.sizer.Add(vbox00,1,wx.TOP,10)
        
        # Tour button
        vbox10 = wx.BoxSizer(wx.VERTICAL)   
        self.btn_tour = wx.Button(self, label="Tour All Edges", size=BUTTON_SIZE)        
        self.btn_tour.Bind(wx.EVT_BUTTON, self.OnTour)
        self.Parent.btn_disabled.append(self.btn_tour)
        self.Parent.buttons.append(self.btn_tour) 
        vbox10.Add(self.btn_tour)        
        self.sizer.Add(vbox10,1,wx.TOP,10) 
        
#         # Show/Hide map route button
        vbox13 = wx.BoxSizer(wx.VERTICAL)  
        self.btn_rte = wx.Button(self, label="Hide Route", size=BUTTON_SIZE)        
        self.btn_rte.Bind(wx.EVT_BUTTON, self.OnShowHideRoute) 
        self.btn_rte.Enable(False)
        self.Parent.buttons.append(self.btn_rte)   
        vbox13.Add(self.btn_rte)        
        self.sizer.Add(vbox13,1, wx.TOP,10)  

        vbox03 = wx.BoxSizer(wx.VERTICAL)
        lbl_font = self.pf.font
        lbl_font.SetPointSize(12)
        self.lbl = wx.StaticText(self, label="Find:", style=wx.CENTER)  
        self.lbl.SetFont(lbl_font)
        self.lbl.SetForegroundColour((85,85,80))
        vbox03.Add(self.lbl, 0, wx.LEFT, 18) 
        self.hbox01.Add(vbox03,0,wx.TOP,19)
        self.pf.font.SetPointSize(14)
        
        # Textbox
        vbox06 = wx.BoxSizer(wx.VERTICAL)     
        vbox06.AddSpacer(7)
        self.txt = wx.TextCtrl(self, size=(58,28), style=wx.TE_CENTER)
        self.txt.SetMaxLength(4)    #Maximum of 4 characters
        self.txt.SetFont(self.pf.font)
        self.txt.SetForegroundColour((255,131,79))
        self.txt.SetBackgroundColour((85,85,80))
        st = ("Usage: write \"N\" or \"E\" followed by a number.\n\n"
              "\"N14\" finds Node 14, \"E23\" finds Edge 23, etc.")
        self.txt.SetToolTip( wx.ToolTip(st) )
        vbox06.Add(self.txt,1,wx.RIGHT,3)
        self.hbox01.Add(vbox06,1,wx.TOP|wx.LEFT,7)
         
        # Go button
        vbox09 = wx.BoxSizer(wx.VERTICAL)     
        vbox09.AddSpacer(9)
        self.btn_go = wx.BitmapButton(self, -1, Resources.getOrangeArrowRightBitmap(),
                                      size=(40,33), style=wx.NO_BORDER)
        self.btn_go.SetToolTip( wx.ToolTip("Find a node or edge") )
#         self.btn_go = wx.Button(self, label="Find", size=(53,32))        
        self.btn_go.Bind(wx.EVT_BUTTON, self.OnFind)
        self.Parent.btn_disabled.append(self.btn_go)
#         self.Parent.buttons.append(self.btn_go) 
        vbox09.Add(self.btn_go)        
        self.hbox01.Add(vbox09,1,wx.TOP|wx.BOTTOM,2)        
        
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
        
    def OnShowHideRoute(self, event):
        if self.btn_rte.GetLabel() == 'Show Route':
            self.mframe.ShowRoute()
            self.btn_rte.SetLabel('Hide Route')
        else:
            self.mframe.HideRoute()
            self.btn_rte.SetLabel('Show Route')           
    
        
    def OnFind(self, event):
        txt = self.txt.GetValue()
        if txt[0] == 'n' or txt[0] == 'N':
            n_id = txt[1:]
            self.GotoNode(n_id)            
        elif txt[0] == 'e' or txt[0] == 'E':
            e_id = txt[1:]
            self.GotoEdge(e_id)            
        else:
            st = ("Usage: write \"N\" or \"E\" followed by a number.\n\n"
              "\"N14\" finds Node 14, \"E23\" finds Edge 23, etc.")
            dlg = wx.MessageDialog(self, st, "Input Error", wx.ICON_ERROR)
            dlg.ShowModal() 
            dlg.Destroy()            
        
#---------------------------------------------------------------------------------------------#    
#    Selects a node and zooms in on it.                                                       #
#    Accepts an integer X from user input. If X is positive, zooms in on Node #X. If X is     #
#    negative, zooms in on Node #(NumTotalNodes - X)                                          #
#---------------------------------------------------------------------------------------------#    
    def GotoNode(self, n_id):          
        try:
            ID = int(n_id)            
            try:
                magnification = self.mframe.image_width / 300.0
                node = self.mframe.nodelist[ID]                
                self.mframe.SelectOneNode(self.mframe.graphics_nodes[ID], True)                
                self.mframe.Zoom(node.coords, magnification)
                print "Zooming to Node %s" % ID
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
    def GotoEdge(self, e_id):         
        try:
            ID = int(e_id)            
            try:
                magnification = self.mframe.image_width / 300.0               
                edge = self.mframe.edgelist[ID] 
                self.mframe.SelectOneEdge(self.mframe.graphics_edges[ID], True)
                end1 = self.mframe.nodelist[int(edge.node1)].coords
                end2 = self.mframe.nodelist[int(edge.node2)].coords
                
                x = int( (math.fabs( end1[0]+end2[0])) /2 )   
                y = int( (math.fabs( end1[1]+end2[1])) /2 )                  
                self.mframe.Zoom((x,y), magnification)
                print "Zooming to Edge %s" % ID
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
        n=self.Parent.gg_const['n']
        k=self.Parent.gg_const['k']
        d=self.Parent.gg_const['d']
        w=self.Parent.gg_const['w']
        e=self.Parent.gg_const['e']
        
        self.mframe.GenerateGraph(n,k,d,w,e)        
            
#     def OnTour(self, event):       
#         self.pf.mp.mframe.SaveCanvasImage("canvas.png")
                   
#---------------------------------------------------------------------------------------------#    
#    Start node_traveller.                                                                    #
#---------------------------------------------------------------------------------------------# 
    def OnTour(self, event):       
        if not self.mframe.modes['pose_est']:
            dlg = wx.MessageDialog(self,
                "Create a 2D pose estimate before\n launching node traveller.",
                "Error", wx.ICON_ERROR)
            dlg.ShowModal() 
            dlg.Destroy()
            return    
        
        if not self.Parent.saved:
            dlg = wx.MessageDialog(self,
                "Map must be saved to proceed.\n Would you like to save now?",
                "Warning", wx.ICON_EXCLAMATION|wx.YES_NO)
            if dlg.ShowModal() == wx.ID_YES:
                self.Parent.OnSave(None)
            else:
                dlg.Destroy()
                return                
            dlg.Destroy()   
        
        map_file = self.mframe.current_map 
        graph_file = "%sgraph" % map_file.rstrip("png")        
        term = """gnome-terminal -e 'bash -c \
        "rosrun node_traveller travel.py _graph:=%s; exec bash\"'"""
        self.proc = subprocess.Popen(term % graph_file, shell=True)
        self.travel_pid = self.proc.pid        
    
if __name__ == '__main__':
    app = wx.App(False)
    
    wx.Log_SetActiveTarget(wx.LogStderr())
    frame = MainFrame(None, "Map Viewer")
    frame.Show()
    app.MainLoop()