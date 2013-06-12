#!/usr/bin/env python

'''
Created on May 30, 2013
'''

import os, time, shutil
import math
import wx 
import listener as ls
from zoompanel import ZoomPanel

class MainFrame(wx.Frame):
    def __init__(self, parent, title):
        wx.Frame.__init__(self, parent, title=title, size=(240, 340),
#                           style=wx.STAY_ON_TOP
                        )
        self.leftDown = False           
        self.ls = ls.listener(None, None)                             
        self.font = wx.Font(pointSize=14, family=wx.FONTFAMILY_DEFAULT, 
                       style=wx.FONTSTYLE_NORMAL, weight=wx.FONTWEIGHT_NORMAL, 
                       faceName="lucida sans")    
        
#         # Create menu bar                 
#         file_menu = wx.Menu()
#         file_menu.Append(101, '&Open...\t')
#         file_menu.Append(102, '&Save\tCtrl+S')        
#         file_menu.Append(103, '&Save As...\tCtrl+Shift+S')
#         file_menu.AppendSeparator()
#         file_menu.Append(109, 'E&xit\tCtrl+Q')
#                
#         menu_bar = wx.MenuBar()
#         menu_bar.Append(file_menu, '&File')
#         self.SetMenuBar(menu_bar)
#         
#         # Menu event binders
#         wx.EVT_MENU(self,101,self.OnOpen)
#         wx.EVT_MENU(self,102,self.OnSave)        
#         wx.EVT_MENU(self,103,self.OnSaveAs)
#         wx.EVT_MENU(self,109,self.OnExit)
         
        self.main_panel = MainPanel(self)  
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.main_panel, 1, wx.EXPAND)
        
        # Mouse capturing events      
        self.Bind(wx.EVT_MOTION, self.OnMouse)
        self.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        self.Bind(wx.EVT_LEFT_UP, self.OnLeftUp) 
        
        self.SetPosition((0,0))        
        self.Layout()    
    
        
    def SetSaveStatus(self, bool_save):
        self.main_panel.SetSaveStatus(bool_save)       
        
    def GetSaveStatus(self):
        return self.main_panel.GetSaveStatus()  
        
    def OnOpen(self, event):   
        if self.main_panel.GetSaveStatus() == False:
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
                self.main_panel.SetSaveStatus(True)
                self.OnOpen(event)
                
        #TODO: case for Save (not Save As)       
        else:
            # Open a file dialog for the user to select a file            
            filters = 'Image files (*.png)|*.png'
            dlg = wx.FileDialog(self, message="Open Map File", defaultDir=os.getcwd(), 
                                defaultFile="", wildcard=filters, style=wx.FD_OPEN)
            
            if dlg.ShowModal() == wx.ID_OK:
                self.main_panel.zoom_panel.SetNodeList([])
                self.main_panel.zoom_panel.SetEdgeList([])
                
                # Set the viewer image to the selected file
                filename = dlg.GetPath()  
                self.SetTitle("%s" % dlg.GetFilename()) 
                
                # Import the node data. For this to work, the node file must have the same
                # name as the map file, but with the extension ".graph"
                try:
                    graph_filename = "%sgraph" % filename.rstrip("png")
                    graph_file = open(graph_filename, "r")
                    self.main_panel.zoom_panel.ImportGraph(graph_file)
                    graph_file.close()
                except IOError:
                    self.main_panel.zoom_panel.SetNodeList([])
                    self.main_panel.zoom_panel.SetEdgeList([])
                
                self.main_panel.zoom_panel.SetImage(filename)  
                self.main_panel.zoom_panel.Show()             
                
                self.main_panel.btn_map.Enable(True)
                self.main_panel.btn_save.Enable(True)
                self.main_panel.btn_exp.Enable(True)       
                self.main_panel.btn_map.SetLabel("Hide Map")
                            
            dlg.Destroy()
            
        
    def OnSave(self, event):
        # Save the file         
        current_map = self.main_panel.zoom_panel.GetCurrentMapPath() 
        if current_map is [] or os.path.basename(current_map)==self.ls.GetDefaultFilename():
            self.OnSaveAs(event)  
        else:
            shutil.move(current_map,current_map) 
            self.main_panel.SetSaveStatus(True) 
    
    def OnSaveAs(self, event):
        filters = 'Image files (*.png)|*.png'
        dlg = wx.FileDialog(self, message="Save Map File", defaultDir=os.getcwd(), 
                            defaultFile="", wildcard=filters, style=wx.FD_SAVE|
                            wx.FD_OVERWRITE_PROMPT)
        
        if dlg.ShowModal() == wx.ID_OK:   
            # Save the file to the path given by the user         
            current_map = self.main_panel.zoom_panel.GetCurrentMapPath()
            filename = dlg.GetPath()
            shutil.copy(current_map,filename)            
            
            # Export the node data into its own file. The node filename is the same as the map
            # filename, but with the extension ".graph"
            graph_filename = "%sgraph" % filename.rstrip("png")
            graph_file = open(graph_filename, "w")
            self.main_panel.zoom_panel.ExportGraph(graph_file)
            graph_file.close()
                        
            self.main_panel.SetSaveStatus(True) 
            self.SetTitle("%s" % dlg.GetFilename())
                        
        dlg.Destroy()
    
    def OnExit(self, event):
        self.main_panel.zoom_panel.Close()
        self.Close()
        #TODO: check unsaved status
        
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
        
        # Set the app background colour
        bmp = wx.EmptyBitmap(1920, 1080)
        dc = wx.MemoryDC()
        dc.SelectObject(bmp)
        solidbrush = wx.Brush(wx.Colour(155,155,155), wx.SOLID)
        dc.SetBrush(solidbrush)
        dc.DrawRectangle(0, 0, 1920, 1080)
        self.bg = wx.StaticBitmap(self, -1, bmp, (0, 0)) 
        
        self.leftDown = False
        self.saved = True
        self.buttons = []
        
        # Set parent frame value
        self.parent_frame = parent 
        while self.parent_frame.GetParent() is not None: 
            self.parent_frame = self.parent_frame.GetParent()
        
        self.ls = self.parent_frame.ls
        
        # Create the sizers 
        self.sizer_main = wx.BoxSizer(wx.HORIZONTAL)         
        self.sizer_menu = wx.BoxSizer(wx.VERTICAL)      
        self.sizer_display = wx.BoxSizer(wx.VERTICAL)
        
        # The image panel     
        self.zoom_panel=ZoomPanel(self, title="Map View",
                                  size=((1024,1024)), 
                                  style=wx.FRAME_SHAPED
                                  )  
        self.zoom_panel.Hide()
#         self.zoom_panel.SetPosition((320,0))       
        self.zoom_panel.SetPosition((550,0))
               
                        
        # Refresh map button
        hbox00 = wx.BoxSizer(wx.HORIZONTAL)     
        hbox00.AddSpacer(20)
        self.btn_rf = wx.Button(self, label="Update Map", size=(180,30))        
        self.btn_rf.Bind(wx.EVT_BUTTON, self.OnRefreshMap)
        self.buttons.append(self.btn_rf) 
        hbox00.Add(self.btn_rf)        
        self.sizer_menu.Add(hbox00,0,wx.TOP|wx.LEFT|wx.RIGHT,10) 

        # Show/Hide map viewer button
        hbox03 = wx.BoxSizer(wx.HORIZONTAL)            
        hbox03.AddSpacer(20)
        self.btn_map = wx.Button(self, label="Show Map", size=(180,30))        
        self.btn_map.Bind(wx.EVT_BUTTON, self.OnShowHideMap)   
        self.btn_map.Enable(False)   
        self.buttons.append(self.btn_map)   
        hbox03.Add(self.btn_map)        
        self.sizer_menu.Add(hbox03,0,
                            wx.TOP|wx.LEFT|wx.RIGHT
                            ,10) 
        
        # Explore button
        hbox06 = wx.BoxSizer(wx.HORIZONTAL)     
        hbox06.AddSpacer(20)
        self.btn_exp = wx.Button(self, label="Explore", size=(180,30))        
        self.btn_exp.Bind(wx.EVT_BUTTON, self.OnExplore)
        self.btn_exp.Enable(False) 
        self.buttons.append(self.btn_exp) 
        hbox06.Add(self.btn_exp)        
        self.sizer_menu.Add(hbox06,0,wx.TOP|wx.LEFT|wx.RIGHT,10)
        
        # Explore panel
        hbox09 = wx.BoxSizer(wx.HORIZONTAL)     
        hbox09.AddSpacer(20)       
        self.exp_panel = ExplorePanel(self)
        self.exp_panel.Hide()
        hbox09.Add(self.exp_panel)   
        self.sizer_menu.Add(hbox09,0,wx.TOP|wx.LEFT|wx.RIGHT,10) 

        # Open button
        hbox10 = wx.BoxSizer(wx.HORIZONTAL)            
        hbox10.AddSpacer(5)
        self.btn_open = wx.Button(self, label="Open Map...", size=(180,30))   
        self.buttons.append(self.btn_open)        
        self.btn_open.Bind(wx.EVT_BUTTON, self.OnOpen)  
        hbox10.Add(self.btn_open)           
        self.sizer_menu.Add(hbox10,0,
                            wx.TOP|wx.LEFT|wx.RIGHT
                            ,25)  
        # Save button
        hbox13 = wx.BoxSizer(wx.HORIZONTAL)            
        hbox13.AddSpacer(20)
        self.btn_save = wx.Button(self, label="Save Map...", size=(180,30)) 
        self.btn_save.Bind(wx.EVT_BUTTON, self.OnSaveAs)        
        self.btn_save.Enable(False)     
        self.buttons.append(self.btn_save)   
        hbox13.Add(self.btn_save)           
        self.sizer_menu.Add(hbox13,0,
                            wx.TOP|wx.LEFT|wx.RIGHT
                            ,10) 
        
        # Exit button
        hbox20 = wx.BoxSizer(wx.HORIZONTAL)             
        hbox20.AddSpacer(5)
        btn_exit = wx.Button(self, label="Exit", size=(180,30))  
        self.buttons.append(btn_exit)      
        hbox20.Add(btn_exit)           
        btn_exit.Bind(wx.EVT_BUTTON, self.OnExit)
        self.sizer_menu.Add(hbox20,0,wx.TOP|wx.LEFT|wx.RIGHT,25)   
                      
        self.PaintButtons( (255,255,255),(65,65,60) )  
                
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
        
    def PaintButtons(self, foreground, background):
        try:
            for btn in self.buttons:
                btn.SetForegroundColour(foreground)
                btn.SetBackgroundColour(background)
        except IOError:
            print "Invalid argument: expected (R,G,B) value"
            
            
    def SetSaveStatus(self, bool_save):
        self.saved = bool_save
        
        
    def GetSaveStatus(self):
        return self.saved
                
                
    def OnRefreshMap(self, event):  
        #TODO: unsaved case  
        
        wx.BeginBusyCursor()    
        # Start listening for a map
        self.ls.listen()        
        print "Creating map..."        
        
        # Loop until the file has been correctly updated
        done = False
        while not done:            
            try:
                while(os.path.getmtime(self.ls.GetDefaultFilename()) < (time.time()-15)):
                    time.sleep(1)
                done = True
                
            except OSError:
                time.sleep(1)
                
        try:
            # Use an image file (png). Must be in the same folder as this file
            map_file = self.ls.GetDefaultFilename()
            self.parent_frame.SetTitle("%s" % map_file)
            
            # Update some statuses
            self.btn_map.Enable(True)
            self.btn_save.Enable(True)
            self.btn_exp.Enable(True)
            self.SetSaveStatus(False)            
            
            # Show the image panel                  
            self.zoom_panel.image_width = self.ls.image_width
            self.zoom_panel.origin = (self.ls.pos, self.ls.orient)
            self.zoom_panel.SetImage(map_file)
            self.zoom_panel.Show()    
            
        except IndexError:
            # Image not found in directory
            pass      
        
        self.btn_map.SetLabel("Hide Map") 
        
        self.Layout()                
        wx.EndBusyCursor()
       
        
    def OnShowHideMap(self, event):
        if self.btn_map.GetLabel()[0]=='S':
            self.zoom_panel.Show()
            self.btn_map.SetLabel("Hide Map")
        else:
            self.zoom_panel.Hide()
            self.btn_map.SetLabel("Show Map")
            
    def OnExplore(self, event):
        if self.exp_panel.IsShown():
            self.exp_panel.Hide()
            self.Layout()
        else:
            self.exp_panel.Show()
            self.exp_panel.txt.Clear()
            self.Layout()
            
    def OnOpen(self, event):
        self.parent_frame.OnOpen(event)
    
    def OnSave(self, event):
        self.parent_frame.OnSave(event)
        
    def OnSaveAs(self, event):
        self.parent_frame.OnSaveAs(event)
        
    def OnExit(self, event):
        self.zoom_panel.Close()
        self.GetParent().Close(force=True)
        
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
        
        self.zoom_panel = self.GetParent().zoom_panel
        
        # Set the background colour
        bmp = wx.EmptyBitmap(500, 500)
        dc = wx.MemoryDC()
        dc.SelectObject(bmp)
        solidbrush = wx.Brush(wx.Colour(155,155,155), wx.SOLID)
        dc.SetBrush(solidbrush)
        dc.DrawRectangle(0, 0, 500, 500)
        self.bg = wx.StaticBitmap(self, -1, bmp, (-2, -2))
        
        # Set parent frame value
        self.parent_frame = parent 
        while self.parent_frame.GetParent() is not None: 
            self.parent_frame = self.parent_frame.GetParent()
        
        self.sizer = wx.BoxSizer(wx.HORIZONTAL)
        
        # Node/Edge radio buttons
        vbox03 = wx.BoxSizer(wx.VERTICAL)
        self.radio_node = wx.RadioButton(self, -1, "Node", style=wx.RB_GROUP) 
        self.radio_edge = wx.RadioButton(self, -1, "Edge")
        vbox03.Add(self.radio_node) 
        vbox03.Add(self.radio_edge) 
        self.sizer.Add(vbox03,0,wx.TOP,5)    
        self.radio_node.Bind(wx.EVT_RADIOBUTTON, self.OnSelectNode)
        self.radio_edge.Bind(wx.EVT_RADIOBUTTON, self.OnSelectEdge)
        
        # Textbox
        vbox06 = wx.BoxSizer(wx.VERTICAL)     
        vbox06.AddSpacer(5)
        self.txt = wx.TextCtrl(self, size=(45,30), style=wx.NO_BORDER|wx.TE_RIGHT)
        self.txt.SetMaxLength(3)    #Maximum of 3 characters
        self.txt.SetFont(self.parent_frame.font)
        self.txt.SetForegroundColour((255,255,255))
        self.txt.SetBackgroundColour((100,100,100))
        vbox06.Add(self.txt)
        self.sizer.Add(vbox06,1,wx.TOP|wx.LEFT,10)
         
        # Go button
        vbox09 = wx.BoxSizer(wx.VERTICAL)     
        vbox09.AddSpacer(5)
        self.btn_go = wx.Button(self, label="Go", size=(50,30))        
        self.btn_go.Bind(wx.EVT_BUTTON, self.OnGotoNode)
        self.GetParent().buttons.append(self.btn_go) 
        vbox09.Add(self.btn_go)        
        self.sizer.Add(vbox09,1,wx.TOP|wx.LEFT,10)
        
        self.SetSizer(self.sizer)
        
    def OnSelectNode(self, event):
        self.btn_go.Bind(wx.EVT_BUTTON, None)
        self.btn_go.Bind(wx.EVT_BUTTON, self.OnGotoNode)
        
    def OnSelectEdge(self, event):
        self.btn_go.Bind(wx.EVT_BUTTON, None)
        self.btn_go.Bind(wx.EVT_BUTTON, self.OnGotoEdge)
        
    # Selects a node and zooms in on it   
    def OnGotoNode(self, event):
        txt = self.txt.GetValue()           
        try:
            ID = int(txt)            
            try:
                node = self.zoom_panel.nodelist[ID]                
                self.zoom_panel.SelectOneNode(self.zoom_panel.graphics_nodes[ID])
                
                self.zoom_panel.Canvas.ZoomToBB()
                self.zoom_panel.Zoom(node.coords, 10.0)
            except ValueError:
                dlg = wx.MessageDialog(self,
                "Node %s does not exist." % str(ID), "Error", wx.ICON_ERROR)
                dlg.ShowModal() 
                dlg.Destroy()
            
        except IndexError:
            dlg = wx.MessageDialog(self,
                "Please enter a positive integer value", "Error", wx.ICON_ERROR)
            dlg.ShowModal() 
            dlg.Destroy()
            
    # Selects an edge and zooms in on it    
    def OnGotoEdge(self, event):
        txt = self.txt.GetValue()            
        try:
            ID = int(txt)            
            try:
                edge = self.zoom_panel.edgelist[ID] 
                self.zoom_panel.SelectOneEdge(self.zoom_panel.graphics_edges[ID])
                end1 = self.zoom_panel.nodelist[int(edge.node1)].coords
                end2 = self.zoom_panel.nodelist[int(edge.node2)].coords
                
                x = int( (math.fabs( end1[0]+end2[0])) /2 )   
                y = int( (math.fabs( end1[1]+end2[1])) /2 ) 
                
                self.zoom_panel.Canvas.ZoomToBB()  
                self.zoom_panel.Zoom((x,y), 10.0)
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
    
if __name__ == '__main__':
    app = wx.App(False)
    wx.Log_SetActiveTarget(wx.LogStderr())
    frame = MainFrame(None, "Map Viewer")
    frame.Show()
    app.MainLoop()