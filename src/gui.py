#!/usr/bin/env python

'''
Created on May 30, 2013
'''

import os, time, shutil
import wx 
import listener as ls
from zoompanel import ZoomPanel

class MainFrame(wx.Frame):
    def __init__(self, parent, title):
        wx.Frame.__init__(self, parent, title=title, size=(240, 243),
#                           style=wx.STAY_ON_TOP
                          )   
              
        self.leftDown = False           
        self.ls = ls.listener(None, None)  
        
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
        hbox0 = wx.BoxSizer(wx.HORIZONTAL)     
        hbox0.AddSpacer(20)
        self.btn_rf = wx.Button(self, label="Update Map", size=(180,30))        
        self.btn_rf.Bind(wx.EVT_BUTTON, self.OnRefreshMap)
        self.buttons.append(self.btn_rf) 
        hbox0.Add(self.btn_rf)        
        self.sizer_menu.Add(hbox0,0,wx.TOP|wx.LEFT|wx.RIGHT,10) 

        # Show/Hide map viewer button
        hbox1 = wx.BoxSizer(wx.HORIZONTAL)            
        hbox1.AddSpacer(20)
        self.btn_map = wx.Button(self, label="Show Map", size=(180,30))        
        self.btn_map.Bind(wx.EVT_BUTTON, self.OnShowHideMap)   
        self.btn_map.Enable(False)   
        self.buttons.append(self.btn_map)   
        hbox1.Add(self.btn_map)        
        self.sizer_menu.Add(hbox1,0,
                            wx.TOP|wx.LEFT|wx.RIGHT
                            ,10) 

        # Open button
        hbox2 = wx.BoxSizer(wx.HORIZONTAL)            
        hbox2.AddSpacer(5)
        self.btn_open = wx.Button(self, label="Open Map...", size=(180,30))   
        self.buttons.append(self.btn_open)        
        self.btn_open.Bind(wx.EVT_BUTTON, self.OnOpen)  
        hbox2.Add(self.btn_open)           
        self.sizer_menu.Add(hbox2,0,
                            wx.TOP|wx.LEFT|wx.RIGHT
                            ,25)  
        # Save button
        hbox3 = wx.BoxSizer(wx.HORIZONTAL)            
        hbox3.AddSpacer(20)
        self.btn_save = wx.Button(self, label="Save Map...", size=(180,30)) 
        self.btn_save.Bind(wx.EVT_BUTTON, self.OnSaveAs)        
        self.btn_save.Enable(False)     
        self.buttons.append(self.btn_save)   
        hbox3.Add(self.btn_save)           
        self.sizer_menu.Add(hbox3,0,
                            wx.TOP|wx.LEFT|wx.RIGHT
                            ,10) 
        
        # Exit button
        hbox9 = wx.BoxSizer(wx.HORIZONTAL)             
        hbox9.AddSpacer(5)
        btn_exit = wx.Button(self, label="Exit", size=(180,30))  
        self.buttons.append(btn_exit)      
        hbox9.Add(btn_exit)           
        btn_exit.Bind(wx.EVT_BUTTON, self.OnExit)
        self.sizer_menu.Add(hbox9,0,wx.TOP|wx.LEFT|wx.RIGHT,25)   
                      
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
        
        # Loop until the file has been updated
        while(os.path.getmtime(self.ls.GetDefaultFilename()) < (time.time()-15)):
            time.sleep(1)
        
        try:
            # Use an image file (png). Must be in the same folder as this file
            map_file = self.ls.GetDefaultFilename()
            self.parent_frame.SetTitle("%s" % map_file)
            
            # Update some statuses
            self.btn_map.Enable(True)
            self.btn_save.Enable(True)
            self.SetSaveStatus(False)            
            
            # Show the image panel                  
            self.zoom_panel.image_width = self.ls.image_width
            self.zoom_panel.origin = (self.ls.pos, self.ls.orient)
            self.zoom_panel.SetImage(map_file)
            self.zoom_panel.Show()    
            
        except ZeroDivisionError:
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
        
if __name__ == '__main__':
    app = wx.App(False)
    frame = MainFrame(None, "Map Viewer")
    frame.Show()
    app.MainLoop()