"""
A Panel that includes the FloatCanvas and Navigation controls

"""

import wx
import FloatCanvas, GUIMode
import Resources

class NavCanvas(wx.Panel):
    """
    NavCanvas.py

    This is a high level window that encloses the FloatCanvas in a panel
    and adds a Navigation toolbar.

    """

    def __init__(self,
                   parent,
                   id = wx.ID_ANY,
                   size = wx.DefaultSize,
                   **kwargs): # The rest just get passed into FloatCanvas
        wx.Panel.__init__(self, parent, id, size=size)
        self.parent = parent
        self.Canvas = FloatCanvas.FloatCanvas(self, **kwargs)
        
        self.Utils = [("Open",      self.OnOpen,         Resources.getFolderIconBitmap()),
                      ("Save",      self.OnSave,         Resources.getSaveIconBitmap()),
                      ]

        self.Modes = [("Zoom In",     GUIMode.GUIZoomIn(),  Resources.getZoomInIconBitmap()),
                      ("Zoom Out",    GUIMode.GUIZoomOut(), Resources.getZoomOutIconBitmap()),
                      ("Pan",         GUIMode.GUIPan(),     Resources.getAeroMoveIconBitmap()),
                      ("Add/Select",  GUIMode.GUIMouse(),   Resources.getAeroArrowBitmap()),
                      ("Lasso Tool",  GUIMode.GUISelect(),  Resources.getSelectButtonBitmap()),
                      ]
        
        self.tools = []
        self.BuildToolbar()
        box = wx.BoxSizer(wx.VERTICAL)
        box.Add(self.ToolBar, 0, wx.ALL | wx.ALIGN_LEFT | wx.GROW, 4)

        box.Add(self.Canvas, 1, wx.GROW)

        self.SetSizerAndFit(box)

        # default to first mode
        self.ToolBar.ToggleTool(self.tools[3].GetId(), True)
        return None

    def BuildToolbar(self):
        """
        This is here so it can be over-ridden in a subclass, to add extra tools, etc
        """
        tb = wx.ToolBar(self, style=wx.TB_HORIZONTAL | wx.NO_BORDER | 
        wx.TB_FLAT | wx.TB_TEXT)
#         tb.SetBackgroundColour((155,155,145))
        self.ToolBar = tb
        tb.SetToolBitmapSize((24,24))
        self.AddToolbarUtilButtons(tb, self.Utils)
        self.AddToolbarModeButtons(tb, self.Modes)
        self.AddToolbarZoomButton(tb)
        tb.Realize()
    
    def AddToolbarUtilButtons(self, tb, utils):
        for util in utils:
            button = wx.BitmapButton(tb, -1, util[2], size=(45,45), style=wx.NO_BORDER)
            button.SetToolTip( wx.ToolTip(util[0]) )
            tb.AddControl(button)
            button.Bind(wx.EVT_BUTTON, util[1])
            button.Bind(wx.EVT_SET_FOCUS, self.OnReceiveFocus)
    
    def AddToolbarModeButtons(self, tb, Modes):
        tb.AddSeparator()
        
        self.ModesDict = {}
        for Mode in Modes:
            tool = tb.AddRadioTool(wx.ID_ANY, shortHelp=Mode[0], bitmap=Mode[2])
            self.Bind(wx.EVT_TOOL, self.SetMode, tool)            
            self.ModesDict[tool.GetId()]=Mode[1]
            
            self.tools.append(tool)
        #self.ZoomOutTool = tb.AddRadioTool(wx.ID_ANY, bitmap=Resources.getMagMinusBitmap(), shortHelp = "Zoom Out")
        #self.Bind(wx.EVT_TOOL, lambda evt : self.SetMode(Mode=self.GUIZoomOut), self.ZoomOutTool)        

    def AddToolbarZoomButton(self, tb):
        tb.AddSeparator()

        self.ZoomButton = wx.BitmapButton(tb, -1, Resources.getZoomToFitIconBitmap(),
                                         size=(45,45), style=wx.NO_BORDER)
                                          
        self.ZoomButton.SetToolTip( wx.ToolTip("Zoom to Fit") )
        tb.AddControl(self.ZoomButton)
        self.ZoomButton.Bind(wx.EVT_BUTTON, self.ZoomToFit)
                 
#         self.CanvasButton = wx.Button(tb, label="View Canvas", size=(110,30))
#         tb.AddControl(self.CanvasButton)       
#         self.CanvasButton.Bind(wx.EVT_BUTTON, self.ZoomToCanvas)
#         try:            
#             self.GetParent().GetParent().buttons.append(self.ZoomButton)
#             self.GetParent().GetParent().buttons.append(self.CanvasButton)
#         except AttributeError:
#             pass
        
#---------------------------------------------------------------------------------------------#    
#    Event handlers passed down to the main panel                                             #
#---------------------------------------------------------------------------------------------#        
    def OnOpen(self, event):
        self.parent.OnOpen(event)
         
    def OnSave(self, event):
        self.parent.OnSave(event) 
                
    def OnSaveAs(self, event):
        self.parent.OnSaveAs(event)         
        
    def OnSettings(self, event):
        self.parent.OnSettings(event)  
        
    def OnCloseMap(self, event):
        self.parent.OnClose(event)       
        
    def OnExit(self, event):
        self.parent.OnExit(event)
        
    def OnReceiveFocus(self, event):
        self.Canvas.SetFocus()           

    def HideShowHack(self):
        ##fixme: remove this when the bug is fixed!
        """
        Hack to hide and show button on toolbar to get around OS-X bug on
        wxPython2.8 on OS-X
        """
        self.ZoomButton.Hide()
        self.ZoomButton.Show()

    def SetMode(self, event):
        ID = event.GetId()
        Mode = self.ModesDict[ID]
        self.Canvas.SetMode(Mode)

    def ZoomToFit(self,event):                
        try:
#             iw = self.GetParent().image_width
#             self.GetParent().Zoom((iw/2,iw/2), (iw/1000.0))
            self.GetParent().ZoomToFit()
        except IndexError:
            pass
        self.Canvas.SetFocus() # Otherwise the focus stays on the Button, and wheel events are lost.

    def ZoomToCanvas(self,event):
        self.Canvas.GetMode()
        self.Canvas.ZoomToBB()
        self.Canvas.SetFocus() # Otherwise the focus stays on the Button, and wheel events are lost.
