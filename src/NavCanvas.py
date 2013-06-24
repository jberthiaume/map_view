"""
A Panel that includes the FloatCanvas and Navigation controls

"""

import wx
import FloatCanvas, GUIMode
from wx.lib.floatcanvas import Resources

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

        self.Modes = [("Add Node", GUIMode.GUIMouse(),   Resources.getPointerBitmap()),
#                       ("Debug 001", GUIMode.GUIMouse2(),  Resources.getMoveRLCursorBitmap()),
                      ("Zoom In",  GUIMode.GUIZoomIn(),  Resources.getMagPlusBitmap()),
                      ("Zoom Out", GUIMode.GUIZoomOut(), Resources.getMagMinusBitmap()),
                      ("Pan",      GUIMode.GUIMove(),    Resources.getHandBitmap()),
                      ]
        
        self.BuildToolbar()
        ## Create the vertical sizer for the toolbar and Panel
        box = wx.BoxSizer(wx.VERTICAL)
        box.Add(self.ToolBar, 0, wx.ALL | wx.ALIGN_LEFT | wx.GROW, 4)

        self.Canvas = FloatCanvas.FloatCanvas(self, **kwargs)
        box.Add(self.Canvas, 1, wx.GROW)

        self.SetSizerAndFit(box)

        # default to first mode
        #self.ToolBar.ToggleTool(self.PointerTool.GetId(), True)
        self.Canvas.SetMode(self.Modes[0][1])

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
        self.AddToolbarModeButtons(tb, self.Modes)
        self.AddToolbarZoomButton(tb)
        tb.Realize()
        ## fixme: remove this when the bug is fixed!
        #wx.CallAfter(self.HideShowHack) # this required on wxPython 2.8.3 on OS-X
    
    def AddToolbarModeButtons(self, tb, Modes):
        self.ModesDict = {}
        for Mode in Modes:
            tool = tb.AddRadioTool(wx.ID_ANY, shortHelp=Mode[0], bitmap=Mode[2])
            self.Bind(wx.EVT_TOOL, self.SetMode, tool)
            self.ModesDict[tool.GetId()]=Mode[1]
        #self.ZoomOutTool = tb.AddRadioTool(wx.ID_ANY, bitmap=Resources.getMagMinusBitmap(), shortHelp = "Zoom Out")
        #self.Bind(wx.EVT_TOOL, lambda evt : self.SetMode(Mode=self.GUIZoomOut), self.ZoomOutTool)

    def AddToolbarZoomButton(self, tb):
        tb.AddSeparator()

        self.ZoomButton = wx.Button(tb, label="Zoom To Fit", size=(110,30))
        tb.AddControl(self.ZoomButton)
        self.ZoomButton.Bind(wx.EVT_BUTTON, self.ZoomToFit)
        
        tb.AddSeparator()
        
        self.CanvasButton = wx.Button(tb, label="View Canvas", size=(110,30))
        tb.AddControl(self.CanvasButton)       
        self.CanvasButton.Bind(wx.EVT_BUTTON, self.ZoomToCanvas)
        try:            
            self.GetParent().GetParent().buttons.append(self.ZoomButton)
            self.GetParent().GetParent().buttons.append(self.CanvasButton)
        except AttributeError:
            pass

    def HideShowHack(self):
        ##fixme: remove this when the bug is fixed!
        """
        Hack to hide and show button on toolbar to get around OS-X bug on
        wxPython2.8 on OS-X
        """
        self.ZoomButton.Hide()
        self.ZoomButton.Show()

    def SetMode(self, event):
        Mode = self.ModesDict[event.GetId()]
        self.Canvas.SetMode(Mode)

    def ZoomToFit(self,event):
        self.Canvas.GetMode()
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
