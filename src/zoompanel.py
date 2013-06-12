#!/usr/bin/env python 

'''
Created on May 30, 2013

@author: jon
'''

import wx 
import pickle
import math
import NavCanvas, FloatCanvas
import node as N
import edge as E
import numpy as NP

''' Global colors '''
NODE_FILL           = (0,200,25)
NODE_BORDER         = (25,25,180)
EDGE_COLOR          = (25,25,180)
HIGHLIGHT_COLOR     = (225,225,0)
TEXT_COLOR          = (25,25,180)

''' Global dimensions for graphical objects '''
NODE_DIAM           = 9
NODE_BORDER_WIDTH   = 2
EDGE_WIDTH          = 7
FONT_SIZE           = 7


class ZoomPanel(wx.Frame): 

    def __init__(self, *args, **kwargs): 
        wx.Frame.__init__(self, *args, **kwargs)
        self.CreateStatusBar() 
        
        # Initialize data structures   
        self.export = False
        self.origin = None
        self.image_width = 2000
        self.last_sel_node = "" 
        self.current_zoom = 1.0 
        self.current_map = []
        
        self.nodelist = []
        self.edgelist = []
        self.graphics_nodes = []
        self.graphics_edges = []
        self.graphics_text = []
        
        self.sel_nodes = []
        self.sel_edges = []   
        
        
        # Connection matrix data structure
        # See 'GenerateConnectionMatrix()' for more info
        self.conn_matrix = NP.empty(shape=(40,40))
        self.conn_matrix[:] = -1
        NP.set_printoptions(edgeitems=10, linewidth=150)
        
        
        # Add the Canvas 
        Canvas = NavCanvas.NavCanvas(self, 
                                     ProjectionFun = None, 
                                     BackgroundColor = "DARK GREY", 
                                     ).Canvas 
        Canvas.MaxScale=4 
        self.Canvas = Canvas
        
        # Bind canvas mouse events
        FloatCanvas.EVT_MOTION(self.Canvas, self.OnMove )        
        self.Canvas.Bind(FloatCanvas.EVT_LEFT_DOWN, self.OnLeftDown)
        self.Canvas.Bind(FloatCanvas.EVT_RIGHT_DOWN, self.OnRightDown)
        
        id_sel_all = wx.NewId()
        id_desel_all = wx.NewId()
        id_sel_nodes = wx.NewId()
        id_sel_edges = wx.NewId()
        id_create_edges = wx.NewId()
        id_del = wx.NewId()
        
        wx.EVT_MENU(self, id_sel_all, self.SelectAll) 
        wx.EVT_MENU(self, id_desel_all, self.DeselectAll) 
        wx.EVT_MENU(self, id_sel_nodes, self.SelectNodes)
        wx.EVT_MENU(self, id_sel_edges, self.SelectEdges)
        wx.EVT_MENU(self, id_create_edges, self.CreateEdges)
        wx.EVT_MENU(self, id_del, self.DeleteSelection)
        
        # Accelerator table for hotkeys
        self.accel_tbl = wx.AcceleratorTable([(wx.ACCEL_CTRL, ord('A'), id_sel_all),
                                              (wx.ACCEL_CTRL, ord('D'), id_desel_all),
                                              (wx.ACCEL_CTRL|wx.ACCEL_SHIFT, ord('N'), id_sel_nodes),
                                              (wx.ACCEL_CTRL|wx.ACCEL_SHIFT, ord('E'), id_sel_edges),
                                              (wx.ACCEL_CTRL, ord('E'), id_create_edges),
                                              (wx.ACCEL_NORMAL, wx.WXK_DELETE, id_del)
                                             ])
        self.SetAcceleratorTable(self.accel_tbl)  
        self.Layout()
                

#---------------------------------------------------------------------------------------------#    
#    Writes the current cursor coordinates to the status bar at the bottom                    #
#---------------------------------------------------------------------------------------------#
    def OnMove(self, event): 
        self.SetStatusText("%i, %i"%tuple(event.Coords))
        
    
#---------------------------------------------------------------------------------------------#    
#    Mouse click handler: left button                                                         #
#---------------------------------------------------------------------------------------------# 
    def OnLeftDown(self, event):
        current_mode = self.Canvas.GetMode()
        
        if current_mode=='GUIMouse':
            self.CreateNode(event.Coords)
            
        elif current_mode=='GUIMouse2':
            # Unimplemented for now
            pass
    
    
#---------------------------------------------------------------------------------------------#    
#    Mouse click handler: right button                                                        #
#---------------------------------------------------------------------------------------------#
    def OnRightDown(self, event):
        # Create the menu which appears on right click
        rc_menu = wx.Menu()
        
        rc_menu.Append(11, "(%s, %s)" % (int(event.Coords[0]), int(event.Coords[1])))
        rc_menu.Enable(11, False)
        
        rc_menu.AppendSeparator()
        
        rc_menu.Append(21, '&Create edges\tCtrl+E')        
        wx.EVT_MENU(self,21,self.CreateEdges)
        if len(self.sel_nodes) < 2:
            rc_menu.Enable(21, False)
        
        rc_menu.Append(22, '&Delete Selection\tDel')        
        wx.EVT_MENU(self,22,self.DeleteSelection)
        if len(self.sel_nodes)<1 and len(self.sel_edges)<1:
            rc_menu.Enable(22, False)
        
        rc_menu.AppendSeparator()
        
        # Select submenu
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
    

#--------------------------------------------------------------------------------------------#    
#     Creates a single node at the given coordinates                                         #
#--------------------------------------------------------------------------------------------#    
    def CreateNode(self, coords):  
        
        # coords are in float, but we need int values for pixels
        node_coords = [int(coords[0]), int(coords[1])]   
        node = N.Node(len(self.nodelist), node_coords)
        
        ID = str(len(self.nodelist))
        xy = node_coords[0], node_coords[1]
        diam = NODE_DIAM
        lw = NODE_BORDER_WIDTH
        lc = NODE_BORDER    
        fc = NODE_FILL  
        fs = FONT_SIZE   
        
        collision = self.DetectCollision(node)
        if collision < 0:  
                   
            # Draw the node on the canvas
            c = self.Canvas.AddCircle(xy, diam, LineWidth=lw, LineColor=lc, FillColor=fc)
            c.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickNode)    # Make the node 'clickable'
            self.graphics_nodes.append(c)
            c.Name = ID
            c.Coords = node_coords    
            
            t = self.Canvas.AddText(ID, xy, Size=fs, Position="cc", Color=TEXT_COLOR)
            self.graphics_text.append(t)        
            
            # Assign the Circle to its node
            node.SetGraphicIndex( int(c.Name) )        
            self.nodelist.append(node) 
            try:
                # Tell the connection matrix that this node now exists
                self.conn_matrix[int(ID)][int(ID)] = 0  
                print "Created Node #%s at (%s, %s)" % (ID, node_coords[0], node_coords[1])
                print "\tMetric -> (%s, %s)" % (node.m_coords[0], node.m_coords[1])
                
            except IndexError:
                # Out of space in the connection matrix - expand it by 20 nodes
                curr_len = len(self.conn_matrix[0])
                self.conn_matrix.resize((curr_len+20, curr_len+20))
                
                # Tell the connection matrix that this node now exists
                self.conn_matrix[int(ID)][int(ID)] = 0 
                print "Created Node #%s at (%s, %s)" % (ID, node_coords[0], node_coords[1])
            
            self.Canvas.Draw(True)
            self.GetParent().SetSaveStatus(False)  
                  
        else:
            print "Could not create node at (%s, %s). (Collision with node %s)" \
                    % (node_coords[0], node_coords[1], str(collision))
        

#--------------------------------------------------------------------------------------------#    
#    Creates edges between all selected nodes. Edges are be created in the order that the    #
#    nodes were selected.                                                                    #
#--------------------------------------------------------------------------------------------#         
    def CreateEdges(self, event):
        if len(self.sel_nodes) >= 2:
                        
            max_x = 0
            max_y = 0
            min_x = 4000
            min_y = 4000
            
            # Add the coordinates of the nodes to be connected to a list of points
            points = []
            for i in range(len(self.sel_nodes)):
                
                # Taking note of the maximum and minimum coordinates for later
                if self.sel_nodes[i].Coords[0] > max_x:
                    max_x = self.sel_nodes[i].Coords[0] + (NODE_DIAM/2)
                if self.sel_nodes[i].Coords[0] < min_x:
                    min_x = self.sel_nodes[i].Coords[0] - (NODE_DIAM/2)
                if self.sel_nodes[i].Coords[1] > max_y:
                    max_y = self.sel_nodes[i].Coords[1] + (NODE_DIAM/2)
                if self.sel_nodes[i].Coords[1] < min_y:
                    min_y = self.sel_nodes[i].Coords[1] - (NODE_DIAM/2)
                                    
                points.append(self.sel_nodes[i].Coords)     
                
            # Create the edges 
            lw = EDGE_WIDTH          
            cl = EDGE_COLOR              
            for j in range(len(points)-1): 
                
                try:
                    self.GenerateConnectionMatrix()
                    node1 = self.sel_nodes[j]
                    node2 = self.sel_nodes[j+1]
                    
                    # Only create the edge if no edge exists between the selected points
                    if int(self.conn_matrix[int(node1.Name)][int(node2.Name)]) < 0:
                        edge = E.Edge(len(self.edgelist), node1.Name, node2.Name, 
                                      self.Distance(node1.Coords, node2.Coords))
                        self.edgelist.append(edge)
                               
                        e = self.Canvas.AddLine([points[j], points[j+1]], 
                                                LineWidth = lw, LineColor = cl)
                        e.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickEdge)
                        self.graphics_edges.append(e)
                        
                        e.Name = str(edge.id)
                        
                        edge.SetGraphicIndex( int(e.Name) )
                        print "Created edge #%s between nodes %s and %s" % (str(edge.id), 
                                                                            str(edge.node1),
                                                                            str(edge.node2))
                        print "\t(%s, %s) -> (%s, %s)" % (str(points[j][0]),
                                                            str(points[j][1]),
                                                            str(points[j+1][0]),
                                                            str(points[j+1][1]))
                    else:
                        print "Did not create edge between nodes %s and %s\n\t(already exists)" \
                        % (node1.Name,node2.Name)
                except IndexError:
                    pass
                
            
            # Refresh nodes which could have been covered by the lines
            for node in self.nodelist:
                if (node.coords[0] <= max_x and node.coords[0] >= min_x and
                    node.coords[1] <= max_y and node.coords[1] >= min_y):
                    
                    ID = str(node.id)
                    xy = node.coords[0], node.coords[1]
                    diam = NODE_DIAM
                    lw = NODE_BORDER_WIDTH
                    lc = NODE_BORDER    
                    fc = NODE_FILL 
                    fs = FONT_SIZE          
                                        
                    # Draw the node on the canvas
                    c = self.Canvas.AddCircle(xy, diam, LineWidth=lw, LineColor=lc, FillColor=fc)
                    c.Bind(FloatCanvas.EVT_FC_LEFT_DOWN, self.OnClickNode)
                    self.graphics_nodes[int(ID)] = c                     
                    c.Name = ID
                    c.Coords = node.coords            
                    
                    # Make the old text invisible and replace it in the data structure
                    t = self.Canvas.AddText(ID, xy, Size=fs, Position="cc", Color=TEXT_COLOR)
                    self.graphics_text[int(ID)].Visible = False
                    self.graphics_text[int(ID)] = t
            
            for obj in self.sel_nodes:
                obj.Visible = False
            
            self.GenerateConnectionMatrix()   
            self.Canvas.Draw(True)
            self.DeselectAll(event)
            self.GetParent().SetSaveStatus(False)


#--------------------------------------------------------------------------------------------#    
#      Deletes all selected nodes and edges                                                  #
#--------------------------------------------------------------------------------------------#           
    def DeleteSelection(self, event):        
        for node in self.sel_nodes:
            self.RemoveNode(node)
        for edge in self.sel_edges:
            self.RemoveEdge(edge)
        
        self.RenumberNodes()  
        self.RenumberEdges()
        self.GenerateConnectionMatrix()        
        
        self.DeselectAll(event=None)
        self.GetParent().SetSaveStatus(False)

#--------------------------------------------------------------------------------------------#    
#     Deletes a node.                                                                        #
#     This function should not be called directly -> use DeleteSelection() instead           #
#--------------------------------------------------------------------------------------------#        
    def RemoveNode(self, node):
        ID = int(node.Name)
        node_obj = self.nodelist[ID] # Pointer to the 'node.Node' data structure

        self.graphics_nodes[ ID ].Visible = False
        self.graphics_text [ ID ].Visible = False
        
        for i in range(len(self.conn_matrix[ID])):
            e = int(self.conn_matrix[ID][i])
            if e >= 0 and i != ID:
                self.RemoveEdge( self.graphics_edges[e] )
        
        self.nodelist[ID] = None
        self.graphics_nodes[ID] = None
        self.graphics_text[ID] = None
        
        print "Removed node #" + str(ID)
        self.Canvas.Draw(True)
        self.GetParent().SetSaveStatus(False)
                
#--------------------------------------------------------------------------------------------#    
#     Deletes an edge.                                                                       #
#     This function should not be called directly -> use DeleteSelection() instead           #
#--------------------------------------------------------------------------------------------#           
    def RemoveEdge(self, edge):   
        try:     
            ID = int(edge.Name)
            edge_obj = self.edgelist[ID]  # Pointer to the 'edge.Edge' data structure
            
#             self.graphics_edges[ edge_obj.graphic ].Visible = False   
            self.graphics_edges[ID].Visible = False                     
            self.edgelist[ID] = None
            self.graphics_edges[ID] = None        
            
            print "Removed edge #" + str(ID)
            self.Canvas.Draw(True)
            self.GetParent().SetSaveStatus(False)
            
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
                nodes[j].graphic = j
                graphics[j].Name = str(j)
                
                # Make the old text invisible and replace it in the data structure                
                text[j].Visible = False
                xy = (nodes[j].coords[0], nodes[j].coords[1])
                t = self.Canvas.AddText(str(j), xy, Size=FONT_SIZE, Position="cc", Color=TEXT_COLOR)
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
                edges[j].graphic = j                     
                graphics[j].Name = str(j) 
                
        self.edgelist = edges
        self.graphics_edges = graphics
    

#--------------------------------------------------------------------------------------------#    
#     Generates the connection matrix data structure                                         #
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
        conn_mtx = NP.empty(shape=Shape)        
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
        
    
#--------------------------------------------------------------------------------------------#    
#     For debugging purposes. Writes the connection matrix to a text file                    #
#--------------------------------------------------------------------------------------------#    
    def ExportConnectionMatrix(self, filename):        
        if self.export is False:
            conn_file = open(filename, "w")
            self.export = True
        else:
            conn_file = open(filename, "a")        
        
        L = len(self.nodelist)   
        export_mtx = self.conn_matrix[0:L,0:L]       
            
        conn_file.write(str( export_mtx ))
        conn_file.write("\n\n")
        conn_file.close()            
            

#--------------------------------------------------------------------------------------------#    
#     Returns -1 if there is no collision with another node. If there is a collision,        #
#     returns the ID of the first node there was a collision with.                           #
#--------------------------------------------------------------------------------------------#     
    
    def DetectCollision(self, new_node): 
        for existing_node in self.nodelist:
            try:
                if self.Distance(new_node.coords, existing_node.coords) <= NODE_DIAM:
                    return existing_node.id
            except AttributeError:
                pass
        return -1  
    
#     def SetOrigin(self, origin):
#         self.origin = origin
    
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
#     Selects a single node and deselects everything else.                                    #
#--------------------------------------------------------------------------------------------#   
    def SelectOneNode(self, obj):     
        self.DeselectAll(event=None)
        print "Selected Node #" + obj.Name      
        self.sel_nodes.append(obj)          
        obj.SetFillColor(HIGHLIGHT_COLOR)    
        self.last_sel_node = obj.Name
        self.Canvas.Draw(True)
        
#--------------------------------------------------------------------------------------------#    
#     Selects a single edge and deselects everything else.                                   #
#--------------------------------------------------------------------------------------------#   
    def SelectOneEdge(self, obj):     
        self.DeselectAll(event=None)
        print "Selected Edge #" + obj.Name     
        self.sel_edges.append(obj)    
        obj.SetLineColor(HIGHLIGHT_COLOR)
        self.Canvas.Draw(True) 
            
#--------------------------------------------------------------------------------------------#    
#     Selects all nodes and deselects everything else.                                       #
#--------------------------------------------------------------------------------------------#        
    def SelectNodes(self, event):
        self.DeselectAll(event)
        
        # Set highlighted colour
        for node in self.nodelist:
            self.graphics_nodes[ node.graphic ].SetFillColor(HIGHLIGHT_COLOR)
            self.sel_nodes.append(self.graphics_nodes[node.graphic])
            
        self.Canvas.Draw(True) 
        print "Selected all nodes"
        
#--------------------------------------------------------------------------------------------#    
#     Selects all edges and deselects everything else.                                       #
#--------------------------------------------------------------------------------------------#    
    def SelectEdges(self, event):
        self.DeselectAll(event)
        
        # Set highlighted colour
        for edge in self.edgelist:
            self.graphics_edges[ edge.graphic ].SetLineColor(HIGHLIGHT_COLOR)
            self.sel_edges.append(self.graphics_edges[edge.graphic])
            
        self.Canvas.Draw(True) 
        print "Selected all edges"
        
#--------------------------------------------------------------------------------------------#    
#     Select/deselect everything                                                             #
#--------------------------------------------------------------------------------------------#             
    def SelectAll(self, event):                       
        self.DeselectAll(event) 
        
        # Set highlighted colour
        for node in self.nodelist:
            self.graphics_nodes[ node.graphic ].SetFillColor(HIGHLIGHT_COLOR)
            self.sel_nodes.append(self.graphics_nodes[node.graphic])
        for edge in self.edgelist:
            self.graphics_edges[ edge.graphic ].SetLineColor(HIGHLIGHT_COLOR)
            self.sel_edges.append(self.graphics_edges[edge.graphic])
                
        self.Canvas.Draw(True) 
        print "Selected all nodes and edges"
            
    def DeselectAll(self, event):
        # Reset to original colour
        for obj in self.sel_nodes:
            obj.SetFillColor(NODE_FILL)
        for obj in self.sel_edges:
            obj.SetLineColor(EDGE_COLOR)
           
        self.Canvas.Draw(True)                
        self.sel_nodes = []
        self.sel_edges = [] 
        self.last_sel_node = ""   

#--------------------------------------------------------------------------------------------#    
#     Event when a node is left-clicked.                                                     #
#     Adds the node to the selection list and highlights it on the canvas                    #
#--------------------------------------------------------------------------------------------# 
    def OnClickNode(self, obj):
        if obj.Name != self.last_sel_node:             
            print "Selected Node #" + obj.Name      
            self.sel_nodes.append(obj)       
            obj.SetFillColor(HIGHLIGHT_COLOR)    
            self.last_sel_node = obj.Name
            self.Canvas.Draw(True)       
        
#--------------------------------------------------------------------------------------------#    
#     Event when an edge is left-clicked.                                                    #
#     Adds the node to the selection list and highlights it on the canvas                    #
#--------------------------------------------------------------------------------------------#            
    def OnClickEdge(self, obj):         
        print "Selected Edge #" + obj.Name     
        self.sel_edges.append(obj)        
        obj.SetLineColor(HIGHLIGHT_COLOR) 
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
        g = [self.nodelist, self.edgelist]
        pickle.dump(g,f)
        self.SetNodeList([])
        self.SetEdgeList([])

#--------------------------------------------------------------------------------------------#    
#     Unpickles an existing graph file from the file system.                                 #
#     NodeList is in slot 0, EdgeList is in slot 1.                                          #
#--------------------------------------------------------------------------------------------#       
    def ImportGraph(self, f):
        g = pickle.load(f)
        self.SetNodeList( g[0] )   
        self.SetEdgeList( g[1] )     

#--------------------------------------------------------------------------------------------#    
#     Iterates through an imported node list and creates the nodes.                          #
#     This function should not be called on its own, but rather as a part of SetImage()      #
#--------------------------------------------------------------------------------------------#                   
    def LoadNodes(self):
        tmp_nodelist = self.nodelist
        self.nodelist = []           
        self.graphics_nodes = []
             
        for node in tmp_nodelist:
            self.CreateNode((node.coords[0],node.coords[1]))        
        self.GetParent().SetSaveStatus(True)
            
    
#--------------------------------------------------------------------------------------------#    
#     Iterates through an imported edge list and creates the edges.                          #
#     This function should not be called on its own, but rather as a part of SetImage()      #
#--------------------------------------------------------------------------------------------#       
    def LoadEdges(self):
        tmp_edgelist = self.edgelist
        self.edgelist = []        
        self.graphics_edges = []
            
        
        for edge in tmp_edgelist:
            self.sel_nodes.append( self.graphics_nodes[ int(edge.node1) ] )
            self.sel_nodes.append( self.graphics_nodes[ int(edge.node2) ] )
            self.CreateEdges(event=None)            
        self.GetParent().SetSaveStatus(True)

#--------------------------------------------------------------------------------------------#    
#     Zooms to a given location with a given floating-point magnification.                   #
#--------------------------------------------------------------------------------------------#        
    def Zoom(self, location, magnification): 
        zoom_amt = magnification / self.current_zoom          
        self.Canvas.Zoom(zoom_amt,location)   

#--------------------------------------------------------------------------------------------#    
#     Saves the canvas as an image. (unused)                                                 #
#--------------------------------------------------------------------------------------------#
    def Binding(self, event): 
        print "Writing a png file:" 
        self.Canvas.SaveAsImage("zzzz.png") 
        print "Writing a jpeg file:" 
        self.Canvas.SaveAsImage("zzzz.jpg",wx.BITMAP_TYPE_JPEG) 
        
    
#--------------------------------------------------------------------------------------------#    
#     Clears the canvas                                                                      #
#--------------------------------------------------------------------------------------------#   
    def Clear(self):
        self.Canvas.InitAll()
        
    
    ''' Sets the image to display on the canvas. If there are nodes and/or edges saved,
        they are loaded onto the image. '''
#--------------------------------------------------------------------------------------------#    
#     Sets the image to display on the canvas. If the map has an associated graph file,      #
#     the corresponding nodes and edges are loaded and drawn onto the canvas.                #
#--------------------------------------------------------------------------------------------#
    def SetImage(self, image_file):         
        self.Clear()   
        
        # create the image 
        image = wx.Image(image_file, wx.BITMAP_TYPE_ANY).ConvertToBitmap()     
        img = self.Canvas.AddScaledBitmap( image, 
                                      (0,0), 
                                      Height=image.GetHeight(), 
                                      Position = 'bl')        
        self.LoadNodes()
        self.LoadEdges()
        self.GenerateConnectionMatrix()
    
        self.SetCurrentMapPath(image_file)
        self.Show() 
        self.Layout()
        self.Canvas.ZoomToBB()
        
        # TODO: figure out a more precise way of zooming to the image
        #         also, remap the Zoom To Fit button to the above.
        self.Zoom((self.image_width/2,self.image_width/2),2)

if __name__ == '__main__':
    app = wx.App(False) 
    F = ZoomPanel(None, title="Map Viewer", size=(700,700) ) 
    app.MainLoop() 