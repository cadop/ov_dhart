from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Tf, PhysxSchema, Vt
from omni.physx.scripts import particleUtils

import omni.physx
import numpy as np

import dhart 
import dhart.geometry
import dhart.raytracer
import dhart.graphgenerator
from dhart.pathfinding import DijkstraShortestPath

class DhartInterface():

    # Global class variable 
    active_selection = None 

    def __init__(self):
        
        # BVH for DHART
        self.bvh = None

        # Selected Starting Location
        self.start_prim = None

        # Set 0 start
        self.start_point = [0,0,0]

        # Set max nodes
        self.max_nodes = 500

        self.grid_spacing = [1,1]
        self.height = 10

        # Track 
        self.gui_start = []
        self.gui_end = []

    def set_as_start(self):
        ''' Sets the DhartInterface.active_selection to the start prim '''
        self.start_point = self.get_selected_as_point()
        self.gui_start[0].model.set_value(self.start_point[0])
        self.gui_start[1].model.set_value(self.start_point[1])
        self.gui_start[2].model.set_value(self.start_point[2])

    def set_as_end(self):
        ''' Sets the DhartInterface.active_selection to the end prim '''
        self.end_point = self.get_selected_as_point()
        self.gui_end[0].model.set_value(self.end_point[0])
        self.gui_end[1].model.set_value(self.end_point[1])
        self.gui_end[2].model.set_value(self.end_point[2])

    def get_selected_as_point(self):
        ''' Sets the DhartInterface.active_selection to the start prim '''

        # Get the current active selection of the stage
        self.stage = omni.usd.get_context().get_stage()

        # Get the selections from the stage
        self._usd_context = omni.usd.get_context()
        self._selection = self._usd_context.get_selection()
        selected_paths = self._selection.get_selected_prim_paths()
        # Expects a list, so take first selection
        prim = [self.stage.GetPrimAtPath(x) for x in selected_paths][0]

        selected_point = omni.usd.utils.get_world_transform_matrix(prim).ExtractTranslation()
        return selected_point

    def modify_start(self,x=None,y=None,z=None):
        if x: self.start_point[0] = x 
        if y: self.start_point[1] = y
        if z: self.start_point[2] = z 

        print(f'START IS: {self.start_point}')

    def modify_end(self,x=None,y=None,z=None):
        if x: self.end_point[0] = x 
        if y: self.end_point[1] = y
        if z: self.end_point[2] = z 
        
        print(f'END IS: {self.end_point}')

    def set_as_bvh(self):
        if DhartInterface.active_selection:
            prim = DhartInterface.active_selection[0]

            prim_type = prim.GetTypeName()

            if prim_type == 'Mesh':
                self.convert_to_mesh(prim)

    def set_max_nodes(self, m):
        self.max_nodes = m

    def set_spacing(self, xy):
        self.grid_spacing = xy

    def set_height(self, z):
        self.height = z

    def generate_graph(self):
        ''' use dhart to generate a graph '''

        if not self.bvh:
            print("No BVH")
            return 

        self.scene_setup()

        spacing = (self.grid_spacing[0], self.grid_spacing[1], self.height)
        max_nodes = self.max_nodes

        self.graph = dhart.graphgenerator.GenerateGraph(self.bvh,
                                                    self.start_point,
                                                    spacing,
                                                    max_nodes,
                                                    up_step = 20,
                                                    down_step = 20,
                                                    up_slope = 40,
                                                    down_slope=40,
                                                    cores=-1)
        if self.graph:
            self.nodes = self.graph.getNodes().array[['x','y','z']]
            print(f'Got {len(self.nodes)} Nodes')
        else:
            print("FAILED")
            return 

        self.create_geompoints(self.nodes.tolist())


    def get_path(self):
        ''' Get the shortest path by distance '''

        # DHART requires passing the desired nodes (not the positions)
        # So, we will allow users to select an arbitrary position and get the closest node ids to it

        p_desired = np.array([self.start_point, self.end_point])
        closest_nodes = self.graph.get_closest_nodes(p_desired)

        path = DijkstraShortestPath(self.graph, closest_nodes[0], closest_nodes[1])
        path_xyz = np.take(self.nodes[['x','y','z']], path['id'])

        self.create_curve(path_xyz.tolist())

        return 

    def scene_setup(self):
        # Get stage.
        self.stage = omni.usd.get_context().get_stage()

    def create_geompoints(self, nodes):
        
        stage = omni.usd.get_context().get_stage()
        prim = UsdGeom.Points.Define(stage, "/World/Points")
        prim.CreatePointsAttr(nodes)
        width_attr = prim.CreateWidthsAttr()
        width_attr.Set([4])

        prim.CreateDisplayColorAttr()
        # For RTX renderers, this only works for UsdGeom.Tokens.constant
        color_primvar = prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant)
        color_primvar.Set([(1,0,0)])

    def create_curve(self, nodes):
        '''Create and draw a BasisCurve on the stage following the nodes'''

        stage = omni.usd.get_context().get_stage()
        prim = UsdGeom.BasisCurves.Define(stage, "/World/Path")
        prim.CreatePointsAttr(nodes)

        # Set the number of curve verts to be the same as the number of points we have
        curve_verts = prim.CreateCurveVertexCountsAttr()
        curve_verts.Set([len(nodes)])

        # Set the curve type to linear so that each node is connected to the next
        type_attr = prim.CreateTypeAttr()
        type_attr.Set('linear')
        type_attr = prim.GetTypeAttr().Get()
        # Set the width of the curve
        width_attr = prim.CreateWidthsAttr()
        width_attr.Set([4 for x in range(len(nodes))])

        color_primvar = prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant)
        color_primvar.Set([(0,1,0)])

        return 

    def convert_to_mesh(self, prim):
        ''' convert a prim to BVH '''

        # Get mesh name (prim name)
        m = UsdGeom.Mesh(prim)

        # Get verts and triangles
        tris = m.GetFaceVertexIndicesAttr().Get()
        verts = m.GetPointsAttr().Get()

        tri_list = np.array(tris)
        vert_list = np.array(verts).flatten()

        MI = dhart.geometry.MeshInfo(tri_list, vert_list, "testmesh", 0)

        self.bvh = dhart.raytracer.EmbreeBVH(MI)

        print(f'BVH is: {self.bvh}')
