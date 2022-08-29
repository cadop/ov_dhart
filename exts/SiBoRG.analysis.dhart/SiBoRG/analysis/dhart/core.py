from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Tf, PhysxSchema, Vt
from omni.physx.scripts import particleUtils

import omni.physx
import numpy as np

import dhart 
import dhart.geometry
import dhart.raytracer
import dhart.graphgenerator

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

    def set_as_start(self):
        ''' Sets the DhartInterface.active_selection to the start prim '''

        # Get the current active selection of the stage
        self.stage = omni.usd.get_context().get_stage()

        # Get the selections from the stage
        self._usd_context = omni.usd.get_context()
        self._selection = self._usd_context.get_selection()
        selected_paths = self._selection.get_selected_prim_paths()
        # Expects a list, so take first selection
        self.start_prim = [self.stage.GetPrimAtPath(x) for x in selected_paths][0]

        print(f'Starting Location is now: {self.start_prim}')

        self.start_point = omni.usd.utils.get_world_transform_matrix(self.start_prim).ExtractTranslation()

        print(f'Starting point is: {self.start_point}')

    def modify_start(self,x=None,y=None,z=None):
        if x:
            self.start_point[0] = x 

    def set_as_bvh(self):
        if DhartInterface.active_selection:
            prim = DhartInterface.active_selection[0]

            prim_type = prim.GetTypeName()

            if prim_type == 'Mesh':
                self.convert_to_mesh(prim)

    def generate_graph(self):
        ''' use dhart to generate a graph '''

        if not self.bvh:
            print("No BVH")
            return 

        self.scene_setup()

        spacing = (10, 10, 100)
        max_nodes = 25000

        graph = dhart.graphgenerator.GenerateGraph(self.bvh,
                                                    self.start_point,
                                                    spacing,
                                                    max_nodes,
                                                    up_step = 20,
                                                    down_step = 20,
                                                    up_slope = 40,
                                                    down_slope=40,
                                                    cores=-1)
        if graph:
            nodes = graph.getNodes().array[['x','y','z']]
            print(f'Got {len(nodes)} Nodes')
        else:
            print("FAILED")
            return 

        self.create_geompoints(nodes.tolist())

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

    def create_curve(self):

        nodes = [[0,0,0],[10,10,10],[20,20,20]]
        
        stage = omni.usd.get_context().get_stage()
        prim = UsdGeom.BasisCurves.Define(stage, "/World/Curves")
        prim.CreatePointsAttr(nodes)


        width_attr = prim.CreateWidthsAttr()
        width_attr.Set([4])

        # prim.CreateDisplayColorAttr()
        # # For RTX renderers, this only works for UsdGeom.Tokens.constant
        # color_primvar = prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant)
        # color_primvar.Set([(1,0,0)])


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
