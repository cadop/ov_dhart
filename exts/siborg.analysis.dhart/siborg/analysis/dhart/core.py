''' This is the core of the DHART interface. 
It is responsible for interfacing with the DHART library and generating the graph and paths. 
WARNING: Do not call this module! Use the dhart_handler to maintain reference'''

from collections import defaultdict
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Tf, PhysxSchema, Vt

try: 
    from omni.usd.utils import get_world_transform_matrix
except: 
    from omni.usd import get_world_transform_matrix

import omni.physx

import numpy as np
from scipy.interpolate import splprep, splev

import dhart 
import dhart.geometry
import dhart.raytracer
import dhart.graphgenerator

from dhart import pathfinding

from dhart.visibilitygraph import VisibilityGraphAllToAll
from dhart.visibilitygraph import VisibilityGraphUndirectedAllToAll
from dhart.visibilitygraph import VisibilityGraphGroupToGroup


from dhart.spatialstructures.cost_algorithms import CalculateEnergyExpenditure, CostAlgorithmKeys
from dhart.spatialstructures import Graph, Direction

from . import usd_utils

import time
class DhartInterface():

    # Global class variable 
    active_selection = None 
    
    def __init__(self):

        print(f'Running INIT on dhart interface')
        # BVH for DHART
        self.bvh = None
        self.vis_bvh = None # Separate BVH for storing opaque objects

        self.VG = None
        self.VG_group = None
        self.graph = None
        
        # Selected Starting Location
        self.start_prim = None
        
        self.node_size = 0.05
        self.path_size = 0.1

        # Set 0 start
        self.start_point = [0,0,0.5]

        # Set max nodes
        self.max_nodes = 400

        self.grid_spacing = [0.1,0.1]
        self.height = 1.60
        self.upstep = 0.2
        self.downstep = 0.2
        self.upslope = 20
        self.downslope = 20

        self.targ_vis_path = '/World/TargetNodes'
        self.path_start_path = '/World/StartPath'
        self.path_end_path = '/World/EndPath'

        # Track 
        self.gui_start = []
        self.gui_end = []

        
        self._current_attrs = defaultdict(list)
        self.graph_nodes = []
        self.node_pnts = []        
        self._initialized = True

        


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
        # try:
        #     selected_point = get_world_transform_matrix(prim).ExtractTranslation()
        # except:
        #     selected_point = omni.usd.get_world_transform_matrix(prim).ExtractTranslation()
        selected_point = get_world_transform_matrix(prim).ExtractTranslation()
        
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
        # Set the graph bvh
        self.set_bvh(vis=False)

    def set_as_vis_bvh(self):
        # Set the visibility-based bvh
        self.set_bvh(vis=True)

    def set_bvh(self, vis=False):

        # Get the current active selection of the stage
        self.stage = omni.usd.get_context().get_stage()

        # Get the selections from the stage
        self._usd_context = omni.usd.get_context()
        self._selection = self._usd_context.get_selection()
        selected_paths = self._selection.get_selected_prim_paths()
        # Expects a list, so take first selection
        prims = [self.stage.GetPrimAtPath(x) for x in selected_paths]

        # Record if a BVH was generated
        made_bvh = False 
   
        for prim in prims: 
            if UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
                continue
            MI = self.convert_to_mesh(prim) 
            if MI is None:
                print("BVH FAILED")
                continue 
            if not made_bvh:
                if vis:
                    self.vis_bvh = dhart.raytracer.EmbreeBVH(MI)
                    print(f'VIS BVH is: {self.vis_bvh}')
                else:
                    self.bvh = dhart.raytracer.EmbreeBVH(MI)
                    # self.bvh = dhart.raytracer.EmbreeBVH(MI, use_precise=True)
                    print(f'BVH is: {self.bvh}')
                made_bvh = True 
            else:
                if vis:
                    self.vis_bvh.AddMesh(MI)
                    print('Added to VIS BVH')
                else:
                    self.bvh.AddMesh(MI)
                    print('Added to BVH')

        # for prim in prims:
    
        #     prim_type = prim.GetTypeName()
        #     # Only add if its a mesh
        #     if prim_type == 'Mesh':
        #         MI = self.convert_to_mesh_old(prim)
        #         # MI = self.convert_to_mesh(prim)
        #         if MI is None:
        #             print("BVH FAILED")
        #             continue 
        #         if not made_bvh:
        #             if vis:
        #                 self.vis_bvh = dhart.raytracer.EmbreeBVH(MI)
        #                 print(f'VIS BVH is: {self.vis_bvh}')
        #             else:
        #                 self.bvh = dhart.raytracer.EmbreeBVH(MI)
        #                 print(f'BVH is: {self.bvh}')
        #             made_bvh = True 
        #         else:
        #             if vis:
        #                 self.vis_bvh.AddMesh(MI)
        #                 print('Added to VIS BVH')
        #             else:
        #                 self.bvh.AddMesh(MI)
        #                 print('Added to BVH')

        # for prim in prims:
    
        #     prim_type = prim.GetTypeName()
        #     # Only add if its a mesh
        #     if prim_type == 'Mesh':
        #         MI = self.convert_to_mesh_old(prim)
        #         # MI = self.convert_to_mesh(prim)
        #         if MI is None:
        #             print("BVH FAILED")
        #             continue 
        #         if not made_bvh:
        #             if vis:
        #                 self.vis_bvh = dhart.raytracer.EmbreeBVH(MI)
        #                 print(f'VIS BVH is: {self.vis_bvh}')
        #             else:
        #                 self.bvh = dhart.raytracer.EmbreeBVH(MI)
        #                 print(f'BVH is: {self.bvh}')
        #             made_bvh = True 
        #         else:
        #             if vis:
        #                 self.vis_bvh.AddMesh(MI)
        #                 print('Added to VIS BVH')
        #             else:
        #                 self.bvh.AddMesh(MI)
        #                 print('Added to BVH')

    def set_max_nodes(self, m):
        self.max_nodes = m

    def set_spacing(self, xy):
        self.grid_spacing = [xy,xy]

    def set_height(self, z):
        self.height = z
    
    def set_upstep(self, u):
        self.upstep = u

    def set_upslope(self, us):
        self.upslope = us

    def set_downstep(self, d):
        self.downstep = d

    def set_downslope(self, ds):
        self.downslope = ds

    def set_nodesize(self, s):
        self.node_size = s

    def set_pathsize(self, s):
        self.path_size = s

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
                                                    up_step = self.upstep,
                                                    down_step = self.downstep,
                                                    up_slope = self.upslope,
                                                    down_slope=self.downslope,
                                                    cores=-1)
        if self.graph:
            self.graph.CompressToCSR()
            self.nodes = self.graph.getNodes().array[['x','y','z']]
            self.graph_nodes = self.graph.getNodes().array['id']
            self.node_pnts = self.graph.get_node_points()
            print(f'Got {len(self.nodes)} Nodes')
        else:
            print("FAILED")
            return 

        self.create_geompoints(self.nodes.tolist())

    def get_to_nodes(self):

        parent_prim =  self.stage.GetPrimAtPath(self.targ_vis_path)
        children = parent_prim.GetAllChildren()
        b_nodes = []
        for child in children:
            t = get_world_transform_matrix(child).ExtractTranslation()
            b_nodes.append(np.asarray(t))

        return b_nodes

    def visibility_graph_groups(self):
        start_t = time.time()

        nodes_a = self.nodes # Define points as the graph nodes
        nodes_b = self.get_to_nodes()

        self.VG_group = VisibilityGraphGroupToGroup(self.vis_bvh, nodes_a, nodes_b, self.height) # Calculate the visibility graph
        if self.VG_group is None: 
            print('VG Failed')
            return 
        # visibility_graph = VG.CompressToCSR() # Convert to a CSR (matrix)
        scores = self.VG_group.AggregateEdgeCosts(2, True) # Aggregate the visibility graph scores
        scores = scores[:-len(nodes_b)] # remove b nodes
        print(f'VG group time = {time.time()-start_t}')

        # add scores to node attributes
        attr = "vg_group"
        ids = self.graph.getNodes().array['id']
        str_scores = [str(1.0/(x+0.01)) for x in scores] # we need scores to be a string for dhart. was to encode any type of node data
        self.graph.add_node_attributes(attr, ids, str_scores)

        self.score_vg(scores)

    def visibility_graph(self):
        start_t = time.time()

        points = self.graph.getNodes() # Define points as the graph nodes        
        self.VG = VisibilityGraphUndirectedAllToAll(self.vis_bvh, points, self.height) # Calculate the visibility graph
        # VG = VisibilityGraphAllToAll(self.bvh, points, self.height) # Calculate the visibility graph
        
        scores = self.VG.AggregateEdgeCosts(2, True) # Aggregate the visibility graph scores
        print(f'VG time = {time.time()-start_t}')

        # add scores to node attributes
        attr = "vg_all"
        ids = self.graph.getNodes().array['id']
        str_scores = [str(x) for x in scores] # we need scores to be a string for dhart. was to encode any type of node data
        self.graph.add_node_attributes(attr, ids, str_scores)


        self.score_vg(scores)

    def reset_endpoints(self):

        s_prim =  self.stage.GetPrimAtPath(self.path_start_path)
        self.start_point = get_world_transform_matrix(s_prim).ExtractTranslation()
        e_prim =  self.stage.GetPrimAtPath(self.path_end_path)
        self.end_point = get_world_transform_matrix(e_prim).ExtractTranslation()

    def get_path(self):
        ''' Get the shortest path by distance '''

        # DHART requires passing the desired nodes (not the positions)
        # So, we will allow users to select an arbitrary position and get the closest node ids to it


        #### !!!! This was screwing up the graph generation because the start point was a bunch of decimals
        self.reset_endpoints()
        self.start_point = [int(x) for x in self.start_point]


        p_desired = np.array([self.start_point, self.end_point])
        closest_nodes = self.graph.get_closest_nodes(p_desired)
 
        # nodes = self.graph.getNodes()
        # matching_items = nodes[np.isin(nodes['id'], closest_nodes)]
        # print(f'Start/End Nodes {matching_items}')
        # self.show_nodes(matching_items[['x','y','z']].tolist())

        path = pathfinding.DijkstraShortestPath(self.graph, closest_nodes[0], closest_nodes[1])
        # print(f'Path Nodes: {path}')
        if path is None:
            print("No PATH Found")
            return

        path_xyz = np.take(self.nodes[['x','y','z']], path['id'])

        print(len(path_xyz.tolist()))

        path_list = path_xyz.tolist()
        path_nodes = self.smooth_path(path_list)
        self.create_curve(path_nodes)
        # self.show_nodes(path_xyz.tolist())


    def smooth_path(self, input_points):
        '''
        Interpolate the path and smooth the verts to be shown
        '''
        def interpolate_curve(points, num_points=100):
            """Interpolate the curve to produce a denser set of points."""
            tck, u = splprep([[p[0] for p in points], [p[1] for p in points], [p[2] for p in points]], s=0)
            u_new = np.linspace(0, 1, num_points)
            x_new, y_new, z_new = splev(u_new, tck)
            return list(zip(x_new, y_new, z_new))

        # Re-define the moving_average function as provided by you
        def moving_average(points, window_size=3):
            """Smoothen the curve using a moving average."""
            if window_size < 3:
                return points  # Too small window, just return original points

            extended_points = points[:window_size-1] + points + points[-(window_size-1):]
            smoothed_points = []

            for i in range(len(points)):
                window = extended_points[i:i+window_size]
                avg_x = sum(pt[0] for pt in window) / window_size
                avg_y = sum(pt[1] for pt in window) / window_size
                avg_z = sum(pt[2] for pt in window) / window_size
                smoothed_points.append((avg_x, avg_y, avg_z))

            return smoothed_points

        # Smooth the original input points
        smoothed_points = moving_average(input_points, window_size=4)

        # Interpolate the smoothed curve to produce a denser set of points
        interpolated_points = interpolate_curve(smoothed_points, num_points=500)

        # Smooth the denser set of points
        smoothed_interpolated_points = moving_average(interpolated_points, window_size=6)

        return smoothed_interpolated_points

    def show_nodes(self, nodes):
        stage = omni.usd.get_context().get_stage()
        prim = UsdGeom.Points.Define(stage, "/World/Graph/Closest")
        prim.CreatePointsAttr(nodes)
        width_attr = prim.CreateWidthsAttr()
        width_attr.Set([self.node_size*2])

        # prim.CreateDisplayColorAttr()
        # For RTX renderers, this only works for UsdGeom.Tokens.constant

        color_primvar = prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant)
        color_primvar.Set([(0,0,1)])


    def get_energy_path(self):
        # Get the key
        energy_cost_key = CostAlgorithmKeys.ENERGY_EXPENDITURE
        CalculateEnergyExpenditure(self.graph)

        self.graph.attrs_to_costs("vg_group", "vg_group_cost", Direction.INCOMING)

        # self.graph.GetEdgeCost(1, 2, "vg_group_cost")

        # get custom path based on vg
        p_desired = np.array([self.start_point, self.end_point])
        closest_nodes = self.graph.get_closest_nodes(p_desired)

        # Call the shortest path again, with the optional cost type
        visibility_path = pathfinding.DijkstraShortestPath(self.graph, closest_nodes[0], closest_nodes[1], 'vg_group_cost')
        path_xyz = np.take(self.nodes[['x','y','z']], visibility_path['id'])
        self.create_curve(path_xyz.tolist())

    def get_vg_path(self):
    
        # make sure visibility graph was made
        
        # get node ids and attrs of vg
        self.reset_endpoints()

        # assign new node attrs for visibility
        csr = self.graph.CompressToCSR()

        # Get attribute scores from the graph
        # out_attrs = self.graph.get_node_attributes(attr)

        self.graph.attrs_to_costs("vg_all", "vg_all_cost", Direction.INCOMING)

        # self.graph.GetEdgeCost(1, 2, "vg_group_cost")

        # get custom path based on vg
        p_desired = np.array([self.start_point, self.end_point])
        closest_nodes = self.graph.get_closest_nodes(p_desired)

        # Call the shortest path again, with the optional cost type
        visibility_path = pathfinding.DijkstraShortestPath(self.graph, closest_nodes[0], closest_nodes[1], 'vg_all_cost')
        path_xyz = np.take(self.nodes[['x','y','z']], visibility_path['id'])
        self.create_curve(path_xyz.tolist())


    def scene_setup(self):
        # Get stage.
        self.stage = omni.usd.get_context().get_stage()


    def score_vg(self, scores):

        scores = np.asarray(scores)
        colors = calc_colors(scores)
        self.create_colored_geompoints(self.nodes.tolist(), colors)

    def create_colored_geompoints(self, nodes, colors=None):

        stage = omni.usd.get_context().get_stage()
        prim = UsdGeom.Points.Define(stage, "/World/Graph/VGPoints")
        prim.CreatePointsAttr(nodes)
        width_attr = prim.CreateWidthsAttr()
        width_attr.Set([self.node_size])

        color_primvar = prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex)
        color_primvar.Set(colors)

    def create_geompoints(self, nodes):
        
        stage = omni.usd.get_context().get_stage()
        prim = UsdGeom.Points.Define(stage, "/World/Graph/Points")
        prim.CreatePointsAttr(nodes)
        width_attr = prim.CreateWidthsAttr()
        width_attr.Set([self.node_size])

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
        # width_attr = prim.CreateWidthsAttr(UsdGeom.Tokens.varying)
        width_attr.Set([self.path_size for x in range(len(nodes))])

        color_primvar = prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant)
        color_primvar.Set([(0,1,0)])

        return 

    def convert_to_mesh(self,prim):

        vert_list, tri_list  = usd_utils.parent_and_children_as_mesh(prim)
        # vert_list, tri_list  = usd_utils.get_mesh([prim])
        tri_list = np.array(tri_list).flatten()
        vert_list = np.array(vert_list).flatten()

        try:
            MI = dhart.geometry.MeshInfo(tri_list, vert_list, "testmesh", 0)
            return MI 
        except:
            print(prim)
        return None

    def convert_to_mesh_old(self, prim):
        ''' convert a prim to BVH '''

        # Get mesh name (prim name)
        m = UsdGeom.Mesh(prim)

        # Get verts and triangles
        tris = m.GetFaceVertexIndicesAttr().Get()

        tris_cnt = m.GetFaceVertexCountsAttr().Get()

        verts = m.GetPointsAttr().Get()

        tri_list = np.array(tris)
        vert_list = np.array(verts)

        xform = UsdGeom.Xformable(prim)
        time = Usd.TimeCode.Default() # The time at which we compute the bounding box
        world_transform: Gf.Matrix4d = xform.ComputeLocalToWorldTransform(time)
        translation: Gf.Vec3d = world_transform.ExtractTranslation()
        rotation: Gf.Rotation = world_transform.ExtractRotationMatrix()
        scale: Gf.Vec3d = Gf.Vec3d(*(v.GetLength() for v in world_transform.ExtractRotationMatrix()))

        vert_rotated = np.dot(vert_list, rotation) # Rotate points

        vert_translated = vert_rotated + translation

        vert_scaled = vert_translated
        vert_scaled[:,0] *= scale[0]
        vert_scaled[:,1] *= scale[1]
        vert_scaled[:,2] *= scale[2]

        vert_list = vert_scaled
        vert_list = vert_translated.flatten()

        # Check if the face counts are 4, if so, reshape and turn to triangles
        if tris_cnt[0] == 4:
            quad_list = tri_list.reshape(-1,4)
            tri_list = quad_to_tri(quad_list)
            tri_list = tri_list.flatten()

        try:
            MI = dhart.geometry.MeshInfo(tri_list, vert_list, "testmesh", 0)
            return MI 
        except:
            print(prim)
        return None


def quad_to_tri(a):
    idx = np.flatnonzero(a[:,-1] == 0)
    out0 = np.empty((a.shape[0],2,3),dtype=a.dtype)      

    out0[:,0,1:] = a[:,1:-1]
    out0[:,1,1:] = a[:,2:]

    out0[...,0] = a[:,0,None]

    out0.shape = (-1,3)

    mask = np.ones(out0.shape[0],dtype=bool)
    mask[idx*2+1] = 0
    return out0[mask]


def calc_colors(scores):
    filtered_scores = [score for score in scores if score>=0]
    max_v = max(filtered_scores)
    min_v = min(filtered_scores)
    print('maxmin')
    print(max_v)
    print(min_v)
    print('')
    if (max_v == 0):
        print("Max is zero!")
        exit()
    elif(max_v - min_v == 0):
        print("All values are the same!")
        return [colorbar(1.0)] * len(scores)
        
    return [
            colorbar((point-min_v)/(max_v-min_v))
            if point >= 0
            else None
            for point in scores
    ]

def colorbar(val):
    r = min(max(0, 1.5-abs(1-4*(val-0.5))),1)
    g = min(max(0, 1.5-abs(1-4*(val-0.25))),1)
    b = min(max(0, 1.5-abs(1-4*val)),1)
    return np.asarray([r,g,b], dtype=float)
