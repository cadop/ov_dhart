from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Tf, PhysxSchema, Vt

import omni.physx
import numpy as np

import dhart 
import dhart.geometry
import dhart.raytracer
import dhart.graphgenerator
from dhart.pathfinding import DijkstraShortestPath
from dhart.visibilitygraph import VisibilityGraphAllToAll
from dhart.visibilitygraph import VisibilityGraphUndirectedAllToAll
from dhart.visibilitygraph import VisibilityGraphGroupToGroup

import time

class DhartInterface():

    # Global class variable 
    active_selection = None 

    def __init__(self):
        
        # BVH for DHART
        self.bvh = None

        # Selected Starting Location
        self.start_prim = None
        
        self.node_size = 0.1
        self.path_size = 0.3

        # Set 0 start
        self.start_point = [0,0,0]

        # Set max nodes
        self.max_nodes = 500

        self.grid_spacing = [.1,.1]
        self.height = .1

        self.targ_vis_path = '/World/TargetNodes'
        self.path_start_path = '/World/StartPath'
        self.path_end_path = '/World/EndPath'

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
    
            prim_type = prim.GetTypeName()
            # Only add if its a mesh
            if prim_type == 'Mesh':
                MI = self.convert_to_mesh(prim)
                if not made_bvh:
                    self.bvh = dhart.raytracer.EmbreeBVH(MI)
                    made_bvh = True 
                    print(f'BVH is: {self.bvh}')
                else:
                    self.bvh.AddMesh(MI)
                    print('Added to BVH')


    def set_max_nodes(self, m):
        self.max_nodes = m

    def set_spacing(self, xy):
        self.grid_spacing = [xy,xy]

    def set_height(self, z):
        self.height = z
    
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
                                                    up_step = 30,
                                                    down_step = 30,
                                                    up_slope = 10,
                                                    down_slope=10,
                                                    cores=-1)
        if self.graph:
            self.nodes = self.graph.getNodes().array[['x','y','z']]
            print(f'Got {len(self.nodes)} Nodes')
        else:
            print("FAILED")
            return 

        self.create_geompoints(self.nodes.tolist())
        # self.create_colored_geompoints(self.nodes.tolist())

    def get_to_nodes(self):

        parent_prim =  self.stage.GetPrimAtPath(self.targ_vis_path)
        children = parent_prim.GetAllChildren()
        b_nodes = []
        for child in children:
            t = omni.usd.utils.get_world_transform_matrix(child).ExtractTranslation()
            b_nodes.append(np.asarray(t))

        return b_nodes

    def visibility_graph_groups(self):

        start_t = time.time()

        nodes_a = self.nodes # Define points as the graph nodes
        
        nodes_b = self.get_to_nodes()

        VG = VisibilityGraphGroupToGroup(self.bvh, nodes_a, nodes_b, self.height) # Calculate the visibility graph
        if VG is None: return 
        # visibility_graph = VG.CompressToCSR() # Convert to a CSR (matrix)
        scores = VG.AggregateEdgeCosts(2, True) # Aggregate the visibility graph scores
        scores = scores[:-len(nodes_b)] # remove b nodes
        print(f'VG group time = {time.time()-start_t}')

        self.score_vg(scores)

    def visibility_graph(self):

        start_t = time.time()

        # height = 10 # Set a height offset to cast rays from the points
        points = self.graph.getNodes() # Define points as the graph nodes
        
        VG = VisibilityGraphUndirectedAllToAll(self.bvh, points, self.height) # Calculate the visibility graph
        # VG = VisibilityGraphAllToAll(self.bvh, points, self.height) # Calculate the visibility graph
        
        # visibility_graph = VG.CompressToCSR() # Convert to a CSR (matrix)
        scores = VG.AggregateEdgeCosts(2, True) # Aggregate the visibility graph scores

        print(f'VG time = {time.time()-start_t}')

        self.score_vg(scores)

    def score_vg(self, scores):

        start_t = time.time()
        scores = np.asarray(scores)

        bin_edges = np.histogram_bin_edges(scores, bins='auto')
        digitized = np.digitize(scores, bin_edges)-1
        idx_bins = np.unique(digitized)

        for idx in idx_bins:
            ind = np.where(digitized == idx)
            scores[ind] = bin_edges[idx]
        scores = np.asarray(scores)

        print(f'Bin time = {time.time()-start_t}')
        start_t = time.time()

        colors = calc_colors(scores)

        print(f'color time = {time.time()-start_t}')
        start_t = time.time()

        self.create_colored_geompoints(self.nodes.tolist(), colors)

        print(f'End time = {time.time()-start_t}')

    def get_path(self):
        ''' Get the shortest path by distance '''

        # DHART requires passing the desired nodes (not the positions)
        # So, we will allow users to select an arbitrary position and get the closest node ids to it

        s_prim =  self.stage.GetPrimAtPath(self.path_start_path)
        self.start_point = omni.usd.utils.get_world_transform_matrix(s_prim).ExtractTranslation()
        e_prim =  self.stage.GetPrimAtPath(self.path_end_path)
        self.end_point = omni.usd.utils.get_world_transform_matrix(e_prim).ExtractTranslation()


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
        prim = UsdGeom.Points.Define(stage, "/World/Graph/Points")
        prim.CreatePointsAttr(nodes)
        width_attr = prim.CreateWidthsAttr()
        width_attr.Set([self.node_size])

        # prim.CreateDisplayColorAttr()
        # For RTX renderers, this only works for UsdGeom.Tokens.constant

        color_primvar = prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant)
        color_primvar.Set([(1,0,0)])

    def create_colored_geompoints(self, nodes, colors=None):
        '''Create a set of geom points given some input number of nodes and colors

        Parameters
        ----------
        nodes : _type_
            _description_
        '''
        stage = omni.usd.get_context().get_stage()
        nodes = np.asarray(nodes)
        colors = np.asarray(colors)

        # Get indices of nodes that align with similar colors

        # Create a set of colors (unique elements)
        unique, color_set = np.unique(colors, axis=0, return_index=True)
        # Go through each color 
        # for color in color_set:
        #     idx = np.where(nodes == colors[color])
        #     # pull out

        # Split colors into bins, and use the indices of the bins to split spheres

        bin_nodes = nodes
        # bin_color = np.asarray([1,0,0])

        for i, color in enumerate(color_set):
            c_val =  colors[color]
            c_idx = np.where(nodes ==c_val)
            t =nodes[:,0]==c_val[0]
            condition = (colors[:,0]==c_val[0]) & (colors[:,1]==c_val[1]) & (colors[:,2]==c_val[2])
            c_idx = np.where(condition)

            if len(c_idx) == 0: continue

            node_set = np.asarray(nodes[c_idx])

            # Create a new points definition based on the number of bins
            prim = UsdGeom.Points.Define(stage, f"/World/Nodes/Points_{i}")
            prim.CreatePointsAttr(node_set)
            width_attr = prim.CreateWidthsAttr()
            width_attr.Set([self.node_size])

            # prim.CreateDisplayColorAttr()
            # For RTX renderers, this only works for UsdGeom.Tokens.constant

            # bin_color = np.random.randint(0,2,3)
            # These should all be the same value
            # bin_color = colors[idx]
            # bin_color = np.asarray(bin_color)
            color_primvar = prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant)
            color_primvar.Set(c_val)


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

    def convert_to_mesh(self, prim):
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

    # def convert_to_mesh(self, prim):
    #     ''' convert a prim to BVH '''

    #     # Get mesh name (prim name)
    #     m = UsdGeom.Mesh(prim)

    #     # Get verts and triangles
    #     tris = m.GetFaceVertexIndicesAttr().Get()

    #     tris_cnt = m.GetFaceVertexCountsAttr().Get()

    #     verts = m.GetPointsAttr().Get()

    #     tri_list = np.array(tris)
    #     vert_list = np.array(verts)

    #     # Apply any transforms to USD points 
    #     world_transform: Gf.Matrix4d = omni.usd.get_world_transform_matrix(prim)
    #     rotmat = world_transform.ExtractRotationMatrix()
    #     trans = world_transform.ExtractTranslation()
    #     vert_rotated = np.dot(vert_list, rotmat) # Rotate points

    #     # trans = np.array(trans).reshape(1,3)
    #     vert_translated = vert_rotated + trans
    #     vert_list = vert_translated.flatten()

    #     # Check if the face counts are 4, if so, reshape and turn to triangles
    #     if tris_cnt[0] == 4:
    #         quad_list = tri_list.reshape(-1,4)
    #         tri_list = quad_to_tri(quad_list)
    #         tri_list = tri_list.flatten()

    #     try:
    #         MI = dhart.geometry.MeshInfo(tri_list, vert_list, "testmesh", 0)
    #     except:
    #         print(prim)
    #     return MI 

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
    Colors = []
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
    b = min(max(0, 1.5-abs(1-4*val)),1)            #conver to hsv?
    return np.asarray([r,g,b], dtype=float)
    # tmp_color = rgb_to_hsv(r*255,g*255,b*255)
    # return ColorHSV(tmp_color[0],tmp_color[1],tmp_color[2])