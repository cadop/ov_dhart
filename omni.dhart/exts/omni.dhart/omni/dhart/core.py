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

    def set_as_start(self):
        ''' Sets the DhartInterface.active_selection to the start prim '''
        if DhartInterface.active_selection:
            self.start_prim = DhartInterface.active_selection[0]
            print(f'Starting Location is now: {self.start_prim}')

            self.start_point = omni.usd.utils.get_world_transform_matrix(self.start_prim).ExtractTranslation()

            print(f'Starting point is: {self.start_point}')

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

        spacing = (2, 2, 100)
        max_nodes = 500

        graph = dhart.graphgenerator.GenerateGraph(self.bvh,
                                                    self.start_point,
                                                    spacing,
                                                    max_nodes,
                                                    up_step = 10,
                                                    down_step = 10,
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
        
        particlePointsPath = Sdf.Path("/particles")

        # No velocity is needed 
        vels = [(0, 0, 0) for x in nodes]
        # Node sizes are all the same
        node_sizes = [1.0 for x in nodes]

        particle_system_path = particleUtils.get_default_particle_system_path(self.stage)

        particles = particleUtils.add_physx_particleset_points(
                                                                self.stage,
                                                                particlePointsPath,
                                                                nodes,
                                                                velocities_list=vels,
                                                                widths_list=node_sizes,
                                                                particle_system_path=particle_system_path,
                                                                self_collision=False,
                                                                fluid=False,
                                                                particle_group=0,
                                                                particle_mass=0.001,
                                                                density=0)

        prototypeStr = str(particlePointsPath) + "/particlePrototype0"
        gprim = UsdGeom.Sphere.Define(self.stage, Sdf.Path(prototypeStr))
        gprim.CreateDisplayColorAttr([(1.0, 1.0, 0.0)])
        gprim.CreateRadiusAttr().Set(0.01)

        return particles

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
