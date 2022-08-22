import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Tf, PhysxSchema, Vt
import omni.physx
from omni.physx.scripts import particleUtils


import dhart
import dhart.geometry
import dhart.raytracer
import dhart.graphgenerator


class DhartInterface():

    # Make some global class variables
    active_selection = None


    def __init__(self) -> None:

        # Container for the selected prims to be used for dhart
        self.selected_prims = None

        # The bvh that will ultimately be used 
        self.bvh = None

        self.stage = None

        # The prim to use for the starting point
        self.start_prim = None


    def set_as_start(self):
        if DhartInterface.active_selection:
            # TODO make sure this is length 1
            self.start_prim = DhartInterface.active_selection[0]
        print(f'Starting Location is now: {self.start_prim}')

    def set_as_mesh(self):
        if DhartInterface.active_selection:
            prim_selections = []

            for prim in DhartInterface.active_selection:
                prim_type = prim.GetTypeName()
                print(prim_type)
                if prim_type == 'Mesh':
                    prim_selections.append(prim)

            if prim_selections:
                self.selected_prims = prim_selections
                self.convert_mesh()
        print(f'Current meshes {self.selected_prims}')

    def scene_setup(self):
        # Setup stage info and spheres
        # Get stage.
        self.stage = omni.usd.get_context().get_stage()

        # Get default prim.
        defaultPrim = self.stage.GetDefaultPrim()
        self.defaultPrimPath = defaultPrim.GetPath().pathString

        self.sphere_path = self.defaultPrimPath + '/spheres'

    def graph(self):
    # def graph(self, start, xy, height, max_nodes, upstep, downstep, upslope, downslope):
        ''' Call DHART with the starting point and user params'''

        if not self.bvh:
            print("no bvh")
            return 

        # start_point = self.start_prim
        # xform = UsdGeom.Xformable(self.start_prim)
        start_point = omni.usd.utils.get_world_transform_matrix(self.start_prim).ExtractTranslation()
        start_point = [start_point[0], start_point[1], start_point[2]]
        spacing = (10, 10, 150)
        max_nodes = 50000

        graph = dhart.graphgenerator.GenerateGraph(self.bvh, 
                                                    start_point, 
                                                    spacing, 
                                                    max_nodes, 
                                                    up_step = 10,
                                                    down_step = 10,
                                                    up_slope = 40,
                                                    down_slope=40,
                                                    cores=-1)
        # csr = graph.CompressToCSR()
        nodes = graph.getNodes().array[['x','y','z']]

        print(f"Got {len(nodes)} Nodes")

        self.plot_nodes(nodes)
        

    def plot_nodes(self, nodes):

        self.create_geompoints(nodes.tolist())

        # # Slow method
        # for idx, node in enumerate(nodes):
        #     x, y, z = node.tolist()

        #     pos = Gf.Vec3f(x, y, z)

        #     self.create_point(f'node_{idx}', pos, 2)

    def cast_ray(self):
        origins = [(0, 20, 0), (1, 10, 1), (0, 10, 1)]
        directions = (0, -1, 0)

        hit_points = dhart.raytracer.embree_raytracer.IntersectForPoint(self.bvh, origins, directions, -1)

        print(hit_points)

        return 

    def set_active_mesh(self, prims):
        ''' set prims to use for BVH '''
        print("Set Active Mesh")
        self.selected_prims = prims

    def convert_mesh(self):
        ''' convert a prim to a meshinfo object for dhart '''
        #### Convert mesh/usd to meshinfo bvh 

        print(f'PRIMMM {self.selected_prims} - prims')

        for prim in self.selected_prims: 

            # Get mesh name (prim name)
            m = UsdGeom.Mesh(prim)

            # Get verts and triangles
            tris = m.GetFaceVertexIndicesAttr().Get()
            verts = m.GetPointsAttr().Get()

            # Get vertices as a list
            tri_list = np.array(tris)
            # tri_list = [x for x in tris]
            # Convert to numpy array of vectors, then flatten
            # vert_list = np.asarray([np.asarray(x) for x in verts]).flatten()
            vert_list = np.array(verts).flatten()

            # Insert into MeshInfo object
            MI = dhart.geometry.MeshInfo(tri_list, vert_list, "TestMesh", 39)

            self.bvh = dhart.raytracer.EmbreeBVH(MI)

        print(f"converted mesh to BVH: {self.bvh} ")

    def create_point(self, name : str, pos : Gf.Vec3f, radius : float):

        prim = self.stage.GetPrimAtPath(self.sphere_path)
        if prim.IsValid() == False:
            UsdGeom.Xform.Define(self.stage, self.sphere_path)

        spherePath = self.sphere_path + '/' + name
        sphereGeom = UsdGeom.Sphere.Define(self.stage, spherePath)

        # Set radius.
        sphereGeom.CreateRadiusAttr(radius)

        # Set color.
        sphereGeom.CreateDisplayColorAttr([(1.0, 0.0, 0.0)])

        # Set position.
        UsdGeom.XformCommonAPI(sphereGeom).SetTranslate((pos[0], pos[1], pos[2]))

    # def create_geompoints(self):
    #     # def create_geompoints(self, nodes):
    #     self.scene_setup()

    #     pointsPrim = self.stage.GetPrimAtPath(self.sphere_path)
    #     usdpoints = UsdGeom.Points(pointsPrim)

    #     return 

    def create_geompoints(self, nodes):

        particlePointsPath = Sdf.Path("/particles")

        # No velocity is needed
        vels = [(0, 0, 0) for x in nodes]
        # Nodes are all same size
        node_sizes = [2.0 for x in nodes]

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
                                                                density=0.0,
                                                            )

        prototypeStr = str(particlePointsPath) + "/particlePrototype0"
        gprim = UsdGeom.Sphere.Define(self.stage, Sdf.Path(prototypeStr))
        gprim.CreateDisplayColorAttr([(1.0, 1.0, 0.0)])
        gprim.CreateRadiusAttr().Set(0.01)

        return particles

