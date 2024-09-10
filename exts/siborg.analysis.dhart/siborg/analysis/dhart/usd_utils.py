import numpy as np
from pxr import UsdGeom, Gf, Usd

def traverse_instanced_children(prim):
    """Get every Prim child beneath `prim`, even if `prim` is instanced.

    Important:
        If `prim` is instanced, any child that this function yields will
        be an instance proxy.

    Args:
        prim (`pxr.Usd.Prim`): Some Prim to check for children.

    Yields:
        `pxr.Usd.Prim`: The children of `prim`.

    """
    for child in prim.GetFilteredChildren(Usd.TraverseInstanceProxies()):
        yield child

        for subchild in traverse_instanced_children(child):
            yield subchild

def parent_and_children_as_mesh(parent_prim):
    
    if UsdGeom.Imageable(parent_prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
        return [], []
    if parent_prim.IsA(UsdGeom.Mesh):
        points, faces = get_mesh([parent_prim])
        return points, faces
    
    found_meshes = []
    for x in traverse_instanced_children(parent_prim):
        # Check if prim is visible in the scene or not
        if UsdGeom.Imageable(x).ComputeVisibility() == UsdGeom.Tokens.invisible:
            continue
        if x.IsA(UsdGeom.Mesh):
            found_meshes.append(x)

    # children = parent_prim.GetAllChildren()
    # children = [child.GetPrimPath() for child in children]
    # points, faces = get_mesh(children)
    points, faces = get_mesh(found_meshes)
    
    return points, faces

def children_as_mesh(stage, parent_prim):
    children = parent_prim.GetAllChildren()
    children = [child.GetPrimPath() for child in children]
    points, faces = get_mesh(stage, children)
    
    return points, faces

def get_all_stage_mesh(stage):

    found_meshes = []

    # Traverse the scene graph and print the paths of prims, including instance proxies
    for x in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if x.IsA(UsdGeom.Mesh):
            found_meshes.append(x)

    points, faces = get_mesh(found_meshes)
   
    return points, faces


def get_mesh(objs):

    points, faces = [],[]

    for obj in objs:
        f_offset = len(points)
        f, p = meshconvert(obj)
        if len(p) == 0: continue
        points.extend(p)
        faces.extend(f+f_offset)

    return points, faces

def meshconvert(prim):

    # Create an XformCache object to efficiently compute world transforms
    xform_cache = UsdGeom.XformCache()

    # Get the mesh schema
    mesh = UsdGeom.Mesh(prim)
    
    # Get verts and triangles
    tris = mesh.GetFaceVertexIndicesAttr().Get()
    if not tris:
        return [], []
    tris_cnt = mesh.GetFaceVertexCountsAttr().Get()

    # Get the vertices in local space
    points_attr = mesh.GetPointsAttr()
    local_points = points_attr.Get()
    
    # Convert the VtVec3fArray to a NumPy array
    points_np = np.array(local_points, dtype=np.float64)
    
    # Add a fourth component (with value 1.0) to make the points homogeneous
    num_points = len(local_points)
    ones = np.ones((num_points, 1), dtype=np.float64)
    points_np = np.hstack((points_np, ones))

    # Compute the world transform for this prim
    world_transform = xform_cache.GetLocalToWorldTransform(prim)

    # Convert the GfMatrix to a NumPy array
    matrix_np = np.array(world_transform, dtype=np.float64).reshape((4, 4))

    # Transform all vertices to world space using matrix multiplication
    world_points = np.dot(points_np, matrix_np)

    tri_list = convert_to_triangle_mesh(tris, tris_cnt)
    # tri_list = tri_list.flatten()

    world_points = world_points[:,:3]

    return tri_list, world_points

def convert_to_triangle_mesh(FaceVertexIndices, FaceVertexCounts):
    """
    Convert a list of vertices and a list of faces into a triangle mesh.
    
    A list of triangle faces, where each face is a list of indices of the vertices that form the face.
    """
    
    # Parse the face vertex indices into individual face lists based on the face vertex counts.

    faces = []
    start = 0
    for count in FaceVertexCounts:
        end = start + count
        face = FaceVertexIndices[start:end]
        faces.append(face)
        start = end

    # Convert all faces to triangles
    triangle_faces = []
    for face in faces:
        if len(face) < 3:
            newface = []  # Invalid face
        elif len(face) == 3:
            newface = [face]  # Already a triangle
        else:
            # Fan triangulation: pick the first vertex and connect it to all other vertices
            v0 = face[0]
            newface = [[v0, face[i], face[i + 1]] for i in range(1, len(face) - 1)]

        triangle_faces.extend(newface)
    
    return np.array(triangle_faces)

