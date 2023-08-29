
from pxr import Sdf, Usd, UsdGeom
import omni

def create_perspective_camera(stage: Usd.Stage, prim_path: str="/World/MyPerspCam") -> UsdGeom.Camera:
    camera_path = Sdf.Path(prim_path)
    usd_camera: UsdGeom.Camera = UsdGeom.Camera.Define(stage, camera_path)
    usd_camera.CreateProjectionAttr().Set(UsdGeom.Tokens.perspective)
    return usd_camera

def assign_camera():
    ''' insert actiongraph to stage (if not already there) and make a camera '''
    stage = omni.usd.get_context().get_stage()

    cam_path = "/World/FlyCam"
    camera = create_perspective_camera(stage, cam_path)


