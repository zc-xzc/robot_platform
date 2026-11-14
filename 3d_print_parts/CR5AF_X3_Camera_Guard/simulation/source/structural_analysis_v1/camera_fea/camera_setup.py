"""Geometry-based definitions for camera-module structural analysis.

Source CAD is read-only. Units throughout: N, mm, MPa, tonne, second.
"""
from pathlib import Path
import json,hashlib,math
import numpy as np
import cadquery as cq
from OCP.BRepClass import BRepClass_FaceClassifier
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_IN,TopAbs_ON

HERE=Path(__file__).resolve().parent
PROJECT=HERE.parents[1]
SOURCE=PROJECT/'recovery_v2/camera/04_Camera_Module_Short45_F.brep'
CART=PROJECT/'recovery_v2/mount/02_Fuse_Cartridge_D8.brep'
CAMERA=PROJECT/'recovery_v2/camera/reference/Insta360_X3_OngXeno_v6.step'
PARAMS=json.loads((PROJECT/'recovery_v2/camera/camera_module_validation.json').read_text())
T_FC=np.asarray(PARAMS['camera_C_to_flange_F'],float)
R=T_FC[:3,:3];T=T_FC[:3,3]
COG_C=np.array([0.,0.,57.]);COG_F=R@COG_C+T
MASS_CAMERA=0.00018
E=2000.;NU=.35;RHO=1.24e-9;G0=9806.65
W,D,H=46.,33.1,114.
I_C=MASS_CAMERA/12*np.diag([D*D+H*H,W*W+H*H,W*W+D*D])
I_F=R@I_C@R.T
M_CAMERA=np.zeros((6,6));M_CAMERA[:3,:3]=MASS_CAMERA*np.eye(3);M_CAMERA[3:,3:]=I_F
assert np.allclose(COG_F,[0,112.3050865276332,25.3050865276332],atol=1e-10)

_PLATFORM_FACES=None
_CAMERA_BOTTOM_FACES=None

def bbox(s):
    b=s.BoundingBox();return [b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax]

def planar_z_faces(s,z):
    return [f for f in s.Faces() if f.geomType()=='PLANE'
            and abs(f.BoundingBox().zmin-z)<1e-6 and abs(f.BoundingBox().zmax-z)<1e-6]

def to_C(x):
    arr=np.asarray(x);shape=arr.shape
    return (R.T@(arr.reshape(3,-1)-T[:,None])).reshape(shape)

def _faces():
    global _PLATFORM_FACES,_CAMERA_BOTTOM_FACES
    if _PLATFORM_FACES is None:
        _PLATFORM_FACES=planar_z_faces(cq.Shape.importBrep(str(CART)),13.)
        solids=cq.importers.importStep(str(CAMERA)).val().Solids()
        _CAMERA_BOTTOM_FACES=planar_z_faces(solids[5],0.)+planar_z_faces(solids[7],0.)
    return _PLATFORM_FACES,_CAMERA_BOTTOM_FACES

def inside_faces(points,faces):
    points=np.asarray(points)
    result=np.zeros(points.shape[1],bool)
    for i,p in enumerate(points.T):
        pt=gp_Pnt(float(p[0]),float(p[1]),float(p[2]))
        result[i]=any(BRepClass_FaceClassifier(f.wrapped,pt,1e-5).State()
                      in (TopAbs_IN,TopAbs_ON) for f in faces)
    return result

def support_pad_mask(x):
    """Actual module foot plane overlapping actual cartridge top faces.

    All 3 translations fixed in the baseline: an engaged, rigid-root tied
    support idealization. It is not frictionless unilateral contact and does
    not solve the as-built 0.3-mm axial / 0.6-mm diametral free play.
    """
    x=np.asarray(x);mask=np.abs(x[2]-13.)<1e-5
    mask &= (np.abs(x[0])<=14.00001)&(x[1]>=38.99999)&(x[1]<=58.00001)
    ids=np.flatnonzero(mask)
    if len(ids):mask[ids]=inside_faces(x[:,ids],_faces()[0])
    return mask

def captured_stud_mask(x):
    """Alternative restraint only in actual head-projected foot-face patches."""
    x=np.asarray(x)
    radial=np.minimum((x[0]-8)**2+(x[1]-50)**2,(x[0]+8)**2+(x[1]-50)**2)<=3.5**2
    return radial & ((np.abs(x[2]-13)<1e-5)|(np.abs(x[2]-16)<1e-5))

def camera_top_contact_mask(x):
    c=to_C(x);mask=np.abs(c[2])<1e-5;ids=np.flatnonzero(mask)
    if len(ids):mask[ids]=inside_faces(c[:,ids],_faces()[1])
    return mask

def screw_head_contact_mask(x):
    c=to_C(x)
    return (np.abs(c[2]+2)<1e-5)&(c[0]**2+c[1]**2<=10.**2+1e-6)

def camera_load_patch_mask(x):
    return camera_top_contact_mask(x)|screw_head_contact_mask(x)

def regions():
    def thin(x):
        c=to_C(x);return (c[2]>=-2.000001)&(c[2]<=.000001)&(np.abs(c[0])<=22)&(np.abs(c[1])<=16)
    def tray(x):
        c=to_C(x);return (c[2]>=-8.000001)&(c[2]<=.000001)&(np.abs(c[0])<=27.2)&(np.abs(c[1])<=21.55)&~thin(x)
    def arms(x):return (np.abs(x[0])>=7.999)&(np.abs(x[0])<=14.001)&(x[1]>=54.999)&(x[1]<=65.001)&(x[2]>=-6.001)&(x[2]<=20.001)
    def foot(x):return (np.abs(x[0])<=14.001)&(x[1]>=38.999)&(x[1]<=58.001)&(x[2]>=12.999)&(x[2]<=16.001)
    def posts(x):return to_C(x)[2]>14.
    return {'2mm_bearing_web':thin,'tray_perimeter':tray,'short_45deg_arms':arms,
            'root_mounting_foot':foot,'guard_posts':posts}

def metadata():
    part=cq.Shape.importBrep(str(SOURCE));platform,camera_faces=_faces()
    report={'source_CAD':str(SOURCE),'source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
      'source_volume_mm3':part.Volume(),'source_valid':part.isValid(),'source_solids':len(part.Solids()),
      'source_frame_mass_kg_assumed_density':part.Volume()*RHO*1000,
      'source_frame_CoG_F_mm':list(part.Center().toTuple()),
      'camera_C_to_F':T_FC.tolist(),'camera_mass_kg':.18,
      'camera_CoG_C_mm':COG_C.tolist(),'camera_CoG_F_mm':COG_F.tolist(),
      'camera_inertia_C_tonne_mm2':I_C.tolist(),'camera_inertia_F_tonne_mm2':I_F.tolist(),
      'inertia_basis':'Uniform rectangular proxy 46 x 33.1 x 114 mm, not measured mass distribution',
      'material_assumptions':{'E_MPa':E,'nu':NU,'density_tonne_mm3':RHO,
        'constitutive_model':'Homogeneous isotropic linear elastic; not measured printed material or tensile/failure strength'},
      'reference_bottom_contact_faces':[{'area_mm2':f.Area(),'bounds_C_mm':bbox(f)} for f in camera_faces],
      'actual_platform_top_faces':[{'area_mm2':f.Area(),'bounds_F_mm':bbox(f)} for f in platform],
      'root_interface_nominal_gaps':{'head_bottom_Fz':16.3,'foot_top_Fz':16.,'axial_gap_mm':.3,
        'slot_width_mm':4.2,'stem_diameter_mm':3.6,'diametral_gap_mm':.6,'gap_contact_solved':False},
      'screw_assumptions':{'thread':'1/4-20 UNC nominal','underhead_length_mm':5,
        'bearing_web_mm':2,'nominal_camera_projection_mm':3,'head_diameter_mm':20,'head_height_mm':3,
        'actual_user_head_dimensions_known':False,'actual_camera_allowed_thread_depth_known':False,
        'axial_pad_or_washer_mm':0},
      'boundary_conditions':{
        'baseline':'Fix all translations on actual foot-bottom/cartridge-top overlap faces at Fz13. Engaged rigid-root support idealization; does not model friction/contact free play.',
        'sensitivity':'Restrain only foot faces inside two diameter7 head projections around (±8,50). Also an engaged restraint idealization.',
        'load_transfer':'Equilibrated resultant force/moment on original camera bottom reference contact plus nominal screw-head underside annulus. No tray face is fixed.',
        'rigid_camera_mass_mapping':'6-column RBE3-like wrench matrix G; q=G.T u; attached mass G M_camera G.T. Verify G.T times rigid-body modes = I6.'},
      'scope_limits':['No preload/contact opening/closing or slip model','No failure/fracture or fatigue model',
        'No printed anisotropy or temperature dependence','Modes/FRF are linearized engaged-support sensitivity',
        '1g and 3g mean total resultant effective acceleration, not gravity plus an extra 3g',
        '10g and quasi-static collision loads are sensitivity cases, not transient impact validation']}
    HERE.mkdir(parents=True,exist_ok=True)
    (HERE/'geometry_BC_load_definitions.json').write_text(json.dumps(report,indent=2))
    return report

if __name__=='__main__':
    d=metadata();print(json.dumps({k:d[k] for k in ['source_volume_mm3','source_frame_mass_kg_assumed_density',
        'source_frame_CoG_F_mm','camera_CoG_F_mm','camera_inertia_F_tonne_mm2',
        'reference_bottom_contact_faces','actual_platform_top_faces']},indent=2),flush=True)