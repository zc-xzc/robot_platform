from pathlib import Path
import sys,json,argparse
from camera_setup import HERE,SOURCE
sys.path.insert(0,str(HERE.parent/'fem_core'))
from solid_fem import gmsh_mesh

LEVELS={
 'coarse':dict(hmax=3.5,hmin=1.3,curvature_segments=12,local=2.2),
 'medium':dict(hmax=2.6,hmin=1.0,curvature_segments=16,local=1.7),
 'fine':dict(hmax=2.0,hmin=.75,curvature_segments=20,local=1.3),
}
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('level',choices=LEVELS);a=p.parse_args()
 q=dict(LEVELS[a.level]);local=q.pop('local')
 # Spatial refinement encloses the inclined thin tray; source CAD is unchanged.
 boxes=[(-24,57,-30,24,86,1,local),(-15,38,12,15,59,17,min(q['hmax'],local+0.2))]
 result=gmsh_mesh(SOURCE,HERE/'meshes'/f'camera_{a.level}.msh',refinement_boxes=boxes,**q)
 print(json.dumps(result,indent=2),flush=True)