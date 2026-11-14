"""Resume expensive completed statics without repeating FE stiffness assembly."""
import argparse
import numpy as np
import scipy.sparse as sp
from run_camera_fea import *

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('level',choices=['coarse','medium','fine'])
    p.add_argument('--sensitivity',action='store_true');p.add_argument('--modes',type=int,default=0)
    p.add_argument('--supplemental',action='store_true');a=p.parse_args()
    out=HERE/a.level
    fe=SolidFE(HERE/'meshes'/f'camera_{a.level}.msh',E,NU,RHO)
    fe.K=sp.load_npz(out/'stiffness_K.npz')
    f=np.load(out/'static_fields.npz');G=f['G'];F=f['F_1g'];U=f['U_1g']
    print('RESTORED',a.level,fe.mesh_info,flush=True)
    if a.sensitivity:
        root_support_sensitivity(fe,G,F,U,out,True)
    if a.modes or a.supplemental:
        print('BASELINE_RESTRAINT',fe.assemble(fe.facets(support_pad_mask),solver='amg'),flush=True)
    if a.supplemental:
        supplemental_static(fe,fe.facets(camera_top_contact_mask),fe.facets(screw_head_contact_mask),out,True)
    if a.modes:
        modal_analysis(fe,G,F/G0,out,a.modes)