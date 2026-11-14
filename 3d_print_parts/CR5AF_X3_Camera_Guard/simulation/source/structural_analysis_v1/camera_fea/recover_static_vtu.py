"""Recover completed static states from actual saved double-precision VTU.

Rebuild K only to independently evaluate original field residuals/reactions.
Save the inertial states before optional subsequent unit-force work.
"""
import argparse,time,json
from scipy.spatial import cKDTree
import meshio
from run_camera_fea import *

def main(level,unit_forces=False):
    out=HERE/level;mesh=HERE/'meshes'/f'camera_{level}.msh'
    fe=SolidFE(mesh,E,NU,RHO)
    support=fe.facets(support_pad_mask)
    fe.fixed=np.unique(fe.basis.get_dofs(facets=support).all())
    fe.free=np.setdiff1d(np.arange(fe.basis.N),fe.fixed)
    top=fe.facets(camera_top_contact_mask);head=fe.facets(screw_head_contact_mask)
    G,mapping=wrench_mapping(fe,np.unique(np.r_[top,head]))
    U=[];F=[];provenance=[]
    xyz=fe.basis.doflocs[:,fe.components[0]].T
    for j in range(3):
        path=out/f'camera_acceleration_plus_{"XYZ"[j]}_1g.vtu'
        m=meshio.read(path);dist,ix=cKDTree(m.points[:,:3]).query(xyz)
        assert max(dist)<1e-8
        disp=m.point_data['displacement_mm'][ix]
        u=np.zeros(fe.basis.N)
        for a in range(3):u[fe.components[a]]=disp[:,a]
        accel=np.eye(3)[j]*G0
        U.append(u);F.append(fe.body_force(accel)+G[:,:3]@(MASS_CAMERA*accel))
        provenance.append({'axis':'XYZ'[j],'source_vtu':str(path),
            'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'max_coordinate_mapping_distance_mm':float(max(dist)),
            'saved_displacement_dtype':str(disp.dtype)})
    U=np.column_stack(U);F=np.column_stack(F)
    Dcg=(G.T@U)[:3];_,sing,Vh=np.linalg.svd(Dcg);direction=Vh[0]
    uw=U@direction;fw=F@direction
    base_fields={'U_1g':U,'F_1g':F,'G':G,'fixed':fe.fixed,'free':fe.free,
        'U_worst_CoG_1g':uw,'F_worst_CoG_1g':fw,'direction_worst_CoG':direction,
        'component_x':fe.components[0],'component_y':fe.components[1],
        'component_z':fe.components[2],'doflocs':fe.basis.doflocs}
    np.savez_compressed(out/'static_fields.npz',**base_fields)
    write_json(out/'recovered_field_provenance.json',{'recovered_from_actual_saved_fields':provenance,
        'note':'Original solved VTU displacement retained; no displacement fabricated or inferred from printed scalar results. K is rebuilt to check residuals and reactions.'})
    print('THREE_AXIS_FIELDS_RESTORED',str(out/'static_fields.npz'),flush=True)
    fe.assemble_stiffness()
    results=[]
    for j in range(3):
        u=U[:,j];f=F[:,j];q=G.T@u
        solve=fe.evaluate_solution(f,u);summary=fe.summarize(u,regions())
        if solve['relative_free_residual']>1e-6:
            raise RuntimeError(('Recovered-field residual too high',j,solve))
        r={'axis':'XYZ'[j],'acceleration_F_mm_s2':(np.eye(3)[j]*G0).tolist(),
            'camera_rigid_motion_about_CoG':q.tolist(),
            'camera_CoG_displacement_mm':float(np.linalg.norm(q[:3])),
            'camera_rotation_rad':float(np.linalg.norm(q[3:])),
            'solver':solve,'summary':summary}
        r['total_resultant_g_scalings']=[scale_summary(r,s) for s in [1,3,10]]
        results.append(r)
    bc=json.loads((out/'mesh_and_BC.json').read_text())['boundary_conditions']
    report={'level':level,'mesh':fe.mesh_info,'boundary_conditions':bc,'mapping_validation':mapping,
        'camera_CoG_F_mm':COG_F.tolist(),'camera_mass_kg':.18,
        'material_assumptions':{'E_MPa':E,'nu':NU,'density_tonne_mm3':RHO},
        'positive_axis_1g_solutions':results,
        'negative_axis_cases':'Exact sign reversal in the same linear model; von Mises and energy unchanged. This does not model different contact opening/slip under reversed forces.',
        'worst_CoG_unit_resultant_direction_F':direction.tolist(),
        'worst_CoG_displacement_mm_at_total_1g':float(sing[0]),
        'worst_CoG_direction_summary_at_1g':fe.summarize(uw,regions()),
        'worst_CoG_direction_energy_Nmm_at_1g':float(.5*uw@(fe.K@uw)),
        'unit_external_force_cases':[],
        'scaling_note':'1g/3g/10g are TOTAL resultant effective acceleration. Linear scalings are conditional, not physical load ratings.',
        'support_note':'All three translations constrained only at actual foot/platform overlap; engaged rigid-root idealization.',
        'recovery_provenance':'Actual saved double-precision VTU fields, revalidated against rebuilt original K and original load construction.',
        'static_inertial_complete':True,'unit_force_complete':False}
    write_json(out/'static_results.json',report)
    sp.save_npz(out/'stiffness_K.npz',fe.K)
    fe.export_vtu(out/'camera_worst_CoG_direction_1g.vtu',uw)
    print('STATIC_CHECKPOINT_READY',str(out/'static_results.json'),flush=True)
    if not unit_forces:return
    fe.assemble(support,solver='amg')
    cases=[];uf=[]
    for j in range(3):
        u,solve=fe.solve(G[:,j]);q=G.T@u;uf.append(u)
        r={'axis':'XYZ'[j],'force_at_camera_CoG_N':np.eye(3)[j].tolist(),
            'camera_rigid_motion_about_CoG':q.tolist(),
            'camera_CoG_displacement_mm_per_N':float(np.linalg.norm(q[:3])),
            'summary':fe.summarize(u,regions()),'solver':solve,
            'meaning':'Static 1N external force at camera CoG, not a transient collision'}
        cases.append(r)
        write_json(out/f'unit_force_{"XYZ"[j]}_1N.json',r)
        np.savez_compressed(out/f'unit_force_{"XYZ"[j]}_1N_fields.npz',U=u,F=G[:,j])
        print('UNIT_FORCE_READY','XYZ'[j],flush=True)
    report['unit_external_force_cases']=cases;report['unit_force_complete']=True
    write_json(out/'static_results.json',report)
    base_fields['U_collision_1N']=np.column_stack(uf)
    np.savez_compressed(out/'static_fields.npz',**base_fields)
    print('ALL_STATIC_RECOVERED',str(out),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('level',choices=['coarse','medium','fine']);p.add_argument('--unit-forces',action='store_true');a=p.parse_args()
    main(a.level,a.unit_forces)