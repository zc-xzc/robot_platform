"""Actual curved tetra10/P2 camera-module solves and attached rigid-mass modes.

The RBE3-like connection is an energy-consistent six-component surface-wrench
map. It adds inertial mass/inertia, not stiffness or prescribed tray motion.
"""
from pathlib import Path
import sys,json,argparse,time,csv
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import eigsh,LinearOperator,splu,lobpcg
from scipy.spatial import cKDTree
from camera_setup import *
sys.path.insert(0,str(HERE.parent/'fem_core'))
from solid_fem import SolidFE,write_json,weighted_quantile

def rigid_modes(fe):
    xyz=fe.basis.doflocs;B=np.zeros((fe.basis.N,6))
    for j,idx in enumerate(fe.components):
        B[idx,j]=1.
        for k in range(3):
            e=np.zeros(3);e[k]=1.
            B[idx,3+k]=np.cross(e,(xyz[:,idx]-COG_F[:,None]).T)[:,j]
    return B

def wrench_mapping(fe,patch):
    columns=[];details=[]
    for k in range(6):
        f=np.zeros(3);m=np.zeros(3)
        if k<3:f[k]=1.
        else:m[k-3]=1.
        column,info=fe.load_wrench(patch,f,m,COG_F)
        columns.append(column);details.append(info)
    G=np.column_stack(columns);B=rigid_modes(fe);identity=G.T@B
    error=float(np.max(np.abs(identity-np.eye(6))))
    if error>1e-6:raise ValueError(('Rigid-motion/wrench duality failed',identity))
    return G,{'unit_wrenches':details,'G_transpose_times_rigid_modes':identity.tolist(),
              'rigid_motion_duality_max_error':error,
              'interpretation':'q=G.T u is the work-conjugate least-squares rigid motion of the loaded surface about the stated camera CoG.'}

def scale_summary(s,scale):
    out={'total_effective_acceleration_g':scale,
         'camera_CoG_displacement_mm':s['camera_CoG_displacement_mm']*scale,
         'max_displacement_mm':s['summary']['max_displacement_mm']*scale,
         'p95_von_mises_MPa':s['summary']['volume_weighted_p95_von_mises_MPa']*scale,
         'p99_von_mises_MPa':s['summary']['volume_weighted_p99_von_mises_MPa']*scale,
         'raw_peak_von_mises_MPa':s['summary']['max_von_mises_MPa']*scale,
         'strain_energy_Nmm':s['solver']['strain_energy_Nmm']*scale**2}
    return out

def run(level,modes=0,export_fields=False,sensitivity=False,supplemental=False):
    out=HERE/level;out.mkdir(exist_ok=True)
    mesh=HERE/'meshes'/f'camera_{level}.msh'
    t0=time.monotonic();fe=SolidFE(mesh,E,NU,RHO)
    print('mesh',level,fe.mesh_info,flush=True)
    support=fe.facets(support_pad_mask)
    top=fe.facets(camera_top_contact_mask);head=fe.facets(screw_head_contact_mask)
    patch=np.unique(np.r_[top,head])
    bc={'root_support':fe.boundary_info(support),'camera_top_contact':fe.boundary_info(top),
        'nominal_head_underside_contact':fe.boundary_info(head),'combined_camera_load_patch':fe.boundary_info(patch)}
    G,mapping=wrench_mapping(fe,patch)
    assembly=fe.assemble(support,solver='amg')
    write_json(out/'mesh_and_BC.json',{'mesh':fe.mesh_info,'boundary_conditions':bc,
                                    'mapping':mapping,'assembly':assembly})
    print('assembled',assembly,flush=True)
    results=[];us=[];fs=[]
    for axis in range(3):
        a=np.eye(3)[axis]*G0
        f=fe.body_force(a)+G[:,:3]@(MASS_CAMERA*a)
        u,solve=fe.solve(f)
        q=G.T@u;summary=fe.summarize(u,regions())
        record={'axis':'XYZ'[axis],'acceleration_F_mm_s2':a.tolist(),
            'camera_rigid_motion_about_CoG':[float(v) for v in q],
            'camera_CoG_displacement_mm':float(np.linalg.norm(q[:3])),
            'camera_rotation_rad':float(np.linalg.norm(q[3:])),
            'solver':solve,'summary':summary}
        record['total_resultant_g_scalings']=[scale_summary(record,s) for s in [1,3,10]]
        results.append(record);us.append(u);fs.append(f)
        if export_fields:fe.export_vtu(out/f'camera_acceleration_plus_{"XYZ"[axis]}_1g.vtu',u)
        print('static',level,'XYZ'[axis],'CoG_mm',record['camera_CoG_displacement_mm'],
              'p99_MPa',summary['volume_weighted_p99_von_mises_MPa'],flush=True)
    U=np.column_stack(us);F=np.column_stack(fs)
    # Exact worst camera-CoG displacement direction for unit resultant acceleration.
    Dcg=(G.T@U)[:3];_,sing,Vh=np.linalg.svd(Dcg);direction=Vh[0]
    u_worst=U@direction;f_worst=F@direction
    worst_summary=fe.summarize(u_worst,regions())
    worst_energy=float(.5*u_worst@(fe.K@u_worst))
    force_cases=[];ucollision=[]
    for axis in range(3):
        u,solve=fe.solve(G[:,axis]);q=G.T@u
        force_cases.append({'axis':'XYZ'[axis],'force_at_camera_CoG_N':np.eye(3)[axis].tolist(),
                            'summary':fe.summarize(u,regions()),'solver':solve,
                            'camera_CoG_displacement_mm_per_N':float(np.linalg.norm(q[:3])),
                            'meaning':'Static unit external force; not a transient collision simulation'})
        ucollision.append(u)
    static={'level':level,'mesh':fe.mesh_info,'boundary_conditions':bc,'mapping_validation':mapping,
       'camera_CoG_F_mm':COG_F.tolist(),'camera_mass_kg':.18,
       'material_assumptions':{'E_MPa':E,'nu':NU,'density_tonne_mm3':RHO},
       'positive_axis_1g_solutions':results,
       'negative_axis_cases':'By exact linearity, displacement/stress tensor sign reverses; strain energy and von Mises values are unchanged. Six signed axes are thereby covered, not six independent nonlinear contact solves.',
       'worst_CoG_unit_resultant_direction_F':direction.tolist(),
       'worst_CoG_displacement_mm_at_total_1g':float(sing[0]),
       'worst_CoG_direction_summary_at_1g':worst_summary,
       'worst_CoG_direction_energy_Nmm_at_1g':worst_energy,
       'unit_external_force_cases':force_cases,
       'scaling_note':'1g/3g/10g are TOTAL resultant effective accelerations. Displacement and stress scale with g, energy with g squared, within this linear model.',
       'support_note':'All three displacements constrained only at actual foot/platform overlap. Engaged rigid-root idealization; actual free play, opening and slip unresolved.',
       'solve_seconds_total':time.monotonic()-t0}
    write_json(out/'static_results.json',static)
    np.savez_compressed(out/'static_fields.npz',U_1g=U,F_1g=F,G=G,fixed=fe.fixed,free=fe.free,
                        U_collision_1N=np.column_stack(ucollision),U_worst_CoG_1g=u_worst,
                        F_worst_CoG_1g=f_worst,direction_worst_CoG=direction,
                        component_x=fe.components[0],component_y=fe.components[1],component_z=fe.components[2],
                        doflocs=fe.basis.doflocs)
    sp.save_npz(out/'stiffness_K.npz',fe.K)
    if export_fields:fe.export_vtu(out/'camera_worst_CoG_direction_1g.vtu',u_worst)
    print('STATIC_CHECKPOINT_READY',str(out/'static_fields.npz'),flush=True)
    if modes:
        modal_analysis(fe,G,F/G0,out,modes)
    if supplemental:
        supplemental_static(fe,top,head,out,export_fields)
    if sensitivity:
        root_support_sensitivity(fe,G,F,U,out,export_fields)
    return static

def supplemental_static(fe,top,head,out,export_fields):
    """Unit preload and guard-tip cases; no torque, fracture or impact inference."""
    n=R[:,2]
    fh,ih=fe.load_force_at(head,n,T)
    ft,it=fe.load_force_at(top,-n,T)
    f=fh+ft;u,solve=fe.solve(f)
    clamp={'applied_clamping_force_N':1.,'head_force_F_N':n.tolist(),
        'camera_bottom_force_F_N':(-n).tolist(),
        'each_force_reference_point_F_mm':T.tolist(),
        'head_nominal_diameter_mm':20.,'camera_bottom_area_mm2':fe.boundary_info(top)['area_mm2'],
        'head_patch_area_mm2':fe.boundary_info(head)['area_mm2'],
        'load_definition':'Equal/opposite equilibrated 1N resultants normal to C z0; no net external force or moment. Distributed top/bottom surface tractions, not pressure-only contact.',
        'relative_approach_work_conjugate_mm_per_N':float(f@u),
        'solver':solve,'summary':fe.summarize(u,regions()),
        'limits':'Unknown screw torque, friction factor, actual head/washer, physical thread engagement, contact pressure and preload are not inferred. Stress per N is a linear idealized sensitivity, not tightening guidance.'}
    write_json(out/'screw_clamp_unit_1N.json',clamp)
    np.savez_compressed(out/'screw_clamp_unit_1N_fields.npz',U=u,F=f)
    if export_fields:fe.export_vtu(out/'screw_clamp_unit_1N.vtu',u)
    def guard_cap(x):
        c=to_C(x)
        return (np.abs(c[2]-118)<1e-5)&(c[0]>23.6)&(c[1]>9.9)&(c[1]<17.1)
    cap=fe.facets(guard_cap)
    point=R@np.array([25.45,13.5,118.])+T
    f,loadinfo=fe.load_force_at(cap,np.array([1.,0.,0.]),point)
    u,solve=fe.solve(f)
    record={'external_force_F_N':[1.,0.,0.],'point_F_mm':point.tolist(),
        'point_C_mm':[25.45,13.5,118.],'cap_patch':fe.boundary_info(cap),
        'displacement_along_unit_force_mm_per_N':float(f@u),
        'solver':solve,'summary':fe.summarize(u,regions()),
        'meaning':'Static 1N applied inward at one rear guard-post cap. No camera/guard contact is modeled, so this is an uncontacted small-deformation stiffness sensitivity, not a real collision prediction or rating.'}
    write_json(out/'guard_tip_unit_1N.json',record)
    np.savez_compressed(out/'guard_tip_unit_1N_fields.npz',U=u,F=f)
    if export_fields:fe.export_vtu(out/'guard_tip_unit_1N.vtu',u)
    print('SUPPLEMENTAL_READY',str(out),flush=True)

def root_support_sensitivity(fe,G,F,Ubaseline,out,export_fields):
    """Actually solve a second engaged-root idealization on the same mesh/K.

    Foot faces within the two mushroom-head projections are captured instead
    of tying the broad actual carrier-pad overlap.  This is not unilateral
    contact and does not close the nominal clearances in a nonlinear solve.
    """
    support=fe.facets(captured_stud_mask)
    assembly=fe.assemble(support,solver='amg')
    records=[];us=[]
    for j in range(3):
        u,solve=fe.solve(F[:,j]);q=G.T@u;qb=G.T@Ubaseline[:,j]
        records.append({'axis':'XYZ'[j],'camera_rigid_motion_about_CoG':q.tolist(),
            'camera_CoG_displacement_mm':float(np.linalg.norm(q[:3])),
            'ratio_to_actual_platform_tied_baseline':float(np.linalg.norm(q[:3])/np.linalg.norm(qb[:3])),
            'summary':fe.summarize(u,regions()),'solver':solve})
        us.append(u)
        print('ROOT_SENSITIVITY',out.name,'XYZ'[j],records[-1]['camera_CoG_displacement_mm'],flush=True)
    out_s=out/'captured_stud_support';out_s.mkdir(exist_ok=True)
    write_json(out_s/'results.json',{'actual_run':True,'mesh':fe.mesh_info,
        'support_patch':fe.boundary_info(support),'assembly':assembly,
        'definition':'All translations restrained on existing foot faces inside two diameter7 mushroom-head projections at F z13 or16. No support added at the tray.',
        'interpretation':'Alternative idealized engaged root, not a validated contact model or a strict upper/lower bound. Nominal gaps/preload/slip remain unresolved.',
        'positive_axis_1g_solutions':records})
    np.savez_compressed(out_s/'fields.npz',U_1g=np.column_stack(us),fixed=fe.fixed,free=fe.free)
    if export_fields:fe.export_vtu(out_s/'camera_acceleration_plus_Z_1g.vtu',us[2])

def modal_analysis(fe,G,force_per_acc,out,nmodes):
    t0=time.monotonic();B=rigid_modes(fe)
    Mframe=fe.mass_matrix()
    # Do not expand the six-rank attachment into a potentially huge dense
    # surface block.  This operator is algebraically G M_camera G.T.
    def mass_all(v):return Mframe@v+G@(M_CAMERA@(G.T@v))
    applied_check=[]
    for j in range(3):
        residual=mass_all(B[:,j])-force_per_acc[:,j]
        applied_check.append(float(np.linalg.norm(residual)/np.linalg.norm(force_per_acc[:,j])))
    assert max(applied_check)<1e-7,applied_check
    Mff=Mframe[fe.free][:,fe.free].tocsr();Gf=G[fe.free]
    def mass_free(v):return Mff@v+Gf@(M_CAMERA@(Gf.T@v))
    Mf=LinearOperator(fe.A.shape,matvec=mass_free,matmat=mass_free,rmatvec=mass_free,dtype=float)
    print('modal mass operator',Mf.shape,'frame nnz',Mframe.nnz,'attachment rank 6; starting eigen',nmodes,flush=True)
    X0,seed_info=modal_seed(fe,out,nmodes)
    # Direct 3-D LU fill exceeded the shared memory allocation on the medium
    # model.  AMG-preconditioned block eigeniteration avoids that fill while
    # solving exactly the same K phi = lambda M phi problem.
    vals,phi_free,residual_history=lobpcg(fe.A,X0,B=Mf,M=fe.preconditioner,
        largest=False,tol=1e-7,maxiter=200,retResidualNormsHistory=True)
    order=np.argsort(vals);vals=vals[order];phi_free=phi_free[:,order]
    assert np.min(vals)>0
    for j in range(nmodes):phi_free[:,j]/=np.sqrt(phi_free[:,j]@(Mf@phi_free[:,j]))
    phi=np.zeros((fe.basis.N,nmodes));phi[fe.free]=phi_free
    freq=np.sqrt(vals)/(2*np.pi);Gamma=phi.T@force_per_acc
    observation=G.T@phi
    residuals=[float(np.linalg.norm(fe.A@phi_free[:,j]-vals[j]*(Mf@phi_free[:,j]))/
                     np.linalg.norm(fe.A@phi_free[:,j])) for j in range(nmodes)]
    # Never turn unconverged eigenvectors into a claimed FRF.  Preserve raw
    # iteration diagnostics even if the numerical quality gate is not met.
    write_json(out/'modal_iteration_diagnostics.json',{'solver':'AMG-preconditioned LOBPCG',
        'seed':seed_info,'absolute_residual_history':np.asarray(residual_history).tolist(),
        'relative_eigen_residuals':residuals})
    if max(residuals)>1e-5:
        np.savez_compressed(out/'modal_unconverged_checkpoint.npz',eigenvalues=vals,phi_free=phi_free)
        raise RuntimeError(('Modal residual quality gate not met; no FRF exported',residuals))
    total_mass=fe.mesh_info['mass_kg']/1000+MASS_CAMERA
    effective=Gamma**2
    report={'mode_count':nmodes,'frequencies_Hz':freq.tolist(),'eigen_residuals_relative':residuals,
        'mass_normalization':'phi.T M_total phi = 1; units use tonne, mm, N and second',
        'rigid_mass_mapping':'M_total = M_frame + G M_camera G.T, q_camera = G.T u. No stiffness added by camera map.',
        'mass_implementation':'Sparse frame consistent mass plus exact six-rank LinearOperator for attached rigid-camera inertia.',
        'eigensolver':'AMG-preconditioned LOBPCG; accepted only after explicit relative eigen-residual checks',
        'initial_subspace':seed_info,
        'camera_mass_matrix_tonne_mm_units':M_CAMERA.tolist(),
        'rigid_translation_inertia_load_consistency_relative':applied_check,
        'total_frame_camera_mass_kg':total_mass*1000,
        'participation_Gamma_XYZ':Gamma.tolist(),'effective_mass_tonne_XYZ':effective.tolist(),
        'resolved_effective_mass_fraction_XYZ':(effective.sum(axis=0)/total_mass).tolist(),
        'camera_rigid_motion_modes':observation.tolist(),
        'conditional_support_state':'Engaged, rigid root contact idealization. Actual snap-joint gaps/preload are not represented by these linear modes.',
        'modal_seconds':time.monotonic()-t0}
    write_json(out/'modal_results.json',report)
    np.savez_compressed(out/'modal_fields.npz',eigenvalues=vals,frequencies_Hz=freq,phi=phi,
                        Gamma=Gamma,camera_observation=observation)
    with (out/'modal_frequencies.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['mode','frequency_Hz','residual','effective_mass_X_kg','effective_mass_Y_kg','effective_mass_Z_kg'])
        for j in range(nmodes):w.writerow([j+1,freq[j],residuals[j],*(effective[j]*1000)])
    for j in range(min(3,nmodes)):
        u=phi[:,j];amp=np.linalg.norm(np.vstack([u[ix] for ix in fe.components]),axis=0).max()
        fe.export_vtu(out/f'mode_{j+1:02d}_shape_max1mm.vtu',u/amp)
    harmonic_response(vals,freq,Gamma,observation,out)
    print('MODES_READY',freq.tolist(),flush=True)

def modal_seed(fe,out,nmodes):
    """Interpolate already converged lower-resolution modes as an initial subspace."""
    candidates=['medium','coarse'] if out.name=='fine' else ['coarse']
    for level in candidates:
        modal=HERE/level/'modal_fields.npz';stat=HERE/level/'static_fields.npz'
        if not modal.exists() or not stat.exists():continue
        m=np.load(modal);s=np.load(stat);old=m['phi']
        if old.shape[1]<nmodes:continue
        xyz_old=s['doflocs'][:,s['component_x']].T
        xyz_new=fe.basis.doflocs[:,fe.components[0]].T
        distances,idx=cKDTree(xyz_old).query(xyz_new,k=8)
        weights=1./np.maximum(distances,1e-10)**2;weights/=weights.sum(axis=1,keepdims=True)
        X=np.zeros((fe.basis.N,nmodes))
        for axis in range(3):
            v=old[s['component_'+'xyz'[axis]],:nmodes]
            X[fe.components[axis]]=np.einsum('ij,ijk->ik',weights,v[idx])
        return X[fe.free],{'from_mesh':level,'method':'8-nearest coordinate inverse-distance-squared interpolation; final modes re-solved and mass normalized, not reused as results.'}
    rng=np.random.default_rng(38122)
    return rng.normal(size=(len(fe.free),nmodes)),{'method':'Deterministic random initial subspace'}

def harmonic_response(vals,freq,Gamma,Hobs,out):
    upper=.9*float(freq[-1]);grid=np.unique(np.r_[np.linspace(0,upper,1601),freq[freq<=upper]])
    omega=2*np.pi*grid;wn=np.sqrt(vals)
    rows=[];peaks=[]
    for damping in [.02,.05,.10]:
        den=vals[:,None]-omega[None,:]**2+2j*damping*wn[:,None]*omega[None,:]
        for axis in range(3):
            modal_amp=(.1*G0*Gamma[:,axis,None])/den
            q=Hobs@modal_amp;t=q[:3]
            a=(t.real*t.real).sum(axis=0);b=(t.imag*t.imag).sum(axis=0);c=(t.real*t.imag).sum(axis=0)
            amp=np.sqrt(.5*(a+b+np.sqrt((a-b)**2+4*c*c)))
            im=int(np.argmax(amp));peaks.append({'input_axis':'XYZ'[axis],'assumed_damping_ratio':damping,
                'frequency_Hz':float(grid[im]),'camera_CoG_peak_vector_displacement_mm':float(amp[im])})
            for k,f in enumerate(grid):rows.append([damping,'XYZ'[axis],f,amp[k],*np.abs(q[:3,k])])
    with (out/'harmonic_camera_CoG_0p1g.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['assumed_damping_ratio','excitation_axis','frequency_Hz','peak_vector_displacement_mm','X_component_amplitude_mm','Y_component_amplitude_mm','Z_component_amplitude_mm']);w.writerows(rows)
    write_json(out/'harmonic_summary.json',{'excitation_amplitude_g':.1,'damping_ratios_assumed':[.02,.05,.10],
        'frequency_span_Hz':[0,upper],'retained_modes':len(vals),'peaks':peaks,
        'interpretation':'Linear modal steady harmonic base/effective-acceleration sensitivity. Peak vector amplitude includes relative component phase. No measured excitation spectrum, damping, joint contact dynamics or transient impact is claimed.',
        'truncation':'Frequency range is limited below the last resolved mode; omitted high modes/residual flexibility remain a limitation.'})

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('level',choices=['coarse','medium','fine']);p.add_argument('--modes',type=int,default=0);p.add_argument('--export-fields',action='store_true');p.add_argument('--support-sensitivity',action='store_true');p.add_argument('--supplemental',action='store_true');a=p.parse_args()
    run(a.level,a.modes,a.export_fields,a.support_sensitivity,a.supplemental)