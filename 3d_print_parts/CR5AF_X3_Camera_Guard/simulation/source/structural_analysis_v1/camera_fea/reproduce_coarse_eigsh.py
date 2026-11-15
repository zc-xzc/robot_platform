"""Reproduction entry for the actually executed coarse direct eigenmethod.

Reconstructed from the recorded executed implementation, not an archived
bit-identical original run_camera_fea.py.  Does not overwrite original results.
The medium/fine LOBPCG route was added later and is not attributed to coarse.
"""
import json,csv
from run_camera_fea import *

def main():
    src=HERE/'coarse';out=HERE/'coarse_eigsh_reproduction';out.mkdir(exist_ok=True)
    fe=SolidFE(HERE/'meshes/camera_coarse.msh',E,NU,RHO)
    fe.K=sp.load_npz(src/'stiffness_K.npz')
    data=np.load(src/'static_fields.npz');G=data['G'];force_per_acc=data['F_1g']/G0
    fe.assemble(fixed_dofs=data['fixed'],solver='amg')
    Mframe=fe.mass_matrix();Gs=sp.csr_matrix(G)
    M=(Mframe+Gs@sp.csr_matrix(M_CAMERA)@Gs.T).tocsr();M=((M+M.T)*.5).tocsr()
    Mf=M[fe.free][:,fe.free].tocsr()
    vals,V=eigsh(fe.A,k=12,M=Mf,sigma=0,which='LM',tol=2e-8)
    ix=np.argsort(vals);vals=vals[ix];V=V[:,ix]
    for j in range(12):V[:,j]/=np.sqrt(V[:,j]@(Mf@V[:,j]))
    phi=np.zeros((fe.basis.N,12));phi[fe.free]=V
    freq=np.sqrt(vals)/(2*np.pi)
    residual=[float(np.linalg.norm(fe.A@V[:,j]-vals[j]*(Mf@V[:,j]))/np.linalg.norm(fe.A@V[:,j])) for j in range(12)]
    Gamma=phi.T@force_per_acc;observation=G.T@phi
    np.savez_compressed(out/'modal_fields.npz',eigenvalues=vals,frequencies_Hz=freq,phi=phi,
        Gamma=Gamma,camera_observation=observation)
    reference_path=src/'modal_results.json'
    reference=json.loads(reference_path.read_text())['frequencies_Hz'] if reference_path.exists() else None
    write_json(out/'reproduction_check.json',{'method':'scipy eigsh, sigma=0, k=12, tol=2e-8; explicit sparse total consistent mass',
        'frequencies_Hz':freq.tolist(),'relative_eigen_residuals':residual,
        'relative_frequency_difference_from_completed_coarse':((freq-np.asarray(reference))/reference).tolist() if reference is not None else None,
        'provenance':'Reconstructed reproduction entry, not a bit-identical archived original script.'})
    B=rigid_modes(fe)
    checks=[float(np.linalg.norm(M@B[:,j]-force_per_acc[:,j])/np.linalg.norm(force_per_acc[:,j])) for j in range(3)]
    total_mass=fe.mesh_info['mass_kg']/1000+MASS_CAMERA
    write_json(out/'modal_results.json',{'mode_count':12,'frequencies_Hz':freq.tolist(),
        'eigen_residuals_relative':residual,'eigensolver':'scipy eigsh sigma=0, k=12, tol=2e-8',
        'camera_mass_matrix_tonne_mm_units':M_CAMERA.tolist(),
        'rigid_translation_inertia_load_consistency_relative':checks,
        'total_frame_camera_mass_kg':total_mass*1000,
        'participation_Gamma_XYZ':Gamma.tolist(),'effective_mass_tonne_XYZ':(Gamma**2).tolist(),
        'resolved_effective_mass_fraction_XYZ':((Gamma**2).sum(axis=0)/total_mass).tolist(),
        'camera_rigid_motion_modes':observation.tolist(),
        'mass_normalization':'phi.T M_total phi = 1, using tonne/mm/N/s',
        'conditional_support_state':'Engaged rigid root idealization, not whole-robot modes.',
        'provenance':'New reproduction output; original coarse files retained separately.'})
    harmonic_response(vals,freq,Gamma,observation,out)
    print(freq,flush=True)

if __name__=='__main__':main()