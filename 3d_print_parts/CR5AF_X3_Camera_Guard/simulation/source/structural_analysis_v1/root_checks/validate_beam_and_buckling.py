"""Independent bending, modal-unit and initial-stress buckling checks.

N, mm, MPa, tonne, s. This validates the numerical formulation on a simple
slender beam; it is not an experimental validation of the camera assembly.
"""
from pathlib import Path
import sys, json, time
import numpy as np
import cadquery as cq
from scipy.sparse.linalg import eigsh, splu, LinearOperator
from skfem import BilinearForm, asm

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'fem_core'))
from solid_fem import SolidFE, gmsh_mesh, write_json


def geometric_stiffness(fe, u):
    sigma,_=fe.stress(u)
    @BilinearForm
    def geo(du,dv,w):
        return np.einsum('ikeq,kleq,ileq->eq',du.grad,w.sigma,dv.grad)
    return asm(geo,fe.basis,sigma=sigma).tocsr()


def buckling_modes(fe, u_reference, k=4, tolerance=1e-6):
    """K phi = -lambda Kg phi; report only positive ideal eigenfactors."""
    kg=geometric_stiffness(fe,u_reference)
    kff=fe.K[fe.free][:,fe.free].tocsc()
    ag=(-kg[fe.free][:,fe.free]).tocsr()
    fac=splu(kff)
    inv=LinearOperator(kff.shape,matvec=fac.solve,dtype=float)
    mu,vec=eigsh(ag,k=k,M=kff,Minv=inv,which='LA',tol=tolerance)
    order=np.argsort(mu)[::-1];mu=mu[order];vec=vec[:,order]
    good=mu>1e-12
    factors=1/mu[good]
    modes=np.zeros((fe.basis.N,len(factors)))
    modes[fe.free]=vec[:,good]
    residual=[]
    for j,m in enumerate(mu[good]):
        v=vec[:,good][:,j]
        residual.append(float(np.linalg.norm(ag@v-m*(kff@v))/max(np.linalg.norm(ag@v),1e-30)))
    return factors,modes,residual


def main():
    out=Path(__file__).resolve().parent/'beam_validation';out.mkdir(exist_ok=True)
    L,b,h=100.,10.,10.;E=2000.;nu=.35;rho=1.24e-9
    shape=cq.Solid.makeBox(L,b,h)
    cad=out/'cantilever.brep';shape.exportBrep(str(cad))
    mesh=out/'cantilever.msh'
    info=gmsh_mesh(cad,mesh,hmax=5.,hmin=2.5,curvature_segments=12,nthreads=1)
    fe=SolidFE(mesh,E=E,nu=nu,density=rho)
    clamp=fe.facets(lambda x:abs(x[0])<1e-6)
    tip=fe.facets(lambda x:abs(x[0]-L)<1e-6)
    solver=fe.assemble(clamp,solver='direct')
    # Free-end transverse load with zero end-face moment about its centroid.
    fy,_=fe.load_wrench(tip,F=(0,0,-1),about=(L,b/2,h/2))
    uy,sol_y=fe.solve(fy)
    delta=float(fy@uy)
    A=b*h;I=b*h**3/12;G=E/(2*(1+nu));kappa=5/6
    EB=L**3/(3*E*I);Timo=EB+L/(kappa*G*A)
    # Independent mass-unit and first bending-mode check.
    M=fe.mass_matrix()[fe.free][:,fe.free]
    kff=fe.K[fe.free][:,fe.free]
    eig,v=eigsh(kff,k=4,M=M,sigma=0,which='LM',tol=1e-8)
    freq=np.sqrt(np.sort(eig))/(2*np.pi)
    beta=1.875104068711961
    f_EB=beta**2/(2*np.pi)*np.sqrt(E*I/(rho*A*L**4))
    # Unit axial compression supplies the initial stress for eigenbuckling.
    fx,_=fe.load_wrench(tip,F=(-1,0,0),about=(L,b/2,h/2))
    ux,sol_x=fe.solve(fx)
    factors,modes,residual=buckling_modes(fe,ux)
    Euler=np.pi**2*E*I/(4*L**2)
    Timo_buckle=Euler/(1+Euler/(kappa*G*A))
    result={'purpose':'independent formulation checks, not hardware validation',
      'units':{'force':'N','length':'mm','stress':'MPa','mass':'tonne','time':'s'},
      'geometry':{'length_mm':L,'width_mm':b,'height_mm':h},
      'mesh':info,'mesh_info':fe.mesh_info,'solver':solver,
      'static_bending':{'applied_force_N':1,'FE_tip_work_conjugate_displacement_mm':delta,
        'Euler_Bernoulli_mm':EB,'Timoshenko_mm':Timo,
        'relative_error_to_Timoshenko':delta/Timo-1,'equilibrium':sol_y},
      'modal_unit_check':{'FE_first_frequencies_Hz':freq.tolist(),
        'Euler_Bernoulli_first_bending_Hz':f_EB,
        'relative_error_first_mode_to_EB':float(freq[0]/f_EB-1)},
      'buckling':{'reference_compression_N':1.,'FE_positive_factors':factors.tolist(),
        'Euler_N':Euler,'Timoshenko_corrected_N':Timo_buckle,
        'relative_error_to_Timoshenko':float(factors[0]/Timo_buckle-1),
        'eigen_residuals':residual,'preload_equilibrium':sol_x},
      'acceptance':{'static_relative_error_limit':.04,'modal_relative_error_limit':.04,
        'buckling_relative_error_limit':.04}}
    result['passed']=(abs(delta/Timo-1)<.04 and abs(freq[0]/f_EB-1)<.04
                      and abs(factors[0]/Timo_buckle-1)<.04 and max(residual)<1e-4)
    write_json(out/'validation.json',result)
    fe.export_vtu(out/'bending_1N.vtu',uy)
    # Eigenvector scale is arbitrary; normalize to 1 mm only for display.
    mode=modes[:,0];mode=mode/max(np.linalg.norm(mode.reshape(-1,3),axis=1))
    fe.export_vtu(out/'buckling_mode_1_unit_display.vtu',mode)
    print(json.dumps({k:v for k,v in result.items() if k in ['passed','static_bending','modal_unit_check','buckling']},indent=2),flush=True)
    if not result['passed']:raise RuntimeError('Independent beam benchmark did not meet the stated gate')


if __name__=='__main__':main()