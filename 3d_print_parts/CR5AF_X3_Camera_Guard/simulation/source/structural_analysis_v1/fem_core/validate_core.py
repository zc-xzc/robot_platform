from pathlib import Path
import numpy as np
import gmsh
from solid_fem import gmsh_mesh,SolidFE,write_json

out=Path(__file__).parent/'validation';out.mkdir(exist_ok=True)
gmsh.initialize();gmsh.option.setNumber('General.Terminal',0)
gmsh.model.add('analytical_cube');gmsh.model.occ.addBox(0,0,0,10,10,10);gmsh.model.occ.synchronize()
gmsh.write(str(out/'analytical_cube.brep'));gmsh.finalize()
meta=gmsh_mesh(out/'analytical_cube.brep',out/'analytical_cube.msh',3.5)
fe=SolidFE(out/'analytical_cube.msh')
fixed=[]
for c,ix in enumerate(fe.components):
    fixed.extend(ix[np.isclose(fe.basis.doflocs[c,ix],0,atol=1e-9)])
info=fe.assemble(fixed_dofs=np.array(fixed),solver='amg')
tip=fe.facets(lambda x:np.isclose(x[0],10))
f,load=fe.load_force_at(tip,(100,0,0),(10,5,5))
u,sol=fe.solve(f)
exact=np.zeros(fe.basis.N)
for c,ix in enumerate(fe.components):
    exact[ix]=fe.basis.doflocs[c,ix]*(1/2000 if c==0 else -.35/2000)
maxerr=float(np.max(abs(exact-u)))
metrics=fe.summarize(u)
probe,probeinfo=fe.load_wrench(tip,(1,2,3),(4,5,6))
report=dict(test='10mm cube, exact 1MPa uniaxial tension, sliding symmetry supports',mesh=meta,
            mesh_info=fe.mesh_info,solver=info,solution=sol,metrics=metrics,
            exact_max_axial_displacement_mm=.005,exact_strain_energy_Nmm=.25,
            max_absolute_displacement_error_mm=maxerr,
            relative_strain_energy_error=abs(sol['strain_energy_Nmm']/.25-1),
            general_wrench_probe=probeinfo)
assert maxerr<1e-8,report
assert abs(sol['strain_energy_Nmm']/.25-1)<1e-7,report
assert abs(metrics['volume_weighted_p95_von_mises_MPa']-1)<1e-6,report
assert np.linalg.norm(np.array(probeinfo['assembled_wrench']['force_N'])-[1,2,3])<1e-9
assert np.linalg.norm(np.array(probeinfo['assembled_wrench']['moment_Nmm'])-[4,5,6])<1e-8
write_json(out/'core_validation.json',report)
print('CORE VALIDATED',maxerr,sol['strain_energy_Nmm'],fe.mesh_info,flush=True)