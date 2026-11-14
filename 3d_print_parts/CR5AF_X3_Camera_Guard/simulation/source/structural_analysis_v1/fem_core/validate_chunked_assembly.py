from pathlib import Path
import numpy as np
from skfem import asm,BilinearForm
from skfem.helpers import dot
from skfem.models.elasticity import linear_elasticity
from solid_fem import SolidFE,write_json
p=Path(__file__).parent/'validation'
fe=SolidFE(p/'analytical_cube.msh')
full=asm(linear_elasticity(fe.lam,fe.mu),fe.basis).tocsr()
chunk=fe.assemble_stiffness(chunk_size=150)
d=(full-chunk).tocsr()
error=float(np.linalg.norm(d.data)/np.linalg.norm(full.data))
report=dict(test='Same curved-P2 discretization, full versus 150-element-block stiffness assembly',
            elements=int(fe.mesh.nelements),dofs=int(fe.basis.N),relative_Frobenius_difference=error,
            max_absolute_coefficient_difference=float(np.max(abs(d.data))) if d.nnz else 0)
assert error<1e-13,report
@BilinearForm
def mass(u,v,w):return fe.density*dot(u,v)
mf=asm(mass,fe.basis).tocsr();mc=fe.mass_matrix(chunk_size=150)
md=mf-mc
report['mass_matrix_relative_Frobenius_difference']=float(np.linalg.norm(md.data)/np.linalg.norm(mf.data))
one=np.zeros(fe.basis.N);one[fe.components[0]]=1
report['rigid_translation_mass_tonne']=float(one@mc@one)
report['expected_rigid_translation_mass_tonne']=fe.density*1000
assert report['mass_matrix_relative_Frobenius_difference']<1e-13
assert abs(report['rigid_translation_mass_tonne']/(fe.density*1000)-1)<1e-12
write_json(p/'chunked_assembly_validation.json',report)
print(report,flush=True)