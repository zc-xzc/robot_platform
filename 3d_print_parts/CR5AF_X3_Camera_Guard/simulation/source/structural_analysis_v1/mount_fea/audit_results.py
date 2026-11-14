"""Numerical/file completeness audit, explicitly not a physical safety approval."""
from pathlib import Path
import json,hashlib
import numpy as np
HERE=Path(__file__).parent
def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
rows=[]
for part,levels in [('base',['coarse','medium','fine']),('cartridge',['coarse','medium','fine']),('cartridge_postbreak',['coarse','medium'])]:
    for level in levels:
        p=HERE/part/level;d=json.loads((p/'results.json').read_text())
        assert d.get('complete'),(part,level,'incomplete')
        source_ok=sha(d['mesh']['source'])==d['source_sha256']==d['mesh']['source_sha256']
        mesh_ok=sha(d['mesh']['mesh'])==d['mesh']['mesh_sha256']
        z=np.load(p/'unit_response_fields.npz');names=list(z['case_names'])
        assert set(names)==set(d['cases']),(part,level,'case array mismatch')
        finite=bool(np.isfinite(z['displacement']).all() and np.isfinite(z['stress6_cell_mean']).all())
        relres=max(c['solution']['relative_free_residual'] for c in d['cases'].values())
        eerr=max(abs(c['solution']['compliance_Nmm']-2*c['solution']['strain_energy_Nmm'])/
                  max(abs(c['solution']['compliance_Nmm']),1e-30) for c in d['cases'].values())
        fbalance=max(np.linalg.norm(c['solution']['force_balance_error_N']) for c in d['cases'].values())
        mbalance=max(np.linalg.norm(c['solution']['moment_balance_error_Nmm']) for c in d['cases'].values())
        vol=float(z['cell_volume'].sum());volerr=abs(vol/d['mesh_info']['volume_mm3']-1)
        row=dict(part=part,level=level,cases=len(names),source_hash_unchanged=source_ok,mesh_hash_unchanged=mesh_ok,
                 finite_field_arrays=finite,max_relative_free_residual=relres,max_relative_energy_identity_error=eerr,
                 max_force_balance_error_N=float(fbalance),max_moment_balance_error_Nmm=float(mbalance),
                 cell_volume_sum_relative_error=volerr,
                 mesh_volume_relative_error_from_CAD=d['volume_relative_error'],
                 curved_folded_elements=d['mesh_info']['folded_elements'])
        row['numerical_internal_checks_pass']=bool(source_ok and mesh_ok and finite and relres<1e-7 and eerr<1e-7 and volerr<1e-12 and d['mesh_info']['folded_elements']==0)
        assert row['numerical_internal_checks_pass'],row
        rows.append(row)
out=dict(scope='Numerical consistency and deliverable completeness only. Not material, fracture, contact, impact or physical safety validation.',
         physical_validation=False,results=rows)
(HERE/'numerical_audit.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2))