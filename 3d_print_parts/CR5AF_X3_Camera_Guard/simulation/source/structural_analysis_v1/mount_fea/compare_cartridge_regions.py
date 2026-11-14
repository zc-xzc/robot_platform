"""Re-evaluate strict neck/stud regions from solved displacement, without re-solving."""
from pathlib import Path
import sys,json,csv,gc
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'fem_core'))
from solid_fem import SolidFE,weighted_quantile,write_json
HERE=Path(__file__).parent
rows=[];summary={}
for level in ['coarse','medium','fine']:
    out=HERE/'cartridge'/level
    if not (out/'unit_response_fields.npz').exists():continue
    report=json.loads((out/'results.json').read_text())
    if not report.get('complete'):continue
    fe=SolidFE(out/'02_Fuse_Cartridge_D8.msh')
    data=np.load(out/'unit_response_fields.npz');U=data['displacement'];names=list(data['case_names'])
    x=np.asarray(fe.basis.global_coordinates());weights=fe.basis.dx
    masks=dict(minimum_D8_neck=(x[2]>=8.1)&(x[2]<=9.1)&(x[0]**2+(x[1]-47)**2<=4.01**2),
               studs=(x[2]>=13)&(x[2]<=17.9)&(np.minimum((x[0]-8)**2+(x[1]-50)**2,(x[0]+8)**2+(x[1]-50)**2)<=3.500001**2))
    common={}
    for i,case in enumerate(names):
        stress,vm=fe.stress(U[:,i]);regions={}
        for name,mask in masks.items():
            q95,q99=weighted_quantile(vm[mask],weights[mask])
            regions[name]=dict(volume_mm3=float(weights[mask].sum()),max_von_mises_MPa=float(vm[mask].max()),p95_MPa=q95,p99_MPa=q99)
        report['cases'][case]['response']['regions'].update(regions)
        if case in [f'cameraCoG_F{a}_1N' for a in 'XYZ']:
            common[case]=regions
            for region,r in regions.items():
                rows.append(dict(level=level,case=case,region=region,p99_MPa=r['p99_MPa'],raw_peak_MPa=r['max_von_mises_MPa'],region_volume_mm3=r['volume_mm3']))
        del stress,vm
    report['region_definitions']=dict(minimum_D8_neck='8.1<=Fz<=9.1, radius<=4.01 around Fxy=(0,47)',
        studs='13<=Fz<=17.9, union of radius3.500001 circles at Fxy=(±8,50); excludes the short latch')
    report['load_path_state']='PRE_BREAK_intact'
    write_json(out/'results.json',report)
    summary[level]=dict(tetrahedra=report['mesh_info']['tetrahedra'],dofs=report['mesh_info']['displacement_dofs'],cases=common)
    print('REGIONS_RECHECKED',level,flush=True)
    del fe,data,U,x,weights,masks;gc.collect()
write_json(HERE/'cartridge_common_cog_region_comparison.json',dict(
    load_point_F_mm=[0,112.3050865,25.3050865],force_magnitude_N=1,
    scope='Camera-only point force; no selfweight in these unit states. Same reference point permits comparison with camera module root/arms. No failure-order prediction.',
    levels=summary))
with (HERE/'cartridge_common_cog_region_comparison.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.DictWriter(f,fieldnames=['level','case','region','p99_MPa','raw_peak_MPa','region_volume_mm3']);w.writeheader();w.writerows(rows)