from pathlib import Path
import json,csv
import numpy as np

ROOT=Path(__file__).resolve().parent
summary={};table=[]
for part in ['base','cartridge','cartridge_postbreak']:
    data={}
    for lev in ['coarse','medium','fine']:
        path=ROOT/part/lev/'results.json'
        if path.exists():
            d=json.loads(path.read_text())
            if d.get('complete'):data[lev]=d
    if not data:continue
    levels=list(data);first=data[levels[0]]
    cases=set.intersection(*(set(d['cases']) for d in data.values()))
    item=dict(levels=levels,source_sha256=first['source_sha256'],load_path_state=first.get('load_path_state'),
              mesh_levels={},cases={},limitations=[
                  'Three nonuniform mesh levels provide an observed change, not a rigorous discretization-error bound.',
                  'Ideal clamp/contact and load-spreading regions can cause local singularities. Peak-stress convergence is not assumed.',
                  'Force/displacement linear scaling is valid only while small strain, closed assumed contacts and the assigned linear material remain applicable.'])
    for lev,d in data.items():
        item['mesh_levels'][lev]=dict(tetrahedra=d['mesh_info']['tetrahedra'],dofs=d['mesh_info']['displacement_dofs'],
            volume_mm3=d['mesh_info']['volume_mm3'],volume_relative_error=d['volume_relative_error'],
            support_area_mm2=d['support']['area_mm2'],support_dofs=d['support']['constrained_dofs'])
    for case in sorted(cases):
        row={}
        for lev,d in data.items():
            c=d['cases'][case];r=c['response'];s=c['solution']
            row[lev]=dict(compliance_Nmm=s['compliance_Nmm'],strain_energy_Nmm=s['strain_energy_Nmm'],
                max_displacement_mm=r['max_displacement_mm'],p95_von_mises_MPa=r['volume_weighted_p95_von_mises_MPa'],
                p99_von_mises_MPa=r['volume_weighted_p99_von_mises_MPa'],raw_peak_von_mises_MPa=r['max_von_mises_MPa'],
                relative_free_residual=s['relative_free_residual'],regions=r.get('regions',{}))
            table.append(dict(part=part,case=case,level=lev,tetrahedra=d['mesh_info']['tetrahedra'],dofs=d['mesh_info']['displacement_dofs'],
                              **{k:v for k,v in row[lev].items() if k!='regions'}))
        if len(levels)>1:
            prev,last=levels[-2:]
            row['last_step_relative_change']={k:float(row[last][k]/row[prev][k]-1)
                for k in ['compliance_Nmm','strain_energy_Nmm','max_displacement_mm','p95_von_mises_MPa','p99_von_mises_MPa','raw_peak_von_mises_MPa']}
        item['cases'][case]=row
    summary[part]=item
(ROOT/'convergence_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
if table:
    with (ROOT/'convergence_table.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(table[0]));w.writeheader();w.writerows(table)
print(json.dumps({p:dict(levels=d['levels'],meshes=d['mesh_levels']) for p,d in summary.items()},indent=2))