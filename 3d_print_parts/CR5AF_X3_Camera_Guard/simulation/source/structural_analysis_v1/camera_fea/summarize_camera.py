"""Summarize only actual completed camera solves; no synthetic FE results."""
from pathlib import Path
import json,csv,hashlib
import numpy as np
from camera_setup import HERE,SOURCE,COG_F,E,NU,RHO,G0

def load(p):return json.loads(p.read_text())
def write(p,d):p.write_text(json.dumps(d,indent=2,ensure_ascii=False),encoding='utf-8')

def main():
    levels=[x for x in ['coarse','medium','fine'] if (HERE/x/'static_results.json').exists()]
    static={x:load(HERE/x/'static_results.json') for x in levels}
    rows=[];regional=[];modal_rows=[]
    for level,d in static.items():
        for r in d['positive_axis_1g_solutions']:
            row={'mesh_level':level,'tetrahedra':d['mesh']['tetrahedra'],'dofs':d['mesh']['displacement_dofs'],
                'input_axis':r['axis'],'total_acceleration_g':1.,'camera_CoG_displacement_mm':r['camera_CoG_displacement_mm'],
                'max_node_displacement_mm':r['summary']['max_displacement_mm'],
                'strain_energy_Nmm':r['solver']['strain_energy_Nmm'],
                'volume_p95_von_mises_MPa':r['summary']['volume_weighted_p95_von_mises_MPa'],
                'volume_p99_von_mises_MPa':r['summary']['volume_weighted_p99_von_mises_MPa'],
                'raw_von_mises_peak_MPa':r['summary']['max_von_mises_MPa']}
            rows.append(row)
            for region,s in r['summary']['regions'].items():
                regional.append({'mesh_level':level,'case':'1g_'+r['axis'],'region':region,**s})
        for r in d['unit_external_force_cases']:
            for region,s in r['summary']['regions'].items():
                regional.append({'mesh_level':level,'case':'1N_at_CoG_'+r['axis'],'region':region,**s})
        mp=HERE/level/'modal_results.json'
        if mp.exists():
            m=load(mp)
            for i,freq in enumerate(m['frequencies_Hz']):
                modal_rows.append({'mesh_level':level,'mode_order':i+1,'frequency_Hz':freq,
                    'eigen_residual_relative':m['eigen_residuals_relative'][i]})
    for name,rr in [('convergence_static.csv',rows),('regional_stress.csv',regional),('convergence_modes.csv',modal_rows)]:
        if rr:
            keys=list(dict.fromkeys(k for r in rr for k in r))
            with (HERE/name).open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rr)
    adjacent=[]
    for lo,hi in zip(levels[:-1],levels[1:]):
        for ax in 'XYZ':
            a=next(r for r in rows if r['mesh_level']==lo and r['input_axis']==ax)
            b=next(r for r in rows if r['mesh_level']==hi and r['input_axis']==ax)
            metrics=['camera_CoG_displacement_mm','strain_energy_Nmm','volume_p95_von_mises_MPa','volume_p99_von_mises_MPa','raw_von_mises_peak_MPa']
            adjacent.append({'coarser':lo,'finer':hi,'input_axis':ax,
                'change_percent_relative_to_finer':{k:100*(b[k]-a[k])/b[k] for k in metrics}})
    report={'available_completed_levels':levels,'source_CAD_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        'adjacent_convergence':adjacent,
        'mesh_volume_mm3':{x:static[x]['mesh']['volume_mm3'] for x in levels},
        'camera_CoG_F_mm':COG_F.tolist(),
        'material_assumptions':{'E_MPa':E,'nu':NU,'density_tonne_mm3':RHO},
        'camera_CoG_displacement_unit_resultant_max_mm':{x:static[x]['worst_CoG_displacement_mm_at_total_1g'] for x in levels},
        'note':'Raw sharp-edge/clamp stress peaks are tracked independently, not substituted by percentiles. Mesh refinement includes small facet-centroid boundary-patch changes. Frequency columns are sorted modes, not guaranteed one-to-one mode-shape tracking.'}
    latest=levels[-1];d=static[latest]
    report['reported_mesh']=latest
    report['total_1g_results']=d['positive_axis_1g_solutions']
    report['total_3g_and_10g_scalings']={r['axis']:r['total_resultant_g_scalings'] for r in d['positive_axis_1g_solutions']}
    report['worst_CoG_direction_at_total_1g']=d['worst_CoG_unit_resultant_direction_F']
    report['q_camera_6x3_per_global_1g']=np.asarray([r['camera_rigid_motion_about_CoG'] for r in d['positive_axis_1g_solutions']]).T.tolist()
    report['q_units']='Rows 0..2: mm; rows 3..5: rad; reference at camera CoG; columns global F X,Y,Z 1g including frame self-weight.'
    fields=np.load(HERE/latest/'static_fields.npz');G=fields['G']
    if 'U_collision_1N' in fields:
        Uc=fields['U_collision_1N']
        report['q_camera_6x3_per_global_1N_at_CoG']=(G.T@Uc).tolist()
    for level in levels:
        mp=HERE/level/'modal_results.json'
        if mp.exists():
            m=load(mp);n=np.load(HERE/level/'modal_fields.npz')
            q0=n['camera_observation']@((G0*n['Gamma'])/n['eigenvalues'][:,None])
            exact=np.asarray([r['camera_rigid_motion_about_CoG'] for r in static[level]['positive_axis_1g_solutions']]).T
            m['zero_frequency_CoG_vector_relative_error_XYZ']=(np.linalg.norm(q0[:3]-exact[:3],axis=0)/np.linalg.norm(exact[:3],axis=0)).tolist()
            report.setdefault('modal_by_mesh',{})[level]=m
        cp=HERE/level/'captured_stud_support/results.json'
        if cp.exists():report.setdefault('support_sensitivity_by_mesh',{})[level]=load(cp)
    for name in ['screw_clamp_unit_1N','guard_tip_unit_1N']:
        p=HERE/latest/(name+'.json')
        if p.exists():report[name]=load(p)
    modal_levels=[level for level in levels if (HERE/level/'modal_results.json').exists()]
    if modal_levels:
        report['reported_modal_mesh']=modal_levels[-1]
        hp=HERE/modal_levels[-1]/'harmonic_summary.json'
        if hp.exists():report['harmonic_sensitivity']=load(hp)
    write(HERE/'camera_FEA_summary.json',report)
    print(json.dumps({'levels':levels,'adjacent_convergence':adjacent,
        'worst_CoG_mm':report['camera_CoG_displacement_unit_resultant_max_mm'],
        'modal_zero_frequency_error':{l:m['zero_frequency_CoG_vector_relative_error_XYZ'] for l,m in report.get('modal_by_mesh',{}).items()}},indent=2))

if __name__=='__main__':main()