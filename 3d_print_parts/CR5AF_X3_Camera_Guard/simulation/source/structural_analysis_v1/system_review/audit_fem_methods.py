"""Independent read-only arithmetic/provenance review of available FEM reports.
Does not assemble stiffness, remesh, solve, or modify any other agent's files.
"""
from pathlib import Path
import json,hashlib,datetime,math
import numpy as np

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1];ANA=ROOT/'structural_analysis_v1'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
frozen=json.loads((ANA/'inputs/frozen_inputs.json').read_text())
input_checks=[]
for x in frozen['critical_inputs']:
 a=ROOT/x['source_path'];b=ANA/x['analysis_copy']
 input_checks.append({'source':str(a),'analysis_copy':str(b),'expected_sha256':x['sha256'],
  'original_matches':sha(a)==x['sha256'],'copy_matches':sha(b)==x['sha256']})

files=[ANA/'fem_core/solid_fem.py',ANA/'mount_fea/run_mount.py',
       ANA/'camera_fea/camera_setup.py',ANA/'camera_fea/generate_mesh.py',
       ANA/'camera_fea/run_camera_fea.py',
       ANA/'camera_fea/recover_static_vtu.py',
       ANA/'camera_fea/reproduce_coarse_eigsh.py',
       ANA/'root_checks/run_pin_fea.py',
       ANA/'root_checks/validate_beam_and_buckling.py']
cases=[];reports=[]
for p in sorted((ANA/'mount_fea').glob('*/*/results.json')):
 j=json.loads(p.read_text());reports.append({'file':str(p),'sha256_at_review':sha(p),
  'complete_at_review':j.get('complete',False),'case_count_at_review':len(j.get('cases',{})),
  'source_sha256':j['source_sha256'],'mesh_sha256':j['mesh']['mesh_sha256'],
  'mesh_volume_relative_error':j.get('volume_relative_error'),
  'folded_elements_at_integration_points':j['mesh_info']['folded_elements'],
  'load_path_state':j.get('load_path_state'),
  'support_idealization':j['assumptions']['fixed_support']})
 for name,c in j.get('cases',{}).items():
  load=c['load'];sol=c['solution'];row={'report':str(p.relative_to(ANA)),'case':name,
    'relative_free_residual':sol['relative_free_residual'],
    'force_equilibrium_error_norm_N':float(np.linalg.norm(sol['force_balance_error_N'])),
    'moment_equilibrium_error_norm_Nmm':float(np.linalg.norm(sol['moment_balance_error_Nmm'])),
    'work_minus_twice_strain_energy_Nmm':float(sol['compliance_Nmm']-2*sol['strain_energy_Nmm'])}
  if 'target_force_N' in load:
   F=np.asarray(load['target_force_N']);M=np.asarray(load['target_moment_Nmm']);r=np.asarray(load['moment_reference_mm'])
   row['requested_to_assembled_force_error_N']=float(np.linalg.norm(F-np.asarray(sol['applied_wrench']['force_N'])))
   row['requested_to_assembled_global_moment_error_Nmm']=float(np.linalg.norm(M+np.cross(r,F)-np.asarray(sol['applied_wrench']['moment_Nmm'])))
  elif load.get('type')=='uniform body acceleration':
   expected=j['mesh_info']['mass_kg']/1000*np.asarray(load['acceleration_mm_s2'])
   row['body_force_units_error_N']=float(np.linalg.norm(expected-np.asarray(sol['applied_wrench']['force_N'])))
  cases.append(row)

cube=json.loads((ANA/'fem_core/validation/core_validation.json').read_text())
beam=json.loads((ANA/'root_checks/beam_validation/validation.json').read_text())
# Formula arithmetic recomputed independently; no matrix solve.
L=100.;b=h=10.;E=2000.;nu=.35;rho=1.24e-9;A=b*h;I=b*h**3/12
eb=L**3/(3*E*I);timo=eb+L/((5/6)*(E/(2*(1+nu)))*A)
freq=1.875104068711961**2/(2*math.pi)*math.sqrt(E*I/(rho*A*L**4))
euler=math.pi**2*E*I/(4*L**2)
checks={'cube_expected_uniaxial_u_mm':1/2000*10,
 'cube_reported_max_absolute_component_error_mm':cube['max_absolute_displacement_error_mm'],
 'cube_reported_energy_relative_error':cube['relative_strain_energy_error'],
 'beam_recomputed_EB_displacement_mm':eb,'beam_recomputed_Timoshenko_displacement_mm':timo,
 'beam_reported_FE_work_conjugate_displacement_mm':beam['static_bending']['FE_tip_work_conjugate_displacement_mm'],
 'beam_recomputed_first_EB_frequency_Hz':freq,
 'beam_reported_FE_first_frequency_Hz':beam['modal_unit_check']['FE_first_frequencies_Hz'][0],
 'beam_recomputed_Euler_buckling_N':euler,'beam_reported_FE_first_buckling_factor':beam['buckling']['FE_positive_factors'][0],
 'beam_validation_is_numerical_benchmark_not_hardware_test':True}
camera=[]
for p in sorted((ANA/'camera_fea').glob('*/static_results.json')):
 j=json.loads(p.read_text()); row={'static_report':str(p),'static_sha256':sha(p),
  'camera_CoG_F_mm':j['camera_CoG_F_mm'],'camera_mass_kg':j['camera_mass_kg'],
  'duality_error':j['mapping_validation']['rigid_motion_duality_max_error'],
  'source_material_assumptions':j['material_assumptions'],
  'coarse_or_fine_level':j['level'],
  'unit_external_force_cases_count':len(j.get('unit_external_force_cases',[])),
  'static_inertial_complete':j.get('static_inertial_complete',len(j.get('positive_axis_1g_solutions',[]))==3),
  'unit_force_complete':j.get('unit_force_complete',len(j.get('unit_external_force_cases',[]))==3),
  'max_CoG_displacement_mm_total_1g':j['worst_CoG_displacement_mm_at_total_1g'],
  'static_cases':[]}
 for c in j['positive_axis_1g_solutions']:
  sol=c['solver'];a=np.asarray(c['acceleration_F_mm_s2'])
  expected=(j['mesh']['mass_kg']+j['camera_mass_kg'])/1000*a
  scaling=[]
  for v in c.get('total_resultant_g_scalings',[]):
   ratio=v['total_effective_acceleration_g']
   scaling.append({'g':ratio,
    'displacement_scaling_error_mm':v['camera_CoG_displacement_mm']-ratio*c['camera_CoG_displacement_mm'],
    'energy_scaling_error_Nmm':v['strain_energy_Nmm']-ratio**2*sol['strain_energy_Nmm'],
    'p99_scaling_error_MPa':v['p99_von_mises_MPa']-ratio*c['summary']['volume_weighted_p99_von_mises_MPa']})
  row['static_cases'].append({'axis':c['axis'],
   'body_plus_camera_expected_force_N':expected.tolist(),
   'mass_acceleration_to_applied_force_error_N':float(np.linalg.norm(expected-np.asarray(sol['applied_wrench']['force_N']))),
   'force_balance_error_N':float(np.linalg.norm(sol['force_balance_error_N'])),
   'moment_balance_error_Nmm':float(np.linalg.norm(sol['moment_balance_error_Nmm'])),
   'relative_free_residual':sol['relative_free_residual'],
   'compliance_minus_2U_Nmm':sol['compliance_Nmm']-2*sol['strain_energy_Nmm'],
   'linear_scaling_checks':scaling})
 field=p.with_name('static_fields.npz')
 if field.exists():
  with np.load(field) as data:
   actual=data['G'].T@data['U_1g']
   expected=np.column_stack([v['camera_rigid_motion_about_CoG'] for v in j['positive_axis_1g_solutions']])
   row['saved_field_GtU_vs_report_max_abs_error']=float(np.max(np.abs(actual-expected)))
  row['saved_fields_sha256']=sha(field)
 support=p.parent/'captured_stud_support/results.json'
 if support.exists():
  sj=json.loads(support.read_text())
  row['alternative_root_support']={'file':str(support),'sha256':sha(support),'actual_run':sj.get('actual_run',False),
   'support_definition':sj['definition'],'support_patch':sj['support_patch'],
   'cases':[{'axis':v['axis'],'CoG_displacement_mm':v['camera_CoG_displacement_mm'],
    'baseline_ratio':v['ratio_to_actual_platform_tied_baseline'],
    'relative_residual':v['solver']['relative_free_residual'],
    'force_balance_N':float(np.linalg.norm(v['solver']['force_balance_error_N'])),
    'moment_balance_Nmm':float(np.linalg.norm(v['solver']['moment_balance_error_Nmm']))}
    for v in sj['positive_axis_1g_solutions']]}
 mp=p.with_name('modal_results.json')
 if mp.exists():
  md=json.loads(mp.read_text()); freq=np.asarray(md['frequencies_Hz'])
  obs=np.asarray(md['camera_rigid_motion_modes']);gam=np.asarray(md['participation_Gamma_XYZ'])
  q0=obs@(gam/(2*np.pi*freq[:,None])**2)*9806.65
  stat_err=[]
  for axis,c in enumerate(j['positive_axis_1g_solutions']):
   direct=np.asarray(c['camera_rigid_motion_about_CoG'])
   stat_err.append({'axis':c['axis'],'direct_translation_mm':direct[:3].tolist(),
    'truncated_modal_zero_frequency_translation_mm':q0[:3,axis].tolist(),
    'relative_translation_vector_difference':float(np.linalg.norm(q0[:3,axis]-direct[:3])/np.linalg.norm(direct[:3]))})
  row['modal']={'file':str(mp),'sha256':sha(mp),'mode_count':md['mode_count'],
   'first_frequency_Hz':freq[0],'last_frequency_Hz':freq[-1],
   'max_eigen_residual':max(md['eigen_residuals_relative']),
   'max_mass_load_consistency_error':max(md['rigid_translation_inertia_load_consistency_relative']),
   'effective_mass_fraction_XYZ':md['resolved_effective_mass_fraction_XYZ'],
   'modal_vs_direct_zero_frequency_checks':stat_err,
   'scope':'Zero-frequency arithmetic consistency only; does not establish FRF modal or mesh convergence.'}
 camera.append(row)
pins=[]
for p in sorted((ANA/'pin_fea').glob('*/results.json')):
 j=json.loads(p.read_text());pin={'level':j['mesh_level'],'file':str(p),'sha256':sha(p),
  'complete':j.get('complete',False),'source_sha256':j['mesh']['source_sha256'],'cases':[]}
 for name,c in j['cases'].items():
  sol=c['solution'];load=c['load'];F=np.asarray(load['target_force_N']);r=np.asarray(load['moment_reference_mm'])
  M=np.asarray(load['target_moment_Nmm'])+np.cross(r,F)
  case={'name':name,'BC':c['BC'],'support':c['support'],
   'max_displacement_mm':c['response']['max_displacement_mm'],
   'p99_MPa':c['response']['volume_weighted_p99_von_mises_MPa'],
   'raw_peak_MPa':c['response']['max_von_mises_MPa'],
   'requested_global_force_error_N':float(np.linalg.norm(F-np.asarray(sol['applied_wrench']['force_N']))),
   'requested_global_moment_error_Nmm':float(np.linalg.norm(M-np.asarray(sol['applied_wrench']['moment_Nmm']))),
   'force_balance_N':float(np.linalg.norm(sol['force_balance_error_N'])),
   'moment_balance_Nmm':float(np.linalg.norm(sol['moment_balance_error_Nmm'])),
   'relative_residual':sol['relative_free_residual']}
  if name=='barb_retention_plusZ_1N':
   strip=lambda radius:2*(.4*math.sqrt(radius**2-.4**2)+radius**2*math.asin(.4/radius))
   expected_area=math.pi*(2.35**2-2.2**2)-(strip(2.35)-strip(2.2))
   case.update({'independent_analytic_band_area_mm2':expected_area,'quadrature_area_mm2':load['area_mm2'],
    'relative_quadrature_area_error':load['area_mm2']/expected_area-1,
    'selection':load.get('load_selection'),'quadrature_order':load.get('facet_quadrature_order')})
  pin['cases'].append(case)
 pins.append(pin)
pin_last_changes={}
by_level={v['level']:v for v in pins}
if 'medium' in by_level and 'fine' in by_level:
 for f in by_level['fine']['cases']:
  m=next(v for v in by_level['medium']['cases'] if v['name']==f['name'])
  pin_last_changes[f['name']]={k+'_relative_change':f[k]/m[k]-1 for k in ['max_displacement_mm','p99_MPa','raw_peak_MPa']}
contact_path=OUT/'contact_compatibility/medium/contact_compatibility.json'
contact=None
if contact_path.exists():
 c=json.loads(contact_path.read_text());contact={'file':str(contact_path),'sha256':sha(contact_path),
  'level':c['level'],'boundary_nodes':c['mesh']['boundary_node_count'],'original_IN':c['original']['IN_nodes'],
  'original_ON':c['original']['ON_nodes'],'case_count':len(c['cases']),
  'any_new_guard_node_intrusion':c['any_new_guard_node_intrusion'],
  'all_new_intrusion_nodes_originally_on_Cz0_plane':all(v['new_IN_nodes']==v['new_IN_by_original_region']['original_bottom_plane_absCz_le1e_5'] for v in c['cases']),
  'bottom_nonpenetration_constraint_satisfied':not any(v['new_IN_nodes']>0 for v in c['cases']),
  'minimum_guard_nodal_AABB_distance_lower_bound_mm':min(v['guard_clearance_bracket']['minimum_sampled_node_clearance_lower_bound_mm'] for v in c['cases']),
  'continuous_surface_collision_verified':False}
result={'review_UTC':datetime.datetime.now(datetime.timezone.utc).isoformat(),
 'scope':'Independent source-code inspection plus arithmetic on available report snapshots; no rerun FEM solves.',
 'critical_input_checks':input_checks,'code_snapshot':[{'file':str(p),'sha256':sha(p)} for p in files],
 'available_report_snapshots':reports,'case_arithmetic_checks':cases,'independent_analytic_arithmetic':checks,
 'all_frozen_analysis_inputs_match':all(x['original_matches'] and x['copy_matches'] for x in input_checks),
 'max_reported_free_residual':max((x['relative_free_residual'] for x in cases),default=None),
 'max_requested_global_moment_error_Nmm':max((x.get('requested_to_assembled_global_moment_error_Nmm',0) for x in cases),default=None),
 'max_force_balance_norm_N':max((x['force_equilibrium_error_norm_N'] for x in cases),default=None),
 'max_moment_balance_norm_Nmm':max((x['moment_equilibrium_error_norm_Nmm'] for x in cases),default=None),
 'physical_joint_or_release_validation':False,
 'camera_solver_results_reviewed':bool(camera),
 'camera_report_snapshots':camera,
 'camera_scope':'Read-only review of setup, mapping, available static/modal/FRF data; completed medium captured-stud support sensitivity included. Final retained dynamics: coarse 12-mode eigsh results only. Newer LOBPCG implementation is not evidence of a completed refined modal solve.',
 'dynamic_mesh_convergence_established':False,'final_retained_dynamic_level':'coarse','final_retained_dynamic_mode_count':12,
 'coarse_executed_method_record':json.loads((ANA/'camera_fea/coarse/EXECUTION_METHOD.json').read_text()),
 'pin_report_snapshots':pins,'pin_medium_to_fine_response_changes':pin_last_changes,
 'independent_contact_compatibility_screen':contact}
(OUT/'fem_method_snapshot.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
print(json.dumps({k:result[k] for k in ['all_frozen_analysis_inputs_match','max_reported_free_residual','max_requested_global_moment_error_Nmm','max_force_balance_norm_N','max_moment_balance_norm_Nmm']},indent=2))
print('Reviewed report snapshots',len(reports),'available cases',len(cases))