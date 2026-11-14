"""Read-only V2R1 joint/system review. mm, N, MPa unless otherwise stated.

No CAD repair, no FEM, no hardware action. Sensitivity rows are not torque or
load recommendations. Run from any directory with the existing recovery_v2.
"""
from pathlib import Path
import csv, hashlib, json, math, datetime
import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
SRC = ROOT / 'recovery_v2'
G = 9.80665

def write_json(name, data):
    (OUT/name).write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')

def csv_out(name, rows):
    with (OUT/name).open('w', newline='', encoding='utf-8-sig') as f:
        w=csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)

def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def bbox(shape):
    b=shape.BoundingBox()
    return [[b.xmin,b.ymin,b.zmin],[b.xmax,b.ymax,b.zmax]]

manifest=json.loads((SRC/'deliverable/assembly_manifest.json').read_text())
freeze=json.loads((OUT/'source_freeze.json').read_text())
assert len(manifest['instances'])==95
assert sum(x['solid_count'] for x in manifest['instances'])==103
assert all(sha(ROOT/x['file'])==x['sha256'] for x in freeze['instances'])
assert all(sha(ROOT/x['file'])==x['sha256'] for x in freeze['files'])
parts={x['name']:x for x in manifest['instances']}
shapes={n:cq.Shape.importBrep(str(SRC/parts[n]['brep'])) for n in
        ['01_Direct_Base','02_Fuse_Cartridge_D8','04_Camera_Module_Short45']}

def cylinder_rows(s):
    rows=[]
    for f in s.Faces():
        a=BRepAdaptor_Surface(f.wrapped)
        if a.GetType()!=GeomAbs_Cylinder: continue
        c=a.Cylinder();p=c.Location();d=c.Axis().Direction()
        rows.append(dict(radius_mm=c.Radius(),axis_point_mm=[p.X(),p.Y(),p.Z()],
                         axis_direction=[d.X(),d.Y(),d.Z()],face_bounds_mm=bbox(f)))
    return rows

base,cart,module=(shapes[x] for x in ['01_Direct_Base','02_Fuse_Cartridge_D8','04_Camera_Module_Short45'])
cy={k:cylinder_rows(v) for k,v in shapes.items()}
def find_cyl(name,x,y,r):
    found=[c for c in cy[name] if abs(c['axis_point_mm'][0]-x)<1e-7 and
           abs(c['axis_point_mm'][1]-y)<1e-7 and abs(c['radius_mm']-r)<1e-7]
    assert found,(name,x,y,r)
    return found

moving=find_cyl('02_Fuse_Cartridge_D8',-21,56.5,2.1)[0]
fixed=find_cyl('01_Direct_Base',-25,35,2.1)[0]
neck=find_cyl('02_Fuse_Cartridge_D8',0,47,4)[0]
stems=find_cyl('02_Fuse_Cartridge_D8',8,50,1.8)+find_cyl('02_Fuse_Cartridge_D8',-8,50,1.8)
heads=find_cyl('02_Fuse_Cartridge_D8',8,50,3.5)+find_cyl('02_Fuse_Cartridge_D8',-8,50,3.5)
slots=find_cyl('04_Camera_Module_Short45',8,50,2.1)+find_cyl('04_Camera_Module_Short45',-8,50,2.1)

# Small section strips measure the actual BRep at the lug's hole centerline.
# A 0.002-mm strip uses the hole-center extremum rather than fitted dimensions.
def box(x0,x1,y0,y1,z0,z1):
    return cq.Solid.makeBox(x1-x0,y1-y0,z1-z0,cq.Vector(x0,y0,z0))
lugleft=cart.intersect(box(-30,-21,56.499,56.501,11,12))
fixedtop=base.intersect(box(-25.001,-24.999,35,45,3,4))
mb=bbox(lugleft);fb=bbox(fixedtop)
moving_ligament=mb[1][0]-mb[0][0]
fixed_ligament=fb[1][1]-fb[0][1]
assert abs(moving_ligament-.9)<1e-5
assert abs(fixed_ligament-1.9)<1e-5

geometry={
 'frame':'F: physical CR5AF flange; length mm',
 'source_manifest_sha256':sha(SRC/'deliverable/assembly_manifest.json'),
 'evidence_method':'Native BRep cylindrical surfaces; centerline thin-strip intersections; source code dimensions cross-check. No mesh fitting or CAD modifications.',
 'moving_tether_lug':{'hole_diameter_mm':2*moving['radius_mm'],'hole_axis_F_mm':moving['axis_point_mm'],
    'thickness_mm':moving['face_bounds_mm'][1][2]-moving['face_bounds_mm'][0][2],
    'nearest_outward_edge_F_X_mm':mb[0][0], 'minimum_nominal_ligament_mm':.9,
    'thin_strip_measured_ligament_mm':moving_ligament,
    'shear_out_area_two_planes_mm2':2*.9*2.5,
    'net_tension_area_for_minusX_pull_mm2':(7-4.2)*2.5,
    'warning':'Thin lug is part of the moving carrier. Its integrity is required after the D8 neck breaks; it has no published/proven catch rating.'},
 'fixed_tether_lug':{'hole_diameter_mm':2*fixed['radius_mm'],'hole_axis_F_mm':fixed['axis_point_mm'],
    'thickness_mm':fixed['face_bounds_mm'][1][2]-fixed['face_bounds_mm'][0][2],
    'nearest_outward_edge_F_Y_mm':fb[1][1],'minimum_nominal_ligament_mm':1.9,
    'thin_strip_measured_ligament_mm':fixed_ligament,
    'shear_out_area_two_planes_mm2':2*1.9*7.5,
    'net_tension_area_for_plusY_pull_mm2':(12-4.2)*7.5,
    'warning':'Shortest-ligament screen only, not full pull-direction failure analysis or rope contact model.'},
 'D8_fuse_neck':{'diameter_mm':8,'straight_zone_F_Z_mm':[8.1,9.1],
    'area_mm2':math.pi*8**2/4,'second_moment_mm4':math.pi*8**4/64,
    'polar_moment_mm4':math.pi*8**4/32,'bending_section_modulus_mm3':math.pi*8**3/32,
    'nominal_bending_stress_MPa_per_Nm':1000/(math.pi*8**3/32),
    'nominal_axial_stress_MPa_per_N':1/(math.pi*8**2/4),
    'nominal_max_transverse_shear_MPa_per_N':4/(3*math.pi*8**2/4),
    'warning':'Nominal gross-section formulas, no stress concentration, print anisotropy, notch sensitivity, fatigue or fracture law. D8 alone does not specify release force.'},
 'quick_release':{'stud_stem_diameter_mm':3.6,'slot_width_mm':4.2,
    'nominal_lateral_diametral_clearance_mm':.6,'per_side_nominal_clearance_mm':.3,
    'stud_underhead_F_Z_mm':heads[0]['face_bounds_mm'][0][2],
    'module_foot_top_F_Z_mm':slots[0]['face_bounds_mm'][1][2],
    'nominal_vertical_free_clearance_mm':heads[0]['face_bounds_mm'][0][2]-slots[0]['face_bounds_mm'][1][2],
    'stud_axis_spacing_mm':16,'entry_diameter_mm':7.4,'head_diameter_mm':7,
    'warning':'Local dimensional clearance is confirmed; actual six-DOF play depends on foot contacts, hook force and print tolerance. No explicit preload/wedge removes this clearance in the CAD.'},
 'actual_analytic_surface_evidence':{'moving_lug':moving,'fixed_lug':fixed,'neck':neck,'stems':stems,'heads':heads,'slot_ends':slots},
 'shapes_valid':all(s.isValid() for s in shapes.values())}
write_json('joint_geometry_verified.json',geometry)

# No material assignment is inferred from visual color, file names, or infill.
materials=[
 dict(name='Bambu PLA Basic',version='TDS V3.0',density=1.24,E_XY=2580,E_XY_pm=220,E_Z=2060,E_Z_pm=170,S_XY=35,S_XY_pm=4,S_Z=31,S_Z_pm=3,B_E_XY=2750,B_E_Z=2370,B_S_XY=76,B_S_Z=59,HDT_1p8=54,HDT_0p45=57,nozzle=220,bed=35,speed=200,post='Annealed and dried 55 C for 8 h',source='Bambu_PLA_Basic_TDS_V3_0.pdf'),
 dict(name='Bambu PETG HF',version='TDS V1.0',density=1.28,E_XY=1810,E_XY_pm=190,E_Z=1540,E_Z_pm=130,S_XY=34,S_XY_pm=4,S_Z=23,S_Z_pm=4,B_E_XY=2050,B_E_Z=1810,B_S_XY=64,B_S_Z=48,HDT_1p8=62,HDT_0p45=69,nozzle=255,bed=70,speed=200,post='Annealed and dried 75 C for 8 h',source='Bambu_PETG_HF_TDS_V1_0.pdf'),
 dict(name='Bambu PETG Basic',version='TDS V3.0 current retrieved formulation',density=1.25,E_XY=2780,E_XY_pm=65,E_Z=2550,E_Z_pm=100,S_XY=51,S_XY_pm=1,S_Z=35,S_Z_pm=6,B_E_XY=1950,B_E_Z=1740,B_S_XY=75,B_S_Z=56,HDT_1p8=68,HDT_0p45=71,nozzle=255,bed=70,speed=150,post='No pre-test annealing stated in specimen section; X1-Carbon stated',source='Bambu_PETG_Basic_TDS_V3_0.pdf')]
material_rows=[]
for m in materials:
    for axis in ['XY','Z']:
        material_rows.append({'material':m['name'],'version':m['version'],'axis':axis,
           'tensile_Young_modulus_MPa':m['E_'+axis], 'tensile_E_published_plusminus_MPa':m['E_'+axis+'_pm'],
           'tensile_strength_MPa':m['S_'+axis], 'tensile_strength_published_plusminus_MPa':m['S_'+axis+'_pm'],
           'flexural_modulus_MPa':m['B_E_'+axis],'flexural_strength_MPa':m['B_S_'+axis],
           'density_g_per_cm3':m['density'],'HDT_1p8MPa_C':m['HDT_1p8'],'HDT_0p45MPa_C':m['HDT_0p45'],
           'specimen_infill_percent':100,'specimen_nozzle_C':m['nozzle'],'specimen_bed_C':m['bed'],
           'specimen_speed_mm_s':m['speed'],'specimen_post_treatment':m['post'],
           'property_page_PDF':2,'print_condition_page_PDF':3,'source_file':m['source']})
csv_out('manufacturer_material_inputs.csv',material_rows)
write_json('manufacturer_material_inputs.json',{'materials':material_rows,
  'tensile_method':'ISO 527 / GB T1040','flexural_method':'ISO 178 / GB T9341',
  'published_plusminus_is_not_design_allowable_or_confidence_bound':True,
  'not_published_or_not_established_here':['Full orthotropic elastic constants and Poisson ratios','Compressive and shear design allowables','Creep/relaxation curves','Fatigue S-N curves','Strain-rate/fracture law','Actual user brand, formulation, infill, wall count, layer orientation, moisture, print temperature and post-treatment'],
  'no_actual_material_assignment':True})

# Washer projected area uses actual larger plastic hole, not merely washer ID.
joint_defs=[
 {'joint':'M6_robot','count':4,'nominal_thread_diameter_mm':6,'washer_OD_mm':12,'washer_ID_mm':6.4,'plastic_hole_mm':6.6,'plate_mm':7.5,'washer_mm':1.6,'nominal_underhead_mm':16,'other_stack_mm':0,'manufacturer_depth_mm':8,'torques':[.25,.5,1,2,15,18]},
 {'joint':'M3_motor','count':4,'nominal_thread_diameter_mm':3,'washer_OD_mm':7,'washer_ID_mm':3.2,'plastic_hole_mm':3.4,'plate_mm':4.5,'washer_mm':.5,'nominal_underhead_mm':10,'other_stack_mm':2.4,'manufacturer_depth_mm':4,'torques':[.05,.1,.2,.3,.5,1]},
 {'joint':'camera_1_4_20_assumed_head','count':1,'nominal_thread_diameter_mm':6.35,'washer_OD_mm':20,'washer_ID_mm':6.35,'plastic_hole_mm':6.7,'plate_mm':2,'washer_mm':0,'nominal_underhead_mm':5,'other_stack_mm':0,'manufacturer_depth_mm':None,'torques':[.1,.25,.5,1]}]
rows=[];joints=[]
for j in joint_defs:
    area=math.pi/4*(j['washer_OD_mm']**2-max(j['washer_ID_mm'],j['plastic_hole_mm'])**2)
    engagement=j['nominal_underhead_mm']-j['plate_mm']-j['washer_mm']-j['other_stack_mm']
    jr={k:v for k,v in j.items() if k!='torques'}
    jr.update(projected_plastic_bearing_area_mm2=area,nominal_thread_projection_mm=engagement,
              nominal_depth_margin_mm=(j['manufacturer_depth_mm']-engagement if j['manufacturer_depth_mm'] is not None else None))
    joints.append(jr)
    for T in j['torques']:
        fmin=T/(.25*j['nominal_thread_diameter_mm']/1000)
        fmax=T/(.15*j['nominal_thread_diameter_mm']/1000)
        rows.append(dict(joint=j['joint'],torque_sensitivity_Nm=T,K_low=.15,K_high=.25,
           per_bolt_preload_min_N=fmin,per_bolt_preload_max_N=fmax,
           projected_contact_area_mm2=area,mean_bearing_pressure_min_MPa=fmin/area,
           mean_bearing_pressure_max_MPa=fmax/area,
           torque_recommendation=False))
csv_out('torque_preload_bearing_sensitivity.csv',rows)
write_json('fastener_stack_and_preload.json',{'formula':'T=K*F*d; consistent SI units. Mean bearing pressure=F/projected supported annular area.',
   'source':'NASA RP-1228, PDF page21 / printed17 Alternative Torque Formula; tableVI PDF page22 / printed18',
   'K_range':[.15,.25],'K_range_status':'Assumed sensitivity range, not measured coefficient for these screw/washer/thread interfaces. NASA gives common K0.2, steel-on-steel typical0.15 and tableVI K0.25 at friction0.2; K is not friction coefficient mu.',
   'joints':joints,'torque_recommendation':None,
   'limitations':['Projection is not effective full-thread engagement; entrance/bolt chamfers and tolerances unmeasured.',
    'No thread material/grade or actual bolt proof strength established; no stripping margin claimed.',
    'The cover A is in the M3 clamp stack. Its material, creep and strength are unknown.',
    'Uniform projected pressure omits local washer bending, edge pressure and plastic creep.',
    'NASA formula is a screening relation; thread/head friction, coating, lubrication and seating change preload.',
    'DOBOT metal-joint table M6 external hex15Nm/internal grade12.9 18Nm does not apply to this printed joint.',
    'Camera head diameter20mm, head-underlength5mm and camera allowed thread depth remain unmeasured.'],
   'bearing_sensitivity_rows':rows})

# Tether energy balance assumes a translating mass falling vertically; h is drop
# before tension begins, v0 is speed BEFORE that fall, and s is added stopping
# displacement after engagement. This avoids double-counting impact velocity.
energy=[]
for m in [.18,.25,.35]:
 for h_mm in [0,20,50]:
  for v0 in [0,.5,1]:
   for stop_mm in [.5,2,10]:
    h=h_mm/1000;s=stop_mm/1000;E=m*G*h+.5*m*v0*v0
    net=E/s;avg=net+m*G;peak_linear=2*avg
    energy.append(dict(mass_assumption_kg=m,slack_drop_assumption_mm=h_mm,
      pre_drop_velocity_assumption_m_s=v0,stop_distance_assumption_mm=stop_mm,
      energy_at_tether_engagement_J=E,average_net_retarding_force_N=net,
      average_tether_tension_vertical_N=avg,ideal_linear_spring_peak_tension_N=peak_linear,
      moving_lug_nominal_shear_out_stress_at_average_MPa=avg/4.5,
      moving_lug_nominal_shear_out_stress_at_ideal_linear_peak_MPa=peak_linear/4.5,
      fixed_lug_nominal_shear_out_stress_at_average_MPa=avg/28.5,
      actual_impact_or_rating=False))
csv_out('tether_energy_sensitivity.csv',energy)
write_json('tether_energy_model.json',{'gravity_m_s2':G,
  'energy_at_engagement':'E0=m*g*h+0.5*m*v0^2',
  'average_net_retarding_force':'E0/s',
  'average_tether_tension_for_vertical_downward_stop':'E0/s+m*g',
  'ideal_zero_initial_tension_linear_spring_peak':'2*(E0/s+m*g)',
  'initial_velocity_definition':'v0 is speed before the slack drop; if supplied measured engagement velocity, use E0=0.5*m*v_engage^2 and do not also add m*g*h.',
  'assumptions':['Point mass, all initial translation aligned with eventual tether extension; vertical gravity during drop and arrest.',
    'Stopping distance is externally specified, not derived from cord stiffness, knot tightening or lug compliance.',
    'No fracture energy, damping or rotational energy modeled; add 0.5*omega^T*I*omega for a validated rotating-body model.',
    'Ideal linear-spring peak is one illustrative force history, not an upper bound or predicted real peak.',
    'For a slanted/horizontal tether, project gravity/displacement and solve actual motion.',
    'Rope route CAD is only a 1.5mm diameter envelope; no cord rating, slack, knot efficiency, stroke or catch validation exists.'],
  'nominal_lug_stress_model':'Shear-out screen tau=T/(2*e*t), shortest edge direction. No shear allowable available; cannot infer pass/fail from tensile strength.',
  'model_type':'Algebraic energy and nominal-section sensitivity, not transient impact simulation',
  'rows':len(energy)})

dense=[]
for x in manifest['instances']:
 if x['category']!='custom_rigid':continue
 for m in materials:
    dense.append(dict(part=x['name'],solid_CAD_volume_mm3=x['volume_mm3'],assumed_material=m['name'],
                      density_g_cm3=m['density'],fully_dense_geometric_mass_g=x['volume_mm3']/1000*m['density'],
                      actual_printed_mass=False))
csv_out('fully_dense_geometric_mass_reference.csv',dense)

# Gross round-neck nominal stresses show load/moment scaling, not fracture force.
ns=[]
for force in [1,5,10,20,50,100]:
 for arm in [25,50,75,100]:
    ns.append(dict(force_assumption_N=force,perpendicular_lever_arm_assumption_mm=arm,
              neck_bending_moment_Nmm=force*arm,
              nominal_D8_bending_stress_MPa=force*arm/(math.pi*8**3/32),
              nominal_D8_max_transverse_shear_MPa=4*force/(3*math.pi*8**2/4),
              not_a_release_force_prediction=True))
csv_out('D8_neck_nominal_sensitivity.csv',ns)

ri=json.loads((SRC/'validation/current_interfaces.json').read_text())
rr=json.loads((SRC/'validation/robot_pairs.json').read_text())
gr=json.loads((SRC/'validation/current_gripper_external_clearance.json').read_text())
er=json.loads((SRC/'validation/export_readback.json').read_text())
cr=json.loads((SRC/'camera/camera_reference_and_access_verification.json').read_text())
summary={
 'frozen_manifest_matches_existing_export_readback':er['manifest_sha256']==sha(SRC/'deliverable/assembly_manifest.json'),
 'export_integrity':[{'file':s['file'],'sha256_matches_current':s['sha256']==sha(SRC/s['file']),
                     'instances':s['actual_geometry_instances'],'native_solids':s['actual_native_solids'],
                     'all_BRep_valid_in_existing_readback':s['all_BRep_valid']} for s in er['STEP']],
 'new_interface_review':{k:v for k,v in ri.items() if isinstance(v,(int,float,bool,str)) or k in ['classification_counts']},
 'robot_pair_review':{k:v for k,v in rr.items() if isinstance(v,(int,float,bool,str)) or k in ['classification_counts','source_robot_overlaps']},
 'gripper_external_review':{'static':{k:v for k,v in gr['static'].items() if isinstance(v,(int,float,bool,str))},
                          'jaw_continuous_translation':{k:v for k,v in gr['jaw_continuous_translation'].items() if isinstance(v,(int,float,bool,str))},
                          'output_group_external_rotation_envelope':{k:v for k,v in gr['output_group_external_rotation_envelope'].items() if isinstance(v,(int,float,bool,str))}},
 'camera_source_Lens_Body_overlap_mm3':cr['source_Lens_Body_internal_overlap_mm3'],
 'historical_supplier_internal_report':'recovery_v2/gripper/known_source_issues_HISTORICAL_NOT_RERUN.json',
 'unqualified_all_4465_pairs_zero_intersection_claim':False,
 'structural_or_dynamic_certification':False,
 'all_freeze_file_hashes_unchanged':all(sha(ROOT/x['file'])==x['sha256'] for x in freeze['files']),
 'all_95_instance_hashes_unchanged':all(sha(ROOT/x['file'])==x['sha256'] for x in freeze['instances'])}
write_json('existing_validation_scope_review.json',summary)
write_json('system_quantitative_summary.json',{'revision':'V2R1 read-only structural input review v1',
 'completed_UTC':datetime.datetime.now(datetime.timezone.utc).isoformat(),
 'source_manifest_sha256':sha(SRC/'deliverable/assembly_manifest.json'),
 'instances':95,'native_solids':103,'native_geometry_modified':False,
 'minimum_moving_tether_lug_ligament_mm':.9,'minimum_fixed_lug_ligament_mm':1.9,
 'moving_lug_thickness_mm':2.5,'fixed_lug_thickness_mm':7.5,
 'quick_release_nominal_lateral_diametral_clearance_mm':.6,
 'quick_release_nominal_lift_clearance_mm':.3,'fuse_neck_diameter_mm':8,
 'M6_nominal_projection_mm':6.9,'M3_nominal_projection_mm':2.6,
 'camera_screw_nominal_projection_mm':3,'actual_camera_thread_depth_known':False,
 'actual_print_material_infill_mass_acceleration_known':False,
 'release_force_established':False,'tether_catch_rating_established':False,
 'actual_torque_recommendation':None,
 'material_input_variants':3,'tether_sensitivity_cases':len(energy),
 'source_files_unchanged':summary['all_freeze_file_hashes_unchanged'] and summary['all_95_instance_hashes_unchanged']})
print(json.dumps({'geometry_checks_valid':geometry['shapes_valid'],'moving_ligament_mm':moving_ligament,
 'fixed_ligament_mm':fixed_ligament,'washer_areas_mm2':[j['projected_plastic_bearing_area_mm2'] for j in joints],
 'tether_cases':len(energy),'source_files_unchanged':summary['all_95_instance_hashes_unchanged']},indent=2))