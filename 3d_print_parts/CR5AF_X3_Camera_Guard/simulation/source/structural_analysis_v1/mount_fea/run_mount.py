"""Actual CAD P2 elasticity of the recovered V2R1 mounting pieces.

The clamped contact conditions are declared idealizations, not physical
fastener preload or unilateral contact simulations. Every force case is
unit-normalized for transparent load/mass scaling by the report author.
"""
from pathlib import Path
import argparse,sys,time,gc,json
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'fem_core'))
from solid_fem import SolidFE,gmsh_mesh,write_json,sha256

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
SOURCE=ROOT/'inputs'
parser=argparse.ArgumentParser();parser.add_argument('part',choices=['base','cartridge']);parser.add_argument('--level',choices=['coarse','medium','fine'],required=True)
parser.add_argument('--mesh-only',action='store_true');parser.add_argument('--solver',default='amg')
parser.add_argument('--post-break-lug',action='store_true')
args=parser.parse_args()
part=args.part;level=args.level
if args.post_break_lug and part!='cartridge':raise ValueError('Post-break lug applies to cartridge only')
out=HERE/(part+'_postbreak' if args.post_break_lug else part)/level;out.mkdir(parents=True,exist_ok=True)
name={'base':'01_Direct_Base','cartridge':'02_Fuse_Cartridge_D8'}[part]
cad=SOURCE/(name+'.brep');meshpath=HERE/part/level/(name+'.msh')
hmax={'base':{'coarse':5.,'medium':3.6,'fine':2.6},'cartridge':{'coarse':2.4,'medium':1.7,'fine':1.2}}[part][level]
if not meshpath.exists():
    boxes=[]
    if part=='cartridge':
        local={'coarse':.75,'medium':.53,'fine':.375}[level]
        boxes=[(-24.1,52.9,10.4,-14.9,60.1,13.1,local),(-6.1,40.9,6.6,6.1,53.1,10.6,hmax*.55)]
    meta=gmsh_mesh(cad,meshpath,hmax,refinement_boxes=boxes)
else:meta=json.loads(meshpath.with_suffix('.mesh.json').read_text())
assert meta['source_sha256']==sha256(cad),'Source CAD changed; stop'
print('MESH',part,level,meta,flush=True)
if args.mesh_only:sys.exit(0)
fe=SolidFE(meshpath);print('DOFS',fe.mesh_info,flush=True)
axes=np.eye(3);cases=[];patches={}
if part=='base':
    bxy=np.array([[25*np.cos(np.deg2rad(a)),25*np.sin(np.deg2rad(a))] for a in (45,135,225,315)])
    def support(x):
        near=np.min(np.sum((x[:2,:,None]-bxy.T[:,None,:])**2,axis=0),axis=1)<=6.0**2
        return np.isclose(x[2],0,atol=1e-7)&near
    fixed=fe.facets(support)
    patches['motor_platform']=fe.facets(lambda x:np.isclose(x[2],26.5,atol=1e-7))
    patches['receiver']=fe.facets(lambda x:(x[0]>=-18.0001)&(x[0]<=15.2001)&(x[1]>=40.59)&(x[1]<=53.41)&(x[2]>=1.9999)&(x[2]<=7.5001)
        &(np.isclose(x[2],2,atol=1e-7)
          | np.isclose(x[1],40.6+(x[2]-2)*1.6/5.7,atol=1e-6)
          | np.isclose(x[1],53.4-(x[2]-2)*1.6/5.7,atol=1e-6)))
    patches['tether_hole']=fe.facets(lambda x:(abs(np.sqrt((x[0]+25)**2+(x[1]-35)**2)-2.1)<.15)&(x[2]>0)&(x[2]<7.5))
    patches['tether_minusX_half_hole']=fe.facets(lambda x:(abs(np.sqrt((x[0]+25)**2+(x[1]-35)**2)-2.1)<.15)&(x[0]<-25)&(x[2]>0)&(x[2]<7.5))
    for j,axis in enumerate('XYZ'):
        cases.append((f'motor_F{axis}_1N_at_z100','motor_platform',axes[j],np.cross([0,0,100],axes[j]),[0,0,0]))
        cases.append((f'motor_M{axis}_1Nmm','motor_platform',[0,0,0],axes[j],[0,0,0]))
        cases.append((f'receiver_F{axis}_1N','receiver',axes[j],[0,0,0],[0,47,6.7]))
        cases.append((f'receiver_M{axis}_1Nmm','receiver',[0,0,0],axes[j],[0,47,6.7]))
    for label,F in [('minusX',[-1,0,0]),('plusZ',[0,0,1]),('minusZ',[0,0,-1])]:
        cases.append((f'tether_{label}_1N','tether_hole',F,[0,0,0],[-25,35,3.75]))
    cases.append(('tether_minusX_bearing_half_1N','tether_minusX_half_hole',[-1,0,0],[0,0,0],[-25,35,3.75]))
    regions=dict(central_post=lambda x:(x[2]>=7.5)&(x[2]<=22)&(x[0]**2+x[1]**2<=12.1**2),
                 platform=lambda x:x[2]>=22,
                 fixed_tether_lug=lambda x:(x[0]<-19)&(x[1]>28)&(x[1]<39),
                 receiver=lambda x:(x[1]>36)&(x[2]<7.6))
    bc='Rigidly fixed all three translations on bottom z=0 annular pad facets whose centroids are within radius6mm of the four PCD50 M6 axes. All other mounting surfaces are free. No bolt preload/contact/friction.'
else:
    fixed=fe.facets(lambda x:x[2]<=6.7000001)
    def carrier_interface(x):
        top=np.isclose(x[2],13,atol=1e-7)&(x[0]>=-14)&(x[0]<=14)&(x[1]>=39)&(x[1]<=58)
        r=np.minimum(np.sqrt((x[0]+8)**2+(x[1]-50)**2),np.sqrt((x[0]-8)**2+(x[1]-50)**2))
        shank=(x[2]>13)&(x[2]<16.3)&(abs(r-1.8)<.15)
        head=np.isclose(x[2],16.3,atol=1e-7)&(r<3.51)
        return top|shank|head
    patches['carrier_stud_interface']=fe.facets(carrier_interface)
    patches['tether_hole']=fe.facets(lambda x:(abs(np.sqrt((x[0]+21)**2+(x[1]-56.5)**2)-2.1)<.08)&(x[2]>10.5)&(x[2]<13))
    patches['tether_minusX_half_hole']=fe.facets(lambda x:(abs(np.sqrt((x[0]+21)**2+(x[1]-56.5)**2)-2.1)<.08)&(x[0]<-21)&(x[2]>10.5)&(x[2]<13))
    for j,axis in enumerate('XYZ'):
        cases.append((f'carrier_F{axis}_1N','carrier_stud_interface',axes[j],[0,0,0],[0,50,13]))
        cases.append((f'carrier_M{axis}_1Nmm','carrier_stud_interface',[0,0,0],axes[j],[0,50,13]))
        point=[0,112.3050865,25.3050865]
        cases.append((f'cameraCoG_F{axis}_1N','carrier_stud_interface',axes[j],np.cross(np.asarray(point)-[0,50,13],axes[j]),[0,50,13]))
        cg=(.18*np.asarray([0,112.3050865276,25.3050865276])+.03570000317*np.asarray([0,82.1837402047,.7842638939]))/.21570000317
        cases.append((f'cameraPlusFrameCoG_F{axis}_1N','carrier_stud_interface',axes[j],np.cross(cg-[0,50,13],axes[j]),[0,50,13]))
    for label,F in [('minusX',[-1,0,0]),('plusZ',[0,0,1]),('minusZ',[0,0,-1])]:
        cases.append((f'tether_{label}_1N','tether_hole',F,[0,0,0],[-21,56.5,11.75]))
    cases.append(('tether_minusX_bearing_half_1N','tether_minusX_half_hole',[-1,0,0],[0,0,0],[-21,56.5,11.75]))
    regions=dict(neck=lambda x:(x[2]>=6.7)&(x[2]<=10.5)&(x[0]**2+(x[1]-47)**2<=6.1**2),
                 minimum_D8_neck=lambda x:(x[2]>=8.1)&(x[2]<=9.1)&(x[0]**2+(x[1]-47)**2<=4.01**2),
                 moving_tether_lug=lambda x:(x[0]<=-15)&(x[1]>=53)&(x[1]<=60)&(x[2]>=10.5)&(x[2]<=13),
                 lug_thin_outer_edge=lambda x:(x[0]<=-23.1)&(x[1]>=53)&(x[1]<=60)&(x[2]>=10.5)&(x[2]<=13),
                 lug_root=lambda x:(x[0]>=-17)&(x[0]<=-15)&(x[1]>=53)&(x[1]<=60)&(x[2]>=10.5)&(x[2]<=13),
                 latch_beam=lambda x:(x[0]>=10)&(x[1]>=41)&(x[2]>=10.5),
                 studs=lambda x:(x[2]>=13)&(x[2]<=17.9)&
                    (np.minimum((x[0]-8)**2+(x[1]-50)**2,(x[0]+8)**2+(x[1]-50)**2)<=3.500001**2))
    bc='Rigidly fixed all three translations on all exposed lower-dovetail facets at or below z6.7. Carrier top within camera foot plus stud shanks/underheads receive equivalent distributed bonded load. Existing clearances/slip/opening/friction/contact are not solved.'
for j,axis in enumerate('XYZ'):
    cases.append((f'self_weight_{axis}_1g',None,axes[j]*9806.65,[0,0,0],[0,0,0]))
if args.post_break_lug:
    fixed=patches['carrier_stud_interface']
    cases=[row for row in cases if row[0].startswith('tether_')]
    bc='POST_BREAK equivalent local lug submodel: lower dovetail is completely free; carrier/stud interface fixed instead as an idealized camera-side inertial reaction. Original uncut CAD is retained; no fracture/contact process is simulated. Fixed interface replaces the remaining moving assembly; unused lower geometry bears no imposed support reaction.'
# Solve the six carrier wrench states first, so all remote loads are exact
# linear combinations rather than independent repeated factorizations/CG runs.
cases.sort(key=lambda row:1 if 'CoG_' in row[0] else 0)
report=dict(part=name,revision='V2R1_reconstructed',source_sha256=sha256(cad),mesh=meta,mesh_info=fe.mesh_info,
            load_path_state=('POST_BREAK_local_lug' if args.post_break_lug else 'PRE_BREAK_intact'),
            volume_relative_error=(fe.mesh_info['volume_mm3']/meta['cad_volume_mm3']-1),
            assumptions=dict(E_MPa=2000,nu=.35,density_tonne_per_mm3=1.24e-9,
                             material='Hypothetical homogeneous isotropic linear elastic print baseline, not measured PETG properties',
                             fixed_support=bc,small_displacement=True,load_type='Static consistent surface traction; unit resultant/wrench',
                             contact_limitations='No contact opening, gaps, plasticity, fatigue, creep, fracture or collision release simulation'),
            support=fe.boundary_info(fixed),load_patches={k:fe.boundary_info(v) for k,v in patches.items()},cases={})
old_report=None;old_fields=None
if (out/'unit_response_fields.npz').exists():
    old_report=json.loads((out/'results.json').read_text());old_fields=np.load(out/'unit_response_fields.npz')
    report['cases']=old_report['cases']
write_json(out/'results.json',report)
print('BC',report['support'],report['load_patches'],flush=True)
report['assembly']=fe.assemble(fixed,solver=args.solver)
print('ASSEMBLED',report['assembly'],flush=True)
Us=[];Ss=[];names=[];Fs=[]
for case,patch,F,M,about in cases:
    if patch is None:
        f=fe.body_force(F)
        load=dict(type='uniform body acceleration',acceleration_mm_s2=np.asarray(F).tolist(),
                  assembled_wrench=fe.resultant(f))
    else:
        f,load=fe.load_wrench(patches[patch],F,M,about)
    same_old_load=(old_report is not None and case in old_report['cases'] and
        old_report['cases'][case]['load']==load)
    if old_fields is not None and case in old_fields['case_names'] and same_old_load:
        idx=list(old_fields['case_names']).index(case);u=old_fields['displacement'][:,idx]
        solution=old_report['cases'][case]['solution']
        stress,fields=fe.summarize(u,regions=regions,return_fields=True)
    elif part=='cartridge' and 'CoG_' in case:
        u=np.zeros(fe.basis.N)
        for j,a in enumerate('XYZ'):
            u+=float(F[j])*Us[names.index(f'carrier_F{a}_1N')]
            u+=float(M[j])*Us[names.index(f'carrier_M{a}_1Nmm')]
        solution=fe.evaluate_solution(f,u,'Exact linear combination of six solved carrier wrench states')
        stress,fields=fe.summarize(u,regions=regions,return_fields=True)
    elif case=='tether_minusZ_1N' and 'tether_plusZ_1N' in names:
        u=-Us[names.index('tether_plusZ_1N')]
        solution=fe.evaluate_solution(f,u,'Sign reversal in linear elasticity')
        stress,fields=fe.summarize(u,regions=regions,return_fields=True)
    else:
        u,solution=fe.solve(f)
        stress,fields=fe.summarize(u,regions=regions,return_fields=True)
    report['cases'][case]=dict(load=load,solution=solution,response=stress)
    print('CASE',case,'u',stress['max_displacement_mm'],'vm99',stress['volume_weighted_p99_von_mises_MPa'],'peak',stress['max_von_mises_MPa'],'energy',solution['strain_energy_Nmm'],'solve',solution['solve_seconds'],flush=True)
    Us.append(u);Ss.append(fields['stress6_cell_mean']);names.append(case);Fs.append(f)
    write_json(out/'results.json',report)
U=np.stack(Us,axis=1);Fmat=np.stack(Fs,axis=1)
C=Fmat.T@U
report['generalized_compliance']=dict(case_names=names,matrix=C.tolist(),max_symmetry_error=float(abs(C-C.T).max()))
np.savez_compressed(out/'unit_response_fields.npz',case_names=np.asarray(names),displacement=U,
    stress6_cell_mean=np.stack(Ss,axis=-1),cell_volume=fields['cell_volume'],cell_centres=fields['cell_centres'],
    geometry_nodes=fe.mesh.p,tetra_vertices=fe.mesh.t,free_dofs=fe.free,fixed_dofs=fe.fixed)
if (level=='fine' and not args.post_break_lug) or (level=='medium' and args.post_break_lug):
    visual_cases=(['tether_minusX_bearing_half_1N','tether_plusZ_1N'] if args.post_break_lug else
                  ['motor_FX_1N_at_z100','motor_FZ_1N_at_z100'] if part=='base' else
                  ['cameraPlusFrameCoG_FX_1N','cameraPlusFrameCoG_FZ_1N','tether_minusX_bearing_half_1N','tether_plusZ_1N'])
    for case in visual_cases:
        fe.export_vtu(out/(case+'.vtu'),U[:,names.index(case)])
report['complete']=True;write_json(out/'results.json',report)
print('COMPLETE',part,level,flush=True)