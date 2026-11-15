"""Combine solved tensor/vector bases into physical service-load cases.

All comparisons use a declared homogeneous material and engaged ideal
interfaces. No von-Mises scalars or percentiles are linearly superposed.
"""
from pathlib import Path
import argparse,sys,json
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'fem_core'))
from solid_fem import SolidFE,write_json,weighted_quantile


def mass_data():
    source=json.loads((ROOT/'root_checks'/'source_mass_geometry.json').read_text())
    entries=[('camera',.18,np.array([0.,112.3050865276,25.3050865276]))]
    for n in ['04_Camera_Module_Short45_F','02_Fuse_Cartridge_D8','03_Cartridge_Retaining_Pin']:
        e=source[n];entries.append((n,e['mass_kg_at_1p24g_cm3'],np.array(e['center_mm'])))
    m=sum(e[1] for e in entries);cg=sum(e[1]*e[2] for e in entries)/m
    mf=entries[0][1]+entries[1][1]
    cgf=(entries[0][1]*entries[0][2]+entries[1][1]*entries[1][2])/mf
    return source,entries,m,cg,mf,cgf


def load_coefficients(part,names,motor_branch_mass_kg=2.,motor_branch_height_mm=100.):
    _,entries,m,cg,mf,cgf=mass_data();g=9.80665
    coeff=np.zeros((len(names),3));observe=np.zeros((len(names),6))
    origin=np.array([0.,47.,6.7]);cam=entries[0][2]
    for j,a in enumerate('XYZ'):
        coeff[names.index(f'self_weight_{a}_1g'),j]=1
        if part=='cartridge':
            coeff[names.index(f'cameraPlusFrameCoG_F{a}_1N'),j]=mf*g
            observe[names.index(f'cameraCoG_F{a}_1N'),j]=1
            observe[names.index(f'carrier_M{a}_1Nmm'),3+j]=1
        else:
            coeff[names.index(f'motor_F{a}_1N_at_z100'),j]=motor_branch_mass_kg*g
            arm=np.array([0.,0.,motor_branch_height_mm-100.])
            extra=np.cross(arm,np.eye(3)[j]*motor_branch_mass_kg*g)
            F=np.eye(3)[j]*m*g;M=np.cross(cg-origin,F)
            moment_virtual=np.cross(cam-origin,np.eye(3)[j])
            for k,b in enumerate('XYZ'):
                coeff[names.index(f'motor_M{b}_1Nmm'),j]+=extra[k]
                coeff[names.index(f'receiver_F{b}_1N'),j]+=F[k]
                coeff[names.index(f'receiver_M{b}_1Nmm'),j]+=M[k]
                observe[names.index(f'receiver_F{b}_1N'),j]+=float(j==k)
                observe[names.index(f'receiver_M{b}_1Nmm'),j]+=moment_virtual[k]
            observe[names.index(f'receiver_M{a}_1Nmm'),3+j]=1
    return coeff,observe


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--part',choices=['base','cartridge'],required=True)
    ap.add_argument('--level',choices=['coarse','medium','fine'],required=True)
    ap.add_argument('--motor-mass',type=float,default=2.);args=ap.parse_args()
    inp=ROOT/'mount_fea'/args.part/args.level
    r=json.loads((inp/'results.json').read_text())
    if not r.get('complete'):raise RuntimeError('Unit-basis source incomplete')
    d=np.load(inp/'unit_response_fields.npz');names=list(d['case_names'])
    coeff,obs=load_coefficients(args.part,names,args.motor_mass)
    U=d['displacement']@coeff
    C=np.asarray(r['generalized_compliance']['matrix'])
    Cnames=r['generalized_compliance']['case_names']
    if Cnames!=names:raise RuntimeError('Compliance ordering mismatch')
    q=obs.T@C@coeff
    mesh=inp/(r['part']+'.msh');fe=SolidFE(mesh)
    if args.part=='cartridge':
        regions={'minimum_D8_neck':lambda x:(x[2]>=8.1)&(x[2]<=9.1)&(x[0]**2+(x[1]-47)**2<=4.01**2),
            'upper_features_z_band':lambda x:(x[2]>=13)&(x[2]<=17.9),
            'moving_tether_lug':lambda x:(x[0]<=-15)&(x[1]>=53)&(x[1]<=60)&(x[2]>=10.5)&(x[2]<=13)}
    else:
        regions={'central_post':lambda x:(x[2]>=7.5)&(x[2]<=22)&(x[0]**2+x[1]**2<=12.1**2),
            'motor_platform':lambda x:x[2]>=22,
            'receiver':lambda x:(x[1]>36)&(x[2]<7.6)}
    out=ROOT/'service_results'/args.part/args.level;out.mkdir(parents=True,exist_ok=True)
    source,entries,m,cg,mf,cgf=mass_data()
    result={'part':args.part,'mesh_level':args.level,'source_sha256':r['source_sha256'],
        'mesh_info':fe.mesh_info,'material_assumptions':r['assumptions'],
        'load_definition':{'g_m_s2':9.80665,'effective_acceleration_definition':'Each X/Y/Z case has total effective acceleration 1g or 3g along that axis, including gravity. The three axes are separate cases, not simultaneous 3g each.',
          'motor_gripper_object_branch_mass_kg_assumed':args.motor_mass if args.part=='base' else None,
          'motor_branch_CoG_F_mm_assumed':[0,0,100] if args.part=='base' else None,
          'camera_mass_kg':.18,'camera_CoG_F_mm_assumed':entries[0][2].tolist(),
          'camera_and_frame_mass_kg':mf,'camera_and_frame_CoG_F_mm':cgf.tolist(),
          'total_camera_side_mass_kg':m,'total_camera_side_CoG_F_mm':cg.tolist(),
          'additional_note':'Printed geometry is assumed fully dense at 1.24g/cm3; camera CoG is a uniform-envelope assumption, not measured.'},
        'q_camera_1g_columns_XYZ':q.tolist(),
        'q_definition':'First 3 rows: work-conjugate virtual camera CoG displacement in mm; last 3 rows: rotation in radians. This part contribution only, using the equivalent interface load distributor.',
        'cases':{},
        'region_definitions':({'upper_features_z_band':'All integration points with 13 <= Fz <= 17.9 mm; includes studs and hook/other upper features. This is not the isolated two-stud region.'} if args.part=='cartridge' else {})}
    for j,a in enumerate('XYZ'):
        u=U[:,j]*3
        summary=fe.summarize(u,regions=regions)
        s,_=fe.stress(u)
        eig=np.linalg.eigvalsh(np.moveaxis(s,(0,1),(-2,-1)))
        summary['max_principal_tension_MPa']=float(eig[:,:,-1].max())
        summary['most_compressive_principal_MPa']=float(eig[:,:,0].min())
        summary['p99_positive_principal_MPa']=weighted_quantile(np.maximum(eig[:,:,-1],0),fe.basis.dx,(.99,))[0]
        summary['camera_virtual_translation_mm']=(q[:3,j]*3).tolist()
        summary['camera_virtual_rotation_rad']=(q[3:,j]*3).tolist()
        c=coeff[:,j]*3
        summary['strain_energy_Nmm']=float(.5*c@C@c)
        result['cases'][f'plus{a}_3g']=summary
        if args.level=='fine':fe.export_vtu(out/f'plus{a}_3g.vtu',u)
    # Parametric mass sweep is exact linear superposition for these fixed BCs.
    if args.part=='base':
        sweep=[]
        for mg in [.5,1.,2.,3.]:
            c,o=load_coefficients(args.part,names,mg)
            for a_mult in [1.,3.]:
                for j,a in enumerate('XYZ'):
                    cs=c[:,j]*a_mult;uu=d['displacement']@cs
                    uv=np.vstack([uu[ix] for ix in fe.components])
                    qq=o.T@C@cs
                    sweep.append({'motor_branch_mass_kg':mg,'acceleration_g':a_mult,'axis':a,
                       'max_displacement_mm':float(np.linalg.norm(uv,axis=0).max()),
                       'camera_virtual_translation_mm':qq[:3].tolist(),
                       'strain_energy_Nmm':float(.5*cs@C@cs)})
        result['motor_branch_mass_sweep']=sweep
    np.savez_compressed(out/'service_displacements.npz',unit_g_columns_XYZ=U,
          q_camera_1g=q,coefficients_1g=coeff,observers=obs,case_names=np.array(names))
    write_json(out/'results.json',result)
    print(json.dumps({'part':args.part,'level':args.level,
         '3g':{k:{q:v[q] for q in ['max_displacement_mm','volume_weighted_p99_von_mises_MPa','max_von_mises_MPa','camera_virtual_translation_mm']} for k,v in result['cases'].items()}},indent=2))


if __name__=='__main__':main()