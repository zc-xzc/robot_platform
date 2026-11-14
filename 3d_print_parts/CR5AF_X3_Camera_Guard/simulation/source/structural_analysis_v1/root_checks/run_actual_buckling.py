"""Ideal linear initial-stress eigenbuckling of frozen V2R1 parts.

This excludes gaps, contact opening, material failure and imperfections.
The eigenfactor is not a certified failure or camera release force.
"""
from pathlib import Path
import argparse,sys,json,time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'fem_core'))
from solid_fem import SolidFE,write_json
from validate_beam_and_buckling import buckling_modes


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--part',choices=['base','cartridge'],required=True)
    parser.add_argument('--level',choices=['coarse','medium'],default='coarse')
    args=parser.parse_args()
    inp=ROOT/'mount_fea'/args.part/args.level
    out=ROOT/'root_checks'/'buckling'/args.part/args.level
    out.mkdir(parents=True,exist_ok=True)
    data=np.load(inp/'unit_response_fields.npz')
    source=json.loads((inp/'results.json').read_text())
    if not source.get('complete'):raise RuntimeError('Static source incomplete')
    names=list(data['case_names']);U=data['displacement']
    def unit(name):return U[:,names.index(name)]
    mass=.21570000317074017;g=9.80665
    if args.part=='cartridge':
        u=-3*(mass*g*unit('cameraPlusFrameCoG_FZ_1N')+unit('self_weight_Z_1g'))
        load_desc='3g effective acceleration along flange -Z: 180g camera plus 35.700g frame at their combined assumed CoG, with cartridge body force'
    else:
        # Motor/gripper/workpiece branch: hypothetical 2kg at z=100mm.
        # Camera branch acts through the actual dovetail receiver reference.
        masses=json.loads((ROOT/'root_checks'/'source_mass_geometry.json').read_text())
        entries=[(.18,np.array([0,112.3050865276,25.3050865276]))]
        for k in ['04_Camera_Module_Short45_F','02_Fuse_Cartridge_D8','03_Cartridge_Retaining_Pin']:
            e=masses[k];entries.append((e['mass_kg_at_1p24g_cm3'],np.array(e['center_mm'])))
        m=sum(e[0] for e in entries);cg=sum(mm*x for mm,x in entries)/m
        F=np.array([0,0,-3*m*g]);M=np.cross(cg-np.array([0,47,6.7]),F)
        u=-3*(2*g*unit('motor_FZ_1N_at_z100')+unit('self_weight_Z_1g'))
        for j,a in enumerate('XYZ'):
            u=u+F[j]*unit(f'receiver_F{a}_1N')+M[j]*unit(f'receiver_M{a}_1Nmm')
        load_desc='3g effective acceleration along flange -Z: assumed 2kg motor/gripper/workpiece at z100 plus camera/frame/cartridge/pin and base body force'
    mesh=inp/(source['part']+'.msh')
    fe=SolidFE(mesh);fe.fixed=data['fixed_dofs'];fe.free=data['free_dofs']
    fe.assemble_stiffness()
    print('START',args.part,args.level,fe.mesh_info,flush=True)
    t=time.monotonic();factors,modes,residual=buckling_modes(fe,u,k=4)
    mode=modes[:,0];amp=np.sqrt(sum(mode[ix]**2 for ix in fe.components));j=int(np.argmax(amp))
    result={'part':args.part,'mesh_level':args.level,'source_sha256':source['source_sha256'],
        'reference_load':load_desc,'mesh_info':fe.mesh_info,
        'support':source['assumptions']['fixed_support'],
        'formulation':'K phi = -lambda Kg(sigma_reference) phi; small-strain isotropic elastic initial-stress eigenproblem',
        'reference_static_response':fe.summarize(u),
        'positive_eigenfactors':factors.tolist(),'eigen_residuals':residual,
        'first_mode_max_amplitude_position_mm':fe.basis.doflocs[:,fe.components[0][j]].tolist(),
        'reference_acceleration_g':3.,'equivalent_acceleration_g_formal_only':(3*factors).tolist(),
        'elapsed_seconds':time.monotonic()-t,
        'interpretation':'Ideal elastic stability screen only. Extrapolation to this factor can exceed material linear range and alter contact. It is not a strength margin, rated acceleration, collision force or release threshold.'}
    write_json(out/'results.json',result)
    np.savez_compressed(out/'eigenvectors.npz',eigenfactors=factors,modes=modes,reference_displacement=u)
    fe.export_vtu(out/'reference_3g_minusZ.vtu',u)
    fe.export_vtu(out/'mode1_unit_display.vtu',mode/amp[j])
    print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':main()