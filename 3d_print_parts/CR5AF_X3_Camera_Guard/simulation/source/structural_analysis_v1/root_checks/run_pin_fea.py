"""Retaining pin elastic bearing/retention sensitivity on frozen native CAD.

The receiver is open above its 2 mm groove floor at the pin location, so the
transverse model supports ONLY the lower 0..2 mm of pin shaft. A fictitious
upper receiver support would be incorrect. Clearance/contact is idealized.
"""
from pathlib import Path
import sys,json,argparse
import numpy as np
from skfem import FacetBasis,LinearForm,asm
from skfem.helpers import dot

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'fem_core'))
from solid_fem import SolidFE,gmsh_mesh,write_json


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--level',choices=['coarse','medium','fine'],required=True);args=ap.parse_args()
    out=ROOT/'pin_fea'/args.level;out.mkdir(parents=True,exist_ok=True)
    source=ROOT/'inputs'/'03_Cartridge_Retaining_Pin.brep';mesh=out/'pin.msh'
    hs={'coarse':1.2,'medium':.85,'fine':.6};curvs={'coarse':16,'medium':22,'fine':30}
    meta=(json.loads(mesh.with_suffix('.mesh.json').read_text()) if mesh.exists() else
       gmsh_mesh(source,mesh,hs[args.level],hmin=hs[args.level]*.35,curvature_segments=curvs[args.level],nthreads=1))
    fe=SolidFE(mesh)
    radial=lambda x:np.sqrt((x[0]+10)**2+(x[1]-47)**2)
    support=fe.facets(lambda x:(x[2]>0)&(x[2]<2.00001)&(abs(radial(x)-2)<.08))
    regions={'split_root':lambda x:(x[2]>=2.5)&(x[2]<=3.3),
        'shaft_in_floor':lambda x:(x[2]>=0)&(x[2]<=2),
        'split_legs':lambda x:(x[2]>=2.8)&(x[2]<=6.9),
        'retention_barbs':lambda x:x[2]>=6.9}
    result={'part':'03_Cartridge_Retaining_Pin','mesh_level':args.level,'mesh':meta,'mesh_info':fe.mesh_info,
       'material':{'E_MPa':2000.,'nu':.35,'density_tonne_mm3':1.24e-9,'status':'assumed homogeneous fully dense isotropic print'},
       'scope':'Unit elastic bearing/retention submodels only. 0.4mm diametral clearance, unilateral contact, friction, insertion snap action, fracture and extraction force are not solved.',
       'cases':{}}
    fe.assemble(support,solver='direct')
    for j,a in enumerate('XY'):
        patch=fe.facets(lambda x,j=j:(x[2]>=2.3)&(x[2]<=6.7)&(abs(radial(x)-2)<.08)&(x[j]<[-10,47][j]))
        f,info=fe.load_wrench(patch,F=np.eye(3)[j],about=(-10,47,4.5))
        u,sol=fe.solve(f);summary=fe.summarize(u,regions=regions)
        result['cases'][f'transverse_{a}_1N']={'BC':'Lower shaft cylindrical surface 0<z<2mm fixed; no upper receiver support',
            'support':fe.boundary_info(support),'load':info,'solution':sol,'response':summary}
        if args.level=='fine':fe.export_vtu(out/f'transverse_{a}_1N.vtu',u)
    # An independent axial pull model: head bears on the underside of base.
    axial_support=fe.facets(lambda x:np.isclose(x[2],0,atol=1e-7)&(radial(x)>2.2))
    barb=fe.facets(lambda x:np.isclose(x[2],6.9,atol=1e-7))
    fe.assemble(axial_support,solver='direct')
    # The nominal hole rim r=2.2 is not a native CAD face edge. Selecting whole
    # triangles by their centroids changes the loaded area with the mesh. Use
    # a high-order facet-quadrature mask instead, then equilibrate force/moment.
    fb=FacetBasis(fe.mesh,fe.element,facets=barb,intorder=12)
    x=np.asarray(fb.global_coordinates());mask=radial(x)>2.2;w=fb.dx*mask
    A=w.sum();c=np.sum(x*w[None,:,:],axis=(1,2))/A;rr=x-c[:,None,None]
    R2=np.einsum('ieq,jeq,eq->ij',rr,rr,w);J=np.trace(R2)*np.eye(3)-R2
    F=np.array([0.,0.,1.]);about=np.array([-10.,47.,6.9]);coef=np.linalg.solve(J,-np.cross(c-about,F))
    @LinearForm
    def band_load(v,ww):
        band=radial(ww.x)>2.2;r=ww.x-c[:,None,None]
        tr=(F[:,None,None]/A+np.cross(coef,r,axisa=0,axisb=0,axisc=0))*band[None,:,:]
        return dot(v,tr)
    f=asm(band_load,fb)
    aa=.4;ro=2.35;ri=2.2
    strip=lambda r:2*(aa*np.sqrt(r*r-aa*aa)+r*r*np.arcsin(aa/r))
    exact=np.pi*(ro*ro-ri*ri)-(strip(ro)-strip(ri))
    info={'area_mm2':float(A),'analytical_band_area_mm2':float(exact),
      'relative_quadrature_area_error':float(A/exact-1),'facet_quadrature_order':12,
      'load_selection':'Quadrature-point band r>2.2 on the actual z6.9 annular shoulder; avoids triangle-centroid selection area drift.',
      'assembled_wrench':fe.resultant(f,about),'target_force_N':F.tolist(),
      'target_moment_Nmm':[0,0,0],'moment_reference_mm':about.tolist()}
    u,sol=fe.solve(f)
    result['cases']['barb_retention_plusZ_1N']={'BC':'Head upper annulus at z0 fixed, shaft otherwise free; barb lower annulus outside nominal hole radius2.2 receives +Z unit force',
       'support':fe.boundary_info(axial_support),'load':info,'solution':sol,'response':fe.summarize(u,regions=regions)}
    if args.level=='fine':fe.export_vtu(out/'barb_retention_plusZ_1N.vtu',u)
    result['complete']=True;write_json(out/'results.json',result)
    print(json.dumps({'level':args.level,'mesh':fe.mesh_info,'results':{k:{s:v['response'][s] for s in ['max_displacement_mm','volume_weighted_p99_von_mises_MPa','max_von_mises_MPa']} for k,v in result['cases'].items()}},indent=2),flush=True)


if __name__=='__main__':main()