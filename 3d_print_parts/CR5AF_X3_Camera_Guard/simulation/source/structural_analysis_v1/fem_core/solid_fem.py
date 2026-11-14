"""Reproducible small-strain 3-D elasticity in N, mm, MPa, tonne, second.

Gmsh OCC imports the unmodified source BREP. 10-node curved tetrahedral
geometry and quadratic displacement are used throughout. This module does
not infer print strength, contacts, fracture, impact or release force.
"""
from __future__ import annotations
import hashlib, json, time, gc, resource
from pathlib import Path
import numpy as np
import scipy.sparse as sp
from scipy.sparse.linalg import cg, splu, LinearOperator
import gmsh, meshio, pyamg
from skfem import Basis, FacetBasis, ElementVector, ElementTetP2, BilinearForm, LinearForm, asm
from skfem.io import from_meshio
from skfem.helpers import dot
from skfem.models.elasticity import linear_elasticity


def sha256(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
    return h.hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value,indent=2,allow_nan=False),encoding='utf-8')


def gmsh_mesh(cad_path, output_path, hmax, hmin=None, curvature_segments=None,
              refinement_boxes=(), nthreads=2):
    """Mesh source BREP; boxes=(xmin,ymin,zmin,xmax,ymax,zmax,size).

    Mesh is saved as MSH 2.2, retaining the original CAD coordinates in mm.
    Mid-edge nodes project to CAD, i.e. this is not a faceted STL remesh.
    """
    t0=time.monotonic(); output_path=Path(output_path);output_path.parent.mkdir(parents=True,exist_ok=True)
    gmsh.initialize()
    try:
        gmsh.option.setNumber('General.Terminal',0)
        gmsh.option.setNumber('General.NumThreads',nthreads)
        gmsh.model.add(output_path.stem)
        gmsh.model.occ.importShapes(str(cad_path));gmsh.model.occ.synchronize()
        volumes=gmsh.model.getEntities(3)
        if len(volumes)!=1: raise ValueError(f'Expected one CAD solid, got {volumes}')
        cad_volume=sum(gmsh.model.occ.getMass(*v) for v in volumes)
        gmsh.option.setNumber('Mesh.MeshSizeMax',hmax)
        gmsh.option.setNumber('Mesh.MeshSizeMin',hmin or hmax*.25)
        gmsh.option.setNumber('Mesh.MeshSizeFromCurvature',curvature_segments or max(12,round(48/hmax)))
        gmsh.option.setNumber('Mesh.MeshSizeExtendFromBoundary',1)
        gmsh.option.setNumber('Mesh.Algorithm',6)
        gmsh.option.setNumber('Mesh.Algorithm3D',10)
        gmsh.option.setNumber('Mesh.Optimize',1)
        gmsh.option.setNumber('Mesh.OptimizeNetgen',0)
        fields=[]
        for box in refinement_boxes:
            tag=gmsh.model.mesh.field.add('Box');fields.append(tag)
            for k,v in zip(('XMin','YMin','ZMin','XMax','YMax','ZMax','VIn'),box):
                gmsh.model.mesh.field.setNumber(tag,k,v)
            gmsh.model.mesh.field.setNumber(tag,'VOut',hmax)
            gmsh.model.mesh.field.setNumber(tag,'Thickness',hmax)
        if fields:
            tag=gmsh.model.mesh.field.add('Min');gmsh.model.mesh.field.setNumbers(tag,'FieldsList',fields)
            gmsh.model.mesh.field.setAsBackgroundMesh(tag)
        gmsh.model.mesh.generate(3)
        gmsh.model.mesh.setOrder(2)
        gmsh.option.setNumber('Mesh.HighOrderOptimize',1)
        gmsh.model.mesh.optimize('HighOrder')
        gmsh.option.setNumber('Mesh.MshFileVersion',2.2)
        gmsh.option.setNumber('Mesh.Binary',1)
        gmsh.write(str(output_path))
        etypes,etags,_=gmsh.model.mesh.getElements(3)
        counts={str(int(k)):len(v) for k,v in zip(etypes,etags)}
        result=dict(source=str(cad_path),source_sha256=sha256(cad_path),mesh=str(output_path),
                    mesh_sha256=sha256(output_path),hmax_mm=hmax,hmin_mm=hmin or hmax*.25,
                    curvature_segments=curvature_segments or max(12,round(48/hmax)),
                    refinement_boxes=list(refinement_boxes),cad_volume_mm3=cad_volume,
                    element_type_counts=counts,meshing_seconds=time.monotonic()-t0)
        write_json(output_path.with_suffix('.mesh.json'),result)
        return result
    finally: gmsh.finalize()


def weighted_quantile(values,weights,qs=(.95,.99)):
    values=np.asarray(values).ravel();weights=np.asarray(weights).ravel()
    order=np.argsort(values);v=values[order];w=weights[order]
    c=np.cumsum(w);c/=c[-1]
    return np.interp(qs,c,v).tolist()


class SolidFE:
    def __init__(self,mesh_path,E=2000.,nu=.35,density=1.24e-9,intorder=5):
        self.mesh_path=str(mesh_path);self.E=E;self.nu=nu;self.density=density
        self.lam=E*nu/((1+nu)*(1-2*nu));self.mu=E/(2*(1+nu))
        self.mesh=from_meshio(meshio.read(str(mesh_path)))
        if self.mesh.__class__.__name__!='MeshTet2': raise ValueError('Need 10-node curved tetrahedra')
        self.element=ElementVector(ElementTetP2())
        self.basis=Basis(self.mesh,self.element,intorder=intorder)
        self.intorder=intorder;self.components=self.basis.split_indices()
        self.boundary_facets=self.mesh.boundary_facets()
        self.facet_centres=self.mesh.p[:,self.mesh.facets[:,self.boundary_facets]].mean(axis=1)
        self.K=None;self.A=None;self.M=None;self._solver=None
        det=self.basis.mapping.detDF(self.basis.X)
        folds=int(np.sum(np.min(det,axis=1)*np.max(det,axis=1)<=0))
        if folds: raise ValueError(f'{folds} folded/inverted curved tetrahedra')
        if np.min(self.basis.dx)<=0: raise ValueError('Expected positive quadrature weights')
        self.mesh_info=dict(tetrahedra=int(self.mesh.nelements),geometry_nodes=int(self.mesh.p.shape[1]),
                            displacement_dofs=int(self.basis.N),integration_order=intorder,
                            integration_points=int(self.basis.X.shape[1]),volume_mm3=float(self.basis.dx.sum()),
                            folded_elements=folds,min_abs_mapping_determinant=float(np.min(abs(det))),
                            E_MPa=E,nu=nu,density_tonne_per_mm3=density,
                            mass_kg=float(self.basis.dx.sum()*density*1000),
                            length_unit='mm',force_unit='N',stress_unit='MPa=N/mm2',time_unit='s')

    def facets(self,predicate):
        ids=self.boundary_facets[np.asarray(predicate(self.facet_centres),dtype=bool)]
        if not len(ids): raise ValueError('Empty boundary selection')
        return ids

    def facet_basis(self,facets):
        return FacetBasis(self.mesh,self.element,facets=facets,intorder=self.intorder)

    def boundary_info(self,facets):
        b=self.facet_basis(facets);area=float(b.dx.sum());x=np.asarray(b.global_coordinates())
        center=np.sum(x*b.dx[None,:,:],axis=(1,2))/area
        return dict(facets=int(len(facets)),area_mm2=area,centroid_mm=center.tolist(),
                    constrained_dofs=int(len(self.basis.get_dofs(facets=facets).all())))

    def assemble_stiffness(self,chunk_size=8000):
        t0=time.monotonic()
        form=linear_elasticity(self.lam,self.mu)
        if self.mesh.nelements<=chunk_size:
            self.K=asm(form,self.basis).tocsr()
        else:
            # The full 30x30xnelements COO temporary can exceed available
            # memory. Integrating disjoint element blocks is algebraically
            # identical; contributions are accumulated on the same DOFs.
            self.K=sp.csr_matrix((self.basis.N,self.basis.N))
            for start in range(0,self.mesh.nelements,chunk_size):
                stop=min(self.mesh.nelements,start+chunk_size)
                local=Basis(self.mesh,self.element,elements=np.arange(start,stop),intorder=self.intorder)
                partial=asm(form,local).tocsr()
                self.K=(self.K+partial).tocsr()
                del local,partial;gc.collect()
                print('K_CHUNK',stop,self.mesh.nelements,flush=True)
        self.K.eliminate_zeros()
        self.assembly_seconds=time.monotonic()-t0
        self.assembly_peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        return self.K

    def assemble(self,fixed_facets=None,solver='amg',fixed_dofs=None):
        t0=time.monotonic()
        if fixed_dofs is None:
            fixed_dofs=self.basis.get_dofs(facets=fixed_facets).all()
        self.fixed=np.unique(fixed_dofs)
        self.free=np.setdiff1d(np.arange(self.basis.N),self.fixed)
        if self.K is None: self.assemble_stiffness()
        self.A=self.K[self.free][:,self.free].tocsr()
        # Keep xyz at each physical node as one elasticity block. Treating
        # three components as unrelated scalar aggregates badly degrades AMG.
        if len(self.free)%3==0 and np.all(np.diff(self.free.reshape(-1,3),axis=1)==1):
            self.A=self.A.tobsr(blocksize=(3,3))
        self.assembly_seconds=time.monotonic()-t0
        t0=time.monotonic()
        if solver=='direct':
            self._solver=splu(self.A.tocsc());self.preconditioner=None
        else:
            # Six exact rigid modes are the elasticity AMG near-nullspace.
            xyz=self.basis.doflocs;center=xyz.mean(axis=1);B=np.zeros((self.basis.N,6))
            for j,idx in enumerate(self.components):
                B[idx,j]=1
                for k in range(3):
                    ek=np.zeros(3);ek[k]=1
                    B[idx,3+k]=np.cross(ek, (xyz[:,idx]-center[:,None]).T)[:,j]
            B=B[self.free];B/=np.maximum(np.linalg.norm(B,axis=0),1e-30)
            self._solver=pyamg.smoothed_aggregation_solver(self.A,B=B,symmetry='symmetric',
                strength=('symmetric',{'theta':0.0}),smooth=('energy',{'degree':2}),
                presmoother=('block_gauss_seidel',{'sweep':'symmetric'}),
                postsmoother=('block_gauss_seidel',{'sweep':'symmetric'}),max_coarse=150)
            self.preconditioner=self._solver.aspreconditioner()
        self.solver_kind=solver;self.preconditioner_seconds=time.monotonic()-t0
        return dict(assembly_seconds=self.assembly_seconds,preconditioner_seconds=self.preconditioner_seconds,
                    free_dofs=int(len(self.free)),fixed_dofs=int(len(self.fixed)),K_nnz=int(self.K.nnz))

    def load_wrench(self,facets,F=(0,0,0),M=(0,0,0),about=(0,0,0)):
        """Consistent surface traction with exact force F and moment M about `about`.

        Traction is F/A + b cross (x-centroid), b=J^-1(M-(centroid-about)xF).
        This transmits a remote wrench without rigidly tying the loaded surface.
        """
        fb=self.facet_basis(facets);x=np.asarray(fb.global_coordinates());w=fb.dx
        A=w.sum();c=np.sum(x*w[None,:,:],axis=(1,2))/A;r=x-c[:,None,None]
        R=np.einsum('ieq,jeq,eq->ij',r,r,w);J=np.trace(R)*np.eye(3)-R
        F=np.asarray(F,float);M=np.asarray(M,float);about=np.asarray(about,float)
        b=np.linalg.solve(J,M-np.cross(c-about,F))
        @LinearForm
        def traction(v,ww):
            rr=ww.x-c[:,None,None]
            t=F[:,None,None]/A+np.cross(b,rr,axisa=0,axisb=0,axisc=0)
            return dot(v,t)
        f=asm(traction,fb)
        info=dict(area_mm2=float(A),centroid_mm=c.tolist(),target_force_N=F.tolist(),
                  target_moment_Nmm=M.tolist(),moment_reference_mm=about.tolist(),
                  traction_constant_N_per_mm2=(F/A).tolist(),traction_rotation_coefficient=b.tolist(),
                  assembled_wrench=self.resultant(f,about))
        return f,info

    def load_force_at(self,facets,F,point):
        F=np.asarray(F,float);point=np.asarray(point,float)
        return self.load_wrench(facets,F,np.cross(point,F),(0,0,0))

    def body_force(self,acceleration_mm_s2):
        force=self.density*np.asarray(acceleration_mm_s2,float)
        @LinearForm
        def rhs(v,w): return dot(v,force[:,None,None])
        return asm(rhs,self.basis)

    def resultant(self,vector,about=(0,0,0)):
        F=np.array([vector[ix].sum() for ix in self.components]);M=np.zeros(3)
        for j,ix in enumerate(self.components):
            e=np.zeros(3);e[j]=1
            M+=np.sum(np.cross((self.basis.doflocs[:,ix]-np.asarray(about)[:,None]).T,e)*vector[ix,None],axis=0)
        return dict(force_N=F.tolist(),moment_Nmm=M.tolist())

    def solve(self,f,rtol=1e-9,maxiter=1000):
        t0=time.monotonic();rhs=np.asarray(f)[self.free];counter=[0]
        def cb(x):counter[0]+=1
        if self.solver_kind=='direct':sol=self._solver.solve(rhs);code=0
        else:sol,code=cg(self.A,rhs,M=self.preconditioner,rtol=rtol,atol=0,maxiter=maxiter,callback=cb)
        rel=float(np.linalg.norm(self.A@sol-rhs)/max(np.linalg.norm(rhs),1e-30))
        if code or rel>rtol*10: raise RuntimeError(f'Solver did not converge: info={code}, residual={rel}')
        u=np.zeros(self.basis.N);u[self.free]=sol
        residual=self.K@u-f;reaction=np.zeros_like(f);reaction[self.fixed]=residual[self.fixed]
        load=self.resultant(f);react=self.resultant(reaction)
        info=dict(solve_seconds=time.monotonic()-t0,iterations=counter[0],relative_free_residual=rel,
                  strain_energy_Nmm=float(.5*np.dot(u,self.K@u)),compliance_Nmm=float(np.dot(f,u)),
                  applied_wrench=load,support_reaction_wrench=react,
                  force_balance_error_N=(np.array(load['force_N'])+react['force_N']).tolist(),
                  moment_balance_error_Nmm=(np.array(load['moment_Nmm'])+react['moment_Nmm']).tolist())
        return u,info

    def evaluate_solution(self,f,u,method='linear superposition'):
        residual=self.K@u-f;reaction=np.zeros_like(f);reaction[self.fixed]=residual[self.fixed]
        load=self.resultant(f);react=self.resultant(reaction)
        return dict(solution_method=method,solve_seconds=0,iterations=0,
            relative_free_residual=float(np.linalg.norm(residual[self.free])/max(np.linalg.norm(f[self.free]),1e-30)),
            strain_energy_Nmm=float(.5*np.dot(u,self.K@u)),compliance_Nmm=float(np.dot(f,u)),
            applied_wrench=load,support_reaction_wrench=react,
            force_balance_error_N=(np.array(load['force_N'])+react['force_N']).tolist(),
            moment_balance_error_Nmm=(np.array(load['moment_Nmm'])+react['moment_Nmm']).tolist())

    def stress(self,u):
        g=self.basis.interpolate(u).grad;e=.5*(g+g.swapaxes(0,1))
        s=2*self.mu*e+self.lam*np.eye(3)[:,:,None,None]*np.einsum('iieq->eq',e)[None,None,:,:]
        vm=np.sqrt(.5*((s[0,0]-s[1,1])**2+(s[1,1]-s[2,2])**2+(s[2,2]-s[0,0])**2)
                   +3*(s[0,1]**2+s[1,2]**2+s[2,0]**2))
        return s,vm

    def summarize(self,u,regions=None,return_fields=False):
        s,vm=self.stress(u);dx=self.basis.dx;vol=dx.sum(axis=1)
        U=np.vstack([u[ix] for ix in self.components]);amp=np.linalg.norm(U,axis=0)
        p95,p99=weighted_quantile(vm,dx)
        imax=np.unravel_index(np.argmax(vm),vm.shape);x=np.asarray(self.basis.global_coordinates())
        info=dict(max_displacement_mm=float(amp.max()),max_von_mises_MPa=float(vm.max()),
                  volume_weighted_p95_von_mises_MPa=p95,volume_weighted_p99_von_mises_MPa=p99,
                  mean_von_mises_MPa=float(np.sum(vm*dx)/dx.sum()),
                  max_stress_location_mm=x[:,imax[0],imax[1]].tolist(),
                  raw_peak_warning='Raw integration-point peaks near sharp/reentrant corners and ideal clamps may be singular; no failure or release force is inferred.')
        if regions:
            out={}
            for name,predicate in regions.items():
                mask=np.asarray(predicate(x),bool)
                if not np.any(mask):continue
                q95,q99=weighted_quantile(vm[mask],dx[mask])
                out[name]=dict(volume_mm3=float(dx[mask].sum()),max_von_mises_MPa=float(vm[mask].max()),p95_MPa=q95,p99_MPa=q99)
            info['regions']=out
        if return_fields:
            stress6=np.stack([s[0,0],s[1,1],s[2,2],s[0,1],s[1,2],s[2,0]])
            fields=dict(displacement=u,stress6_cell_mean=np.sum(stress6*dx[None,:,:],axis=2)/vol[None,:],
                        von_mises_cell_mean=np.sum(vm*dx,axis=1)/vol,
                        cell_volume=vol,cell_centres=np.sum(x*dx[None,:,:],axis=2)/vol[None,:])
            return info,fields
        return info

    def mass_matrix(self,chunk_size=8000):
        if self.M is None:
            rho=self.density
            @BilinearForm
            def mass(u,v,w):return rho*dot(u,v)
            if self.mesh.nelements<=chunk_size:
                self.M=asm(mass,self.basis).tocsr()
            else:
                self.M=sp.csr_matrix((self.basis.N,self.basis.N))
                for start in range(0,self.mesh.nelements,chunk_size):
                    stop=min(self.mesh.nelements,start+chunk_size)
                    local=Basis(self.mesh,self.element,elements=np.arange(start,stop),intorder=self.intorder)
                    partial=asm(mass,local).tocsr()
                    self.M=(self.M+partial).tocsr()
                    del local,partial;gc.collect()
                    print('M_CHUNK',stop,self.mesh.nelements,flush=True)
            self.M.eliminate_zeros()
        return self.M

    def export_vtu(self,path,u):
        """VTK 10-node tetra output. Uses meshio's standard node ordering."""
        original=meshio.read(self.mesh_path)
        tet=original.cells_dict['tetra10'];coords=original.points
        # Imported MeshTet2 can renumber edge nodes; evaluate by matching coordinates.
        from scipy.spatial import cKDTree
        ux=np.vstack([u[ix] for ix in self.components]).T
        pts=self.basis.doflocs[:,self.components[0]].T
        d,ix=cKDTree(pts).query(coords[:,:3])
        if np.max(d)>1e-7:raise ValueError('VTK displacement node mismatch')
        info,fields=self.summarize(u,return_fields=True)
        meshio.write(str(path),meshio.Mesh(coords,[('tetra10',tet)],point_data={'displacement_mm':ux[ix]},
            cell_data={'von_mises_MPa':[fields['von_mises_cell_mean']],
                       'stress_XX_YY_ZZ_XY_YZ_ZX_MPa':[fields['stress6_cell_mean'].T]}))
        return info
