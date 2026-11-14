"""Read-only displacement/actual-camera-solid compatibility screen.

No FEM matrix loading, assembly or solve. Reads Gmsh boundary nodes and the
already solved displacement fields; checks original camera BReps by OCCT.
This is a node screen, not a continuous surface collision/contact solution.
"""
from pathlib import Path
import argparse,csv,hashlib,json,struct,time,math
import numpy as np
from scipy.spatial import cKDTree
import cadquery as cq
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_IN,TopAbs_ON
from OCP.BRepExtrema import BRepExtrema_DistShapeShape

HERE=Path(__file__).resolve().parent;ANA=HERE.parent;ROOT=ANA.parent
TOL=1e-6

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def read_gmsh22_boundary(path):
    """Read only binary Gmsh 2.2 nodes and triangle6 surface connectivity.

    Source gmsh_mesh() writes one volume and its original CAD boundary faces;
    no stiffness, basis, quadrature or solve object is constructed here.
    """
    nodes=None;node_ids=None;surfaces=[];types={};endian='<'
    nnodes={15:1,1:2,8:3,2:3,9:6,4:4,11:10}
    with path.open('rb') as f:
        while True:
            line=f.readline()
            if not line:break
            tag=line.strip()
            if tag==b'$MeshFormat':
                fmt=f.readline().split();assert fmt==[b'2.2',b'1',b'8'],fmt
                marker=f.read(4)
                if struct.unpack('<i',marker)[0]!=1:endian='>'
                assert struct.unpack(endian+'i',marker)[0]==1
            elif tag==b'$Nodes':
                n=int(f.readline());dtype=np.dtype([('id',endian+'i4'),('xyz',endian+'f8',(3,))])
                a=np.fromfile(f,dtype=dtype,count=n);assert len(a)==n
                node_ids=a['id'].copy();nodes=a['xyz'].copy()
            elif tag==b'$Elements':
                total=int(f.readline());seen=0
                while seen<total:
                    etype,count,ntags=struct.unpack(endian+'3i',f.read(12))
                    if etype not in nnodes:raise ValueError(('Unsupported source element type',etype))
                    width=1+ntags+nnodes[etype]
                    block=np.fromfile(f,dtype=endian+'i4',count=count*width).reshape(count,width)
                    types[str(etype)]=types.get(str(etype),0)+count
                    if etype in [2,9]:surfaces.append(block[:,-nnodes[etype]:].copy())
                    seen+=count
            elif tag.startswith(b'$') and tag not in [b'$EndMeshFormat',b'$EndNodes',b'$EndElements']:
                raise ValueError(('Unexpected Gmsh section',tag))
    assert nodes is not None and surfaces
    boundary_tags=np.unique(np.concatenate([v.ravel() for v in surfaces]))
    lookup={int(t):i for i,t in enumerate(node_ids)}
    indices=np.array([lookup[int(t)] for t in boundary_tags])
    return nodes[indices],boundary_tags,{'node_count':len(nodes),'boundary_node_count':len(indices),
        'element_type_counts':types,'boundary_surface_elements':sum(len(v) for v in surfaces)}

def skew(t):
    x,y,z=t;return np.array([[0,-z,y],[z,0,-x],[-y,x,0]],float)

def classify(points,solids):
    # Union membership: IN takes priority over ON because the reference solids
    # contain known author-internal overlaps (especially lens/body).
    state=np.zeros(len(points),dtype=np.int8);inside_bits=np.zeros(len(points),dtype=np.uint16)
    on_bits=np.zeros(len(points),dtype=np.uint16);candidates={}
    for i,(name,s,bb,cl) in enumerate(solids):
        ids=np.flatnonzero(np.all((points>=bb[0]-TOL)&(points<=bb[1]+TOL),axis=1))
        candidates[name]=len(ids)
        for k in ids:
            p=points[k];cl.Perform(gp_Pnt(float(p[0]),float(p[1]),float(p[2])),TOL)
            answer=cl.State()
            if answer==TopAbs_IN:state[k]=2;inside_bits[k]|=1<<i
            elif answer==TopAbs_ON:
                if state[k]==0:state[k]=1
                on_bits[k]|=1<<i
    return state,inside_bits,on_bits,candidates

def region_counts(mask,regions):
    return {name:int(np.sum(mask&region)) for name,region in regions.items()}

def point_boundary_distance(p,shape):
    vertex=cq.Vertex.makeVertex(*[float(v) for v in p])
    shells=cq.Compound.makeCompound(shape.Shells())
    query=BRepExtrema_DistShapeShape(vertex.wrapped,shells.wrapped);query.Perform()
    if not query.IsDone():return None
    return float(query.Value())

def nodal_clearance_bracket(points,mask,state,solids):
    """Bracket the minimum nodal clearance without pretending to solve contact."""
    ids=np.flatnonzero(mask)
    if len(ids)==0:return {'node_count':0}
    if np.any(state[ids]>0):
        return {'node_count':len(ids),'nonseparated_node_count':int(np.sum(state[ids]>0)),
                'positive_clearance_not_established':True}
    samples=[];lower=math.inf
    for name,shape,bb,_ in solids:
        delta=np.maximum(np.maximum(bb[0]-points[ids],points[ids]-bb[1]),0)
        bound=np.linalg.norm(delta,axis=1);j=int(np.argmin(bound));node=int(ids[j])
        lower=min(lower,float(bound[j]))
        distance=point_boundary_distance(points[node],shape)
        samples.append({'solid':name,'boundary_row':node,'AABB_lower_bound_mm':float(bound[j]),
                        'exact_distance_at_evaluated_node_mm':distance})
    upper=min(v['exact_distance_at_evaluated_node_mm'] for v in samples
              if v['exact_distance_at_evaluated_node_mm'] is not None)
    return {'node_count':len(ids),'minimum_sampled_node_clearance_lower_bound_mm':lower,
      'minimum_sampled_node_clearance_upper_bound_mm':upper,'bracket_width_mm':upper-lower,
      'evaluated_exact_distance_samples':samples,
      'scope':'Bounds apply to minimum over these boundary nodes against union of eight actual source solids; not continuous surface clearance.'}

def write_brief(report,path):
    lines=[f'# 相机接触兼容性节点筛查（{report["level"]}）','',
      '只读取已有位移、G 和网格边界节点，没有加载 K、装配或求解。相机采用原作者 8 个实际实体；AABB 只作排除和间隙下界，不把矩形包络侵入当作实际碰撞。','',
      f'原始 {report["mesh"]["boundary_node_count"]:,} 个边界节点：相机内部 {report["original"]["IN_nodes"]} 个，边界接触 {report["original"]["ON_nodes"]} 个；其中原始 Cz=0 底平面接触 {report["original"]["ON_by_region"]["original_bottom_plane_absCz_le1e_5"]} 个，Cz>10 接触 {report["original"]["ON_by_region"]["guard_above_Cz10"]} 个。','',
      '| 总有效加速度 | 新进入实体节点 | 原先不接触的节点 | Cz>10 新侵入 | 样本侵入深度最大值 mm |',
      '|---|---:|---:|---:|---:|']
    for c in report['cases']:
        lines.append(f'| {c["case"]} | {c["new_IN_nodes"]} | {c["new_IN_from_original_OUT"]} | {c["new_IN_by_original_region"]["guard_above_Cz10"]} | {c["max_sampled_inside_depth_mm"]:.6f} |')
    all_bottom=all(c['new_IN_nodes']==c['new_IN_by_original_region']['original_bottom_plane_absCz_le1e_5'] for c in report['cases'])
    lines+=['',('本次所有新增侵入节点均来自原始 Cz=0 底平面。' if all_bottom else '本次有新增侵入节点来自原始底平面以外，须进一步查对应位置。'),'',
      '所有计数以原始框体节点所在区域分类，避免节点移动跨区后漏报。深度是已核实选定样本的点到实际实体边界距离，不是全局最大侵入深度。','',
      '**解释：** 如果新增点只在底平面，说明等效牵引/G 映射没有强制刚性相机与托板满足单边非穿透；不能把对应底部局部应力当作已解决接触后的真实值。这与护柱实际碰到相机是不同问题。护柱节点没有侵入，不等于连续表面、打印公差和全部运动方向均已验证。','',
      '相机运动使用题设的小转角仿射场，并精确反解 I+[theta]x。3g 下应同时查看旋转量及与一阶相对位移的差值；没有用有限转角或非线性接触修正原解。','',
      '各区域的节点最小间隙区间在 JSON 的 *_clearance_bracket 中：下界来自保守 AABB 距离，上界来自少量实际 BRep 精确点距。区间仅适用于所检查节点。',
      '相机是社区作者参考模型，其镜头总深度与官方外包尺寸有差别，不能把这组节点间隙当作真机制造公差或打印试装余量。']
    path.write_text('\n'.join(lines)+'\n')

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--level',choices=['coarse','medium','fine'],default='medium')
    args=ap.parse_args();level=args.level;t0=time.monotonic()
    out=HERE/'contact_compatibility'/level;out.mkdir(parents=True,exist_ok=True)
    field=ANA/'camera_fea'/level/'static_fields.npz'
    mesh=ANA/'camera_fea/meshes'/f'camera_{level}.msh'
    manifest_path=ROOT/'recovery_v2/camera/camera_reference_manifest.json'
    source_step=ROOT/'recovery_v2/camera/reference/Insta360_X3_OngXeno_v6.step'
    manifest=json.loads(manifest_path.read_text());assert sha(source_step)==manifest['source_sha256']
    params=json.loads((ANA/'camera_fea/geometry_BC_load_definitions.json').read_text())
    T=np.asarray(params['camera_C_to_F']);R=T[:3,:3];offset=T[:3,3];COM=np.asarray(params['camera_CoG_F_mm'])
    saved=np.load(field);U=saved['U_1g'];G=saved['G'];loc=saved['doflocs']
    component=[saved['component_'+a] for a in 'xyz'];positions=loc[:,component[0]].T
    assert max(np.max(np.abs(positions-loc[:,ix].T)) for ix in component)<1e-10
    boundary,gmsh_ids,mesh_info=read_gmsh22_boundary(mesh)
    errors,scalar_node_index=cKDTree(positions).query(boundary)
    assert float(errors.max())<1e-8 and len(np.unique(scalar_node_index))==len(scalar_node_index)
    displacement=np.stack([U[ix[scalar_node_index]] for ix in component],axis=1) # node, xyz, load-axis
    Q=G.T@U
    originalC=(boundary-offset)@R
    # Regions use the undeformed frame point in camera C coordinates; no
    # changing region membership can hide a moved upper-guard node.
    regions={'bottom_and_root_Cz_le10':originalC[:,2]<=10,
             'guard_above_Cz10':originalC[:,2]>10,
             'lens_height_band_Cz_ge76p635':originalC[:,2]>=76.63576799,
             'original_bottom_plane_absCz_le1e_5':np.abs(originalC[:,2])<=1e-5}
    native=cq.importers.importStep(str(source_step)).val().Solids()
    assert len(native)==8
    solids=[]
    for entry,s in zip(manifest['instances'],native):
        assert s.isValid() and len(s.Solids())==1
        bb=s.BoundingBox();bounds=np.array([[bb.xmin,bb.ymin,bb.zmin],[bb.xmax,bb.ymax,bb.zmax]])
        cl=BRepClass3d_SolidClassifier(s.wrapped)
        solids.append((entry['name'],s,bounds,cl))
    baseline,base_in,base_on,candidates=classify(originalC,solids)
    print('BASELINE',{'boundary_nodes':len(boundary),'IN':int(sum(baseline==2)),'ON':int(sum(baseline==1)),
                     'upper_IN':int(sum((baseline==2)&regions['guard_above_Cz10']))},flush=True)
    records=[];node_rows=[];states={'original':baseline};points={'original':originalC}
    for magnitude in [1,3]:
        for axis in range(3):
            for sign in [1,-1]:
                scale=sign*magnitude;q=Q[:,axis]*scale
                moved=boundary+displacement[:,:,axis]*scale
                # Exact inverse of the specified affine small-rotation map:
                # p_def = p_ref + qt + theta x (p_ref-COM).
                A=np.eye(3)+skew(q[3:])
                relativeF=np.linalg.solve(A,(moved-q[:3]+skew(q[3:])@COM).T).T
                relativeC=(relativeF-offset)@R
                state,bits,onbits,aabb=classify(relativeC,solids)
                new=(state==2)&(baseline!=2)
                new_from_clear=(state==2)&(baseline==0)
                new_from_contact=(state==2)&(baseline==1)
                name=f'{"plus" if sign>0 else "minus"}{"XYZ"[axis]}_{magnitude}g'
                # First-order relative displacement is also retained as a
                # kinematic truncation indicator; no second solve is involved.
                relative_first=boundary+displacement[:,:,axis]*scale-q[:3]-np.cross(q[3:],boundary-COM)
                first_order_delta=np.linalg.norm(relativeF-relative_first,axis=1)
                # Exact point-to-BRep-boundary depths for up to 12 new nodes
                # selected separately in bottom/upper/lens regions. Selection
                # is by relative nodal motion, not a claim of global max depth.
                relnorm=np.linalg.norm(relativeC-originalC,axis=1);verify_ids=[]
                for region in regions.values():
                    ii=np.flatnonzero(new&region)
                    verify_ids.extend(ii[np.argsort(relnorm[ii])[-12:]].tolist())
                depth_samples=[]
                for k in sorted(set(verify_ids)):
                    for sidx,(sname,shape,_,_) in enumerate(solids):
                        if bits[k]&(1<<sidx):
                            depth=point_boundary_distance(relativeC[k],shape)
                            depth_samples.append({'boundary_row':int(k),'scalar_dof_node':int(scalar_node_index[k]),
                              'gmsh_node_tag':int(gmsh_ids[k]),'solid':sname,'inside_solid_boundary_distance_mm':depth,
                              'original_C_mm':originalC[k].tolist(),'relative_C_mm':relativeC[k].tolist(),
                              'original_state':['OUT','ON','IN'][baseline[k]],
                              'guard_above_Cz10':bool(regions['guard_above_Cz10'][k])})
                record={'case':name,'axis':'XYZ'[axis],'signed_total_effective_g':scale,
                   'q_translation_F_mm':q[:3].tolist(),'q_rotation_vector_F_rad':q[3:].tolist(),
                   'q_rotation_norm_degrees':float(np.linalg.norm(q[3:])*180/np.pi),
                   'IN_nodes':int(sum(state==2)),'ON_nodes':int(sum(state==1)),
                   'new_IN_nodes':int(sum(new)),'new_IN_from_original_OUT':int(sum(new_from_clear)),
                   'new_IN_from_original_ON':int(sum(new_from_contact)),
                   'new_IN_by_original_region':region_counts(new,regions),
                   'new_IN_from_OUT_by_original_region':region_counts(new_from_clear,regions),
                   'only_existing_bottom_contact_nodes_intrude':bool(np.any(new) and np.all((baseline[new]==1)&regions['original_bottom_plane_absCz_le1e_5'][new])),
                   'AABB_candidate_counts':aabb,
                   'max_affine_inverse_vs_first_order_relative_point_difference_mm':float(first_order_delta.max()),
                   'exact_inside_depth_samples':depth_samples,
                   'max_sampled_inside_depth_mm':max([v['inside_solid_boundary_distance_mm'] for v in depth_samples if v['inside_solid_boundary_distance_mm'] is not None],default=0),
                   'nonpenetration_compatibility_of_current_linear_solution':'violated_at_sampled_boundary_nodes' if np.any(new) else 'no_new_sampled_node_intrusion_detected'}
                record['guard_clearance_bracket']=nodal_clearance_bracket(relativeC,regions['guard_above_Cz10'],state,solids)
                record['lens_height_clearance_bracket']=nodal_clearance_bracket(relativeC,regions['lens_height_band_Cz_ge76p635'],state,solids)
                records.append(record);states[name]=state;points[name]=relativeC
                for k in np.flatnonzero(new):
                    node_rows.append({'case':name,'gmsh_node_tag':int(gmsh_ids[k]),'scalar_dof_node':int(scalar_node_index[k]),
                      'original_state':['OUT','ON','IN'][baseline[k]],'original_Cx_mm':originalC[k,0],
                      'original_Cy_mm':originalC[k,1],'original_Cz_mm':originalC[k,2],
                      'relative_Cx_mm':relativeC[k,0],'relative_Cy_mm':relativeC[k,1],'relative_Cz_mm':relativeC[k,2],
                      'inside_solids':'|'.join(solids[i][0] for i in range(8) if bits[k]&(1<<i)),
                      'guard_above_Cz10':bool(regions['guard_above_Cz10'][k]),
                      'lens_height_band':bool(regions['lens_height_band_Cz_ge76p635'][k])})
                print(name,{'new_IN':record['new_IN_nodes'],'new_from_clear':record['new_IN_from_original_OUT'],
                            'upper':record['new_IN_by_original_region']['guard_above_Cz10'],
                            'sample_depth_mm':record['max_sampled_inside_depth_mm']},flush=True)
    report={'level':level,'method':'OCCT point-in-solid on the eight original source camera solids, AABB screened; solved FE frame boundary nodes inverse-mapped through camera linear rigid-motion field',
      'source_files':{str(p.relative_to(ROOT)):{'sha256':sha(p),'bytes':p.stat().st_size} for p in [field,mesh,source_step,manifest_path]},
      'classification_tolerance_mm':TOL,'node_map_max_error_mm':float(errors.max()),'mesh':mesh_info,
      'camera_reference_is_community_model_not_manufacturer_certified':True,
      'known_source_lens_body_internal_overlap_mm3':manifest['native_source_Lens_Body_overlap_mm3'],
      'camera_COM_F_mm':COM.tolist(),'Q_1g':Q.tolist(),
      'original':{'IN_nodes':int(sum(baseline==2)),'ON_nodes':int(sum(baseline==1)),'OUT_nodes':int(sum(baseline==0)),
        'IN_by_region':region_counts(baseline==2,regions),'ON_by_region':region_counts(baseline==1,regions),'AABB_candidate_counts':candidates},
      'cases':records,'new_intrusion_node_records':len(node_rows),
      'any_new_guard_node_intrusion':any(r['new_IN_by_original_region']['guard_above_Cz10']>0 for r in records),
      'any_new_intrusion_from_initially_clear_nodes':any(r['new_IN_from_original_OUT']>0 for r in records),
      'limits':['Boundary-node screen only, not continuous deformed-surface intersection or a nonlinear contact solve.',
        'Uses the requested affine small-rotation camera displacement map, not finite-rotation rigid kinematics.',
        'No reaction/contact forces, peak impact, fracture or altered load sharing are computed.',
        'Eight source solids are tested as a union; their known source internal overlap is retained.',
        'Point-to-boundary depths are explicitly selected samples, not the global maximum penetration depth.',
        'No box-envelope intrusion is treated as actual camera collision.'],
      'CAD_modified':False,'stiffness_matrix_loaded':False,'FEM_solved':False,'runtime_s':time.monotonic()-t0}
    report['original']['guard_clearance_bracket']=nodal_clearance_bracket(originalC,regions['guard_above_Cz10'],baseline,solids)
    report['original']['lens_height_clearance_bracket']=nodal_clearance_bracket(originalC,regions['lens_height_band_Cz_ge76p635'],baseline,solids)
    (out/'contact_compatibility.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    write_brief(report,out/'CONTACT_SCREEN_zh.md')
    if node_rows:
        with (out/'new_intrusion_nodes.csv').open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=list(node_rows[0]));w.writeheader();w.writerows(node_rows)
    np.savez_compressed(out/'node_screen_fields.npz',boundary_F_mm=boundary,original_C_mm=originalC,
        scalar_dof_node=scalar_node_index,gmsh_node_tag=gmsh_ids,
        **{'state_'+k:v for k,v in states.items()},**{'relativeC_'+k:v for k,v in points.items()})
    print('COMPLETE',str(out/'contact_compatibility.json'),'seconds',time.monotonic()-t0,flush=True)

if __name__=='__main__':main()