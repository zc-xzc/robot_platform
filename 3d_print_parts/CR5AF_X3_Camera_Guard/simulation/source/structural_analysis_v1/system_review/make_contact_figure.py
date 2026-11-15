"""Plot actual classified FE boundary nodes; no solve and no stress fabrication."""
from pathlib import Path
import argparse,csv,json,hashlib
import numpy as np
import cadquery as cq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import Normalize
from fontTools.ttLib import TTFont
from check_camera_contact_compatibility import point_boundary_distance

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
ap=argparse.ArgumentParser();ap.add_argument('--level',choices=['coarse','medium','fine'],default='medium');args=ap.parse_args()
out=HERE/'contact_compatibility'/args.level
report=json.loads((out/'contact_compatibility.json').read_text())
field=np.load(out/'node_screen_fields.npz');original=field['original_C_mm']
rows=list(csv.DictReader((out/'new_intrusion_nodes.csv').open(encoding='utf-8-sig')))
manifest=json.loads((ROOT/'recovery_v2/camera/camera_reference_manifest.json').read_text())
source=ROOT/'recovery_v2/camera/reference/Insta360_X3_OngXeno_v6.step'
solids=cq.importers.importStep(str(source)).val().Solids()
byname={v['name']:s for v,s in zip(manifest['instances'],solids)}
font=HERE/'font_cache/NotoSansCJKSC-Regular.otf';font.parent.mkdir(exist_ok=True)
if not font.exists():TTFont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',fontNumber=2).save(str(font))
font_manager.fontManager.addfont(str(font))
plt.rcParams.update({'font.family':'Noto Sans CJK SC','axes.unicode_minus':False,'svg.fonttype':'path',
 'font.size':11,'figure.facecolor':'white','savefig.facecolor':'white','axes.spines.top':False,'axes.spines.right':False})

cases=['minusX_1g','minusX_3g'];data=[]
for name in cases:
 for row in [v for v in rows if v['case']==name]:
    point=np.array([float(row['relative_C'+a+'_mm']) for a in 'xyz'])
    depths=[point_boundary_distance(point,byname[n]) for n in row['inside_solids'].split('|')]
    depths=[v for v in depths if v is not None]
    data.append({'case':name,'gmsh_node_tag':row['gmsh_node_tag'],'original_Cx_mm':float(row['original_Cx_mm']),
      'original_Cy_mm':float(row['original_Cy_mm']),'original_Cz_mm':float(row['original_Cz_mm']),
      'relative_Cx_mm':point[0],'relative_Cy_mm':point[1],'relative_Cz_mm':point[2],
      'distance_to_entered_solid_boundary_mm':min(depths),'entered_solid':row['inside_solids'],
      'initial_classification':row['original_state']})
with (out/'contact_bottom_node_depths.csv').open('w',newline='',encoding='utf-8-sig') as f:
 w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)

fig,ax=plt.subplots(1,2,figsize=(13.2,6.2),sharex=True,sharey=True)
fig.subplots_adjust(left=.07,right=.87,bottom=.32,top=.74,wspace=.12)
vmax=max(v['distance_to_entered_solid_boundary_mm'] for v in data);norm=Normalize(0,max(.04,vmax*1.05))
initial_contact=field['state_original']==1
metadata={'level':args.level,'source_reference_step_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
 'contact_report_sha256':hashlib.sha256((out/'contact_compatibility.json').read_bytes()).hexdigest(),
 'calculation':'Exact OCCT point-to-entered-solid-boundary distance for every new IN node in the two displayed cases; no stress field, no contact reaction force.',
 'display_coordinates':'Original boundary node Cx/Cy coordinates, not an exaggerated deformed geometry.',
 'cases':[]}
for a,name in zip(ax,cases):
 a.scatter(original[initial_contact,0],original[initial_contact,1],s=8,color='#D0D6DA',label='初始接触节点',zorder=1)
 # Original actual camera bottom-plane face edges, including holes, are
 # sampled directly from analytical BRep curves and are never redrawn.
 for solid in [solids[5],solids[7]]:
  faces=[f for f in solid.Faces() if f.geomType()=='PLANE' and abs(f.BoundingBox().zmin)<1e-6 and abs(f.BoundingBox().zmax)<1e-6]
  for face in faces:
   for e in face.Edges():
    p=np.array([e.positionAt(float(t)).toTuple() for t in np.linspace(0,1,61)])
    a.plot(p[:,0],p[:,1],lw=.6,color='#53636E',alpha=.8,zorder=2)
 d=[v for v in data if v['case']==name]
 sc=a.scatter([v['original_Cx_mm'] for v in d],[v['original_Cy_mm'] for v in d],
   c=[v['distance_to_entered_solid_boundary_mm'] for v in d],cmap='magma',norm=norm,s=13,zorder=3)
 maxdepth=max(v['distance_to_entered_solid_boundary_mm'] for v in d)
 a.set_title(f'−X，总有效加速度 {"1g" if name.endswith("1g") else "3g"}',fontsize=14,fontweight='bold',pad=12)
 a.set_xlim(-24,24);a.set_ylim(-12.5,12.5);a.set_aspect('equal')
 a.set_xlabel('原始节点 Cx（mm）');a.grid(alpha=.13);a.set_axisbelow(True)
 a.text(.02,.96,f'{len(d)} 个新增穿入节点\n本图全部穿入点的最大边界距离：{maxdepth:.5f} mm',
   transform=a.transAxes,va='top',fontsize=9.5,bbox={'facecolor':'white','alpha':.9,'edgecolor':'none','pad':3})
 metadata['cases'].append({'case':name,'all_new_inside_nodes_evaluated':len(d),'max_exact_boundary_distance_mm':maxdepth})
ax[0].set_ylabel('原始节点 Cy（mm）')
cax=fig.add_axes([.9,.34,.018,.36]);cb=fig.colorbar(sc,cax=cax)
cb.set_label('到所侵入参考实体边界的距离（mm）',fontsize=10)
fig.suptitle('底面接触兼容性：线性等效载荷未强制非穿透',fontsize=19,fontweight='bold',y=.97)
fig.text(.07,.86,'着色点为实际 FE 边界节点，经相机同一线性刚体运动反变换后，进入原作者 Body / Threaded Insert 实体。',fontsize=10.6,color='#465460')
fig.text(.07,.23,'灰点：原始接触；细线：参考相机真实底面轮廓。所有新增穿入节点原始 Cz=0；颜色不是应力或接触力。',fontsize=10.4,color='#465460')
fig.text(.07,.15,'护柱 Cz>10 及镜头高度区：±XYZ 的 1g/3g 共 12 个工况均未检出新增节点侵入。',fontsize=10.6,color='#176B87')
fig.text(.07,.08,'底面局部接触结果不能据此判为合格；节点筛查不替代连续表面、打印公差或真实相机接触验证。',fontsize=10.6,color='#A65A2A')
fig.text(.97,.025,f'实际数据：{args.level}；{report["mesh"]["boundary_node_count"]:,} 个边界节点；无重求解',ha='right',fontsize=9,color='#465460')
for ext in ['png','svg']:fig.savefig(out/f'contact_bottom_nodes_1g_3g.{ext}',dpi=180,bbox_inches='tight')
plt.close(fig)
(out/'contact_figure_metadata.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2))
print(json.dumps(metadata['cases'],ensure_ascii=False,indent=2))