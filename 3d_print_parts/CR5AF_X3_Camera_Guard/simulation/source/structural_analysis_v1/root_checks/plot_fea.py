"""CPU-only plots of actual VTU finite-element results; no synthetic contours.

VTK extracts the quadratic-element surface without an OpenGL renderer.
Matplotlib draws depth-sorted triangles and an explicit numeric color scale.
"""
from pathlib import Path
import argparse,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.colors import Normalize
from matplotlib.font_manager import FontProperties
import vtk
from vtk.util.numpy_support import vtk_to_numpy

FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
fp=FontProperties(fname=FONT)


def surface(vtu):
    r=vtk.vtkXMLUnstructuredGridReader();r.SetFileName(str(vtu));r.Update()
    s=vtk.vtkDataSetSurfaceFilter();s.SetInputConnection(r.GetOutputPort());s.SetNonlinearSubdivisionLevel(1)
    t=vtk.vtkTriangleFilter();t.SetInputConnection(s.GetOutputPort());t.Update();o=t.GetOutput()
    p=vtk_to_numpy(o.GetPoints().GetData())
    tri=vtk_to_numpy(o.GetPolys().GetData()).reshape(-1,4)[:,1:]
    fields={}
    for j in range(o.GetCellData().GetNumberOfArrays()):
        a=o.GetCellData().GetArray(j);fields[a.GetName()]=vtk_to_numpy(a)
    points={}
    for j in range(o.GetPointData().GetNumberOfArrays()):
        a=o.GetPointData().GetArray(j);points[a.GetName()]=vtk_to_numpy(a)
    return p,tri,fields,points


def draw(vtu,output,title='',quantity='stress',eye=(1.5,-2.5,1.7),clip_percentile=99.5,
         deformation_scale=0.,note=''):
    p,tri,cdata,pdata=surface(vtu)
    if quantity=='stress':
        val=cdata['von_mises_MPa'];label='单元平均 von Mises 应力 / MPa'
    else:
        val=np.linalg.norm(pdata['displacement_mm'][tri],axis=2).mean(axis=1)
        label='位移幅值 / mm'
    if deformation_scale:
        p=p+deformation_scale*pdata['displacement_mm']
    center=(p.max(axis=0)+p.min(axis=0))/2
    view=np.asarray(eye,float);view/=np.linalg.norm(view)
    right=np.cross([0.,0.,1.],view);right/=np.linalg.norm(right)
    up=np.cross(view,right)
    xyz=p-center;xy=np.column_stack([xyz@right,xyz@up]);depth=xyz@view
    order=np.argsort(depth[tri].mean(axis=1))
    vmax=float(np.percentile(val,clip_percentile));vmax=max(vmax,1e-12)
    norm=Normalize(0,vmax);cmap=plt.get_cmap('turbo')
    fig,ax=plt.subplots(figsize=(9,6.1),dpi=170)
    fig.subplots_adjust(left=.025,right=.83,bottom=.14,top=.90)
    col=PolyCollection(xy[tri[order]],facecolors=cmap(norm(val[order])),edgecolors='none',antialiased=False)
    ax.add_collection(col);ax.autoscale_view();ax.set_aspect('equal');ax.axis('off')
    lims=ax.axis();span=max(lims[1]-lims[0],lims[3]-lims[2]);origin=np.array([lims[0]+.10*span,lims[2]+.12*span])
    length=.13*span
    for i,(a,color) in enumerate(zip('XYZ',['#b3353a','#329553','#3864b0'])):
        vec=np.array([right[i],up[i]])*length
        ax.annotate('',xy=origin+vec,xytext=origin,arrowprops={'arrowstyle':'->','color':color,'lw':1.2})
        ax.text(*(origin+vec*1.18),a,color=color,fontsize=9,ha='center',va='center')
    cax=fig.add_axes([.855,.22,.022,.55])
    cb=fig.colorbar(plt.cm.ScalarMappable(norm=norm,cmap=cmap),cax=cax)
    cb.set_label(label,fontproperties=fp,fontsize=10,labelpad=11);cb.ax.tick_params(labelsize=9)
    fig.suptitle(title,fontproperties=fp,fontsize=15,color='#173247',y=.96)
    desc=f'真实 P2 四面体场结果；曲面由网格提取。色标上限 {vmax:.3g}，表面单元均值最大 {float(val.max()):.3g}。'
    desc+='\n'+('几何显示未放大变形。' if not deformation_scale else f'显示变形放大倍数 {deformation_scale:g}；特征向量幅值没有实际载荷意义。')
    if note:desc+=' '+note
    fig.text(.04,.05,desc,fontproperties=fp,fontsize=8.4,color='#4b5b68',va='bottom')
    out=Path(output);out.parent.mkdir(parents=True,exist_ok=True);fig.savefig(out,facecolor='white');plt.close(fig)
    meta={'source_vtu':str(vtu),'quantity':quantity,'surface_triangles':len(tri),
          'surface_numeric_max':float(val.max()),'color_scale_max':vmax,
          'color_scale_percentile':clip_percentile,'deformation_scale':deformation_scale,
          'note':'Stress is element averaged and is not the raw integration-point peak reported in JSON.'}
    out.with_suffix('.json').write_text(json.dumps(meta,indent=2),encoding='utf-8')
    return meta


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('vtu');ap.add_argument('output');ap.add_argument('--title',default='')
    ap.add_argument('--quantity',choices=['stress','displacement'],default='stress')
    args=ap.parse_args();print(draw(args.vtu,args.output,args.title,args.quantity))