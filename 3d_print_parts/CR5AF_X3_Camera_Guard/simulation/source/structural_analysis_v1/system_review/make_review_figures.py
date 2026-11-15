"""Exact algebraic sensitivity plots and source-BRep section figures.
No simulated stress fields, no CAD edits. Units and assumptions appear on plots.
"""
from pathlib import Path
import json,csv,hashlib,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from matplotlib import font_manager
import cadquery as cq
from OCP.BRepAlgoAPI import BRepAlgoAPI_Section

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
OUT=HERE/'figures';OUT.mkdir(exist_ok=True)
from fontTools.ttLib import TTFont
# Matplotlib's cached font list may omit the SC face inside the Noto TTC.
# Extract the installed, exact SC face and register it explicitly.
font_cache=HERE/'font_cache';font_cache.mkdir(exist_ok=True)
FONT=str(font_cache/'NotoSansCJKSC-Regular.otf')
if not Path(FONT).exists():
    TTFont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',fontNumber=2).save(FONT)
font_manager.fontManager.addfont(FONT)
assert font_manager.FontProperties(fname=FONT).get_name()=='Noto Sans CJK SC'
plt.rcParams.update({'font.family':'Noto Sans CJK SC','font.size':11,
 'axes.unicode_minus':False,'svg.fonttype':'path','axes.spines.top':False,
 'axes.spines.right':False,'axes.titleweight':'bold','axes.titlepad':10,
 'savefig.facecolor':'white','figure.facecolor':'white'})
BLUE='#176B87';ORANGE='#C86B23';GRAY='#394957'
meta={'font':FONT,'native_CAD_modified':False,'stress_clouds_generated':False,'figures':[]}
def save(fig,name):
    fig.savefig(OUT/(name+'.png'),dpi=180,bbox_inches='tight')
    fig.savefig(OUT/(name+'.svg'),bbox_inches='tight')
    plt.close(fig)
def csvsave(name,rows):
    with (OUT/name).open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

# 1. h/s sensitivity: displayed mass and initial velocity are assumptions only.
m=.25;g=9.80665;v0=.5
h=np.linspace(0,100,101);s=np.geomspace(.5,20,121)
S,H=np.meshgrid(s,h);E=m*g*H/1000+.5*m*v0*v0
AVG=E/(S/1000)+m*g;PEAK=2*AVG
fig,ax=plt.subplots(1,2,figsize=(12.8,6.5),sharey=True)
fig.subplots_adjust(left=.07,right=.88,bottom=.27,top=.78,wspace=.15)
norm=LogNorm(vmin=3,vmax=1200)
for a,Z,title in zip(ax,[AVG,PEAK],['平均绳张力  E0/s + mg','假设线性弹簧峰值  2(E0/s + mg)']):
    im=a.pcolormesh(S,H,Z,norm=norm,cmap='viridis',shading='auto',rasterized=True)
    levels=[10,25,50,100,250,500,1000]
    cs=a.contour(S,H,Z,levels=levels,colors='white',linewidths=.55,alpha=.8)
    a.clabel(cs,fmt='%g N',fontsize=8,inline=True)
    a.set_xscale('log');a.set_xticks([.5,1,2,5,10,20],['0.5','1','2','5','10','20'])
    a.set_xlim(.5,20);a.set_ylim(0,100);a.set_title(title,fontsize=13)
    a.set_xlabel('受力后的有效止动行程 s（mm，对数轴）')
    val=(m*g*.02+.5*m*v0*v0)/.002+m*g
    if Z is PEAK:val*=2
    a.plot(2,20,'o',mec='white',mfc='#E26322',ms=7,zorder=5)
    a.annotate(f'h=20，s=2\n{val:.2f} N',xy=(2,20),xytext=(3.1,32),
        arrowprops={'arrowstyle':'->','color':'white'},color='white',fontsize=10,
        bbox={'boxstyle':'round,pad=.35','fc':'#183545','ec':'none','alpha':.88})
ax[0].set_ylabel('受绳前的竖直松弛下降 h（mm）')
cax=fig.add_axes([.9,.25,.018,.5]);cb=fig.colorbar(im,cax=cax)
cb.set_label('绳张力（N，色阶为对数）')
fig.suptitle('保险绳承接：停止行程越短，拉力越大',x=.49,y=.98,fontsize=19,fontweight='bold')
fig.text(.07,.89,'假设：随动质量 m=0.25 kg；下降开始前速度 v0=0.5 m/s；E0=mgh+½mv0²。',fontsize=11,color=GRAY)
fig.text(.07,.13,'峰值图仅假定零初始张力的线性弹簧，不是实际峰值预测或上界。',fontsize=11,color=ORANGE)
fig.text(.07,.045,'这是竖直点质量的能量敏感性模型；绳结、材料刚度、转动、阻尼和真实停止行程均未实测。\n若使用受绳前实测速率，应只用 ½mv²，不能重复加入 mgh。',fontsize=10,color=GRAY,linespacing=1.5)
save(fig,'01_tether_energy_h_stop_sensitivity')
csvsave('01_tether_energy_grid.csv',[dict(mass_kg=m,pre_drop_speed_m_s=v0,slack_drop_mm=float(H[i,j]),
 stop_distance_mm=float(S[i,j]),energy_J=float(E[i,j]),average_tension_N=float(AVG[i,j]),
 assumed_linear_spring_peak_N=float(PEAK[i,j])) for i in range(len(h)) for j in range(len(s))])
meta['figures'].append({'name':'01_tether_energy_h_stop_sensitivity','type':'Analytical energy sensitivity, not impact simulation','m_kg':m,'v0_m_s':v0,'g_m_s2':g,'data':'01_tether_energy_grid.csv'})

# 2. Each torque is an input to a sensitivity calculation, not an instruction.
fig,ax=plt.subplots(1,2,figsize=(12.8,6.5))
fig.subplots_adjust(left=.08,right=.9,bottom=.31,top=.77,wspace=.42)
rows=[]
for a,label,d,od,hole,tmax,markT in zip(ax,['M6：Ø12 垫圈 / Ø6.6 塑料孔','M3：Ø7 垫圈 / Ø3.4 塑料孔'],[6,3],[12,7],[6.6,3.4],[2,.6],[1,.2]):
    A=math.pi/4*(od*od-hole*hole);T=np.linspace(0,tmax,151)
    low=T/(.25*d/1000);high=T/(.15*d/1000);mid=T/(.2*d/1000)
    a.fill_between(T,low/A,high/A,color=BLUE,alpha=.23,label='K=0.15～0.25 的假设区间')
    a.plot(T,high/A,color=BLUE,lw=1.6,label='K=0.15')
    a.plot(T,low/A,color=BLUE,lw=1.2,ls='--',label='K=0.25')
    a.plot(T,mid/A,color=ORANGE,lw=1.6,label='K=0.20')
    a.set_title(label,fontsize=12.3);a.set_xlim(0,tmax);a.set_ylim(bottom=0)
    a.set_xlabel('输入扭矩 T（N·m；仅用于敏感性计算）')
    a.set_ylabel('投影面积上的平均承压 p（MPa）')
    a.grid(alpha=.2);a.set_axisbelow(True)
    sec=a.secondary_yaxis('right',functions=(lambda p,A=A:p*A,lambda f,A=A:f/A))
    sec.set_ylabel('每颗螺钉预紧力 F（N）',color=GRAY)
    ymin=markT/(.25*d/1000)/A;ymax=markT/(.15*d/1000)/A
    a.vlines(markT,ymin,ymax,color=ORANGE,lw=3)
    a.text(.04,.95,f'投影承压面积 A={A:.3f} mm²',transform=a.transAxes,va='top',color=GRAY,fontsize=9.5)
    a.annotate(f'{markT:g} N·m → {ymin:.2f}～{ymax:.2f} MPa',xy=(markT,ymax),
      xytext=(.06,.75),textcoords='axes fraction',fontsize=10,
      arrowprops={'arrowstyle':'->','color':ORANGE},color=ORANGE,
      bbox={'facecolor':'white','edgecolor':'none','alpha':.9,'pad':2})
    for i in range(len(T)):
        rows.append(dict(nominal_thread_diameter_mm=d,torque_assumption_Nm=float(T[i]),
          washer_OD_mm=od,plastic_hole_mm=hole,area_mm2=A,K_low=.15,K_high=.25,
          preload_min_N=float(low[i]),preload_max_N=float(high[i]),
          mean_pressure_min_MPa=float(low[i]/A),mean_pressure_max_MPa=float(high[i]/A),torque_recommendation=False))
handles,labels=ax[0].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',bbox_to_anchor=(.5,.18),ncol=4,frameon=False,fontsize=10)
fig.suptitle('螺钉预紧与打印件承压：扭矩系数敏感性',x=.5,y=.98,fontsize=19,fontweight='bold')
fig.text(.08,.89,'T=KFd；p=F/A。K 是扭矩系数，0.15～0.25 为演示范围，未测量实际摩擦与预紧。',fontsize=11,color=GRAY)
fig.text(.08,.12,'两图均不提供安装扭矩建议，也不证明材料承压合格。压溃、蠕变、垫圈弯曲与护夹变形未计入。',fontsize=10.5,color=ORANGE)
fig.text(.08,.045,'来源：NASA RP-1228，PDF 21–22；承压面积使用 V2R1 实际孔径。\n金属 M6 15 N·m 表值对应本假设压强 126.77～211.28 MPa，不能套用于该打印叠层。',fontsize=10,color=GRAY,linespacing=1.5)
save(fig,'02_fastener_preload_bearing_sensitivity');csvsave('02_fastener_curve_data.csv',rows)
meta['figures'].append({'name':'02_fastener_preload_bearing_sensitivity','type':'Torque/preload/average pressure sensitivity, no torque recommendation','K_range':[.15,.25],'data':'02_fastener_curve_data.csv'})

# 3. Actual source BRep sections, sampled from analytical section edges.
source={
 'cartridge':ROOT/'recovery_v2/assembly_instances_brep/02_Fuse_Cartridge_D8.brep',
 'camera_module':ROOT/'recovery_v2/assembly_instances_brep/04_Camera_Module_Short45.brep'}
sh={k:cq.Shape.importBrep(str(v)) for k,v in source.items()}
def section_edges(shape,point,normal):
    plane=cq.Face.makePlane(500,500,cq.Vector(*point),cq.Vector(*normal))
    sec=BRepAlgoAPI_Section(shape.wrapped,plane.wrapped,False);sec.ComputePCurveOn1(True);sec.Approximation(False);sec.Build()
    if not sec.IsDone():raise RuntimeError('Native BRep section failed')
    return cq.Shape.cast(sec.Shape()).Edges()
fig,ax=plt.subplots(1,2,figsize=(12.8,6.5))
fig.subplots_adjust(left=.07,right=.97,bottom=.23,top=.79,wspace=.22)
edge_rows=[]
for part,color in [('cartridge',BLUE),('camera_module',ORANGE)]:
    edges=section_edges(sh[part],(0,50,0),(0,1,0))
    for ei,e in enumerate(edges):
        pts=np.array([e.positionAt(float(t)).toTuple() for t in np.linspace(0,1,101)])
        ax[0].plot(pts[:,0],pts[:,2],color=color,lw=1.8,label=part if ei==0 else None)
        edge_rows.extend(dict(section='Y50_XZ',part=part,edge=ei,point=i,X_mm=float(p[0]),Y_mm=float(p[1]),Z_mm=float(p[2])) for i,p in enumerate(pts))
edges=section_edges(sh['cartridge'],(0,0,11.75),(0,0,1))
for ei,e in enumerate(edges):
    pts=np.array([e.positionAt(float(t)).toTuple() for t in np.linspace(0,1,161)])
    ax[1].plot(pts[:,0],pts[:,1],color=BLUE,lw=1.8)
    edge_rows.extend(dict(section='Z11p75_XY',part='cartridge',edge=ei,point=i,X_mm=float(p[0]),Y_mm=float(p[1]),Z_mm=float(p[2])) for i,p in enumerate(pts))
ax[0].set_xlim(4,12);ax[0].set_ylim(12.5,18.5);ax[0].set_aspect('equal')
ax[0].set_title('快拆局部截面：F 的 Y=50 mm',fontsize=12.5)
ax[0].set_xlabel('F 的 X（mm）');ax[0].set_ylabel('F 的 Z（mm）')
ax[0].annotate('',xy=(10.8,16.3),xytext=(10.8,16),arrowprops={'arrowstyle':'<->','color':'#9D1C3C','lw':1.2})
ax[0].annotate('抬升间隙 0.3 mm',xy=(10.8,16.15),xytext=(8.5,18.1),fontsize=10,color='#9D1C3C',
    arrowprops={'arrowstyle':'-','color':'#9D1C3C'},bbox={'facecolor':'white','edgecolor':'none','pad':2})
ax[0].annotate('',xy=(10.1,15.5),xytext=(5.9,15.5),arrowprops={'arrowstyle':'<->','color':GRAY,'lw':1.1})
ax[0].text(8,15.68,'槽宽 4.2',ha='center',va='bottom',fontsize=10,color=GRAY,bbox={'facecolor':'white','edgecolor':'none','pad':1})
ax[0].annotate('',xy=(9.8,14.2),xytext=(6.2,14.2),arrowprops={'arrowstyle':'<->','color':BLUE,'lw':1.1})
ax[0].text(8,13.8,'杆径 3.6',ha='center',va='top',fontsize=10,color=BLUE,bbox={'facecolor':'white','edgecolor':'none','pad':1})
ax[0].text(.04,.04,'横向名义总间隙：4.2−3.6=0.6 mm',transform=ax[0].transAxes,fontsize=9,color=GRAY,bbox={'facecolor':'white','edgecolor':'none','pad':2})
ax[1].set_xlim(-25,-14);ax[1].set_ylim(52,61.5);ax[1].set_aspect('equal')
ax[1].set_title('移动绳耳截面：F 的 Z=11.75 mm',fontsize=12.5)
ax[1].set_xlabel('F 的 X（mm）');ax[1].set_ylabel('F 的 Y（mm）')
ax[1].plot(-21,56.5,'+',color=GRAY,ms=9)
ax[1].annotate('Ø4.2 孔',xy=(-19.2,57.55),xytext=(-18.8,59.8),fontsize=10,color=GRAY,
 arrowprops={'arrowstyle':'->','color':GRAY},bbox={'facecolor':'white','edgecolor':'none','pad':1})
ax[1].annotate('',xy=(-23.1,56.5),xytext=(-24,56.5),arrowprops={'arrowstyle':'<->','color':'#9D1C3C','lw':1.3})
ax[1].annotate('最近净边 0.9 mm',xy=(-23.55,56.5),xytext=(-24.3,60.6),fontsize=10,color='#9D1C3C',
 arrowprops={'arrowstyle':'-','color':'#9D1C3C'},bbox={'facecolor':'white','edgecolor':'none','pad':2})
ax[1].text(.05,.05,'孔壁厚度方向：Fz 10.5～13 → 厚 2.5 mm',transform=ax[1].transAxes,fontsize=9,color=GRAY,bbox={'facecolor':'white','edgecolor':'none','pad':2})
for a in ax:a.grid(alpha=.16);a.set_axisbelow(True)
fig.suptitle('原生 CAD 实际截面：快拆间隙与保险绳耳薄边',x=.5,y=.98,fontsize=19,fontweight='bold')
fig.text(.07,.89,'蓝色：保险卡匣；橙色：相机一体框。线条来自冻结 V2R1 BRep 的平面截交，不是重画假想结构。',fontsize=10.8,color=GRAY)
fig.text(.07,.12,'显示的是未受力名义几何；未包含打印偏差、弹性变形或接触闭合。',fontsize=11,color=GRAY)
fig.text(.07,.065,'移动绳耳属于卡匣；若钥匙槽、蘑菇头或短钩先失效，当前保险绳不会直接约束脱出的相机模块。',fontsize=10.5,color=ORANGE)
save(fig,'03_actual_CAD_clearance_ligament');csvsave('03_native_section_edges.csv',edge_rows)
meta['figures'].append({'name':'03_actual_CAD_clearance_ligament','type':'Actual analytical BRep sections, no deformation/stress simulation',
 'section_planes_F_mm':[{'Y':50},{'Z':11.75}],
 'source_files':{k:{'path':str(v),'sha256':hashlib.sha256(v.read_bytes()).hexdigest()} for k,v in source.items()},
 'data':'03_native_section_edges.csv'})
(OUT/'figure_metadata.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2))
print('Created 3 figures (PNG/SVG) and 3 exact data CSVs. Font:',FONT)