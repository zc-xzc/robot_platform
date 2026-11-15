"""Read-only FEM result figures. No CAD imports, mesh assembly, or solves.

Usage:
  python structural_analysis_v1/final_plot_helpers/plot_final_results.py
  python structural_analysis_v1/final_plot_helpers/plot_final_results.py --level medium

auto: choose the finest completed camera dynamic data; mesh plots include all
completed levels. An explicit level selects that exact camera dynamic level
and caps mesh comparisons at that level. Incomplete source reports are omitted,
never extrapolated or silently treated as completed. Existing source files are
not changed. All plotted numbers and source hashes are written next to figures.
"""
from pathlib import Path
import argparse,csv,datetime,hashlib,json,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from fontTools.ttLib import TTFont

HERE=Path(__file__).resolve().parent
ANA=HERE.parent
LEVELS=['coarse','medium','fine']
ZH={'coarse':'粗','medium':'中','fine':'细'}
COLORS={'X':'#176B87','Y':'#B56B24','Z':'#7863A8'}
DAMP_COLORS={.02:'#B4522B',.05:'#176B87',.1:'#57905B'}
GRAY='#465460'
SOURCES={}
WARNINGS=[]

def setup_font():
    cache=HERE/'font_cache';cache.mkdir(exist_ok=True)
    p=cache/'NotoSansCJKSC-Regular.otf'
    if not p.exists():
        TTFont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',fontNumber=2).save(str(p))
    font_manager.fontManager.addfont(str(p))
    assert font_manager.FontProperties(fname=str(p)).get_name()=='Noto Sans CJK SC'
    plt.rcParams.update({'font.family':'Noto Sans CJK SC','font.size':11,
      'axes.unicode_minus':False,'svg.fonttype':'path','figure.facecolor':'white',
      'savefig.facecolor':'white','axes.spines.top':False,'axes.spines.right':False,
      'axes.titleweight':'bold','axes.titlepad':10})
    return str(p)

def source_bytes(p):
    data=p.read_bytes()
    SOURCES[str(p.relative_to(ANA))]={'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),
                                   'mtime_ns':p.stat().st_mtime_ns}
    return data

def readj(p):
    return json.loads(source_bytes(p))

def csv_out(p,rows):
    if not rows:return
    with p.open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

def save(fig,out,name):
    fig.savefig(out/(name+'.png'),dpi=180,bbox_inches='tight')
    fig.savefig(out/(name+'.svg'),bbox_inches='tight')
    plt.close(fig)

def camera_static_complete(j):
    return ({v.get('axis') for v in j.get('positive_axis_1g_solutions',[])}==set('XYZ')
       and len(j.get('unit_external_force_cases',[]))==3
       and 'worst_CoG_displacement_mm_at_total_1g' in j)

def dynamic_completed(level):
    p=ANA/'camera_fea'/level
    files=[p/x for x in ['static_results.json','modal_results.json','harmonic_summary.json',
                         'harmonic_camera_CoG_0p1g.csv']]
    if not all(f.exists() for f in files):return False
    try:
        s=readj(files[0]);m=readj(files[1]);h=readj(files[2])
        if not camera_static_complete(s) or len(m.get('frequencies_Hz',[]))<6:return False
        if h.get('retained_modes')!=m.get('mode_count'):return False
        # Detect old dynamic data remaining while a level is being rerun.
        if any(f.stat().st_mtime_ns<files[0].stat().st_mtime_ns for f in files[1:]):return False
        return True
    except (json.JSONDecodeError,KeyError,OSError):return False

def choose_dynamic(requested):
    available=[v for v in LEVELS if dynamic_completed(v)]
    if requested!='auto':
        if requested not in available:
            raise RuntimeError(f'Requested {requested} camera dynamic data are not complete. Available: {available}. No silent fallback.')
        return requested,available
    if not available:raise RuntimeError('No completed camera static + modal + harmonic result set is available.')
    return available[-1],available

def plot_frf(out,level):
    p=ANA/'camera_fea'/level
    modal=readj(p/'modal_results.json');h=readj(p/'harmonic_summary.json')
    if not np.isclose(h['excitation_amplitude_g'],.1):
        raise ValueError('This requested plot requires actual 0.1g source results.')
    text=source_bytes(p/'harmonic_camera_CoG_0p1g.csv').decode('utf-8-sig')
    all_rows=[]
    for row in csv.DictReader(text.splitlines()):
        all_rows.append({k:(v if k=='excitation_axis' else float(v)) for k,v in row.items()})
    selected=[r for r in all_rows if r['frequency_Hz']<=80 and
              (r['excitation_axis']=='Z' or np.isclose(r['assumed_damping_ratio'],.05))]
    fig,ax=plt.subplots(1,2,figsize=(13,6.4))
    fig.subplots_adjust(left=.075,right=.97,top=.78,bottom=.26,wspace=.25)
    for damping in [.02,.05,.1]:
        r=sorted([v for v in selected if v['excitation_axis']=='Z' and
                  np.isclose(v['assumed_damping_ratio'],damping)],key=lambda v:v['frequency_Hz'])
        if len(r)<20:raise ValueError(('Insufficient completed harmonic rows','Z',damping))
        ax[0].plot([v['frequency_Hz'] for v in r],[v['peak_vector_displacement_mm'] for v in r],
                    color=DAMP_COLORS[damping],ls='-',lw=1.8,label=f'假设阻尼比 {damping:.0%}')
    for axis in 'XYZ':
        r=sorted([v for v in selected if v['excitation_axis']==axis and
                  np.isclose(v['assumed_damping_ratio'],.05)],key=lambda v:v['frequency_Hz'])
        if len(r)<20:raise ValueError(('Insufficient completed harmonic rows',axis,.05))
        ax[1].plot([v['frequency_Hz'] for v in r],[v['peak_vector_displacement_mm'] for v in r],
                    color=COLORS[axis],ls='-',lw=1.8,label=f'{axis} 向输入')
    f1=float(modal['frequencies_Hz'][0])
    for a in ax:
        a.set_xlim(0,80);a.set_ylim(bottom=0);a.grid(alpha=.2);a.set_axisbelow(True)
        a.set_xlabel('激励频率（Hz）');a.set_ylabel('相机质心相对根部的位移幅值（mm）')
        a.axvline(f1,color='#75828A',ls=':',lw=1)
        a.text(f1+1,a.get_ylim()[1]*.92,f'f1={f1:.3f} Hz',color=GRAY,fontsize=9,
               bbox={'facecolor':'white','alpha':.8,'edgecolor':'none','pad':1})
        a.legend(frameon=False,fontsize=10,loc='upper right')
    ax[0].set_title('Z 向 0.1g 输入：三种假设阻尼')
    ax[1].set_title('5% 假设阻尼：X / Y / Z 向 0.1g 输入')
    fig.suptitle('相机模块低频稳态响应：0–80 Hz',fontsize=19,fontweight='bold',y=.97)
    fig.text(.075,.87,'仅相机框架 + 180 g 刚体相机的接合根部模型；不是机械臂完整总装模态。',fontsize=11,color=GRAY)
    fig.text(.97,.035,f'实际动态数据：{level}（{ZH[level]}网格） / {modal["mode_count"]} 阶',
             ha='right',fontsize=10,color=GRAY)
    fig.text(.075,.13,'实线均为有效基座加速度幅值 0.1g 的线性稳态响应；阻尼未经实测。',fontsize=10.5,color='#AA572D')
    fig.text(.075,.075,'位移包含分量相位效应；接触间隙、机械臂柔度与瞬态碰撞未模拟。',fontsize=10,color=GRAY)
    save(fig,out,'01_camera_module_FRF_0to80Hz')
    rows=[dict(level=level,**r) for r in selected]
    csv_out(out/'01_camera_module_FRF_plot_data.csv',rows)
    return {'level':level,'first_mode_Hz':f1,'retained_modes':modal['mode_count'],
            'excitation_g':.1,'frequency_plot_span_Hz':[0,80],'row_count':len(rows)}

def collect_mesh_results(cap):
    allowed=LEVELS if cap=='auto' else LEVELS[:LEVELS.index(cap)+1]
    groups={'base':{},'cartridge':{},'camera':{}};excluded=[]
    for part in ['base','cartridge']:
        for level in allowed:
            p=ANA/'mount_fea'/part/level/'results.json'
            if not p.exists():excluded.append({'part':part,'level':level,'reason':'missing'});continue
            try:j=readj(p)
            except json.JSONDecodeError:excluded.append({'part':part,'level':level,'reason':'being written'});continue
            required=[f'motor_F{a}_1N_at_z100' if part=='base' else f'cameraCoG_F{a}_1N' for a in 'XYZ']
            if j.get('complete') is not True or not all(k in j.get('cases',{}) for k in required):
                excluded.append({'part':part,'level':level,'reason':'complete=true and all plotted cases required'});continue
            groups[part][level]=j
    for level in allowed:
        p=ANA/'camera_fea'/level/'static_results.json'
        if not p.exists():excluded.append({'part':'camera','level':level,'reason':'missing'});continue
        try:j=readj(p)
        except json.JSONDecodeError:excluded.append({'part':'camera','level':level,'reason':'being written'});continue
        if camera_static_complete(j):groups['camera'][level]=j
        else:excluded.append({'part':'camera','level':level,'reason':'static checkpoint incomplete'})
    # Reject mixed CAD/material/load definitions instead of drawing a fake mesh trend.
    for part in ['base','cartridge']:
        ref=None;reference_wrenches=None
        for level,j in groups[part].items():
            signature={'sha':j['source_sha256'],'E':j['mesh_info']['E_MPa'],'nu':j['mesh_info']['nu'],
                        'rho':j['mesh_info']['density_tonne_per_mm3']}
            if ref is None:ref=signature
            if signature!=ref:raise ValueError(('Incompatible mesh trend sources',part,level,signature,ref))
            wrenches=[]
            for axis in 'XYZ':
                name=f'motor_F{axis}_1N_at_z100' if part=='base' else f'cameraCoG_F{axis}_1N'
                load=j['cases'][name]['load'];force=np.asarray(load['target_force_N'])
                moment=np.asarray(load['target_moment_Nmm'])+np.cross(load['moment_reference_mm'],force)
                wrenches.append(np.r_[force,moment])
            if reference_wrenches is None:reference_wrenches=np.asarray(wrenches)
            if not np.allclose(wrenches,reference_wrenches,rtol=1e-10,atol=1e-7):
                raise ValueError(('Mesh levels use different requested force/moment definitions',part,level))
    camera_ref=None
    for level,j in groups['camera'].items():
        signature={'material':j['material_assumptions'],'camera_mass':j['camera_mass_kg'],'CG':j['camera_CoG_F_mm']}
        if camera_ref is None:camera_ref=signature
        if signature!=camera_ref:raise ValueError(('Incompatible camera mesh trend',level))
    return groups,excluded,allowed

def mesh_plot_rows(groups):
    rows=[]
    for part,levels in groups.items():
        for level,j in levels.items():
            info=j['mesh'] if part=='camera' else j['mesh_info']
            for axis in 'XYZ':
                if part=='camera':
                    c=next(v for v in j['positive_axis_1g_solutions'] if v['axis']==axis)
                    value=c['camera_CoG_displacement_mm'];resp=c['summary'];case=f'camera_CoG_plus{axis}_total1g'
                    quantity='Camera CoG translation magnitude mm at total 1g'
                else:
                    case=f'motor_F{axis}_1N_at_z100' if part=='base' else f'cameraCoG_F{axis}_1N'
                    c=j['cases'][case];value=c['solution']['compliance_Nmm'];resp=c['response']
                    quantity='Generalized compliance 2U/P^2 mm/N, P=1N'
                rows.append({'part':part,'level':level,'case':case,'axis':axis,
                  'tetrahedra':info['tetrahedra'],'displacement_dofs':info['displacement_dofs'],
                  'quantity':quantity,'response_value':float(value),
                  'raw_peak_von_mises_MPa':resp['max_von_mises_MPa'],
                  'volume_weighted_p99_von_mises_MPa':resp['volume_weighted_p99_von_mises_MPa']})
    return rows

def plot_mesh_trends(out,cap):
    groups,excluded,allowed=collect_mesh_results(cap);rows=mesh_plot_rows(groups)
    titles={'base':'底座：1 N 电机远程力，F 系 z=100 mm',
            'cartridge':'D8 卡匣断前：相机质心处 1 N',
            'camera':'相机模块：总有效加速度 1g'}
    summaries={}
    for peak in [False,True]:
        fig,axs=plt.subplots(1,3,figsize=(14.2,6.3))
        fig.subplots_adjust(left=.07,right=.98,bottom=.30,top=.77,wspace=.32)
        for a,part in zip(axs,['base','cartridge','camera']):
            levels=[l for l in allowed if l in groups[part]]
            for axis in 'XYZ':
                r=[next(v for v in rows if v['part']==part and v['level']==l and v['axis']==axis) for l in levels]
                xx=[allowed.index(l) for l in levels]
                yy=[v['raw_peak_von_mises_MPa'] if peak else v['response_value'] for v in r]
                marker,style={'X':('o','-'),'Y':('s','--'),'Z':('^','-.')}[axis]
                a.plot(xx,yy,marker=marker,ls=style,color=COLORS[axis],label=f'{axis} 向',
                       lw=1.7,ms=6,markerfacecolor='white' if axis=='Y' else COLORS[axis])
            labels=[]
            for l in allowed:
                if l in levels:
                    n=next(v['displacement_dofs'] for v in rows if v['part']==part and v['level']==l)
                    labels.append(f'{ZH[l]} {l}\n{n/1000:.1f}k DOF')
                else:labels.append(f'{ZH[l]} {l}\n尚未完成')
            a.set_xticks(range(len(allowed)),labels,fontsize=9)
            a.set_xlim(-.25,len(allowed)-.75 if len(allowed)>1 else .5)
            a.set_title(titles[part],fontsize=11)
            a.grid(alpha=.2);a.set_axisbelow(True)
            if peak:a.set_ylabel('原始积分点 von Mises 峰值（MPa）',fontsize=10)
            elif part=='camera':a.set_ylabel('相机质心位移幅值（mm）',fontsize=10)
            else:a.set_ylabel('单位力广义柔度 2U/P²（mm/N）',fontsize=10)
            a.ticklabel_format(axis='y',style='sci',scilimits=(-2,3),useMathText=True)
            a.set_ylim(0,max(a.get_ylim()[1],1e-12)*1.18)
            if not levels:a.text(.5,.5,'无完整结果',transform=a.transAxes,ha='center',color=GRAY)
            label=' → '.join(levels) if levels else '无'
            a.text(.02,.98,f'实际级别：{label}',transform=a.transAxes,va='top',fontsize=8.5,
                   bbox={'facecolor':'white','edgecolor':'none','alpha':.9,'pad':2})
            if not peak:
                delta={}
                if len(levels)>=2:
                    for axis in 'XYZ':
                        r=[next(v for v in rows if v['part']==part and v['level']==l and v['axis']==axis) for l in levels[-2:]]
                        delta[axis]=(r[1]['response_value']/r[0]['response_value']-1)*100
                summaries[part]={'completed_levels':levels,'latest_level':levels[-1] if levels else None,
                  'last_step_response_change_percent':delta,'convergence_certified':False}
        handles,labels=axs[0].get_legend_handles_labels()
        fig.legend(handles,labels,ncol=3,loc='lower center',bbox_to_anchor=(.5,.18),frameon=False)
        if peak:
            title='原始应力尖峰的网格敏感性：单独检查'
            note='尖角、约束边缘与载荷突变可能产生不收敛尖峰；不据此直接判定释放力、强度或安全系数。'
            name='03_raw_stress_peak_mesh_sensitivity'
        else:
            title='网格敏感性：广义柔度与相机质心位移'
            note='显示已完成算例的变化，不自动宣称已收敛；各面板载荷和量纲按标题与纵轴分别解释。'
            name='02_response_mesh_sensitivity'
        fig.suptitle(title,fontsize=19,fontweight='bold',y=.97)
        fig.text(.07,.87,'三个独立接合状态子模型；相机面板不是机械臂完整总装。未完成网格没有补点或外推。',fontsize=10.5,color=GRAY)
        fig.text(.07,.12,note,fontsize=10.5,color='#AA572D')
        fig.text(.98,.045,'实际处理级别见每个面板角标；来源哈希见 plot_manifest.json',ha='right',fontsize=9,color=GRAY)
        save(fig,out,name)
    csv_out(out/'02_03_mesh_sensitivity_data.csv',rows)
    return {'parts':summaries,'excluded_incomplete_or_missing':excluded,'included_rows':len(rows)}

def mode_table(out,level):
    m=readj(ANA/'camera_fea'/level/'modal_results.json')
    effective=np.asarray(m['effective_mass_tonne_XYZ'],float)*1000
    total=float(m['total_frame_camera_mass_kg']);n=min(6,len(m['frequencies_Hz']))
    rows=[]
    for i in range(n):
        rows.append({'level':level,'mode':i+1,'frequency_Hz':m['frequencies_Hz'][i],
          'eigen_relative_residual':m['eigen_residuals_relative'][i],
          'effective_mass_X_kg':effective[i,0],'effective_mass_Y_kg':effective[i,1],'effective_mass_Z_kg':effective[i,2],
          'effective_mass_fraction_X':effective[i,0]/total,'effective_mass_fraction_Y':effective[i,1]/total,
          'effective_mass_fraction_Z':effective[i,2]/total,
          'cumulative_mass_fraction_X':float(effective[:i+1,0].sum()/total),
          'cumulative_mass_fraction_Y':float(effective[:i+1,1].sum()/total),
          'cumulative_mass_fraction_Z':float(effective[:i+1,2].sum()/total),
          'total_frame_camera_mass_kg_assumed':total})
    csv_out(out/'04_camera_first6_modal_data.csv',rows)
    fig,ax=plt.subplots(figsize=(12.8,5.7));ax.axis('off')
    fig.subplots_adjust(left=.07,right=.96,top=.76,bottom=.29)
    table_rows=[[str(r['mode']),f"{r['frequency_Hz']:.3f}",
                 *[f"{r[f'effective_mass_{a}_kg']*1000:.4f}" for a in 'XYZ']] for r in rows]
    table=ax.table(cellText=table_rows,colLabels=['阶次','频率（Hz）','X 有效质量（g）','Y 有效质量（g）','Z 有效质量（g）'],
                    colWidths=[.11,.18,.23,.23,.23],cellLoc='center',colLoc='center',loc='center',bbox=[0,0,1,1])
    table.auto_set_font_size(False);table.set_fontsize(12)
    for (r,c),cell in table.get_celld().items():
        cell.set_edgecolor('white');cell.set_linewidth(2)
        if r==0:cell.set_facecolor('#176B87');cell.get_text().set_color('white');cell.get_text().set_weight('bold')
        else:cell.set_facecolor('#EAF1F3' if r%2 else '#F5F7F8')
    cumulative=effective[:n].sum(axis=0)/total*100
    all_cov=np.asarray(m['resolved_effective_mass_fraction_XYZ'])*100
    fig.suptitle('相机模块前 6 阶模态：频率与平移有效质量',fontsize=19,fontweight='bold',y=.97)
    fig.text(.07,.86,'均质框架 + 180 g 相机惯量代理，刚性接合根部；不是机械臂整机模态。',fontsize=11,color=GRAY)
    fig.text(.07,.20,'前 6 阶累计覆盖：'+ '；'.join(f'{a} {v:.2f}%' for a,v in zip('XYZ',cumulative)),fontsize=11,color=GRAY)
    fig.text(.07,.135,f"全部 {m['mode_count']} 阶累计覆盖："+'；'.join(f'{a} {v:.2f}%' for a,v in zip('XYZ',all_cov)),fontsize=10.5,color=GRAY)
    fig.text(.07,.075,'有效质量源于质量归一化模态参与系数；阻尼、连接间隙与真实相机质量分布均未标定。',fontsize=10,color='#AA572D')
    fig.text(.96,.025,f'实际数据：{level}（{ZH[level]}网格），假设总质量 {total:.6f} kg',ha='right',fontsize=9,color=GRAY)
    save(fig,out,'04_camera_first6_modes_table')
    return {'level':level,'displayed_modes':n,'retained_modes':m['mode_count'],
            'first6_cumulative_effective_mass_fraction_XYZ':(cumulative/100).tolist(),
            'all_retained_effective_mass_fraction_XYZ':(all_cov/100).tolist(),'actual_total_mass_measurement':False}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--level',choices=['auto',*LEVELS],default='auto')
    ap.add_argument('--output-dir',type=Path,default=HERE/'outputs')
    args=ap.parse_args();out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    font=setup_font();level,available=choose_dynamic(args.level)
    result={'created_UTC':datetime.datetime.now(datetime.timezone.utc).isoformat(),
      'requested_level':args.level,'camera_dynamic_level':level,'available_completed_camera_dynamic_levels':available,
      'read_only_results':True,'FEM_solved':False,'CAD_modified':False,'font':font,
      'FRF':plot_frf(out,level),'mesh_sensitivity':plot_mesh_trends(out,args.level),
      'first6_modes':mode_table(out,level),'source_files':SOURCES,'warnings':WARNINGS}
    names=['01_camera_module_FRF_0to80Hz','02_response_mesh_sensitivity',
           '03_raw_stress_peak_mesh_sensitivity','04_camera_first6_modes_table']
    paths=[out/(n+ext) for n in names for ext in ['.png','.svg']]
    paths+=sorted(out.glob('*.csv'))
    result['outputs']=[{'path':str(p),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in paths]
    (out/'plot_manifest.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps({'output_dir':str(out),'actual_camera_dynamic_level':level,
      'mesh_completed_levels':{k:v['completed_levels'] for k,v in result['mesh_sensitivity']['parts'].items()},
      'figures':names,'no_FEM_solve':True},ensure_ascii=False,indent=2))

if __name__=='__main__':main()