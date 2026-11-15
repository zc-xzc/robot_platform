"""Traceable linear 1g-to-3g field scaling and real modal/static plots."""
from pathlib import Path
import json, hashlib
import meshio
from plot_fea import draw

ROOT=Path(__file__).resolve().parents[1]
source=ROOT/'camera_fea/fine/camera_acceleration_plus_Z_1g.vtu'
target=ROOT/'service_results/camera/fine/plusZ_3g.vtu'
target.parent.mkdir(parents=True,exist_ok=True)
m=meshio.read(source)
for key in ['displacement_mm']:
    m.point_data[key]*=3.
for key in ['von_mises_MPa','stress_XX_YY_ZZ_XY_YZ_ZX_MPa']:
    m.cell_data[key]=[3.*v for v in m.cell_data[key]]
meshio.write(target,m,binary=True)
target.with_suffix('.json').write_text(json.dumps({
    'source':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
    'scale':3.,'basis':'Exact linear postprocessing of completed +Z total-effective-1g displacement and stress fields; no extra 1g added. No nonlinear or contact solution is implied.'},indent=2))
draw(target,ROOT/'report_figures/camera_plusZ_3g_stress.png',
     '相机框：+Z 合成 3g / fine',eye=(1.8,-2.2,1.6))
draw(ROOT/'camera_fea/coarse/mode_01_shape_max1mm.vtu',ROOT/'report_figures/camera_mode1.png',
     '相机模块第一阶振型 / coarse',quantity='displacement',eye=(1.8,-2.2,1.6),
     deformation_scale=3.,note='振型归一化最大值 1 mm，非实际振幅。')
print('Camera plots written from completed VTU fields.')