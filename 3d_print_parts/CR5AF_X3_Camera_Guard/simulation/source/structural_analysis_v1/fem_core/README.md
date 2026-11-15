# 真实 CAD 的线弹性实体有限元核心

本目录使用 Gmsh/OpenCASCADE 读取冻结的 BREP，生成 **10 节点曲边四面体**，以 scikit-fem 的二次位移单元进行三维小变形、均匀各向同性线弹性分析。没有修改原 CAD，也没有使用示意网格代替零件。

## 单位和假设

- 长度 mm，力 N，应力 MPa=N/mm²，能量 N·mm，时间 s，质量 tonne。
- 基线材料：E=2000 MPa，ν=0.35，ρ=1.24e-9 tonne/mm³。它们是分析假设，不是该打印件的实测性能。
- 1g=9806.65 mm/s²；1kg 在1g下产生9.80665N。
- 正权14点四面体积分（精度阶5）。体积/应力分位数按真实积分权重统计，p95/p99不是节点数量分位数。
- CG+弹性刚体近零空间 AMG，目标相对残差1e-9；零位移约束直接消元。保持同一节点xyz的3×3 BSR块。
- K和M按最多8000单元分块装配，减少稀疏临时矩阵内存；此分块不改变单元、积分或边界条件。全量/分块K一致性验证见 `validation/chunked_assembly_validation.json`。
- 源形状、网格、体积差、边界面积、自由度、平衡残差、能量、应力分区和单位载荷都存入结果JSON。

## 已完成的核心数值验证

`validate_core.py` 对10mm立方体施加1MPa单轴均匀拉伸，用分量滑移对称边界保持解析泊松变形。解析伸长0.005mm、应变能0.25N·mm、von Mises应力1MPa。实际结果与解析解的详细误差见 `validation/core_validation.json`。另检验了任意三维合力及合矩的一致面载荷离散平衡。

## 主要接口

```python
gmsh_mesh(source_brep, output_msh, hmax=2.0)
fe = SolidFE(output_msh, E=2000, nu=.35, density=1.24e-9)
fixed = fe.facets(lambda x: ...)   # x为边界三角面中心，shape=(3,n)
fe.assemble(fixed)                # 或 assemble_stiffness() 得到无约束 K
f, load_info = fe.load_force_at(load_facets, F=(1,0,0), point=(0,0,100))
u, solve_info = fe.solve(f)
response, fields = fe.summarize(u, regions=..., return_fields=True)
fe.export_vtu(path, u)
```

`load_wrench` 用连续面力 `F/A + b×(x−c)` 同时保持指定合力与合矩。它是远程载荷的静力等效分布，不把该面强制成刚性平面；因此不能解释为已求解的实际螺钉、接触压力或绳子局部包角。

`assemble_stiffness()`、`mass_matrix()`、`K`、`basis` 可由其他分析程序构造模态、附加质量或线性约束。`evaluate_solution(f,u)` 可核对单位解线性叠加后的能量与反力。此目录的静力结果不自行代表模态、屈曲或动态碰撞结论。

## 挂绳受力路径

`mount_fea/cartridge` 的 PRE_BREAK 案例固定下燕尾，属于未断开的完整卡匣。`cartridge_postbreak` 则解除下燕尾约束，改固定相机侧承载板/柱接触片，检查局部挂绳耳。后者仍保留未切割源CAD，固定接口作为剩余相机模块惯性反力的理想化边界；它不是实际断裂过程或落绳冲击模拟。

负X挂绳载荷同时提供完整孔壁均布和受力侧半孔壁案例。±Z是孔壁轴向等效面剪力。由于真实绳材、结法、包角和边缘接触未定义，二者都只能作为声明边界条件下的局部弹性响应，不能据此保证防坠或给出实测承载额定值。

## 适用范围

未求解层间各向异性、打印缺陷、塑性、蠕变、疲劳、裂纹扩展、预紧、摩擦、间隙闭合、脱离或冲击。原有尖角、薄边及理想固定边界可能产生网格相关峰值；保留原始峰值并同时提供体积加权p95/p99与受力区域指标。实体应力云图是计算结果，但不等于制造件性能认证。