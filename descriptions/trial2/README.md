# Trial 2：三指、每指两关节的腱绳灵巧手

本目录从原始双轴舵机 STEP 和重新编写的建模程序独立生成。三个手指间隔 120°，每指两个被动屈伸关节，由一根独立腱绳牵引、两只扭簧复位；总计 **6 个手指自由度、3 路驱动**。指间没有固定角度比例，接触物体后两个关节可以重新分配弯曲角。掌下卷线轮通过方轴套与原始舵机主舵盘连接，绳索通过掌内导向通道、指间导孔和指垫内衬到达远端锚点。

本设计按“每指两个自由度”解释任务中的“三指两自由度”。新增件使用本目录内的独立源码生成，原始输入仅为 `descriptions/support_frames/金属舵机最小单元-双轴.STEP`，来源 SHA-256 见 [来源记录](reports/source.json)。

## 打开设计

| 文件 | 用途 |
| --- | --- |
| [张开总装，多实体单零件 STEP](tendon_hand_open_multibody.step) | SolidWorks 首次导入优先使用，单位 mm，一个产品节点含全部实体 |
| [张开总装，装配 STEP](tendon_hand_open.step) | 保留装配实例结构，具体拆分行为取决于 STEP 导入选项 |
| [夹持参考姿态 STEP](tendon_hand_grasp.step) | MCP 9°、PIP 60° 的静态参考姿态，不应与张开模型叠放 |
| [单独零件目录](parts/) | 所有新增件及标准件简化模型的单独 STEP |
| [零件与材料清单](PARTS.md) | 数量、材料、紧固件和加工要求 |
| [装配步骤](ASSEMBLY.md) | 轴、弹簧、卷线轮和穿绳顺序 |
| [装配变换](assembly.json) | 各部件的实例名称与变换矩阵，单位 mm |

STEP 是实体几何，不包含 SolidWorks 原生特征树、运动配合或驱动。多实体版适合查看整机；需要在 SolidWorks 中运动时，应按 `design.py` 中的轴线和装配关系另加转动配合。标准螺纹以光杆、底径和孔代替；扭簧使用安装包络表示，腿部和实际绕丝形状未建模。它们需要按清单的尺寸及性能要求选型，不应把包络直接打印成弹簧。

主要尺寸：手掌外径 200 mm、中心通孔 50 mm、基板厚 5 mm；指根分布半径 60 mm，轴高 15 mm；近端两轴中心距 45 mm，远端指节长约 36 mm。铰链轴径 3 mm，孔径 3.2 mm；槽内使用轴套为回弹元件保留空间。其余加工尺寸以单件 STEP 为准。

## 运行仿真

从仓库根目录执行：

```bash
conda activate newton
mjpython descriptions/trial2/run_hand.py
```

演示每 20 秒完成张开、收绳夹球、抬升、放回和松绳张开，下一周期重置测试球的位置。也可直接加载 `hand.xml` 查看模型；完整的绳索松弛逻辑由 `run_hand.py` 执行。

```bash
mjpython descriptions/trial2/run_hand.py --check
```

检查涵盖：三指独立牵引、三个指垫同时接触球体、抬升、返回和回弹张开，以及绳长约束误差。数值结果保存在 [仿真报告](reports/simulation.json)。场景中的升降轴、托盘、立柱、地面和球体是测试夹具，不是灵巧手上的附加驱动机构。

仿真使用与 CAD 相同的零件 STL、关节坐标及绳路。实际施加驱动的是掌下卷线轮关节，通过 `L - L0 = -r·theta` 的张紧绳约束带动两个手指关节；程序在反力变成推力时解除约束，绳路再次拉紧时恢复，避免用绳索推开手指。MuJoCo 的 tendon equality 使用相对初始绳长的变化，参见 [官方 XML 说明](https://mujoco.readthedocs.io/en/stable/XMLreference.html#equality-tendon)。

当前假设：球体半径 22 mm、质量 30 g；接触摩擦系数 1.2；卷线轮有效半径 7.8 mm、驱动扭矩限值 0.075 N·m；MCP/PIP 回弹刚度分别为 0.06/0.05 N·m/rad。这些是设计与仿真假设，尚非实测舵机或弹簧数据。骨架使用胶囊体接触近似，指垫使用与 CAD 相近的圆头包络，掌板使用保留中心开口的碰撞近似；视觉模型使用实际 CAD 网格。未模拟绳索伸长、导管摩擦、打印变形、关节磨损和电气控制延迟。

## 验证范围

[CAD 报告](reports/cad.json) 检查每个部件和交付总装，完整 STEP 会重新导入并验证实体有效性。[干涉报告](reports/interference.json) 对六组角度作实体相交抽样，包含腱绳和导向结构；阈值为 0.03 mm³。原始舵机轴与舵盘的配合重叠以及绳端与绳结的连接单独声明，不当作自由空间干涉。抽样检查不证明整个二维关节范围处处无碰撞；过度同时弯曲会使三指接近中心，应保留自接触检测和行程限制。

尚未在本机 SolidWorks 或实物中打开、装配和承载测试。打印公差、嵌件固定、扭簧选型及实际舵机行程仍需单指试装验证；该交付是可导入、可修改、带完整仿真的工程原型设计，不是经加工验证的生产图纸。

## 重建

CAD 依赖列于 `requirements-cad.txt`，仿真依赖列于 `requirements-sim.txt`。在另行准备的 Python CAD 环境中运行：

```bash
python descriptions/trial2/build_cad.py
python descriptions/trial2/validate_cad.py
python descriptions/trial2/build_sim.py
```

`design.py` 是共享尺寸和绳路定义。修改尺寸后应依次重建 CAD、重新检查干涉并更新 MJCF；仅改变仿真控制时不需要重新生成 STEP。预览可用 `mjpython descriptions/trial2/render_preview.py` 生成。
