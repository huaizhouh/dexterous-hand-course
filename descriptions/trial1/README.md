# 三指舵机直接驱动灵巧手

## 方案与边界

本方案独立使用 `support_frames` 中原始双轴舵机 STEP 和长 U 支架 SLDPRT 转换结果，参考《灵巧手课程成果展示 - 2026年春.pdf》第 1 页的**舵机直接驱动**：三个手指沿圆周间隔 120°，每指由一只舵机直接转动长 U 支架，指垫贴在 U 支架内侧。每指 1 个主动转动关节，总计 3 个独立主动自由度。与展示图片相同的是直接驱动、径向夹持；手指外形改为现有长 U 支架，故不是对原作品外形或尺寸的逐项复刻。

新增设计只有手掌板 1 件和柔性指垫 3 件。舵机、主从舵盘及长 U 支架使用原始几何，补充数量和标准紧固件见 [补充零件清单.csv](补充零件清单.csv)。手掌直径 224 mm、底板厚 4 mm，中心通孔直径 74 mm，四个直径 4.4 mm 的安装孔按 90 mm 方形布置；三个舵机轴距中心 70 mm，轴线高度 20 mm。长 U 耳板采用每侧 4 个 M2 薄头螺钉及薄螺母连接舵盘；每舵机用两条 3 mm 宽扎带固定到支座。M2 杆穿过直径约 3 mm 的原孔，存在约 0.5 mm 径向定位间隙，安装时应对中。舵盘与实际舵机是否一致、螺母安装工具能否到达、扎带可靠性和舵机扭矩等尚需实物验证，不能单凭仿真保证抓取性能。

## CAD 导入

- [张开姿态，多实体单零件 STEP](direct_drive/direct_drive_open_multibody.step)：SolidWorks 打开整机时优先使用，单位 mm；一个产品节点、70 个实体。它不带原生特征树、关节配合或运动驱动。
- [张开姿态，装配 STEP](direct_drive/direct_drive_open.step)：70 个实例，具体文档拆分方式取决于 SolidWorks STEP 导入选项。
- [40° 姿态，装配 STEP](direct_drive/direct_drive_40deg.step)：同一结构的静态闭合姿态，勿与张开文件叠放。
- 新增件分别为 [手掌 STEP](direct_drive/palm.step) 和 [指垫 STEP](direct_drive/pad.step)。标准 M2 螺钉／螺母和扎带在总装中是简化实体；螺纹、扎带锁扣及指垫胶层未建模。
- [CAD 检查报告](direct_drive/cad_validation.json) 记录实体数、实例范围、体积和重叠；[运动检查报告](direct_drive/motion_validation.json) 对 0°、10°、20°、30°、40° 同步姿态进行了实体相交抽样，不代表连续全行程的证明。三个 STEP 均由 Open CASCADE 重读为 70 个有效实体；未在本机 SolidWorks 中实际打开。

原始舵机 STEP 的舵盘与舵机轴本来就有实体重叠：原始从舵盘约 16.27 mm³、主舵盘约 28.17 mm³。本方案将从舵盘沿轴向向内调整 0.05 mm，以避免原孔边缘与 U 支架的数值干涉，因此前者在总装中约 17.90 mm³；这部分按原组件的轴／舵盘配合处理，在运动报告中单独排除，不等于验证了物理配合公差。除该配合外，采样姿态中未发现大于 0.05 mm³ 的实体相交。轴向位置及舵盘螺钉、M2 薄螺母的实际可装配性仍需在实物确认。

## mjpython 仿真

在仓库根目录运行：

```bash
conda activate newton
mjpython descriptions/trial1/direct_drive/run_hand.py
```

仿真自动循环：张开、闭合夹球、升起、放回、张开。交互窗口使用的是上述 CAD 导出的各实体 STL 网格；指垫的接触形状与 CAD 方块一致。演示中的竖直滑动自由度和圆柱台是测试夹具，不是手掌内的新增驱动机构。

无界面验证也使用同一环境的 `mjpython`：

```bash
mjpython descriptions/trial1/direct_drive/run_hand.py --check
```

在本机 `newton` 环境中，该检查验证了三个指垫同时受力、球体被抬起后放回、手指重新张开，以及三指分别独立运动。演示球半径 30 mm、质量 50 g；接触摩擦系数 2，理想舵机扭矩限值 1 N·m，都是仿真假设而非实测参数。碰撞求解启用指垫、球、测试台和手掌的近似环形碰撞体；舵机与支架网格用作显示，因此仿真不替代 CAD 干涉检查，也不证明实物抓取或负载能力。图形窗口已使用 `newton` 环境的 `mjpython` 启动检查。

[张开预览](direct_drive/preview_open.png)、[夹持预览](direct_drive/preview_grasp.png)、[抬升预览](direct_drive/preview_lift.png) 由同一 MJCF 模型渲染。

## 修改与重新生成

`direct_drive/build_cad.py` 从源 STEP 生成整机、分件和仿真网格；`direct_drive/validate_motion.py` 重新导入整机，检查离散姿态并生成 40° STEP；`direct_drive/build_sim.py` 按同一坐标生成 `hand.xml`。仿真无需 CAD 内核。重新生成 CAD 需另装 `direct_drive/requirements-cad.txt` 中的 Open CASCADE Python 依赖，详见 [转换记录](direct_drive/source_parts/README.md)。

装配前建议先核对实物舵机轮廓及 4 个舵盘孔位、薄螺母间隙和接线余量，试装单指后再制作其余两指；控制范围、碰撞和抓取力均应从低速低负载逐步验证。
