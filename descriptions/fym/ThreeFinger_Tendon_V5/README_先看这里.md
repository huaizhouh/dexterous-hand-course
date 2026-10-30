# 三指腱绳灵巧手 V5

本版按你的要求优化皮筋回弹分配、分段收绳和小物体指垫。**三指根位置固定、根分布半径 65 mm；每指总长仍为 110 mm（62+48 mm）**，没有可调指距机构，没有更换舵机或增加驱动力。

## 先打开哪个文件

解压整个文件夹，双击根目录 **`00_OPEN_IN_SOLIDWORKS.step`**。没有文件关联时，用 SolidWorks「文件 → 打开」选择它，单位毫米、比例 1:1，按 F 查看全图。

需要颜色与零件层级时，打开 `cad/OPEN_ASSEMBLY.step`。另外两个 STEP 为预弯和包覆的静态参考姿态。交付包含整机 STEP、单件 STEP、打印 STL 和参数化 Python CAD 源码；STEP 不含 SolidWorks 原生特征树、运动配合或自动变形皮筋。老师原始 SLDPRT/STP 在 `reference` 中。

## V5 修改内容

- 每指根部一条较软回弹环，第二关节两条并联；全手 9 条皮筋。
- 第二关节挂点向背侧外移 3 mm。挂点细颈直径 3.2 mm、宽 6 mm、帽径 7.2 mm，可并排容纳两条窄环。
- 收绳分为预弯和最终包覆，再抬升；先让指根靠近物体，减轻第二关节提前卷起的问题。
- 指垫改为浅 V 槽，指端收窄并加柔性挡边，减少轻小球沿指长方向滑出。
- 仿真中保留三路真实卷线转动、单向绳张力和六个被动关节；没有直接控制关节角、黏住物体或抬升物体脚本。

## 运行仿真

根目录运行 **`run_v5_sim.py`**。详细的 Windows/Anaconda 操作和逐条命令见 `docs/SIMULATION_仿真操作.md`。

```bat
conda activate newton
python -m pip install -r requirements_sim.txt
python run_v5_sim.py --object pingpong --cycles 1
```

先看纯动作与回弹：`python run_v5_sim.py --mode motion --cycles 1`。

自动检查六种物体：`python run_v5_sim.py --check --object all`。失败项会如实报告并返回退出码 2。完整 V4/V5 对照：`python simulation/run_benchmarks.py --design both --full`。

最终统计和失败原因见 `docs/TEST_RESULTS_仿真对照.md`。它们是给定参数下的确定性仿真样本，不是已经打印和实测的成功率。

## 文件导航

| 路径 | 用途 |
|---|---|
| `cad` / `parts_step` | 整机参考姿态 / 单件实体 |
| `print_stl` | 14 类打印件，40 个装配件；数量见 BOM |
| `source/build_hand.py` | 用解析几何重新生成 CAD |
| `source/parameters.json` | 固定尺寸、挂点及软垫参数 |
| `source/pad_geometry.py` / `pad_collision.json` | CAD 与仿真共用的软垫几何 |
| `simulation/grasp_profiles.json` | 各物体预弯、最终收绳量与虚拟夹具高度 |
| `simulation/run_benchmarks.py` | 同物体、同转矩和摩擦参数的对照测试 |
| `checks/benchmarks` | 每次抓取的关节角、接触、张力与物体轨迹记录 |
| `comparison/baseline_v4` | V4 原脚本、原网格与参数，用于复现基线 |
| `docs` | 装配、BOM、仿真步骤、测试统计及实物验收空表 |
| `hardware_control` | 原课程舵机通信程序与 V5 标定说明 |
| `preview` | 实际 CAD 预览和仿真画面 |
| `reference` | 老师原始硬件模型、来源及哈希 |

## 实物开始前

先试打一根手指，按 `docs/ASSEMBLY_装配与穿绳.md` 装配。仿真中的皮筋刚度、摩擦、质量、转矩上限和舵机响应都是未实测假设；不能从 STEP 获得真实舵机规格。先标定零点、收绳方向和毫米与 PWM 的关系，再小步试夹。预设收绳量不能直接作为 PWM 下发。

仿真抓取由虚拟夹具移动整只手。实物抬升由人手或已有工装完成，工程中没有新增升降机构。
