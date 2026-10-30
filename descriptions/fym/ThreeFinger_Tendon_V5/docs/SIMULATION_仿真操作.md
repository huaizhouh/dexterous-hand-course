# V5 仿真：按顺序操作

## 1. 解压整个工程

把 ZIP 解压到一个固定目录，例如 `D:\Hand\ThreeFinger_Tendon_V5`。根目录应同时有 `run_v5_sim.py`、`source`、`print_stl`、`simulation`、`checks`。不要只取出一个脚本，也不要放回旧 V4 文件夹运行。

## 2. 打开 Anaconda Prompt

从 Windows 开始菜单打开 Anaconda Prompt。若已有课程 `newton` 环境，执行：

```bat
conda activate newton
```

若没有这个环境，只需先创建一次，再激活：

```bat
conda create -n newton python=3.11 -y
conda activate newton
```

本脚本使用 MuJoCo，环境名沿用课程即可；不需要另外安装 Newton。

## 3. 进入解压后的根目录

把下面路径替换成你电脑的实际路径：

```bat
cd /d D:\Hand\ThreeFinger_Tendon_V5
dir run_v5_sim.py
python -m pip install -r requirements_sim.txt
```

`dir` 应找到该文件。安装只需首次进行；脚本在 Python 3.11 / MuJoCo 3.3.7 下测试。不需要安装 CADQuery，也不需要 SolidWorks 正在运行。

## 4. 先看无物体动作

```bat
python run_v5_sim.py --mode motion --cycles 1
```

观察预弯时指根先动、第二关节较少弯，最终再包覆，松绳后回位。窗口内鼠标左键拖动旋转视角，滚轮缩放，P 暂停，R 重置，Esc 退出。

## 5. 看乒乓球抓取

```bat
python run_v5_sim.py --object pingpong --cycles 1
```

完整周期 18 秒：

| 仿真时间 | 动作 |
|---|---|
| 0-1 s | 高位等待 |
| 1-3 s | 预弯收绳 |
| 3-5 s | 保持预弯并下降接近物体 |
| 5-8 s | 缓慢收至最终绳量 |
| 8-9 s | 贴合稳定 |
| 9-11 s | 整只手抬升 70 mm |
| 11-13 s | 保持，检查物体是否独立离开支撑 |
| 13-15 s | 回落 |
| 15-17 s | 松绳并上撤 25 mm，给手指回弹空间 |
| 17-18 s | 等待复位 |

仿真时间与电脑墙钟时间可能不同。红色为腱绳，黄色为等效回弹路径，蓝绿色为软垫。并联皮筋在 CAD 显示两圈，在仿真中合为一个等效回弹元件。

## 6. 逐个换物体

```bat
python run_v5_sim.py --object sphere --cycles 1
python run_v5_sim.py --object cube --cycles 1
python run_v5_sim.py --object cylinder --cycles 1
python run_v5_sim.py --object small_sphere --cycles 1
python run_v5_sim.py --object capsule --cycles 1
```

`sphere` 为直径 60 mm 球，`cube` 为 40 mm 方块，`cylinder` 为直径 40、高 50 mm 圆柱，`small_sphere` 为直径 10 mm 小球，`capsule` 为直径 20、长 70 mm 的竖放圆头细长体。小球按 V4 场景放在高支撑上，不代表已证明能从平桌面拾起 10 mm 球。细长体保留失败记录。

## 7. 自动检查并复现对照

不打开窗口，检查回弹、单指独立性以及六种物体居中抓取：

```bat
python run_v5_sim.py --check --object all
```

原始 XML 和 JSON 写入自动生成的 `v5_sim_output`。`full_cycle_success` 才是完整周期判断；失败退出码为 2，程序报错退出码为 1。整批出现已记录的细长体失败不等于安装坏了。

七种位置完整对照（需要几分钟，视电脑性能而定）：

```bat
python simulation/run_benchmarks.py --design both --full
```

结果在 `checks/benchmarks/v4_summary.json` 和 `v5_summary.json`。七个位置为居中、X/Y 各正负 1 mm，以及两组正负 2 mm 对角偏移。物体大小、质量、支撑、转矩上限和摩擦保持一致，控制策略与新版机械接触面改变。V4 额外补入乒乓球测试，不把它算成原来的五种。

## 8. 手动调动作（可选）

```bat
python run_v5_sim.py --manual --object pingpong
```

O 松绳；C 收至该物体预设终点；1/2/3 选择单指、0 选择全部；上/下键每次增减 0.5 mm 收绳量；PageUp/PageDown 每次升降手的夹具 1 mm；R 恢复初始状态。手动模式不会自动下降和抬升，要自己操作高度。

命令行临时覆盖收绳量示例：

```bat
python run_v5_sim.py --object pingpong --takeup-mm 9 --cycles 1
```

正式预设在 `simulation/grasp_profiles.json`：`pre_mm`、`final_mm` 是 A/B/C 三指卷线目标，`base_m` 是虚拟夹具抓取高度，`approach_m` 是接近净空；单位与实物 PWM 不同。

## 常见问题

- 找不到 `mujoco`：检查当前环境，重跑第 3 步的 pip 命令。
- 找不到网格或 JSON：完整解压，确认执行的是 V5 根目录脚本。
- 窗口启动失败：先运行 `python run_v5_sim.py --headless --object pingpong` 检查数值仿真，再检查本机显卡/OpenGL；无窗口模式通常可用。
- 画面中手在移动：这是测试夹具移动整只手。物体是独立自由体，只受重力和接触，没有绑定。
- 想运行导出的 XML：XML 不含 Python 中绳张力与卷线轮反力耦合，必须运行 Python 脚本。

## 重新生成 CAD（只有改几何才需要）

在单独 CAD 环境安装 `source/requirements_cad.txt`，然后在根目录依次运行：

```bat
python -m pip install -r source/requirements_cad.txt
python source/build_hand.py
python source/validate_design.py
python source/check_fasteners.py
```

修改软垫后必须重建，以同步 `source/pad_collision.json` 与 STL/STEP。普通仿真无需执行这些命令。原始老师文件需要保留在 `reference`；生成步骤不会修改它们。
