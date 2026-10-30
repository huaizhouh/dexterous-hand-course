# 来源与改动记录

1. 用户提供的 `dexterous-hand-course-spontaneous.zip`，并核对课程仓库：
   https://github.com/huaizhouh/dexterous-hand-course
   核对时仓库最新提交：`3c1131ba58a3aa5a3841445622ebc9e59e31f8d2`。
   `descriptions/trial3` 在附件及在线目录中仅含 `aero-hand-open-main.zip`；实际旧 CAD 参照为 `trial2` 与 `new`。本包新写三指 CAD，不宣称旧模型已经正确，也没有修改原始历史方案。
2. 课程成果展示 PDF，共 8 页；第 2-4 页为腱绳驱动展示。借鉴其掌下执行机构和腱绳驱动布局，不根据展示照片推断未经给定的精确尺寸。
3. 老师舵机 `双轴舵机-260528.stp`，以及后补 `双轴舵机-260528.SLDPRT`、`长u支架.SLDPRT`、`短u支架.SLDPRT`。本目录保存的是原始文件的未改动副本，哈希见 `input_hashes.json`。
4. CAD 直接读取老师 STEP 的 8 个原始实体。其中体积排序第 0、1、4、7 为舵机外壳/轴组件，第 5、6 为两个舵盘；第 2、3 为附带 U 支架，在本版整机中不装入。没有把附带 U 支架误当舵机外壳去做夹座。
5. 原 STEP 的体积排序第 4 个实体拓扑无效，体积约 2725.054399 mm³。使用 OCCT ShapeFix 修复导入副本后为有效实体，体积约 2725.054527 mm³，变化约 0.000128 mm³；原始文件未修改。原 STEP 本身没有电气规格，也不能替代实物测量。
6. 补充 U 支架 SLDPRT 的只读尺寸提取使用旧课程包内自带的开源 sldprt2step 工具。其提示从导入特征恢复实体，不能视为 SolidWorks 完整特征树重建。提取尺寸只用于参考，未使用转换后的支架实体制造本版手指，也未把转换工具重新打包。
7. Aero Hand Open：
   https://github.com/Chestnut-Robotics/aero-hand-open
   https://docs.tetheria.ai/docs/assembly/
   参考用户包中随附的装配指南及 BOM，特别是独立回位元件、分段装配、止退绳结、卷线固定与张紧检查。原版是人形多指、专用伺服与金属回位弹簧，本版是三指两关节、现有总线舵机与闭环皮筋，没有复用其专用 PCB/固件或完整指节几何。
8. 本版 `build_hand.py` 的打印件由独立解析几何重新生成。源硬件实体属于老师提供的文件；课程控制脚本保持原样，权利和来源仍属于各自原作者。

本工程以课程学习和个人原型为用途。Aero Hand 仓库将其设计文件声明为 CC BY-NC-SA 4.0；如果后续直接引入其设计几何或发布相应衍生设计，应保留原项目许可与署名，商业制造另按原项目要求处理。本包的来源说明不授予原作者未授予的权利。

SolidWorks STEP 导入说明（菜单因版本而异）：
https://help.solidworks.com/2024/english/SolidWorks/sldworks/t_reading_step_iges_acis_sw.htm
