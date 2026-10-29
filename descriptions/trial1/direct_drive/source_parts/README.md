# 原始支架转换记录

长U支架.step 由 descriptions/support_frames/长U支架.SLDPRT 独立转换，记录见 conversion.json；使用原始输入，不依赖其他手部设计。转换器为 [sldprt2step](https://github.com/BlinkingSun/sldprt2step)，本次使用提交 fa954124f4f126e0c465c8f96454978ac80b812e。转换器报告该零件无近似警告；仍需在实际 CAD 和零件上核对。

双轴舵机本体和两个舵盘从 descriptions/support_frames/金属舵机最小单元-双轴.STEP 原始文件读取。build_cad.py 不需要重新转换 SLDPRT，直接读取这里的长 U STEP 与原始舵机 STEP。
