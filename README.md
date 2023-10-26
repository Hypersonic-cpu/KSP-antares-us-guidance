# Antares-US-guidance
A second stage guidance for Antares 230 in KSP via kRPC.

一个比较粗糙的制导，在俯仰通道中控制近地点与远地点。发射窗口等可以交由 MechJeb 计算。

### User Guide
将目标点的近地点高度、远地点高度(m)输入 Target.txt，替换文件原有的示例。

一级可以由 MechJeb 等任意方式制导，程序运行后将在一二级分离后开始控制。此时需要关闭其他制导。

### About Project
C__for_KRPC 是主控制程序，GuidanceCmd 负责制导命令的计算以及轨道力学等一些 class definition. GuidanceMath 支持行列式、向量的简单计算。

### To-do:
* 更改制导方式，使用 double arc 或者类似的稳定方式。
* 增加对近地点幅角的控制。
* 
