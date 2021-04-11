# Sensor Fusion: Lidar Odometry -- 多传感器融合定位与建图: 基于图优化的定位方法

深蓝学院, 多传感器融合定位与建图, 第10章Graph Optimization for Localization through Sliding Window代码框架.

---

## Overview

本作业旨在加深对**基于图优化的定位, 滑动窗口, 方法**的理解.

补全基于滑动窗口的融合定位方法的实现, 并分别与:

* 不加融合
* EKF融合

的效果做对比.

---

## 基础部分

在参考群里发的葛大佬的作业文件后跑的结果：

推到残差与雅克比见后部分。

参考葛大佬提示的全局地图截图：

<img src="doc/images/global-4.png" alt="global-4" style="zoom:50%;" />

自己根据上一章程序（还有一些小问题）修改后跑出来的全局地图截图：

<img src="doc/images/global-1.png" alt="global-1" style="zoom:50%;" />

evo评估：

通过与上一章的没加滑窗，或者说将其看作一个超大的滑窗，只优化了一次，下面将上一章的实验结果再次放出来进行对比：

说明：这里放lidar的图主要是做一个参照，因为在跑程序的过程中发现有时候得出的结果会偏离的特别大，这种情况下往往lidar的结果也跟着一起大幅偏离。以lidar做一个参考说明都是在正常情况下的得出的数据。

|                 | sliding-window                                               | one-window                                                   |
| --------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| laser_traj      | <img src="doc/images/4laser-1.png" alt="4laser-1" style="zoom: 25%;" /> | <img src="doc/images2/3lsaer_1.png" alt="3lsaer_1" style="zoom: 50%;" /> |
| laser_error     | <img src="doc/images/4laser-2.png" alt="4laser-2" style="zoom:50%;" /> | <img src="doc/images2/3lsaer_2.png" alt="3lsaer_2" style="zoom: 50%;" /> |
| optimized_traj  | <img src="doc/images/4op-1.png" alt="4op-1" style="zoom:33%;" /> | <img src="doc/images2/3opt_1.png" alt="3opt_1" style="zoom: 25%;" /> |
| optimized_error | <img src="doc/images/4op-2.png" alt="4op-2" style="zoom:50%;" /> | <img src="doc/images2/3opt_2.png" alt="3opt_2" style="zoom: 60%;" /> |

**sliding-window实验分析：**optimized后的评价参数基本全面超过了laser的，只有均值和中位数没有略有落后，标准差，累计误差，最大值都优于laser的，说明滑窗法在融合的基础上不断的更新数据更好的抑制了误差，消除了矛刺（最大值）以及累计误差。

**有无sliding-window对比分析：**通过对optimized的轨迹图对比可以发现：滑窗法的轨迹结果消除了波浪型翻褶，这是其不断更新参数的一个优势，而只优化一次的融合则存在这样的问题。

其二就是误差参数滑窗的结果也比滤波的要好，尤其是累计误差、标准差变为了滤波的1/3，最大值和均值变为了滤波的1/2, 这说明了滑窗的步长设置对结果有着非常大的影响。

### sliding-window与EKF的对比：

|             | sliding-window                                        | EKF                                                          |
| :---------: | ----------------------------------------------------- | ------------------------------------------------------------ |
| laser_error | <img src="doc/images/4laser-2.png" alt="4laser-2"  /> | ![l-posi](/home/yhg/Documents/Lidar-Slam/Lidar-Slam/09-sliding-window/doc/images2/l-posi.png) |
|  opt_error  | <img src="doc/images/4op-2.png" alt="4op-2"  />       | ![f-posi](/home/yhg/Documents/Lidar-Slam/Lidar-Slam/09-sliding-window/doc/images2/f-posi.png) |

通过对比可以看出：EKF的整体效果要比sliding-window的更优。

## 改变滑窗步长

然后我们再次改变滑窗的步长来看其最优值究竟在哪个范围。

原来默认的是sliding_window_size: 20、5（1/4X）、 10（1/2X）、40（2X）、60（3X）

数据的统计结果进行了如下汇总，具体的详细截图见后面。

误差最大值、均值和标准差在不同滑窗步长下的对比：

![data1](doc/images/data1.png)

累计误差因为数值较大，故单独展示。

累计误差在不同滑窗步长下的对比：

![data2](doc/images/data2.png)

结论：从上面的2幅图可以看出，横轴window-size在20和40处较小的误差，综合来看window-size在20处较小的值更多一些，故暂且认为在该模型框架下window-size为20是综合最优步长。



| size |             laser_odom             |            optimized            |
| :--: | :--------------------------------: | :-----------------------------: |
|  5   |  ![5laser](doc/images/5laser.png)  |  ![5laser](doc/images/5op.png)  |
|  10  | ![10laser](doc/images/10laser.png) |  ![10op](doc/images/10op.png)   |
|  20  | ![4laser](doc/images/4laser-2.png) |  ![4op](doc/images/4op-2.png)   |
|  40  | ![40laser](doc/images/40laser.png) | ![40laser](doc/images/40op.png) |
|  60  | ![60laser](doc/images/60laser.png) | ![60laser](doc/images/60op.png) |

部分问题发现：

目前还没找到解决办法，有可能是地图的问题，有可能是算法的问题。

|                                 | 问题路段                                                     |
| ------------------------------- | ------------------------------------------------------------ |
| 雷达和imu数据引起优化结果的跳跃 | <img src="/home/yhg/Documents/Lidar-Slam/Lidar-Slam/09-sliding-window/doc/images/wenti-map.png" alt="wenti-map" style="zoom:50%;" /> |
|                                 |                                                              |
| 优化引起的数据跳跃              | <img src="/home/yhg/Documents/Lidar-Slam/Lidar-Slam/09-sliding-window/doc/images/wenti-map4.png" alt="wenti-map4" style="zoom:50%;" /> |
| 雷达引起的数据跳跃              | <img src="/home/yhg/Documents/Lidar-Slam/Lidar-Slam/09-sliding-window/doc/images/wenti-map5.png" alt="wenti-map5" style="zoom:50%;" /> |

推到残差与雅克比：

地图匹配位姿和优化变量的残差雅可比



激光里程计相对位姿和优化变量的残差雅可比



IMU预积分和优化变量的残差雅可比



边缘化形成的先验因子对应的残差雅可比

