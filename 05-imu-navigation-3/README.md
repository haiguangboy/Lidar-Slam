# Sensor Fusion: Lidar Odometry -- 多传感器融合定位与建图: 惯性导航原理

深蓝学院, 多传感器融合定位与建图, 第6章IMU Navigation代码框架.

## Overview

本作业旨在加深对**惯性导航解算**的理解.

本次作业写了很长时间，但是收货也是最大的，感觉通过这次作业把前面的好几章学过的内容都串了起来，以前只是知识点，现在成了线和面，收货很大。

用于误差评价数据的保存参考了第二章的作业，数据的生成参考了作业中的数据生成格式。还有一个小遗憾就是误差评价的数据ground_truth的显示或者加载有点问题，后期有时间再改进一下。

关于作业的代码填写如下：



```bash
// get deltas:
        const size_t index_curr = 1;
        const size_t index_prev = 0;
        Eigen::Vector3d angular_delta;
        Eigen::Matrix3d R_curr;
       Eigen::Matrix3d R_prev;
        Eigen::Vector3d velocity_delta;
        double delta_t;


        GetAngularDelta( index_curr, index_prev,angular_delta);

        // update orientation:

        // get velocity delta:
        GetVelocityDelta( index_curr, index_prev,R_curr, R_prev, delta_t, velocity_delta);

         UpdateOrientation(angular_delta,R_curr, R_prev);

        // update position:
        UpdatePosition(delta_t,velocity_delta );

        // move forward -- 
        
        // NOTE: this is NOT fixed. you should update your buffer according to the method of your choice:
        imu_data_buff_.pop_front();
```

```bash
    #分别算了中值法和欧拉法
    Eigen::Vector3d angular_vel_curr = GetUnbiasedAngularVel(imu_data_curr.angular_velocity);
    Eigen::Vector3d angular_vel_prev = GetUnbiasedAngularVel(imu_data_prev.angular_velocity);
   #基于中值法
     angular_delta = 0.5*delta_t*(angular_vel_curr + angular_vel_prev);
    #基于欧拉法
      angular_delta = delta_t*angular_vel_prev;

    return true;
```

分别算了中值法和欧拉法得出的结果：

中值法：

![中值法](doc/images/结果-中值.png)

欧拉法：

<img src="doc/images/oula.png" alt="结果-欧拉法" style="zoom:50%;" />

精度分析对比：参考第二章的误差评价程序修改而来。

中值法

![med精度对比-raw](doc/images/med精度对比-raw.png)

![med精度对比-map](doc/images/med精度对比-map.png)

![med精度对比-map](doc/images/med精度对比-方差.png)

欧拉法

![ape-oula精度对比-raw](doc/images/ape-oula精度对比-raw.png)

![ape-oula精度对比-map](doc/images/ape-oula精度对比-map.png)

![ape-oula精度对比-raw](doc/images/allen方差数据.png)

### 自己建立仿真数据

![biange](doc/images/zhongzhi/biange.png)

结果分析

全部数据用中值法估算位姿

<img src="doc/images/zhongzhi/medquan0.png" alt="medquan0" style="zoom:50%;" />

<img src="doc/images/zhongzhi/medquan1.png" alt="medquan1" style="zoom:50%;" />

<img src="doc/images/zhongzhi/medquan2.png" alt="medquan2" style="zoom:50%;" />

误差分析

![medquan](doc/images/oula/oula-quan.png)

欧拉法位姿估算

全程

<img src="doc/images/oula/oula-quan0.png" alt="oula-quan0" style="zoom:50%;" />



<img src="doc/images/oula/oula-quan2.png" alt="oula-quan2" style="zoom:50%;" />



<img src="doc/images/oula/oula-quan3.png" alt="oula-quan3" style="zoom:50%;" />

误差分析

![oula-quan](doc/images/zhongzhi/medquan.png)

## 分段统计

### 静止状态

#### 中值法

![jingzhi](doc/images/zhongzhi/jingzhi.png)

<img src="doc/images/zhongzhi/jingzhi2.png" alt="jingzhi2" style="zoom:50%;" />

#### 欧拉法

![jingzhi](doc/images/oula/jingzhi.png)



<img src="doc/images/oula/jingzhi2.png" alt="jingzhi2" style="zoom:50%;" />

### 匀速状态

#### 中值法

![yunsu](doc/images/zhongzhi/yunsu.png)

<img src="doc/images/zhongzhi/yunsu2.png" alt="yunsu2" style="zoom:50%;" />

#### 欧拉法

![yunsu](doc/images/oula/yunsu.png)

<img src="doc/images/oula/yunsu2.png" alt="yunsu2" style="zoom:50%;" />

### 加速状态（0.2m2/s）

#### 中值法

![jiasu](doc/images/zhongzhi/jiasu.png)

<img src="doc/images/zhongzhi/jiasu2.png" alt="jiasu2" style="zoom:50%;" />

#### 欧拉法

<img src="doc/images/oula/jiasu.png" alt="jiasu" style="zoom:50%;" />

<img src="doc/images/oula/jiasu2.png" alt="jiasu2" style="zoom:50%;" />

### 减速状态（-0.3m2/s）

#### 中值法

![jianshu](doc/images/zhongzhi/jianshu.png)

<img src="doc/images/zhongzhi/jianshu2.png" alt="jianshu2" style="zoom:50%;" />

#### 欧拉法

<img src="doc/images/oula/jiansu.png" alt="jiansu" style="zoom:50%;" />



<img src="doc/images/oula/jiansu2.png" alt="jiansu2" style="zoom:50%;" />

### 转弯状态

#### 中值法

<img src="doc/images/zhongzhi/zhuanwan.png" alt="zhuanwan" style="zoom:50%;" />

<img src="doc/images/zhongzhi/zhuanwan2.png" alt="zhuanwan2" style="zoom:50%;" />

<img src="doc/images/zhongzhi/zhuanwan3.png" alt="zhuanwan3" style="zoom:50%;" />

#### 欧拉法

<img src="doc/images/oula/zhuanwan.png" alt="zhuanwan" style="zoom:50%;" />

<img src="doc/images/oula/zhuanwan2.png" alt="zhuanwan2" style="zoom:50%;" />



<img src="doc/images/oula/zhuanwan3.png" alt="zhuanwan3" style="zoom: 50%;" />

### 结论与分析：

首先，好像有什么问题导致ground_truth的线没有显示出来或者是被重合了，如果单看laser_odom的绿线的话，基本上中值法和欧拉法差别不大，但是减速状态欧拉法比中值法好一些，转弯状态中值法要好于欧拉法，匀速，静止都差不多。从全程的数据来看，中值法的误差要比欧拉法好，中值法的误差最大值和均值都比欧拉法小，这也与实际的感受一致。