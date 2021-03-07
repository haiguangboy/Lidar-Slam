# 第五章作业

深蓝学院, 多传感器融合定位与建图, 基于第5章IMU Calib代码框架.

## Overview

本作业旨在加深对不依赖转台的标定方法的理解.

## 内容

1. ### 雅克比推导

   ![雅克比推到](doc/pictures/雅克比推到.jpg)

   ![雅克比推到3](doc/pictures/雅克比推到3.jpg)

   ![雅克比推到2](doc/pictures/雅克比推到2.jpg)

2. ### 解析式求导

   2.1构建参数矩阵以及残差函数

![构建残差函数](doc/pictures/构建残差函数.png)

2.2 解析式的雅克比求导



![雅克比解析式求导](doc/pictures/雅克比解析式求导.png)

![雅克比解析式求导2](doc/pictures/雅克比解析式求导2.png)

2.3更新CostFunction

 ![雅克比解析式求导4](doc/pictures/雅克比解析式求导4.png)

2.4 构建下三角矩阵

![雅克比解析式求导3](doc/pictures/雅克比解析式求导3.png)

## 3.运行程序结果

![结果4](doc/pictures/结果4.png)