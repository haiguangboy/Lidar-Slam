# Sensor Fusion: Lidar Odometry -- 多传感器融合定位与建图: 基于滤波的融合方法I

深蓝学院, 多传感器融合定位与建图, 第7章Filtering Basic代码框架.

---

## Overview

本作业旨在加深对**基于滤波的融合方法**的理解.

代码填充

```bash
//
  // TODO: perform Kalman prediction
  //
  X_ = F*X_ ;                                           // fix this   + B*w_b
  P_ =F* P_*F.transpose() + B*Q_*B .transpose() ; // fix this
}

/**
 * @brief  correct error estimation using pose measurement
 * @param  T_nb, input pose measurement
 * @return void
 */
void ErrorStateKalmanFilter::CorrectErrorEstimationPose(
    const Eigen::Matrix4d &T_nb, Eigen::VectorXd &Y, Eigen::MatrixXd &G,
                 Eigen::MatrixXd &K) {
  //
  // TODO: set measurement:
  //这是观测方程的参数
  //下面的更新有问题
  //Eigen::Matrix4f &pose;
  Eigen::Vector3d P_nn_obs =  pose_.block<3, 1>(0, 3) - T_nb.block<3, 1>(0, 3); // fix this
  Eigen::Matrix3d R_nn_obs =   T_nb.block<3, 3>(0,0) .transpose() * pose_.block<3, 3>(0, 0) ;//* ); // fix this


  YPose_.block<3, 1>(0, 0) = P_nn_obs;
  YPose_.block<3, 1>(3, 0) =  Sophus::SO3d::vee (R_nn_obs -Eigen:: Matrix3d::Identity() );//).transpose() ;
//pose_.block<3, 1>(0, 3);    vee为 反对称矩阵 到 向量  相当于下尖尖运算 
  Y = YPose_;

  // set measurement equation:
  G = GPose_;
  //C = CPose_;

  //有观测量时的更新
  // TODO: set Kalman gain:
  //
  MatrixRPose R = GPose_*P_*GPose_.transpose() + RPose_;// fix this  GPose_*P_*GPose_.transpose() +
  K =  P_ * G .transpose()*(G *P_ * G .transpose() + R ).inverse();                 // fix this
  
}

/**
 * @brief  correct error estimation
 * @param  measurement_type, measurement type
 * @param  measurement, input measurement
 * @return void
 */
void ErrorStateKalmanFilter::CorrectErrorEstimation(
    const MeasurementType &measurement_type, const Measurement &measurement) {
  //
  // TODO: understand ESKF correct workflow
  //
  Eigen::VectorXd Y;
  Eigen::MatrixXd G, K;
  switch (measurement_type) {
  case MeasurementType::POSE:
    CorrectErrorEstimationPose(measurement.T_nb, Y, G, K);
    break;
  default:
    break;
  }

  //
  // TODO: perform Kalman correct:
  //

  P_ =  (MatrixP::Identity() - K*G)*P_ ; // fix this
  X_ = X_+ K*(Y - G * X_);              // fix this
}

/**
 * @brief  eliminate error
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::EliminateError(void) {
  //
  // TODO: correct state estimation using the state of ESKF
  //
  // a. position:
  pose_.block<3, 1>(0, 3) -= X_.block<3, 1>(INDEX_ERROR_POS, 0); // fix this
          
  // b. velocity:
  vel_ -= X_.block<3, 1>(INDEX_ERROR_VEL, 0);; // fix this
  // c. orientation:
  Eigen::Matrix3d C_nn = Eigen::Matrix3d::Identity() -
      Sophus::SO3d::hat(X_.block<3, 1>(INDEX_ERROR_ORI, 0)).matrix();
  pose_.block<3, 3>(0, 0) = pose_.block<3, 3>(0, 0) *C_nn ; // fix this

  // d. gyro bias:
  if (IsCovStable(INDEX_ERROR_GYRO)) {
    gyro_bias_ -= X_.block<3, 1>(INDEX_ERROR_GYRO, 0);
  }

  // e. accel bias:
  if (IsCovStable(INDEX_ERROR_ACCEL)) {
    accl_bias_ -= X_.block<3, 1>(INDEX_ERROR_ACCEL, 0);
  }
}
```

## 运行结果

* **黄色**轨迹为**GNSS Localization**, 此处用作**Ground Truth**

* **蓝色**轨迹为**ESKF Fused Estimation**

![角度偏离](doc/images/角度偏离2.png)

### 误差分析

#### Laser与Ground Truth的轨迹对比：

![4误差评价2](doc/images/4误差评价2.png)

#### Laser与Ground Truth的轨迹方差等偏离数据数据：

![3误差评价1](doc/images/4误差评价1.png)

除了个别转弯的地方偏离的数值会突然增大，其他地方基本处于均值附近

#### Fused（ESKF方法滤波）与Ground Truth的轨迹对比：

![4f误差评价2](doc/images/4f误差评价2.png)

#### Fused（ESKF方法滤波）与Ground Truth对比的轨迹方差等偏离数据：

![4f误差评价1](doc/images/4f误差评价1.png)

Laser与Ground Truth的轨迹方差以及Fused（ESKF方法滤波）与Ground Truth对比的轨迹方差进行对比可以发现：两者都在相同的转弯处出现了较大的偏离值，但是Fused（ESKF方法滤波）的最大偏离值要比Laser的稍微小一点，两者偏离的均值以及中位数基本相当，但是最小值Fused（ESKF方法滤波）的更优一点。

## 增加噪声的改变

covariance:先改变Q，R不变，然后改变R，保持Q不变。

#### Q改变：改变 gyro，将其提高两个数量级：

```bash
 process:
            gyro: 1.0e-2
            accel: 2.5e-3
            bias_accel: 2.5e-3
            bias_gyro: 1.0e-4
        measurement:
            pose:
                pos: 1.0e-4
                ori: 1.0e-4
            pos: 1.0e-4
            vel: 2.5e-3
```

实验结果如下所示：

Laser与Ground Truth的轨迹方差等偏离数据数据：

![gryo1](doc/images/R/gryo1.png)

Fused（ESKF方法滤波）与Ground Truth对比的轨迹方差等偏离数据：

![F-gryo1](doc/images/R/F-gryo1.png)



#### 改变Q:改变 accel，将其提高2个数量级：

```
 process:
            gyro: 1.0e-2
            accel: 2.5e-1
            bias_accel: 2.5e-3
            bias_gyro: 1.0e-4
```

Laser与Ground Truth的轨迹方差等偏离数据数据：

![acc-1](doc/images/R/acc-1.png)

Fused（ESKF方法滤波）与Ground Truth对比的轨迹方差等偏离数据：

![f-acc-1](doc/images/R/f-acc-1.png)

结论：相比于原始数据，改变gyro和accel参数都会使得轨迹方差改变，方差最大值、均值和中位数都非常接近，融合后的稍微变大了一点，最小值则变小了，说明测试的预测对结果影响不大。