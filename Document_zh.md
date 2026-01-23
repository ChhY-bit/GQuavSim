# GQuavSim - 通用四旋翼无人机仿真模型 (MATLAB & ROS2)

**项目作者:** C.Yang （北京理工大学自动化学院，*预测智能控制实验室 PI-Control Lab*）
**联系邮箱:** [ych_0872@126.com](mailto:ych_0872@126.com)
**项目版本:** 1.0
**最近更新:** 2026-1-23

## 1 项目简介

- 本项目是一个四旋翼无人机通用仿真模型，旨在利用ROS2便捷的通信管理与指令发布机制，对基于MATLAB开发的控制算法实施便捷部署与快速验证。
- 提供了低速动态下尽可能接近真实的四旋翼无人机正向动力学模型。可将<u>*非对称惯性张量*、*陀螺效应*、*空气阻尼*、*外界扰动*</u>等通常被简化或忽略的因素纳入考虑范围，以更真实地模拟四旋翼无人机动态。
- 针对不同的控制层次需求，提供了以下几种控制输入接口：
  - **加速度-偏航角($a_x,a_y,a_z,\psi_d$)**：直接输入加速度指令，将无人机视为“听话的质点”，不关心其低层控制的细节，适合任务级控制。
  - **姿态-升力($\phi_d,\theta_d,\psi_d,F$)**：输入无人机期望的姿态与升力大小，适合需要高机动能力的控制。
  - **角速度-升力($\omega_x,\omega_y,\omega_z,F$)**：输入无人机在机体坐标系下的角速度与升力大小。其优点是响应较快，但模型复杂，对用户的专业知识水平要求较高。
  - **电机转速($\Omega_1,\Omega_2,\Omega_3,\Omega_4$)**：直接输入最底层的旋翼电机转速。这是最贴近硬件底层的控制接口，适用于需要精确仿真的场景。

- 提供了基于MATLAB-ROS2联合的四旋翼无人机仿真环境，包括：
  1. 四旋翼无人机始终监听ROS2话题指令，并依据自身动态模型实时响应指令输入；
  2. 四旋翼无人机状态的真值（位置、姿态）实时发布到ROS2话题，可供其他节点订阅使用；
  3. 四旋翼无人机状态的测量值，用于模拟可能的测量噪声、通信延迟等实际情况，并实时发布到ROS2话题，可供其他节点订阅使用；
  4. 四旋翼无人机的扰动通道，用于模拟可能的匹配或非匹配外部扰动，并实时发布到ROS2话题，可供其他节点订阅使用；
  5. 航行日志，用于记录仿真过程中的各项数据。


## 2 项目结构

### 2.1 核心文件

### 2.1 使用示例

### 2.2 通用模板

## 3 核心文件及其说明

### 3.1 动态模型类

动态模型类文件主要负责实现四旋翼无人机真实运动模拟。

#### 3.1.1 GQUAV_Model_UAV.m

##### 概述

- <span style="color: blue;">GQUAV_Model_UAV.m</span> 是一个MATLAB句柄类文件
- 包含属性：
  - Params：无人机参数结构体
  - States：无人机状态结构体
- 包含方法：
  - GQUAV_Model_UAV(Init)：初始化
  - UpdateState：在每个仿真时间步更新无人机状态

##### 属性
- <span style="color: red;">obj.Params</span>
  1. <span style="color: magenta;">Params.C_A</span>：气动升力系数，单位：$N/(rad/s)^2$
    气动升力系数决定了无人机在不同旋翼电机转速$\Omega_i$下的升力大小:$$F_{i} = C_{A} \Omega_i^2,$$该升力方向始终沿机体$z$轴方向。
  2. <span style="color: magenta;">Params.C_R</span>：反力矩系数，单位：$N \cdot m/(rad/s)^2$
    反力矩系数决定了无人机在不同旋翼电机转速$\Omega_i$下的反力矩大小:$$T_{R,i} = C_{R} \Omega_i^2,$$该力矩向始终沿机体$z$轴方向，主要影响无人机的偏航运动。
  3. <span style="color: magenta;">Params.L</span>：无人机旋翼臂长，单位：$m$
    无人机旋翼臂长$L$是指无人机每个旋翼电机与无人机中心位置的距离，主要影响由旋翼升力引起的力矩大小:$$T_{F,i} = F_i L.$$
  4. <span style="color: magenta;">Params.m</span>：无人机质量，单位：$kg$
  5. <span style="color: magenta;">Params.J</span>：无人机惯性张量，单位：$kg \cdot m^2$
    无人机惯性张量是3x3矩阵，于结构对称的无人机，惯性张量主要集中在主对角线上，其他元素接近于0，即$$J = diag(J_{xx},J_{yy},J_{zz}).$$
  6. <span style="color: magenta;">Params.J_r</span>：单个旋翼螺旋桨转动惯量，单位：$kg \cdot m^2$
    主要影响因电机高速旋转的同时机体发生偏转而造成的陀螺效应。
  7. <span style="color: magenta;">Params.mu_x</span>：无人机关于$x$方向的平移空气阻力系数，单位：$N/(m/s)$
  8. <span style="color: magenta;">Params.mu_y</span>：无人机关于$y$方向的平移空气阻力系数，单位：$N/(m/s)$
  9. <span style="color: magenta;">Params.mu_z</span>：无人机关于$z$方向的平移空气阻力系数，单位：$N/(m/s)$
  10. <span style="color: magenta;">Params.lambda_x</span>：无人机绕$x$轴的旋转空气阻力矩系数，单位：$N \cdot m/(rad/s)$
  11. <span style="color: magenta;">Params.lambda_y</span>：无人机绕$y$轴的旋转空气阻力矩系数，单位：$N \cdot m/(rad/s)$
  12. <span style="color: magenta;">Params.lambda_z</span>：无人机绕$z$轴的旋转空气阻力矩系数，单位：$N \cdot m/(rad/s)$
  13. <span style="color: magenta;">Params.g</span>：重力加速度，单位：$m/s^2$
  
  <span style="color: purple;"> *以上参数中，最重要的是C_A、C_R、L、m、J、g，其余参数在近似条件下可以置0。* </span>

- <span style="color: red;">obj.States</span>
  1. 位置信息：$(x,y,z)$
    <span style="color: green;">States.x</span>, <span style="color: green;">States.y</span>, <span style="color: green;">States.z</span>
  3. 姿态信息：滚转(roll)$\phi$、俯仰角(pitch)$\theta$、偏航角(yaw)$\psi$
    <span style="color: green;">States.phi</span>, <span style="color: green;">States.theta</span>, <span style="color: green;">States.psi</span>
  4. 速度信息(位置信息的时间导数)：$(\dot{x}, \dot{y}, \dot{z})$
    <span style="color: green;">States.dx</span>, <span style="color: green;">States.dy</span>, <span style="color: green;">States.dz</span>
  5. 机体角速度信息：$(\omega_x,\omega_y,\omega_z)$，部分文献也记作$(p,q,r)$
    <span style="color: green;">States.w_x</span>, <span style="color: green;">States.w_y</span>, <span style="color: green;">States.w_z</span>

##### 方法
- <span style="color: red;">GQUAV_Model_UAV(Init)</span>
  - 该方法用于初始化四旋翼无人机对象
  - 输入：<span style="color:green">(params,states_init)</span>
    - （可选）params: 无人机参数表，为18维向量，其各元素含义规定如表1所示。缺省时采用默认参数（通过调用<span style="color: blue;">GQUAV_Model_GetDefaultParams.m</span>函数获得）。
      <div align="center">表1 无人机参数表各元素含义与默认值

      | 元素索引 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 |
      | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
      | 含义 | C_A | C_R | L | m | J_xx | J_yy | J_zz | J_xy | J_xz | J_yz | J_r | mu_x | mu_y | mu_z | lambda_x | lambda_y | lambda_z | g |
      | 默认值 | 3.13e-6 | 7.50e-7 | 0.23 | 1.65 | 15.50e-3 | 15.50e-3 | 3.30e-2 | 6.00e-5 | 6.00e-5 | 6.00e-5 | 6.00e-5 | 0.01 | 0.01 | 0.01 | 0.01 | 0.01 | 0.01 | 9.81 |
      </div>
    - （可选）states_init: 无人机初始状态表，为12维向量，其各元素含义规定如表2所示，缺省时均默认为0。
      <div align="center">
      表2 无人机初始状态表各元素含义

      | 元素索引 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 |
      | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
      | 含义 | x | y | z | phi | theta | psi | dx | dy | dz | w_x | w_y | w_z |
      </div>
    
  - 输出：一个四旋翼无人机句柄类对象

- <span style="color: red;">obj.UpdateState</span>
  - 该方法用于在每个仿真时间步更新无人机状态
  - 输入：<span style="color:green">(Omega_1,Omega_2,Omega_3,Omega_4,t,dt)</span>
    - Omega_1,Omega_2,Omega_3,Omega_4: 当前时刻的四旋翼电机转速，单位：$rad/s$
    - t:当前仿真时间，单位：$s$
    <span style="color: purple;"> *注意：t为仿真开始后的实时时间，保留用于模拟时变因素（如参数漂移）；在时不变系统中不需使用时，可传入空参数"[]"。* </span>
    - dt：时间步长，单位：$s$
    <span style="color: purple;"> *注意：dt为仿真时间步长，即仿真世界里的“普朗克时间”，而不是控制周期（或采样时间间隔）。一般而言，dt应远小于控制周期。*
    </span>

#### 3.1.2 GQUAV_Model_GetDefaultParams.m

- <span style="color: blue;">GQUAV_Model_GetDefaultParams.m</span> 是一个MATLAB函数文件
- 用于在用户输入缺省时获取一组默认的无人机参数
- 输入参数：无
- 输出参数：<span style="color:green">(params)</span>
  - <span style="color: red;">params</span>：无人机参数表，为18维向量，其各元素含义规定如表1所示。



### 3.2 控制接口类

控制接口类文件主要负责实现不同控制层次的输入接口。

#### 3.2.1 GQUAV_Controller_ComputeAttitude.m

- <span style="color: blue;">GQUAV_Controller_ComputeAttitude.m</span> 是一个MATLAB函数文件
- 根据期望的加速度$(a_x,a_y,a_z)$与偏航角$\psi_d$计算期望的滚转角$\phi_d$与俯仰角$\theta_d$，计算公式为
$$
\begin{cases}
\theta_d &= \mathrm{atan2}(\alpha_y \sin\psi_d+ \alpha_x\cos\psi_d,\alpha_z),\\
\phi_d &= \mathrm{atan2}(\alpha_x \sin\psi_d-\alpha_y\cos\psi_d,{\alpha_z}/{\cos\theta_d}),
\end{cases}\\
\text{where } \boldsymbol{\alpha} = \dfrac{\boldsymbol{a}}{A(\boldsymbol{a},g)}, \quad A(\boldsymbol{a},g) = \sqrt{a_x^2 + a_y^2 + (a_z + g)^2}.
$$
- 输入参数：<span style="color:green">(a_x,a_y,a_z,psi_d)</span>
  - <span style="color: red;">a_x,a_y,a_z</span>：期望的加速度，单位：$m/s^2$
  - <span style="color: red;">psi_d</span>：期望的偏航角，单位：$rad$
- 输出参数：<span style="color:green">(phi_d,theta_d)</span>
  - <span style="color: red;">phi_d</span>：期望的滚转角，单位：$rad$
  - <span style="color: red;">theta_d</span>：期望的俯仰角，单位：$rad$

#### 3.2.2 GQUAV_Controller_ComputeThrust.m

- <span style="color: blue;">GQUAV_Controller_ComputeThrust.m</span> 是一个MATLAB函数文件
- 根据期望的加速度$(a_x,a_y,a_z)$与机体质量$m$计算期望的推力$F$，计算公式为
$$
F = m A(\boldsymbol{a},g).
$$
- 输入参数：<span style="color:green">(a_x,a_y,a_z,m)</span>
  - <span style="color: red;">a_x,a_y,a_z</span>：期望的加速度，单位：$m/s^2$
  - <span style="color: red;">m</span>：机体质量，单位：$kg$
- 输出参数：<span style="color:green">(F)</span>
  - <span style="color: red;">F</span>：期望的推力，单位：$N$
- 内部参数：<span style="color:green">(g)</span>
  - <span style="color: red;">g</span>：重力加速度，单位：$m/s^2$
#### 3.3.3 GQUAV_Controller_AngleSpeedLoop.m
- <span style="color: blue;">GQUAV_Controller_AngleSpeedLoop.m</span> 是一个MATLAB函数文件
- 依据期望的机体角速度$(\omega_{x,d},\omega_{y,d},\omega_{z,d})$与当前的反馈信息$(\omega_x,\omega_y,\omega_z)$与惯性张量$J$计算升控制量（机体坐标系下的旋转力矩）。控制律设计为状态反馈形式：
$$
\boldsymbol{T} = K_\omega J \left(\boldsymbol{\omega}_{d} - \boldsymbol{\omega}\right).
$$其中，$K_\omega$为角速度反馈增益矩阵，需设计为对称正定的（通常为对角型）。
- 输入参数：<span style="color:green">(w_d,w,J)</span>
  - <span style="color: red;">w_d</span>：期望的机体角速度向量，单位：$rad/s$
  - <span style="color: red;">w</span>：当前的机体角速度向量，单位：$rad/s$
  - <span style="color: red;">J</span>：惯性张量，单位：$kg \cdot m^2$
- 输出参数：<span style="color:green">(T_x,T_y,T_z)</span>
  - <span style="color: red;">T_x,T_y,T_z</span>：期望的机体坐标系下的旋转力矩，单位：$N\cdot m$
- 内部参数：<span style="color:green">(K_w)</span>
  - <span style="color: red;">K_w</span>：角速度反馈增益矩阵

#### 3.3.4 GQUAV_Controller_AttitudeLoop.m

- <span style="color: blue;">GQUAV_Controller_AttitudeLoop.m</span> 是一个MATLAB函数文件
- 依据期望的姿态$\eta_d=[\phi_d,\theta_d,\psi_d]^\top$与当前的反馈信息$\eta=[\phi,\theta,\psi]^\top$以及微分前馈量$\dot{\eta}_d=[\dot{\phi}_d,\dot{\theta}_d,\dot{\psi}_d]^\top$计算期望的机体角速度向量$\omega_d=[\omega_{x,d},\omega_{y,d},\omega_{z,d}]^\top$，控制律设计为状态反馈-微分前馈复合形式：
$$
\boldsymbol{\omega}_{d} =  W^{-1}(\eta)\left[K_\eta \left(\eta_d - \eta\right) + \dot{\eta}_d\right],
$$其中，$K_\eta$为姿态反馈增益矩阵，需设计为对称正定的（通常为对角型）。
<span style="color: purple;"> *注意：微分前馈项是为了消除姿态控制的稳态误差。若不需要微分前馈项或微分质量欠佳，则可将$\dot{\eta}_d$设为0。* </span>
- 输入参数：<span style="color:green">(eta_d,dot_eta_d,eta)</span>
  - <span style="color: red;">eta_d</span>：期望的姿态向量，单位：$rad$
  - <span style="color: red;">dot_eta_d</span>：期望的姿态微分向量，单位：$rad/s$
  - <span style="color: red;">eta</span>：当前的姿态向量，单位：$rad$
- 输出参数：<span style="color:green">(w_d)</span>
  - <span style="color: red;">w_d</span>：期望的机体角速度向量，单位：$rad/s$
- 内部参数：<span style="color:green">(J,K_eta)</span>
  - <span style="color: red;">K_eta</span>：姿态反馈增益矩阵

#### 3.3.5 GQUAV_Controller_Allocate.m

- <span style="color: blue;">GQUAV_Controller_Allocate.m</span> 是一个MATLAB函数文件
- 利用分配矩阵将期望升力$F$与期望力矩$T_x,T_y,T_z$映射为旋翼电机转速，分配公式为
$$
\begin{bmatrix}
\Omega_1^2 \\
\Omega_2^2 \\
\Omega_3^2 \\
\Omega_4^2 \\
\end{bmatrix} = B^{-1} \begin{bmatrix}
F \\
T_x \\
T_y \\
T_z \\
\end{bmatrix}，
$$ 其中，$B$为分配矩阵，定义为
$$
B = \mathrm{diag}\left[C_A,\dfrac{LC_A}{\sqrt{2}},\dfrac{LC_A}{\sqrt{2}},C_R\right]\underbrace{
\begin{bmatrix}
1 & 1 & 1 & 1 \\
1 & 1 & -1 & -1 \\
-1 & 1 & 1 & -1 \\
-1 & 1 & -1 & 1 \\
\end{bmatrix}}_E.
$$ 矩阵E**满秩**且元素均为1或-1，具体形式与电机旋转方向及编号的定义有关。
- 输入参数：<span style="color:green">(F,T_x,T_y,T_z)</span>
  - <span style="color: red;">F</span>：期望的推力，单位：$N$
  - <span style="color: red;">T_x,T_y,T_z</span>：期望的力矩，单位：$N\cdot m$
- 输出参数：<span style="color:green">(Omega_1,Omega_2,Omega_3,Omega_4)</span>
  - <span style="color: red;">Omega_1,Omega_2,Omega_3,Omega_4</span>：期望的四旋翼电机转速，单位：$rad/s$
- 内部参数：<span style="color:green">(C_A,LC_A,C_R)</span>
  - <span style="color: red;">C_A</span>：升力系数，单位：$N/(rad/s)^2$
  - <span style="color: red;">L</span>：四旋翼的半长轴（机臂长），单位：$m$
  - <span style="color: red;">C_R</span>：力矩系数，单位：$N \cdot m/(rad/s)^2$

### 3.3 仿真环境类

仿真环境类文件主要负责与ROS2的联合协同。

### 3.4 辅助功能类
#### 3.4.1 GQUAV_Utils_rotx.m
- <span style="color: blue;">GQUAV_Utils_rotx.m</span> 是一个MATLAB函数文件
- 用于计算绕x轴的旋转矩阵
- 输入参数：<span style="color:green">(phi)</span>
  - <span style="color: red;">phi</span>：绕x轴的旋转角度，单位：$rad$
- 输出参数：<span style="color:green">(R)</span>
  - <span style="color: red;">R</span>：绕x轴的旋转矩阵，为3x3矩阵，其元素定义如下：
    $$
    \mathrm{Rot}_x = \begin{bmatrix}
    1 & 0 & 0 \\
    0 & \cos\phi & -\sin\phi \\
    0 & \sin\phi & \cos\phi
    \end{bmatrix}
    $$

#### 3.4.2 GQUAV_Utils_roty.m
- <span style="color: blue;">GQUAV_Utils_roty.m</span> 是一个MATLAB函数文件
- 用于计算绕y轴的旋转矩阵
- 输入参数：<span style="color:green">(theta)</span>
  - <span style="color: red;">theta</span>：绕y轴的旋转角度，单位：$rad$
- 输出参数：<span style="color:green">(R)</span>
  - <span style="color: red;">R</span>：绕y轴的旋转矩阵，为3x3矩阵，其元素定义如下：
    $$
    \mathrm{Rot}_y = \begin{bmatrix}
    \cos\theta & 0 & \sin\theta \\
    0 & 1 & 0 \\
    -\sin\theta & 0 & \cos\theta
    \end{bmatrix}
    $$
#### 3.4.3 GQUAV_Utils_rotz.m
- <span style="color: blue;">GQUAV_Utils_rotz.m</span> 是一个MATLAB函数文件
- 用于计算绕z轴的旋转矩阵
- 输入参数：<span style="color:green">(psi)</span>
  - <span style="color: red;">psi</span>：绕z轴的旋转角度，单位：$rad$
- 输出参数：<span style="color:green">(R)</span>
  - <span style="color: red;">R</span>：绕z轴的旋转矩阵，为3x3矩阵，其元素定义如下：
    $$
    \mathrm{Rot}_z = \begin{bmatrix}
    \cos\psi & -\sin\psi & 0 \\
    \sin\psi & \cos\psi & 0 \\
    0 & 0 & 1
    \end{bmatrix}
    $$
#### 3.4.4 GQUAV_Utils_R.m
- <span style="color: blue;">GQUAV_Utils_R.m</span> 是一个MATLAB函数文件
- 利用滚转角、俯仰角、偏航角计算四旋翼无人机的旋转矩阵
- 输入参数：<span style="color:green">(phi,theta,psi)</span>
  - <span style="color: red;">phi</span>：绕x轴的旋转角度，单位：$rad$
  - <span style="color: red;">theta</span>：绕y轴的旋转角度，单位：$rad$
  - <span style="color: red;">psi</span>：绕z轴的旋转角度，单位：$rad$
- 输出参数：<span style="color:green">(R)</span>
  - <span style="color: red;">R</span>：四旋翼无人机的旋转矩阵，为3x3矩阵，其元素定义如下：
    $$
    R = \mathrm{Rot}_z(\psi) \mathrm{Rot}_y(\theta) \mathrm{Rot}_x(\phi)
    $$
#### 3.4.5 GQUAV_Utils_W.m
- <span style="color: blue;">GQUAV_Utils_W.m</span> 是一个MATLAB函数文件
- 利用当前姿态$(\phi,\theta,\psi)$计算**机体角速度$(\Omega_x,\Omega_y,\Omega_z)$** 到 **惯性坐标系下姿态变化率$(\dot{\phi},\dot{\theta},\dot{\psi})$** 的转换矩阵。
- 输入参数：<span style="color:green">(phi,theta,psi)</span>
  - <span style="color: red;">phi</span>：绕x轴的旋转角度，单位：$rad$
  - <span style="color: red;">theta</span>：绕y轴的旋转角度，单位：$rad$
  - <span style="color: red;">psi</span>：绕z轴的旋转角度，单位：$rad$
- 输出参数：<span style="color:green">(W,W_inv)</span>
  - <span style="color: red;">W</span>：机体角速度到惯性坐标系下姿态变化率的转换矩阵，为3x3矩阵，其元素定义如下：
    $$
    W = \begin{bmatrix}
    1 & \sin\phi \tan\theta & \cos\phi \tan\theta \\
    0 & \cos\phi & -\sin\phi \\
    0 & \sin\phi / \cos\theta & \cos\phi / \cos\theta
    \end{bmatrix}.
    $$
  - <span style="color: red;">W_inv</span>：转换矩阵$W$的逆矩阵，其元素定义如下：
    $$
    W^{-1} = \begin{bmatrix}
    1 & 0 & -sin\theta \\
    0 & \cos\phi & \sin\phi \cos\theta \\
    0 & -\sin\phi & \cos\phi \cos\theta \\
    \end{bmatrix}.
    $$

#### 3.4.6 GQUAV_Utils_UpdateRK4.m
- <span style="color: blue;">GQUAV_Utils_UpdateRK4.m</span> 是一个MATLAB函数文件
- 利用Runge-Kutta 4阶方法更新四旋翼无人机的动态状态
- 输入参数：<span style="color:green">(dynamic_fun,x,u,t,dt)</span>
  - <span style="color: red;">dynamic_fun</span>：动态方程函数句柄，用于计算四旋翼无人机的状态导数，即$\dot{x}(t) = f(x(t),u(t),t).$
  - <span style="color: red;">x</span>：当前时刻的四旋翼无人机状态向量
  - <span style="color: red;">u</span>：当前时刻的四旋翼无人机控制输入向量
  - <span style="color: red;">t</span>：当前仿真时间，单位：$s$
  - <span style="color: red;">dt</span>：时间步长，单位：$s$
- 输出参数：<span style="color:green">(x_next)</span>
  - <span style="color: red;">x_next</span>：更新后的四旋翼无人机状态向量