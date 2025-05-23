# ZJU-Bot MPC-WBC专题

针对自研人形机器人展开的模型预测控制（Model Predictive Control，MPC）与全身控制（Whole-Body Control，WBC）的研究。

NoKi41，2025年5月， xiaominggu@zju.edu.cn

***
## 目录
- [0. 数学基础](#0-数学基础)
   - [0.1 符号体系与核心公式](#01-符号体系与核心公式)
   - [0.2 最优化问题](#02-最优化问题)
- [1. 人形机器人动力学建模](#1-人形机器人动力学建模)
   - [1.1 URDF|XML解析：关节——连杆树](#11-urdfxml解析关节连杆树)
   - [1.2 惯量链](#12-惯量链)
   - [1.3 全身动力学（Whole-Body Dynamics）](#13-全身动力学whole-body-dynamics)
   - [1.4 质心动力学（Centroidal Dynamics）](#14-质心动力学centroidal-dynamics)
   - [1.5 简化模型：线性倒立摆（Linear Inverted Pendulum Model，LIPM）](#15-简化模型线性倒立摆linear-inverted-pendulum-modellipm)
   - [1.6 接触雅可比Jacobian与支撑多边形](#16-接触雅可比jacobian与支撑多边形)
- [2. MPC原理与行走约束](#2-mpc原理与行走约束)
   - [2.1 MPC总体框架](#21-mpc总体框架)
   - [2.2 预测模型选择谱](#22-预测模型选择谱)
   - [2.3 LIPM连续-离散化推导](#23-lipm连续-离散化推导)
   - [2.4 成本函数与约束显式化](#24-成本函数与约束显式化)
   - [2.5 终端约束与闭环稳定性简析](#25-终端约束与闭环稳定性简析)
   - [2.6 步态时序联动（Footstep-MPC）](#26-步态时序联动footstep-mpc)
   - [2.7 抬升到质心-MPC（非线性情形概览）](#27-抬升到质心-mpc非线性情形概览)
   - [2.8 调参与实践要点](#28-调参与实践要点)
- [3. WBC原理——层次QP与任务空间推导](#3-wbc原理层次qp与任务空间推导)
   - [3.1 控制目标与问题定义](#31-控制目标与问题定义)
   - [3.2 决策向量与硬等式约束](#32-决策向量与硬等式约束)
   - [3.3 任务参考生成与线性化](#33-任务参考生成与线性化)
   - [3.4 单层QP组装](#34-单层qp组装)
   - [3.5 约束线性化](#35-约束线性化)
   - [3.6 层次QP（HQP）机制](#36-层次qphqp机制)
   - [3.7 Null-space冗余利用](#37-null-space冗余利用)
   - [3.8 力矩回代与硬件接口](#38-力矩回代与硬件接口)

***
## 0. 数学基础

### 0.1 符号体系与核心公式
1. 对于浮动基座的人形机器人，含有“浮基 6 自由度 + 关节自由度”；状态向量$(q,\dot{q},\ddot{q})$分别对于位置，速度，加速度。共同描述机器人在任意时刻的运动状态。

2. 对于机器人机构，$N$为刚体/连杆数（浮动基下要+1），$n$为关节自由度总数

3. 从拉格朗日到牛顿-欧拉：**谁阻碍运动（惯性+被动力）=谁激发运动（主动力+外力）**
   - 动能 
   \[T(q,\dot{q}) = \frac{1}{2}\dot{q}^\top M(q)\dot{q}\tag{0.1.1}\] 

   - 势能
   \[V(q) = \sum_i m_i \mathbf{g}^{\top} {^0\mathbf{p}_{c_i}(q)}\tag{0.1.2}\]
      - $\mathbf{g}=[0,0,-9.81]\text{m}/\text{s}^2$是用世界坐标系$\{0\}$表示的重力加速度；
      - $^0\mathbf{p}_{c_i}(q)$是第$i$个连杆的质心在世界坐标系下的位置向量；
      - $m_i$是第$i$个连杆的质量。

   - 拉格朗日方程，其中$Q$为广义力。
   \[\frac{d}{dt}(\frac{\partial T}{\partial \dot{q}}) - \frac{\partial T}{\partial q} + \frac{\partial V}{\partial q}  = Q\tag{0.1.3}\]

   - 牛顿-欧拉法采用“正向运动学+反向动力学递推”，整理上述方程，写作机器人动力学标准形。
   \[M(q) \ddot{q} + C(q, \dot{q}) \dot{q} + g(q) = \tau + \sum_{i=1}^{n_c} J_i^\top f_i\tag{0.1.4}\]

      <!-- 1. $M(q)$——质量矩阵，对称正定，源自动能的二阶偏导；
      1. $C(q, \dot{q})$——非惯性项，包含科里奥利力、离心力、摩擦等，由$\dot M$和$\partial T/\partial q$组合；
      2. $g(q)=\partial V/\partial q$——重力项；
      3. $\tau$——执行器力矩；
      4. $J_i^\top f_i$——外部接触力在关节空间的映射，其中$J_i$，$f_i$分别的第$i$个接触点的雅各比和接触力。 -->

<!-- 3. 质心动力学（Centroidal Dynamics）：把全身看成刚体汇总到质心，只显式保留质心线动量$\mathbf{p}$和角动量$\mathbf{L}$。进一步假设质心高度常数$z_c$，得到线性倒立摆模型（Linear-Inverted Pendulum Model，LIPM），是后续MPC脚步与ZMP约束的理论基础。
\[\ddot{x} = \frac{g}{z_c}(x-x_{ZMP}), \, \ddot{y} = \frac{g}{z_c}(y-y_{ZMP})\] -->

### 0.2 最优化问题
1. 线性规划（Linear Programming）
   \[\min_{x} c^\top x,
   \quad s.t.
   \, Ax \leq b\tag{0.2.1}\]
2. 二次规划（Quadratic Programming，QP）：
   \[\min_{x} \frac{1}{2} x^\top H x + c^\top x,
   \quad s.t.
   \, Ax \leq b,
   \, A_{eq}x = b_{eq}\tag{0.2.2}\]
   其中$H$为正定或半正定矩阵，保证凸性。MPC 在采用倒立摆或线性质心模型时可落到 QP，实时性佳。
3. 非线性规划（Nonlinear Programming）：
   \[\min_{x} f(x),
   \quad s.t.
   \, g_i(x)=0,
   \, h_j(x)\leq 0\tag{0.2.3}\]
   模型或约束条件含非线性项，需使用 SQP、IPOPT、Acados 等求解器；Centroidal MPC 或全身动力学优化通常就是此类。
4. KKT条件（Karush-Kuhn-Tucker Conditions，KKT）：
   \[\nabla_x \mathcal{L}(x,\lambda,\mu) = 0,
   \, g_i(x) = 0,
   \, h_j(x) \leq 0,
   \, \mu_jh_j(x) = 0,
   \, \mu_j \geq 0\tag{0.2.4}\] 
   若目标函数凸，约束凸，则满足KKT条件，即为全局最优。

***

## 1. 人形机器人动力学建模

### 1.1 URDF|XML解析：关节——连杆树
URDF 将机器人表达为一棵由 `<link>` 与 `<joint>` 组成的有向树结构，每个 `<joint>` 节点携带类型（`revolute`/`prismatic`/`fixed`）、轴向、限位与阻尼等信息；`<inertial>` 块给出每个连杆的质量、质心与惯性张量$I$。解析过程的关键输出是：

| 数据 | 记号 | 用途 |
| --- | --- | --- |
| 关节数 | $n$ | 决定广义坐标系维度 |
| 父子变换 | $^pT_c(q)$ | 正运动学级联 |
| 惯性参数 | $m_i, I_i, ^ir_c$ | 构建质量矩阵与重力项 |

读取每个`<link>`惯量后，可用复合刚体算法 (Composite Rigid Body, CRBA) 线性时间组装质量矩阵$M(q)$；反向遍历可用 RNEA 计算关节力矩$\tau$。

### 1.2 惯量链
1. 单链（连杆）空间惯量 $\Phi_i(6\times 6)$ 
对于每一个连杆本体$i$，描述连杆自身质量、质心、转动惯量：
\[
\Phi_i = \begin{bmatrix}
I_i & m_iS(c_i)\\
m_iS(c_i)^\top & m_i I_3 
\end{bmatrix} , \,
S(c) = \begin{bmatrix}
0 & -c_z & c_y \\
c_z & 0 & -c_x \\
-c_y & c_x & 0
\end{bmatrix} .
\tag{1.2.1}\]
   - $m_i$是连杆本体质量，$I_3$是3阶单位矩阵；
   - $c_i = [c_x, c_y, c_z]^\top$描述本体坐标原点到连杆质心的位移，$S(c_i)$是对应的反对称（叉乘）矩阵；
   - $I_i$是绕本体坐标原点的转动惯量张量（惯性张量），在CAD建模转URDF文件时根据实际情况计算给定；
   - $\Phi_i$本身对称正定，在转换参考系时，世界参考系下：
      \[^0\Phi_i = X_{0i}^\top \Phi_i X_{0i} \tag{1.2.2}\]
      系统质心参考系下：
      \[^c\mathbf{G}_i = X_{c0}^\top {^0\Phi_i} X_{c0} \tag{1.2.3}\]
      区别于作用于位姿的$4\times 4$齐次变换矩阵（Homogeneous Transformation Matrix， HTM）$T_{0i}$，$X_{0i}$是作用于空间速度、空间作用力和空间惯量的$6\times 6$**空间变换矩阵（Spatial/Adjoint Transformation Matrix）**。
   \[
{T}_{0i} = 
\begin{bmatrix}
\mathbf{R} & \mathbf{p} \\
\mathbf{0} & 1
\end{bmatrix}, \quad
{X}_{0i} = 
\begin{bmatrix}
\mathbf{R} & \mathbf{0} \\
\mathbf{S}(\mathbf{p})\mathbf{R} & \mathbf{R}
\end{bmatrix}, \,
{X}_{0i}^{\top} = 
\begin{bmatrix}
\mathbf{R}^{\top} & \mathbf{R}^{\top}\mathbf{S}(\mathbf{p})^{\top} \\
\mathbf{0} & \mathbf{R}^{\top}
\end{bmatrix}
 \tag{1.2.4}\]

2. 系统空间惯量$I(6N\times 6N)$
   沿主对角线堆叠世界参考系下的连杆惯量：
   \[ I= \text{diag}(^0\Phi_1,^0\Phi_2, \cdots,^0\Phi_N)  \tag{1.2.5}\]
   $I$仍然对称正定，但维度很大。

3. 关节惯量矩阵$M(q) \ (n\times n)$
   利用系统速度雅可比Jacobian：
   \[
      V_{\text{all}} =
      \begin{bmatrix}
      V_1 \\
      V_2 \\
      \vdots \\
      V_N
      \end{bmatrix} =
      J(q) \dot{q} , \quad
      J(q) \in \mathbb{R}^{6N \times n} 
   \tag{1.2.6}\]
   系统动能可以被表示为：
\[
T = \frac{1}{2} {V}_{\text{all}}^{\top} {I} {V}_{\text{all}} = \frac{1}{2} \dot{{q}}^{\top} \underbrace{{J}^{\top} {I} {J}}_{{M}(q)} \dot{{q}} \tag{1.2.7}\]
\]
   于是得到关节惯量矩阵：
   \[ M(q) = {{J}^{\top} {I} {J}}  \tag{1.2.8}\]

### 1.3 全身动力学（Whole-Body Dynamics）
\[M(q) \ddot{q} + C(q, \dot{q}) \dot{q} + g(q) = \tau + \sum_{i=1}^{n_c} J_i^\top f_i \tag{1.3.1}\]

该公式为浮动基机器人全身动力学标准形公式（The Standard Form of Whole-Body Dynamics of a Floating-Base Robot），简称**WBD公式**，其中：

   - $M(q)$——关节惯量矩阵，对称正定，源自动能的二阶偏导；
   - $C(q, \dot{q})$——非惯性项，包含科里奥利力（Coriolis）和离心力（Centrifugal)，由$\dot M$和$\partial T/\partial q$组合；
   - $g(q)=\partial V/\partial q$——重力项；
   - $\tau$——执行器力矩；
   - $J_i^\top f_i$——外部接触力在关节空间的映射，其中$J_i$，$f_i$分别的第$i$个接触点的雅各比和接触力。

### 1.4 质心动力学（Centroidal Dynamics）

将全身投影到质心（Center of Mass，CoM）：

- ***怎么投影的？***
对于机器人系统的**总线动量**和**总角动量**定义为所有连杆在同一参考点（一般取CoM）动量的向量和，称质心动量向量：
\[
\mathbf{h}_G = 
\begin{bmatrix}
\mathbf{p}_G \\
\mathbf{L}_G
\end{bmatrix}
= \sum_{i = 1}^{N} {^c\mathbf{G}_i}
\begin{bmatrix}
{\mathbf{v}}_i \\
\boldsymbol{\omega}_i
\end{bmatrix} \tag{1.4.1}
\]
其中${^c\mathbf{G}_i}$为将第$i$个连杆的线、角动量转换到质心坐标系的$6\times 6$空间惯量矩阵。利用关节速度雅可比，定义**质心动量矩阵（Centroidal Momentum Matrix，CMM）**——$\mathbf{A}_G(q)_{6\times n}$w，将$\mathbf{h}_G$转换成：
\[
\mathbf{h}_G  = \sum_{i = 1}^{n} {^c\mathbf{G}_i} J_i(q) \cdot \dot{q}  = \mathbf{A}_G(q) \dot{q} \tag{1.4.2}
\]
对时间求导得到质心动量向量导数$\dot{\mathbf{h}}_G$：
\[
\dot{\mathbf{h}}_G = \underbrace{\mathbf{A}_G(q) \ddot{q}}_{\text{关节加速度项}} + 
\underbrace{\dot{\mathbf{A}_G}(q, \dot{q}) \dot{q}}_{\text{关节速度耦合项}} \tag{1.4.3}
\]
WBD公式中的加速度项放在一侧，同乘$M^{-1}(q)$，得到$\ddot q$的显示解：
\[\tag{1.4.4}
\begin{aligned}
M(q) \ddot{q}  & = \tau - C(q, \dot{q}) \dot{q} - g(q) + \sum_{i=1}^{n_c} J_i^\top f_i
\\
\ddot{q}  & = M^{-1} \left( {\tau} - {C} \dot{{q}} - {g} + {J}_c^{\top} {f}_c \right)
\end{aligned}
\]
代入$\dot{\mathbf{h}_G}$表达式得到：
\[\tag{1.4.5}
\dot{\mathbf{h}_G} = \mathbf{A}_G M^{-1} \left( {\tau} - {C} \dot{{q}} - {g} + {J}_c^{\top} {f}_c \right) + \dot{\mathbf{A}}_G \dot{{q}} 
\]
利用广义动量与质心动量在能量保守下的对偶性$T = \frac{1}{2}\dot q ^\top M \dot q = \frac{1}{2}\dot q ^\top (\mathbf{A}_G^\top \mathbf{A}_G) \dot q$，即正交恒等式$\mathbf{A}_GM^{-1}{\mathbf{A}_G^\top} = I_6$：
   1. 关节力矩项：  $\mathbf{A}_G {M}^{-1} {\tau} \rightarrow
   \mathbf{A}_G {M}^{-1} \mathbf{A}_G^{\top} \overbrace{\mathbf{A}_G^{-\top} {\tau}}^{\text{内部力偶}} = 0$，内部力偶互相平衡；
   2. 非惯性项：  $\mathbf{A}_G {M}^{-1} C \dot{q}$，在质心处与$\dot{\mathbf{A}}_G \dot{{q}}$相消；
   <!-- 推导如下：
   
      1. 经典刚体动力学性质，$\dot M - 2C$是一个反对称矩阵，常写作$\dot M = C + C^\top$
      2. 对正交恒等式求导：
      \[
         \underbrace{\dot{\mathbf{A}_G}M^{-1}\mathbf{A}_G^{\top}}_{\text{第一项}} + \underbrace{\mathbf{A}_G\dot{M}^{-1}\mathbf{A}_G^{\top}}_{\text{第二项}} + \underbrace{\mathbf{A}_GM^{-1}\dot{\mathbf{A}_G}^{\top}}_{\text{第三项}} = 0
         \]
      3. 根据逆矩阵求导公式：
      \[
         {\dot{M}^{-1}} = -M^{-1}\dot{M}M^{-1}
         \newline
         \overbrace{\mathbf{A}_G\dot{M}^{-1}\mathbf{A}_G^{\top}}^{\text{第二项}} = -\mathbf{A}_G M^{-1}\dot{M}M^{-1}\mathbf{A}^{\top}_G = -\mathbf{A}_GM^{-1}(C+C^\top)M^{-1}\mathbf{A}^{\top}_G
         \]
      4. 代回正交恒等式求导方程，右乘$\mathbf{A}_GM^{-1}$，对于正交矩阵$\mathbf{A}^{\top}_G\mathbf{A}_G=I$：
         1. 第一项：
         $\dot{\mathbf{A}_G}M^{-1}\mathbf{A}_G^{\top} \cdot \mathbf{A}_GM^{-1} = \dot{\mathbf{A}}_G M^{-1}(\mathbf{A}_G^{\top}\mathbf{A}_G) M^{-1} = \dot{\mathbf{A}}_G M^{-1}IM^{-1} = \dot{\mathbf{A}}_G  M^{-1}$ 
         2. 第二项：
         $-\mathbf{A}_G M^{-1}(C + C^{\top})M^{-1}\mathbf{A}_G^{\top} \cdot \mathbf{A}_GM^{-1} = -\mathbf{A}_G M^{-1}(C + C^{\top})M^{-1}(\mathbf{A}_G^{\top}\mathbf{A}_G) M^{-1} = -\mathbf{A}_G M^{-1}(C + C^{\top})M^{-1}IM^{-1} = -\mathbf{A}_G M^{-1}(C + C^{\top})M^{-1}$  
         3. 第三项：
         $\mathbf{A}_G M^{-1}\dot{\mathbf{A}_G}^{\top} \cdot \mathbf{A}_G M^{-1} = (\mathbf{A}_G M^{-1}\dot{\mathbf{A}}_G^{\top})\mathbf{A}_G M^{-1}$  -->

   3. 重力项： $\mathbf{A}_G {M}^{-1} g = \mathbf{g}_c$，合并计入；

   4. 外部接触项： $\mathbf{A}_G M^{-1} {J}_c^{\top} {f}_c$，变换到CoM的$6$维外力矩：
   \[\tag{1.4.6}
   \mathbf{A}_G M^{-1} {J}_c^{\top} {f}_c = \sum_k {^cX_k^\top f_k} 
   \]
   其中${^cX_k}$为将第$k$个接触点的接触力转换到CoM系$\{c\}$的变换矩阵。
最终得到质心动力学守恒式：
\[\tag{1.4.7}
\dot{\mathbf{h}}_G = \sum_k {^cX_k^\top f_k} + \mathbf g_c
\]

这说明质心动量的变化完全由外力矩决定，内部关节力矩在投影下互相抵消，正是动量定理即牛顿运动学闭环的数学体现。

\[\tag{1.4.8}
\underbrace{
\begin{bmatrix}
\dot{\mathbf{p}_G} \\
\dot{\mathbf{L}_G}
\end{bmatrix}
}_{\mathbf{\dot{h}}_G} = 
\begin{bmatrix}
\overbrace{\sum_{k} f_k + m \mathbf g}^{\text{近静态跳过线动量书写}} \\
\sum_{k} (\mathbf{p}_k - \mathbf{r}_{CoM}) \times {f}_k
\end{bmatrix}
\]
其中，${\mathbf{A}}_G(q)$可通过Pinocchio库中`computeCentroidalMap`得出，其物理含义是关节速度经过CMM时间导数映射出的假象力。
在行走MPC/WBC任务中,大多把平动（质心线动量）交给倒立摆模型或质心质量点处理,角动量才是**控制躯干摆动、双臂补偿**的要点；做静力平衡或慢速近静态规划时会认为线动量为0，但研究高速步行、跑跳等动态行为，就必须保留这一行，否则会丢失惯性效应。

质心动力学把$n$维广义速度投影到$6$维质心动量空间，消除了$n$维关节惯性，保留了$6$维动量，并且不丢失任何质量或惯性信息，常用于**外环MPC**。

### 1.5 简化模型：线性倒立摆（Linear Inverted Pendulum Model，LIPM）
在质心高度$z_c$近似于常数的情况下，可以把重心运动简化为：
\[\tag{1.5.1}
   \ddot{x} = \frac{g}{z_{CoM}}\left(x-x_{ZMP}\right), \, \ddot{y} = \frac{g}{z_{CoM}}\left(y-y_{ZMP}\right)\]

1. 核心假设：
   - 质心是点质量，忽略腿质量；
   - 质心高度恒定；
   - 踝关节可顺势产生水平力矩（把质心视为铰接在足底上）；
   - 支撑足与地面保持无滑动接触。
2. 零水平力矩点（Zero Moment Point，ZMP）是接触平面上一个特殊点，使得关于ZMP点的力矩在水平方向恰好为零；这一条件与质心高度近似不变的假设，即$\sum_k {^zf_z} + m\mathbf{g}=0$共同确定出ZMP点的位置$[x_\text{ZMP}, y_{ZMP}, 0]$；
3. 该关系线性、无关关节姿态，是行走 MPC 最常用的预测模型；
4. 支撑多边形为双足/单足接触点凸包（直观地讲，就是用一根橡皮筋把所有点圈起来后松手得到的多边形），零力矩点（Zero Moment Point，ZMP）必须保持在其中以防翻倒。若采用单足支撑，则多边形退化为脚底凸包；双足支撑时两脚凸包并集.

### 1.6 接触雅可比Jacobian与支撑多边形
1. 接触雅可比：特指把关节速度映射到接触点/接触面速度的矩阵。
   \[\tag{1.6.1}
      \mathbf{v}_{CoM} = J_c(q) \dot{q}
      \]
   当一只脚处于支撑相，要求其在世界坐标系中速度为零，保证脚不滑：
   \[\tag{1.6.2}
      J_c(q) \dot{q} = 0, \quad J_c(q)\ddot q + \dot J_c \dot q = 0
      \]
   二阶形式要求加速度为0用于逆动力学/WBC求解关节加速度。
2. 支撑多边形：
   - 单足支撑：脚底离散接触点（通常 4 个角点）凸包。
   - 双足支撑：左右脚多边形并集的凸包。
   - 多接触（脚+手攀爬）：所有有效接触面的投影统一求凸包。

   静态时CoM在支撑多边形内，动态运动时CoM会在支撑多边形外，但若ZMP落在凸包之外，则机器人倾覆。

### 本章小结
URDF$\overset{\text{CRBA/RNEA}}{\longrightarrow}$WBD模型$\overset{\text{投影CoM}}{\longrightarrow}$CD模型$\overset{\text{定高无踝力矩}}{\longrightarrow}$LIPM模型

- 自左向右逐层抽象，自右向左逐层回代。
- 外环MPC基于LIPM或质心模型；内环WBC解全身动力学闭环满足外环条件。
***

## 2. MPC原理与行走约束
本章思路建立在先用LIPM推导一条在二次规划（Quadratic Programming，QP）中实时求解的最简链路，再讨论如何抬升到质心-MPC。
行走 MPC 在使用 LIPM / ALIP 这类线性模型时，就能把滚动优化写成 QP，配合 OSQP、qpOASES 等求解器以毫秒级速度实时求解。

### 2.1 MPC总体框架
1. 预测模型：描述未来$N$步系统演化
   简单来说，把连续机器人、环境相互作用的运动规律，变成未来$N$步可迭代计算的差分方程；MPC就靠它预测 “如果我接下来推多少力（$u$），重心会走到哪（$x$）”。
   \[\tag{2.1.1}
      x_{k + 1} = f_d(x_k, u_k)\]
   - $x_k$为第$k$步的系统状态，在行走任务中通常取质心处的位置、速度甚至加速度；
   - $u_k$为第$k$步的控制输入，在LIPM场景下为$x_\text{ZMP},y_\text{ZMP}$，在质心MPC下则是接触力$f_k$；
   - $f_d$为离散化动力学，有连续模型$\dot x = f(x,u)$以采样周期$T_s$映射而来。

2. 滚动优化的代价函数：
   代价函数衡量“两件事”，其一是走得准（$x$靠近参考），其二是走得稳/省力（$u$不过激）。而终端项让问题在有限步长内依旧能保持闭环稳定。
   \[\tag{2.1.2}
   \min_{\{u_k\}} \sum_{k = 0}^{N - 1} \|x_k - x_k^{\text{ref}}\|_Q^2 + \|u_k\|_R^2 + \|x_N - x_N^{\text{ref}}\|_P^2
   \]
   - $x_k^{\text{ref}}$为参考轨迹（高层规划的理想质心/脚步位置）；
   - $||\cdot||^2_Q=(\cdot)^\top Q(\cdot)$;
   - $Q \succeq 0$为状态误差权重，越大对接近参考轨迹要求越严格；
   - $R \succeq 0$为控制努力权重，抑制ZMP抖动或接触力尖峰，越大动作越平滑越省能；
   - $P \succeq 0$为终端权重，保证到第$N$步仍位于安全“俘获项”，越大终端稳定裕度越高。
   
3. 约束层：依次为一般约束（**硬件红线**）、ZMP约束（**动态平衡**）和力-力矩约束（**不打滑不过载**）
   \[\tag{2.1.3}
   \begin{cases}
   x_k \in \mathcal{X}, u_k \in \mathcal{U}\\
   \text{ZMP}(x_k) \in \mathcal{P}_{\text{support}}\\
   (\tau_k, f_k) \text{ 满足摩擦锥/力矩限幅}
   \end{cases}
   \]
   - $\mathcal{X}$为状态可行域，典型如关节角限幅、质心高度限制；
   - $\mathcal{U}$为输入可行域，典型如ZMP最大位移、接触力上限、关节力矩饱和；
   - $\text{ZMP}(\cdot)$表示由系统状态算得的零力矩点
   - $\mathcal{P}_{\text{support}}$为支撑多边形，即全部接触点在地面投影的凸包，约束ZMP点必须在支撑多边形内；
   - 摩擦锥，即$\sqrt{f_x^2 + f_y^2} \leq \mu f_z, f_z \geq 0$，保证脚不打滑，后续详细展开。
   
4. 滚动执行：求得首控制量$u_0^*$立即施加，时域窗口向前滑动一步再求下一次。这一循环把“最优控制 + 在线再规划”结合起来，让控制器始终用最新状态重算未来最优路径，可自适应外界扰动；这称为后退式（Receding Horizon）控制。

### 2.2 预测模型选择谱
| 模型层级 | 维度$n_x$ | 特点 | 典型用处 |
| --- | --- | --- | --- |
| LIPM（重点） | 水平位移&速度：4$(x,\dot x,y,\dot y)$ | 线性、解析可离散化 | 基础行走MPC控制 |
| VHIP | 水平竖直位移速度6 | 可变高度、弱非线性 | 台阶、起伏地形 |
| ALIP | （4或6）+绕水平角动量2 | 线性、需准确惯量模型 | 抗侧向推、涉及角动量交换 |
| 质心动力学 | 质心位置、线速度、全身角动量、机体躯干朝向：12 | 明确角动量，需外力决策 | 力矩-受限步态 |
| 全身刚体动力学 | 浮动基6+关节数 | 最完整，非线性 | 深层次研究、慢速操作 |

1. 可变高度倒立摆模型（Variable-Height Inverted Pendulum Model，V-HIP）：在LIPM基础上把质心高度$z_{CoM}$作为状态或控制引入，使得：
\[\tag{2.2.1}
   \ddot{x} = \frac{g}{z_{CoM}}\left(x-x_{ZMP}\right) + \frac{\dot{z_{CoM}}}{z_{CoM}}\dot{x}\]
因此可以描述上下台阶、起伏地形、抬腿动作等质心高度明显变化的行走场景。模型弱非线性，常用一阶泰勒线性化+NMPC来求解。

1. 含角动量的线性倒立摆（Angular-momentum Linear Inverted Pendulum，ALIP）：在 LIPM 的线动量方程外，再把角动量线性化并显式加入状态：$\mathbf{\dot h} = [\dot x, \ddot x, \dot y, \ddot y, \dot{L_x}, \dot{L_y}]^\top$。
   \[\tag{2.2.2}
      \ddot x = \omega^2(x - x_\text{ZMP}) + \frac{\dot L_y}{m z_{CoM}}, \quad
      \dot L_y = \tau_\text{hip}\]
   这能把躯干/手臂摆动对平衡的影响（角动量交换）$\dot L_y$纳入 MPC，而仍保持整体线性，从而保持QP级求解速度。

### 2.3 LIPM连续-离散化推导
- 矩阵$A$（autonomous）描述系统本身的“内在运动规律”，是系统状态到系统状态的映射；
- 矩阵$B$（boost）描述外部输入对系统的直接作用，是控制输入到系统状态的映射。
- 下标$c$意味continuous连续时间，$d$意味discrete采样后的离散时间。
1. 连续方程：
   \[\tag{2.3.1}
      \ddot x=\omega^2(x - x_\text{ZMP}),\quad \ddot y=\omega^2(y - y_\text{ZMP}),\; \omega=\sqrt{g/z_{CoM}}\]

2. 状态-输入空间定义：
   \[\tag{2.3.2}
      \mathbf{x} = [x, \dot x, y, \dot y]^\top,\, \mathbf{u} = [x_\text{ZMP}, y_\text{ZMP}]^\top\]

3. 连续-时间矩阵：
   \[\tag{2.3.3}
      \boxed{\mathbf{\dot x}(t) = A_c \mathbf{x}(t) + B_c \mathbf{u}(t)},\quad
      A_c = \begin{bmatrix}
      0 & 1 & 0 & 0 \\ 
      \omega^2 & 0 & 0 & 0 \\ 
      0 & 0 & 0 & 1 \\ 
      0 & 0 & \omega^2 & 0 
      \end{bmatrix}
      , \,
      B_c = \begin{bmatrix}
      0 & 0 \\
      -\omega^2 & 0 \\
      0 & 0 \\
      0 & -\omega^2
      \end{bmatrix}
   \]
   显然，$A_c, B_c$由两个$2 \times 2$的倒立摆block对角排布，水平两正交方向互不耦合。
   对于线性时不变系统（Linear Time-Invariant System，LTI），齐次解满足：
   \[\tag{2.3.4}
      \dot x = A_cx \rightarrow x(t)=e^{A(t-t_0)}x(t-t_0)
      \]
   状态转移矩阵是系统矩阵的矩阵指数，描述系统自身在$\Delta t$内怎样从旧状态到新状态：
      \[\tag{2.3.5}
      e^{A\Delta t} = \sum_{k=0}^{\infty} \frac{A_c^k\Delta t^k}{k!}
      \]
   非齐次系统的完整解可写成：
      \[\tag{2.3.6}
      \mathbf{x}(t) = e^{{A_c}(t - t_0)} \mathbf{x}(t_0) + \int_{t_0}^{t} e^{{A_c}(t - \tau)} {B_c} \mathbf{u}(\tau) d\tau
      \]
   
4. 精确离散化（采样周期$T_s$）
   引入零阶保持（Zero-Order Hold，ZOH）假设：
   \[\tag{2.3.7}
      \begin{aligned}
      \mathbf{x}_{k+1} &= e^{A_c T_s} \mathbf{x}_k + \left(\int_{0}^{T_s}e^{A\tau}B_c d\tau\right)\mathbf{u}_k = \boxed{A_d \mathbf{x}_k + B_d \mathbf{u}_k}
      \\ 
      A_d = e^{A_c T_s} &= \begin{bmatrix}
      \cosh (\omega T_s) & \sinh (\omega T_s)/\omega & 0 & 0 \\
      \omega \sinh (\omega T_s) & \cosh (\omega T_s) & 0 & 0 \\
      0 & 0 & \cosh (\omega T_s) & \sinh (\omega T_s)/\omega \\
      0 & 0 & \omega \sinh (\omega T_s) & \cosh (\omega T_s)
      \end{bmatrix}
      \\ 
      B_d &= \int_{0}^{T_s} e^{A_c \tau} B_c d\tau = \begin{bmatrix}
      1 - \cosh (\omega T_s) & 0 \\
      -\omega \sinh (\omega T_s) & 0 \\
      0 & 1 - \cosh (\omega T_s) \\
      0 & -\omega \sinh (\omega T_s)
      \end{bmatrix}
      \end{aligned}
   \]
   简易近似：若$T_s \ll {1/\omega}$，可用一次Euler（$a \ll 1 \Rightarrow \cosh a\approx 1 + \frac{1}{2}a^2, \,\sinh a\approx a $）简化得到$A_d \approx I + A_c T_s, \, B_d \approx B_c T_s$——实际行走取$T_s$。

### 2.4 成本函数与约束显式化
1. 栈式变量：把$N$步后所有状态、输入一次性写成线性组合，便于后面构造二次代价和线性约束。
   \[ \tag{2.4.1}  
      \begin{aligned}
      \mathbf{x}_{1} &= A_d \mathbf{x}_{0} + B_d \mathbf{u}_{0}
      \\
      \mathbf{x}_{2} &= A_d^2 \mathbf{x}_{0} + A_d B_d \mathbf{u}_{0} + B_d \mathbf{u}_{1}
      \\
      &\vdots
      \\
      \mathbf{x}_{N} &= A_d^{N} \mathbf{x}_{0} + A_d^{N-1} B_d \mathbf{u}_{0} + \cdots + A_d B_d^{N-1} \mathbf{u}_{N-2} + B_d \mathbf{u}_{N-1}
      \end{aligned}
   \]
   把$N$个方程堆叠：
   \[ \tag{2.4.2} 
      \mathbf{X} = \begin{bmatrix}
      \mathbf{x}_1 \\
      \mathbf{x}_2 \\
      \vdots \\
      \mathbf{x}_{N}
      \end{bmatrix},\quad
      \mathbf{U} = \begin{bmatrix}
      \mathbf{u}_0 \\
      \mathbf{u}_1 \\
      \vdots \\
      \mathbf{u}_{N-1}
      \end{bmatrix}
      \]
      用“预测矩阵”，$\mathbf{X} = S_x \mathbf{x}_0 + S_u \mathbf{U}$，其中“**自由漂移矩阵**”$S_x$和“**输入累计矩阵**”$S_u$由$A_d, B_d$递推叠加得到：
      \[ \tag{2.4.3} 
         S_x = \begin{bmatrix}
         A_d \\
         A_d^2 \\
         \vdots \\
         A_d^N
         \end{bmatrix}, \quad
         S_u = 
         \begin{bmatrix}
         B_d & 0 & \cdots & 0 \\
         A_d B_d & B_d & \cdots & 0 \\
         \vdots & \vdots & \ddots & \vdots \\
         A_d^{N - 1} B_d & A_d^{N - 2} B_d & \cdots & B_d
         \end{bmatrix}
      \]

2. 二次成本及QP目标
   逐步的代价函数：
   \[ \tag{2.4.4} 
   J = \sum_{k = 0}^{N - 1} \|x_k - x_k^{\text{ref}}\|_Q^2 + \|u_k\|_R^2 + \|x_N - x_N^{\text{ref}}\|_P^2
   \]
   换到栈式向量形式：
   \[ \tag{2.4.5} 
   J = \frac{1}{2} (\mathbf{X} - x_k^{\text{ref}})^\top Q_b (\mathbf{X} - x_k^{\text{ref}}) + \frac{1}{2} \mathbf{U}^{\top} R_b \mathbf{U}
   \]
   简化后：
   \[ \tag{2.4.6} 
   J = \frac{1}{2} \mathbf{U}^{\top} {H} \mathbf{U} + \mathbf{U} ^{\top}{h} , \quad
   {H} = {S}_u^{\top} {Q}_b {S}_u + {R}_b, \,{h} = {S}_u^{\top} {Q}_b ({S}_x {x}_0 - {x}^{\text{ref}})
\]
其中${Q}_b = \text{blkdiag}({Q}, \dots, {Q}, {P})$, ${R}_b = \text{blkdiag}({R}, \dots, {R})$。

3. ZMP约束：
   单脚或双脚支撑时，支撑多边形$\mathcal{P}$在平面上是凸多边形，可用顶点-半空间表示：
   \[ \tag{2.4.7}
      F_{poly} \begin{bmatrix} x_\text{ZMP} \\ y_\text{ZMP} \end{bmatrix} \leq b_{poly}
      \]
   其中，$F_{poly} \in \mathbb{R}^{m\times 2}, b_{poly}\in \mathbb{R}^m$，$m$为凸多边形边数。在Pinocchio和MuJoCo中给出足底的接触几何$\rightarrow$取四角顶点$\rightarrow$自动生成$F_{poly}, b_{poly}$。
   LIPM里，控制输入本身$u=[x_\text{ZMP}, y_\text{ZMP}]^\top$代入后得到：
   \[ \tag{2.4.8}
      F_{poly} {u}_k \leq b_{poly}
      \]
   即始终保持线性。*但对于质心模型或ALIP，ZMP不再等于输入，需要：*$F_{poly}\left(Cx_k+Du_k\right)\leq b_{poly}$*，依旧可以保持线性。*
   对于LIMP中的每一步$k = 0, 1, \dots, N-1$都应该满足ZMP约束。把不等式对角块排进大矩阵中：
   \[ \tag{2.4.9}
   {A}_{\text{ineq}} = 
   \begin{bmatrix}
   {F}_{\text{poly}} &  &  & \\
   & {F}_{\text{poly}} &  & \\
   &  & \ddots & \\
   &  &  & {F}_{\text{poly}}
   \end{bmatrix}_{(Nm) \times (2N)}, \quad 
   {b}_{\text{ineq}} = 
   \begin{bmatrix}
   {b}_{\text{poly}} \\
   {b}_{\text{poly}} \\
   \vdots \\
   {b}_{\text{poly}}
   \end{bmatrix}
   \]   

   \[ \tag{2.4.10}
      A_\text{ineq} \mathbf{U} \leq b_\text{ineq}
   \]

   $|x_{\text{ZMP}}| \leq d_{\text{max}},\,|u_k-u_{k-1}|\leq \Delta_{max}$等同样写成不等式施加，拼接到大矩阵中。到此MPC转化为稠密QP问题：
\[ \tag{2.4.11}
   \min_{\mathbf{U}} \frac{1}{2} \mathbf{U}^{\top} {H} \mathbf{U} + {h}^{\top} \mathbf{U} \quad \text{s.t.} \quad {A}_{\text{ineq}} \mathbf{U} \leq {b}_{\text{ineq}}
\]
结构与经典轨迹跟踪 QP 完全一致，可用OSQP或qpOASES在$\geq1kHz$解。

### 2.5 终端约束（Terminal Ingredients）与闭环稳定性简析
- ***为什么要“补一口锅盖”？***
MPC 只优化有限$N$步。如果没有对“窗口尾端”做任何保证，算法可能在前$N-1$步里把系统推到一个无路可退的死角，下一轮就会“不可行”。解决思路是在第$N$步加一道安全阀。

1. 等价线性二次型调节器（Linear Quadratic Regulator，LQR）终端权重：
   取$P$为离散LQR的Riccati方程解，即$P = A_d^\top P A_d - A_d^\top P B_d(R+B_d^\top P B_d)^{-1} B_d^\top P A_d + Q$。若不加显式终端约束而仅用该$P$，再把权重$Q,R$设为LQR同一对，因为代价函数是一个Lyapunov函数，就可证明MPC闭环与LQR闭环在无约束时渐进等价切稳定（MPC-LQR等价定理）。

2. 捕获域 (Capture Point) 终端
   1. 只要脚步落在捕获点$x_{CP} = x + \frac{\dot x}{\omega}$，倒立摆可在一步内停住。
   2. 要求第$N$步状态$x_N, \dot x_N$落在一阶捕获域内，即$|x_N + \frac{\dot x_N}{\omega}| \leq d_{max}, \, |y_N + \frac{\dot y_N}{\omega}| \leq \omega_{max}$，可在较短$N$时仍保持可行，可以直接扩展到约束大矩阵中。
   
3. 递归可行（Recursive Feasibility）与ISS
   对线性系统+多面体约束，加终端集合$x_N \in \mathcal{X_f}$正不变可保证递归可行，即当前时刻QP可解就能保证下一时刻依旧可解。实际步行多用“加权软约束+松弛变量”折中实时性与可行域大小。

### 2.6 步态时序联动（Footstep-MPC）——不只“摆重心”，还同时调脚步

- Q1：ZMP轨迹为什么要和脚步绑定？
A1：因为 ZMP 必须落在“**当前支撑足的多边形**”里，而脚步换腿会使多边形在时间上跳跃。
 
- Q2：脚步是离散事件，怎么塞进连续QP？
A2：把“哪只脚支撑”编码成整数/布尔变量，与连续ZMP、质心变量组合$\rightarrow$Mixed-Integer QP。
 
- Q3：MIQP太慢怎么办？
A3：先用**离散事件规划器**给出左右脚序列，再让MPC微调**实数脚位**（两层法）。

1. 步位变量$p_k(p_k^x, p_k^y)$与ZMP约束耦合：
   对于摆动-支撑切换明确的时刻$k$，$\delta_k$是ZMP在脚底局部坐标上的偏移，受脚底多边形限制：
   \[ \tag{2.6.1}
      x_{\text{ZMP},k} = p_k^x + \delta_k^x, \quad y_{\text{ZMP},k} = p_k^y + \delta_k^y,
   \]
   ZMP约束变成$F_\text{foot} \mathbf{\delta}_k \leq b_\text{foot}$，联动$p_k$进入决策向量。

2. 整数变量（如左脚支撑1，右脚支撑0）决定左右脚序列$\rightarrow$MIQP；通常先用 “离散事件规划器” 产出脚序列以及对应的$p_k^{left},p_k^{right}$。实操中，常采用两层分解避免NP难度：
   1. 事件层（低频、只含整数）：步态机、有限状态机，决定左右脚序列和触地腾空时间；
   2. 连续MPC层（高频）：序列固定，只优化$p_k$、质心、ZMP，回到纯QP。
3. 文献常用：Kajita Preview Control、Capture-step MPC、Time-varying ZMP Reference

### 2.7 抬升到质心-MPC（非线性情形概览）——把力也纳入决策
1. 状态$\mathbf{x} = [\mathbf{p}_G, \mathbf{L}_G]$+足底接触表征；
2. 控制$\mathbf{f}_c$（每只脚6维力矩）+切换布尔量；
3. 动力学：$\dot{\mathbf{h}}_G = \sum_k {^cX_k^\top f_k} + \mathbf g_c$
4. 约束摩擦锥$||\mathbf{f}_{xy}|| \leq \mu f_z, \, f_z \geq 0$；
5. 求解$\rightarrow$SQP或ACADOS NMPC，20-100Hz足够；
6. 为何仍实时：决策维度$\approx(6\times$接触数$)\times N$，远小于全身关节数。

### 2.8 调参与实践要点
| 项目 | 经验值 | 影响 |
| --- | --- | --- |
| 预测步长$N$ | 20-40 | 越长越稳，但计算量也越大 |
| 采样周期$T_s$ | 5-10ms | 轨迹平滑，与内环对齐 |
| 权重$Q$  | 位置10-50，速度1-5 | 越大轨迹越精准，越小抖动越少 |
| 权重$R$ | 0.001-0.01 | 越大力越平滑，越小响应越快 |
| 终端权重$P$ | 由DARE求解 | 稳定性保障 |
| 求解器 | OSQP,qpOASES | 迭代次数20以内 |

- 内环WBC一般采样频率1khz，外环MPC采样周期保持与其整数倍同步。
- 软约束：在$A_\text{ineq}U \leq b + \sigma, \sigma \geq 0$上再加成本$\rho||\sigma||_1$，既避免不可行又不至过度违反。

### 本章小结
1. MPC核心三件套：预测模型+二次成本+线性约束；
2. 倒立摆LIPM把人形行走压缩到4维线性系统$\rightarrow$QP可千赫兹求解；
3. ZMP+支撑多边形可全程保持线性，实时性强；
4. 终端权重/捕获域解决有限步长稳定性；
5. 提升模型$\rightarrow$质心-MPC/VHIP/全身动力学，代价是转向 NLP，但频率要求较低；
6. MPC外环产出的质心轨迹&脚底力/位点将交由WBC内环生成关节力矩闭环执行。

## 3. WBC原理——层次QP与任务空间推导
内环WBC把MPC输出的**质心/ZMP/接触力参考**转化为毫米级可执行的关节力矩。

### 3.1 控制目标与问题定义
1. 输入：
   1. 期望质心加速度$\ddot{c}_G^{\text{ref}}$，**MPC外环**输出的质心线加速度参考，决定“质心往哪儿加速”——保证重心运动轨迹符合行走规划；
   2. 期望角动量变化$\dot{\mathbf{L}}_G^{\text{ref}}$，，**MPC外环**输出的质心角动量变化率参考（合外力矩），	约束躯干/手臂产生的角动量交换，抵消外推力或急转弯；
      - 若仅做“行走不摆手”，可以设为零向量；若要抗推/跑跳，则需给出非零角动量变化参考。
   3. 期望摆动足轨迹、躯干姿态等任务量 (上层规划/行为层)：
      1. $\mathbf{x}_\text{foot}^\text{des}(t)$，由步态规划器产生，给摆动足末端一个连续轨迹（位置+朝向）保证抬脚不绊地；
      2. $\mathbf{R}_\text{torso}^\text{des}$，来自高层姿态规划/默认直立，躯干姿态（roll-pitch-yaw）参考，抑制上身晃动。
2. 输出： WBC的求解器同时生成$\ddot{q}, \mathbf{f}_c, \boldsymbol{\tau}$，但实际发送给电机的只有力矩
   1. 关节力矩向量
   \[ \tag{3.1.1}
   \boldsymbol{\tau} = [\tau _\text{hip\_roll}, \tau _\text{hip\_pitch}, \cdots, \tau _n] \in \mathbb{R}^n
   \]
   使机器人同时满足动力学、接触、执行器极限，并按优先级跟踪各任务。
      - 本质是 **受约束逆动力学最优化**。

   2. 接触力向量（内部变量，最后要满足摩擦锥）：$\mathbf{f}_c = [f_{x,1}, f_{y,1}, f_{z,1}, \cdots]^\top$
   
### 3.2 决策向量与硬等式约束
1. 决策向量与维度估计
在满足全身动力学与接触硬约束的前提下，让各任务加速度尽量贴合参考，同时输出实机可执行的关节力矩：
\[ \tag{3.2.1}
{M}({q})\ddot{{q}} + {h}({q},\dot{{q}}) = {S}^{\top}\boldsymbol{\tau} + {J}_c^{\top}\mathbf{f}_c \implies \mathbf{A}_{\text{dyn}}
\underbrace{\begin{bmatrix}
\ddot{{q}} \\
\mathbf{f}_c \\
\boldsymbol{\tau}
\end{bmatrix}}_{\mathbf{z}} = \mathbf{b}_{\text{dyn}}
\]
其中：
\[ \tag{3.2.2}
   \mathbf{z} = \begin{bmatrix}
   \ddot{{q}} \\
   \mathbf{f}_c \\
   \boldsymbol{\tau}
   \end{bmatrix} \in \mathbb{R}^{n_{\ddot q}+n_{f}+n{\tau}}
   ,\quad
   \mathbf{A}_{\text{dyn}} = [M ,-J_c^\top ,-S^\top]
   , \quad
   \mathbf{b}_{\text{dyn}} = -h
\]

2. 硬等式约束
   1. 动力学一致性：保证任何的输出量（电机角加速度，接触力，力矩）不违反牛顿–欧拉方程
   \[
   {M}({q})\ddot{{q}} + {h}({q},\dot{{q}}) = {S}^{\top}\boldsymbol{\tau} + {J}_c^{\top}\mathbf{f}_c
   \]
   - \( {M} \) — 质量矩阵; \( {h} = {C}\dot{{q}} + {g} \)。
   - \( {S} = [{0}_{n \times 6} \, {I}_n] \) 抽取关节子集。
   - \( {J}_c\mathbf{f}_c \) — 支撑足零速度约束产生的接触力。

   2. 接触保持：足底在支撑期速度、加速度恒为零，不打滑不离地
   \[ \tag{3.2.3}
   \begin{cases}
   {J}_c({q})\dot{{q}} = 0 \\
   {J}_c({q})\ddot{{q}} + \dot{{J}}_c({q},\dot{{q}})\dot{{q}} = 0
   \end{cases}
   \implies 
   {J}_c({q})\ddot{{q}} + \dot{{J}}_c\dot{{q}} = 0
   \]
   - 若进入摆动期，只需把对应行从$J_c$删掉即可；
   - 额外的“静摩擦不滑”最终由摩擦锥不等式（3.7）保证。
  
   3. 执行器力矩/速度限幅：来自 XML/URDF `<motor ctrlrange="−200 200">` 等字段。
   
3. 统一矩阵形式
   叠加(3.2.2)和(3.2.3)，得到：
   \[ \tag{3.2.4}
   \boxed{\mathbf{Az} = \mathbf{b}}, \quad 
   \mathbf{A}=\begin{bmatrix}{M}&-{J}_c^{\top}&-{S}^{\top}\\{J}_c&{0}&{0}\end{bmatrix}, \quad 
   \mathbf{b}=\begin{bmatrix}-{h}\\-\dot{{J}}_c\dot{{q}}\end{bmatrix}
   \]
   上块是**动力学方程**，下块是接**触零加速度等式**。

### 3.3 任务参考生成与线性化
1. 任务清单

   | 任务编号 | 任务名称 | 任务空间变量$\mathbf{x}_i$ | 关联雅可比$J_i(q)$ |
   | --- | --- | --- | --- |
   | 1 | 质心线加速度 | $\dot c_G \in \mathbb{R}^3$ | $J_\text{CoM}$ |
   | 2 | 角动量变化率 | $\mathbf{L}_G \in \mathbb{R}^3$ | $A_\text{ang}$ |
   | 3 |	摆动足末端 | $\mathbf{p}_\text{sw} \in \mathbb{R}^3$ | $J_\text{sw}$ |
   | 4 | 躯干姿态 | $\phi_\text{torso} \in \mathbb{R}^3$ | $J_\text{torso}$ |

   - 所有变量均在世界参考系表述；
   - 上身姿态可选用roll-pitch-yaw或Rodrigues向量表示。

2. PD 型期望加速度
   对每个任务 \( i \)：
   \[ \tag{3.3.1}
   \ddot{\mathbf{x}}_i^{\text{ref}} = \ddot{\mathbf{x}}_i^{\text{des}} + K_{d,i}(\dot{\mathbf{x}}_i^{\text{des}} - \dot{\mathbf{x}}_i) + K_{p,i}(\mathbf{x}_i^{\text{des}} - \mathbf{x}_i) 
   \]

   - 前馈项$\ddot{\mathbf{x}}_i^{\text{des}}$：来自外环MPC轨迹；
   - 反馈项：用误差$e_i=\mathbf{x}_i^{\text{des}} - \mathbf{x}_i$设计二阶临界阻尼；常设$K_{p,i} = \omega_i^2 I, \quad K_{d,i} = 2\zeta_i\omega_i I \text{, with } \zeta_i = 1$。

3. 线性任务方程——位置类
   对CoM与摆动足末端（任务1、3）：
   \[ \tag{3.3.2}
      \underbrace{{J}i({q})}_{3\times(6 + n)}\ddot{{q}}=\underbrace{\ddot{\mathbf{x}}_i^{\text{ref}}-\dot{{J}}i({q},\dot{{q}})\dot{{q}}}_{{b}_i}
   \]
   - ${J}i({q}) =\frac{\partial\mathbf{x}_i}{\partial{q}}$由正运动学自动求导（Pinocchio `computeJointJacobian`)；
   - $\dot{{J}}_i\dot{{q}}=\frac{d}{dt}({J}_i\dot{{q}})$在库中亦可直接求。

4. 线性任务方程——姿态类
   对躯干姿态类（任务4）采用Rodrigues误差：
   \[ \tag{3.3.3}
   \phi = \log (R_\text{torso}^\top R^\text{des}_\text{torso})
   \]
   线速度、加速度参考：
   \[ \tag{3.3.4}
   \dot{\phi}^\text{ref} = -K_{p,4} \phi, \quad \ddot{\phi}^\text{ref} = -K_{d,4} \dot\phi
   \]
   因为$\phi$与加速度$\omega$在小角度内近似线性，可写：
   \[ \tag{3.3.5}
   J_4(q) \ddot{q} = \ddot{\phi}^\text{ref} - \dot{J}_4(q, \dot{q}) \dot{q}
   \]
   $J_4$摄动矩阵由**基础twist Jacibian**取旋转部分即可。

5. 角速度变化率任务
   由质心动量矩阵$A(q)$（1.4中）分角行：
   \[ \tag{3.3.6}
   \dot{\mathbf{h}}_{G,ang} = {A}_{\text{ang}}({q})\ddot{{q}} + \dot{{A}}_{\text{ang}}(q, \dot q)\dot{{q}}
   \]
   令
   \[ \tag{3.3.7}
   {A}_{\text{ang}}({q})\ddot{{q}} = \dot{\mathbf{h}}_{G,ang}^{\text{ref}} - \dot{{A}}_{\text{ang}}\dot{{q}}
   \]
   即得到任务 2 的线性方程。

6. 任务堆叠与权重矩阵
   将所有任务行垂直拼接：
   \[ \tag{3.3.8}
   J_\text{task} = \begin{bmatrix}
   J_{\text{CoM}} \\
   A_{\text{ang}} \\
   J_{\text{sw}} \\
   J_{\text{torso}} \\
   \end{bmatrix} , \quad
   {b}_{\text{task}} = \begin{bmatrix}
   b_{\text{CoM}} \\
   b_{\text{ang}} \\
   b_{\text{sw}} \\
   b_{\text{torso}} \\
   \end{bmatrix}
   \]
   对应权重：
   \[ \tag{3.3.9}
   W = \text{blkdiag}(W_{\text{CoM}}, W_{\text{ang}}, W_{\text{sw}}, W_{\text{torso}})
   \]
   - 经验权重：$W_{\text{CoM}} \gg W_{\text{sw}} > W_{\text{torso}}$，例如$100,10,1$，角动量任务通常与CoM同层或稍低。
  
7. 硬任务与软任务
   - 硬任务：若必须 100%跟踪（如飞跳落脚位置），可将对应行移入硬等式层，与 (3.2.4) 合并。
   - 软任务：绝大多数情况下采用二次权重；调高权重即可近似硬跟踪而不引致不可行。

### 3.4 单层QP组装
如何把所有的：硬等式（3.2）、软任务（3.3）、不等式（3.5），整合成一次可由OSQP/qpOASES在1kHz内求解的凸二次规划。

1. 目标函数（Cost）
   软任务误差项＋关节力矩正则项：
   \[ \tag{3.4.1}
   J(\mathbf{z}) = \frac{1}{2} ({J}_{\text{task}} \ddot{{q}} - \mathbf{b}_{\text{task}})^{\top} {W} ({J}_{\text{task}} \ddot{{q}} - {b}_{\text{task}}) + \frac{1}{2} {\tau}^{\top} {R} {\tau}
   \]
   其中
   - 任务权重$W = \text{blkdiag}(W_{\text{CoM}}, W_{\text{ang}}, W_{\text{sw}}, W_{\text{torso}})$
   - 力矩正则$R = \alpha I_n, \,(\alpha \in [10^{-3}, 10^{-2}])$
   - 目标函数为各任务加速度误差与调节力矩平滑的加权和；
      - 加速度误差项：尽量跟踪（质心、角速度、摆腿、躯干……）加速度参考；
      - 力矩平滑项：相当于“最小控制能量/抑振”正则；若关节电机效率高、电流成本低，可把权重矩阵$\mathbf{R}$设小甚至0。
   - 约束依次为全身动力学、接触保持（支撑足零加速度）、摩擦锥与电机力矩限幅；
   - 若所有任务同优先级，则公式(3.4.1)本身就是一个凸QP，可用qpOASES/OSQP在1kHz内求解。

2. 将$J(\mathbf{z})$写成二次型
   把决策向量$\mathbf{z} = [\ddot{{q}}; \mathbf{f}_c; \boldsymbol{\tau}]$分块，对(3.4.1)展开可得
   \[ \tag{3.4.2}
   J(\mathbf{z}) = \frac{1}{2} \mathbf{z}^{\top} H \mathbf{z} + h^{\top} \mathbf{z} + \text{const}
   \]

   \[ \tag{3.4.3}
   H = \begin{bmatrix}
   {J}_{\text{task}}^{\top} {W} {J}_{\text{task}} & 0 & 0 \\
   0 & 0 & 0 \\
   0 & 0 & {R}
   \end{bmatrix}, \quad
   h = \begin{bmatrix} - {J}_{\text{task}}^{\top} {W} {b}_{\text{task}} \\
   0 \\
   0
   \end{bmatrix}
   \]

   - **正定性**:
     - 若${W} \succ 0, {R} \succ 0$，则$H \succeq 0 \Rightarrow$问题凸；
     - $\mathbf{f}_c$块为空，使接触力自由地被动力学方程调节。

3. 线性等式约束
   - 同章节3.2，结论为公式(3.2.4)。
  
4. 线性不等式约束
   - 包含摩擦锥线性近似和力矩上下界，见章节3.5。

5. 最终QP形式
   \[ \tag{3.4.4}
   \begin{aligned}
   \underset{\mathbf{z}}{\text{min}} \ & \frac{1}{2} \mathbf{z}^{\top} H \mathbf{z} + h^{\top} \mathbf{z} \\
   \text{s.t.} \ 
   & \mathbf A \mathbf{z} = \mathbf b \quad \text{硬等式} \\
   & G \mathbf{z} \leq g \quad \text{线性不等式}
   \end{aligned}
   \]

   - 问题规模示例（双足支撑12dof）:
     - 变量 $|\mathbf{z}| = (6+12) + (2\times 6) + 12 =42$
     - 等式行 $= (6 + 12) + (2\times 6) = 30$
     - 不等式行 $\approx$ (每脚20边锥 $\to$ 两脚40边锥) + $12\times 2$（力矩限幅） $=  64$
     OSQP 典型迭代 $\leq 15$, 执行时间 $\approx 0.1$ ms @ Intel i7 - 1185G7。

6. 软约束
   若担心偶发不可行，可在(3.4.5)引入非负松弛变量$\boldsymbol\sigma$：
   \[
   G \mathbf{z} - \boldsymbol{\sigma} \leq g, \quad \boldsymbol{\sigma} \geq 0,
   \]
   并把成本改为$\frac{1}{2} \mathbf{z}^{\top} H \mathbf{z} + h^{\top} \mathbf{z} + \rho \| \boldsymbol{\sigma} \|_1$，（$\rho$ 大权重）。
   - **优点**: QP始终可行;
   - **缺点**: 任务允许小幅违背$\to$依赖$\rho$调参。

### 3.5 约束线性化
- 把**摩擦锥**、**足底力矩**、**关节力矩限幅**写成显示线性不等式的形式；
- 构造拼装矩阵$G$，向量$g$，供单层QP调用。

1. 单脚6维接触扳矢（Wrench Cone）的符号表示：脚底整体的压力积分与摩擦剪应力积分
   \[ \tag{3.5.1}
   \mathbf{w}_k = \begin{bmatrix}
   f_{x,k} \\
   f_{y,k} \\
   f_{z,k} \\
   \tau_{x,k} \\
   \tau_{y,k} \\
   \tau_{z,k}
   \end{bmatrix}, \quad k \in \{Left,\, Right\}
   \]

   - $(f_x, f_y, f_z)$——足底坐标系原点（几何中心）处的力
   - $(\tau_x, \tau_y, \tau_z)$——同原点处的力矩（扳矢）
   - 足底坐标系: $x$ 向前, $y$ 向左, $z$ 向上（右手系）

2. Coulomb摩擦锥的线性内逼近
   - 理想摩擦圆锥(Coulomb Cone)
   \[ \tag{3.5.2}
   \sqrt{f_x^2 + f_y^2} \leq \mu f_z, \quad f_z \geq 0
   \]

   - 八边形近似：用横截面的切平面包络（常用 4~8 条边）
   \[ \tag{3.5.3}
   \begin{aligned}
   \pm f_x + \underbrace{\frac{1}{\tan (360/8)^{\circ}}}_{=1} f_y &\leq \mu f_z \\
   \pm f_x - f_y &\leq \mu f_z \implies {F}_{\text{fric}} \mathbf{w}_k \leq 0 \\
   f_z &\geq 0
   \end{aligned}
   \]
     - 行数8（锥侧面）+1（$f_z \geq 0$)=9；
     - 系数矩阵${F}_{\text{fric}} \in \mathbb{R}^{9 \times 6}$ 只在 $(f_x, f_y, f_z)$列。
   - **思想核心**：连续圆锥在横截面取圆，用多边形切线包络得到线性半空间，$F_\text{fric}$行数等于边数+1，误差随边数增加单调递减。

3. 足底力矩/中心压力（Center of Pressure，CoP）约束
   
   CoP是让足底的俯仰/滚转力矩为0的点，设足底长宽半尺寸$l_x, l_y$（约0.09m），不发生翻倒/脚边缘翘起即要求CoP位于矩形内:
   \[ \tag{3.5.4}
   \begin{cases}
   |\tau_{x,k}| \leq l_y f_{z,k} \\
   |\tau_{y,k}| \leq l_x f_{z,k}
   \end{cases}
   \implies {F}_{\text{CoP}} \mathbf{w}_k \leq 0
   \]
   - 展开即4条不等式:$\tau_x - l_y f_z \leq 0,\, -\tau_x - l_y f_z \leq 0,\, \tau_y - l_x f_z \leq 0,\, -\tau_y - l_x f_z \leq 0$
   
   若要限制**垂向扭矩**$\tau_{z,k}$ 与摩擦耦合，可加
   \[ \tag{3.5.5}
   |\tau_{z,k}| \leq \gamma \mu l_{\text{avg}} f_{z,k}
   \]
   （$\gamma \approx 0.5$越小越保守、$l_{\text{avg}} = (l_x + l_y)/2$）——同样化成2条线性面。

4. 拼装单脚约束矩阵
   \[ \tag{3.5.6}
   {F}_{\text{foot}} = \begin{bmatrix}
   {F}_{\text{fric}} \\
   {F}_{\text{CoP}}
   \end{bmatrix} \in \mathbb{R}^{m_f \times 6}, \quad m_f = (8+1) + 4 (+2) = 13 \text{ 或 } 15
   \]

   对应向量全为零: ${F}_{\text{foot}} \mathbf{w}_k \leq {0}$，对应落在扳锥内，说明**任何细分的接触压力/摩擦分布都能找到一个满足库仑模型的解，从而不滑、不翻、不扭裂**。

5. 双足/单足场景的整体${G}_f, {g}_f$
   - **双足支撑**：
   \[ \tag{3.5.7}
   {G}_f = \text{blkdiag}({F}_{\text{foot}}, {F}_{\text{foot}}), \quad {g}_f = {0}_{2m_f}
   \]
   - **单足支撑**：只保留当前支撑脚那一块即可。
   - 在变量$\mathbf{z} = [\ddot{{q}}; \mathbf{f}_c; \boldsymbol{\tau}]$中，$\mathbf{f}_c$排列先左脚后右脚$\Rightarrow {G}_f$直接乘在$\mathbf{z}$的相应12列，其他列填0。

6. 关节力矩限幅
   URDF/XML给出
   \[ \tag{3.5.8}
   \boldsymbol{\tau}_{\min} \leq \boldsymbol{\tau} \leq \boldsymbol{\tau}_{\max} \quad 
   \]
   化成两组行:
   \[ \tag{3.5.9}
   \begin{aligned} + I_n \boldsymbol{\tau} &\leq \boldsymbol{\tau}_{\max} \\ - I_n \boldsymbol{\tau} &\leq - \boldsymbol{\tau}_{\min}
   \end{aligned}
   \]
   记
   \[ \tag{3.5.10}
   {G}_{\tau} = \begin{bmatrix}
   0 & 0 & I_n \\
   0 & 0 & -I_n
   \end{bmatrix}, \quad {g}_{\tau} = \begin{bmatrix}
   \boldsymbol{\tau}_{\max} \\
   -\boldsymbol{\tau}_{\min}
   \end{bmatrix}
   \]

7. 矩阵拼装规则
   \[ \tag{3.5.11}
   {G} = \begin{bmatrix}
   {G}_f \\
   {G}_{\tau}
   \end{bmatrix}, \quad
   {g} = \begin{bmatrix}
   {g}_f \\
   {g}_{\tau}
   \end{bmatrix}
   \]

   - 拼装行数示例（双足支撑 12 DoF）:
     - ${G}_f$: $2 \times 13 = 26$行（如果加$\tau_z$约束则 30 行）；
     - ${G}_{\tau}$: $2n = 24$行；
     - 总计$50 - 54$行。

### 3.6 层次QP（Hierarchical QP，HQP）机制
- Q：为什么要搞“层次”，权重调不就行了吗？
- Q：严格优先级在数学上如何实现，不让低层任务“顶掉”高层？
- 多任务往往有硬优先级：接触保持 > 质心平衡 > 摆动足 > 关节姿态；
- 硬约束必须满足；软约束通过权重矩阵调节优先级；
- **思路**：按层次递归解QP，每层在上一层null-space内再优化下一优先级任务；
- 严格多任务优先级，无权重冲突。

1. 为何需要“硬优先级”
   - 软权重法（单层QP）把所有任务塞进一个二次项，用$W_1 \gg W_2 \gg \cdots$控制优先级。
     - 缺点：
       - 极端权比导致Hessian病态（条件数变大），求解误差放大；
       - 当约束临界不可行时，低层仍可能“挤走”高层。
   - 硬优先级法（HQP）保证：
     - 所有高层等式/不等式先满足；
     - 低层只能用剩余自由度（null-space）优化，不会破坏高层结果。

2. 相关符号表示
   | 符号 | 含义 | 维度 |
   | --- | --- | --- |
   | $k$ | HQP 层号，0 最高 | — |
   | $\mathcal{T}_k$ | 第$k$层任务集 | — |
   | ${J}_k, {b}_k$ | 该层堆叠雅可比/偏置 | $m_k \times (6 + n)$, $m_k$ |
   | ${N}_k$ | 第$k$层null-space投影 | $(6 + n) \times (6 + n)$ |

3. 第0层：硬等式约束 QP
   \[ \tag{3.6.1}
   \begin{aligned}
   \underset{\ddot{{q}}^{(0)}, \mathbf{f}_c^{(0)}, \boldsymbol{\tau}^{(0)}}{\text{min}} \ & 0 \\
   \text{s.t.} \quad 
   & \mathbf{A} \mathbf{z}^{(0)} = \mathbf{b} \quad  \quad (3.2.4)
   \end{aligned}
   \]

   - 只求**一个可行点**满足动力学和接触保持；
   - 得到$\ddot{{q}}^{(0)}$与null-space投影矩阵。$\boldsymbol{N}_0 = I - \mathbf{A}^+ \mathbf{A}$，$\mathbf A^+$ 为阻尼伪逆。

4. 第1层：质心&角动量任务
   \[ \tag{3.6.2}
   \begin{aligned}
   \underset{z_1}{\text{min}} \ 
   & \left\lVert {J}_{\text{CoM}} (\ddot{{q}}^{(0)} + \boldsymbol{N}_0 z_1) - {b}_{\text{CoM}} \right\rVert^2_{{W}_{\text{CoM}}} + \left\lVert {A}_{\text{ang}} (\ddot{{q}}^{(0)} + \boldsymbol{N}_0 z_1) - {b}_{\text{ang}} \right\rVert^2_{{W}_{\text{ang}}} \\
   & \text{s.t.} \quad {G}_f(\mathbf{f}_c^{(0)}) \leq 0, \quad (\text{可选})
   \end{aligned}
   \]

   - **变量**只有$z_1$（维度=null-space rank）。
   - 更新$\ddot{{q}}^{(1)} = \ddot{{q}}^{(0)} + \boldsymbol{N}_0 z_1^{\star}$，继续算下一层投影$\boldsymbol{N}_1 = \boldsymbol{N}_0 - \boldsymbol{N}_0 {J}_1^+ {J}_1$。

5. 第$k$层通式
\[ \tag{3.6.3}
\underset{z_k}{\text{min}} \ 
\left\lVert {J}_k \left( \ddot{{q}}^{(k - 1)} + \boldsymbol{N}_{k - 1} z_k \right) - \mathbf{b}_k \right\rVert^2_{{W}_k}
\]
更新
\[ \tag{3.6.4}
\ddot{{q}}^{(k)} = \ddot{{q}}^{(k - 1)} + \boldsymbol{N}_{k - 1} z_k^{\star}, \quad
\boldsymbol{N}_k = \boldsymbol{N}_{k - 1} - \boldsymbol{N}_{k - 1} {J}_k^+ {J}_k
\]递归直到最低层$\bar{k}$。

6. 工程替代：分块软/硬HQP
   实际实操中常用两层混合法：
   1. 层0（硬等式）=动力学+接触保持；
   2. 层1（大QP）把所有任务$J_{\text{task}}$软权重堆叠，摩擦锥/扭矩限幅做不等式

   \[
      \min_{\mathbf{z}} \frac{1}{2} \mathbf{z}^T H \mathbf{z} + {h}^T      \mathbf{z} \quad \text{s.t.} \quad \mathbf A\mathbf{z} = \mathbf{b}, \quad G\mathbf{z} \leq {g} \quad \quad (3.4.4)
   \]

   - 把优先级体现在权重比 $W_{\text{CoM}}:W_{\text{sw}}:W_{\text{torso}} = 100:10:1$。
   - 只需一次QP，省去多层伪逆；在绝大部分行走任务足够稳定。

7. 阻尼伪逆与数值稳定
   - 阻尼Tikhonov
   \[ \tag{3.6.5}
      J^{+} = J^{\top} (J J^{\top} + \lambda^{2} I)^{-1}, \quad \lambda = 10^{-3} - 10^{-2}
   \]
   防奇异；$\lambda$越大$\to$越保守。
   - 递归正交化：每更新$N_k$后做QR正交，抑制累积误差。
   - 任务过约束：若$J_k$行多于列&排满，使该层成为硬等式直接放在0层可能更稳。

8. HQP收敛与可行性讨论
   1. 层0可行$\Rightarrow$上层必可行，因为低层只在$N_0$内优化。
   2. 捕获早期不可行：若某层QP无解（如摆动足无法跟轨迹），可以：
      - 降低该层权重（软约束松弛）
      - 缩短期望步长，重新规划。
   3. 输入饱和：力矩限幅违背会传递到动力学等式$\Rightarrow$0层不可行$\Rightarrow$必须退步计划（replan）。

### 3.7 Null-space冗余利用
- Q：高优任务满足后，尚未被使用的自由度如何被“二次利用”？
- Q：最小加速度范数、关节中立位、关节限幅回避等常见二级目标如何写成线性/二次形式？
- Q：数值上如何在HQP或单层QP中安全添加这些附加代价？

1. Null-space基本公式
   \[ \tag{3.7.1}
      \boldsymbol N_k \in \mathbb{R}^{(6 + n) \times (6 + n)}
   \]
   为到第 $k$ 层 HQP 结束后剩余自由度（**硬等式和高优任务把系统的一部分自由度用掉，强行规定了$\ddot q$必须落在某个线性子空间里**）的正交投影（**保证不破坏已有的自由度分配，数值稳定，误差累计小**）（参见 3.6.4）。
   高优任务给出的关节加速度已固定为 $\ddot{{q}}^{(k)}$。
   二级修正可写成：
   \[ \tag{3.7.2}
      \ddot{{q}} = \ddot{{q}}^{(k)} + \boldsymbol N_k \ddot{{q}}_{\text{null}}
   \]
   其中$\ddot{{q}}_{\text{null}}$不会改变先前任何层任务/约束，但可继续最小化次级目标。

2. 最小加速度范数（抑振）
   \[ \tag{3.7.3}
   \min_{\ddot{{q}}_{\text{null}}} \frac{1}{2} \| \ddot{{q}}^{(k)} + \boldsymbol N_k \ddot{{q}}_{\text{null}} \|^2
   \]
   解为：
   \[ \tag{3.7.4}
   \ddot{{q}}_{\text{null}}^{\star} = - \left( \boldsymbol N_k^{\top} \boldsymbol N_k \right)^+ \boldsymbol N_k^{\top} \ddot{{q}}^{(k)}
   \]
   - 物理意义：找到欧几里得范数最小的关节加速度，实现“最软动作”，抑制结构振动。
   - 数值上可直接在末层QP加一项$\| \ddot{{q}} \|^2_{W_{\text{null}}}$，权重较小即可。

3. 关节中立位保持
   定义势能：
   \[ \tag{3.7.5}
   U_{\text{cent}} = \frac{1}{2} ({q} - {q}_0)^{\top} K_c ({q} - {q}_0)
   \]
   梯度：$\nabla_{{q}} U_{\text{cent}} = K_c ({q} - {q}_0)$
   二级最小化：
   \[ \tag{3.7.6}
   \min_{\ddot{{q}}_{\text{null}}} \| K_c ({q} - {q}_0) \|^2
   \]
   等效地在 QP 末层加“姿态偏移”二次项：
   \[ \tag{3.7.7}
   \| {q} - {q}_0 \|^2_{K_c}
   \]
   - 常设$K_c = \text{diag}(1 - 5)$，${q}_0$为各关节中位或零位。
   - 好处：行走时自动让手臂/膝盖收回舒展，不显僵硬。

4. 节限幅回避（Barrier / Potential）
   对每关节$j$近极限$q_{j,\max}$处放置势场：
   \[ \tag{3.7.8}
   U_j = \frac{1}{2} k_j \left( \frac{1}{q_{j,\max} - q_j} \right)^2, \quad q_j \to q_{j,\max} \Rightarrow U_j \to \infty
   \]
   梯度$\nabla_{q_j} U_j = k_j \frac{1}{(q_{j,\max} - q_j)^3}$
   将所有 $U_j$ 梯度映射成二次惩罚
   \[ \tag{3.7.9}
   \| \nabla_{{q}} U \|^2_{W_{\text{avoid}}} \quad \text{or} \quad w_j (q_{j,\max} - q_j)^{-6}
   \]
   - 权重$w_j$在数值实现中随余量递增（动态势场），防碰撞/打到底。
   - 高于力矩限幅前被触发，提前引导避限。

5. 组合式Null-space目标
   在末层QP统一添加
   \[ \tag{3.7.10}
   \min_{\mathbf{z}} \frac{1}{2} \| \ddot{{q}} \|^2_{W_{\text{null}}} + \frac{1}{2} \| {q} - {q}_0 \|^2_{K_c} + \sum_{j} w_j (q_{j,\max} - q_j)^{-6}
   \]
   - $W_{\text{null}} \ll$主任务权重，确保不破坏平衡/摆足。
   - 动态调整$w_j$根据剩余空间：$\Delta_j < 5^{\circ} \Rightarrow w_j \uparrow$。

6. 走路抑振 vs 摆臂平衡
   - 给膝/踝加重$K_c$立位$\rightarrow$腿部在摆动返回时不会过度伸屈。
   - 给肩/肘加“最小加速度+关节中立位”$\rightarrow$在保证CoM、足端任务不变下，手臂自然随步伐平衡，减少躯干扭动。
   - 遇到推扰时，**高层CoM/角动量**任务先抢自由度，手臂抑振任务自动让位。

### 3.8 力矩回代与硬件接口
   1. 关节力矩回代公式
   求得$\ddot q ^*, f_c^*$后，代回公式(3.2.1)反算关节力矩：
   \[ \tag{3.8.1}
   \tau^* = S(M \ddot q^* + h - J_c^\top f_c^*)
   \]
      1. 时间一致性：$M,H,J_c$必须与同一周期的$\ddot q^*, \mathbf{f}_c^*$对应（不能用旧缓存）；
      2. 数值检查：若$|\tau_j^\star| > \tau_{\text{max},j} + 5\%$说明求解器或模型失配，触发安全回退。
   
   2. 下发模式
      1. 纯力矩模式（ID$+\tau$）：直接把电机力矩$\tau_\text{cmd} = \tau^*$设定送给电机驱动器；优点稳定、柔顺、弹性好；但需高带宽扭矩控制器和零偏/摩擦建模；
      2. 力矩-PD叠加：发送$\tau_\text{cmd} = \tau^* + K_p (q_\text{ref} - q) + K_d (\dot q_\text{ref} - \dot q)$。；在真实机器人摩擦、建模误差大时更抗漂移；$K_p, K_d$取50–100Hz带宽即可，数值过大会破坏HQP优先级。经验值$K_p = 50-100\text{N} \cdot \text{m/rad}$，$K_d = 2\sqrt{K_pI}$。

### 本章小结
由外环MPC提供质心参考、角动量参考，另外设定步态参考和躯干姿态参考，交由WBC-HQP满足动力学接触硬等式约束、扳锥和力矩限幅不等式约束下的任务误差二次项成本最小化，最后发送执行力矩。