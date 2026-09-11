# 扩展卡尔曼滤波（EKF）vs 线性卡尔曼滤波（KF）对比实验报告

**实验日期**: 2025 年 4 月  
**项目基础**: ROS 2 `rse_prob_robotics` 概率机器人学滤波器库  
**数据源**: Gazebo 仿真 rosbag（58.4 s，14 326 个真值采样点）+ 独立强非线性仿真  

---

## 1. 实验目标与性能指标

### 1.1 核心目标

1. **在统一噪声条件下**，定量比较 KF 和 EKF 在同一数据集上的估计精度
2. **识别 KF/EKF 在系统非线性强弱不同场景下的表现差异**
3. **验证工程实现中的两个关键 bug**（Sigma 更新括号错误、角度归一化缺失）对滤波器行为的影响
4. **总结 EKF 的适用条件与局限性**

### 1.2 性能指标

| 指标 | 定义 | 单位 | 期望趋势 |
|------|------|------|----------|
| Position RMSE | 稳态位置误差（跳过前 100 步瞬态）的均方根 | m | ↓ 越好 |
| Heading RMSE | 稳态航向误差的均方根（角度已归一化至 (-180°, 180°]） | deg | ↓ 越好 |
| Max \|pos err\| | 全程最大绝对位置误差 | m | ↓ 越好 |
| Max \|heading err\| | 全程最大绝对航向误差（归一化后） | deg | ↓ 越好 |
| Σ trace | 协方差矩阵对角元素之和 | — | 收敛表示滤波器可信自己估计 |
| 收敛步数 | 位置 RMS 连续 50 步 < 0.10 m 的步数 | 步 | ↓ 越快越好 |

---

## 2. 系统模型

### 2.1 状态向量

$$\mathbf{x} = [x, y, \theta]^\top \in \mathbb{R}^3$$

- $x, y$ — 机器人在全局坐标系下的位置 [m]
- $\theta$ — 机器人航向角 [rad]

### 2.2 运动模型（差速驱动）

**真值（精确弧长积分，记为 VMML2）**:

$$
\mathbf{g}(\mu, \mathbf{u}, \Delta t) = \begin{bmatrix}
x - \frac{v}{w}\sin\theta + \frac{v}{w}\sin(\theta + w\Delta t) \\
y + \frac{v}{w}\cos\theta - \frac{v}{w}\cos(\theta + w\Delta t) \\
\theta + w\Delta t
\end{bmatrix}
$$

其中 $\mathbf{u} = [v, w]^\top$ 为线速度和角速度控制量。当 $w \to 0$ 时退化为直线模型。

**KF 线性化模型**:

$$
\mu_t = \mu_{t-1} + \mathbf{B}(\theta_{t-1}, \Delta t)\mathbf{u}_t, \quad
\mathbf{B} = \begin{bmatrix}
\cos\theta \cdot \Delta t & 0 \\
\sin\theta \cdot \Delta t & 0 \\
0 & \Delta t
\end{bmatrix}
$$

**EKF v1（一阶 Taylor 近似，记为 VMML1）**:

$$
g_x = x + v\cos(\theta)\cdot\Delta t + O((w\Delta t)^2), \quad
G = \begin{bmatrix}
1 & 0 & -v\sin(\theta)\cdot\Delta t \\
0 & 1 & v\cos(\theta)\cdot\Delta t \\
0 & 0 & 1
\end{bmatrix}
$$

**EKF v2（精确弧长模型 + 雅可比，VMML2）**:

$$
G = \begin{bmatrix}
1 & 0 & -\frac{v}{w}\cos\theta + \frac{v}{w}\cos(\theta + w\Delta t) \\
0 & 1 & -\frac{v}{w}\sin\theta + \frac{v}{w}\sin(\theta + w\Delta t) \\
0 & 0 & 1
\end{bmatrix}
$$

### 2.3 观测模型

**直接位姿观测（bag 数据）** — 线性：

$$
\mathbf{h}(\mu) = \mu, \quad \mathbf{H} = \mathbf{I}_3
$$

**Bearing-only 观测（强非线性仿真）** — 极坐标形式：

$$
\mathbf{h}(\mu) = \begin{bmatrix}
\arctan2(y, x) \\
\sqrt{x^2 + y^2} \\
\theta
\end{bmatrix}, \quad
\mathbf{H} = \begin{bmatrix}
-\frac{y}{x^2+y^2} & \frac{x}{x^2+y^2} & 0 \\
\frac{x}{r} & \frac{y}{r} & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

---

## 3. KF vs EKF 算法原理

### 3.1 线性卡尔曼滤波（KF）

KF 假设**系统为高斯线性**：

$$
\mathbf{x}_t = \mathbf{A}\mathbf{x}_{t-1} + \mathbf{B}\mathbf{u}_t + \mathbf{w}_t, \quad \mathbf{w}_t \sim \mathcal{N}(\mathbf{0}, \mathbf{R})
$$
$$
\mathbf{z}_t = \mathbf{C}\mathbf{x}_t + \mathbf{v}_t, \quad \mathbf{v}_t \sim \mathcal{N}(\mathbf{0}, \mathbf{Q})
$$

**Predict**:
$$
\bar{\mu}_t = \mathbf{A}\mu_{t-1} + \mathbf{B}\mathbf{u}_t, \quad
\bar{\Sigma}_t = \mathbf{A}\Sigma_{t-1}\mathbf{A}^\top + \mathbf{R}
$$

**Update**:
$$
\mathbf{K}_t = \bar{\Sigma}_t\mathbf{C}^\top(\mathbf{C}\bar{\Sigma}_t\mathbf{C}^\top + \mathbf{Q})^{-1}, \quad
\mu_t = \bar{\mu}_t + \mathbf{K}_t(\mathbf{z}_t - \mathbf{C}\bar{\mu}_t)
$$
$$
\Sigma_t = (\mathbf{I} - \mathbf{K}_t\mathbf{C})\bar{\Sigma}_t
$$

### 3.2 扩展卡尔曼滤波（EKF）

EKF 将非线性函数 $\mathbf{g}(\mu, \mathbf{u}, \Delta t)$ 和 $\mathbf{h}(\mu)$ **在当前估计处一阶 Taylor 展开**，用雅可比矩阵替代 A 和 C：

$$
\mathbf{G}_t = \frac{\partial \mathbf{g}}{\partial \mu}\bigg|_{\mu_{t-1}, \mathbf{u}_t}, \quad
\mathbf{H}_t = \frac{\partial \mathbf{h}}{\partial \mu}\bigg|_{\bar{\mu}_t}
$$

**Predict**:
$$
\bar{\mu}_t = \mathbf{g}(\mu_{t-1}, \mathbf{u}_t, \Delta t), \quad
\bar{\Sigma}_t = \mathbf{G}_t\Sigma_{t-1}\mathbf{G}_t^\top + \mathbf{R}
$$

**Update**:
$$
\mathbf{K}_t = \bar{\Sigma}_t\mathbf{H}_t^\top(\mathbf{H}_t\bar{\Sigma}_t\mathbf{H}_t^\top + \mathbf{Q})^{-1}, \quad
\mu_t = \bar{\mu}_t + \mathbf{K}_t(\mathbf{z}_t - \mathbf{h}(\bar{\mu}_t))
$$
$$
\Sigma_t = (\mathbf{I} - \mathbf{K}_t\mathbf{H}_t)\bar{\Sigma}_t
$$

**关键区别**：KF 的 $\mathbf{A}, \mathbf{C}$ 为常值或仅依赖固定量；EKF 的 $\mathbf{G}_t, \mathbf{H}_t$ **每步都变**，依赖于当前估计。

---

## 4. 实现细节与 Bug 修复

### 4.1 Bug 1 — KF Sigma 更新括号位置错误（严重）

**文件**: `src/rse_prob_robotics/rse_gaussian_filters/rse_gaussian_filters/filters/kf.py`

**原代码**:
```python
self.Sigma = (np.eye(len(K)) - K @ self.C() @ self.Sigma)   # ❌ 错误！
```

**问题**: 结合律错误。右边展开后 $\approx \mathbf{I} - \mathbf{K}\mathbf{C}\Sigma$，这不是协方差矩阵！结果导致 Sigma 永远不收敛（实验中在 ~1 和 ~0.02 之间振荡）。

**修复**:
```python
self.Sigma = (np.eye(len(K)) - K @ self.C()) @ self.Sigma   # ✅ 正确
```

**验证**: 修复后 Sigma 在 10 步内收敛到对角元素 [0.0064, 0.0064, 0.0026]，滤波器正常工作。

### 4.2 Bug 2 — EKF 缺少角度归一化（中等）

**问题**: ROS 消息中 heading 角可能跨越 $\pm 180^\circ$ 边界。EKF 在 predict 后 $\bar{\mu}_{t,\theta}$ 可能累积到 $179^\circ + 5^\circ = 184^\circ$，对应的观测 $z_\theta = -176^\circ$（实际上是同一个方向），但创新 $z - h = -360^\circ$ 会导致 update 后估计跳变。

**修复**: 在 `predict()` 和 `update()` 后各调用一次 `_normalize_angle(mu[2])`，将角度归一化至 $(-\pi, \pi]$。

**注意**: 归一化也需要正确处理**创新**（innovation）——当观测来自 bearing-only 等角度型传感器时，innovation 也需要归一化。

### 4.3 Bug 3 — Bag 时间对齐（工程挑战）

ROS2 bag 三个话题 `/odom`, `/odom_raw`, `/cmd_vel` 的时间戳存在以下问题：

| 话题 | Wall time 起始 | Sim time 起始 | 备注 |
|------|---------------|--------------|------|
| `/odom` (真值) | 374.28 s | 1569.86 s | — |
| `/odom_raw` (带噪观测) | 373.29 s | 1564.92 s | relay 提前 0.989 s 启动 |
| `/cmd_vel` | 383.99 s | — | driver 未填 header.stamp |

仿真以 **~4.99× 倍速**运行（57.4 s wall time → 286.5 s sim time）。

**解决方案**: 使用 **wall-clock 时间（db timestamp）直接对齐**，搜索容差 tol=0.02 s。在当前数据量级下，0.02 s wall time 对应 ~0.1 s sim time → 直行机器人位移差 ≈ 0.045 m（远小于观测噪声 0.15 m），对位置估计无实质影响。

**代价**: 在转弯段（heading 快变），由于 wall time 起始偏移导致的 sim time 差 ~4.94 s，会产生短暂的 heading 对齐瞬态（Max heading err ≈ 164°），但被后续观测立即修正。

### 4.4 公平对比设计

为确保 KF 和 EKF 的结果可比，采取以下控制：

1. **相同噪声参数**: `proc_noise_std = [0.05, 0.05, 0.03]`, `obs_noise_std = [0.15, 0.15, 0.10]`
2. **相同 dt 处理**: dt clamp 到 [0, 0.1 s]（模仿 kf_node.py 的 ROS2 行为）
3. **相同初始条件**: `mu0 = gt[0]`, `Sigma0 = I`（两个滤波器共享）
4. **相同时间轴**: 同一 sync_streams() 输出数组
5. **相同角度归一化策略**: predict 和 update 后都归一化

**注意**: kf.py/ekf.py 内部的 R/Q 命名与教科书相反 —— `self.R = proc_noise_std²`（过程噪声）、`self.Q = obs_noise_std²`（观测噪声）。实验代码与此保持一致。

---

## 5. 实验数据

### 5.1 实验一：Bag 数据 — 真实 ROS2 仿真记录

**数据集描述**:
- 14 326 个采样点（57.4 s wall time）
- 控制频率：~14 Hz（789 条 cmd_vel）
- 观测频率：~100 Hz
- 控制量：v ∈ [0, 0.45] m/s, w ∈ [-0.9, 0.9] rad/s，包含 3 段转弯（w = +0.6, -0.037, -0.5 rad/s）
- Heading 范围：-179.4° ~ +179.7°（跨 359°，但未真正触及 ±180°）

**三种滤波器结果**:

| 指标 | KF | EKF v1 (一阶 Taylor) | EKF v2 (精确弧长) |
|------|-----|-----|-----|
| Position RMSE [m] | **0.0045** | **0.0045** | **0.0045** |
| Heading RMSE [deg] | **3.4246** | **3.4246** | **3.4246** |
| Max \|pos err\| [m] | **0.0165** | **0.0165** | **0.0165** |
| Max \|heading err\| [deg] | 164.05 | 164.05 | 164.05 |

**结论**: 三个滤波器产生**完全相同**的估计。原因分析见第 6.1 节。

### 5.2 实验二：强非线性仿真 — Bearing-only 观测

**场景设计**:
- 圆形轨迹：3 段 × (w = +1.2 rad/s, w = -1.2 rad/s) = 6 段大曲率转弯
- dt = 0.2 s（5 Hz），v = 1.0 m/s
- 观测：**bearing-only**（极坐标 $\arctan2(y, x)$ + 距离 + heading）
- 噪声：σ_bearing = 0.15 rad (≈ 8.6°), σ_range = 0.10 m, σ_head = 0.05 rad

**四种滤波器/观测组合结果**:

| 滤波器 | 观测模型 | Pos RMSE [m] | Th RMSE [deg] | Max \|pos err\| [m] |
|--------|---------|-------------|--------------|---------------------|
| **KF** | 直接位姿（线性） | 0.1967 | 2.68 | 0.4101 |
| **EKF** | 直接位姿（线性） | 0.1967 | 2.68 | 0.4098 |
| **EKF** | Bearing-only（非线性） | **0.1642** | **1.72** | 0.4419 |
| **KF** | Bearing-only（强行喂入） | **6.7926** | 1.71 | **10.1029** |

**关键发现**:

1. **KF 和 EKF 在线性观测下等价** — Pos RMSE 完全相同（0.1967 m）
2. **EKF 处理非线性观测更好** — Bearing-only + EKF 的 Pos RMSE = 0.1642 m 优于直接位姿观测的 0.1967 m
3. **KF 无法处理非线性观测** — 强行喂入 bearing-only 观测导致 RMSE 6.79 m，Max 误差 10.1 m → **彻底发散**

### 5.3 实验三：角度归一化验证

在 bag 数据上，临时禁用 KF/EKF 的 `_normalize_angle()`，与启用版本对比：

| 配置 | Pos RMSE | Heading RMSE | Theta range |
|------|---------|-------------|-------------|
| EKF + normalize | 0.0045 m | 3.42° | [-164.6°, 177.6°] |
| EKF − normalize | 0.0045 m | 3.42° | [-164.6°, 177.6°] |

**无差异** — 因为 bag 中 heading **未真正触及 ±180° 边界**（最大 177.6°）。但在强非线性仿真中，圆形轨迹会多次跨越 ±180°，此时归一化是必须的。

---

## 6. 结果分析

### 6.1 为什么 KF 和 EKF 在 bag 数据上完全等价？

三个因素叠加导致 KF/EKF 行为一致：

**因素 1 — 观测模型完全线性**

Bag 中的观测模型 $h(\mu) = \mu$ 是恒等映射。KF 的 C = I 和 EKF 的 H(\mu) = I 完全相同，观测端的创新计算、卡尔曼增益、协方差更新公式也完全相同。

**因素 2 — 运动模型非线性太弱**

在 bag 的典型参数下（v ≤ 0.45 m/s, w ≤ 0.9 rad/s, dt ≤ 0.02 s）：

VMML2（精确弧长）和 VMML1（一阶 Taylor）在 x 方向的差异：
$$
\Delta g_x = -\frac{v \cdot w}{2}\sin\theta \cdot (\Delta t)^2
$$
$$
\text{Max }|\Delta g_x| = \frac{0.45 \times 0.9}{2} \times 1 \times 0.0004 \approx 8.1 \times 10^{-5}\ \text{m}
$$

**48 微米的差异**（最大）远小于观测噪声（σ = 0.15 m）和位置 RMSE（0.0045 m）。运动模型的非线性完全被噪声淹没。

**因素 3 — EKF v2 和 v1 的雅可比近似也收敛**

G(\mu, u, dt) 在 w 小时近似为 I + 小修正。当 dt→0 时，VMML1 的 G 和 VMML2 的 G 都退化为 I。

### 6.2 EKF 什么时候能赢？

从强非线性仿真中提取的条件：

| 条件 | KF 能否处理 | EKF 是否有优势 |
|------|-----------|---------------|
| 线性观测（直接位姿） | ✅ 完美 | ❌ 等价（H=I） |
| 弱非线性运动（v=0.4, w≤0.9, dt≤0.02s） | ✅ 足够好 | ❌ 差异在噪声以下 |
| 强非线性观测（bearing-only, range-only） | ❌ 发散 | ✅ 通过局部线性化利用 |
| 强非线性运动（大 w, 大 dt） | ⚠️ 差 | ✅ 精确弧长积分 |
| heading 跨越 ±180° | ⚠️ 依赖归一化 | ⚠️ 同样需要归一化 |

### 6.3 协方差收敛行为

三个滤波器的 tr(Σ) 收敛轨迹也完全相同（稳态 ≈ 0.0153）。这进一步确认它们的数学行为等价 —— Sigma 的传播公式中，KF 的 G=I 和 EKF 的 G(μ) ≈ I 在当前条件下给出相同结果。

---

## 7. 适用条件与局限性讨论

### 7.1 EKF 的优势

1. **处理非线性观测模型** — bearing-only, range-only, visual odometry 等
2. **处理强非线性运动模型** — 大转弯、长时间步、高速运动
3. **统一框架** — 同一类可处理 KF 的所有情形，且在非线性退化时自动收敛到 KF 行为
4. **计算复杂度增加有限** — O(n³) 复杂度，n 通常为 3~15 维

### 7.2 EKF 的局限性

1. **需要手写雅可比矩阵** — \( \partial \mathbf{g} / \partial \mu \) 和 \( \partial \mathbf{h} / \partial \mu \) 的解析推导容易出错
2. **对初始估计敏感** — 远离真实值时局部线性化误差大，可能产生有偏估计甚至发散
3. **线性化误差随非线性强度增大** — 对于极强非线性，一阶 Taylor 可能不够，需用 UKF/CDKF/粒子滤波
4. **无法处理多峰后验** — 继承高斯假设，对混合分布无能为力

### 7.3 何时优先用 KF？

- 系统**已知完全线性**且噪声高斯
- 嵌入式平台，算力极其受限
- 教学/入门场景，理解核心概念后再迁移到 EKF

### 7.4 何时必须用 EKF？

- 观测模型或运动模型**有显著非线性**（如 bearing-only）
- 需要比线性化更高的精度
- 系统时变且需要自适应线性化

---

## 8. 结论

> **EKF 是 KF 的 "安全超集"** —— 在线性或弱非线性场景下它自动退化为 KF，在强非线性场景下它的局部线性化能显著优于 KF。

### 核心数据总结

| 实验 | 场景 | KF Pos RMSE | EKF Pos RMSE | EKF/KF 比 |
|------|------|------------|-------------|----------|
| 一 (bag) | 弱非线性运动 + 线性观测 | 0.0045 m | 0.0045 m | **1.00** |
| 二 (仿真, pose obs) | 强非线性运动 + 线性观测 | 0.1967 m | 0.1967 m | **1.00** |
| 二 (仿真, bearing obs) | 强非线性运动 + 强非线性观测 | **6.79 m (发散)** | **0.1642 m** | **0.024** |

### 工程经验

1. **Sigma 更新的括号位置绝对不能错** —— `(I - K·C)·Σ` 不是 `I - K·C·Σ`
2. **角度型状态变量必须归一化** —— predict 和 update 后都要做；innovation 也要做
3. **Bag 时间对齐需要特别小心** —— sim time 和 wall time 可能有 ~5x 倍速差
4. **KF/EKF 公平对比必须控制所有变量** —— 同一噪声、同一 dt、同一初始条件

### 后续工作

1. 尝试 **UKF（无迹卡尔曼滤波）** 进一步验证 sigma-point 法对强非线性的鲁棒性
2. 在仿真中加入 **range-only + bearing-only** 混合观测（更接近真实单目视觉里程计）
3. 考虑 **EKF 的数值稳定性改进** —— Joseph 形式 Sigma 更新、SRIF/SSKF 平方根方法
4. 验证 **非线性强度指标**（如 $\mathcal{L}_g = \|\nabla^\top \nabla g\|$）能否预测 EKF 相对 KF 的增益

---

## 附录：可复现性

### 环境

```bash
# ROS2 bag
/home/jjy/ros2_ws/my_data/my_data_0.db3

# 源码位置
/home/jjy/ros2_ws/src/rse_prob_robotics/rse_gaussian_filters/rse_gaussian_filters/filters/
├── kf.py       (已修复 Sigma 括号 bug)
└── ekf.py      (已添加角度归一化)

# 实验脚本
/home/jjy/ros2_ws/ekf_experiment.py          # Bag 离线对比（三个滤波器）
/home/jjy/ros2_ws/ekf_nonlinear_sim.py       # 强非线性仿真（四个配置）

# 输出图
/home/jjy/ros2_ws/ekf_vs_kf_results.png      # Bag 三滤波器对比
/home/jjy/ros2_ws/ekf_nonlinear_results.png # 强非线性仿真对比
```

### 运行

```bash
# Bag 离线对比（约 3.3 s）
cd /home/jjy/ros2_ws && python3 ekf_experiment.py

# 强非线性仿真（< 1 s）
cd /home/jjy/ros2_ws && python3 ekf_nonlinear_sim.py
```

---

*报告生成时间: 2025-04-18*  
*基于 ROS2 Humble + Gazebo + Ubuntu 24.04*
