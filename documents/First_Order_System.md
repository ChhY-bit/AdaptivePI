# 自适应PI控制器（以温度控制为例）

## 1 温度控制系统的数学模型

一般而言，以温度为被控量的系统（例如电炉、热水壶等），通常可以认为是一个具有较大惯性的一阶系统。温度信号 $y(t)$ 与输入功率 $u(t)$ 之间的关系满足微分方程：

$$T\dot{y}(t)+y(t)=Ku(t) \tag{1}$$

其中，$T$ 为惯性时间常数，$K$ 为静态增益。则开环系统的传递函数表示为：

$$G(s):=\dfrac{Y(s)}{U(s)}=\dfrac{K}{Ts+1} \tag{2}$$

---

## 2 PI控制器

### 2.1 基本原理

PID 是应用最广泛的一种控制算法，包含两个部分：比例 P（传递函数为 $K_p$）、积分 I（传递函数为 $\dfrac{K_i}{s}$）。二者线性相加，组合成标准的 PI 控制器：

$$C(s)=K_p+\dfrac{K_i}{s}=\dfrac{K_ps+K_i}{s} \tag{3}$$

其中，$K_p, K_i$ 分别称作比例、积分系数。

对于一般的使用，可基于实验人工地对三个参数进行试凑和调整，以达到较好的控制效果，常用方法有飞升曲线法、Ziegler-Nichols 法等。

然而，这些调参方法往往过多地依赖经验与实验，难以从理论上保证控制效果的最优；此外，在系统本身或实验环境发生变化时，先前整定完毕的参数可能不再适用，需重新整定。因此，标准 PI 算法存在较大的局限和不便。（此处还未考虑积分饱和等问题，如果考虑这些因素，标准 PI 的性能还将进一步下降）

闭环传递函数为：

$$\Phi(s)=\dfrac{C(s)G(s)}{1+C(s)G(s)}=\dfrac{N(s)}{D(s)} \tag{4}$$

计算得：

$$N(s) = K(K_ps + K_i) \tag{5}$$

$$D(s) = Ts^2 + (KK_p+1)s + KK_i \tag{6}$$

由 Laplace 变换的终值定理，当参考值为阶跃信号 $R(s)=\dfrac{R_0}{s}$ 时，有：

$$y(\infty)=\lim_{s\to 0}sR(s)\dfrac{N(s)}{D(s)}=R_0 \tag{7}$$

该结果表明，采用 PI 控制器，只要系统能够稳定且 $K_i\neq 0$，则必定对阶跃信号无静差。

考虑等价的特征多项式：

$$F(s) = s^2 +  \dfrac{KK_p+1}{T}s+\dfrac{KK_i}{T} \tag{8}$$

各参数与自然频率、阻尼比之间的关系为：

$$\omega_n^2 = \dfrac{KK_i}{T},\quad 2\zeta\omega_n = \dfrac{KK_p+1}{T} \tag{9}$$

对于期望的自然频率与阻尼比 $\omega_n^*, \zeta^*$，则应取控制参数为：

$$K_i^* = \dfrac{{\omega_n^*}^2T}{K},\quad K_p^* = \dfrac{2\zeta^*\omega_n^*T-1}{K} \tag{10}$$

**注**：自然频率 $\omega_n$、阻尼比 $\zeta$ 与实际响应性能的关系近似满足：

$$\sigma = e^{-\frac{\zeta\pi}{\sqrt{1-\zeta^2}}}\times 100\%,\quad t_s \approx \dfrac{4}{\zeta\omega_n} \tag{11}$$

其中，$\sigma$ 为超调量，$t_s$ 为输出进入 2% 误差带所需的调节时间。反之，则有：

$$\zeta^* = \sqrt{\dfrac{\ln^2\sigma}{\pi^2+\ln^2\sigma}},\quad \omega_n^* = \dfrac{4}{t_s \zeta^* }. \tag{12}$$

### 2.2 离散实现

采用前向差分离散法：

$$s = \dfrac{1-z^{-1}}{T_s z^{-1}} \tag{13}$$

则离散化的 PI 控制器为：

$$\begin{aligned}C(z) & = K_p+K_i\cdot\dfrac{T_s z^{-1}}{1-z^{-1}} \\ & = \dfrac{K_p + (K_iT_s - K_p) z^{-1}}{1-z^{-1}} \tag{14}\end{aligned}$$

执行时：

$$u(k) = u(k-1) + K_p e(k) + (K_iT_s - K_p) e(k-1) \tag{15}$$

该式即为**增量式** PI 控制器的数学形式。若在控制程序中人为地对 $u(k)$ 限幅（而不是仅依靠实际执行器的物理约束），则该式具有天然的抗积分饱和能力。

---

## 3 自适应PI控制器

要得到理想或指定（通过指定 $\omega_n, \zeta$）控制效果的前提是，系统参数 $K, T$ 已知。然而实际系统的参数往往是未知的，甚至还可能是时变的。因此，要实现性能更稳健、适应性更强的控制，就需要引入自适应机制。

在计算机控制系统中，所有信号均以零阶保持的采样形式存在，若以 $T_s$ 为采样时间，则传递函数应离散化为：

$$G(z):=\dfrac{Y(z)}{U(z)}=\dfrac{Kz^{-1}\left(1-e^{-T_s/T}\right)}{1-z^{-1}e^{-T_s/T}}=:\dfrac{q z^{-1}}{1-pz^{-1}} \tag{16}$$

其中，$q:=K\left(1-e^{-T_s/T}\right),\;p:=e^{-T_s/T}$，写作差分方程形式：

$$y(k+1) = py(k) + qu(k) \tag{17}$$

定义以下向量：

$$ \boldsymbol{\varphi}(k-1):=\left[y(k-1),u(k-1)\right]^\top,\quad \hat{\boldsymbol{\theta}}(k):=\left[\hat{p}(k),\hat{q}(k)\right]^\top \tag{18}$$

则系统参数可通过下式进行递推估计：

$$\begin{cases}\hat{\boldsymbol{\theta}}(k)=\hat{\boldsymbol{\theta}}(k-1) + \boldsymbol{H}(k)\left[y(k)-\boldsymbol{\varphi}^\top(k-1)\hat{\boldsymbol{\theta}}(k-1)\right],\\ \\ \boldsymbol{H}(k) = \dfrac{\boldsymbol{P}(k-1)\boldsymbol{\varphi}(k-1)}{1+\boldsymbol{\varphi}^\top(k-1)\boldsymbol{P}(k-1)\boldsymbol{\varphi}(k-1)},\\ \\ \boldsymbol{P}(k) = \left[\boldsymbol{I}-\boldsymbol{H}(k)\boldsymbol{\varphi}^\top(k-1)\right]\boldsymbol{P}(k-1). \tag{19}\end{cases}$$

其中的初值确定为 $\boldsymbol{P}(0) = \boldsymbol{0},\;\hat{\boldsymbol{\theta}}(0)=\left[p_0,q_0\right]^\top$。

在充分激励的条件下，可以保证 $\lim_{k\to \infty}\hat{\boldsymbol{\theta}}(k)=\boldsymbol{\theta}(k)$。

由估计结果还原出系统参数：

$$\begin{cases}\hat{T}=-\dfrac{T_s}{\ln(\hat{p})}, \\ \hat{K} = \dfrac{\hat{q}}{1-\hat{p}}. \tag{20}\end{cases}$$

此基于参数估计结果设计符合需求的 PI 控制器即可：

$$ \hat{K}_p = \dfrac{2\zeta^* \omega_n^*\hat{T}-1}{\hat{K}},\quad \hat{K}_i = \dfrac{{\omega_n^*}^2\hat{T}}{\hat{K}} \tag{21}$$

---

## 4 算法总结

该自适应 PI 算法适用于任意的**一阶**被控对象。<span style="color: red">若明确已知系统不为线性的一阶系统，则应慎用本算法！</span>

**算法流程：**

1. **指定控制性能**

   根据控制需求，指定超调量 $\sigma$ 与调节时间 $t_s$，利用式 <span style="color: blue">(12)</span> 确定阻尼比 $\zeta^*$ 与自然频率 $\omega_n^*$。

2. **给定初始值**

   依据经验或实验，粗略地确定系统参数 $T, K$，并由式 <span style="color: blue">(16)</span> 确定参数初值 $p_0, q_0$。同时利用式 <span style="color: blue">(10)</span> 确定此时的 PI 参数 $K_{p}, K_{i}$。

3. **实施控制**

   将 PI 控制器投入实施，即式 <span style="color: blue">(15)</span>。

4. **实施辨识**

   利用上一控制周期的信息 $\boldsymbol{\varphi}(k-1)$ 与当前信息 $y(k)$，使用式 <span style="color: blue">(19)</span> 进行迭代估计，由式 <span style="color: blue">(20)</span> 还原出参数 $T, K$。

5. **调整参数**

   利用式 <span style="color: blue">(21)</span> 更新 PI 参数。

6. **回到第 3 步。**

