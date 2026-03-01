# 自适应PI控制器-温度控制系统

## 1 温度控制系统的数学模型

一般而言，以温度为被控量的系统，例如电炉、热水壶等，通常可以认为是一个具有较大惯性的一阶系统,温度信号$y(t)$与输入功率$u(t)$之间的关系满足微分方程
$$T\dot{y}(t)+y(t)=Ku(t),$$其中$T$为惯性时间常数，$K$为静态增益。则开环系统的传递函数表示为
$$G(s):=\dfrac{Y(s)}{U(s)}=\dfrac{K}{Ts+1}.$$

## 2 PI控制器

### 基本原理

PID是应用最广泛的一种控制算法，包含两个部分：比例P（传递函数为%K_p%）、积分I（传递函数为$\dfrac{K_i}{s}$）。二者线性相加，组合成标准的PI控制器：
$$C(s)=K_p+\dfrac{K_i}{s}=\dfrac{K_ps+K_i}{s},$$其中，$K_p,K_i$分别称作比例、积分系数。

对于一般的使用，可基于实验人工地对三个参数进行试凑和调整，以达到较好的控制效果，常用方法有飞升曲线法、Ziegler-Nichols法等。

然而，这些调参方法往往过多地依赖经验与实验，难以从理论上保证控制效果的最优；此外，在系统本身或实验环境发生变化时，先前整定完毕的参数可能不再适用，需重新整定。因此，标准PI算法存在较大的局限和不便。（此处还未考虑积分饱和等问题，如果考虑这些因素，标准PI的性能还将进一步下降）

$$\Phi(s)=\dfrac{C(s)G(s)}{1+C(s)G(s)}=\dfrac{N(s)}{D(s)}.$$             计算得
$$N(s) = K(K_ps + K_i),$$ $$D(s) = Ts^2 + (KK_p+1)s + KK_i.$$由Laplace变换的终值定理，当参考值为阶跃信号$R(s)=\dfrac{R_0}{s}$时，有
$$y(\infty)=\lim_{s\to 0}sR(s)\dfrac{N(s)}{D(s)}=R_0.$$该结果表明，采用PI控制器，只要系统能够稳定且$K_i\neq 0$，则必定对阶跃信号无静差。

考虑等价的特征多项式$$F(s) = s^2 +  \dfrac{KK_p+1}{T}s+\dfrac{KK_i}{T},$$各参数与自然频率、阻尼比之间的关系为
$$\omega_n^2 = \dfrac{KK_i}{T},\quad 2\zeta\omega_n = \dfrac{KK_p+1}{T}.$$
对于期望的自然频率与阻尼比$\omega_n^*,\zeta^*$，则应取控制参数为
$$K_i^* = \dfrac{{\omega_n^*}^2T}{K},\quad K_p^* = \dfrac{2\zeta^*\omega_n^*T-1}{K}.$$
注：自然频率与阻尼比$\omega_n,\zeta$与实际响应性能的关系近似满足：
$$\sigma = e^{-\frac{\zeta\pi}{\sqrt{1-\zeta^2}}}\times 100\%,\quad t_s \approx \dfrac{4}{\zeta\omega_n}.$$其中，$\sigma$为超调量，$t_s$为输出进入2%误差带所需的调节时间。

### 抗饱和措施
采用最简单的积分分离法抗饱和（又称钳位法），即积分系数$K_i$在输出饱和时置零：
$$K_i = \begin{cases}\dfrac{{\omega_n^*}^2T}{K}, & u_{\min} \le u(t)\le u_{\max}\\ 0, & \mathrm{else}\end{cases}.$$

### 离散实现
采用前向差分离散法：
$$s = \dfrac{1-z^{-1}}{T_s z^{-1}},$$则离散化的PI控制器为
$$\begin{aligned}C(z) & = K_p+K_i\cdot\dfrac{T_s z^{-1}}{1-z^{-1}} \\ & = \dfrac{K_p + (K_iT_s - K_p) z^{-1}}{1-z^{-1}}.\end{aligned}$$
执行时：
$$u(k) = u(k-1) + K_p e(k) + (K_iT_s - K_p) e(k-1).$$

## 3 自适应PI控制器
要得到理想或指定（通过指定$\omega_n,\zeta$）控制效果的前提是，系统参数$K,T$已知。然而实际系统的参数往往是未知的，甚至还可能是时变的。因此，要实现性能更稳健、适应性更强的控制，就需要引入自适应机制。

在计算机控制系统中，所有信号均以零阶保持的采样形式存在，若以$T_s$为采样时间，则传递函数应离散化为
$$G(z):=\dfrac{Y(z)}{U(z)}=\dfrac{Kz^{-1}\left(1-e^{-T_s/T}\right)}{1-z^{-1}e^{-T_s/T}}=:\dfrac{q z^{-1}}{1-pz^{-1}}.$$其中$q:=K\left(1-e^{-T_s/T}\right),\;p:=e^{-T_s/T}$，写作差分方程形式：
$$y(k+1) = py(k) + qu(k).$$
定义以下向量：
$$ \boldsymbol{\varphi}(k):=\left[y(k),u(k)\right]^\top,\quad \hat{\boldsymbol{\theta}}(k):=\left[\hat{p}(k),\hat{q}(k)\right]^\top.$$则系统参数可通过下式进行估计：
$$\begin{cases}\hat{\boldsymbol{\theta}}(k)=\hat{\boldsymbol{\theta}}(k-1) + \boldsymbol{H}(k)\left[y(k)-\boldsymbol{\varphi}^\top(k)\hat{\boldsymbol{\theta}}(k-1)\right],\\ \\ \boldsymbol{H}(k) = \dfrac{\boldsymbol{P}(k-1)\boldsymbol{\varphi}(k)}{1+\boldsymbol{\varphi}^\top(k)\boldsymbol{P}(k-1)\boldsymbol{\varphi}(k)},\\ \\ \boldsymbol{P}(k) = \left[\boldsymbol{I}-\boldsymbol{H}(k)\boldsymbol{\varphi}^\top(k)\right]\boldsymbol{P}(k-1).\end{cases}$$
初值确定为$\boldsymbol{P}(0) = \boldsymbol{0},\;\hat{\boldsymbol{\theta}}(0)=\left[p_0,q_0\right]^\top.$

由估计结果还原出系统参数：
$$\begin{cases}\hat{T}=-\dfrac{T_s}{\ln(\hat{p})}, \\ \hat{K} = \dfrac{\hat{q}}{1-\hat{p}}.\end{cases}$$
