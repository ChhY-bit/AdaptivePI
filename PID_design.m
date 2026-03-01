clear,clc
%% 系统设置
T = 30;    % 惯性常数
K = 2;    % 静态增益
%% 控制器设计
% 期望性能（可调）
Ts = 30;   % 调节时间
PO = 0.02;  % 超调量
% 计算
zeta = sqrt(log(PO)^2/(pi^2+log(PO)^2));
omega = 4/zeta/Ts;
Kp = (2*zeta*omega*T - 1)/K;
Ki = (omega^2*T)/K;
Kd = 0;
% 预估结果
fprintf("======================================\n")
fprintf("参考调节时间:\t\t%.2f s\n",4/zeta/omega);    % 2%误差带
fprintf("参考超调量:\t\t%.2f%%\n",exp(-zeta*pi/sqrt(1-zeta^2))*100);
fprintf("--------------------------------------\n")
%% 设计结果
% 开环环节
G = tf(K,[T,1]);
% 控制器
s = tf('s');
PID = Kp + Ki/s +Kd *s;
% 闭环系统
Phi = feedback(PID*G,1);
% 阶跃
step(Phi);
copy = Phi;
copy.Numerator{1} = [0 0 K*Ki];
hold on
step(copy)
% 输出结果
info = stepinfo(Phi);
fprintf("--------------------------------------\n")
fprintf("实际调节时间:\t\t%.2f s\n",info.SettlingTime);    % 2%误差带
fprintf("实际超调量:\t\t%.2f%%\n",info.Overshoot);
fprintf("======================================\n")
