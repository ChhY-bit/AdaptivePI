%% 自适应PI控制器仿真-一阶系统
clear,clc,close all
% 以下三者必须且仅开启1者：
ifadapt = 1;    % 是否开启自适应
ifideal = 0;    % 是否理想参数
ifnorml = 0;    % 是否开启常规
%% 仿真参数（可调）
T_max = 250;    % 仿真时长
dt = 1e-2;      % 仿真步长
%% 系统参数（可调）
T = 40;     % 惯性时间常数
K = 1.5;      % 静态增益
y_ini = 0;  % 初值
y_ref = 60; % 给定值
%% 控制参数（可调）
T_stl = 30;     % 期望调节时间
PO = 0.05;      % 期望超调量
r_fun = @(t) 0.5*y_ref + 0.5*y_ref*(t>80) - 0.5*y_ref*(t>160);    % 参考信号
% r_fun = @(t) y_ref-20 + 0.*t + 20*sin(0.04*t);    % 参考信号
Ts = 1;         % 控制周期
u_max = 60;
u_min = 0;
% 手动PI参数
Kp_norml = 1;
Ki_norml = 0.5;
%% 预计算
% 理想参数
zeta_ideal = sqrt(log(PO)^2/(pi^2+log(PO)^2));
omega_ideal = 4/zeta_ideal/T_stl;
Kp_ideal = (2*zeta_ideal*omega_ideal*T - 1)/K;
Ki_ideal = (omega_ideal^2*T)/K;
% 预估结果
fprintf("======================================\n")
fprintf("参考调节时间:\t\t%.2f s\n",4/zeta_ideal/omega_ideal);    % 2%误差带
fprintf("参考超调量:\t\t%.2f%%\n",exp(-zeta_ideal*pi/sqrt(1-zeta_ideal^2))*100);
fprintf("======================================\n")
% 系统动态
dynamic_fun = @(y,u,t) -y/T + K*u/T;
%% 初始化
tspan = 0:dt:T_max;
N = length(tspan);

y = zeros(1,N);
u = zeros(1,N);
r = r_fun(tspan);
e = zeros(1,N);

y(1) = y_ini;

%% 参数辨识
theta = zeros(2,N);
theta(:,1) = [0.8;0.1];
P = zeros(2,2,N);
P(:,:,1) = 0.001*[10,1;1,10];   % 自适应速率
T_hat = zeros(1,N);
K_hat = zeros(1,N);

Kp_adapt = zeros(1,N);
Ki_adapt = zeros(1,N);
Kp_adapt(1) = Kp_norml;
Ki_adapt(1) = Ki_norml;
%% 仿真计算
if ifideal
    Ki = Ki_ideal;
    Kp = Kp_ideal;
elseif ifadapt
    Ki = Ki_adapt(1);
    Kp = Kp_adapt(1);
else
    Kp = Kp_norml;
    Ki = Ki_norml;
end
for k = 1:N-1
    if mod(tspan(k),Ts) == 0    % 进入采样控制周期
        if ifadapt && k > 1
            Kp = Kp_adapt(k-1);
            Ki = Ki_adapt(k-1);
        end
        % 获取误差
        e(k) = r(k) - y(k);
        % 控制律
        try 
            u(k) = u(k-1) + Kp*e(k) + (Ki*Ts-Kp)*e(k-1);
            % （自带抗积分饱和）
        catch
            u(k) = Kp*e(k);
        end
        % 限幅
        u(k) = max(min(u_max,u(k)),u_min);

        % 参数估计
        try
            steps = ceil(Ts/dt);
            phi = [y(k-steps);u(k-steps)];  % 应获取前一采样时刻，而非前一仿真时刻
            H = P(:,:,k-1)*phi/(1 + phi'*P(:,:,k-1)*phi);
            P(:,:,k) = (eye(2) - H*phi')*P(:,:,k-1);
            theta(:,k) = theta(:,k-1) + H * (y(k)-phi'*theta(:,k-1));
            % disp((y(k)-phi'*theta(:,k-1)))
        catch
        end
        T_hat(k) = -Ts/log(theta(1,k));
        K_hat(k) = theta(2,k)/(1-theta(1,k));
        % 根据估计结果计算控制参数
        Kp_adapt(k) = (2*zeta_ideal*omega_ideal*T_hat(k) - 1)/K_hat(k);
        Ki_adapt(k) = (omega_ideal^2*T_hat(k))/K_hat(k);
    else    % 零阶保持
        u(k) = u(k-1);
        e(k) = e(k-1);
        theta(:,k) = theta(:,k-1);
        P(:,:,k) = P(:,:,k-1);
        T_hat(k) = T_hat(k-1);
        K_hat(k) = K_hat(k-1);
        Kp_adapt(k) = Kp_adapt(k-1);
        Ki_adapt(k) = Ki_adapt(k-1);
    end

    % 更新系统动态
    y(k+1) = update_rk4(dynamic_fun,y(k),u(k),[],dt);
end
Preview(tspan,y,u,r);
%% 保存数据
% 创建results文件夹
if ~exist('results', 'dir')
    mkdir('results');
end

% 将仿真数据保存到结构体
data.t = tspan;
data.y = y;
data.u = u;
data.r = r;
data.e = e;

% 系统参数
data.T = T;
data.K = K;
data.y_ref = y_ref;

% 控制参数
if ifideal
    data.Kp = Kp_ideal.*ones(1,N);
    data.Ki = Ki_ideal.*ones(1,N);
elseif ifadapt
    data.Kp = Kp_adapt;
    data.Ki = Ki_adapt;
else
    data.Kp = Kp_norml.*ones(1,N);
    data.Ki = Ki_norml.*ones(1,N);
end
data.Ts = Ts;

% 保存到mat文件（results子文件夹）
if ifadapt
    save_filename = 'results/3_Results_AdaptivePI.mat';
elseif ifideal
    save_filename = 'results/1_Results_IdealPI.mat';
else
    save_filename = 'results/2_Results_NormalPI.mat';
end
save(save_filename, 'data');
fprintf('仿真数据已保存到: %s\n', save_filename);


