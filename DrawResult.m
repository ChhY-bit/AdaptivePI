warning off
%% 绘图设置
set(0, 'DefaultAxesFontName', 'Times New Roman');   % 坐标轴字体
set(0, 'DefaultAxesFontSize', 12);                   % 坐标轴字号
set(0, 'DefaultTextFontName', 'Times New Roman');   % 文本字体
set(0, 'DefaultTextFontSize', 12);                  % 文本字号
% 默认解释器 - latex
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');

%% 从output子文件夹读取数据
% 获取所有.mat文件
folder = 'results';
mat_files = dir([folder,'/*.mat']);
fprintf('找到 %d 个结果文件\n', length(mat_files));

% 读取数据并存储到单元格数组
data_list = cell(length(mat_files), 1);
for i = 1:length(mat_files)
    filepath = fullfile(folder, mat_files(i).name);
    load(filepath);
    data_list{i} = data;
    fprintf('已加载: %s\n', mat_files(i).name);
end
%% 结果绘制
% 样式列表：
expr_name = {'Ideal PI','Manual PI','Adaptive PI'};
expr_color= {'b',[0,0.6,0],'r'};
% 公共部分：
t = data_list{1}.t;
r = data_list{1}.r;
for i = 1:length(mat_files) + 1
    %% 系统输出
    figure(1)
    if i == length(mat_files) + 1
        plot(t,r,'k--','DisplayName','Reference','LineWidth',1.5);
        continue
    end
    y = data_list{i}.y;
    plot(t,y,'DisplayName',expr_name{i},'Color',expr_color{i},...
         'LineWidth',1.5);
    hold on
    %% 控制输入
    figure(2)
    if i == length(mat_files) + 1
        continue
    end
    u = data_list{i}.u;
    plot(t(1:end-1),u(1:end-1),'DisplayName',expr_name{i},'Color',expr_color{i},...
         'LineWidth',1.5);
    hold on
    %% 参数估计
    figure(3)
    subplot(2,1,1)
    Kp = data_list{i}.Kp;
    plot(t(1:end-1),Kp(1:end-1),'DisplayName',expr_name{i},'Color',expr_color{i},...
         'LineWidth',1.5)
    hold on

    subplot(2,1,2)
    Ki = data_list{i}.Ki;
    plot(t(1:end-1),Ki(1:end-1),'DisplayName',expr_name{i},'Color',expr_color{i},...
         'LineWidth',1.5)
    hold on
end
figure(1)
xlabel('Time $t$(s)')
ylabel('System Output $y(t)$')
title('The Response of \emph{1st-Order} System')
grid on
legend('Location','best')

figure(2)
xlabel('Time $t$(s)')
ylabel('Control Input $u(t)$')
title('The Sytem Input from Controller')
grid on
legend('Location','best')

figure(3)
subplot(2,1,1)
xlabel('Time $t$(s)')
ylabel('Parameter $K_p$')
title('The Proportional Coefficient $K_p$')
grid on
legend('Location','best')
subplot(2,1,2)
xlabel('Time $t$(s)')
ylabel('Parameter $K_i$')
title('The Integral Coefficient $K_i$')
grid on
legend('Location','best')