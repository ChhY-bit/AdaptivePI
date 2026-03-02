function Preview(tspan,y,u,r)
warning off
% 绘图设置
set(0, 'DefaultAxesFontName', 'Times New Roman');   % 坐标轴字体
set(0, 'DefaultAxesFontSize', 12);                   % 坐标轴字号
set(0, 'DefaultTextFontName', 'Times New Roman');   % 文本字体
set(0, 'DefaultTextFontSize', 12);                  % 文本字号
% 默认解释器 - latex
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');

N = length(tspan);
T_max = tspan(end);

%% 系统输出
figure(1)
plot(tspan,y,'LineWidth',1.5);
hold on
plot(tspan,r,'k--','LineWidth',1.5)

% 找到峰值和调节时间
[peak_value, peak_idx] = max(y);
peak_time = tspan(peak_idx);

% 2%误差带调节时间（最后一次进入误差带的时刻）
target = r(end);
lower_band = 0.98 * target;
upper_band = 1.02 * target;

% 找到所有在误差带内的索引
in_band = y >= lower_band & y <= upper_band;
% 找到所有脱离误差带的索引
out_band = ~in_band;
% 调节时间：最后一次脱离误差带之后不再脱离的起始时刻
if any(in_band)
    % 找到最后一次脱离误差带的索引
    out_indices = find(out_band);
    if ~isempty(out_indices)
        last_out_idx = out_indices(end);
        % 找到这次脱离之后首次进入误差带的索引
        settle_idx = find(in_band & (1:N) > last_out_idx, 1);
        if ~isempty(settle_idx)
            settle_time = tspan(settle_idx);
        else
            % 如果最后一次脱离之后没有再进入，使用第一次进入的时间
            settle_idx = find(in_band, 1);
            settle_time = tspan(settle_idx);
        end
    else
        % 如果从未脱离误差带，使用第一次进入的时间
        settle_idx = find(in_band, 1);
        settle_time = tspan(settle_idx);
    end
else
    settle_time = NaN;
end

% 绘制2%误差带
% 添加半透明阴影区域
fill([0, T_max, T_max, 0], [lower_band, lower_band, upper_band, upper_band], ...
    'g', 'FaceAlpha', 0.15, 'EdgeColor', 'none');
% 绘制误差带边界线
plot([0, T_max], [lower_band, lower_band], 'g-', 'LineWidth', 1);
plot([0, T_max], [upper_band, upper_band], 'g-', 'LineWidth', 1);
text(T_max*0.02, upper_band*1.03, '2% Band', 'Color', [0,0.7,0]);

% 标注峰值超调量（直线标注）
if peak_value > target
    % 在峰值点和期望值之间画直线
    arrow_offset_x = 0; % 水平偏移量
    arrow_x = peak_time + arrow_offset_x;
    
    % 画连接线
    plot([arrow_x, arrow_x], [target, peak_value], 'r-', 'LineWidth', 1.5);
    
    % 标注文字
    text(arrow_x*1.05, (peak_value+target)/2, ...
        sprintf('P.Overshoot: %.1f%%', (peak_value-target)/target*100), ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', 'Color', 'r');
end

% 标注调节时间竖线
if ~isnan(settle_time) && settle_time > 0
    line([settle_time, settle_time], [0, max(y)*1.1], 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1.5);
    text(settle_time*1.05, max(y)*1.02, ...
        sprintf('S.Time: %.2f s', settle_time), ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', 'Color', 'b');
end

xlabel('Time (s)');
ylabel('System Output');
title('The Response of \emph{1st-Order} System');
legend('Actual', 'Reference','Location','best');
grid on;

%% 控制输入
figure(2)
plot(tspan(1:end-1),u(1:end-1),'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Control Input');
title('The Sytem Input from Controller');
grid on;
end