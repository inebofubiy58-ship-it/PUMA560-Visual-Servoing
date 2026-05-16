close all; clear; clc;
imaqreset; % 强制重置硬件占用，防止报错

% ==========================================================
% 1. 系统参数与数学模型
% ==========================================================
disp('正在加载 Overhead 系统参数...');
K = [1150.5,      0, 957.4;
          0, 1149.6, 534.2;
          0,      0,     1];
inv_K = inv(K);

Z_c = 2.0; % 深度假设：相机在 2 米高俯视
T_c_to_b = SE3(0, 0, Z_c) * SE3.Rz(-pi/2) * SE3.Rx(pi);

% ==========================================================
% 2. 数据记录器初始化 
% ==========================================================
time_history = [];    
q_history = [];       
error_history = [];   
P_ee_history = [];    
P_target_history = [];
t_start = tic;        

% ==========================================================
% 3. 硬件与机器人初始化
% ==========================================================
disp('正在启动摄像头与 PUMA 560 模型...');
vid = videoinput('macvideo', 1);
vid.ReturnedColorSpace = 'rgb';
triggerconfig(vid, 'manual');
start(vid);
pause(2); 

mdl_puma560;
q = qn; % 初始姿态

% ==========================================================
% 4. UI 界面布局
% ==========================================================
fig = figure('Name', 'T2: Overhead PBVS 数据采集系统', 'Position', [50, 100, 1200, 600]);

subplot(1,2,1);
vidRes = vid.VideoResolution;
h_img = image(zeros(vidRes(2), vidRes(1), 3, 'uint8')); 
axis image off; title('2D 图像追踪 (关闭此窗口可自动导出报告数据)');
hold on;
h_plot = plot(NaN, NaN, 'gO', 'MarkerSize', 15, 'LineWidth', 3);
h_text = text(20, 40, '寻找红色目标...', 'Color', 'green', 'FontSize', 14, 'FontWeight', 'bold');

subplot(1,2,2);
p560.plot(q, 'workspace', [-2 2 -2 2 -1 2]);
title('3D 空间轨迹 (红色:目标, 蓝色:机械臂)');
hold on;
h_target_3d = plot3(NaN, NaN, NaN, 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% ==========================================================
% 5. 实时控制与数据拦截循环
% ==========================================================
disp('✅ 系统已启动！请移动红色物体进行追踪。');
disp('💡 提示：测试完成后，直接点击图像窗口右上角的 [X] 退出，程序将自动生成报表。');

while ishandle(fig)
    frame = getsnapshot(vid);
    
    % 【防线 1】：在执行任何计算前，确认图窗还在
    if ~ishandle(fig)
        break;
    end
    
    set(h_img, 'CData', frame); 
    hsv = rgb2hsv(frame);
    red_mask = (hsv(:,:,1) < 0.05 | hsv(:,:,1) > 0.95) & (hsv(:,:,2) > 0.4) & (hsv(:,:,3) > 0.4);
    red_mask = bwareaopen(red_mask, 200); 
    stats = regionprops(red_mask, 'Area', 'Centroid');
    
    if ~isempty(stats)
        [~, max_idx] = max([stats.Area]);
        u = stats(max_idx).Centroid(1); v = stats(max_idx).Centroid(2);
        
        P_cam = Z_c * (inv_K * [u; v; 1]); 
        P_base_homo = T_c_to_b.T * [P_cam; 1]; 
        P_target = P_base_homo(1:3); 
        
        T_ee = p560.fkine(q); P_ee = T_ee.t;        
        error_pos = P_target - P_ee;
        V_base = 0.3 * error_pos; 
        J = p560.jacob0(q);
        q_dot = pinv(J(1:3, :)) * V_base; 
        q = q + (q_dot * 0.1)';
        
        current_t = toc(t_start);
        time_history = [time_history; current_t];
        q_history = [q_history; q];
        error_history = [error_history; norm(error_pos)];
        P_ee_history = [P_ee_history; P_ee'];
        P_target_history = [P_target_history; P_target'];
        
        % 【防线 2】：安全渲染机制，捕获销毁瞬间的报错
        try
            set(h_plot, 'XData', u, 'YData', v);
            set(h_target_3d, 'XData', P_target(1), 'YData', P_target(2), 'ZData', P_target(3));
            p560.animate(q);
            plot3(P_ee(1), P_ee(2), P_ee(3), 'b.', 'MarkerSize', 5);
            set(h_text, 'String', sprintf('目标 X:%.2f Y:%.2f', P_target(1), P_target(2)));
            drawnow;
        catch
            break; % 捕获到界面已销毁，安静退出循环
        end
    else
        try
            set(h_plot, 'XData', NaN, 'YData', NaN);
            set(h_text, 'String', '目标丢失', 'Color', 'red');
            drawnow;
        catch
            break;
        end
    end
end

% ==========================================================
% 6. 自动生成实验报表与数据保存
% ==========================================================
try stop(vid); delete(vid); catch; end
disp('正在整理实验数据并生成图表...');

% 检查是否记录到了有效数据
if isempty(time_history)
    disp('警告：没有记录到有效数据。');
    return;
end

figure('Name', 'Report: Error Convergence', 'Position', [100, 100, 600, 400]);
plot(time_history, error_history, 'r-', 'LineWidth', 2);
title('视觉伺服误差收敛曲线 (T2-b)', 'FontSize', 12);
xlabel('时间 (s)'); ylabel('位置误差模长 (m)'); grid on;

figure('Name', 'Report: Joint Trajectories', 'Position', [750, 100, 600, 400]);
plot(time_history, q_history, 'LineWidth', 1.5);
title('关节角度变化曲线 (T2-c)', 'FontSize', 12);
xlabel('时间 (s)'); ylabel('角度 (rad)');
legend('q1','q2','q3','q4','q5','q6', 'Location', 'bestoutside'); grid on;

if exist('red_mask','var')
    figure('Name', 'Report: Object Detection Mask');
    imshow(red_mask); title('二值化掩膜图 (T2-a)');
end

save('T2_FixedCamera_Data.mat', 'time_history', 'q_history', 'error_history', 'P_ee_history', 'P_target_history', 'K');
disp('✅ 实验数据已成功保存至 T2_FixedCamera_Data.mat。');