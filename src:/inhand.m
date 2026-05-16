close all; clear; clc;
imaqreset; 

% ==========================================================
% 1. 系统参数与数学模型
% ==========================================================
disp('正在加载 Eye-in-Hand 系统参数...');
K = [1150.5,      0, 957.4;
          0, 1149.6, 534.2;
          0,      0,     1];
inv_K = inv(K);
cx = K(1,3); cy = K(2,3); 

Z_c = 0.6; 

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
q = [0, -pi/4, pi/2, 0, pi/4, 0]; 

% ==========================================================
% 4. UI 界面布局
% ==========================================================
fig = figure('Name', 'T2: Eye-in-Hand PBVS 数据采集系统', 'Position', [50, 100, 1200, 600]);

subplot(1,2,1);
vidRes = vid.VideoResolution;
h_img = image(zeros(vidRes(2), vidRes(1), 3, 'uint8')); 
axis image off; title('相机实时视角 (Eye-in-Hand)');
hold on;
plot(cx, cy, 'b+', 'MarkerSize', 30, 'LineWidth', 2); 
h_plot = plot(NaN, NaN, 'gO', 'MarkerSize', 15, 'LineWidth', 3);
h_text = text(20, 40, '等待目标...', 'Color', 'green', 'FontSize', 14, 'FontWeight', 'bold');

subplot(1,2,2);
p560.plot(q, 'workspace', [-2 2 -2 2 -1 2]);
title('3D 空间映射 (红色:目标, 蓝色:轨迹)');
hold on;
h_target_3d = plot3(NaN, NaN, NaN, 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% ==========================================================
% 5. 实时控制与数据拦截循环
% ==========================================================
disp('✅ Eye-in-Hand 模式启动！');
disp('💡 结束实验：直接点击图像窗口右上角的 [X]，程序将自动安全生成报表。');

while ishandle(fig)
    frame = getsnapshot(vid);
    
    % 在执行任何 UI 更新前，再次检查图窗是否存活
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
        
        % --- 核心计算 ---
        P_cam = Z_c * (inv_K * [u; v; 1]); 
        T_ee = p560.fkine(q);
        P_target_world = T_ee.T * [P_cam; 1];
        P_target = P_target_world(1:3);
        
        P_cam_desired = [0; 0; Z_c];
        e_cam = P_cam - P_cam_desired;
        
        lambda = 0.4; 
        v_base = T_ee.R * (lambda * e_cam); 
        
        J = p560.jacob0(q);
        q_dot = pinv(J(1:3, :)) * v_base; 
        
        dt = 0.1;
        q = q + (q_dot * dt)';
        
        % --- 数据提取 ---
        current_t = toc(t_start);
        time_history = [time_history; current_t];
        q_history = [q_history; q];
        error_history = [error_history; norm([u-cx, v-cy])]; 
        P_ee_pos = T_ee.t;
        P_ee_history = [P_ee_history; P_ee_pos'];
        P_target_history = [P_target_history; P_target'];
        
        % --- 【修复区：安全渲染机制】 ---
        try
            set(h_plot, 'XData', u, 'YData', v);
            p560.animate(q);
            set(h_target_3d, 'XData', P_target(1), 'YData', P_target(2), 'ZData', P_target(3));
            plot3(P_ee_pos(1), P_ee_pos(2), P_ee_pos(3), 'b.', 'MarkerSize', 5);
            set(h_text, 'String', sprintf('偏差: du=%.1f, dv=%.1f', u-cx, v-cy));
            drawnow;
        catch
            % 捕获到界面已销毁，安静退出循环
            break;
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
% 6. 自动生成实验报表
% ==========================================================
try stop(vid); delete(vid); catch; end
disp('正在整理实验数据并生成图表...');

if isempty(time_history)
    disp('警告：没有记录到有效数据。');
    return;
end

figure('Name', 'Report: In-Hand Pixel Error', 'Position', [100, 100, 600, 400]);
plot(time_history, error_history, 'm-', 'LineWidth', 2);
title('图像平面像素误差收敛曲线 (T2-b)', 'FontSize', 12);
xlabel('时间 (s)'); ylabel('像素偏差模长 (pixels)'); grid on;

figure('Name', 'Report: In-Hand Joint States', 'Position', [750, 100, 600, 400]);
plot(time_history, q_history, 'LineWidth', 1.5);
title('眼在手上模式：关节角度变化 (T2-c)', 'FontSize', 12);
xlabel('时间 (s)'); ylabel('角度 (rad)');
legend('q1','q2','q3','q4','q5','q6', 'Location', 'bestoutside'); grid on;

save('T2_InHand_Data.mat', 'time_history', 'q_history', 'error_history', 'P_ee_history', 'P_target_history', 'K');
disp('✅ 眼在手上实验数据已保存至 T2_InHand_Data.mat。');