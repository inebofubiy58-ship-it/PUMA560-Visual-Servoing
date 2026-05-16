close all; clear; clc;

% 1. 初始化相机
vid = videoinput('macvideo', 1);
vid.ReturnedColorSpace = 'rgb';

% 【物理修复】：让相机常开，避免硬件启停导致的黑屏闪烁
triggerconfig(vid, 'manual');
start(vid); 

disp('摄像头正在预热，请等待 2 秒钟...');
pause(2); 

% 2. 创建显示窗口
fig = figure('Name', 'Task 2 基础：实时红色追踪', 'Position', [100, 100, 800, 600]);
disp('程序已启动！');

% 先抓取一帧作为初始画面
frame = getsnapshot(vid);
h_img = imshow(frame); 
hold on;

% 初始化追踪标记（绿色圈圈）
h_marker1 = plot(NaN, NaN, 'g+', 'MarkerSize', 20, 'LineWidth', 3);
h_marker2 = plot(NaN, NaN, 'go', 'MarkerSize', 20, 'LineWidth', 3);

% 【核心改动】：初始化动态坐标文本框
h_coord_text = text(20, 40, '正在寻找目标...', 'Color', 'green', ...
    'FontSize', 18, 'FontWeight', 'bold', 'BackgroundColor', 'black');

% 3. 实时追踪循环
while ishandle(fig)
    % 抓取新画面
    frame = getsnapshot(vid); 
    set(h_img, 'CData', frame); % 极速更新图像数据
    
    % --- 图像识别逻辑 ---
    hsv_img = rgb2hsv(frame);
    H = hsv_img(:,:,1); S = hsv_img(:,:,2); V = hsv_img(:,:,3); 
    
    % 红色阈值分割
    red_mask = (H < 0.05 | H > 0.95) & (S > 0.4) & (V > 0.4);
    red_mask = bwareaopen(red_mask, 200); 
    
    stats = regionprops(red_mask, 'Area', 'Centroid');
    
    if ~isempty(stats)
        % 锁定最大的红色区域
        [~, max_idx] = max([stats.Area]);
        u = stats(max_idx).Centroid(1); 
        v = stats(max_idx).Centroid(2);
        
        % 更新绿色准星位置
        set(h_marker1, 'XData', u, 'YData', v);
        set(h_marker2, 'XData', u, 'YData', v);
        
        % 【核心改动】：更新左上角的实时坐标文字
        set(h_coord_text, 'String', sprintf('X (u): %.1f   Y (v): %.1f', u, v), ...
            'Color', 'green');
    else
        % 目标丢失处理
        set(h_marker1, 'XData', NaN, 'YData', NaN);
        set(h_marker2, 'XData', NaN, 'YData', NaN);
        set(h_coord_text, 'String', '未检测到目标', 'Color', 'red');
    end
    
    drawnow; % 刷新画布
    pause(0.03); % 给系统留出响应关闭按钮的时间
end

% 4. 善后处理
try
    stop(vid); 
    delete(vid);
    disp('已安全断开摄像头。');
catch
end