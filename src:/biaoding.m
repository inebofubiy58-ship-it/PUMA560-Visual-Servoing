close all; clear; clc;

% 1. 创建照片存储文件夹（【修复点】：强制使用物理绝对路径）
save_dir = fullfile(pwd, 'Calibration_Images');
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

% 2. 启动摄像头
disp('正在唤醒 Mac 摄像头...');
vid = videoinput('macvideo', 1);
vid.ReturnedColorSpace = 'rgb';

% 3. 创建一个纯粹的实时预览窗口
fig = figure('Name', '实时采集 (按空格键拍照)', 'Position', [100, 100, 1000, 750], 'Color', 'black');
ax = axes('Parent', fig, 'Position', [0, 0.05, 1, 0.9]); 

% 创建空画板，让底层驱动直接把实时视频流打在画板上
vidRes = vid.VideoResolution;
hImage = image(zeros(vidRes(2), vidRes(1), 3), 'Parent', ax);
axis(ax, 'image', 'off');

% 开启异步实时预览
preview(vid, hImage);

% 4. 初始化计数器
fig.UserData = 1;

% 5. 绑定键盘快捷键
set(fig, 'WindowKeyPressFcn', @(src, event) captureImage(src, vid, save_dir));

disp('=======================================');
disp('📷 【实时画面已开启】！画面绝对丝滑。');
disp(['👉 照片将强制保存在绝对路径: ', save_dir]);
disp('1. 保持图像窗口被选中（在最前面），直接按下键盘上的【空格键】。');
disp('2. 听到"咔嚓"提示（窗口会闪一下），就代表照片存好了！');
disp('拍满 15-20 张后，直接点击窗口左上角的红叉关闭即可。');
disp('=======================================');


% ====================================================
% 下面是本地动作函数 (专门负责拍照和存图)
% ====================================================
function captureImage(fig, vid, save_dir)
    % 抓取当前这一瞬间的高清画面
    frame = getsnapshot(vid);
    
    % 读取当前是第几张
    count = fig.UserData;
    
    % 拼接文件名并保存到文件夹
    img_name = fullfile(save_dir, sprintf('calib_img_%02d.png', count));
    imwrite(frame, img_name);
    
    % 在命令行打印成功日志
    disp(['✅ 咔嚓！成功保存第 ', num2str(count), ' 张']);
    
    % 把计数器加 1
    fig.UserData = count + 1;
    
    % 物理反馈：给窗口加一个白色的"闪光灯"特效
    set(fig, 'Color', 'white');
    pause(0.05);
    set(fig, 'Color', 'black');
end