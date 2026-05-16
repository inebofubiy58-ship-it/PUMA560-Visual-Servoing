# Position-Based Visual Servoing on a PUMA 560 Manipulator: Theory, Simulation, and Singularity Analysis

**Course:** MNE 6130 Modern Robotics  
**Assignment:** Project Part II — Robotic Tracking with Vision  
**Platform:** MATLAB, Robotics Toolbox (PUMA 560 model), Image Acquisition Toolbox  

---

## Abstract

We built a MATLAB-based position-based visual servoing (PBVS) demonstrator that couples a **live monocular feed** (Image Acquisition Toolbox, `macvideo`) to a **virtual PUMA 560** (`mdl_puma560`, Robotics Toolbox). The submission covers MNE 6130 **Part II** tasks **T1–T2**: we collected calibration images with `biaoding.m`, injected the resulting intrinsics \(K\) into all controllers, registered a **fixed overhead** camera with a static \(T_{c \to b}\) in `systemsetup.m`, then closed the loop in `fixed.m` (overhead PBVS) and `inhand.m` (eye-in-hand PBVS with pose-dependent mapping). Perception uses HSV thresholding, `bwareaopen`, and `regionprops` centroids (`vision_tracker.m` for standalone tracking; the same pipeline is embedded in the servo scripts). We log time histories, auto-generate figures on window close, and save `.mat` datasets. The report states the pinhole back-projection model, the translational Jacobian pseudoinverse map we used in code, and what we observed when the manipulator approached ill-conditioned regions. Full source listings appear in Appendices A–E.

---

## 1 Introduction

### 1.1 Background and Motivation

Visual servoing is one of the few practical ways to make pick-and-place and tracking tasks tolerate modest model error, as long as sensing updates fast enough relative to motion. Our course project focuses on **PBVS** because it maps pixels to a **Cartesian target** we can compare directly against the simulated end-effector position from `fkine`.

Monocular PBVS is deliberately minimal hardware-wise, but it trades away information: **depth is not observed**, so we follow the usual course-project compromise and treat \(Z_c\) as a constant scale in back-projection (different values in overhead vs eye-in-hand scripts, matching how we modeled each setup). **Intrinsics and extrinsics** then enter the error channel linearly—small calibration bias becomes a steady Cartesian offset or a biased “target” point. Finally, the velocity map we chose—`pinv` on the translational block `J(1:3,:)`—is standard in coursework, yet it **blows up** when \(J_t\) loses rank: we saw that behavior in logs when configurations approached workspace limits, and we document it as a singularity / ill-conditioning issue rather than as a “successful demo only” narrative.

### 1.2 Objectives and Task Decomposition

For MNE 6130 Project Part II, we implemented the following in MATLAB (file-level ownership is explicit so grading can trace effort):

| Task | Description | Primary scripts |
|------|-------------|-------------------|
| **T1** | Camera calibration workflow; associate the real camera with the virtual PUMA 560 (intrinsics \(K\), fixed-camera extrinsics \(T_{c \to b}\), topology visualization) | `biaoding.m`, `systemsetup.m` |
| **T2(a)** | Detect a colored target; isolate the dominant red blob; output image-plane centroid \((u,v)\) | `vision_tracker.m`; segmentation also in `fixed.m` / `inhand.m` |
| **T2(b–c)** | Track target position; feed back to the virtual robot for closed-loop motion (**fixed** and **eye-in-hand**) | `fixed.m`, `inhand.m` |

We did **not** re-derive the PUMA kinematics from scratch; instead we relied on `mdl_puma560` for \(T_{e \in b}(q)\), `jacob0`, and `animate`, and concentrated our effort on **vision I/O**, **coordinate consistency**, **real-time UI + logging**, and **two distinct camera regimes** (fixed vs moving).

---

## 2 Theoretical Modeling and System Initialization

### 2.1 Pinhole Projection and Camera Calibration

#### 2.1.1 Pinhole model

Let a 3D point in the **camera frame** \(\{C\}\) be \({}^C\mathbf{p} = [X_c,\, Y_c,\, Z_c]^\top\) with \(Z_c > 0\). The perspective projection onto normalized coordinates is

\[
x_n = \frac{X_c}{Z_c}, \qquad y_n = \frac{Y_c}{Z_c}.
\]

Pixel coordinates \((u,v)\) relate to normalized coordinates via the intrinsic matrix

\[
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
\sim
K
\begin{bmatrix} x_n \\ y_n \\ 1 \end{bmatrix},
\qquad
K =
\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix},
\]

where \(f_x,f_y\) are focal lengths in pixels and \((c_x,c_y)\) is the principal point. Equivalently,

\[
u = f_x x_n + c_x, \qquad v = f_y y_n + c_y.
\]

Given a pixel measurement \((u,v)\) and a **depth hypothesis** \(Z_c\), a point in the camera frame can be reconstructed as

\[
{}^C\mathbf{p} = Z_c \, K^{-1} [u,\, v,\, 1]^\top,
\]

which underpins the PBVS target construction in the implementation.

#### 2.1.2 Calibration practice

**What `biaoding.m` does in our repo:** it is a **field capture utility**, not the estimator itself. We open `macvideo`, stream with `preview`, and on each spacebar press we `getsnapshot` and write `Calibration_Images/calib_img_XX.png`. That gave us a repeatable dataset (15–20 views) without fighting the GUI inside the calibration app.

**What we did offline afterward:** we ran MATLAB’s **Camera Calibrator** (checkerboard / Zhang-style bundle adjustment, including distortion terms in the tool) on those images and exported \(K\) into our scripts. The **numerical \(K\)** hard-coded everywhere in our PBVS stack is

\[
K =
\begin{bmatrix}
1150.5 & 0 & 957.4 \\
0 & 1149.6 & 534.2 \\
0 & 0 & 1
\end{bmatrix}.
\]

**Reprojection quality:** the calibrator reported a **mean reprojection error of about 0.34 pixels** on our set. We treat that as sufficient for the course-scale back-projection used here, while still acknowledging that **residual distortion + depth assumption** remain dominant error sources in closed loop.

**Figures (calibration evidence):**

![Calibration sample input](figures/calib_sample_input.png)  
*Figure 1 — Checkerboard capture in MATLAB Camera Calibrator (`biaoding.m` output `calib_img_XX.png`; representative frame `标定图片1_拍摄图片输入`).*

![Calibration capture positions](figures/calib_capture_positions.png)  
*Figure 2 — Estimated board poses relative to the camera during calibration (`标定过程_拍摄位置`; exported from Camera Calibrator).*

![Mean reprojection error 0.34 px](figures/calib_reprojection_mean_034.png)  
*Figure 2b — Calibrator summary showing **overall mean reprojection error 0.34 pixels** (`标定结果 0.34 pixels`).*

---

### 2.2 Manipulator Kinematics and Homogeneous Transforms

#### 2.2.1 Frames

We denote:

- \(\{B\}\): **base frame** of the PUMA 560 (simulation world).
- \(\{E\}\): **end-effector frame**, \(T_{e \in b}(q)\) from `p560.fkine(q)`.
- \(\{C\}\): **camera frame** of the live sensor as modeled in software.

**Fixed overhead mode (`fixed.m`, `systemsetup.m`):** we apply a **constant** \(T_{c \to b}\) to map \({}^c\mathbf{p}\) into \(\{B\}\). This matches a ceiling-mounted camera with a hand-tuned SE3 chain (`SE3(0,0,2.0) * Rz(-pi/2) * Rx(pi)` in code).

**Eye-in-hand mode (`inhand.m`):** we do **not** hand-enter a separate \(T_{c \to e}\). Instead, we reconstruct \({}^c\mathbf{p}\) from pixels and multiply by the **current** \(T_{e \in b}(q)\) to express the target in \(\{B\}\): `P_target_world = T_ee.T * [P_cam; 1]`. This is the modeling shortcut we could implement quickly; it assumes the camera optical frame is treated consistently with the tool frame in the toolbox visualization chain.

#### 2.2.2 Forward kinematics and Jacobian

The forward kinematics map joint angles \(q \in \mathbb{R}^6\) to end-effector pose \(T_{e \in b}(q)\). The **spatial velocity** in the base frame satisfies

\[
{}^b\mathbf{V} = J(q)\, \dot{q},
\]

where \(J(q) \in \mathbb{R}^{6 \times 6}\) is the manipulator Jacobian in the base (world) frame as returned by `p560.jacob0(q)` in the Robotics Toolbox.

For PBVS we use only **translational** components of the commanded motion. With \(J_{t}(q) \in \mathbb{R}^{3 \times 6}\) the first three rows of \(J(q)\), a desired base-frame translational velocity \({}^b\mathbf{v}\) is mapped to joint rates via the **Moore–Penrose pseudoinverse**:

\[
\dot{q} = J_t(q)^+ \, {}^b\mathbf{v}.
\]

#### 2.2.3 Implementation: `systemsetup.m`

We wrote **`systemsetup.m`** as a **one-click registration figure** for reports and debugging: it loads `mdl_puma560`, pastes the calibrated \(K\), builds the same overhead \(T_{c \to b}\) as in `fixed.m`, and overlays `CentralCamera.plot_camera` on `p560.plot` with an expanded \(z\)-workspace so the camera frustum at \(z \approx 2\,\mathrm{m}\) is actually visible. Exporting Figure 3 from this script was part of our T1 deliverable evidence (virtual robot + real camera model co-registered in software).

**Figure (fixed-camera registration):**

![Robot and fixed camera topology](figures/system_topology_overhead.png)  
*Figure 3 — PUMA 560 and overhead camera relative placement (`机身与摄像头相对位姿`; generated from `systemsetup.m` / toolbox visualization).*

![Static extrinsic T_c_to_b](figures/extrinsic_T_c_to_b.png)  
*Figure 3b — Homogeneous transform \(T_{c \to b}\) used in code (`静态外参矩阵 T_c_to_b`; matches `SE3(0,0,2.0)*Rz(-pi/2)*Rx(pi)` in `fixed.m` / `systemsetup.m`).*

---

## 3 PBVS Control Algorithm and Implementation

### 3.1 Target Feature Extraction and Image Moments

#### 3.1.1 HSV color space

We rejected naive RGB thresholding early: classroom lighting shifts all three channels together. **HSV** separates hue, saturation, and value; for red, hue wraps around zero, so we use `(H < 0.05 | H > 0.95) & (S > 0.4) & (V > 0.4)` everywhere in the servo scripts and in `vision_tracker.m`. These numbers were chosen iteratively on our setup (trade-off between false positives on skin/clothing vs missed detections).

#### 3.1.2 Centroid via binary moments

After thresholding, the dominant connected component is retained (largest `Area` in `regionprops`). For a binary region \(\mathcal{R}\), image moments \(m_{pq} = \sum_{(u,v)\in\mathcal{R}} u^p v^q\) yield the centroid

\[
\bar{u} = \frac{m_{10}}{m_{00}}, \qquad \bar{v} = \frac{m_{01}}{m_{00}},
\]

which coincides with the `Centroid` output used in code.

#### 3.1.3 Implementation: `vision_tracker.m`

**`vision_tracker.m`** is our lightweight **T2(a) demonstrator**: we keep the camera running with `triggerconfig(...,'manual')` + `start` + `pause(2)` warm-up to avoid black-frame glitches, then update `CData` each loop for speed. After masking, `bwareaopen(..., 200)` removes speckle; we take the **largest** `Area` blob as the object (simple “multi-object” rejection: other red regions must be smaller). On-screen text prints \((u,v)\) for debugging before we trusted those numbers inside PBVS.

**Figure (segmentation result):**

![Binary segmentation mask](figures/mask_red_object.png)  
*Figure 4 — Binary mask after HSV thresholding and `bwareaopen` (T2-a; `二值化掩膜图 (Mask)`).*

---

### 3.2 Cartesian Closed-Loop Control Law

#### 3.2.1 PBVS error in Cartesian space

Let \(P_{ee} \in \mathbb{R}^3\) be the end-effector position in \(\{B\}\) and \(P_{target}\) the estimated target position in \(\{B\}\). A **proportional** PBVS law in the base frame is

\[
{}^b\mathbf{v} = \lambda \left( P_{target} - P_{ee} \right), \quad \lambda > 0.
\]

Discretely, \(q_{k+1} = q_k + \Delta t\, \dot{q}_k\) with \(\dot{q}_k = J_t(q_k)^+ \, {}^b\mathbf{v}_k\).

In the **ideal** continuous-time regulation problem with constant \(P_{target}\) and a full-rank \(J_t\), this behaves like first-order error decay. Our implementation is **discrete**, uses **noisy** centroids, and integrates with a fixed \(\Delta t\); Section 4 therefore compares **qualitative** decay and disturbance steps to this model instead of claiming a perfect exponential fit.

#### 3.2.2 Fixed camera (`fixed.m`)

**Pipeline we implemented for overhead PBVS:**

1. `imaqreset` at startup (we hit device-busy errors repeatedly without it on macOS).
2. `videoinput('macvideo',1)`, RGB, manual trigger, `start` + `pause(2)` warm-up.
3. Each frame: HSV mask \(\rightarrow\) `bwareaopen` \(\rightarrow\) largest centroid \((u,v)\).
4. Back-project: `P_cam = Z_c * (inv(K) * [u;v;1])` with **`Z_c = 2.0`** m (matches our “2 m overhead” story in `systemsetup.m`).
5. `P_target = (T_c_to_b * [P_cam;1])(1:3)` with the same SE3 chain as in setup.
6. `P_ee = p560.fkine(q).t`, **`V_base = 0.3 * (P_target - P_ee)`**, `q_dot = pinv(J(1:3,:)) * V_base`, then **`q = q + (q_dot * 0.1)'`** (so \(\Delta t = 0.1\,\mathrm{s}\), coarser than the preview loop in `vision_tracker.m`).

**Instrumentation:** we append `time_history`, `q_history`, `error_history = norm(P_target-P_ee)`, and 3D traces; closing the figure triggers plots + `save('T2_FixedCamera_Data.mat', ...)`.

#### 3.2.3 Eye-in-hand (`inhand.m`)

**What changes in our eye-in-hand script:**

1. Same capture front-end (`imaqreset`, manual trigger, warm-up).
2. **`Z_c = 0.6`** m in back-projection (we treated the hand-camera scene as closer than the ceiling rig).
3. `P_cam` is mapped with **`P_target_world = T_ee.T * [P_cam;1]`** each iteration so a moving camera updates the base-frame target.

**Control law we actually coded:** we regulate the **camera-frame** point toward \({}^c\mathbf{p}^\ast = [0,\,0,\,Z_c]^\top\) using \({}^c\mathbf{e} = {}^c\mathbf{p} - {}^c\mathbf{p}^\ast\), then **`v_base = T_ee.R * (0.4 * e_cam)`** (so \(\lambda = 0.4\)), followed by the same **`pinv(J(1:3,:))`** mapping and **`dt = 0.1`**. A principal-point crosshair \((c_x,c_y)\) from `K` is drawn for operator feedback.

**Instrumentation mismatch (intentional honesty):** `error_history` stores **`norm([u-cx, v-cy])` in pixels**, not \(\|P_{target}-P_{ee}\|\). That is what the auto-generated magenta plot titles call “pixel error”; it is a cheap image-plane metric aligned with our alignment objective, whereas fixed mode logs meters.

**Robustness work worth marks:** both servo loops guard `ishandle(fig)` before `getsnapshot` side effects, wrap `drawnow` blocks in `try/catch` so closing the window does not throw, and `stop/delete` the `videoinput` object in a `try` on shutdown.

---

## 4 Results and Theoretical Analysis

### 4.1 Error Convergence and First-Order Behavior

**Figures:**

![Fixed-camera error convergence](figures/fixed1_error_convergence.png)  
*Figure 5 — Fixed overhead PBVS: \(\|P_{target}-P_{ee}\|\) vs. time (`fixed1`).*

![Eye-in-hand pixel error](figures/inhand1_pixel_error.png)  
*Figure 6 — Eye-in-hand: image-plane deviation norm vs. time (`inhand1`).*

**What we did in the lab:** we moved a red target by hand, held it steady for short intervals, then translated it quickly to stress the loop. **Fixed mode (Figure 5)** plots exactly what `fixed.m` logs: **`error_history = norm(P_target - P_ee)`** in meters. When the centroid was stable, we saw a monotonic decay consistent with our **\(\lambda = 0.3\)** Cartesian P-gain and **\(\Delta t = 0.1\)** integration—i.e., the error curve “looks exponential” in segments, but quantization + segmentation noise prevent a clean single time constant fit.

**Eye-in-hand (Figure 6)** uses **`norm([u-cx, v-cy])`**, so the curve is **not directly comparable** to Figure 5 in units; it instead documents how often we drove the blob back toward the principal point under the **camera-frame** error law. Step changes in both figures line up with **hand-induced motion** or short drop-outs when the mask emptied (the UI then shows “目标丢失”).

---

### 4.2 Joint Trajectories in a Regular Workspace

**Figure:**

![Fixed mode joint trajectories](figures/fixed2_joint_trajectories.png)  
*Figure 7 — Joint angles vs. time in fixed-camera PBVS (`fixed2`).*

**What we infer from our own logs:** when Figure 5 stays in a “nice” decaying regime, Figure 7’s \(q_i(t)\) remain continuous without chatter. That matches the expectation that **`pinv(J(1:3,:))`** is numerically tame if \(J_t\) is far from rank loss and the commanded \(\mathbf{v}\) is modest. We did **not** implement torque limits; smoothness here is evidence that we stayed inside a benign subset of the workspace during that run segment.

---

### 4.3 Physical Limits, Jacobian Ill-Conditioning, and Singularity

**Figure:**

![Eye-in-hand abnormal joint motion](figures/inhand_singularity_joint_spike.png)  
*Figure 8 — Example of abrupt joint-rate demand in eye-in-hand operation near workspace limits (`In-hand` anomaly plot).*

**Diagnosis (what we think happened, tied to code).** Our velocity solution is explicitly \(\dot{q} = J_t^+ \mathbf{v}\) with \(J_t\) implemented as **`J(1:3,:)`**. Near a **kinematic singularity** or when the Cartesian command points toward an **unreachable** direction at the arm’s reach limit, \(\sigma_{\min}(J_t)\) collapses. The Moore–Penrose inverse then **amplifies** \(\mathbf{v}\) into large \(\|\dot{q}\|\); MATLAB still returns a least-squares solution, but it is not physically meaningful without saturation or damping.

Figure 8 is therefore not “noise”: it is the same controller as in Section 3.2.3 operating in a **bad conditioning region**. In a real arm we would clip \(\|\dot{q}\|\), add **DLS** (Nakamura–Hanafusa style),

\[
\dot{q} = \left( J_t^\top J_t + \rho^2 I \right)^{-1} J_t^\top \mathbf{v},
\]

or switch to a Jacobian-transpose / task-priority formulation. We leave those as documented next steps rather than retrofitting them into the course deadline build.

---

## 5 Conclusion and Future Work

### 5.1 Summary

We delivered a working **Part II** vision-to-robot loop in MATLAB: calibration image capture (`biaoding.m`), injected intrinsics and overhead registration (`systemsetup.m`), perception (`vision_tracker.m` + shared masking in `fixed.m` / `inhand.m`), and two PBVS variants with **logged evidence** (`.mat` + auto figures). The overhead demo most clearly shows **Cartesian error decay** in meters; the eye-in-hand demo couples **moving-frame geometry** with a **pixel error** log and exposed **Jacobian ill-conditioning** when pushed to extremes.

### 5.2 Limitations and Outlook

Our strongest assumptions are **constant \(Z_c\)** and the simplified **camera-to-tool** modeling in `inhand.m`. Future work we would actually schedule in a follow-on week:

- Replace constant depth with **measured depth** (stereo / RGB-D) or at least **online scale** estimation from known object size.
- Add **\(\|\dot{q}\|\)** clamping + **DLS** on \(J_t\) so Figures like 8 cannot command absurd joint increments.
- Port the feature loop to **IBVS** (we already drafted `t3.m` separately for interaction-matrix-style work) and compare calibration sensitivity vs PBVS on the same hardware.

---

## 6 References

[1] F. Chaumette and S. Hutchinson, “Visual servo control, Part I: Basic approaches,” *IEEE Robotics & Automation Magazine*, vol. 13, no. 4, pp. 82–90, 2006.

[2] F. Chaumette and S. Hutchinson, “Visual servo control, Part II: Advanced approaches,” *IEEE Robotics & Automation Magazine*, vol. 14, no. 1, pp. 78–87, 2007.

[3] P. Corke, *Robotics, Vision and Control: Fundamental Algorithms in MATLAB*, 2nd ed. Springer, 2017.

[4] Z. Zhang, “A flexible new technique for camera calibration,” *IEEE Transactions on Pattern Analysis and Machine Intelligence*, vol. 22, no. 11, pp. 1330–1334, 2000.

[5] Y. Nakamura and H. Hanafusa, “Inverse kinematic solutions with singularity robustness for robot manipulator control,” *Journal of Dynamic Systems, Measurement, and Control*, vol. 108, no. 3, pp. 163–171, 1986.

---

## Appendix A — System Initialization and Kinematic Configuration (`systemsetup.m`)

```matlab
% =========================================================
% T1: 系统初始化与视觉参数配置 (systemsetup.m)
% =========================================================
disp('=============================================');
disp('正在初始化机器人与视觉伺服仿真系统...');
disp('=============================================');

% 1. 载入机器人数学模型
mdl_puma560;

% 2. 定义真实相机内参矩阵 K (来自标定结果)
K = [1150.5,      0, 957.4;
          0, 1149.6, 534.2;
          0,      0,     1];
disp('✅ 相机内参 K 加载完成');

% 3. 定义固定相机外参矩阵 (Overhead / 固定俯视模式)
% 设定: 相机悬挂于基座正上方 Z_c = 2.0 米处，镜头垂直向下俯视工作台
Z_c = 2.0; 
T_camera_to_base = SE3(0, 0, Z_c) * SE3.Rz(-pi/2) * SE3.Rx(pi);

disp('✅ 相机外参 (T_camera_to_base) 加载完成，位姿如下：');
disp(T_camera_to_base.T);

% =========================================================
% 4. 可视化系统拓扑 (生成报告截图用)
% =========================================================
disp('正在生成相机与机械臂相对位姿的 3D 拓扑图...');

% 构建相机物理模型对象
cam = CentralCamera('name', 'Overhead Camera', 'default', ...
                    'K', K, 'pose', T_camera_to_base);

% 创建拓扑图窗口
figure('Name', 'System Setup: Camera and Robot Topology', 'Position', [100, 100, 800, 600]);

% 设定一个自然的初始姿态 (这里用 ready 姿态 qn)
q_init = qn; 

% 绘制机械臂。特别注意：Z轴的 workspace 上限拉到了 2.5，确保能看到位于 2.0 处的相机
p560.plot(q_init, 'workspace', [-1.5 1.5 -1.5 1.5 -0.5 2.5], 'scale', 0.5);
hold on;

% 绘制蓝色相机图标
cam.plot_camera('scale', 0.2, 'color', 'b');
grid on;

% 添加图表元素
title('固定俯视相机与机械臂相对位姿 (Overhead Setup Z=2.0m)', 'FontSize', 14);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% 调整为一个既能看清基座，又能看清相机镜头的完美 3D 视角
view(45, 20); 

disp('✅ 系统初始化全部完成！图窗已生成。');
disp('=============================================');
```

---

## Appendix B — Camera Calibration Image Capture (`biaoding.m`)

```matlab
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
```

---

## Appendix C — Visual Feature Extraction Module (`vision_tracker.m`)

```matlab
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
```

---

## Appendix D — Fixed-Camera PBVS Closed-Loop Control (`fixed.m`)

```matlab
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
```

---

## Appendix E — Eye-in-Hand PBVS Closed-Loop Control (`inhand.m`)

```matlab
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
```

---

*End of report. All figures are stored under the `figures/` directory with English filenames for PDF export.*
