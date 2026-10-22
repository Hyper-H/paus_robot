# Whole Thyroid Gland Segmentation

独立 ROS2 Humble 包。只做完整甲状腺腺体分割，不做结节分割，不读取 TN3K
掩膜或 OpenCAS DICOM，不导入或启动机器人运动控制。

**当前交付是通过消息链路测试的接入实现，不是已经部署成功的训练模型。**
2026-10-08，训练服务器 SSH 的已有凭据被拒绝，未取得源码和 checkpoint；
本机 `nvidia-smi` 不能连接驱动；指定 Conda 环境缺少 PyTorch，HTTPS 安装预检
又遇到证书校验失败。没有禁用证书验证、安装驱动、索要凭据或以其他模型代替。
真实模型启动、训练预处理逐像素对齐、CUDA warm-up 和真实性能验收仍未完成。

## 已确认的输入

- 主仓库 `/mnt/data/projects/paus_robot/repo`，基线为当前 `dev` 分支。
- 本实现工作区 `/mnt/data/projects/paus_robot/worktrees/thyroid-gland-segmentation`，
  分支 `codex/thyroid-gland-segmentation`。只新增本包。
- ROS2 Humble；`paus_robot` Conda Python 3.10.20、NumPy 1.26.4、
  OpenCV 4.11.0；`rclpy`、`cv_bridge`、`rosbag2_py` 可导入。
- `ultrasound-online-stack` worktree 的接收器发布
  `/ultrasound/us/image_raw` (`sensor_msgs/msg/Image`)：
  BEST_EFFORT、VOLATILE、KEEP_LAST depth=1；当前无在线发布者。
- 现有 bag 保存 `/ultrasound/us/frame`
  (`paus_interfaces/msg/UltrasoundFrame`)，图像位于 `.image`。
  `dev` 尚无该消息；可使用现有已构建超声分支作为可选 underlay，
  **不需要合并超声分支或修改消息定义**。不同时启动在线接收器和 bag 转发器。
- `us_real_20260924_connector1`：2852 张 `179x1129 mono8`，首末图像间
  47.1637 FPS。bag 总时长含无图像时段，不能直接当图像区间 FPS。
- `us_real_20260924_interactive_scan`：4805 张 `mono8`，宽 179，
  高 756/880/1005/1129/1255/1379，首末图像间 44.4461 FPS。
- frame_id 为 `ultrasound_image`。元数据 intensity_mapping 为
  `displaydata_direct_gray_fullrange`；前几帧最大像素仅 21--32。
  互动 bag 全量像素范围为 0--164，不能把首帧范围推广到整段数据。
  **不要因此自动拉伸直方图或每帧归一化**。强度分布、极窄长宽比、
  图像方向、探头、设备和训练集的差异需要另外验证。
- 源协议支持 mono16，但当前 bag 没有 mono16，未发现已证明有效位深的驱动约定。
  不根据容器类型推断 12/14/16 位。

## 构建与运行环境

每个终端按 AGENTS.md 顺序初始化。首次构建还没有本 worktree 的 install，
因此先用已有规范仓库的 install；构建后使用本工作区的 overlay：

```bash
cd /mnt/data/projects/paus_robot/worktrees/thyroid-gland-segmentation
source /opt/ros/humble/setup.bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate paus_robot
source /mnt/data/projects/paus_robot/repo/install/local_setup.bash
which python3
python3 -c 'import sys; print(sys.executable)'

# 追加发行版 colcon/lark，同时保留当前 Python 完整的包搜索优先级。
# 直接把 dist-packages 加到 PYTHONPATH 前面会意外切换 NumPy/OpenCV。
export PYTHONPATH="$(python3 -c 'import sys; print(":".join(p for p in sys.path if p) + ":/usr/lib/python3/dist-packages")')"
# 不直接运行绑定 /usr/bin/python3 的 colcon 脚本。
python3 /usr/bin/colcon build --packages-select paus_thyroid_segmentation
source install/local_setup.bash
python3 /opt/ros/humble/bin/ros2 pkg executables paus_thyroid_segmentation
```

此环境的 setuptools 与 Humble colcon 的 `--symlink-install` 不兼容
（`option --editable not recognized`）；本次没有降级共享环境，使用普通构建。
修改源码后重新 build 再 source。实际使用时 Python 搜索路径也不要混入其他
Conda 环境的 Python 3.9/3.11 包。

生产还需要在 **同一 `paus_robot` 环境**安装兼容的 PyTorch、
训练模型使用的 torchvision 等依赖。版本应根据取得的训练环境锁定，
CUDA wheel 必须匹配可工作的本机驱动；不要全局升级 NumPy/cv_bridge。
本包不在 `setup.py` 自动下载 PyTorch、预训练权重或模型。
`torch.load(..., weights_only=True)` 必须受当前版本支持，失败时不回退到不安全 pickle。

GPU 使用前先运行：

```bash
nvidia-smi
python3 -c 'import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available())'
```

节点自身还会在任何 CUDA 初始化前运行 nvidia-smi：查询所有设备的利用率、
已用显存和计算进程。任一设备利用率 >20%、显存占用 >80% 或存在计算进程时拒绝
GPU 启动。策略保守，避免 CUDA_VISIBLE_DEVICES 索引重排造成误判；不终止其他任务。
`cpu_fallback:=true` 才会明确告警后回退 CPU；默认失败退出。模型加载/推理错误不触发
静默 CPU 重试。显式 `device:=cpu` 不需要 GPU。

## 取得本地模型

运行时只读本地模型，不连接训练服务器。只使用已有 SSH 凭据。
下面是访问恢复后的操作，不代表本次已经复制：

```bash
ssh -o BatchMode=yes -o StrictHostKeyChecking=yes -o ConnectTimeout=8 \
  hhp@172.16.33.73 \
  'ls -l /data3/hhp/thyroid_gland_segmentation/runs/tg3k_resnet18_unet_seed42/best_model.pth'

# mktemp 创建新目录，不覆盖已有模型或用户源码。不要复制数据集。
target=$(mktemp -d /mnt/data/projects/paus_robot/models/thyroid_gland.XXXXXXXX)
scp -o BatchMode=yes -o StrictHostKeyChecking=yes \
  hhp@172.16.33.73:/data3/hhp/thyroid_gland_segmentation/runs/tg3k_resnet18_unet_seed42/best_model.pth \
  "$target/best_model.pth"
mkdir -p "$target/reference/models" "$target/reference/datasets" "$target/reference/scripts"
scp -o BatchMode=yes -o StrictHostKeyChecking=yes \
  hhp@172.16.33.73:/home/hhp/projects/thyroid_gland_segmentation/src/thyroid_segmentation/models/resnet_unet.py \
  hhp@172.16.33.73:/home/hhp/projects/thyroid_gland_segmentation/src/thyroid_segmentation/models/__init__.py \
  "$target/reference/models/"
scp -o BatchMode=yes -o StrictHostKeyChecking=yes \
  hhp@172.16.33.73:/home/hhp/projects/thyroid_gland_segmentation/src/thyroid_segmentation/datasets/preprocessing.py \
  "$target/reference/datasets/"
scp -o BatchMode=yes -o StrictHostKeyChecking=yes \
  hhp@172.16.33.73:/home/hhp/projects/thyroid_gland_segmentation/scripts/infer.py \
  "$target/reference/scripts/"
sha256sum "$target/best_model.pth"
```

随后必须阅读真实源码、config 和依赖，正式打包必要训练模块到新的本地 bundle。
不能仅凭 “ResNet18-U-Net” 重写网络然后容忍缺失 state_dict keys。
本次没有猜测上游工厂函数名、构造参数、checkpoint config 的键名、
插值库、rounding 或模型层名称。

### 本地训练适配器契约

`model_bundle` 目录必须包含 `adapter.py` 和已核验的训练运行时代码。
适配器是待取得源码后完成的唯一训练专用连接点，本次**尚未提供真实适配器**。
它暴露：

| 函数 | 契约 |
|---|---|
| `validate_config(config)` | 检查真实 checkpoint 的模型、单通道、224x224、letterbox、/255 配置，不符合则抛错 |
| `build_model(config)` | 返回实际训练网络；禁用任何 ImageNet/网络下载；不要自行加载 checkpoint |
| `prepare_image(gray)` | 输入 HxW uint8，返回 float32 NumPy `[1,1,224,224]` 和本包 `imaging.Letterbox`；复用上游精确 resize/padding |
| `restore_mask(binary, geometry)` | 输入 224x224 uint8 0/255；裁掉 padding 后用上游 nearest 实现恢复 HxW，返回 uint8 0/255 |

核心只加载一次 `config` 和 `model_state`，strict state_dict、eval、
inference_mode；CUDA warm-up 默认 5 次，单实例加锁。
sigmoid 后 `>= threshold` 二值化，再恢复尺寸。若参考 infer.py 使用严格 `>`，
需在固定 logits=0 等边界输入上先对齐此约定，不可忽略差异。
`imaging.py` 提供可单测的 OpenCV 参考函数（Python round、居中 padding、
INTER_LINEAR、float32 /255、INTER_NEAREST），**未经源代码核验不得当成训练预处理**。
只有验证证明相同时才让 adapter 复用这些函数。

bundle 的 `manifest.json` 格式：

```json
{
  "task": "whole_thyroid_gland",
  "model": "resnet18_unet",
  "input_shape": [1, 1, 224, 224],
  "resize_mode": "letterbox",
  "normalization": "float32_div_255",
  "reference_parity_verified": false,
  "checkpoint_sha256": "<sha256 of best_model.pth>",
  "files": {
    "adapter.py": "<sha256>",
    "thyroid_segmentation/models/resnet_unet.py": "<sha256>"
  }
}
```

所有 bundle 内的 `.py` 都必须列入哈希，外部依赖版本另行锁定。
`reference_parity_verified` 默认 false，缺文件、哈希错误或未核验均启动失败。
此标记是人工验证记录，不是密码学质量证明。不要仅为了启动而置 true。
bundle 是受信任本地 Python 代码，哈希只检测完整性，不能使不可信代码安全。

## 参数与话题

所有参数都是启动时只读，修改后重启，避免参数显示已更新但模型行为未更新。

| 参数 | 默认值 / 意义 |
|---|---|
| `checkpoint`, `model_bundle` | 空；必须指向已核验本地文件/目录 |
| `device`, `cpu_fallback` | `cuda:0`, `false` |
| `input_topic` | `/ultrasound/us/image_raw`，sensor_msgs/msg/Image |
| `mask_topic` | `/thyroid/gland/mask`，Image mono8，背景0/腺体255 |
| `overlay_topic` | `/thyroid/gland/overlay`，Image bgr8，原图加绿色外轮廓 |
| `status_topic` | `/thyroid/gland/status`，diagnostic_msgs/msg/DiagnosticArray |
| `threshold`, `min_area` | 0.5、1；面积以原图像素计，空掩膜永远 invalid |
| `warmup_frames`, `cpu_threads` | 5、1 |
| `mono16_mode` | `reject`；可显式改成 `fixed_window` |
| `mono16_low`, `mono16_high` | 0.0、65535.0；仅 fixed_window 生效，必须由驱动证据确认 |
| `publish_overlay` | true |
| `stale_timeout_sec` | 2.0；无完成推理时发送 STALE/valid=false |
| `use_sim_time` | ROS 标准参数；配合 bag --clock 使用 |

mono16 的固定规则：
`floor(clip((float32(x)-low)*255/(high-low),0,255)+0.5)`。
不截断 uint16，不按每帧 min/max 归一化。mono16 overlay 是这个固定窗口的
8 位显示，原始 16 位数据仍只在输入话题，不能声称 overlay 无损保留 16 位强度。
mono8 原值保留，rgb8/bgr8 用正确通道顺序转换灰度；彩色 overlay 保留原色。
未知 encoding、畸形 payload 和未配置 mono16 告警、丢弃、发 ERROR/valid=false，
不发布看似有效的空掩膜；后续正常帧可恢复。

mask/overlay/每帧 status 都复制输入 header.stamp/frame_id；图像宽高保持原始尺寸。
无输入时 watchdog 无新图像 header，使用上次完成帧 header 或空 header。
ERROR/STALE 状态中不可用的面积、confidence 和阶段延迟为 -1，valid=false；
不要当作一次新的空预测或纳入性能统计。
min_area 只决定 valid，不擦除小掩膜，不做额外形态学处理。
confidence 是去 padding 后 224 网格中预测前景概率的平均值，空预测为0；
它未经校准，不是诊断置信概率。

订阅和图像发布为 BEST_EFFORT/VOLATILE/depth=1，status 为 RELIABLE/VOLATILE/depth=1。
回调只保存最新帧，最多一帧在推理、一帧等待；等待帧被新帧替换。
模型从不由两个 callback 并发调用。

诊断包含 valid、mask_area_px/fraction、confidence、device、宽高/encoding、
gray_min/max、queue_ms、preprocess_ms、model_ms、postprocess_ms、
end_to_end_ms、最近256帧 end_to_end_mean/p95、processing_fps、
received/processed/errors/overwritten/pending、GPU allocated/peak MB。

- `model_ms` 含 forward 和输出校验，CUDA 同步计时；非仅异步 kernel 入队时间。
- `preprocess_ms` 含 cv_bridge、颜色转换、letterbox、归一化和 H2D。
- `postprocess_ms` 含 sigmoid、D2H、阈值化、恢复、overlay、消息构造及 mask/overlay publish 调用。
- `end_to_end_ms` 为回调接收至 mask/overlay publish 返回的单调时钟时间，含排队，
  不含相机曝光、上游处理、DDS 到达前时间、诊断发布和下游显示。
- `source_age_ms` 为 ROS 当前时间减输入 stamp，未设置/未来时间为-1。
  `source_age_valid` 只表示时间戳非零且差值非负，**不证明时钟同步或年龄可信**。
  bag 不开 sim_time 时显示历史年龄，不要把它当模型延迟。
- overwritten 仅为应用层覆盖数；sensor_msgs/Image 没有序号，不能推算 DDS/驱动总丢帧。
  需要 bag 总帧数、输入观察者计数和原 UltrasoundFrame.sequence_id 联合分析。
- processing_fps 从第一帧 callback 起累计，包含输入停顿；离线指标脚本另报观察到的输出 FPS。

## 启动与观察

先完成本地 bundle 和 PyTorch/CUDA 验收。启动前按仓库规范清理旧运行时：

```bash
python3 /opt/ros/humble/bin/ros2 run paus_bringup stop_paus_runtime --kill --force
python3 /opt/ros/humble/bin/ros2 launch paus_thyroid_segmentation segmentation.launch.py \
  checkpoint:=/absolute/local/best_model.pth \
  model_bundle:=/absolute/local/verified_bundle device:=cuda:0 cpu_fallback:=false
```

本 launch 只有分割节点，不调用 bringup，无运动控制参数需要打开。
本仓库其他 launch 若另行使用，始终 `execute_motion:=false`。
更完整的参数可复制 `config/segmentation.yaml` 到新的本地配置，
或直接使用 `ros2 run ... --ros-args -p name:=value`。
launch 参数优先于 YAML 同名键，checkpoint/model_bundle/device 等需通过 launch 参数传入。

```bash
python3 /opt/ros/humble/bin/ros2 topic list -t
python3 /opt/ros/humble/bin/ros2 topic type /ultrasound/us/image_raw
python3 /opt/ros/humble/bin/ros2 topic info /ultrasound/us/image_raw --verbose
python3 /opt/ros/humble/bin/ros2 topic echo /ultrasound/us/image_raw --once --no-arr \
  --qos-reliability best_effort
python3 /opt/ros/humble/bin/ros2 topic echo /thyroid/gland/status --once
python3 /opt/ros/humble/bin/ros2 topic hz /thyroid/gland/mask
```

## 回放、合成输入、性能

优先回放，不连接机器人。每个回放终端在前述环境激活后再 source：

```bash
source /mnt/data/projects/paus_robot/worktrees/ultrasound-online-stack/install/local_setup.bash
source /mnt/data/projects/paus_robot/worktrees/thyroid-gland-segmentation/install/local_setup.bash
export ROS_DOMAIN_ID=87
export ROS_LOCALHOST_ONLY=1
```

四个终端同 domain：分割节点加 `use_sim_time:=true`；转发；回放；观察。
转发器只是发布 `.image`，不重写 header、像素或尺寸。

```bash
# 转发终端
python3 /opt/ros/humble/bin/ros2 run paus_thyroid_segmentation thyroid_frame_bridge
# 回放终端
python3 /opt/ros/humble/bin/ros2 bag play \
  /mnt/data/projects/paus_robot/rosbags/us_real_20260924_interactive_scan \
  --clock --topics /ultrasound/us/frame
# 测量终端，至少完整运行一份 bag，并用相同设置重复
python3 /opt/ros/humble/bin/ros2 run paus_thyroid_segmentation thyroid_validate benchmark --seconds 100
# GPU 工作之前以及测量期间的另一个终端
nvidia-smi
```

没有 bag 时可使用合成图。它只验证消息链路，不验证模型质量：

```bash
python3 /opt/ros/humble/bin/ros2 run paus_thyroid_segmentation thyroid_validate synthetic \
  --encoding mono8 --width 179 --height 1129 --fps 45 --seconds 20
```

benchmark JSON 包含各阶段平均/P95、输出 FPS、节点显存和丢帧计数；
无输出/无推理状态时命令返回失败。`gpu_peak_allocated_mb` 仅是该模型 PyTorch
分配的峰值，不含全部驱动/其他进程显存，需与 nvidia-smi 联合记录。
把测量输出保存到 `/mnt/data/projects/paus_robot/runs`，不要提交 bag 或 checkpoint。

## 一致性验证

拿到真实源码后，用固定灰度图覆盖方形、极窄、横向、奇数尺寸、
全黑、低动态范围；通过训练 `infer.py` 的同一预处理/后处理路径导出 NPZ：
`gray` (HxW uint8)、`input` (float32 [1,1,224,224])、
`mask` (HxW uint8 0/255)。不能用本包生成参考答案。
先直接比较 adapter 与参考实现，成功后再记录 manifest parity=true，
随后运行部署加载校验：

```bash
python3 /opt/ros/humble/bin/ros2 run paus_thyroid_segmentation thyroid_validate compare \
  --checkpoint /absolute/local/best_model.pth \
  --model-bundle /absolute/local/verified_bundle \
  --reference /absolute/local/from_training_infer.npz --device cpu
```

此命令要求输入 tensor 和 mask 逐元素相等，CPU/CUDA 分别验证，
临界阈值差异单独分析。随后停止其他图像发布者，在独立测试 domain 中启动
真实分割节点，使用同一 NPZ 检查 ROS 输出：

```bash
python3 /opt/ros/humble/bin/ros2 run paus_thyroid_segmentation thyroid_validate compare-ros \
  --reference /absolute/local/from_training_infer.npz
```

该命令同时检查 mask、尺寸、encoding、stamp 和 frame_id；
`tests/test_ros_pipeline.py` 覆盖消息层保留，但本次测试引擎不能替代真实权重对齐。

## 自动测试及验收边界

```bash
# 不加载环境中无关 pytest 插件；不会跳过本包测试本身
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 \
  TG_TEST_BAG=/mnt/data/projects/paus_robot/rosbags/us_real_20260924_interactive_scan \
  TG_TEST_REPORT_DIR=/mnt/data/projects/paus_robot/runs/thyroid_gland_integration_20261008 \
  python3 -m pytest src/paus_thyroid_segmentation/tests -q -rs

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 \
  python3 /usr/bin/colcon test --packages-select paus_thyroid_segmentation \
  --pytest-args -q
python3 /usr/bin/colcon test-result --verbose
```

测试覆盖编码/强度转换、letterbox 和去 padding、nearest 尺寸恢复、畸形消息、
大小端与行 padding、header 保留、未知编码恢复、watchdog、单槽覆盖、真实 DDS
传输和真实 rosbag 回放。Torch 生命周期测试需要 PyTorch，否则显式 skip。
真实 bag 测试需要 `TG_TEST_BAG` 和 UltrasoundFrame underlay，否则显式 skip。

本次 NumPy 测试引擎只在测试代码中注入，无生产 ROS 参数可切换到它。
测试人为 sleep 40 ms 以制造过载；这类吞吐和延迟只能说明队列行为。
模型验收仍缺：上游源码审计、checkpoint 与依赖打包、真实 infer.py/ROS parity、
Torch 生命周期执行、CUDA 推理与显存/FPS/P95、真实设备域偏移和长时间稳定性。

### 2026-10-08 验证记录

- 普通 colcon build 成功，安装节点 shebang 指向
  `/home/chen_lab/miniconda3/envs/paus_robot/bin/python3`。
- pytest：43 passed、3 skipped；colcon test-result：
  46 tests、0 errors、0 failures、3 skipped。跳过项都是缺少 PyTorch 的
  resident/eval/inference_mode、CPU fallback、min_area 真实 Tensor 测试。
- 两份真实 bag 分别完成短时 `ros2 bag play --rate 2` 传输测试；
  每份测试运行4秒，不是整段 bag 的模型验收。元数据/像素统计是完整顺序读取。
- 固定参考 NPZ 的 compare-ros 命令已在真实 DDS 上测试 mask/header/尺寸相等；
  参考和引擎仅为测试用 NumPy 阈值规则，不是训练 infer.py 的输出。
- 缺少依赖/模型时生产节点非零退出；包的3个 console entry points、
  launch `--show-args`、bag 全量 inspect 命令可用。
- 没有在线超声设备，没有启动或修改运动控制，没有复制训练数据或权重。

以下仅为 **NumPy 测试引擎 + 人为40 ms sleep** 的一次快照，
不是 ResNet18-U-Net 性能，GPU 显存不适用：

| 输入 | 输出累计 FPS | 节点内平均/P95 ms | 收到/覆盖/完成 |
|---|---:|---:|---:|
| 合成 100 Hz，200帧 | 24.12 | 46.47 / 51.45 | 200 / 150 / 49 |
| 互动 bag，2倍速短时回放 | 23.62 | 48.37 / 58.70 | 332 / 244 / 87 |

快照时还有待处理帧，所以不是完整运行结束后的总数。
原始分阶段报告位于：
`/mnt/data/projects/paus_robot/runs/thyroid_gland_integration_20261008/synthetic_transport.json`
和同目录 `real_bag_transport.json`。重跑同报告目录会更新测试报告。
本次没有任何可以报告为“实际训练模型 GPU FPS/平均/P95/显存”的数字。

## 研究与许可

用户提供的历史模型结果为：544 张测试图，mean Dice 0.8880、median Dice
0.9403、mean IoU 0.8283、空预测2张、Dice<0.5 共23张，纯模型延迟约2.9 ms。
本次没有重算这些指标；2.9 ms 不包含 ROS 和预/后处理，不能作为实时端到端保证。
valid 仅表示处理成功且预测面积达标，不代表正确解剖定位。
这是研究原型，不是临床保证，也不能用于自动决策或机器人运动依据。

TG3K 数据许可按项目提供信息为 CC BY-NC 3.0；本模型限非商业研究，
除非另行解决数据许可。分发权重/训练源码前保留来源、作者归属、修改说明和原许可证；
本包的 Proprietary 元数据不替代上游代码和数据的许可。
