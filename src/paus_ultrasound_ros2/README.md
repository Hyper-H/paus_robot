# USFR v2 单路 US 图像接收

本包接收 Windows 窗口采集器或已有 `display_ros2` 程序发送的 USFR v2 US 图像。
它只负责 TCP 接收和 ROS 2 发布，**不会**启动 VSX、超声硬件或机器人运动。
`protocol_v2.py` 是 Windows 发送端编解码文件
`components/display_ros2/python/protocol_v2.py` 的兼容扩展，
保留 Apache-2.0 许可；见 [Apache-2.0.txt](Apache-2.0.txt)。原有 v2 基础字段和 ACK 不变。

## 主控集成与窗口采集

`ui.launch.py` 可同时启动主控页面和单路接收器；默认不启用超声，
显式启用并指定 Linux 局域网地址：

```bash
ros2 launch paus_bringup ui.launch.py execute_motion:=false \
  enable_ultrasound:=true ultrasound_listen_host:=<LINUX_IPV4>
```

`online_stack.launch.py` 同样接受 `enable_ultrasound`、
`ultrasound_listen_host`、`ultrasound_listen_port` 和
`ultrasound_stale_timeout_s`。两个入口都默认回环地址和 `55001`；
不要在同一端口重复启动接收器。Windows 可先启动并等待连接，也可在
Linux 之后启动，软件连接不会触发机械臂运动。

只验证超声和主控页面、不开相机或标定节点时：

```bash
ros2 launch paus_bringup ui.launch.py execute_motion:=false \
  start_camera_bridge:=false start_image_receiver:=false start_calibration_node:=false \
  enable_ultrasound:=true ultrasound_listen_host:=<LINUX_IPV4>
```

主控页面新增独立的“超声图像”折叠区，不覆盖手眼相机；它通过
BEST_EFFORT、深度 1 订阅 `/ultrasound/us/frame`，使预览和元数据同帧。
HTTP JPEG 是监看用副本，最多每 250 ms 请求一次，不改变 ROS 原始图像。

`stale_timeout_s` 默认 1 秒。TCP 保持连接但没有完整新帧时变为 `STALE`，
`timed_out=true`，丢弃尚未发布的旧帧；恢复完整新帧后回到 `STREAMING`。
断线后回到 `LISTENING` 并清空待发布缓冲。此超时检测的是传输新鲜度，
不能检测厂家持续重复发送同一冻结画面，也不能撤回已经发布的 ROS 消息。
消费者仍须检查自己的超时和状态，不得把图像流可用当成运动许可。

窗口采集器可附带以下可选元数据（旧发送器可省略）：

- `roi_confirmed`、`capture_size: [w,h]`、`roi_pixels: [x,y,w,h]`；ROI 尺寸须与输出尺寸一致。
- `profile_revision`、`imaging_context`，用于记录人工确认的配置和深度/缩放等条件。
- `display_scale_mm_per_pixel: {horizontal: number|null, vertical: number|null}`，来自显示标尺的单轴比例。

未知轴保持 `null`。显示标尺比例不会填写 `pixel_spacing_m` 或把
`geometry_valid` 改成 `true`；它不证明物理尺寸精度或机器人空间标定。
这些字段保留在 `frame.metadata_json`，状态中的 `frame_metadata` 提供
最近接收帧的摘要；断线后的元数据可能是历史值，须同时检查状态。

## 构建与本机测试

在本工作树根目录、实验室的 `paus_robot` 环境中执行：

```bash
source /opt/ros/humble/setup.bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate paus_robot
source /mnt/data/projects/paus_robot/repo/install/local_setup.bash
# 系统 ROS 接口生成器路径仅用于本次构建，不带入主控运行环境。
PYTHONNOUSERSITE=1 PYTHONPATH="$PYTHONPATH:/usr/lib/python3/dist-packages" \
  colcon build --packages-select paus_interfaces paus_ultrasound_ros2 paus_ui paus_bringup
source install/local_setup.bash
```

本机定向测试关闭自动加载的无关 pytest 插件；启动文件测试需要的系统
`lark` 路径放到最后，避免旧版系统 Web 依赖覆盖 Conda 环境：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1 \
  python3 -c 'import sys; sys.path.append("/usr/lib/python3/dist-packages"); import pytest; sys.exit(pytest.main(["-q", "src/paus_ultrasound_ros2/tests", "src/paus_ui/tests", "src/paus_bringup/tests"]))'
```

## 链路与启动

Windows 超声电脑的 VSX 根据选定的 Setup 生成、重建并显示 US 图像；
该电脑上的 `display_ros2` 从显示回调取图，按 USFR v2 通过 TCP 发送；
Linux 接收节点校验帧并发布 ROS 话题。这里的“单窗口”是指 Setup
有一个 US 图像显示窗口，不是指只用一块物理显示器。

Linux 上先启动接收节点。默认只监听 `127.0.0.1:55001`，不能用于跨机；
跨机时明确指定 Windows 超声电脑可以访问的 Linux 网卡 IPv4 地址：

```bash
ros2 launch paus_ultrasound_ros2 ultrasound.launch.py listen_host:=<LINUX_IPV4>
```

Windows 端使用**超声电脑已安装的** `display_ros2` 通信助手；
不要改用先前单独提供的同名 EXE，它与当前安装的文件不一致。
助手 GUI 是运行配置与启停的图形界面；其控制器后端也可以通过 SSH 调用，
但 GUI 内已有设置不一定适合本次测试，启动前必须核对配置。
Setup 是 VSX 的 MATLAB 成像配置文件，决定采集/重建/显示行为；
它不是协议版本，也不是 IP 设置。

仿真联调时，明确选择已验证的单窗口 US Setup
`D:\VantageNXT-2.0.0\WJD\us\flashangle\SetUpDoppler_L2_5vFlashAngles.m`、
`SoftwareOnly`（助手配置中的 `simulation`）、USFR v2、
目标 `<LINUX_IPV4>:55001`，再启动成像和发送。此模式不验证真实探头。
本轮不启动机器人控制。

查看接收结果：

```bash
ros2 topic list | grep /ultrasound/us
ros2 topic echo /ultrasound/us/status
ros2 topic echo /ultrasound/us/image_raw --field height --once --qos-reliability best_effort
ros2 topic echo /ultrasound/us/frame --once --qos-reliability best_effort
```

- 一键查看真实图像参数并录制：

  ```bash
  ros2 run paus_ultrasound_ros2 record_ultrasound_bag
  ```

  命令会等待首个 `/ultrasound/us/frame`，打印实际
  `width`、`height`、`encoding`、`step` 和每帧字节数，然后开始录制。
  完成后按 `Ctrl+C`，数据默认保存到
  `/mnt/data/projects/paus_robot/rosbags/us_real_YYYYmmdd_HHMMSS`，
  并自动执行 `ros2 bag info`。默认录制 `/ultrasound/us/frame` 和
  `/ultrasound/us/status`；`frame` 已包含完整图像，所以不重复录制
  `image_raw`。需要标准图像话题时使用：

  ```bash
  ros2 run paus_ultrasound_ros2 record_ultrasound_bag --include-image-raw
  ```

- `/ultrasound/us/image_raw`：`sensor_msgs/Image`，行优先 `mono8` 或 `mono16` 图像。
- `/ultrasound/us/frame`：`paus_interfaces/msg/UltrasoundFrame`，同时包含图像和已校验的原始帧头；后续算法优先订阅它。
- `/ultrasound/us/status`：JSON 状态，包含连接状态、收帧/ACK 数、跳号、覆盖、拒收、断线、近两秒帧率及最新帧年龄。

图像的 ROS 时间戳是 **Linux 收帧时间**；帧头中的 `source_time_s`
是 Windows 显示回调时间，不能当成经过标定的声学采集时间。

**ACK 是确认应答**：接收端完整读取、验证一帧并放入仅保留最新一帧的
缓冲后，向发送端回复对应 session 和序号的 ACK。ACK 不表示 ROS
订阅者已经显示、处理或录制该帧；网络接收快于发布时旧帧也可能被覆盖。
验收时必须同时检查状态计数和实际 ROS 图像，而不能只看 Windows ACK。

## 后续真人/实机使用的建议顺序

以下是工程联调顺序，**不是临床操作规程，也不是已通过的实机验收步骤**：

1. 由有资质的现场操作者按设备规定连接、检查探头及超声硬件，
   确认使用场景和安全流程；本接收节点不控制硬件。
2. Linux 先启动接收节点，确认监听 IP/端口；不要启动机器人运动。
3. 超声电脑选择**经现场确认适用于实际探头和硬件**的单窗口 US Setup，
   将助手切到真实硬件模式，核对协议为 v2、目标为 Linux 的 IP 和 `55001`。
   仿真用 Setup 是否适用于该探头不能由本次 SoftwareOnly 测试推断。
4. 现场操作者启动 VSX 成像并确认 Windows 屏幕图像正常，再开启发送；
   在 Linux 核对 `/ultrasound/us/status` 的收帧数持续增加，且
   `/ultrasound/us/image_raw` 持续更新、尺寸、方向和图像内容符合预期。
5. 单独测量实际帧率、延迟、断线重连和图像方向；进行时钟、几何及设备
   安全验证后，才能另行讨论任何依赖图像的机器人功能。

目前没有完成真实探头、人体图像、空间标定、PA 或图像引导运动的验收。
后续 v3/PA 应另加协议解码和 PA 接收端，保持现有 US 话题及聚合帧接口不变。

## 验证记录（2026-09-17）

- 17 项本机测试通过：ROS 发布、坏包/截断包、正常断线及重连、
  `mono8`/`mono16` 非对称矩阵像素位置与数值。
- Windows `172.16.33.58` 用已安装的 v2 编解码器发送 `2x3`
  固定矩阵至 Linux `172.16.33.132:55001`；收到匹配 ACK，
  ROS 帧中的数值与矩阵位置一致。
- 通过 SSH 调用已安装的 `desktop_control` 控制器后端启动 VSX
  `SoftwareOnly`；使用上文单窗口 Setup、仿真模式、v2、
  `172.16.33.132:55001`，没有改动 GUI 已保存的硬件/v3 配置。
  首次运行结束时网络侧记录 26 次 ACK、生产端记录 65 帧；
  Linux 接收端采样时记录 27 帧接收和 27 次 ACK。
- 第二次仿真运行结束时记录 15 次 ACK、生产端 35 帧；
  活跃的 ROS 订阅者实际收到 `/ultrasound/us/image_raw`，
  读取到图像高度 1129。各计数采样时机和所在处理阶段不同，
  不应直接要求相等。
- Windows 运行记录位于
  `D:\VantageNXT-2.0.0\WJD\.codex-work\display-ros2\ssh-v2-validation\runs\`
  下的 `20260917_225048_fa370a` 和 `20260917_225320_7624bc`。
  这只证明软件仿真图像可以跨机进入 ROS；真实超声硬件仍待现场验收。
