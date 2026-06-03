# 实验性功能 (Experimental)

以下功能仅在 psacal 构建的镜像中可用，未合入 [KawaiiHachimi/linux-gaokun-buildbot](https://github.com/KawaiiHachimi/linux-gaokun-buildbot) upstream。

## 概览

| 功能 | 组件 | 状态 | 默认启用 |
|------|------|------|---------|
| 加速度计 evdev 桥接 | ssc-bridge | 实验性 | 否 |
| 陀螺仪 evdev 桥接 | ssc-bridge | 实验性 | 否 |
| 光照传感器文件输出 | ssc-bridge | 实验性 | 否 |
| hexagonrpcd \r strip | hexagonrpcd | 实验性 | 是 (编译进二进制) |
| uinput 内核模块 | linux-modules | 稳定 | 是 |

## SLPI 传感器 evdev 桥接 (ssc-bridge)

将 Qualcomm SLPI DSP 传感器数据桥接到标准 Linux evdev：
- **加速度计** → uinput evdev (`ssc-accelerometer`, ABS_X/Y/Z, INPUT_PROP_ACCELEROMETER)
- **陀螺仪** → uinput evdev (`ssc-gyroscope`, ABS_RX/RY/RZ)
- **光照传感器** → `/run/ssc-bridge/light` (文本文件, lux 值)

### 依赖

- **hexagonrpcd** 带 `\r` strip 补丁 (本仓库 `patches/hexagonfs-cr-strip.patch`)
- **libssc** (https://codeberg.org/dylanvanassche/libssc)
- **传感器配置文件** — 需从 Windows 分区手动部署 (见下方)

### 开启方法

```bash
# 1. 部署传感器配置文件 (一次性)
sudo bash deploy-sensors.sh

# 2. 启动桥接服务
sudo systemctl start ssc-bridge

# 3. 验证
sudo evtest /dev/input/event13    # 加速度计 — 晃动机器看 XYZ 变化
sudo evtest /dev/input/event14    # 陀螺仪
cat /run/ssc-bridge/light         # 光照 (lux)
```

### 已知限制

- **mount matrix**: 当前使用单位矩阵。方向检测 (iio-sensor-proxy) 可检测旋转，但轴线可能需要按设备校准。可通过环境变量 `SSCB_ACCEL_MOUNT_MATRIX` 和 `SSCB_GYRO_MOUNT_MATRIX` 设置
- **光照传感器**: 仅输出到 `/run/ssc-bridge/light`，不支持自动亮度调节 (iio-sensor-proxy 的 `drv-input-light` 尚未实现)
- **传感器并发**: ssc-bridge 运行时会占用 QMI 会话，此时 ssccli 会超时——这是正常行为
- **仅 Gaokun3 测试过**: SC8280XP / Huawei MateBook E Go 2023。其他 SLPI 设备未测试

### 传感器配置文件部署

传感器配置文件 (JSON 驱动配置、sns_reg.conf、校准数据) 来自 Windows DriverStore/DriverData，
不可进入本仓库的 CI 流程。

部署方式：参考仓库中的 `deploy-sensors.sh` 脚本，或手动参考 `DEPLOY.md`。

## hexagonrpcd \r strip 补丁

Qualcomm DSP 固件 (`qcslpi8280.mbn`) 由 Windows 工具链编译，文件路径中带
trailing `\r`（如 `hw_platform\r`）。Linux hexagonrpcd 的 hexagonfs 虚拟文件系统
不识别此字符，导致 `sensors/registry/registry` 等文件无法被 DSP 找到。

补丁位于 `patches/hexagonfs-cr-strip.patch`：在 `copy_segment_and_advance()` 函数中
strip 路径段末尾的 `\r`。

镜像构建时通过 `scripts/ci/lib/install-hexagonrpcd.sh` 自动应用并编译。

## 上游计划

| 组件 | 计划 |
|------|------|
| `CONFIG_INPUT_UINPUT=m` | 已向 KawaiiHachimi 提 PR |
| ssc-bridge | 待 hexagonrpcd \r patch 上游化和 libssc API 稳定后提 PR |
| hexagonrpcd \r patch | 待多平台测试后向 linux-msm/hexagonrpc 提 PR |
