# car-for2 小车项目：AI 修改与调试交接包

> 快照日期：2026-10-27，用户时区 Asia/Shanghai。
> 用途：交给另一个 AI，在本地工程继续修改、构建、烧录和定位实车问题。
> **先阅读第 1、5、6、9 节。源码已经修改，不等于最新固件已构建、烧录或实车通过。**

## 目录

1. 当前结论、用户要求与验证边界
2. 工程路径、平台、路线及单位约定
3. 硬件接口与操作方法
4. 工程结构和运行契约
5. 已完成的两批修改
6. 当前参数与尚未应用的建议值
7. 构建、测试、烧录与只读诊断命令
8. 诊断记录、OLED 和历史实测数据
9. 待解决问题与建议接手顺序
10. 不应破坏的约束
11. 当前参数头文件快照
12. 接口和到点识别参考源码
13. 当前修改的统一补丁
14. 源码 SHA-256 清单
15. 可直接交给下一位 AI 的任务提示

## 1. 当前结论、用户要求与验证边界

### 1.1 用户最新要求

- 小车行驶太慢。
- A 点原地转向太慢，且出现转过头再反向纠偏。
- 用户观察只有 A→C 成功，小车到 C 后未看到声光提示，也未看到后续路段执行。
- **已有速度、转向增益等参数：告诉用户位置和建议值，由用户自行修改。逻辑代码由 AI 修改。**
- 当前必须保持 **FULL 全程 A→C→B→D→A**。用户明确纠正过：不要误烧 AC/DA 等单段模式。
- 本文中的“建议试值”没有替用户写入现有速度/增益宏；新增逻辑必要参数已加入头文件。

### 1.2 两个版本的验证证据必须分开

| 阶段 | 已有证据 | 不能据此声称的内容 |
| --- | --- | --- |
| 原始 FULL 固件 | 用户本机在 NORMAL、100 kHz 下下载并校验成功，执行过复位 | 不代表 IMU 或整条路线通过实车验收 |
| IMU 参考工程移植后 | 用户本机 FULL 构建成功：RAM 12632 B、FLASH 49380 B；12/12 主机测试通过，包含 imu_heading_test | 这次通过发生在最新“转向/入弧/声光”改动之前 |
| 最新转向、C/D 捕获、声光和 OLED 修改 | 改动源文件和扩展的 motion 测试经 ARM Clang Cortex-M3 对象编译检查；已补回归用例 | **尚未得到本批修改的完整 GNU 链接成功、主机回归执行通过、新固件烧录或实车验证证据** |

当前 AI 工具环境运行 ARM GCC / MinGW GCC 时返回 `Illegal System DLL Relocation`，直接访问 ST-LINK 时返回 `DEV_CONNECT_ERR` 或 `No debug probe detected`。用户自己的 PowerShell 则能正常构建、运行测试、连接并烧录。

**因此：需要在用户本机重新构建并测试最新源码。不要把现有旧 ELF 当作刚生成的新固件。也不要把当前 AI 的 ST-LINK 访问失败当作小车硬件通信失败。**

## 2. 工程路径、平台、路线及单位约定

| 项目 | 当前信息 |
| --- | --- |
| 当前工程 | `D:\STM32project\car-for2` |
| IMU 参考工程（只读参考，未由本次任务修改） | `D:\STM32project\car-02-` |
| Git 分支 | `main` |
| 补丁对应 Git HEAD | `0d903bf5959cdabfc2fb9b7c9ca66b15a50b9499` |
| 主板 | C10B |
| 工程 MCU 配置 | STM32F103RCT6 / STM32F103xE / Cortex-M3 |
| 用户 ST-LINK 识别 | Device ID 0x414，High-density，Flash 256 KiB |
| 主频 | HSE 8 MHz，经 PLL 到 72 MHz，参见 main.c |
| RAM | 48 KiB |
| 程序 Flash 链接区 | 254 KiB |
| Flash 最后 2 KiB | `0x0803F800` 到 `0x0803FFFF`，保存灰度学习阈值 |
| FULL 构建目录 | `build\Mission-Debug` |
| FULL 目标文件 | `build\Mission-Debug\C10B_Test.elf` |
| 地图文件 | 同目录 `C10B_Test.map` |
| 默认烧录配置 | `.vscode/settings.json` 中 `stm32flash.buildDir = build/Mission-Debug` |

路线尺寸：AB=1000 mm，AD=BC=800 mm，半圆半径 400 mm。A 点起步时车头水平朝右。

| 路段 | 行为 | 目标方向/几何 |
| --- | --- | --- |
| AC | A 点原地右转，再 IMU 保持航向直行，确认 C 的白→黑边沿 | -38.66°，对角线约 1281 mm |
| CB | C 点连续交接，捕获右侧半圆并巡线到 B | 入弧切线 0°，出口切线 180°，半圆约 1257 mm |
| BD | B 点先制动静止，再原地左转并保持航向直行 | 218.66°，等价于 -141.34°，对角线约 1281 mm |
| DA | D 点连续交接，捕获左侧半圆并巡线回 A，确认后制动 | 入弧切线 180°，出口切线 0°，半圆约 1257 mm |

- 航向：右/东 0°，上/北 +90°，左/西 180°；`MOTION_YAW_SIGN=+1`。
- 速度单位是 **编码器计数/秒 cps**，不是 PWM、mm/s 或 RPM。
- 当前左右均用 6.730141612 counts/mm，这是待实车复核的旧估计值。
- 换算：900 cps ≈133.7 mm/s；1500 cps ≈222.9 mm/s。
- 编码器归一化后两轮前进计数均为正；不要再次反转已经归一化的右轮。
- 路段距离基准独立：直行距离在转向/制动稳定后开始；半圆距离在交接时开始。
- `nominal_mm` 是名义值和诊断信息；**不是“到这个距离就强制完成”的条件**。

## 3. 硬件接口与操作方法

以下依据当前源代码以及同 C10B 参考工程。替换硬件后仍须实测确认方向、极性和安装。

| 模块 | 引脚/外设 | 说明 |
| --- | --- | --- |
| MPU6050 SCL | PB14 | 软件 I²C，开漏 |
| MPU6050 SDA | PB15 | 软件 I²C，开漏 |
| MPU6050 地址/标识 | 0x68（兼容探测地址 0x69）/ WHO_AM_I 0x68 | 不能把地址应答等同于测量完全正常 |
| MPU6050 INT | 不使用 | 轮询 INT_STATUS/DATA_RDY |
| 灰度 UART TX/RX | UART4 PC10/PC11 | 115200，8N1，只有 line_sensor 驱动拥有 UART4 接收 |
| 左编码器 | TIM4，PB6/PB7 | 当前 Encoder_Data 中逻辑左轮 |
| 右编码器 | TIM8，PC6/PC7 | 原始反向已在 encoder.c 中归一化 |
| 左电机 | TIM3 CH1/CH2，PA6/PA7 | 电机方向宏 LEFT_DIRECTION=+1 |
| 右电机 | TIM3 CH3/CH4，PB0/PB1 | 电机方向宏 RIGHT_DIRECTION=-1 |
| USER 按键 | PA0，高电平按下 | 待机短按松开启动；运行按下停止 |
| LED | PC2，低电平亮 | 本次加入与到点蜂鸣同步的 300 ms 提示 |
| 蜂鸣器 | PA15，默认高电平有效 | 按 car-02- 同主板映射；若硬件极性不同查 MISSION_BUZZER_ACTIVE_HIGH |
| OLED | PC14 时钟、PB5 数据、PB4 复位、PB3 D/C | 软件 SPI，按当前 OLED.h 接线；并非 IMU 的 I²C |
| SWD | PA13/PA14 | 保留调试；MSP 已关闭 JTAG 释放 PA15/PB3/PB4 |

按键流程：上电保持传感器平放固定、Z 轴垂直，静止 3～5 秒；灰度已完成黑白学习且在线后，按所选模式摆放，再短按 USER 并松开。

- 运行时按下立即停止，释放不能再次启动。
- 待机按住约 6 秒后释放进入灰度校准；校准期间再按取消。
- 启动被拒绝后不会自动起跑，条件恢复后须再次明确短按启动。
- IMU 就绪只表示当前 Z 轴航向通道通过约定校准，不表示 X 轴饱和已恢复或六轴均健康。

## 4. 工程结构和运行契约

```text
Core/Src/main.c
  Mission_Init / Mission_Update
    MissionPort_Service / Capture / Command / NotifyPoint / Draw
      采集编码器、灰度、IMU；统一电机执行、OLED 和声光
    MissionControl_Start / Update
      FULL 状态机、传感器门槛、段间交接、到点事件
      segment_ac / segment_cb / segment_bd / segment_da
        motion_straight / motion_line
        motion_distance / motion_edge
```

| 文件/目录 | 用途 |
| --- | --- |
| `Core/Inc/motion_tuning.h` | 统一运动参数入口 |
| `Core/Src/Modules/imu_config.h` | IMU 量程、采样、校准参考及门槛 |
| `Core/Src/Modules/imu_mpu6050.c/.h` | GPIO 软件 I²C、初始化、采样、故障及诊断记录 |
| `Core/Src/Modules/imu_heading.c/.h` | Z 轴静止零偏估计、相对航向积分、独立手动 gyro test |
| `Core/Src/Modules/encoder.c` | 16 位计数回绕、右轮符号归一化、累计距离 |
| `Core/Src/Modules/speed_control.c` | 轮速 PI、速度斜坡、转向轮同步、反馈保护 |
| `Core/Src/Modules/line_sensor.c` | 8 路灰度 UART 模拟量协议及新帧序号 |
| `Core/Src/Modules/line_calibration*.c` | 黑白学习和 Flash 阈值保存 |
| `Core/Src/Mission/mission.c` | 按键、采样快照、执行、144 字节 mission_diag |
| `Core/Src/Mission/mission_control.c` | 总调度器；完整模式自动串接四段 |
| `Core/Src/Mission/mission_segment.h` | 输入、输出、上下文、阶段和故障枚举 |
| `Core/Src/Mission/motion_straight.c` | 静止→转向→稳定→航向直行 |
| `Core/Src/Mission/motion_line.c` | 入弧捕获和正常巡线 |
| `Core/Src/Mission/motion_edge.c` | 限定距离内新帧黑白转换检测 |
| `Core/Src/Mission/motion_drive.c` | 唯一驱动适配；C/D 连续运行，A/B 切换制动/转向配置 |
| `Core/Src/Mission/mission_port_stm32.c` | 硬件采样、蜂鸣/LED、OLED |
| `scripts/build_mission.ps1` | FULL 或单段 ARM 构建 |
| `scripts/test_mission.ps1` | 主机 CMake/CTest 回归 |
| `scripts/read_mission.ps1` | 根据 map 自动找到并解码运行诊断 |
| `tests/test_imu_heading.c` | 前一批新增的零偏、量程、超时和计数回绕测试 |
| `tests/test_motion_algorithms.c` | 本批扩展转向末段和 C/D 捕获测试 |
| `tests/test_mission_actual_route.c` | 真实控制器/驱动组合的理想反馈全程测试，不是真实轮胎动力学仿真 |

关键契约：

1. 每次前台循环采集一次不可变 MissionInput，由当前段和下一段共享同一快照。
2. 段函数只返回目标，不能自行驱动电机或重置 IMU/编码器。
3. FULL 模式确认 C/D 后，在同一交接快照进入下一段，保持前进配置，不重置 PI/斜坡/累计编码器。
4. B 明确制动并原地转向；回到 A 制动完成。
5. 完成一段才产生一个到点事件；起始 A 不产生事件，事件顺序是 C、B、D、A。
6. 下一段进入失败也保留已经发生的到点事件。
7. 调度器已经实现 FULL 自动交接；不能仅凭“看上去到 C”就认定代码已经确认了 C。

## 5. 已完成的两批修改

### 5.1 前一批：参考 car-02- 移植 IMU

参考文件：

- `D:\STM32project\car-02-\Core\Src\mpu6050.c`
- `D:\STM32project\car-02-\Core\Src\imu_filter.c`
- `D:\STM32project\car-02-\Core\Inc\car_config.h`

具体改动：

- PB14/PB15 保持不变，HAL 开漏 GPIO 写入、高速 GPIO 配置、5 us 软件 I²C 阶段；保留 ACK、时钟拉伸/事务超时和 9 脉冲总线恢复。
- 对已经唤醒的器件直接写配置，不再执行旧版额外芯片复位/信号路径复位。
- INT_PIN_CFG=0；DLPF=3；采样分频 0；器件采样 1 kHz，主机轮询 5 ms。
- GYRO_CONFIG=0x08，即 ±500 deg/s；换算统一用 **65.5 LSB/(deg/s)**。旧版 ±250 的 131 换算不能再用。
- 单独读 0x3A 状态并要求 DATA_RDY，然后单独读取 0x47/0x48 的新 Z 样本。
- 每次开机以 400 个新样本、至少 1995 ms、轮子静止、驱动停止、标准差 ≤0.8 deg/s 实测本次零偏。
- 静止参考中心是参考工程对同一已安装传感器实测的约 -23 deg/s；允许 ±10 deg/s 校准窗口。**不是直接把固定 -23 当实际零偏**。
- 参考值用于识别静止窗口，实际 mean/bias 每次重新估计。换传感器后须重测参考值。
- 当前只用 Z 轴估计相对偏航，X 加速度异常仍保留诊断，但不再决定 Z 航向是否完成校准。
- 30 ms 新样本间隔保护、陀螺超量程保护及任务启动/运行传感器门槛仍保留。
- Imu_Update 接口增加 wheels_still；硬件适配通过新鲜编码器、左右零增量、speed_control.running=0 生成该条件。
- 手动 gyro test 和诊断输出同步使用新量程；手动 test 不授予电机启动权限。

### 5.2 本批：针对速度慢、转向过冲及 C 后不继续

**用户原有巡航速度、转向最大角速度、增益数值没有自动修改。** 代码新增：

1. 转向接近目标的最后 12° 内，下调最低轮速至独立的 120 cps 下限，避免旧固定 350 cps 抵消减速。
2. 仅朝目标旋转时添加 0.04 s 角速度制动提前量，制动角度限制在原 2～3° 区间内；保留有限纠偏和转向异常保护。
3. 轮速控制在精细转向区间同步减小固定 30% 摩擦前馈，避免低速目标仍被较大启动补偿推着走。
4. C→CB、D→DA 连续交接的捕获阶段，增加朝入口切线的航向辅助，仍保持两轮正向；FOLLOW 阶段恢复纯巡线 PD。
5. 捕获阶段可接受单组连续宽黑带；宽黑带不能单独确认捕获成功。需要新鲜、居中的窄黑线帧及入口方向误差 ≤20°。
6. 捕获阶段丢线/歧义允许有界的 350 ms 搜索；FOLLOW 仍保留原 100 ms 门槛。总捕获 1500 ms/200 mm 界限没有放开。
7. 到点事件加入 PA15 蜂鸣器，与 PC2 LED 非阻塞提示 300 ms；没有把“名义距离到达”改成强制到点。
8. OLED 显示具体段故障及最近到点、次数、灰度 mask、确认帧数。
9. 扩展 motion 回归用例：两种转向方向的末段减速/制动，C/D 航向辅助，宽黑带不能提前进入 FOLLOW，捕获/巡线不同保护时限。

**重要：这批是对代码中已发现薄弱环节的修正，不是已经通过实车试验证实 C 点唯一根因。还需要 C 点后的实际诊断记录。**

## 6. 当前参数与尚未应用的建议值

所有表格当前值均来自此时 `Core/Inc/motion_tuning.h`。

| 参数 | 已写入的当前值 | 给用户的首轮建议（未应用） |
| --- | --- | --- |
| MOTION_AC_CRUISE_CPS / MOTION_BD_CRUISE_CPS | 900 | 1500 |
| MOTION_CB_CRUISE_CPS / MOTION_DA_CRUISE_CPS | 900 | 1200 |
| MOTION_CB_CAPTURE_CPS / MOTION_DA_CAPTURE_CPS | 600 | 700 |
| MOTION_TURN_RATE_MAX_DPS | 50 | 75 |
| MOTION_TURN_ANGLE_KP | 2 | 3 |
| MOTION_TURN_RATE_FF | 16 | 轮距约 120～130 mm 时可先试约 8 |
| MOTION_LINE_PERIOD_MS | 50 | 提高半圆速度时试 20 |

转向前馈近似公式：`轮距 mm / 2 × 编码器 counts/mm × π/180`，单位 cps/(deg/s)。不能把 FF 数值越大等同于越准确、更快。

本批新增参数已写入：

| 参数 | 值 |
| --- | --- |
| MOTION_TURN_APPROACH_DEG | 12° |
| MOTION_TURN_APPROACH_MIN_CPS | 120 cps |
| MOTION_TURN_BRAKE_LEAD_S | 0.04 s |
| MOTION_CAPTURE_HEADING_KP | 10 cps/deg |
| MOTION_CAPTURE_HEADING_MAX_CPS | 350 cps |
| MOTION_CAPTURE_HEADING_ACCEPT_DEG | 20° |
| MOTION_CAPTURE_UNCERTAIN_MS | 350 ms |
| MISSION_POINT_SIGNAL_MS | 300 ms |

保留参数：直线 C/D 确认窗口 1000～1550 mm，半圆 B/A 窗口 1000～1500 mm；到点和捕获均要求 3 个新帧；转向原粗调最低 350 cps、最高 1200 cps、验收 3°、静止确认 500 ms、最多 3 次纠偏。

调整顺序建议：先在原速度确认 C 能交接；再直行速度；再转向参数组；最后半圆/捕获速度与巡线周期。不要同时改计距标定、传感器映射和整组控制增益而丢失因果。

灰度条安装在车轴前方，看到终点黑线时车轴可能尚未到几何终点。若第一次白→黑发生在 1000 mm 以前，现检测器会拒绝本次候选；应记录事件位置后调整 `MOTION_AC_POINT_MIN_MM`，不要仅按 1281 mm 对角线推断识别位置。

## 7. 构建、测试、烧录与只读诊断命令

### 7.1 在用户本机 PowerShell 构建/测试

```powershell
Set-Location 'D:\STM32project\car-for2'
.\scripts\build_mission.ps1 -Mode FULL
.\scripts\test_mission.ps1
```

若工具路径环境变量不一致，使用已确认的绝对路径：

```powershell
.\scripts\build_mission.ps1 -Mode FULL -ArmBin 'C:\Users\21656\AppData\Local\stm32cube\bundles\gnu-tools-for-stm32\14.3.1+st.2\bin' -Ninja 'C:\Users\21656\AppData\Local\stm32cube\bundles\ninja\1.13.2+st.1\bin\ninja.exe'
```

测试脚本默认 `C:/MinGW/bin/gcc.exe` 和 `D:/python311/Scripts/ninja.exe`。参数可通过 `-Compiler`、`-Ninja` 覆盖。

当前有 12 个 CTest 项：mission_control_test、mission_readiness_0～4、mission_runtime_test、motion_algorithms_test、mission_actual_route_test、imu_heading_test、encoder_wrap_test、mission_reader_test。扩展用例在原 motion_algorithms_test 中，因此总项数未必增加。

### 7.2 烧录完整 FULL 固件

先确认刚刚完整构建成功，并核对 CMakeCache 中 `MISSION_MODE:STRING=FULL`。不要烧 `build/Debug` 或 `build/Mission-Debug-DA` 的旧文件。

```powershell
& 'C:\Users\21656\AppData\Local\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe' -c port=SWD mode=NORMAL reset=SWrst freq=100 -d 'D:\STM32project\car-for2\build\Mission-Debug\C10B_Test.elf' -v -rst
```

用户已证明可成功的参数：NORMAL、软件复位、100 kHz。成功证据应包含 `File download complete`、`Download verified successfully` 和 `MCU Reset`。

历史 HOTPLUG/4000 kHz 烧录在擦除页时失败；NORMAL/100 kHz 后曾写入成功。不能仅凭这段历史断言唯一原因就是时钟速度或 HOTPLUG。

- 用户已经读过 Option Bytes：RDP Disabled；WRP0～31 全部未启用写保护。没有解除保护的依据。
- 不要直接使用全片擦除或修改 RDP/WRP 来“碰碰运气”；全片擦除还会清除末页灰度学习数据。
- 若确需 UR/硬件复位模式，先确认 ST-LINK 的 NRST 已连接。不要假设只有 SWDIO/SWCLK/GND 时也能实现硬件复位。
- ST-LINK 显示 3.25 V 是目标板检测电压，不等于已经直接测量过 IMU 芯片供电。

### 7.3 C 点故障后先读诊断，再复位/重启

```powershell
.\scripts\read_mission.ps1 -AsJson
```

工具路径不对时：

```powershell
.\scripts\read_mission.ps1 -AsJson -ProgrammerPath 'C:\Users\21656\AppData\Local\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe'
```

这是 map 自动解析地址的 HOTPLUG 只读操作，不烧录、不复位、不启动电机。map 必须与板上固件匹配。读到奇数或首尾不相等的序号，说明采样跨更新，重新读即可。

历史 FULL map 的地址曾为：imu_diag=0x20002058、imu_raw_diag=0x200020D8、mission_diag=0x200023AC。**这些只是旧 map 的例子，重编译后必须重新查，不得硬编码为永久地址。**

```powershell
Select-String -Path '.\build\Mission-Debug\C10B_Test.map' -Pattern '\s(imu_diag|imu_raw_diag|mission_diag)\s*$'
```

如果确要原始内存读取，使用刚查到的对应地址，imu_diag 长度 0x80，mission_diag 长度 0x90：

```powershell
# 用实际 map 的地址替换下面示例地址。
& 'C:\Users\21656\AppData\Local\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe' -c port=SWD mode=HOTPLUG -r32 0x20002058 0x80
```

## 8. 诊断记录、OLED 和历史实测数据

### 8.1 OLED 最新布局

- 第 1 行：MISSION 与 FULL/单段模式。
- 第 2 行：IDLE/RUNNING/DONE/STOPPED/REJECTED/FAULT 与 AC/CB/BD/DA。
- 第 3 行：具体原因；段故障不再只显示 SEGMENT FAULT，而会显示 DISTANCE、HEADING、CAPTURE TIME/DIST、NO LINE 等。
- 第 4 行：阶段与分段 MM。
- 第 5 行：识别距离窗口 WIN。
- 第 6 行：目标方向 DEG 与航向误差 ERR。
- 第 7 行：P=最近到点；N=到点数；M=十进制黑线通道位图；C=到点确认帧数。
- 第 8 行：启动、停止、校准提示。

| FULL 模式下看到的结果 | 能证明什么 |
| --- | --- |
| P:-、N:0 | 本次运行还没有确认 C，不能把物理到达直接等同于软件完成 AC |
| P:C、N:1 | 已确认 C；若停下，再查当前 CB 的 fault/phase |
| State=DONE、Mode=AC | 这是单段模式完成，不是 FULL 后续代码未实现 |
| Segment=AC、DISTANCE | 通常是到距离上界仍未确认 C；核对 mask、校准、首次遇线距离 |
| Segment=CB、CAPTURE TIME/DIST | 已交接，但未在捕获边界内满足窄线/中心/方向要求 |
| NO LINE / LINE AMBIGUOUS | 区分捕获阶段和 FOLLOW 阶段的不同搜索时限 |

### 8.2 历史 IMU 读取事实与解释边界

旧 ±250 deg/s 配置下，用户多次读取：

- address=0x68，WHO_AM_I=0x68，9 项配置回读匹配，通信错误计数 0。
- Z 原始角速度约 -3040，对应旧换算约 -23.2 deg/s。
- X 加速度长期等于 -32768；慢速/单独读取也复现。
- 状态为 CALIBRATING，有效校准计数 0，每收到一帧就重启校准；reject_reason=1。
- 旧校准要求各轴原始角速度约 ±5 deg/s 内，还检查 X/Y 加速度接近 0、Z 接近 1g，所以无法通过。
- 参考 car-02- 的同车记录已有约 -23 deg/s 静止零偏与 Z 通道校准方案；因此移植其参考中心、实测零偏和 Z-only 处理。

这些事实不等于已经确认六轴完全健康。最新量程下必须用 65.5 换算；不要继续用 -3040/131 的旧计算解释新 raw。

### 8.3 mission_diag V2

36 个 32-bit word，共 144 B；magic=0x4D534E31，version=2；首尾序号相等且为偶数才接受。

| word 索引 | 字段 | 类型 |
| --- | --- | --- |
| 0 | `magic` | uint32_t |
| 1 | `version` | uint32_t |
| 2 | `sequence_begin` | uint32_t |
| 3 | `mode` | uint32_t |
| 4 | `state` | uint32_t |
| 5 | `reason` | uint32_t |
| 6 | `segment` | uint32_t |
| 7 | `segment_fault` | uint32_t |
| 8 | `uptime_ms` | uint32_t |
| 9 | `elapsed_ms` | uint32_t |
| 10 | `arrivals` | uint32_t |
| 11 | `last_point` | uint32_t |
| 12 | `last_event_ms` | uint32_t |
| 13 | `event_count` | uint32_t |
| 14 | `left_target_cps` | int32_t |
| 15 | `right_target_cps` | int32_t |
| 16 | `speed_fault` | uint32_t |
| 17 | `gyro_ready` | uint32_t |
| 18 | `gyro_age_ms` | uint32_t |
| 19 | `line_calibrated` | uint32_t |
| 20 | `line_age_ms` | uint32_t |
| 21 | `line_mask` | uint32_t |
| 22 | `calibrating` | uint32_t |
| 23 | `phase` | uint32_t |
| 24 | `motion_mode` | uint32_t |
| 25 | `distance_mm` | int32_t |
| 26 | `point_min_mm` | int32_t |
| 27 | `point_max_mm` | int32_t |
| 28 | `target_yaw_mdeg` | int32_t |
| 29 | `heading_error_mdeg` | int32_t |
| 30 | `line_position` | int32_t |
| 31 | `point_confirm_frames` | uint32_t |
| 32 | `turn_corrections` | uint32_t |
| 33 | `nominal_mm` | int32_t |
| 34 | `capture_frames` | uint32_t |
| 35 | `sequence_end` | uint32_t |

单位：时间为 ms；target_yaw_mdeg/heading_error_mdeg 为千分之一度；距离为 mm；轮速为 cps；line_position 为探头位置单位。

关键枚举（以当前头文件为准）：

- mode：0 FULL，1 AC，2 CB，3 BD，4 DA。
- state：0 IDLE，1 RUNNING，2 DONE，3 STOPPED，4 REJECTED，5 FAULT。
- segment：0 AC，1 CB，2 BD，3 DA。
- point：0 NONE，1 A，2 B，3 C，4 D。
- phase：0 IDLE，1 STILL，2 TURN，3 SETTLE，4 STRAIGHT，5 CAPTURE，6 FOLLOW，7 COMPLETE，8 FAULT。
- motion mode：0 FORWARD，1 TURN，2 BRAKE。
- reason：0 READY，1 NOT_IMPLEMENTED，2 USER_STOP，3 BAD_CONFIG，4 BAD_TARGET，5 SEGMENT_FAULT，6 GYRO_UNAVAILABLE，7 LINE_UNAVAILABLE，8 ENCODER_STALE，9 SPEED_FAULT，10 CALIBRATING。
- segment fault：0 NONE，1 CONFIG，2 DISTANCE，3 TIMEOUT，4 TURN_TIMEOUT，5 TURN_DIRECTION，6 TURN_TRANSLATION，7 GYRO_RESPONSE，8 WHEEL_SYNC，9 HEADING，10 CAPTURE，11 NO_LINE，12 AMBIGUOUS。

`Arrivals` 每次任务重置；`Event_count` 为本次上电累计事件数。读取脚本已经按有符号数解释负航向/轮速。

### 8.4 imu_diag V2

32 个 32-bit word，共 128 B；magic=0x494D5531，version=2；旧版 version=1 不能按新版量程和拒绝原因直接解码。

| word 索引 | 含义 |
| --- | --- |
| 0 | magic |
| 1 | version |
| 2 | sequence_begin |
| 3 | state |
| 4 | fault |
| 5 | address |
| 6 | WHO_AM_I |
| 7 | sample_count |
| 8 | last_frame_ms |
| 9 | now_ms |
| 10 | sample_age_ms |
| 11 | gyro_x_raw（周期诊断） |
| 12 | gyro_y_raw（周期诊断） |
| 13 | gyro_z_raw（新控制样本） |
| 14 | accel_x_raw |
| 15 | accel_y_raw |
| 16 | accel_z_raw |
| 17 | bias_millideg_per_s |
| 18 | yaw_millideg |
| 19 | rate_millideg_per_s |
| 20 | calibration_samples |
| 21 | calibration_restarts |
| 22 | bus_errors |
| 23 | max_update_us |
| 24 | last_bus_fault |
| 25 | data_not_ready_polls |
| 26 | gyro_config（0x08） |
| 27 | sample_divider（0） |
| 28 | dlpf（3） |
| 29 | zero_count |
| 30 | calibration_reject_reason |
| 31 | sequence_end |

- IMU state：0 BOOT，1 RESET_WAIT（新版用作非阻塞配置阶段），2 SETTLE，3 CALIBRATING，4 READY，5 FAULT，6 SIGNAL_RESET_WAIT（兼容旧枚举，新流程不用）。
- IMU fault：0 OK，1 NO_ACK，2 BAD_ID，3 BUS_ERROR，4 CONFIG_ERROR，5 SAMPLE_GAP，6 GYRO_SATURATED，7 CLOCK_ERROR，8 BUS_STUCK。
- 新校准 reject_reason：1 参考窗口/噪声不通过，2 编码器未静止/过期或驱动仍在运行，3 样本时间间隔过大。
- 有符号原始量需按 int32 解码；gyro raw 再除以 65.5，accel 配置仍为 ±2g，raw 除以 16384。

### 8.5 imu_raw_diag V3

64 个 word，共 256 B；magic=0x49525731，version=3，首尾序号在 word 2/63。

- word 3：交叉读取轮数；4：发布时间；5：Z 控制样本时间。
- word 6/7/8：慢速整块、单独 AX、单独 GZ 的诊断读取时间。
- word 9：交叉通信错误；10：旧信号路径复位标志，新版保持 0。
- word 11～25：15 个 normal_bytes；其中索引 0/13/14 是控制状态与 Z，其余来自周期诊断。
- word 26～39：慢速 14 字节数据；40/41：AX 字节；42/43：GZ 字节。
- word 44～52：0x6B、0x6C、0x19、0x1A、0x1B、0x1C、0x23、0x6A、0x38 的回读记录。
- word 53：交叉读取阶段；54：完成的配置项数；55/56：慢/普通时序值 10/5 us。
- word 57：启动尝试；58：总线恢复次数；59：恢复时钟数；60：引脚电平（1=SCL高，2=SDA高）；61：恢复成功；62：启动失败数。

**X/Y 加速度、X/Y 角速度由约 250 ms 周期诊断更新，不能把整个 normal_bytes 当作同时刻完整六轴样本。Z 控制通道以单独的新帧读数为准。**

## 9. 待解决问题与建议接手顺序

1. 先确认本机工作树和本文件快照是否一致，检查用户是否已经自行修改参数，不覆盖用户之后的修改。
2. 本机重新构建 FULL、运行全部主机测试；先解决新回归失败。必须区分工具环境无法运行与源代码编译失败。
3. 用 NORMAL/100 kHz 烧录新 FULL，确认校验成功；不要从存在旧文件就推断新版已烧入。
4. 原速度进行一次试跑，在 C 点停下后不复位、不重新启动，记录 OLED 或 `read_mission -AsJson`。
5. 若 C 未识别：看当前 mask、灰度学习、新鲜度、首次白→黑分段距离、窗口及 3 个新帧是否足够；检查灰度条前伸量、左右顺序和探头高度。
6. 若 C 已识别：看 CB CAPTURE 的方向误差、捕获帧、窄/宽黑带、丢线状态和时间/距离边界。新增辅助仍需实车调试，不保证参数一次合适。
7. 确认交接后再按用户选择分组提高速度；观察实际编码器速度，而不只看目标 cps。
8. 转向记录目标/实际角速度、末段角误差、左右编码器和 corrections；必要时按轮距重新估算 TURN_RATE_FF，别只增大速度或放宽角度验收。
9. 最后验证 C/D 连续运行、B 制动转向、A 制动、四次声光事件、不重复计点，以及运行中按键停止。

仍未知/不能凭空确认：最新 C 故障原因、轮距和 counts/mm 的当前实测精度、探头前伸量、修改后的动态惯性与刹车距离、蜂鸣器实际工作情况、最新版完整路线成功率。

## 10. 不应破坏的约束

- 保持用户选择 FULL；单段测试必须明确标注并在结束后恢复 FULL。
- 不清零段间 IMU，不伪造 ready，不将读到 ACK 作为运动传感器就绪。
- 不把时间/名义距离直接当作 C/B/D/A 到点事件，不跳过新鲜灰度确认。
- 不为了消除提示而直接禁用 gyro/line/encoder/speed 故障保护。
- 不把旧版 131 灵敏度带回新 ±500 量程。
- 不在 C/D 交接重置速度 PI、编码器累计或变成停车再启动。
- 不覆盖用户独立的 IDE/编辑器改动。补丁未包含 .clangd/c_cpp_properties 等环境调整。
- 不在硬件未知时随意改变 PA13/PA14 SWD、UART4 所有权或向 IMU 芯片直供未经确认的电压。
- 不把 X 加速度饱和“忽略用于偏航校准”描述成已经修好该轴。
- 不将本次 ARM 对象编译检查、此前 12/12 主机通过或理想反馈全程测试描述成最新实车验收通过。

## 11. 当前参数头文件快照

以下三份是生成本文时的原文，便于下一位 AI 快速查找，不代表要求重新覆盖当前文件。

### Core/Inc/motion_tuning.h

```c
#ifndef MOTION_TUNING_H
#define MOTION_TUNING_H

/* All physical defaults below are FIRST-TRIAL values, not new car calibration.
 * cps = normalized encoder counts/second; mm = ground distance; deg = degrees.
 * Higher gains/speeds can cause oscillation or missed lines: change one group
 * at a time and use the distance/heading/phase diagnostics to check the result. */

/* 1. Wheel calibration: increase counts/mm to reduce displayed travel. */
#define MOTION_LEFT_COUNTS_PER_MM  6.730141612f
#define MOTION_RIGHT_COUNTS_PER_MM 6.730141612f
#define MOTION_ENCODER_PERIOD_MS 10U

/* 2. Actuator limits and speed PI. Feed-forward remains the measured old fit. */
#define MOTION_MAX_CPS 3200U
#define MOTION_SPEED_KP 0.010f
#define MOTION_SPEED_KI 0.030f
#define MOTION_PWM_LIMIT 65.0f
#define MOTION_SPEED_INTEGRAL_LIMIT 20.0f
#define MOTION_TARGET_RAMP_CPS_S 5000.0f

/* 3. Gyro convention: +1 means raw positive yaw is a physical left turn.
 * The course reference is east/right = 0, north/up = +90, west/left = 180.
 * Probe order lists raw indices from VEHICLE LEFT to VEHICLE RIGHT. */
#define MOTION_YAW_SIGN 1.0f
#define MOTION_LINE_ORDER {0U,1U,2U,3U,4U,5U,6U,7U}

/* 4. Per-segment controls. AB=1000, AD=BC=800, radius=400 mm.
 * Angles are course headings, not an increment from a possibly wrong arrival.
 * Identification windows are inclusive. Nominal lengths do NOT end a segment. */
#define MOTION_AC_TARGET_DEG (-38.66f)
#define MOTION_BD_TARGET_DEG (180.0f + 38.66f)
#define MOTION_AC_CRUISE_CPS 900.0f
#define MOTION_BD_CRUISE_CPS 900.0f
#define MOTION_CB_CRUISE_CPS 900.0f
#define MOTION_DA_CRUISE_CPS 900.0f
#define MOTION_CB_CAPTURE_CPS 600.0f
#define MOTION_DA_CAPTURE_CPS 600.0f
#define MOTION_AC_NOMINAL_MM 1281.0f
#define MOTION_BD_NOMINAL_MM 1281.0f
#define MOTION_CB_NOMINAL_MM 1257.0f
#define MOTION_DA_NOMINAL_MM 1257.0f
#define MOTION_AC_POINT_MIN_MM 1000.0f
#define MOTION_AC_POINT_MAX_MM 1550.0f
#define MOTION_BD_POINT_MIN_MM 1000.0f
#define MOTION_BD_POINT_MAX_MM 1550.0f
#define MOTION_CB_POINT_MIN_MM 1000.0f
#define MOTION_CB_POINT_MAX_MM 1500.0f
#define MOTION_DA_POINT_MIN_MM 1000.0f
#define MOTION_DA_POINT_MAX_MM 1500.0f
#define MOTION_DRIVE_TIMEOUT_MS 30000U
#define MOTION_ARC_TIMEOUT_MS 30000U

/* 5. Heading PD: stronger Kp corrects angle sooner; Kd damps yaw rate. */
#define MOTION_HEADING_KP 10.0f
#define MOTION_HEADING_KD 3.0f
#define MOTION_HEADING_TRIM_MAX_CPS 150.0f
#define MOTION_HEADING_PERIOD_MS 20U
#define MOTION_HEADING_FAULT_DEG 15.0f

/* 6. In-place turn: angle -> desired angular rate -> opposite wheel targets. */
#define MOTION_TURN_ANGLE_KP 2.0f
#define MOTION_TURN_RATE_FF 16.0f
#define MOTION_TURN_RATE_KP 6.0f
#define MOTION_TURN_RATE_MAX_DPS 50.0f
#define MOTION_TURN_MIN_CPS 350.0f
#define MOTION_TURN_MAX_CPS 1200U
/* Fine approach is separate from the coarse stiction/minimum-speed floor.
 * Existing cruise/rate/gain values above are left for the operator to tune. */
#define MOTION_TURN_APPROACH_DEG 12.0f
#define MOTION_TURN_APPROACH_MIN_CPS 120.0f
#define MOTION_TURN_BRAKE_LEAD_S 0.04f
#define MOTION_TURN_BRAKE_DEG 2.0f
#define MOTION_TURN_ACCEPT_DEG 3.0f
#define MOTION_STILL_RATE_DPS 3.0f
#define MOTION_STILL_WHEEL_CPS 60.0f
#define MOTION_SETTLE_MS 500U
#define MOTION_TURN_MAX_CORRECTIONS 3U
#define MOTION_TURN_TIMEOUT_MS 25000U
#define MOTION_TURN_TRANSLATION_MAX_MM 80.0f
#define MOTION_TURN_WHEEL_MISMATCH_MM 20.0f
#define MOTION_TURN_GYRO_RESPONSE_MM 40.0f
#define MOTION_TURN_RESPONSE_GRACE_MS 500U
#define MOTION_TURN_DIRECTION_GRACE_MS 300U
#define MOTION_TURN_MIN_RESPONSE_DEG 1.0f
#define MOTION_TURN_WRONG_DIRECTION_DEG 5.0f
#define MOTION_TURN_FOLLOWING_MIN_FRACTION 0.15f

/* 7. Old line-position PD, in probe-position units (-3500 left, +3500 right).
 * Preserve the old speed shaping and clear-group selection. */
#define MOTION_LINE_KP 0.18f
#define MOTION_LINE_KD 0.006f
#define MOTION_LINE_D_FILTER_S 0.060f
#define MOTION_LINE_PERIOD_MS 50U
#define MOTION_LINE_STEERING_MAX_CPS 650.0f
#define MOTION_LINE_CORNER_BASE_CPS 300.0f
#define MOTION_LINE_MIN_WHEEL_CPS 150.0f
#define MOTION_LINE_HISTORY_MAX 1000.0f
#define MOTION_LINE_GROUP_MARGIN 1500.0f
#define MOTION_LINE_POSITION_SLOWDOWN 0.10f
#define MOTION_LINE_RECOVERY_FRACTION 0.75f
#define MOTION_LINE_HIGH_SPEED_THRESHOLD_CPS 900.0f
#define MOTION_LINE_HIGH_BASE_FLOOR_CPS 900.0f
#define MOTION_LINE_HIGH_WHEEL_FLOOR_CPS 450.0f

/* 8. Capture and point confirmation: count NEW frames, not main-loop polls. */
#define MOTION_POINT_CONFIRM_FRAMES 3U
#define MOTION_CAPTURE_CONFIRM_FRAMES 3U
#define MOTION_CAPTURE_CENTER_LIMIT 1000.0f
#define MOTION_CAPTURE_TIMEOUT_MS 1500U
#define MOTION_CAPTURE_MAX_MM 200.0f
/* Diagonal -> semicircle: align with the entry tangent while driving forward.
 * Wide single stripes are accepted only during this bounded capture phase. */
#define MOTION_CAPTURE_HEADING_KP 10.0f
#define MOTION_CAPTURE_HEADING_MAX_CPS 350.0f
#define MOTION_CAPTURE_HEADING_ACCEPT_DEG 20.0f
#define MOTION_CAPTURE_UNCERTAIN_MS 350U
#define MOTION_LINE_UNCERTAIN_MS 100U
#define MOTION_REVERSE_DISTANCE_FAULT_MM 30.0f
#endif
```

### Core/Src/Modules/imu_config.h

```c
#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

/* MPU6050 settings ported from car-02-/Core/Src/mpu6050.c. */
#define IMU_GYRO_CONFIG 0x08U /* FS_SEL=1, +/-500 degrees/second */
#define IMU_GYRO_LSB_PER_DPS 65.5f
#define IMU_GYRO_MAX_ABS_DPS 490.0f
#define IMU_SAMPLE_DIVIDER 0U /* DLPF=3: 1 kHz sensor, 5 ms host polling */
#define IMU_DLPF_CONFIG 3U
#define IMU_POLL_MS 5U
#define IMU_MAX_SAMPLE_GAP_MS 30U

/* Installed sensor's measured stationary reference, from car-02-.
 * This ONLY centers the calibration gate. Each boot measures its own bias.
 * Re-measure this reference after replacing the sensor. Raw sensor sign is
 * retained here; MOTION_YAW_SIGN is applied once by the mission motion controller. */
#define IMU_Z_BIAS_PRIOR_DPS (-23.0f)
#define IMU_CAL_WINDOW_DPS 10.0f
#define IMU_CAL_SAMPLES 400U
#define IMU_CAL_MIN_MS ((IMU_CAL_SAMPLES - 1U) * IMU_POLL_MS)
#define IMU_CAL_STD_DPS 0.8f

#endif
```

### Core/Src/Mission/mission_config.h

```c
#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H
#include "motion_tuning.h"

/* CMake -DMISSION_MODE=FULL|AC|CB|BD|DA selects one firmware. */
#define MISSION_MODE_FULL 0
#define MISSION_MODE_AC   1
#define MISSION_MODE_CB   2
#define MISSION_MODE_BD   3
#define MISSION_MODE_DA   4
#ifndef MISSION_BUILD_MODE
#define MISSION_BUILD_MODE MISSION_MODE_FULL
#endif
#if MISSION_BUILD_MODE < MISSION_MODE_FULL || MISSION_BUILD_MODE > MISSION_MODE_DA
#error Invalid MISSION_BUILD_MODE
#endif

#define MISSION_MAX_TARGET_CPS ((float)MOTION_MAX_CPS)
#define MISSION_GYRO_MAX_AGE_MS 30U
#define MISSION_LINE_MAX_AGE_MS 150U
#define MISSION_CAL_HOLD_MS 6000U
/* C10B/car-02-: PA15 active-high buzzer, PC2 active-low LED. */
#define MISSION_POINT_SIGNAL_MS 300U
#define MISSION_BUZZER_ACTIVE_HIGH 1
#endif
```



## 12. 接口和到点识别参考源码

这些是理解补丁所需的关键未修改接口/调度代码。实际执行仍以工程最新文件为准。

### Core/Src/Mission/mission_segment.h

```c
#ifndef MISSION_SEGMENT_H
#define MISSION_SEGMENT_H
#include <stdint.h>

typedef enum { SEGMENT_AC, SEGMENT_CB, SEGMENT_BD, SEGMENT_DA, SEGMENT_COUNT } MissionSegmentId;
typedef enum { POINT_NONE, POINT_A, POINT_B, POINT_C, POINT_D } MissionPoint;
typedef enum { SEG_RUNNING, SEG_COMPLETE, SEG_FAULT, SEG_UNIMPLEMENTED } MissionSegmentStatus;
typedef enum { EXIT_COMPLETE, EXIT_STOP, EXIT_FAULT } MissionExitReason;
enum { SENSOR_GYRO = 1U, SENSOR_LINE = 2U };
typedef enum { MOTION_FORWARD, MOTION_TURN, MOTION_BRAKE } MissionMotionMode;
typedef enum { PHASE_IDLE, PHASE_STILL, PHASE_TURN, PHASE_SETTLE, PHASE_STRAIGHT,
               PHASE_CAPTURE, PHASE_FOLLOW, PHASE_COMPLETE, PHASE_FAULT } MissionPhase;
enum { MOTION_KIND_TEST, MOTION_KIND_STRAIGHT, MOTION_KIND_ARC };
enum { MOTION_FAULT_NONE, MOTION_FAULT_CONFIG, MOTION_FAULT_DISTANCE,
       MOTION_FAULT_TIMEOUT, MOTION_FAULT_TURN_TIMEOUT, MOTION_FAULT_TURN_DIRECTION,
       MOTION_FAULT_TURN_TRANSLATION, MOTION_FAULT_GYRO_RESPONSE,
       MOTION_FAULT_WHEEL_SYNC, MOTION_FAULT_HEADING, MOTION_FAULT_CAPTURE,
       MOTION_FAULT_NO_LINE, MOTION_FAULT_AMBIGUOUS };

/* One immutable foreground snapshot, shared by outgoing/incoming segments.
 * Wheel position is normalized encoder counts; speed is counts/second.
 * Yaw/rate are degrees/degrees per second, never reset at a handoff. */
typedef struct {
    uint32_t now_ms, encoder_sequence, encoder_age_ms;
    int64_t left_counts, right_counts;
    float left_cps, right_cps, yaw_deg, yaw_rate_dps;
    uint32_t gyro_ready, gyro_fault, gyro_age_ms;
    uint32_t line_sequence, line_age_ms, line_calibrated, line_mask;
    uint32_t line_strength[8];
    uint32_t calibrating, speed_fault;
} MissionInput;

typedef struct { int64_t left_origin, right_origin; uint32_t active; } MissionDistance;
typedef struct {
    uint32_t last_sequence, run_frames, candidate_frames;
    uint8_t have_sequence, run_known, run_black, stable_known, stable_black, candidate;
} MissionEdge;
typedef struct {
    MissionDistance distance, turn_distance;
    MissionEdge edge;
    float distance_mm, target_yaw_deg, heading_error_deg;
    float line_position, last_position, derivative;
    float left_target, right_target, recovery_left, recovery_right;
    float turn_origin_yaw, turn_start_error;
    uint32_t turn_started_ms, control_ms, line_sequence, have_line_sequence;
    uint32_t still_since_ms, still_tracking, corrections, uncertain, uncertain_ms;
    uint32_t capture_frames, capture_sequence, have_capture_sequence, have_position;
} MissionMotionState;
typedef struct {
    MissionInput entry;
    uint32_t active, phase, phase_started_ms;
    float course_origin_deg;
    uint32_t from_handoff;
    MissionMotionState motion;
} MissionSegmentContext;

typedef struct {
    float cruise_cps;
    uint32_t timeout_ms;
    uint32_t kind;
    float capture_cps, target_yaw_deg, nominal_mm, point_min_mm, point_max_mm;
} MissionSegmentParams;
typedef struct {
    MissionSegmentStatus status;
    uint32_t fault;
    float left_cps, right_cps;
    MissionMotionMode mode;
} MissionSegmentOutput;

typedef struct {
    MissionSegmentId id;
    uint32_t implemented, required_sensors;
    const MissionSegmentParams *params;
    MissionSegmentOutput (*Enter)(MissionSegmentContext *, const MissionInput *, const MissionSegmentParams *);
    MissionSegmentOutput (*Update)(MissionSegmentContext *, const MissionInput *, const MissionSegmentParams *);
    /* Cleanup only: no motor, encoder, IMU or other hardware calls. */
    void (*Exit)(MissionSegmentContext *, const MissionInput *, MissionExitReason);
    uint8_t (*Validate)(const MissionSegmentParams *);
} MissionSegment;

extern const MissionSegment mission_segment_ac, mission_segment_cb;
extern const MissionSegment mission_segment_bd, mission_segment_da;
#endif
```

### Core/Src/Mission/mission.h

```c
#ifndef MISSION_H
#define MISSION_H
#include "mission_control.h"

/* Foreground API, after Cube GPIO/TIM/UART initialization. Init leaves drive
 * braked; poll Update continuously. Start returns 0 if the route is not ready. */
void Mission_Init(uint32_t now_ms);
void Mission_Update(uint32_t now_ms);
uint8_t Mission_Start(uint32_t now_ms);
void Mission_Stop(uint32_t now_ms);

/* V2 diagnostic ABI (144 bytes). Old task2/mode2/line-follow layouts are retired.
 * Read while stopped or accept only equal even sequence_begin/sequence_end
 * across a read. Enumerations are defined in mission_control/segment.h. */
typedef struct {
    uint32_t magic, version, sequence_begin, mode;
    uint32_t state, reason, segment, segment_fault;
    uint32_t uptime_ms, elapsed_ms, arrivals, last_point;
    uint32_t last_event_ms, event_count;
    int32_t left_target_cps, right_target_cps;
    uint32_t speed_fault, gyro_ready, gyro_age_ms;
    uint32_t line_calibrated, line_age_ms, line_mask;
    uint32_t calibrating, phase, motion_mode;
    int32_t distance_mm, point_min_mm, point_max_mm, target_yaw_mdeg, heading_error_mdeg;
    int32_t line_position;
    uint32_t point_confirm_frames, turn_corrections;
    int32_t nominal_mm;
    uint32_t capture_frames, sequence_end;
} MissionDiagnostics;
extern volatile MissionDiagnostics mission_diag;
#endif
```

### Core/Src/Mission/mission_control.c

```c
#include "mission_control.h"
#include "motion_control.h"
#include <math.h>
#include <stddef.h>

static const MissionPoint endpoints[SEGMENT_COUNT] = { POINT_C, POINT_B, POINT_D, POINT_A };
static MissionOutput Output(MissionDriveAction action)
{
    return (MissionOutput){ .action = action, .mode = MOTION_BRAKE };
}
static uint8_t ValidDescriptor(const MissionSegment *s, uint32_t id)
{
    return s && s->id == id && s->params && s->Enter && s->Update && s->Exit &&
        !(s->required_sensors & ~(SENSOR_GYRO | SENSOR_LINE));
}
static MissionReason CheckInput(const MissionSegment *s, const MissionInput *in)
{
    if (in->calibrating) return MISSION_CALIBRATING;
    if (in->speed_fault) return MISSION_SPEED_FAULT;
    if (in->encoder_age_ms > 100U) return MISSION_ENCODER_STALE;
    if ((s->required_sensors & SENSOR_GYRO) && (!in->gyro_ready || in->gyro_fault ||
        in->gyro_age_ms > MISSION_GYRO_MAX_AGE_MS || !isfinite(in->yaw_deg) || !isfinite(in->yaw_rate_dps)))
        return MISSION_GYRO_UNAVAILABLE;
    if ((s->required_sensors & SENSOR_LINE) && (!in->line_calibrated || !in->line_sequence ||
        in->line_age_ms > MISSION_LINE_MAX_AGE_MS)) return MISSION_LINE_UNAVAILABLE;
    return MISSION_OK;
}
static uint8_t ValidTarget(MissionSegmentOutput out)
{
    if (out.mode==MOTION_BRAKE) return out.left_cps==0.0f && out.right_cps==0.0f;
    if (out.mode==MOTION_TURN)
        return ((out.left_cps<0.0f && out.right_cps>0.0f) || (out.left_cps>0.0f && out.right_cps<0.0f)) &&
            fabsf(out.left_cps)<=MOTION_TURN_MAX_CPS && fabsf(out.right_cps)<=MOTION_TURN_MAX_CPS;
    return out.mode==MOTION_FORWARD && out.left_cps>0.0f && out.left_cps<=MISSION_MAX_TARGET_CPS &&
        out.right_cps>0.0f && out.right_cps<=MISSION_MAX_TARGET_CPS;
}
static void Exit(MissionControl *m, const MissionInput *in, MissionExitReason reason)
{
    MissionSegmentContext *ctx = &m->contexts[m->segment];
    if (ctx->active) {
        m->segments[m->segment]->Exit(ctx, in, reason);
        ctx->active = 0U;
    }
}
void MissionControl_Init(MissionControl *m, uint32_t mode, const MissionSegment *const segments[SEGMENT_COUNT])
{
    *m = (MissionControl){ .mode = mode, .segment = SEGMENT_AC };
    for (uint32_t i = 0; i < SEGMENT_COUNT; ++i) m->segments[i] = segments[i];
    if (mode >= MISSION_MODE_AC && mode <= MISSION_MODE_DA) m->segment = (MissionSegmentId)(mode - 1U);
}
MissionOutput MissionControl_Fail(MissionControl *m, const MissionInput *in, MissionReason reason, uint32_t detail)
{
    if (m->state == MISSION_RUNNING) Exit(m, in, EXIT_FAULT);
    m->state = MISSION_FAULT; m->reason = reason; m->segment_fault = detail; m->ended_ms = in->now_ms;
    return Output(DRIVE_STOP);
}
static MissionOutput Accept(MissionControl *m, const MissionInput *in, MissionSegmentOutput result, MissionDriveAction action)
{
    if (result.status == SEG_UNIMPLEMENTED)
        return MissionControl_Fail(m, in, MISSION_NOT_IMPLEMENTED, result.fault);
    if (result.status != SEG_RUNNING)
        return MissionControl_Fail(m, in, MISSION_SEGMENT_FAULT, result.fault);
    if (!ValidTarget(result)) return MissionControl_Fail(m, in, MISSION_BAD_TARGET, 0U);
    MissionOutput out = Output(action);
    out.left_cps = result.left_cps; out.right_cps = result.right_cps;
    out.mode = result.mode;
    return out;
}
static MissionSegmentOutput Enter(MissionControl *m, const MissionInput *in)
{
    const MissionSegment *s = m->segments[m->segment];
    MissionSegmentContext *ctx = &m->contexts[m->segment];
    *ctx = (MissionSegmentContext){ .entry = *in, .active = 1U, .phase_started_ms = in->now_ms,
        .course_origin_deg=m->course_origin_deg, .from_handoff=m->arrivals!=0U };
    return s->Enter(ctx, in, s->params);
}
MissionOutput MissionControl_Start(MissionControl *m, const MissionInput *in)
{
    if (m->state == MISSION_RUNNING) return Output(DRIVE_HOLD);
    m->reason = MISSION_OK; m->segment_fault = m->arrivals = 0U; m->last_point = POINT_NONE;
    m->started_ms = m->ended_ms = m->run_started = 0U;
    uint32_t first = m->mode == MISSION_MODE_FULL ? 0U : m->mode - 1U;
    uint32_t end = m->mode == MISSION_MODE_FULL ? SEGMENT_COUNT : first + 1U;
    if (m->mode > MISSION_MODE_DA || !Motion_ValidateTuning()) {
        m->state = MISSION_REJECTED; m->reason = MISSION_BAD_CONFIG; return Output(DRIVE_STOP);
    }
    m->segment = (MissionSegmentId)first;
    /* Check the WHOLE selected route before any Enter/drive operation. */
    for (uint32_t i = first; i < end; ++i) {
        if (!ValidDescriptor(m->segments[i], i)) {
            m->segment = (MissionSegmentId)i; m->state = MISSION_REJECTED;
            m->reason = MISSION_BAD_CONFIG; return Output(DRIVE_STOP);
        }
        if (!m->segments[i]->implemented) {
            m->segment = (MissionSegmentId)i; m->state = MISSION_REJECTED;
            m->reason = MISSION_NOT_IMPLEMENTED; return Output(DRIVE_STOP);
        }
        if (m->segments[i]->Validate && !m->segments[i]->Validate(m->segments[i]->params)) {
            m->segment=(MissionSegmentId)i; m->state=MISSION_REJECTED;
            m->reason=MISSION_BAD_CONFIG; return Output(DRIVE_STOP);
        }
    }
    MissionReason reason = CheckInput(m->segments[first], in);
    if (reason != MISSION_OK) {
        m->state = MISSION_REJECTED; m->reason = reason; return Output(DRIVE_STOP);
    }
    /* Standalone BD/DA face west; FULL/AC/CB face east. */
    if (m->segments[first]->params->kind==MOTION_KIND_ARC && !in->line_mask) {
        m->state=MISSION_REJECTED; m->reason=MISSION_LINE_UNAVAILABLE; return Output(DRIVE_STOP);
    }
    float initial_heading=(m->mode==MISSION_MODE_BD || m->mode==MISSION_MODE_DA) ? 180.0f : 0.0f;
    m->course_origin_deg=in->yaw_deg-initial_heading/MOTION_YAW_SIGN;
    m->state = MISSION_RUNNING; m->started_ms = in->now_ms; m->run_started = 1U;
    MissionOutput out = Accept(m, in, Enter(m, in), DRIVE_BEGIN);
    /* An unexpected Enter failure must not energize the motors either. */
    if (m->state == MISSION_FAULT) m->state = MISSION_REJECTED;
    return out;
}
MissionOutput MissionControl_Update(MissionControl *m, const MissionInput *in)
{
    if (m->state != MISSION_RUNNING) return Output(DRIVE_HOLD);
    const MissionSegment *s = m->segments[m->segment];
    MissionReason reason = CheckInput(s, in);
    if (reason != MISSION_OK) return MissionControl_Fail(m, in, reason, in->speed_fault);
    MissionSegmentOutput result = s->Update(&m->contexts[m->segment], in, s->params);
    if (result.status != SEG_COMPLETE) return Accept(m, in, result, DRIVE_TARGET);

    MissionPoint arrived = endpoints[m->segment];
    Exit(m, in, EXIT_COMPLETE);
    m->last_point = arrived; ++m->arrivals;
    MissionOutput out;
    if (m->mode != MISSION_MODE_FULL || m->segment == SEGMENT_DA) {
        m->state = MISSION_DONE; m->reason = MISSION_OK; m->ended_ms = in->now_ms;
        out = Output(DRIVE_STOP);
    } else {
        ++m->segment;
        reason = CheckInput(m->segments[m->segment], in);
        if (reason != MISSION_OK) out = MissionControl_Fail(m, in, reason, in->speed_fault);
        else out = Accept(m, in, Enter(m, in), DRIVE_TARGET);
    }
    /* Same snapshot: C/D retain forward drive; B explicitly requests the
     * approved stopped in-place-turn phase. */
    out.point = arrived;
    return out;
}
MissionOutput MissionControl_Stop(MissionControl *m, const MissionInput *in)
{
    if (m->state == MISSION_RUNNING) {
        Exit(m, in, EXIT_STOP);
        m->ended_ms = in->now_ms;
    } else if (!m->run_started) m->ended_ms = in->now_ms;
    m->state = MISSION_STOPPED; m->reason = MISSION_USER_STOP;
    return Output(DRIVE_STOP);
}
```

### Core/Src/Mission/motion_edge.c

```c
#include "motion_control.h"
#include "mission_config.h"

void MotionEdge_Reset(MissionEdge *e) { *e = (MissionEdge){0}; }
void MotionEdge_SeedBlack(MissionEdge *e, const MissionInput *in)
{
    *e = (MissionEdge){.last_sequence=in->line_sequence, .have_sequence=1U,
        .run_known=1U, .run_black=1U, .run_frames=MOTION_POINT_CONFIRM_FRAMES,
        .stable_known=1U, .stable_black=1U};
}
uint8_t MotionEdge_Update(MissionEdge *e, const MissionInput *in, float mm,
                         float minimum, float maximum, uint8_t target_black)
{
    uint8_t window = mm >= minimum && mm <= maximum;
    if (!window) { e->candidate=0U; e->candidate_frames=0U; }
    /* The caller also rejects stale sensors; this gate keeps the standalone
     * detector from treating a driver's stale mask=0 as white. */
    if (!in->line_calibrated || !in->line_sequence || in->calibrating ||
        in->line_age_ms > MISSION_LINE_MAX_AGE_MS) {
        e->candidate=0U; e->candidate_frames=0U; return 0U;
    }
    if (e->have_sequence && e->last_sequence == in->line_sequence) return 0U;
    e->have_sequence=1U; e->last_sequence=in->line_sequence;
    uint8_t black = in->line_mask != 0U;
    if (!e->run_known || e->run_black != black) {
        e->run_known=1U; e->run_black=black; e->run_frames=1U;
        /* Eligibility is decided on the FIRST raw frame of the transition.
         * Opening the window halfway through an old transition cannot arm it. */
        e->candidate=window && e->stable_known && e->stable_black != target_black && black==target_black;
    } else if (e->run_frames < MOTION_POINT_CONFIRM_FRAMES) ++e->run_frames;
    e->candidate_frames=e->candidate ? e->run_frames : 0U;
    if (e->run_frames < MOTION_POINT_CONFIRM_FRAMES) return 0U;
    uint8_t event = e->candidate && window && black==target_black;
    e->stable_known=1U; e->stable_black=black;
    e->candidate=0U;
    return event;
}
```



## 13. 当前修改的统一补丁

- 补丁基线是 Git HEAD `0d903bf5959cdabfc2fb9b7c9ca66b15a50b9499`。
- 覆盖本任务改动的源代码、测试、README；包括新增 imu_config.h 和 test_imu_heading.c。
- 当前工作树已经包含这些修改，**不要在当前工作树重复应用**。
- 如果恢复到另一份干净的同基线副本，可将下列 diff 保存为 patch，先 `git apply --check`，再决定应用；不要用 reset/覆盖命令丢弃其他人的改动。
- `.vscode/settings.json` 中只需核对 `stm32flash.buildDir` 为 `build/Mission-Debug`；未把整个用户编辑器配置打入补丁。

````diff
diff --git a/Core/Inc/motion_tuning.h b/Core/Inc/motion_tuning.h
index ef8e16a..40b7945 100644
--- a/Core/Inc/motion_tuning.h
+++ b/Core/Inc/motion_tuning.h
@@ -65,6 +65,11 @@
 #define MOTION_TURN_RATE_MAX_DPS 50.0f
 #define MOTION_TURN_MIN_CPS 350.0f
 #define MOTION_TURN_MAX_CPS 1200U
+/* Fine approach is separate from the coarse stiction/minimum-speed floor.
+ * Existing cruise/rate/gain values above are left for the operator to tune. */
+#define MOTION_TURN_APPROACH_DEG 12.0f
+#define MOTION_TURN_APPROACH_MIN_CPS 120.0f
+#define MOTION_TURN_BRAKE_LEAD_S 0.04f
 #define MOTION_TURN_BRAKE_DEG 2.0f
 #define MOTION_TURN_ACCEPT_DEG 3.0f
 #define MOTION_STILL_RATE_DPS 3.0f
@@ -104,6 +109,12 @@
 #define MOTION_CAPTURE_CENTER_LIMIT 1000.0f
 #define MOTION_CAPTURE_TIMEOUT_MS 1500U
 #define MOTION_CAPTURE_MAX_MM 200.0f
+/* Diagonal -> semicircle: align with the entry tangent while driving forward.
+ * Wide single stripes are accepted only during this bounded capture phase. */
+#define MOTION_CAPTURE_HEADING_KP 10.0f
+#define MOTION_CAPTURE_HEADING_MAX_CPS 350.0f
+#define MOTION_CAPTURE_HEADING_ACCEPT_DEG 20.0f
+#define MOTION_CAPTURE_UNCERTAIN_MS 350U
 #define MOTION_LINE_UNCERTAIN_MS 100U
 #define MOTION_REVERSE_DISTANCE_FAULT_MM 30.0f
 #endif
diff --git a/Core/Src/Mission/mission_config.h b/Core/Src/Mission/mission_config.h
index 22e8953..0187f46 100644
--- a/Core/Src/Mission/mission_config.h
+++ b/Core/Src/Mission/mission_config.h
@@ -19,4 +19,7 @@
 #define MISSION_GYRO_MAX_AGE_MS 30U
 #define MISSION_LINE_MAX_AGE_MS 150U
 #define MISSION_CAL_HOLD_MS 6000U
+/* C10B/car-02-: PA15 active-high buzzer, PC2 active-low LED. */
+#define MISSION_POINT_SIGNAL_MS 300U
+#define MISSION_BUZZER_ACTIVE_HIGH 1
 #endif
diff --git a/Core/Src/Mission/mission_port_stm32.c b/Core/Src/Mission/mission_port_stm32.c
index ebf1c5b..6281a4a 100644
--- a/Core/Src/Mission/mission_port_stm32.c
+++ b/Core/Src/Mission/mission_port_stm32.c
@@ -14,21 +14,39 @@ static uint32_t shown_phase;
 static uint32_t point_lit, point_ms;
 _Static_assert(SPEED_CONTROL_MAX_CPS == MOTION_MAX_CPS, "Review mission target limit");

+static void PointSignal(uint8_t on)
+{
+    HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,on ? GPIO_PIN_RESET : GPIO_PIN_SET);
+    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,
+        (on == (MISSION_BUZZER_ACTIVE_HIGH != 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
+}
 void MissionPort_Init(uint32_t now)
 {
     Motor_Init(); Encoder_Init(); MotionDrive_Init();
+    /* Same PA15 buzzer mapping as car-02-. HAL MSP has disabled JTAG while
+     * retaining PA13/PA14 SWD. Set the idle level before enabling output. */
+    __HAL_RCC_GPIOA_CLK_ENABLE();
+    PointSignal(0U);
+    GPIO_InitTypeDef buzzer={0};buzzer.Pin=GPIO_PIN_15;
+    buzzer.Mode=GPIO_MODE_OUTPUT_PP;buzzer.Pull=GPIO_NOPULL;buzzer.Speed=GPIO_SPEED_FREQ_LOW;
+    HAL_GPIO_Init(GPIOA,&buzzer);
     LineSensor_Init(now); Imu_Init(now); OLED_Init(now);
     last_draw = now - 200U; shown_state = shown_reason = shown_segment = shown_cal = UINT32_MAX;
     shown_phase=UINT32_MAX;
     point_lit = 0U;
-    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
+    PointSignal(0U);
 }
 void MissionPort_Service(uint32_t now)
 {
     /* Each producer is polled once per loop, including while parked. */
-    Encoder_Update(now); LineSensor_Update(now); Imu_Update(now);
-    if (point_lit && (uint32_t)(now - point_ms) >= 150U) {
-        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
+    Encoder_Update(now); LineSensor_Update(now);
+    uint8_t wheels_still = encoder_data.sample_count != 0U &&
+        (uint32_t)(now - Encoder_LastSampleTime()) <= 30U &&
+        encoder_data.left_delta == 0 && encoder_data.right_delta == 0 &&
+        !speed_control.running;
+    Imu_Update(now, wheels_still);
+    if (point_lit && (uint32_t)(now - point_ms) >= MISSION_POINT_SIGNAL_MS) {
+        PointSignal(0U);
         point_lit = 0U;
     }
     OLED_Update(now);
@@ -67,10 +85,10 @@ void MissionPort_CalibrationCancel(void) { LineCalibration_Cancel(); }
 void MissionPort_NotifyPoint(MissionPoint point, uint32_t now)
 {
     (void)point;
-    /* LED implementation only. Add a nonblocking buzzer here once its actual
-     * board pin/driver is confirmed; never delay the motion loop for a beep. */
+    /* Nonblocking C/B/D/A sound+light pulse, including arrivals whose
+     * incoming segment fails. Completion events are owned by the scheduler. */
     point_ms = now; point_lit = 1U;
-    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
+    PointSignal(1U);
 }
 void MissionPort_Draw(const MissionDiagnostics *d)
 {
@@ -78,6 +96,10 @@ void MissionPort_Draw(const MissionDiagnostics *d)
     static const char *const states[] = {"IDLE", "RUNNING", "DONE", "STOPPED", "REJECTED", "FAULT"};
     static const char *const segments[] = {"AC", "CB", "BD", "DA"};
     static const char *const phases[] = {"IDLE","STILL","TURN","SETTLE","STRAIGHT","CAPTURE","FOLLOW","COMPLETE","FAULT"};
+    static const char *const points[] = {"-","A","B","C","D"};
+    static const char *const faults[] = {"NONE","CONFIG","DISTANCE","TIMEOUT","TURN TIMEOUT",
+        "TURN DIRECTION","TURN TRANSLATION","GYRO RESPONSE","WHEEL SYNC","HEADING",
+        "CAPTURE TIME/DIST","NO LINE","LINE AMBIGUOUS"};
     static const char *const reasons[] = {
         "READY", "NOT IMPLEMENTED", "USER STOP", "BAD CONFIG", "BAD TARGET", "SEGMENT FAULT",
         "GYRO NOT READY", "LINE NOT READY", "ENCODER STALE", "SPEED FAULT", "CALIBRATING"
@@ -89,12 +111,18 @@ void MissionPort_Draw(const MissionDiagnostics *d)
     OLED_Text(0, 0, "MISSION"); OLED_Text(0, 9, d->mode <= MISSION_MODE_DA ? modes[d->mode] : "INVALID");
     OLED_Text(1, 0, d->state <= MISSION_FAULT ? states[d->state] : "INVALID");
     OLED_Text(1, 13, d->segment < SEGMENT_COUNT ? segments[d->segment] : "??");
-    OLED_Text(2, 0, d->reason <= MISSION_CALIBRATING ? reasons[d->reason] : "UNKNOWN");
+    const char *reason=d->reason <= MISSION_CALIBRATING ? reasons[d->reason] : "UNKNOWN";
+    if (d->reason==MISSION_SEGMENT_FAULT && d->segment_fault<sizeof(faults)/sizeof(faults[0]))
+        reason=faults[d->segment_fault];
+    OLED_Text(2, 0, reason);
     OLED_Text(3, 0, d->phase<=PHASE_FAULT ? phases[d->phase] : "INVALID");
     OLED_Text(3, 11, "MM:"); OLED_Number(3,14,d->distance_mm);
     OLED_Text(4, 0, "WIN:"); OLED_Number(4,4,d->point_min_mm); OLED_Text(4,9,"/"); OLED_Number(4,10,d->point_max_mm);
     OLED_Text(5, 0, "DEG:"); OLED_Number(5,4,d->target_yaw_mdeg/1000); OLED_Text(5,10,"ERR:"); OLED_Number(5,14,d->heading_error_mdeg/1000);
-    OLED_Text(6, 0, "CONF:"); OLED_Number(6,5,(int32_t)d->point_confirm_frames); OLED_Text(6,9,"PTS:"); OLED_Number(6,13,(int32_t)d->arrivals);
+    OLED_Text(6,0,"P:"); OLED_Text(6,2,d->last_point<=POINT_D ? points[d->last_point] : "?");
+    OLED_Text(6,4,"N:"); OLED_Number(6,6,(int32_t)d->arrivals);
+    OLED_Text(6,8,"M:"); OLED_Number(6,10,(int32_t)d->line_mask);
+    OLED_Text(6,15,"C:"); OLED_Number(6,17,(int32_t)d->point_confirm_frames);
     OLED_Text(7, 0, d->calibrating ? "CALIBRATING / CANCEL" : (d->state==MISSION_RUNNING ? "PRESS: STOP" : "CLICK START HOLD6 CAL"));
     OLED_Present();
     last_draw = d->uptime_ms; shown_state = d->state; shown_reason = d->reason;
diff --git a/Core/Src/Mission/motion_control.c b/Core/Src/Mission/motion_control.c
index eaa8bea..4b4c321 100644
--- a/Core/Src/Mission/motion_control.c
+++ b/Core/Src/Mission/motion_control.c
@@ -26,10 +26,13 @@ uint8_t Motion_ValidateTuning(void)
         MOTION_TURN_ACCEPT_DEG,MOTION_TURN_TRANSLATION_MAX_MM,MOTION_TURN_WHEEL_MISMATCH_MM,
         MOTION_TURN_GYRO_RESPONSE_MM,MOTION_REVERSE_DISTANCE_FAULT_MM,MOTION_TARGET_RAMP_CPS_S,
         MOTION_TURN_MIN_RESPONSE_DEG,MOTION_TURN_WRONG_DIRECTION_DEG,MOTION_LINE_HIGH_SPEED_THRESHOLD_CPS,
-        MOTION_LINE_HIGH_BASE_FLOOR_CPS,MOTION_LINE_HIGH_WHEEL_FLOOR_CPS};
+        MOTION_LINE_HIGH_BASE_FLOOR_CPS,MOTION_LINE_HIGH_WHEEL_FLOOR_CPS,
+        MOTION_TURN_APPROACH_DEG,MOTION_TURN_APPROACH_MIN_CPS,
+        MOTION_CAPTURE_HEADING_MAX_CPS,MOTION_CAPTURE_HEADING_ACCEPT_DEG,
+        MOTION_CAPTURE_UNCERTAIN_MS};
     const float gains[] = {MOTION_SPEED_KP,MOTION_SPEED_KI,MOTION_HEADING_KP,MOTION_HEADING_KD,
         MOTION_TURN_ANGLE_KP,MOTION_TURN_RATE_FF,MOTION_TURN_RATE_KP,MOTION_LINE_KP,MOTION_LINE_KD,
-        MOTION_LINE_POSITION_SLOWDOWN};
+        MOTION_LINE_POSITION_SLOWDOWN,MOTION_TURN_BRAKE_LEAD_S,MOTION_CAPTURE_HEADING_KP};
     for (uint32_t i=0;i<sizeof(positive)/sizeof(positive[0]);++i)
         if (!isfinite(positive[i]) || positive[i]<=0.0f) return 0U;
     for (uint32_t i=0;i<sizeof(gains)/sizeof(gains[0]);++i)
@@ -38,6 +41,9 @@ uint8_t Motion_ValidateTuning(void)
         MOTION_CAPTURE_CONFIRM_FRAMES<1U || MOTION_TURN_MIN_CPS>MOTION_TURN_MAX_CPS ||
         MOTION_HEADING_TRIM_MAX_CPS>=(float)MOTION_MAX_CPS*0.5f ||
         MOTION_TURN_ACCEPT_DEG<MOTION_TURN_BRAKE_DEG ||
+        MOTION_TURN_APPROACH_MIN_CPS>MOTION_TURN_MIN_CPS ||
+        MOTION_TURN_APPROACH_DEG<=MOTION_TURN_BRAKE_DEG ||
+        MOTION_CAPTURE_UNCERTAIN_MS>MOTION_CAPTURE_TIMEOUT_MS ||
         !(MOTION_PWM_LIMIT>0.0f && MOTION_PWM_LIMIT<=100.0f) ||
         !(MOTION_SPEED_INTEGRAL_LIMIT>0.0f && isfinite(MOTION_SPEED_INTEGRAL_LIMIT)) ||
         !(MOTION_LINE_RECOVERY_FRACTION>0.0f && MOTION_LINE_RECOVERY_FRACTION<=1.0f) ||
diff --git a/Core/Src/Mission/motion_line.c b/Core/Src/Mission/motion_line.c
index b95995d..637166f 100644
--- a/Core/Src/Mission/motion_line.c
+++ b/Core/Src/Mission/motion_line.c
@@ -3,7 +3,7 @@

 /* Extracted from the old line_follow position/PD algorithm. Hardware, lap
  * counting and stop decisions belong to the mission, not this controller. */
-static uint32_t Position(MissionMotionState *s, const MissionInput *in, float *position)
+static uint32_t Position(MissionMotionState *s, const MissionInput *in, uint8_t capturing, float *position)
 {
     static const uint8_t order[8]=MOTION_LINE_ORDER;
     uint32_t groups=0U, previous=0U, total=0U, active=0U;
@@ -21,7 +21,9 @@ static uint32_t Position(MissionMotionState *s, const MissionInput *in, float *p
         previous=on;
     }
     if (!total) return MOTION_FAULT_NO_LINE;
-    if (active>4U) return MOTION_FAULT_AMBIGUOUS;
+    /* A diagonal entry may illuminate most of the bar. Permit only a
+     * SINGLE contiguous wide stripe during bounded capture, never at cruise. */
+    if (active>4U && (!capturing || groups!=1U)) return MOTION_FAULT_AMBIGUOUS;
     if (groups==1U) { *position=(float)weighted[0]/(float)weights[0]; return MOTION_FAULT_NONE; }
     if (!s->have_position || groups!=2U) return MOTION_FAULT_AMBIGUOUS;
     float p0=(float)weighted[0]/(float)weights[0], p1=(float)weighted[1]/(float)weights[1];
@@ -31,13 +33,26 @@ static uint32_t Position(MissionMotionState *s, const MissionInput *in, float *p
     *position=d0<d1 ? p0 : p1;
     return MOTION_FAULT_NONE;
 }
-static void Targets(MissionMotionState *s, float cruise)
+static float CaptureHeadingError(const MissionSegmentContext *ctx, const MissionInput *in,
+                                 const MissionSegmentParams *p)
+{
+    /* These descriptors are semicircles: entry tangent is 180 deg before exit.
+     * CB: 0 deg, DA: 180 deg. Standalone tests retain their existing pose. */
+    return Motion_WrapDegrees(p->target_yaw_deg-180.0f-Motion_Yaw(ctx,in));
+}
+static uint8_t NarrowStripe(uint32_t mask)
+{
+    uint32_t count=0U;
+    for (uint32_t i=0;i<8U;i++) count+=(mask>>i)&1U;
+    return count<=4U;
+}
+static void Targets(MissionMotionState *s, float cruise, float heading_trim)
 {
     float floor=cruise<MOTION_LINE_HIGH_SPEED_THRESHOLD_CPS ? MOTION_LINE_CORNER_BASE_CPS : MOTION_LINE_HIGH_BASE_FLOOR_CPS;
     float base=Motion_Clamp(cruise-fabsf(s->line_position)*MOTION_LINE_POSITION_SLOWDOWN,fminf(floor,cruise),cruise);
     float wheel_floor=cruise<MOTION_LINE_HIGH_SPEED_THRESHOLD_CPS ? MOTION_LINE_MIN_WHEEL_CPS : MOTION_LINE_HIGH_WHEEL_FLOOR_CPS;
     float cap=Motion_Clamp(base-wheel_floor,0.0f,MOTION_LINE_STEERING_MAX_CPS);
-    float turn=Motion_Clamp(MOTION_LINE_KP*s->line_position+MOTION_LINE_KD*s->derivative,-cap,cap);
+    float turn=Motion_Clamp(MOTION_LINE_KP*s->line_position+MOTION_LINE_KD*s->derivative-heading_trim,-cap,cap);
     base=fminf(base,(float)MOTION_MAX_CPS-fabsf(turn));
     s->left_target=base+turn; s->right_target=base-turn;
 }
@@ -57,8 +72,13 @@ MissionSegmentOutput MotionArc_Update(MissionSegmentContext *ctx, const MissionI
 {
     MissionMotionState *s=&ctx->motion;
     s->distance_mm=MotionDistance_GetMm(&s->distance,in);
-    /* Arc heading is a diagnostic difference to its exit tangent. Steering
-     * remains line-position PD, not a straight heading hold. */
+    /* Guide C/D diagonal handoffs toward the entry tangent during
+     * capture; normal following retains line PD and exit-heading diagnostics. */
+    uint8_t align_capture=ctx->phase==PHASE_CAPTURE && ctx->from_handoff;
+    float entry_error=CaptureHeadingError(ctx,in,p);
+    float heading_trim=align_capture ? Motion_Clamp(MOTION_CAPTURE_HEADING_KP*entry_error,
+        -MOTION_CAPTURE_HEADING_MAX_CPS,MOTION_CAPTURE_HEADING_MAX_CPS) : 0.0f;
+    s->target_yaw_deg=align_capture ? Motion_WrapDegrees(p->target_yaw_deg-180.0f) : p->target_yaw_deg;
     s->heading_error_deg=Motion_WrapDegrees(s->target_yaw_deg-Motion_Yaw(ctx,in));
     if (s->distance_mm>p->point_max_mm || s->distance_mm < -MOTION_REVERSE_DISTANCE_FAULT_MM)
         return Motion_Fail(ctx,MOTION_FAULT_DISTANCE);
@@ -70,13 +90,19 @@ MissionSegmentOutput MotionArc_Update(MissionSegmentContext *ctx, const MissionI
      * is seeded only on a confirmed straight->arc handoff, not a single bit. */
     if (MotionEdge_Update(&s->edge,in,s->distance_mm,p->point_min_mm,p->point_max_mm,0U)) return Motion_Complete(ctx);
     float position=0.0f;
-    uint32_t fault=Position(s,in,&position);
+    uint32_t fault=Position(s,in,ctx->phase==PHASE_CAPTURE,&position);
     uint8_t new_capture=!s->have_capture_sequence || in->line_sequence!=s->capture_sequence;
     if (new_capture) {
         s->capture_sequence=in->line_sequence; s->have_capture_sequence=1U;
-        if (!fault && fabsf(position)<=MOTION_CAPTURE_CENTER_LIMIT) ++s->capture_frames;
+        uint8_t aligned=!align_capture || fabsf(entry_error)<=MOTION_CAPTURE_HEADING_ACCEPT_DEG;
+        if (!fault && NarrowStripe(in->line_mask) && aligned && fabsf(position)<=MOTION_CAPTURE_CENTER_LIMIT)
+            ++s->capture_frames;
         else s->capture_frames=0U;
-        if (ctx->phase==PHASE_CAPTURE && s->capture_frames>=MOTION_CAPTURE_CONFIRM_FRAMES) ctx->phase=PHASE_FOLLOW;
+        if (ctx->phase==PHASE_CAPTURE && s->capture_frames>=MOTION_CAPTURE_CONFIRM_FRAMES) {
+            ctx->phase=PHASE_FOLLOW;heading_trim=0.0f;
+            s->target_yaw_deg=p->target_yaw_deg;
+            s->heading_error_deg=Motion_WrapDegrees(s->target_yaw_deg-Motion_Yaw(ctx,in));
+        }
     }
     if (fault) {
         if (!s->uncertain) {
@@ -88,8 +114,15 @@ MissionSegmentOutput MotionArc_Update(MissionSegmentContext *ctx, const MissionI
             s->recovery_left=Motion_Clamp(l*MOTION_LINE_RECOVERY_FRACTION,MOTION_LINE_MIN_WHEEL_CPS,l);
             s->recovery_right=Motion_Clamp(r*MOTION_LINE_RECOVERY_FRACTION,MOTION_LINE_MIN_WHEEL_CPS,r);
         }
-        if ((uint32_t)(in->now_ms-s->uncertain_ms)>=MOTION_LINE_UNCERTAIN_MS) return Motion_Fail(ctx,fault);
-        return Motion_Output(ctx,MOTION_FORWARD,s->recovery_left,s->recovery_right);
+        uint32_t grace=ctx->phase==PHASE_CAPTURE ? MOTION_CAPTURE_UNCERTAIN_MS : MOTION_LINE_UNCERTAIN_MS;
+        if ((uint32_t)(in->now_ms-s->uncertain_ms)>=grace) return Motion_Fail(ctx,fault);
+        /* Preserve positive forward motion while seeking the entry tangent.
+         * The overall capture time/distance bounds remain in force above. */
+        float base=0.5f*(s->recovery_left+s->recovery_right);
+        float cap=fmaxf(0.0f,fminf(base-MOTION_LINE_MIN_WHEEL_CPS,(float)MOTION_MAX_CPS-base));
+        float trim=Motion_Clamp(heading_trim,-cap,cap);
+        if (!align_capture) return Motion_Output(ctx,MOTION_FORWARD,s->recovery_left,s->recovery_right);
+        return Motion_Output(ctx,MOTION_FORWARD,base-trim,base+trim);
     }
     uint8_t recovering=s->uncertain;
     if (recovering) {
@@ -105,10 +138,10 @@ MissionSegmentOutput MotionArc_Update(MissionSegmentContext *ctx, const MissionI
         }
         s->line_position=position; s->last_position=position; s->have_position=1U;
         s->line_sequence=in->line_sequence; s->have_line_sequence=1U; s->control_ms=in->now_ms;
-        Targets(s,ctx->phase==PHASE_CAPTURE ? p->capture_cps : p->cruise_cps);
+        Targets(s,ctx->phase==PHASE_CAPTURE ? p->capture_cps : p->cruise_cps,heading_trim);
     }
     /* Capture may finish on a fresh frame before the PD period. Update the
      * cruise target immediately without differentiating the same frame twice. */
-    if (ctx->phase==PHASE_FOLLOW && s->capture_frames==MOTION_CAPTURE_CONFIRM_FRAMES) Targets(s,p->cruise_cps);
+    if (ctx->phase==PHASE_FOLLOW && s->capture_frames==MOTION_CAPTURE_CONFIRM_FRAMES) Targets(s,p->cruise_cps,0.0f);
     return Motion_Output(ctx,MOTION_FORWARD,s->left_target,s->right_target);
 }
diff --git a/Core/Src/Mission/motion_straight.c b/Core/Src/Mission/motion_straight.c
index 8fc367f..a4b004f 100644
--- a/Core/Src/Mission/motion_straight.c
+++ b/Core/Src/Mission/motion_straight.c
@@ -39,15 +39,27 @@ static MissionSegmentOutput Turn(MissionSegmentContext *ctx, const MissionInput
     if (elapsed>MOTION_TURN_DIRECTION_GRACE_MS && s->turn_start_error*(Motion_Yaw(ctx,in)-s->turn_origin_yaw)<
         -fabsf(s->turn_start_error)*MOTION_TURN_WRONG_DIRECTION_DEG)
         return Motion_Fail(ctx,MOTION_FAULT_TURN_DIRECTION);
-    if (fabsf(s->heading_error_deg)<=MOTION_TURN_BRAKE_DEG || s->turn_start_error*s->heading_error_deg<=0.0f) {
+    /* Anticipate the short braking delay only while rotating TOWARDS the
+     * target. Cap the lead inside the accepted angle, avoiding an early stop
+     * far outside tolerance and an unnecessary extra correction cycle. */
+    float toward_rate=s->heading_error_deg>0.0f ? MOTION_YAW_SIGN*in->yaw_rate_dps :
+        -MOTION_YAW_SIGN*in->yaw_rate_dps;
+    float brake_deg=Motion_Clamp(MOTION_TURN_BRAKE_DEG+
+        fmaxf(toward_rate,0.0f)*MOTION_TURN_BRAKE_LEAD_S,
+        MOTION_TURN_BRAKE_DEG,MOTION_TURN_ACCEPT_DEG);
+    if (fabsf(s->heading_error_deg)<=brake_deg || s->turn_start_error*s->heading_error_deg<=0.0f) {
         ctx->phase=PHASE_SETTLE; ctx->phase_started_ms=in->now_ms; s->still_tracking=0U;
         return Motion_Output(ctx,MOTION_BRAKE,0.0f,0.0f);
     }
     if ((uint32_t)(in->now_ms-s->control_ms)>=MOTION_HEADING_PERIOD_MS) {
         float desired=Motion_Clamp(MOTION_TURN_ANGLE_KP*s->heading_error_deg,-MOTION_TURN_RATE_MAX_DPS,MOTION_TURN_RATE_MAX_DPS);
         float command=MOTION_TURN_RATE_FF*desired+MOTION_TURN_RATE_KP*(desired-MOTION_YAW_SIGN*in->yaw_rate_dps);
-        command=s->heading_error_deg>0.0f ? Motion_Clamp(command,MOTION_TURN_MIN_CPS,MOTION_TURN_MAX_CPS) :
-            Motion_Clamp(command,-(float)MOTION_TURN_MAX_CPS,-MOTION_TURN_MIN_CPS);
+        /* Fade the coarse floor near the target. The old fixed 350 cps
+         * floor overrode deceleration even when only 2-3 degrees remained. */
+        float minimum=Motion_Clamp(MOTION_TURN_MIN_CPS*fabsf(s->heading_error_deg)/MOTION_TURN_APPROACH_DEG,
+            MOTION_TURN_APPROACH_MIN_CPS,MOTION_TURN_MIN_CPS);
+        command=s->heading_error_deg>0.0f ? Motion_Clamp(command,minimum,MOTION_TURN_MAX_CPS) :
+            Motion_Clamp(command,-(float)MOTION_TURN_MAX_CPS,-minimum);
         s->left_target=-command; s->right_target=command; s->control_ms=in->now_ms;
     }
     return Motion_Output(ctx,MOTION_TURN,s->left_target,s->right_target);
diff --git a/Core/Src/Modules/imu_heading.c b/Core/Src/Modules/imu_heading.c
index 5612e50..77dd921 100644
--- a/Core/Src/Modules/imu_heading.c
+++ b/Core/Src/Modules/imu_heading.c
@@ -1,54 +1,53 @@
 #include "imu_heading.h"
-/* MPU6050: gyro FS_SEL=0 (131 LSB/dps), accelerometer AFS_SEL=0.
- * Relative yaw around sensor Z only; module must lie flat on the car.
- * No accelerometer-derived heading or fictitious absolute yaw correction. */
+#include "imu_config.h"
+#include <math.h>
+/* car-02- uses the installed sensor's measured Z reference to identify
+ * stillness, then estimates the actual bias on every boot. Acceleration is
+ * diagnostic only: yaw is relative rotation around the sensor's vertical Z. */
 static int32_t AbsRaw(int16_t value) { return value<0 ? -(int32_t)value : value; }
 void ImuHeading_Reset(ImuHeading *h) { *h=(ImuHeading){0}; }
 static void RestartWindow(ImuHeading *h, uint32_t reason)
 {
     h->samples=0U;h->mean_z=h->m2_z=0.0f;h->restarts++;h->reject_reason=reason;
 }
-uint32_t ImuHeading_Sample(ImuHeading *h, const int16_t accel[3], const int16_t gyro[3], uint32_t now_ms)
+uint32_t ImuHeading_Sample(ImuHeading *h, int16_t gyro_z, uint8_t wheels_still, uint32_t now_ms)
 {
     uint32_t dt=h->have_time ? now_ms-h->last_ms : 0U;
     if (h->have_time && !dt) return h->ready ? HEADING_READY : HEADING_CALIBRATING;
     h->last_ms=now_ms;h->have_time=1U;
+    if ((float)AbsRaw(gyro_z)>IMU_GYRO_MAX_ABS_DPS*IMU_GYRO_LSB_PER_DPS) { h->ready=0U;return HEADING_SATURATED; }
     if (h->ready) {
-        if (dt>30U) { h->ready=0U;return HEADING_GAP; }
-        for (uint32_t i=0;i<3;i++) if (AbsRaw(gyro[i])>=32700) { h->ready=0U;return HEADING_SATURATED; }
-        h->rate_dps=((float)gyro[2]-h->bias_raw)/131.0f;
+        if (dt>IMU_MAX_SAMPLE_GAP_MS) { h->ready=0U;return HEADING_GAP; }
+        h->rate_dps=((float)gyro_z-h->bias_raw)/IMU_GYRO_LSB_PER_DPS;
         h->yaw_deg+=(h->previous_rate+h->rate_dps)*0.0005f*(float)dt;
         h->previous_rate=h->rate_dps;
         return HEADING_READY;
     }
-    if (dt>30U && h->samples) RestartWindow(h,3U);
-    uint8_t moving=0U;
-    for (uint32_t i=0;i<3;i++) if (AbsRaw(gyro[i])>655) moving=1U;
-    if (moving) { RestartWindow(h,1U);return HEADING_CALIBRATING; }
-    if (AbsRaw(accel[0])>4000 || AbsRaw(accel[1])>4000 || AbsRaw(accel[2])<14000 || AbsRaw(accel[2])>18500) {
-        RestartWindow(h,2U);return HEADING_CALIBRATING;
-    }
-    if (!h->samples) {
-        h->first_ms=now_ms;
-        for (uint32_t i=0;i<3;i++) h->low[i]=h->high[i]=(float)gyro[i];
-    }
-    for (uint32_t i=0;i<3;i++) {
-        if ((float)gyro[i]<h->low[i]) h->low[i]=(float)gyro[i];
-        if ((float)gyro[i]>h->high[i]) h->high[i]=(float)gyro[i];
-        if (h->high[i]-h->low[i]>196.5f) { RestartWindow(h,1U);return HEADING_CALIBRATING; }
+    if (dt>IMU_MAX_SAMPLE_GAP_MS && h->samples) RestartWindow(h,3U);
+    if (!wheels_still) { RestartWindow(h,2U);return HEADING_CALIBRATING; }
+    float z_dps=(float)gyro_z/IMU_GYRO_LSB_PER_DPS;
+    if (!isfinite(IMU_Z_BIAS_PRIOR_DPS) ||
+        fabsf(z_dps-IMU_Z_BIAS_PRIOR_DPS)>IMU_CAL_WINDOW_DPS) {
+        RestartWindow(h,1U);return HEADING_CALIBRATING;
     }
+    if (!h->samples) h->first_ms=now_ms;
     h->samples++;
-    float delta=(float)gyro[2]-h->mean_z;
+    float delta=(float)gyro_z-h->mean_z;
     h->mean_z+=delta/(float)h->samples;
-    h->m2_z+=delta*((float)gyro[2]-h->mean_z);
-    if (h->samples>=200U && (uint32_t)(now_ms-h->first_ms)>=1900U) {
-        if (h->m2_z/(float)h->samples>1544.49f) { RestartWindow(h,1U);return HEADING_CALIBRATING; }
+    h->m2_z+=delta*((float)gyro_z-h->mean_z);
+    if (h->samples>=IMU_CAL_SAMPLES && (uint32_t)(now_ms-h->first_ms)>=IMU_CAL_MIN_MS) {
+        float max_variance=IMU_CAL_STD_DPS*IMU_GYRO_LSB_PER_DPS;
+        max_variance*=max_variance;
+        if (h->m2_z/(float)(h->samples-1U)>max_variance) {
+            RestartWindow(h,1U);return HEADING_CALIBRATING;
+        }
         h->bias_raw=h->mean_z;h->yaw_deg=0.0f;
-        h->rate_dps=((float)gyro[2]-h->bias_raw)/131.0f;
+        h->rate_dps=((float)gyro_z-h->bias_raw)/IMU_GYRO_LSB_PER_DPS;
         h->previous_rate=h->rate_dps;h->ready=1U;h->reject_reason=0U;
     }
     return h->ready ? HEADING_READY : HEADING_CALIBRATING;
 }
+
 void ImuHeading_Zero(ImuHeading *h) { if (h->ready) h->yaw_deg=0.0f; }

 void GyroCheck_Reset(GyroCheck *h) { *h=(GyroCheck){0}; }
@@ -80,7 +79,7 @@ void GyroCheck_Sample(GyroCheck *h,const int16_t gyro[3],uint32_t now_ms)
     if (dt>30U) { GyroCheck_Fail(h,GYRO_TEST_GAP);return; }
     for (uint32_t i=0;i<3U;i++) if (AbsRaw(gyro[i])>=32700) { GyroCheck_Fail(h,GYRO_TEST_RAIL);return; }
     if (h->state==GYRO_TEST_READOUT) {
-        h->rate=((float)gyro[2]-h->bias)/131.0f;
+        h->rate=((float)gyro[2]-h->bias)/IMU_GYRO_LSB_PER_DPS;
         h->yaw+=(h->previous_rate+h->rate)*0.0005f*(float)dt;h->previous_rate=h->rate;
         if (h->yaw>36000.0f || h->yaw< -36000.0f) GyroCheck_Fail(h,GYRO_TEST_RANGE);
         return;
@@ -97,13 +96,13 @@ void GyroCheck_Sample(GyroCheck *h,const int16_t gyro[3],uint32_t now_ms)
     for (uint32_t i=0;i<3U;i++) {
         if ((float)gyro[i]<h->low[i]) h->low[i]=(float)gyro[i];
         if ((float)gyro[i]>h->high[i]) h->high[i]=(float)gyro[i];
-        if (h->high[i]-h->low[i]>196.5f) { GyroCheck_Restart(h);return; }
+        if (h->high[i]-h->low[i]>1.5f*IMU_GYRO_LSB_PER_DPS) { GyroCheck_Restart(h);return; }
     }
     h->samples++;float delta=(float)gyro[2]-h->mean;
     h->mean+=delta/(float)h->samples;h->m2+=delta*((float)gyro[2]-h->mean);
     if (h->samples>=400U && (uint32_t)(now_ms-h->first_ms)>=3900U) {
-        if (h->m2/(float)h->samples>1544.49f) { GyroCheck_Restart(h);return; }
-        h->bias=h->mean;h->rate=((float)gyro[2]-h->bias)/131.0f;
+        if (h->m2/(float)h->samples>0.09f*IMU_GYRO_LSB_PER_DPS*IMU_GYRO_LSB_PER_DPS) { GyroCheck_Restart(h);return; }
+        h->bias=h->mean;h->rate=((float)gyro[2]-h->bias)/IMU_GYRO_LSB_PER_DPS;
         h->previous_rate=h->rate;h->yaw=0.0f;h->zero_ms=now_ms;h->state=GYRO_TEST_READOUT;
     }
 }
diff --git a/Core/Src/Modules/imu_heading.h b/Core/Src/Modules/imu_heading.h
index a7fb8a4..d2fb53c 100644
--- a/Core/Src/Modules/imu_heading.h
+++ b/Core/Src/Modules/imu_heading.h
@@ -1,13 +1,15 @@
 #ifndef IMU_HEADING_H
 #define IMU_HEADING_H
 #include <stdint.h>
+/* Diagnostic reject_reason: 1=reference/noise, 2=wheels not stationary, 3=gap. */
 enum { HEADING_CALIBRATING, HEADING_READY, HEADING_GAP, HEADING_SATURATED };
 typedef struct {
     uint32_t samples, restarts, last_ms, first_ms, have_time, ready, reject_reason;
     float mean_z, m2_z, low[3], high[3], bias_raw, yaw_deg, rate_dps, previous_rate;
 } ImuHeading;
 void ImuHeading_Reset(ImuHeading *h);
-uint32_t ImuHeading_Sample(ImuHeading *h, const int16_t accel[3], const int16_t gyro[3], uint32_t now_ms);
+/* Z-only calibration: fresh encoders, no wheel travel, stopped drive. */
+uint32_t ImuHeading_Sample(ImuHeading *h, int16_t gyro_z, uint8_t wheels_still, uint32_t now_ms);
 void ImuHeading_Zero(ImuHeading *h);
 /* Independent manual experiment. GYRO_TEST_READOUT never certifies a sensor
  * or grants permission to a motor controller. Operator must ensure stillness. */
diff --git a/Core/Src/Modules/imu_mpu6050.c b/Core/Src/Modules/imu_mpu6050.c
index 01c1846..886c123 100644
--- a/Core/Src/Modules/imu_mpu6050.c
+++ b/Core/Src/Modules/imu_mpu6050.c
@@ -1,5 +1,6 @@
 #include "imu_mpu6050.h"
 #include "imu_heading.h"
+#include "imu_config.h"
 #include "main.h"
 /* C10B H6, schematic sheet 3: pin 3 PB14/SCL, pin 4 PB15/SDA.
  * Open-drain software I2C. PB9/INT is unused; poll DATA_RDY.
@@ -16,46 +17,65 @@ static uint32_t samples, errors, misses, max_update_us, zero_count;
 static uint32_t transaction_cycles, bus_fault;
 static uint8_t address, identity, configured;
 static int16_t accel[3], gyro[3];
-static uint8_t slow_bus, signal_reset_done, config_readback[9];
+static uint8_t slow_bus, signal_reset_done, interrupt_configured, config_readback[9];
+/* Raw V3: normal_bytes[0,13,14] are the status/Z control sample;
+ * bytes [1..12] and accel/gyro X/Y come from the timestamped slow diagnostic
+ * transaction. They are not a simultaneous full six-axis measurement. */
 static uint8_t normal_bytes[15], slow_bytes[14], ax_bytes[2], gz_bytes[2];
 static uint32_t cross_stage, cross_count, cross_errors, cross_begin_ms;
 static uint32_t slow_frame_ms, ax_frame_ms, gz_frame_ms;
 static uint32_t boot_attempts, boot_failures, bus_clear_count, clear_clocks, recovery_ok;
 static uint32_t Cycles(void) { return DWT->CYCCNT; }
 static uint32_t PinLevels(void) { uint32_t pins=GPIOB->IDR;return ((pins&SCL)!=0U)|(((pins&SDA)!=0U)<<1U); }
-static void Delay(void) { uint32_t n=Cycles();uint32_t ticks=SystemCoreClock/(slow_bus ? 166666U : 333333U);for (uint32_t guard=0;guard<10000U && (uint32_t)(Cycles()-n)<ticks;guard++) __NOP(); }
-static void Low(uint32_t pin) { GPIOB->BRR=pin; }
-static void Release(uint32_t pin) { GPIOB->BSRR=pin; }
+/* Match car-02-'s 5 us setup/clock phases and HAL open-drain GPIO writes.
+ * Keep bounded clock stretching, transaction timeouts and bus recovery. */
+static void Delay(void)
+{
+    uint32_t n=Cycles();
+    uint32_t ticks=(SystemCoreClock/1000000U)*(slow_bus ? 10U : 5U);
+    for (uint32_t guard=0;guard<10000U && (uint32_t)(Cycles()-n)<ticks;guard++) __NOP();
+}
+static void Low(uint32_t pin) { HAL_GPIO_WritePin(GPIOB,(uint16_t)pin,GPIO_PIN_RESET); }
+static void Release(uint32_t pin) { HAL_GPIO_WritePin(GPIOB,(uint16_t)pin,GPIO_PIN_SET); }
 static uint8_t HighClock(void)
 {
     Release(SCL);
     uint32_t begin=Cycles();
     for (uint32_t guard=0;guard<20000U;guard++) {
-        if ((uint32_t)(Cycles()-transaction_cycles)>SystemCoreClock/(slow_bus ? 125U : 250U)) { bus_fault=2U;return 0U; }
-        if (GPIOB->IDR&SCL) return 1U;
-        if ((uint32_t)(Cycles()-begin)>SystemCoreClock/5000U) break;
+        if ((uint32_t)(Cycles()-transaction_cycles)>SystemCoreClock/(slow_bus ? 83U : 166U)) { bus_fault=2U;return 0U; }
+        if (HAL_GPIO_ReadPin(GPIOB,SCL)==GPIO_PIN_SET) return 1U;
+        if ((uint32_t)(Cycles()-begin)>=SystemCoreClock/10000U) break;
     }
     bus_fault=1U;return 0U;
 }
+static uint8_t Clock(uint8_t high)
+{
+    if (high) { if (!HighClock()) return 0U; }
+    else Low(SCL);
+    Delay();return 1U;
+}
 static void Stop(void)
 {
-    Low(SCL);Low(SDA);Delay();(void)HighClock();Delay();Release(SDA);Delay();
+    (void)Clock(0U);Low(SDA);Delay();(void)Clock(1U);Release(SDA);Delay();
     Release(SCL|SDA);
 }
 static uint8_t Start(void)
 {
-    Release(SDA);Delay();if (!HighClock()) return 0U;Delay();
-    if (!(GPIOB->IDR&SDA)) { bus_fault=3U;return 0U; }
-    Low(SDA);Delay();Low(SCL);return 1U;
+    Release(SDA);
+    if (!Clock(1U)) return 0U;
+    if (HAL_GPIO_ReadPin(GPIOB,SDA)==GPIO_PIN_RESET) { bus_fault=3U;return 0U; }
+    Low(SDA);Delay();return Clock(0U);
 }
 static uint8_t WriteByte(uint8_t value)
 {
     for (uint32_t i=0;i<8U;i++) {
         if (value&0x80U) Release(SDA);else Low(SDA);
-        Delay();if (!HighClock()) return 0U;Delay();Low(SCL);value<<=1;
+        Delay();if (!Clock(1U)) return 0U;
+        (void)Clock(0U);value<<=1;
     }
-    Release(SDA);Delay();if (!HighClock()) return 0U;Delay();
-    uint8_t ack=(GPIOB->IDR&SDA)==0U;Low(SCL);
+    Release(SDA);Delay();if (!Clock(1U)) return 0U;
+    uint8_t ack=HAL_GPIO_ReadPin(GPIOB,SDA)==GPIO_PIN_RESET;
+    (void)Clock(0U);
     if (!ack) bus_fault=4U;
     return ack;
 }
@@ -63,11 +83,13 @@ static uint8_t ReadByte(uint8_t *value,uint8_t ack)
 {
     *value=0;Release(SDA);
     for (uint32_t i=0;i<8U;i++) {
-        Delay();if (!HighClock()) return 0U;Delay();
-        *value=(uint8_t)((*value<<1)|((GPIOB->IDR&SDA)!=0U));Low(SCL);
+        if (!Clock(1U)) return 0U;
+        *value=(uint8_t)((*value<<1)|(HAL_GPIO_ReadPin(GPIOB,SDA)==GPIO_PIN_SET));
+        (void)Clock(0U);
     }
     if (ack) Low(SDA);else Release(SDA);
-    Delay();if (!HighClock()) return 0U;Delay();Low(SCL);Release(SDA);return 1U;
+    if (!Clock(1U)) return 0U;
+    (void)Clock(0U);return 1U;
 }
 static uint8_t ReadRegs(uint8_t device,uint8_t reg,uint8_t *data,uint32_t size)
 {
@@ -110,7 +132,7 @@ static uint8_t InitBus(void)
     if (Cycles()==before) return 0U;
     Release(SCL|SDA);
     GPIO_InitTypeDef pins={0};pins.Pin=SCL|SDA;pins.Mode=GPIO_MODE_OUTPUT_OD;
-    pins.Pull=GPIO_NOPULL;pins.Speed=GPIO_SPEED_FREQ_LOW;HAL_GPIO_Init(GPIOB,&pins);
+    pins.Pull=GPIO_NOPULL;pins.Speed=GPIO_SPEED_FREQ_HIGH;HAL_GPIO_Init(GPIOB,&pins);
     /* Bus recovery runs after the initial power-up wait, before each probe. */
     bus_fault=0U;return 1U;
 }
@@ -128,19 +150,19 @@ static int16_t BigEndian(const uint8_t *data)
 static void Publish(uint32_t now_ms)
 {
     uint32_t seq=(imu_diag[2]+2U)&~1U;imu_diag[2]=seq-1U;
-    imu_diag[0]=0x494D5531U;imu_diag[1]=1U;imu_diag[3]=state;imu_diag[4]=fault;
+    imu_diag[0]=0x494D5531U;imu_diag[1]=2U;imu_diag[3]=state;imu_diag[4]=fault;
     imu_diag[5]=address;imu_diag[6]=identity;imu_diag[7]=samples;imu_diag[8]=last_frame_ms;
     imu_diag[9]=now_ms;imu_diag[10]=samples ? now_ms-last_frame_ms : UINT32_MAX;
     for (uint32_t i=0;i<3;i++) { imu_diag[11+i]=(uint32_t)(int32_t)gyro[i];imu_diag[14+i]=(uint32_t)(int32_t)accel[i]; }
-    imu_diag[17]=(uint32_t)(int32_t)(heading.bias_raw*1000.0f/131.0f);
+    imu_diag[17]=(uint32_t)(int32_t)(heading.bias_raw*1000.0f/IMU_GYRO_LSB_PER_DPS);
     imu_diag[18]=(uint32_t)(int32_t)(heading.yaw_deg*1000.0f);imu_diag[19]=(uint32_t)(int32_t)(heading.rate_dps*1000.0f);
     imu_diag[20]=heading.samples;imu_diag[21]=heading.restarts;imu_diag[22]=errors;
     imu_diag[23]=max_update_us;imu_diag[24]=bus_fault;imu_diag[25]=misses;
-    imu_diag[26]=configured==9U ? 0U : UINT32_MAX;imu_diag[27]=configured==9U ? 9U : UINT32_MAX;
-    imu_diag[28]=configured==9U ? 3U : UINT32_MAX;imu_diag[29]=zero_count;imu_diag[30]=heading.reject_reason;
+    imu_diag[26]=configured==9U ? IMU_GYRO_CONFIG : UINT32_MAX;imu_diag[27]=configured==9U ? IMU_SAMPLE_DIVIDER : UINT32_MAX;
+    imu_diag[28]=configured==9U ? IMU_DLPF_CONFIG : UINT32_MAX;imu_diag[29]=zero_count;imu_diag[30]=heading.reject_reason;
     imu_diag[31]=seq;imu_diag[2]=seq;publish_ms=now_ms;
     seq=(imu_raw_diag[2]+2U)&~1U;imu_raw_diag[2]=seq-1U;
-    imu_raw_diag[0]=0x49525731U;imu_raw_diag[1]=2U;imu_raw_diag[3]=cross_count;
+    imu_raw_diag[0]=0x49525731U;imu_raw_diag[1]=3U;imu_raw_diag[3]=cross_count;
     imu_raw_diag[4]=now_ms;imu_raw_diag[5]=last_frame_ms;imu_raw_diag[6]=slow_frame_ms;
     imu_raw_diag[7]=ax_frame_ms;imu_raw_diag[8]=gz_frame_ms;imu_raw_diag[9]=cross_errors;
     imu_raw_diag[10]=signal_reset_done;
@@ -149,7 +171,7 @@ static void Publish(uint32_t now_ms)
     for (uint32_t i=0;i<2U;i++) { imu_raw_diag[40+i]=ax_bytes[i];imu_raw_diag[42+i]=gz_bytes[i]; }
     for (uint32_t i=0;i<9U;i++) imu_raw_diag[44+i]=config_readback[i];
     imu_raw_diag[53]=cross_stage;imu_raw_diag[54]=configured;
-    imu_raw_diag[55]=6U;imu_raw_diag[56]=3U;
+    imu_raw_diag[55]=10U;imu_raw_diag[56]=5U;
     imu_raw_diag[57]=boot_attempts;imu_raw_diag[58]=bus_clear_count;imu_raw_diag[59]=clear_clocks;
     imu_raw_diag[60]=PinLevels();imu_raw_diag[61]=recovery_ok;imu_raw_diag[62]=boot_failures;
     imu_raw_diag[63]=seq;imu_raw_diag[2]=seq;
@@ -159,7 +181,7 @@ static void Publish(uint32_t now_ms)
     imu_gyro_test[5]=gyro_check.samples;imu_gyro_test[6]=gyro_check.restarts;
     imu_gyro_test[7]=now_ms;imu_gyro_test[8]=gyro_check.last_ms;
     imu_gyro_test[9]=gyro_check.have_time ? now_ms-gyro_check.last_ms : UINT32_MAX;
-    imu_gyro_test[10]=(uint32_t)(int32_t)(gyro_check.bias*1000.0f/131.0f);
+    imu_gyro_test[10]=(uint32_t)(int32_t)(gyro_check.bias*1000.0f/IMU_GYRO_LSB_PER_DPS);
     imu_gyro_test[11]=(uint32_t)(int32_t)(gyro_check.rate*1000.0f);
     imu_gyro_test[12]=(uint32_t)(int32_t)(gyro_check.yaw*1000.0f);
     imu_gyro_test[13]=gyro_check.zero_count;imu_gyro_test[14]=gyro_check.zero_ms;
@@ -173,14 +195,14 @@ static void Publish(uint32_t now_ms)
     imu_gyro_test[22]=0U; /* Motor control enabled: NEVER set by this test. */
     imu_gyro_test[23]=400U;imu_gyro_test[24]=gyro_check.first_ms;
     imu_gyro_test[25]=(uint32_t)(int32_t)gyro[2];
-    imu_gyro_test[26]=(gyro_check.bias>2620.0f || gyro_check.bias< -2620.0f);
+    imu_gyro_test[26]=(gyro_check.bias>20.0f*IMU_GYRO_LSB_PER_DPS || gyro_check.bias< -20.0f*IMU_GYRO_LSB_PER_DPS);
     imu_gyro_test[31]=seq;imu_gyro_test[2]=seq;
 }
 void Imu_Init(uint32_t now_ms)
 {
     ImuHeading_Reset(&heading);GyroCheck_Reset(&gyro_check);state=IMU_BOOT;fault=IMU_OK;phase_ms=poll_ms=now_ms;
     samples=errors=misses=max_update_us=zero_count=0U;address=identity=configured=0U;last_frame_ms=0U;
-    slow_bus=signal_reset_done=0U;cross_stage=cross_count=cross_errors=0U;cross_begin_ms=now_ms;
+    slow_bus=signal_reset_done=interrupt_configured=0U;cross_stage=cross_count=cross_errors=0U;cross_begin_ms=now_ms;
     slow_frame_ms=ax_frame_ms=gz_frame_ms=0U;
     boot_attempts=boot_failures=bus_clear_count=clear_clocks=recovery_ok=0U;
     for (uint32_t i=0;i<3U;i++) { accel[i]=0;gyro[i]=0; }
@@ -212,7 +234,7 @@ void Imu_GetGyroSample(uint32_t now_ms, ImuGyroSample *sample)
     sample->device_fault=fault;sample->last_ms=gyro_check.last_ms;
     sample->age_ms=gyro_check.have_time ? now_ms-gyro_check.last_ms : UINT32_MAX;
     sample->yaw=gyro_check.yaw;sample->rate=gyro_check.rate;
-    sample->bias_dps=gyro_check.bias/131.0f;sample->accel_mask=0U;
+    sample->bias_dps=gyro_check.bias/IMU_GYRO_LSB_PER_DPS;sample->accel_mask=0U;
     for (uint32_t i=0;i<3U;i++) if (accel[i]==-32768 || accel[i]==32767) sample->accel_mask|=1U<<i;
 }
 void Imu_GetHeadingSample(uint32_t now_ms, ImuHeadingSample *sample)
@@ -224,7 +246,7 @@ void Imu_GetHeadingSample(uint32_t now_ms, ImuHeadingSample *sample)
         .yaw_deg = heading.yaw_deg, .rate_dps = heading.rate_dps
     };
 }
-static void Advance(uint32_t now_ms)
+static void Advance(uint32_t now_ms, uint8_t wheels_still)
 {
     if (state==IMU_FAULT) return;
     if (state==IMU_BOOT) {
@@ -237,20 +259,24 @@ static void Advance(uint32_t now_ms)
         }
         if (!response) { BootFailure(IMU_NO_ACK,now_ms);return; }
         if (identity!=0x68U) { Fail(IMU_BAD_ID);return; }
-        if (!WriteReg(0x6BU,0x80U)) { BootFailure(IMU_BUS_ERROR,now_ms);return; }
-        state=IMU_RESET_WAIT;phase_ms=now_ms;return;
+        /* car-02- configures the awake device directly. Do not add a
+         * device/signal-path reset that the reference driver does not use. */
+        state=IMU_RESET_WAIT;phase_ms=now_ms-100U;return;
     }
     if (state==IMU_RESET_WAIT) {
         if ((uint32_t)(now_ms-phase_ms)<100U) return;
-        if (!signal_reset_done) {
-            /* RM-MPU-6000A-00 section 4.26: reset analog/digital paths.
-             * This write-only, self-clearing register is not readback-tested. */
-            if (!WriteReg(0x68U,0x07U)) { errors++;Fail(IMU_BUS_ERROR);return; }
-            signal_reset_done=1U;phase_ms=now_ms;state=IMU_SIGNAL_RESET_WAIT;return;
+        if (!interrupt_configured) {
+            uint8_t value=0xFFU;
+            /* Clear DATA_RDY only by reading INT_STATUS, as in car-02-. */
+            if (!WriteReg(0x37U,0x00U) || !ReadRegs(address,0x37U,&value,1U)) {
+                errors++;Fail(IMU_BUS_ERROR);return;
+            }
+            if (value!=0U) { Fail(IMU_CONFIG_ERROR);return; }
+            interrupt_configured=1U;return;
         }
         /* Configure and verify one register per main iteration. */
         static const uint8_t regs[]={0x6B,0x6C,0x19,0x1A,0x1B,0x1C,0x23,0x6A,0x38};
-        static const uint8_t vals[]={0x01,0x00,0x09,0x03,0x00,0x00,0x00,0x00,0x01};
+        static const uint8_t vals[]={0x01,0x00,IMU_SAMPLE_DIVIDER,IMU_DLPF_CONFIG,IMU_GYRO_CONFIG,0x00,0x00,0x00,0x01};
         if (configured<sizeof(regs)) {
             uint8_t value=0U;
             if (!WriteReg(regs[configured],vals[configured]) || !ReadRegs(address,regs[configured],&value,1U)) { errors++;Fail(IMU_BUS_ERROR);return; }
@@ -260,27 +286,26 @@ static void Advance(uint32_t now_ms)
         }
         state=IMU_SETTLE;phase_ms=now_ms;return;
     }
-    if (state==IMU_SIGNAL_RESET_WAIT) {
-        if ((uint32_t)(now_ms-phase_ms)<100U) return;
-        state=IMU_RESET_WAIT;phase_ms=now_ms-100U;return;
-    }
     if (state==IMU_SETTLE) {
         if ((uint32_t)(now_ms-phase_ms)<200U) return;
-        state=IMU_CALIBRATING;poll_ms=now_ms-5U;last_frame_ms=now_ms;
+        state=IMU_CALIBRATING;poll_ms=now_ms-IMU_POLL_MS;last_frame_ms=now_ms;
     }
-    if ((uint32_t)(now_ms-poll_ms)<5U) return;
+    if ((uint32_t)(now_ms-poll_ms)<IMU_POLL_MS) return;
     poll_ms=now_ms;
-    uint8_t data[15]={0};
-    if (!ReadRegs(address,0x3AU,data,sizeof(data))) { errors++;Fail(IMU_BUS_ERROR);return; }
-    if (!(data[0]&1U)) {
+    uint8_t status=0U, z_bytes[2]={0};
+    if (!ReadRegs(address,0x3AU,&status,1U)) { errors++;Fail(IMU_BUS_ERROR);return; }
+    if (!(status&1U)) {
         misses++;if ((uint32_t)(now_ms-last_frame_ms)>50U) Fail(IMU_SAMPLE_GAP);
         return;
     }
-    for (uint32_t i=0;i<3;i++) { accel[i]=BigEndian(&data[1+i*2]);gyro[i]=BigEndian(&data[9+i*2]); }
-    for (uint32_t i=0;i<15U;i++) normal_bytes[i]=data[i];
+    /* Same fresh Z-only read as car-02-. Other axes are collected
+     * separately by CrossRead for diagnostics, never for yaw readiness. */
+    if (!ReadRegs(address,0x47U,z_bytes,2U)) { errors++;Fail(IMU_BUS_ERROR);return; }
+    gyro[2]=BigEndian(z_bytes);
+    normal_bytes[0]=status;normal_bytes[13]=z_bytes[0];normal_bytes[14]=z_bytes[1];
     samples++;last_frame_ms=now_ms;
     GyroCheck_Sample(&gyro_check,gyro,now_ms);
-    uint32_t result=ImuHeading_Sample(&heading,accel,gyro,now_ms);
+    uint32_t result=ImuHeading_Sample(&heading,gyro[2],wheels_still,now_ms);
     if (result==HEADING_GAP) Fail(IMU_SAMPLE_GAP);
     else if (result==HEADING_SATURATED) Fail(IMU_GYRO_SATURATED);
     else state=heading.ready ? IMU_READY : IMU_CALIBRATING;
@@ -303,6 +328,9 @@ static void CrossRead(uint32_t now_ms)
     if (!ok) { cross_errors++;cross_stage=0U;return; }
     if (cross_stage==1U) {
         for (uint32_t i=0;i<14U;i++) slow_bytes[i]=bytes[i];
+        for (uint32_t i=0;i<3U;i++) accel[i]=BigEndian(&bytes[i*2U]);
+        for (uint32_t i=0;i<2U;i++) gyro[i]=BigEndian(&bytes[8U+i*2U]);
+        for (uint32_t i=0;i<12U;i++) normal_bytes[1U+i]=bytes[i];
         slow_frame_ms=now_ms;cross_stage=2U;
     } else if (cross_stage==2U) {
         ax_bytes[0]=bytes[0];ax_bytes[1]=bytes[1];ax_frame_ms=now_ms;cross_stage=3U;
@@ -310,9 +338,9 @@ static void CrossRead(uint32_t now_ms)
         gz_bytes[0]=bytes[0];gz_bytes[1]=bytes[1];gz_frame_ms=now_ms;cross_stage=0U;cross_count++;
     }
 }
-void Imu_Update(uint32_t now_ms)
+void Imu_Update(uint32_t now_ms, uint8_t wheels_still)
 {
-    uint32_t begin=Cycles();Advance(now_ms);CrossRead(now_ms);
+    uint32_t begin=Cycles();Advance(now_ms,wheels_still);CrossRead(now_ms);
     GyroCheck_Tick(&gyro_check,now_ms,state==IMU_FAULT);
     uint32_t us=(uint32_t)(Cycles()-begin)/(SystemCoreClock/1000000U);
     if (us>max_update_us) max_update_us=us;
diff --git a/Core/Src/Modules/imu_mpu6050.h b/Core/Src/Modules/imu_mpu6050.h
index 02ab92f..766a96f 100644
--- a/Core/Src/Modules/imu_mpu6050.h
+++ b/Core/Src/Modules/imu_mpu6050.h
@@ -20,7 +20,7 @@ typedef struct {
 } ImuHeadingSample;
 void Imu_GetHeadingSample(uint32_t now_ms, ImuHeadingSample *sample);
 void Imu_Init(uint32_t now_ms);
-void Imu_Update(uint32_t now_ms);
+void Imu_Update(uint32_t now_ms, uint8_t wheels_still);
 void Imu_Calibrate(uint32_t now_ms);
 void Imu_Zero(void);
 void Imu_GyroTestBegin(uint32_t now_ms);
diff --git a/Core/Src/Modules/speed_control.c b/Core/Src/Modules/speed_control.c
index df891e9..f41ad64 100644
--- a/Core/Src/Modules/speed_control.c
+++ b/Core/Src/Modules/speed_control.c
@@ -211,6 +211,9 @@ static int32_t ParkingWheel(float reference, float measured, float dt, float ff,
     float pwm_limit=parking_mode==2U ? 65.0f : 40.0f;
     float integral_limit=parking_mode==2U ? 25.0f : 15.0f;
     float friction=parking_mode==2U ? 30.0f : 18.0f;
+    /* Preserve coarse-turn breakaway torque, but fade that feed-forward in
+     * the fine approach. A fixed 30% kick defeats a small yaw-rate command. */
+    if (parking_mode==2U) friction*=Clamp((reference<0.0f ? -reference : reference)/MOTION_TURN_MIN_CPS,0.0f,1.0f);
     float base=ff*reference+(reference>0.0f ? friction : -friction)+SPEED_KP*error;
     float next=Clamp(*integral+SPEED_KI*error*dt,-integral_limit,integral_limit);
     float proposed=base+next;
diff --git a/README.md b/README.md
index 966983e..152d61e 100644
--- a/README.md
+++ b/README.md
@@ -150,3 +150,118 @@ main：Cube 外设初始化
 场地尺寸已按确认值形成转角和名义距离初值；距离区间是首版试调设置。轮计数/mm仍是历史估计，需要针对轮胎和载荷复核。分段建立基准可隔离前段累计计数，不会消除已经形成的位置偏移、轮滑或比例误差。

 验收证据分开记录：本轮为实际算法代码、主机合成反馈测试和ARM构建；尚未烧录，未做板卡行为或实车路线验证。
+
+
+## IMU port from car-02- (2026-10-27)
+
+The MPU6050 transport/settings now follow `D:/STM32project/car-02-/Core/Src/mpu6050.c`:
+PB14/SCL and PB15/SDA, high-speed open-drain GPIO, 5 us software-I2C phases,
+DLPF 3, a 1 kHz sensor sample rate, +/-500 deg/s, and 65.5 LSB/(deg/s).
+The 5 ms control poll reads INT_STATUS (0x3A), requires DATA_RDY, then reads
+GYRO_ZOUT (0x47/0x48) separately. Initialization configures the awake sensor
+directly, without the previous extra device/signal-path resets. ACK checks,
+clock/transaction timeouts, identity/configuration readback and bus clear remain.
+
+Yaw calibration follows the reference `imu_filter.c` and its measured installed
+sensor bias: 400 fresh samples over at least 1995 ms, stopped drive, fresh
+stationary encoder feedback, Z rate within 10 deg/s of the -23 deg/s reference,
+and sample standard deviation <=0.8 deg/s. The reference only centers the gate;
+each boot estimates and subtracts its actual bias. Re-measure this reference
+after changing the sensor. Parameters are in `Core/Src/Modules/imu_config.h`.
+Z must remain vertical on the car. Acceleration X saturation remains visible in
+diagnostics; it does not certify a healthy six-axis device and no longer gates
+Z-only yaw calibration, matching the reference project. Sample gaps and gyro
+saturation still invalidate yaw; the mission still refuses/stops on invalid IMU.
+
+`imu_diag` is V2 (same 32-word layout): configuration words 26/27/28 are now
+0x08/0/3, raw gyro units are 65.5 LSB/(deg/s), and calibration rejection 2 means
+wheel feedback is moving/stale or drive active. `imu_raw_diag` is V3 (same
+64-word layout): the control sample contains status and Z, while other axes
+are updated by timestamped slow diagnostic transactions every ~250 ms. They
+are not a simultaneous six-axis frame. Diagnose addresses from the newly built
+map, not an old firmware's RAM offsets.
+
+Default flashing remains the FULL four-segment route. Build the updated image
+before flashing; an older ELF does not include source edits:
+
+```powershell
+.\scripts\build_mission.ps1 -Mode FULL
+.\scripts\test_mission.ps1
+```
+
+The new `imu_heading_test` covers the measured bias, both yaw directions,
++/-500 scaling, moving wheels/reference/noise rejection, insufficient calibration
+time, stale data, gyro rails, tick wrap, and the manual gyro-test scale.
+
+
+## 2026-10-27: turn approach, C/D capture and arrival feedback
+
+Existing cruise/rate/gain tuning values were deliberately retained for the user
+to change. `motion_tuning.h` now separates coarse turn breakaway speed from the
+fine approach floor (120 cps within the last 12 degrees), fades the turn friction
+feed-forward in that region, and adds a bounded yaw-rate braking lead (inside
+the existing 3 degree acceptance). Overshoot still needs real-car tuning; the
+bounded correction and motion fault checks remain enabled.
+
+A confirmed C/D handoff now guides forward capture toward the semicircle entry
+tangent (CB 0 degrees; DA 180 degrees). Capture tolerates a single contiguous
+wide stripe and a bounded 350 ms missing/ambiguous-line interval. A wide mask
+cannot certify capture: fresh centered narrow-line frames and an entry-heading
+error <=20 degrees are required. Normal following retains its 100 ms line-fault
+protection, and absolute capture time/distance bounds still apply.
+
+The C10B PA15 buzzer (active high, same mapping as car-02-) now accompanies the
+PC2 LED on each confirmed C/B/D/A event for 300 ms, without blocking control.
+The scheduler already advances a FULL route in the same confirmed-arrival tick;
+no second button press is needed. An unconfirmed C never produces an arrival.
+
+OLED now shows the specific segment fault (DISTANCE, CAPTURE TIME/DIST, NO LINE,
+LINE AMBIGUOUS, etc.). Row 6 contains P=last point, N=arrival count, M=decimal
+black-channel mask, C=endpoint confirmation frames. P:- N:0 means C has not been
+confirmed; P:C N:1 means C has been confirmed even if incoming CB then faults.
+The actual reason for the reported C stop remains unconfirmed until a matching
+runtime record or display reading is collected. Run after the failure, before
+resetting or starting again:
+
+```powershell
+.\scripts\read_mission.ps1 -AsJson
+```
+
+Suggested first trial changes, not calibrated or applied values (cps are encoder
+counts per second; current 6.73014 counts/mm makes 900 cps about 134 mm/s):
+
+| Parameter in Core/Inc/motion_tuning.h | Existing | Suggested trial |
+| --- | --- | --- |
+| MOTION_AC_CRUISE_CPS / MOTION_BD_CRUISE_CPS | 900 | 1500 |
+| MOTION_CB_CRUISE_CPS / MOTION_DA_CRUISE_CPS | 900 | 1200 |
+| MOTION_CB_CAPTURE_CPS / MOTION_DA_CAPTURE_CPS | 600 | 700 |
+| MOTION_TURN_RATE_MAX_DPS | 50 | 75 |
+| MOTION_TURN_ANGLE_KP | 2 | 3 |
+| MOTION_TURN_RATE_FF | 16 | about 8 for a 120-130 mm wheel track |
+| MOTION_LINE_PERIOD_MS | 50 | 20 when increasing arc speed |
+
+Adjust straight speeds first, then the turn group, then arc/capture speeds. Turn
+feed-forward is wheel cps per yaw deg/s, approximately
+`wheel_track_mm / 2 * counts_per_mm * pi / 180`; excessive feed-forward does not
+make actual yaw track its requested rate accurately. Keep angle tolerance at
+3 degrees initially instead of masking overshoot with a large tolerance.
+
+AC/BD endpoint windows remain 1000-1550 mm. Their FIRST black transition and
+all confirmation frames must fall inside the window. The forward-mounted gray
+bar reaches the line before the axle reaches the geometric point. If the
+transition occurs before 1000 mm, lower the corresponding POINT_MIN_MM using
+measured data; do not force completion on nominal distance or simply bypass
+line confirmation. Raising speed does not repair a missed endpoint.
+
+New regression cases cover predictive braking in both turn directions, fine
+approach targets, C/D tangent steering, wide-stripe capture without premature
+promotion, capture search deadlines, and unchanged FOLLOW ambiguity protection.
+Rebuild FULL and run the host tests before flashing a newly adjusted image.
+
+
+Validation for this edit: changed controller/port sources and expanded motion
+tests compile to Cortex-M3 objects with the installed ARM Clang. Full GNU
+firmware linking and execution of the updated regression suite remain blocked
+inside the current tool environment by Illegal System DLL Relocation. The
+previous 12/12 host result predates these motion changes. Run build_mission and
+test_mission on the host again; the existing ELF is not the updated image.
diff --git a/tests/CMakeLists.txt b/tests/CMakeLists.txt
index 615792f..15b9b06 100644
--- a/tests/CMakeLists.txt
+++ b/tests/CMakeLists.txt
@@ -31,6 +31,7 @@ endforeach()
 mission_test(mission_runtime_test test_mission_runtime.c mission_fake_segments.c ${RUNTIME})
 mission_test(motion_algorithms_test test_motion_algorithms.c ${MOTION} ${SEGMENTS} ${MISSION_DIR}/mission_control.c)
 mission_test(mission_actual_route_test test_mission_actual_route.c ${RUNTIME} ${SEGMENTS})
+mission_test(imu_heading_test test_imu_heading.c ${MODULE_DIR}/imu_heading.c)
 mission_test(encoder_wrap_test test_encoder_wrap.c ${MODULE_DIR}/encoder.c)
 target_include_directories(encoder_wrap_test BEFORE PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/stubs)
 find_program(MISSION_POWERSHELL NAMES pwsh powershell)
diff --git a/tests/test_motion_algorithms.c b/tests/test_motion_algorithms.c
index ac38f33..5e31f27 100644
--- a/tests/test_motion_algorithms.c
+++ b/tests/test_motion_algorithms.c
@@ -96,12 +96,12 @@ static void ArcTests(void)
     MissionInput in=Fresh(); in.line_mask=0x18U; in.line_strength[3]=in.line_strength[4]=1000U;
     MissionSegmentContext ctx=Context(&in);
     MissionSegmentOutput out=MotionArc_Enter(&ctx,&in,p);
-    assert(ctx.phase==PHASE_CAPTURE && out.left_cps==600.0f && out.right_cps==600.0f);
+    assert(ctx.phase==PHASE_CAPTURE && out.left_cps==p->capture_cps && out.right_cps==p->capture_cps);
     for(uint32_t i=0;i<100U;++i) (void)MotionArc_Update(&ctx,&in,p);
     assert(ctx.motion.capture_frames==1U);
     Frame(&in,0x18U); (void)MotionArc_Update(&ctx,&in,p);
     Frame(&in,0x18U); out=MotionArc_Update(&ctx,&in,p);
-    assert(ctx.phase==PHASE_FOLLOW && out.left_cps==900.0f);
+    assert(ctx.phase==PHASE_FOLLOW && out.left_cps==p->cruise_cps);
     Frame(&in,0x20U); in.now_ms+=20U; out=MotionArc_Update(&ctx,&in,p);
     assert(out.left_cps>out.right_cps); /* black right -> turn right */
     Frame(&in,0x04U); in.now_ms+=20U; out=MotionArc_Update(&ctx,&in,p);
@@ -159,6 +159,68 @@ static void TurnFailureTests(void)
     out=MotionStraight_Update(&ctx,&in,p);
     assert(out.mode==MOTION_TURN && ctx.motion.corrections==3U);
 }
+static void TurnApproachTests(void)
+{
+    const MissionSegmentParams *p=mission_segment_ac.params;
+    for (int sign=-1;sign<=1;sign+=2) {
+        MissionInput in=Fresh();in.yaw_deg=p->target_yaw_deg-(float)sign*5.0f;
+        MissionSegmentContext ctx=Context(&in);
+        (void)MotionStraight_Enter(&ctx,&in,p);(void)MotionStraight_Update(&ctx,&in,p);
+        in.now_ms=500U;MissionSegmentOutput out=MotionStraight_Update(&ctx,&in,p);
+        assert(out.mode==MOTION_TURN);
+        assert(fabsf(out.left_cps)<MOTION_TURN_MIN_CPS && fabsf(out.right_cps)<MOTION_TURN_MIN_CPS);
+        assert(out.left_cps*(float)sign<0.0f && out.right_cps*(float)sign>0.0f);
+        /* Moving away must not receive predictive early braking. */
+        in.yaw_deg=p->target_yaw_deg-(float)sign*2.8f;in.yaw_rate_dps=-(float)sign*40.0f;
+        in.now_ms+=20U;out=MotionStraight_Update(&ctx,&in,p);assert(out.mode==MOTION_TURN);
+        /* Moving towards the target brakes inside tolerance before crossing. */
+        in.yaw_rate_dps=(float)sign*40.0f;in.now_ms+=20U;
+        out=MotionStraight_Update(&ctx,&in,p);
+        assert(out.mode==MOTION_BRAKE && ctx.phase==PHASE_SETTLE);
+        in.yaw_rate_dps=0.0f;in.now_ms+=20U;(void)MotionStraight_Update(&ctx,&in,p);
+        in.now_ms+=MOTION_SETTLE_MS;out=MotionStraight_Update(&ctx,&in,p);
+        assert(out.mode==MOTION_FORWARD && ctx.phase==PHASE_STRAIGHT && !ctx.motion.corrections);
+    }
+}
+static void CaptureHandoffTests(void)
+{
+    const MissionSegmentParams *p=mission_segment_cb.params;
+    MissionInput in=Fresh();Frame(&in,0xFFU);in.yaw_deg=MOTION_AC_TARGET_DEG;
+    MissionSegmentContext ctx=Context(&in);ctx.from_handoff=1U;
+    MissionSegmentOutput out=MotionArc_Enter(&ctx,&in,p);
+    assert(out.status==SEG_RUNNING && ctx.phase==PHASE_CAPTURE);
+    assert(out.left_cps>0.0f && out.left_cps<out.right_cps); /* CB entry: steer left. */
+    assert(ctx.motion.capture_frames==0U); /* Wide stripe cannot certify capture. */
+    for (uint32_t i=0;i<3U;i++) {
+        Frame(&in,0x18U);out=MotionArc_Update(&ctx,&in,p);
+        assert(ctx.phase==PHASE_CAPTURE && !ctx.motion.capture_frames); /* Still diagonal. */
+    }
+    in.yaw_deg=0.0f;
+    for (uint32_t i=0;i<MOTION_CAPTURE_CONFIRM_FRAMES;i++) {
+        Frame(&in,0x18U);out=MotionArc_Update(&ctx,&in,p);
+    }
+    assert(ctx.phase==PHASE_FOLLOW && out.status==SEG_RUNNING);
+    /* Wide masks still fault during normal following. */
+    Frame(&in,0xFFU);out=MotionArc_Update(&ctx,&in,p);assert(out.status==SEG_RUNNING);
+    in.now_ms+=MOTION_LINE_UNCERTAIN_MS;out=MotionArc_Update(&ctx,&in,p);
+    assert(out.status==SEG_FAULT && out.fault==MOTION_FAULT_AMBIGUOUS);
+
+    p=mission_segment_da.params;in=Fresh();Frame(&in,0xFFU);in.yaw_deg=MOTION_BD_TARGET_DEG;
+    ctx=Context(&in);ctx.from_handoff=1U;out=MotionArc_Enter(&ctx,&in,p);
+    assert(out.status==SEG_RUNNING && out.left_cps>out.right_cps && out.right_cps>0.0f); /* DA: right. */
+    Frame(&in,0U);out=MotionArc_Update(&ctx,&in,p);
+    assert(out.status==SEG_RUNNING);
+    in.now_ms+=MOTION_LINE_UNCERTAIN_MS;out=MotionArc_Update(&ctx,&in,p);
+    assert(out.status==SEG_RUNNING); /* Capture has its own bounded search grace. */
+    in.now_ms=ctx.motion.uncertain_ms+MOTION_CAPTURE_UNCERTAIN_MS;
+    out=MotionArc_Update(&ctx,&in,p);assert(out.status==SEG_FAULT && out.fault==MOTION_FAULT_NO_LINE);
+
+    /* A wide stripe can never bypass the absolute capture timeout. */
+    in=Fresh();Frame(&in,0xFFU);ctx=Context(&in);ctx.from_handoff=1U;
+    (void)MotionArc_Enter(&ctx,&in,p);
+    in.now_ms=ctx.phase_started_ms+MOTION_CAPTURE_TIMEOUT_MS+1U;
+    out=MotionArc_Update(&ctx,&in,p);assert(out.status==SEG_FAULT && out.fault==MOTION_FAULT_CAPTURE);
+}
 static void ConfigTests(void)
 {
     MissionSegmentParams p=*mission_segment_ac.params;
@@ -174,7 +236,7 @@ static void ConfigTests(void)
 }
 int main(void)
 {
-    assert(Motion_ValidateTuning()); EdgeTests(); DistanceTests(); StraightTests(); ArcTests(); TurnFailureTests(); ConfigTests();
+    assert(Motion_ValidateTuning()); EdgeTests(); DistanceTests(); StraightTests(); ArcTests(); TurnFailureTests(); TurnApproachTests(); CaptureHandoffTests(); ConfigTests();
     assert(Motion_WrapDegrees(218.66f)<0.0f);
     puts("actual motion: edge windows, distinct frames, per-leg distance, turn/heading/line PD and faults passed");
     return 0;
diff --git a/Core/Src/Modules/imu_config.h b/Core/Src/Modules/imu_config.h
new file mode 100644
--- /dev/null
+++ b/Core/Src/Modules/imu_config.h
@@ -0,0 +1,23 @@
+#ifndef IMU_CONFIG_H
+#define IMU_CONFIG_H
+
+/* MPU6050 settings ported from car-02-/Core/Src/mpu6050.c. */
+#define IMU_GYRO_CONFIG 0x08U /* FS_SEL=1, +/-500 degrees/second */
+#define IMU_GYRO_LSB_PER_DPS 65.5f
+#define IMU_GYRO_MAX_ABS_DPS 490.0f
+#define IMU_SAMPLE_DIVIDER 0U /* DLPF=3: 1 kHz sensor, 5 ms host polling */
+#define IMU_DLPF_CONFIG 3U
+#define IMU_POLL_MS 5U
+#define IMU_MAX_SAMPLE_GAP_MS 30U
+
+/* Installed sensor's measured stationary reference, from car-02-.
+ * This ONLY centers the calibration gate. Each boot measures its own bias.
+ * Re-measure this reference after replacing the sensor. Raw sensor sign is
+ * retained here; MOTION_YAW_SIGN is applied once by the mission motion controller. */
+#define IMU_Z_BIAS_PRIOR_DPS (-23.0f)
+#define IMU_CAL_WINDOW_DPS 10.0f
+#define IMU_CAL_SAMPLES 400U
+#define IMU_CAL_MIN_MS ((IMU_CAL_SAMPLES - 1U) * IMU_POLL_MS)
+#define IMU_CAL_STD_DPS 0.8f
+
+#endif
diff --git a/tests/test_imu_heading.c b/tests/test_imu_heading.c
new file mode 100644
--- /dev/null
+++ b/tests/test_imu_heading.c
@@ -0,0 +1,95 @@
+#include <assert.h>
+#include <math.h>
+#include <stdio.h>
+#include "imu_heading.h"
+#include "imu_config.h"
+
+static const int16_t bias_raw=-1520; /* -23.2061 dps at +/-500 dps */
+static uint32_t Calibrate(ImuHeading *h, uint32_t start)
+{
+    ImuHeading_Reset(h);
+    for (uint32_t i=0;i<IMU_CAL_SAMPLES;i++) {
+        uint32_t result=ImuHeading_Sample(h,bias_raw,1U,start+i*IMU_POLL_MS);
+        assert(result==(i+1U==IMU_CAL_SAMPLES ? HEADING_READY : HEADING_CALIBRATING));
+    }
+    assert(h->ready && h->samples==IMU_CAL_SAMPLES);
+    assert(fabsf(h->bias_raw-(float)bias_raw)<0.01f);
+    return start+(IMU_CAL_SAMPLES-1U)*IMU_POLL_MS;
+}
+static void TestMeasuredBiasAndScale(void)
+{
+    ImuHeading h;
+    uint32_t now=Calibrate(&h,0U);
+    assert(fabsf(h.bias_raw/IMU_GYRO_LSB_PER_DPS-IMU_Z_BIAS_PRIOR_DPS)>0.1f);
+    for (uint32_t i=0;i<200U;i++) {
+        now+=IMU_POLL_MS;
+        assert(ImuHeading_Sample(&h,(int16_t)(bias_raw+5895),0U,now)==HEADING_READY);
+    }
+    assert(fabsf(h.rate_dps-90.0f)<0.01f);
+    assert(fabsf(h.yaw_deg-90.0f)<0.3f);
+    ImuHeading_Zero(&h);assert(h.yaw_deg==0.0f);
+    now+=IMU_POLL_MS;
+    assert(ImuHeading_Sample(&h,(int16_t)(bias_raw-5895),0U,now)==HEADING_READY);
+    assert(fabsf(h.rate_dps+90.0f)<0.01f);
+}
+static void TestStationaryAndReferenceGates(void)
+{
+    ImuHeading h;ImuHeading_Reset(&h);
+    for (uint32_t i=0;i<500U;i++) ImuHeading_Sample(&h,bias_raw,0U,i*5U);
+    assert(!h.ready && !h.samples && h.reject_reason==2U);
+    for (uint32_t i=0;i<500U;i++) ImuHeading_Sample(&h,0,1U,2500U+i*5U);
+    assert(!h.ready && !h.samples && h.reject_reason==1U);
+    for (uint32_t i=0;i<200U;i++) ImuHeading_Sample(&h,bias_raw,1U,5000U+i*5U);
+    assert(h.samples==200U);
+    ImuHeading_Sample(&h,bias_raw,0U,6000U);
+    assert(!h.samples && !h.ready);
+}
+static void TestVarianceGapAndRails(void)
+{
+    ImuHeading h;ImuHeading_Reset(&h);
+    for (uint32_t i=0;i<400U;i++)
+        ImuHeading_Sample(&h,(int16_t)(bias_raw+(i%2U ? 100 : -100)),1U,i*5U);
+    assert(!h.ready && !h.samples && h.reject_reason==1U);
+    ImuHeading_Reset(&h);
+    ImuHeading_Sample(&h,bias_raw,1U,0U);
+    ImuHeading_Sample(&h,bias_raw,1U,5U);
+    ImuHeading_Sample(&h,bias_raw,1U,40U);
+    assert(h.samples==1U && h.restarts==1U && h.reject_reason==3U);
+    uint32_t now=Calibrate(&h,100U);
+    assert(ImuHeading_Sample(&h,bias_raw,0U,now+31U)==HEADING_GAP);
+    assert(!h.ready);
+    now=Calibrate(&h,100U);
+    assert(ImuHeading_Sample(&h,-32768,0U,now+5U)==HEADING_SATURATED);
+    assert(!h.ready);
+    ImuHeading_Reset(&h);
+    assert(ImuHeading_Sample(&h,32767,1U,0U)==HEADING_SATURATED);
+    assert(!h.ready);
+}
+static void TestTimingAndWrap(void)
+{
+    ImuHeading h;
+    uint32_t now=Calibrate(&h,UINT32_MAX-1000U);
+    uint32_t count=h.samples;
+    assert(ImuHeading_Sample(&h,bias_raw,0U,now)==HEADING_READY);
+    assert(h.samples==count && h.yaw_deg==0.0f);
+    ImuHeading_Reset(&h);
+    for (uint32_t i=0;i<400U;i++) ImuHeading_Sample(&h,bias_raw,1U,i);
+    assert(!h.ready); /* 400 samples alone cannot bypass stationary duration. */
+}
+static void TestManualGyroScale(void)
+{
+    GyroCheck h;GyroCheck_Begin(&h,0U);
+    int16_t axes[3]={0,0,bias_raw};
+    for (uint32_t now=0U;now<=4990U;now+=10U) GyroCheck_Sample(&h,axes,now);
+    assert(h.state==GYRO_TEST_READOUT && fabsf(h.bias-(float)bias_raw)<0.01f);
+    axes[2]=(int16_t)(bias_raw+655);
+    GyroCheck_Sample(&h,axes,5000U);
+    assert(fabsf(h.rate-10.0f)<0.01f);
+}
+int main(void)
+{
+    TestMeasuredBiasAndScale();TestStationaryAndReferenceGates();
+    TestVarianceGapAndRails();TestTimingAndWrap();TestManualGyroScale();
+    puts("IMU: measured bias, +/-500 scale, stationary/noise gates, gap, rail and clock wrap passed");
+    return 0;
+}
````

## 14. 源码 SHA-256 清单

用于核对这份文档与当前工作树是否一致；未来修改后哈希不同是正常的。

| 相对路径 | SHA-256 |
| --- | --- |
| `Core/Inc/motion_tuning.h` | `9bd63e59becf01206c8e76c3665d8364ffef651ba22864322533433925094b62` |
| `Core/Src/Mission/mission_config.h` | `2271777fc4037a01538948e592f09e04a752299ed80e3b7cbdcd3b7075dcfc68` |
| `Core/Src/Mission/mission_port_stm32.c` | `448374f5fdd421eae7c031ce856bd0e62104fc84d2cfc2681578b4ba912a9290` |
| `Core/Src/Mission/motion_control.c` | `b6ebe06228bd6bdbf1417dde3bc1b140db37aacef6f1c0cca502b7069287f4b1` |
| `Core/Src/Mission/motion_line.c` | `308d08c49d53e79df733df2a4af8df66fdbd05d244b733ce3bf9ac68c58d954a` |
| `Core/Src/Mission/motion_straight.c` | `7d2d17da1d16c3907851120c87721a704262a3546e52923ad0c8d94900b77867` |
| `Core/Src/Modules/imu_heading.c` | `33d430d5650f260205dd86bb101568185596703f166caaa6e044841fcc92d04c` |
| `Core/Src/Modules/imu_heading.h` | `9edb72008ea69ec71bda8f69df686f3cec25c39db82f5e47e0d68d6f9e8c337e` |
| `Core/Src/Modules/imu_mpu6050.c` | `ee704d16262e036019072dbffda4c2efa82b2c0599c2547f2e947f89a82b858c` |
| `Core/Src/Modules/imu_mpu6050.h` | `5262c105145115d8e8e6b79814d087232b7ce4b657135cbbead0340e4f3e3ecf` |
| `Core/Src/Modules/speed_control.c` | `4d069ac372253ff60ad0f60692b29435eb7fb41fc9fc8ab7573f7f486a143f5c` |
| `tests/CMakeLists.txt` | `f6e88d8fb203d86f1601cdd6694cab76f61777da72cef93728f26f03a56ef7e1` |
| `tests/test_motion_algorithms.c` | `7c573fa14a0c05c0804fb761ea1d3a1633733d90067adefbb126c71a3c32a3fd` |
| `README.md` | `8950171aed4376e84d86982598b9d6e90ed1032bbd39da53774299e57dc37dca` |
| `Core/Src/Modules/imu_config.h` | `0aa9450719a03ce6bb8105a34078ce48059e5e4dee0f2dffb8b0b13725922a00` |
| `tests/test_imu_heading.c` | `9de93bf2a6e245910e179f61ef0a9d5067a23e393f5c66c1c00e49d35a7c41bb` |

## 15. 可直接交给下一位 AI 的任务提示

> 请先阅读本 Markdown，打开 `D:\STM32project\car-for2`。当前是 C10B/STM32F103RCT6 的 FULL 四路段路线 A→C→B→D→A。IMU 已参考 `D:\STM32project\car-02-` 改为 ±500 deg/s、Z-only 采样和每次静止实测零偏。上一版用户本机 12/12 主机测试通过，但之后又修改了转向末段、C/D 入弧、声光提示和 OLED；最新这一批还没有完整构建、回归和实车成功证据。用户要自行修改已有参数，AI 负责代码逻辑。请先查看实际工作树和用户是否已调参，重新构建测试并核对板上固件。针对“车慢、A 转向过冲、C 后不继续”，先收集匹配固件的 mission_diag 或 OLED，区分 C 未识别与 CB 捕获失败，不要直接跳过到点识别或关闭保护。修改后说明改动、验证证据和仍未知之处。
