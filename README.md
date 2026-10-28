# 小车单任务：A → C → B → D → A

当前版本已写入四段实际运动算法和集中调参表。上电保持制动，按键或 API 明确启动后才运行；参数采用首版试调初值，尚未进行板卡或实车验证。四段均为 `implemented=1`，启动前检查参数、IMU、编码器和灰度数据。

确认的场地尺寸为 AB=1000 mm、AD=BC=800 mm，两侧半径400 mm。A处车头水平朝右，先原地右转38.66°，再沿AC直行；B处允许停车后原地左转。C、D连续交接进入半弧巡线，最终回到A制动停车。

## 运动参数：统一在这里修改

**所有主要运动调参入口：`Core/Inc/motion_tuning.h`。修改后重新构建。** 参数按编号分组，四段文件只绑定该段参数和共用控制器。

| 路段 | 目标方向及动作 | 基础速度 | 名义距离 | 端点区间（含边界） |
| --- | --- | --- | --- | --- |
| AC | 朝右水平为0°，右转到−38.66°后直行 | 900 cps | 1281 mm | 1000～1550 mm |
| CB | 右半弧，目标出口朝左 | 捕线600 / 巡线900 cps | 1257 mm | 1000～1500 mm |
| BD | 朝左水平为180°，左转到213.66°后直行 | 900 cps | 1281 mm | 1000～1550 mm |
| DA | 左半弧，目标出口朝右 | 捕线600 / 巡线900 cps | 1257 mm | 1000～1500 mm |

方向采用左转为正，直行目标锚定整条路线的起始水平参考。BD的213.66°等价于−146.34°；控制器将角度误差归一到±180°，不会把B处的航向偏差直接作为新目标。B点标称折角是38.66°，但2026-10-28第六次上板实测车头比黑线多偏约5°，所以 `MOTION_BD_CORNER_OFFSET_DEG` 取 −5°，该处左转从38.66°缩到33.66°（见参数表最后一行）。

| 在调参头文件内搜索 | 初值及单位 | 调整作用 |
| --- | --- | --- |
| `MOTION_AC_*`、`BD_*`、`CB_*`、`DA_*` | 上表对应的速度、角度、名义距离及 `POINT_MIN_MM / POINT_MAX_MM` | 每段分别设置；扩大区间会放宽端点识别范围，名义距离只用于显示 |
| `MOTION_LEFT_COUNTS_PER_MM / RIGHT_COUNTS_PER_MM` | 两侧均6.730141612 counts/mm | 历史轮校准值；增大后相同计数显示的距离减小，左右可以分别标定 |
| `MOTION_SPEED_KP / KI` | 0.010 / 0.030 | 轮速PI；Kp增大加快纠偏，Ki消除持续误差，过大可能振荡 |
| `MOTION_PWM_LIMIT / SPEED_INTEGRAL_LIMIT` | 正向PWM上限65% / 积分限幅20 | 约束正向输出；转向专用驱动限幅沿用原有值 |
| `MOTION_TARGET_RAMP_CPS_S` | 5000 cps/s | 增大后加减速更快 |
| `MOTION_HEADING_KP / KD / TRIM_MAX_CPS` | 10 / 3 / 150 cps | 直行航向角度纠偏、角速度阻尼和差速修正上限 |
| `MOTION_HEADING_FAULT_DEG` | 15° | 直行偏航超过阈值即故障停车 |
| `MOTION_TURN_ANGLE_KP / RATE_FF / RATE_KP` | 3.2 / 16 / 6 | 原地转向的角度增益、角速度前馈和角速度反馈 |
| `MOTION_TURN_RATE_MAX_DPS / MIN_CPS / MAX_CPS` | 65°/s / 350 / 1400 cps | 转向角速度及左右轮速度限幅 |
| `MOTION_TURN_BRAKE_DEG / ACCEPT_DEG` | 2° / 3° | 进入制动及稳定后允许进入直行的角度误差 |
| `MOTION_STILL_MS / MOTION_SETTLE_MS` | 均 200 ms | 入段静止确认 / 转到位后停稳确认的连续时长 |
| `MOTION_STILL_RATE_DPS / STILL_WHEEL_CPS` | 3°/s / 60 cps | 判定"车已静止"的角速度与轮速阈值 |
| `MOTION_TURN_MAX_CORRECTIONS / TIMEOUT_MS` | 3次 / 25000 ms | 转向修正次数和总超时（包含等待稳定） |
| `MOTION_LINE_KP / KD / D_FILTER_S` | 0.18 / 0.006 / 0.060 s | 旧巡线PD及微分滤波；位置单位为探头坐标−3500～+3500 |
| `MOTION_LINE_CURVATURE_FF` | −0.1625 | 半圆曲率前馈：巡航速度的这个比例直接做成差速，PD 只补扰动。负号对应逆时针赛道（圆弧左转、右轮领先），顺时针跑要改成正号 |
| `MOTION_LINE_STEERING_MAX_CPS` | 650 cps | 巡线差速上限，实际仍受最低轮速和总轮速上限约束 |
| `MOTION_LINE_HIGH_* / CORNER_BASE_CPS / MIN_WHEEL_CPS` | 沿用旧巡线速度分段公式 | 调整高速/捕线状态的基础速度和轮速下限 |
| `MOTION_CAPTURE_CENTER_LIMIT / CONFIRM_FRAMES` | 3000位置单位 / 3个新帧 | 捕线转入正常巡线的中心条件。放宽到阵列内任意位置：滚着切入时线先出现在阵列边缘，旧的 ±1000 会把这些帧全部丢掉 |
| `MOTION_CAPTURE_TIMEOUT_MS / MAX_MM` | 2000 ms / 320 mm | 捕线任一条件超限即故障；必须覆盖"过点后线暂时不在探头下"的盲区 |
| `MOTION_STRAIGHT_HANDOFF_CPS / POINT_APPROACH_MM` | 1200 cps / 140 mm | 直线段在距**标称点** 140 mm 内降到该速度，把弯道入段所需的轮速差速留给转向。不要低于 1000 cps：600 cps 的占空比只有约 9%，低于持续滚动所需，车会停下并报 SPEED_FAULT |
| `MOTION_CAPTURE_HEADING_KP / ACCEPT_DEG / LOOKAHEAD_MM` | 10 / 20° / 150 mm | 未见可用线时按已知半圆和前视距离引导；宽判点条保留入口航向修正。看到可用窄线后由巡线 PD 加曲率前馈接管，入口航向门限只用于确认进入 FOLLOW |
| `MOTION_CAPTURE_LINE_PERIOD_MS` | 10 ms | C/D 滚动捕获看到窄线时的控制最小间隔，第一帧立即响应；增大会延迟扫线纠偏，减小到灰度帧间隔以下不会增加有效观测。FOLLOW 仍为 50 ms |
| `MOTION_POINT_CONFIRM_FRAMES` | 3个新帧 | 黑白状态与端点跳变确认，重复读取同一帧不累加 |
| `MOTION_BD_CORNER_OFFSET_DEG` | −5° | B 点转向角的实测修正：BD 目标航向 = 180+38.66+该偏移 = 213.66°，即 B 处左转 33.66°。只影响 B 点；换赛道或改转向参数后先用 `Heading_error_deg` 复核 |
| `MOTION_LINE_UNCERTAIN_MS` | 100 ms | 丢线、歧义的暂时恢复时限 |
| `MOTION_DRIVE_TIMEOUT_MS / ARC_TIMEOUT_MS` | 均30000 ms | 直行阶段/整个半弧段超时 |
| `MOTION_HEADING_PERIOD_MS / LINE_PERIOD_MS / ENCODER_PERIOD_MS` | 20 / 50 / 10 ms | 外层控制及轮速采样周期；改变周期后需复核控制效果 |
| `MOTION_YAW_SIGN / LINE_ORDER` | +1 / `{0,1,2,3,4,5,6,7}` | 陀螺仪正方向、原始通道按车体左→右排列的映射；默认恢复旧程序约定，需实车核对 |

例如只改AC速度和C点识别范围：修改 `MOTION_AC_CRUISE_CPS`、`MOTION_AC_POINT_MIN_MM`、`MOTION_AC_POINT_MAX_MM`。同一轮先改一组参数，观察分段距离、航向误差、轮速目标和停止原因后再继续调整。

## 代码入口与职责

```text
main：Cube 外设初始化
  └─ Mission_Init / Mission_Update
       ├─ mission_port_stm32：集中采样、按键 GPIO、执行电机目标、OLED、到点提示
       └─ mission_control：纯状态机，只读快照、输出命令
            ├─ segment_ac：A → C，空白直行
            ├─ segment_cb：C → B，右侧半弧
            ├─ segment_bd：B → D，空白直行
            └─ segment_da：D → A，左侧半弧
```

任务代码集中于 `Core/Src/Mission/`：

| 文件 | 负责内容 |
| --- | --- |
| `mission.h / mission.c` | `Mission_Init(now)`、`Mission_Update(now)`、`Mission_Start(now)`、`Mission_Stop(now)`；按键、诊断及唯一执行出口 |
| `mission_control.h / .c` | 路线预检、四段调度、交接、完成及故障处理，不依赖 HAL |
| `mission_segment.h` | 快照、路段上下文、参数、输出和 `Enter / Update / Exit` 契约 |
| `mission_config.h` | 编译模式、目标速度上限、传感器时效和校准长按时间 |
| `segment_ac.c / cb.c / bd.c / da.c` | 各段独立参数绑定，分别使用共用直线/巡线控制器，四段均已实现 |
| `motion_straight.c / motion_line.c` | 原地转向、航向保持直行、旧巡线位置计算与PD、捕线及恢复 |
| `motion_distance.c / motion_edge.c` | 按段计距、严格距离区间内的灰度跳变确认 |
| `motion_drive.c` | 唯一运动执行适配器，管理前进、转向和制动模式切换 |
| `mission_port.h / mission_port_stm32.c` | 硬件边界；主机测试替换这一层，仍执行正式调度器和速度控制代码 |

共用驱动位于 `Core/Src/Modules/`，保留电机、编码器、速度 PI、按键消抖、灰度 UART、黑白校准及 Flash 存储、IMU、OLED。蓝牙底层 `bt_uart` 源码保留但不编译、不初始化；蓝牙模式协议和旧任务调度已移除。

旧任务 `car/task2/task4/mode2`、其专用控制及诊断记录、旧计圈/位置停车/巡线任务包装和更早的重复应用源码已删除。删除前的源码归档在本地 `backups/before-mission-refactor-20261026.zip`。既有备份和日志保留；`backups/`、`logs/` 和 `build/` 不纳入版本控制。

## 路段接口与交接规则

- `Enter(ctx, input, params)`：建立路段局部状态，返回第一组有效左右轮目标。只能返回运行中、故障或未实现；不能在入口直接宣告完成。
- `Update(ctx, input, params)`：返回 `SEG_RUNNING / SEG_COMPLETE / SEG_FAULT / SEG_UNIMPLEMENTED`，运行时带左右轮目标，单位为**编码器计数/秒（cps）**。故障可附带路段专用错误码。
- `Exit(ctx, input, reason)`：只清理内部状态，不调用电机、编码器或 IMU。完成、手动停止、故障分别传入不同退出原因，每次激活最多退出一次。
- 每段拥有独立上下文和参数。上下文含入口快照、活动状态及局部阶段；后续新增本段参数和状态应在该段维护，需要扩展共享上下文时显式更新接口。
- 每轮先各更新一次编码器、灰度及 IMU，再生成一份不可修改快照。出段和入段读取同一份快照；路段只计算目标，由任务统一执行。
- 运动输出明确区分：前进时两轮均 `0 < cps <= 3200`；原地转向时两轮反向且绝对值≤1200 cps；制动时两轮目标均为零。非法模式、符号和NaN/无穷大目标被拒绝。
- 全程启动前检查四段描述符及 `implemented`，单段模式只检查所选路段。任何必需段未实现均在电机启动前拒绝运行。
- AC、BD执行 `STILL → TURN → SETTLE → STRAIGHT`，静止条件需连续满足至少 `MOTION_STILL_MS`(200 ms)；转到位后同样等 `MOTION_SETTLE_MS`(200 ms) 再起线，两者都由"角速度≤3°/s 且两轮<60 cps"把关，10 ms 采样下已是 20 个连续样本。角度已满足时可直接进入稳定阶段。转向最多修正3次，保留方向、平移、双轮同步及陀螺仪响应保护。
- 直线计距在转向及制动稳定结束后建立基准；半弧计距在接管时建立，包含捕线过程。共用距离公式为 `0.5 * ((L-L0)/kL + (R-R0)/kR)`，先作整数差再换算毫米，左右轮有符号行程取平均。每段持有独立基准，编码器生产者和全局累计值持续保留。
- C、D交接时仅修改正向目标，保留编码器、航向、滤波、PI及速度斜坡；B处先制动，再原地转向，允许重建速度环。既有 `SpeedControl_StartDifferential()` 的重置语义保留，新任务仅使用保留计数的启动接口。
- 四段都要求有效航向、编码器和已校准的新鲜灰度数据。AC、BD允许全白，仍需灰度通信正常；单段CB、DA启动要求灰度排在线上。校准期间不允许运动；故障后需明确重新启动，速度环锁存故障需复位后重新检查。
- 直行段稳定白→稳定黑分别识别C、D；半弧段稳定黑→稳定白分别识别B、A。端点区间包含上下限，跳变第一帧和全部确认帧均须在区间内。窗口外的跳变不保留到窗口内；超过距离上限仍未确认则故障停车。
- 半弧先 `CAPTURE` 后 `FOLLOW`：捕获期沿**已知半圆**边滚边转（死推算位姿 + 前视修正，误差 >8° 时饱和打满），连续3个新帧进入探头阵列后升至巡线速度。捕获期丢线不算故障——车刚越过判点黑条、圆弧线还在侧面是正常现象，只有 2000 ms / 320 mm 的捕获预算耗尽才报 `CAPTURE`；进入 `FOLLOW` 后短暂丢线保持方向，100 ms 仍未恢复则故障；合法出口识别优先于普通丢线保护。灰度过期不能作为白色到点事件。任务按传感器事件切段，名义距离和定时器都不单独宣告到点。

## 启停、显示与到点事件

- USER 按键：PA0，高电平按下；上电时按住必须先释放。空闲短按在消抖释放后请求启动，运行中按下立即请求停车，停车后的释放不会再次启动。
- 空闲按住约 6 秒后释放进入灰度黑白校准；校准中再按一次取消。保留原校准过程及存储格式；校准和等待期间持续更新传感器。
- OLED显示模式、当前路段、任务状态、停止原因、内部阶段、分段距离、识别区间、目标航向、航向误差、端点确认帧数和到点次数；校准状态在末行显示。
- 每个路段完成时总调度器发出一次到点事件。全程顺序为 **C、B、D、A**；初始放在 A 不发事件。事件与是否能继续进入下一段分别处理：例如到 C 后下一段传感器失效，仍保留 C 到达事件并停车。
- 到点 LED1（PC2，低有效）非阻塞点亮 150 ms。蜂鸣器未绑定引脚，本轮只预留 `MissionPort_NotifyPoint()` 接口，尚无声音输出。（2026-10-27 更新：PA15 蜂鸣器已接入，见下文同名日期小节。）
- `mission_diag` 升级为152字节V3（`version=3`），保留原有模式、状态、目标、传感器和事件字段，新增内部阶段、执行模式、分段距离、识别区间、目标航向/误差（毫度）、巡线位置、确认帧数和转向修正次数；V3 再加圈数 `lap` 与 A 点漂移残差 `drift_mdeg`。读取器同步升级，旧版本记录不会被误读。常规20 ms发布，阶段和命令变化当轮发布。`arrivals` 为本次任务到点数，`event_count` 为本次上电累计事件数；头尾序号须相等且为偶数。
- 比赛任务为四圈：FULL 模式跑完 A→C→B→D→A 后不直接结束，而是回到 AC 再跑，共 `MISSION_FULL_LAPS`（4）圈才进 `DONE`。每圈回到 A 点时用当前航向重取航向基准（A 点是课程中唯一已知航向），残差累加进 `drift_mdeg`，既抵消陀螺零漂又留下证据。单段固件（AC/CB/BD/DA）仍只跑一段；台架可用 `Mission_SetLaps()` 或把 `MISSION_FULL_LAPS` 改 1 做单圈调试。
- 直行航向误差参与纠偏；半弧的航向误差显示为当前方向到出口切线方向的角度差，用于观察轨迹，转向输出仍由灰度PD决定。

## 构建与单段选择

已有 `build/Debug`、`build/Release` 产物保持原样。新构建使用 `build/Mission-*`，输出文件名仍为 `C10B_Test.elf`。以下 PowerShell 命令只编译，不烧录：

```powershell
# 全程 Debug / Release
.\scripts\build_mission.ps1
.\scripts\build_mission.ps1 -Configuration Release

# 单段 Debug；将 AC 替换为 CB、BD、DA 即可
.\scripts\build_mission.ps1 -Mode AC
```

全程目录为 `build/Mission-Debug` 和 `build/Mission-Release`，单段目录例如 `build/Mission-Debug-AC`。脚本支持用 `-ArmBin`、`-Ninja` 指定工具链。底层 CMake 参数为 `-DMISSION_MODE=FULL|AC|CB|BD|DA`，默认 FULL；非法值配置失败。单段模式完成后停车。

单段调试初始姿态：AC朝右水平；BD朝左水平；CB放在C点黑线上并沿切线朝右；DA放在D点黑线上并沿切线朝左。启动时依据所选模式建立水平参考，段间保持此参考，不重新清零陀螺仪。

也可在配置好 ARM/Ninja PATH 后执行 `cmake --preset Mission-Debug`、`cmake --build --preset Mission-Debug`。VS Code 原构建任务已改为 `Mission-Debug`，调试 ELF 路径同步到该目录；需要先配置这个 preset。

## 烧录（ST-LINK + STM32_Programmer_CLI）

构建的 POST_BUILD 步骤会用 `arm-none-eabi-objcopy` 在同目录生成 `C10B_Test.hex` 和 `C10B_Test.bin`，`.bin` 大小等于 FLASH 占用，可直接用于 CLI 烧录。

```powershell
# 构建 + 烧录 + 校验（mode=UR reset=HWrst freq=100）
.\scripts\flash_mission.ps1

# 只烧已构建的映像 / 单段固件 / NRST 未接时改用软件复位
.\scripts\flash_mission.ps1 -NoBuild
.\scripts\flash_mission.ps1 -Mode AC
.\scripts\flash_mission.ps1 -Connect Normal

# 只打印将要执行的命令，不碰板子
.\scripts\flash_mission.ps1 -NoBuild -DryRun
```

等价的手动命令：

```powershell
& 'C:\Users\21656\AppData\Local\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe' `
  -c port=SWD mode=UR reset=HWrst freq=100 `
  -d 'D:\STM32project\car-for2\build\Mission-Debug\C10B_Test.hex' `
  -v -rst
```

`-c` 决定连接方式：`mode=UR reset=HWrst` 需要 NRST 接到 SWD 排针，未接时用 `mode=NORMAL reset=SWrst`；`freq=100` 是 100 kHz，故意降速以稳定连接。`-v` 下载后校验（成功打印 `Download verified successfully`），`-rst` 下载完复位运行。脚本默认 `mode=UR` 失败会自动回退到 `NORMAL + SWrst` 再试一次。烧录后固件停在 `MISSION / IDLE`，短按 USER_KEY 才启动。

## 测试与读取诊断

```powershell
.\scripts\test_mission.ps1

# 已烧录匹配固件后才使用；只读，不烧录、不复位、不控制电机
.\scripts\read_mission.ps1 -AsJson
.\scripts\read_mission.ps1 -BuildDir build\Mission-Debug-AC -AsJson
# 离线解码历史导出，不连接板卡
.\scripts\read_mission.ps1 -DumpFile path\to\mission-dump.txt -AsJson
```

主机测试在 `build/Mission-Host` 构建，包含调度器回归、五种真实配置的启动预检、实际转向/直行/巡线与端点检测测试、实际编码器回绕测试，以及真实四段算法的完整路线合成反馈测试。运行正式调度器、运动执行适配器、按键和速度PI/斜坡代码，硬件用替身隔离。调度器回归用的模拟路段只链接进该项测试。

测试覆盖转向正负方向、航向纠偏方向、巡线差速、捕线/转向超时及保护、距离上下界、窗口外跳变、重复帧、确认中越界、端点去重、C/D不停车交接、B原地转向、A停车、传感器/速度故障、手动停车、长按校准、上电按住、16位计数及毫秒回绕。上电不运动；现在显式启动且预检通过后会执行实际算法。

完整路线测试提供理想轮速反馈和按阶段生成的灰度轨迹，用于验证接口、状态和故障边界；它没有模拟真实电机负载、轮胎打滑、光照和物理赛道，不能用通过结果代替实车验证。

`read_task2`、`read_speed_test`、`read_line_follow`、`read_landmark`、`read_control_trace` 已停止在线使用，仅允许 `-DumpFile` 解码旧记录并明确警告。依赖旧停车状态的 `measure_wheel_distance` 整体停用。灰度、OLED 和 UART 硬件诊断保留，灰度/OLED 读取器默认改为 `build/Mission-Debug`。

## 后续实车调参顺序

1. 核对正向电机/编码器、陀螺仪正方向与探头左→右顺序，确认灰度校准有效。
2. 用独立模式调AC、BD转向角和航向保持，再用已知直线距离校准左右计数/mm，观察实际端点计距后调整各段识别区间。
3. 调CB、DA捕线速度、巡线速度与PD，分别验证正常出口与中途丢线；随后验证C/D连续交接和B停车转向。
4. 最终A点以灰度排离开黑线末端为参考，确认事件后立即制动；测量车身参考点与制动惯性带来的实际停车误差。蜂鸣器接线确认后再接入声音输出。

场地尺寸已按确认值形成转角和名义距离初值；距离区间是首版试调设置。轮计数/mm仍是历史估计，需要针对轮胎和载荷复核。分段建立基准可隔离前段累计计数，不会消除已经形成的位置偏移、轮滑或比例误差。

验收证据分开记录：本轮为实际算法代码、主机合成反馈测试和ARM构建；尚未烧录，未做板卡行为或实车路线验证。


## IMU port from car-02- (2026-10-27)

The MPU6050 transport/settings now follow `D:/STM32project/car-02-/Core/Src/mpu6050.c`:
PB14/SCL and PB15/SDA, high-speed open-drain GPIO, 5 us software-I2C phases,
DLPF 3, a 1 kHz sensor sample rate, +/-500 deg/s, and 65.5 LSB/(deg/s).
The 5 ms control poll reads INT_STATUS (0x3A), requires DATA_RDY, then reads
GYRO_ZOUT (0x47/0x48) separately. Initialization configures the awake sensor
directly, without the previous extra device/signal-path resets. ACK checks,
clock/transaction timeouts, identity/configuration readback and bus clear remain.

Yaw calibration follows the reference `imu_filter.c` and its measured installed
sensor bias: 400 fresh samples over at least 1995 ms, stopped drive, fresh
stationary encoder feedback, Z rate within 10 deg/s of the -23 deg/s reference,
and sample standard deviation <=0.8 deg/s. The reference only centers the gate;
each boot estimates and subtracts its actual bias. Re-measure this reference
after changing the sensor. Parameters are in `Core/Src/Modules/imu_config.h`.
Z must remain vertical on the car. Acceleration X saturation remains visible in
diagnostics; it does not certify a healthy six-axis device and no longer gates
Z-only yaw calibration, matching the reference project. Sample gaps and gyro
saturation still invalidate yaw; the mission still refuses/stops on invalid IMU.

`imu_diag` is V2 (same 32-word layout): configuration words 26/27/28 are now
0x08/0/3, raw gyro units are 65.5 LSB/(deg/s), and calibration rejection 2 means
wheel feedback is moving/stale or drive active. `imu_raw_diag` is V3 (same
64-word layout): the control sample contains status and Z, while other axes
are updated by timestamped slow diagnostic transactions every ~250 ms. They
are not a simultaneous six-axis frame. Diagnose addresses from the newly built
map, not an old firmware's RAM offsets.

Default flashing remains the FULL four-segment route. Build the updated image
before flashing; an older ELF does not include source edits:

```powershell
.\scripts\build_mission.ps1 -Mode FULL
.\scripts\test_mission.ps1
```

The new `imu_heading_test` covers the measured bias, both yaw directions,
+/-500 scaling, moving wheels/reference/noise rejection, insufficient calibration
time, stale data, gyro rails, tick wrap, and the manual gyro-test scale.


## 2026-10-27: turn approach, C/D capture and arrival feedback

Existing cruise/rate/gain tuning values were deliberately retained for the user
to change. `motion_tuning.h` now separates coarse turn breakaway speed from the
fine approach floor (120 cps within the last 12 degrees), fades the turn friction
feed-forward in that region, and adds a bounded yaw-rate braking lead (inside
the existing 3 degree acceptance). Overshoot still needs real-car tuning; the
bounded correction and motion fault checks remain enabled.

A confirmed C/D handoff now guides forward capture toward the semicircle entry
tangent (CB 0 degrees; DA 180 degrees). Capture tolerates a single contiguous
wide stripe and a bounded 350 ms missing/ambiguous-line interval. A wide mask
cannot certify capture: fresh centered narrow-line frames and an entry-heading
error <=20 degrees are required. Normal following retains its 100 ms line-fault
protection, and absolute capture time/distance bounds still apply.

The C10B PA15 buzzer (active high, same mapping as car-02-) now accompanies the
PC2 LED on each confirmed C/B/D/A event for 300 ms, without blocking control.
The scheduler already advances a FULL route in the same confirmed-arrival tick;
no second button press is needed. An unconfirmed C never produces an arrival.

OLED now shows the specific segment fault (DISTANCE, CAPTURE TIME/DIST, NO LINE,
LINE AMBIGUOUS, etc.). Row 6 contains P=last point, N=arrival count, M=decimal
black-channel mask, C=endpoint confirmation frames. P:- N:0 means C has not been
confirmed; P:C N:1 means C has been confirmed even if incoming CB then faults.
The actual reason for the reported C stop remains unconfirmed until a matching
runtime record or display reading is collected. Run after the failure, before
resetting or starting again:

```powershell
.\scripts\read_mission.ps1 -AsJson
```

Suggested first trial changes, not calibrated or applied values (cps are encoder
counts per second; current 6.73014 counts/mm makes 900 cps about 134 mm/s):

| Parameter in Core/Inc/motion_tuning.h | Existing | Suggested trial |
| --- | --- | --- |
| MOTION_AC_CRUISE_CPS / MOTION_BD_CRUISE_CPS | 2000 | 2400 |
| MOTION_CB_CRUISE_CPS / MOTION_DA_CRUISE_CPS | 1400 | 1600 |
| MOTION_CB_CAPTURE_CPS / MOTION_DA_CAPTURE_CPS | 1000 | 1200 (see the 3200/2400/1000 capture failure first) |
| MOTION_TURN_RATE_MAX_DPS | 65 | 80 |
| MOTION_TURN_ANGLE_KP | 3.2 | 3.6 |
| MOTION_TURN_RATE_FF | 16 | about 8 for a 120-130 mm wheel track |
| MOTION_LINE_PERIOD_MS | 50 | 20 when increasing arc speed |

Adjust straight speeds first, then the turn group, then arc/capture speeds. Turn
feed-forward is wheel cps per yaw deg/s, approximately
`wheel_track_mm / 2 * counts_per_mm * pi / 180`; excessive feed-forward does not
make actual yaw track its requested rate accurately. Keep angle tolerance at
3 degrees initially instead of masking overshoot with a large tolerance.

Capture budget couples to the straight speed: the controller brakes from cruise
to the capture speed with a 5000 cps/s ramp, and the ramp itself covers
`[capture*(cruise-capture)/5000 + (cruise-capture)^2/10000] / 6.73` mm of the
`MOTION_CAPTURE_MAX_MM` allowance (107 mm at 2800 -> 800, 143 mm at 3200 -> 800).
That deceleration also eats the wheel differential the corner needs, which is
why the straight segment now slows to `MOTION_STRAIGHT_HANDOFF_CPS` over its
last `MOTION_POINT_APPROACH_MM`. Keep `MOTION_CAPTURE_MAX_MM` at 320 mm: the
first ~80 mm of the corner have no line under the probe bar at all, because the
only black there is the bar the car just crossed.

AC/BD endpoint windows remain 1000-1550 mm. Their FIRST black transition and
all confirmation frames must fall inside the window. The forward-mounted gray
bar reaches the line before the axle reaches the geometric point. If the
transition occurs before 1000 mm, lower the corresponding POINT_MIN_MM using
measured data; do not force completion on nominal distance or simply bypass
line confirmation. Raising speed does not repair a missed endpoint.

New regression cases cover predictive braking in both turn directions, fine
approach targets, C/D tangent steering, wide-stripe capture without premature
promotion, capture search deadlines, and unchanged FOLLOW ambiguity protection.
Rebuild FULL and run the host tests before flashing a newly adjusted image.


Validation for this edit: changed controller/port sources and expanded motion
tests compile to Cortex-M3 objects with the installed ARM Clang. Full GNU
firmware linking and execution of the updated regression suite remain blocked
inside the current tool environment by Illegal System DLL Relocation. The
previous 12/12 host result predates these motion changes. Run build_mission and
test_mission on the host again; the existing ELF is not the updated image.

## 2026-10-28：C/D 滚动捕获时越过黑线的局部修正

实车反馈：C 点确认后边走边转，偶尔扫过黑线，转向结束时探头再次全白。
代码中发现两项相关风险：可用窄线出现后仍叠加入口切线航向修正；捕获期
沿用 50 ms 巡线更新间隔，短暂扫过的灰度帧可能未及时改变轮速目标。

- C/D 交接捕获看到可用窄线时，立即使用该线的位置 PD 和曲率前馈，停止
  叠加入口切线航向修正。首次接线清微分，避免把判点横条到窄线的跳变当成
  线位置变化率；之后按 `MOTION_CAPTURE_LINE_PERIOD_MS=10` ms 更新新帧。
- 连续三个新帧、中心范围和 20° 入口航向条件仍共同决定进入 FOLLOW；
  宽判点横条不能直接确认捕获。未见可用线时沿用原有路径搜索及保护预算。
- FOLLOW 的 50 ms 周期、直道速度、捕获速度、PI 和 A 点航向原点策略保持
  现值。本次修改作用于共享圆弧控制器，因此 D 点也需要实车检查。

验证：主机 14/14 通过，滚动入弯测试由 7 个扩为 9 个，新增 C/D 两侧的
反向循线纠偏、新帧快速扫线响应和重复帧不重复计数检查。FULL/AC/CB/BD/DA
的 ARM Debug 与 FULL Release 均成功构建；FULL Debug 为 RAM 12 760 B、
FLASH 58 620 B。本次未烧录，也未进行实车运动验证。以上构建结果更新了
上一节记录的旧环境限制。

全路线测试同时移除了捕获期“无论航向如何都给居中黑线”的输入，捕获和
巡线均按航向误差生成灰度，并在超出模型视野时给全白。合成四圈为
99 160 ms、16 次到点；该模型没有横向轮胎动力学或真实轮速响应，且反馈
模型已变化，此时间不能作为实车成绩或与旧模型直接比较速度提升的证据。

下一次实车重点：C 点窄线首次出现后轮速差是否及时收回或反向、是否进入
FOLLOW，以及 D 点有无同样越线。若仍发生，记录故障原因、Phase、Line_mask、
Line_position、Capture_frames、两侧目标/实测轮速及角速度，继续区分捕获控制
延迟、进入 FOLLOW 后丢线与机械转向惯性，再单独调整参数。

### 2026-10-28 15:05：烧录与后续交付流程

上述捕线修正版已通过 `flash_mission.ps1 -Mode FULL -Connect Normal` 烧录，
CubeProgrammer 返回 `Download verified successfully` 并复位。HOTPLUG 读回为
V3、FULL、IDLE / READY，Speed_fault=0、Segment_fault=0，陀螺仪及灰度均就绪，
两轮目标为 0，输出模式 BRAKE。尚未按键启动或验证实车路线。

后续每次代码修改均按以下流程交付：完成相应主机测试和 ARM 构建；烧录
当前 FULL 固件并校验、读回诊断；检查待提交内容中的敏感信息；提交并推送
到当前项目 `navihouse/car-for2` 的 `main` 分支，最后核对远端提交 SHA。
单段调试或其他分支以当次明确要求为准。烧录或推送失败时应明确报告实际
结果，不能以本地构建成功代替烧录成功，也不能以本地提交代替远端上传成功。
