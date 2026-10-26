# 小车单任务：A → C → B → D → A

当前版本已写入四段实际运动算法和集中调参表。上电保持制动，按键或 API 明确启动后才运行；参数采用首版试调初值，尚未进行板卡或实车验证。四段均为 `implemented=1`，启动前检查参数、IMU、编码器和灰度数据。

确认的场地尺寸为 AB=1000 mm、AD=BC=800 mm，两侧半径400 mm。A处车头水平朝右，先原地右转38.66°，再沿AC直行；B处允许停车后原地左转。C、D连续交接进入半弧巡线，最终回到A制动停车。

## 运动参数：统一在这里修改

**所有主要运动调参入口：`Core/Inc/motion_tuning.h`。修改后重新构建。** 参数按编号分组，四段文件只绑定该段参数和共用控制器。

| 路段 | 目标方向及动作 | 基础速度 | 名义距离 | 端点区间（含边界） |
| --- | --- | --- | --- | --- |
| AC | 朝右水平为0°，右转到−38.66°后直行 | 900 cps | 1281 mm | 1000～1550 mm |
| CB | 右半弧，目标出口朝左 | 捕线600 / 巡线900 cps | 1257 mm | 1000～1500 mm |
| BD | 朝左水平为180°，左转到218.66°后直行 | 900 cps | 1281 mm | 1000～1550 mm |
| DA | 左半弧，目标出口朝右 | 捕线600 / 巡线900 cps | 1257 mm | 1000～1500 mm |

方向采用左转为正，直行目标锚定整条路线的起始水平参考。BD的218.66°等价于−141.34°；控制器将角度误差归一到±180°，不会把B处的航向偏差直接作为新目标。

| 在调参头文件内搜索 | 初值及单位 | 调整作用 |
| --- | --- | --- |
| `MOTION_AC_*`、`BD_*`、`CB_*`、`DA_*` | 上表对应的速度、角度、名义距离及 `POINT_MIN_MM / POINT_MAX_MM` | 每段分别设置；扩大区间会放宽端点识别范围，名义距离只用于显示 |
| `MOTION_LEFT_COUNTS_PER_MM / RIGHT_COUNTS_PER_MM` | 两侧均6.730141612 counts/mm | 历史轮校准值；增大后相同计数显示的距离减小，左右可以分别标定 |
| `MOTION_SPEED_KP / KI` | 0.010 / 0.030 | 轮速PI；Kp增大加快纠偏，Ki消除持续误差，过大可能振荡 |
| `MOTION_PWM_LIMIT / SPEED_INTEGRAL_LIMIT` | 正向PWM上限65% / 积分限幅20 | 约束正向输出；转向专用驱动限幅沿用原有值 |
| `MOTION_TARGET_RAMP_CPS_S` | 5000 cps/s | 增大后加减速更快 |
| `MOTION_HEADING_KP / KD / TRIM_MAX_CPS` | 10 / 3 / 150 cps | 直行航向角度纠偏、角速度阻尼和差速修正上限 |
| `MOTION_HEADING_FAULT_DEG` | 15° | 直行偏航超过阈值即故障停车 |
| `MOTION_TURN_ANGLE_KP / RATE_FF / RATE_KP` | 2 / 16 / 6 | 原地转向的角度增益、角速度前馈和角速度反馈 |
| `MOTION_TURN_RATE_MAX_DPS / MIN_CPS / MAX_CPS` | 50°/s / 350 / 1200 cps | 转向角速度及左右轮速度限幅 |
| `MOTION_TURN_BRAKE_DEG / ACCEPT_DEG` | 2° / 3° | 进入制动及稳定后允许进入直行的角度误差 |
| `MOTION_SETTLE_MS / STILL_RATE_DPS / STILL_WHEEL_CPS` | 500 ms / 3°/s / 60 cps | 连续静止确认时间及阈值 |
| `MOTION_TURN_MAX_CORRECTIONS / TIMEOUT_MS` | 3次 / 25000 ms | 转向修正次数和总超时（包含等待稳定） |
| `MOTION_LINE_KP / KD / D_FILTER_S` | 0.18 / 0.006 / 0.060 s | 旧巡线PD及微分滤波；位置单位为探头坐标−3500～+3500 |
| `MOTION_LINE_STEERING_MAX_CPS` | 650 cps | 巡线差速上限，实际仍受最低轮速和总轮速上限约束 |
| `MOTION_LINE_HIGH_* / CORNER_BASE_CPS / MIN_WHEEL_CPS` | 沿用旧巡线速度分段公式 | 调整高速/捕线状态的基础速度和轮速下限 |
| `MOTION_CAPTURE_CENTER_LIMIT / CONFIRM_FRAMES` | 1000位置单位 / 3个新帧 | 捕线转入正常巡线的中心条件 |
| `MOTION_CAPTURE_TIMEOUT_MS / MAX_MM` | 1500 ms / 200 mm | 捕线任一条件超限即故障 |
| `MOTION_POINT_CONFIRM_FRAMES` | 3个新帧 | 黑白状态与端点跳变确认，重复读取同一帧不累加 |
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
- AC、BD执行 `STILL → TURN → SETTLE → STRAIGHT`，静止条件需连续满足至少500 ms；角度已满足时可直接进入稳定阶段。转向最多修正3次，保留方向、平移、双轮同步及陀螺仪响应保护。
- 直线计距在转向及制动稳定结束后建立基准；半弧计距在接管时建立，包含捕线过程。共用距离公式为 `0.5 * ((L-L0)/kL + (R-R0)/kR)`，先作整数差再换算毫米，左右轮有符号行程取平均。每段持有独立基准，编码器生产者和全局累计值持续保留。
- C、D交接时仅修改正向目标，保留编码器、航向、滤波、PI及速度斜坡；B处先制动，再原地转向，允许重建速度环。既有 `SpeedControl_StartDifferential()` 的重置语义保留，新任务仅使用保留计数的启动接口。
- 四段都要求有效航向、编码器和已校准的新鲜灰度数据。AC、BD允许全白，仍需灰度通信正常；单段CB、DA启动要求灰度排在线上。校准期间不允许运动；故障后需明确重新启动，速度环锁存故障需复位后重新检查。
- 直行段稳定白→稳定黑分别识别C、D；半弧段稳定黑→稳定白分别识别B、A。端点区间包含上下限，跳变第一帧和全部确认帧均须在区间内。窗口外的跳变不保留到窗口内；超过距离上限仍未确认则故障停车。
- 半弧先 `CAPTURE` 后 `FOLLOW`：低速巡线，连续3个新帧居中后升至巡线速度。短暂丢线保持一次性降速后的方向，100 ms仍未恢复则故障；合法出口识别优先于普通丢线保护。灰度过期不能作为白色到点事件。任务按传感器事件切段，名义距离和定时器都不单独宣告到点。

## 启停、显示与到点事件

- USER 按键：PA0，高电平按下；上电时按住必须先释放。空闲短按在消抖释放后请求启动，运行中按下立即请求停车，停车后的释放不会再次启动。
- 空闲按住约 6 秒后释放进入灰度黑白校准；校准中再按一次取消。保留原校准过程及存储格式；校准和等待期间持续更新传感器。
- OLED显示模式、当前路段、任务状态、停止原因、内部阶段、分段距离、识别区间、目标航向、航向误差、端点确认帧数和到点次数；校准状态在末行显示。
- 每个路段完成时总调度器发出一次到点事件。全程顺序为 **C、B、D、A**；初始放在 A 不发事件。事件与是否能继续进入下一段分别处理：例如到 C 后下一段传感器失效，仍保留 C 到达事件并停车。
- 到点 LED1（PC2，低有效）非阻塞点亮 150 ms。蜂鸣器未绑定引脚，本轮只预留 `MissionPort_NotifyPoint()` 接口，尚无声音输出。
- `mission_diag` 升级为144字节V2，保留原有模式、状态、目标、传感器和事件字段，新增内部阶段、执行模式、分段距离、识别区间、目标航向/误差（毫度）、巡线位置、确认帧数和转向修正次数。读取器同步升级，V1记录不会误读为V2。常规20 ms发布，阶段和命令变化当轮发布。`arrivals` 为本次任务到点数，`event_count` 为本次上电累计事件数；头尾序号须相等且为偶数。
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

也可在配置好 ARM/Ninja PATH 后执行 `cmake --preset Mission-Debug`、`cmake --build --preset Mission-Debug`。VS Code 原构建任务已改为 `Mission-Debug`，调试 ELF 路径同步到该目录；需要先配置这个 preset。调试/烧录需另行发起，本次实施不连接或烧录板卡。

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
