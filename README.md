# 小车单任务：A → C → B → D → A

当前版本完成任务结构改造。四段运动算法均为**未实现占位**：上电保持制动，按键或 API 启动返回 `REJECTED / NOT_IMPLEMENTED`，不会驱动车辆，也不会自动切段或假报完成。

路线要求：AC、BD 穿过空白区域；CB、DA 沿黑色半弧巡线；经过 C、B、D 时连续行驶；最后回到 A 停车。直线保持航向、巡线、捕线、脱线、端点识别、最终停车距离与实车调参留在下一阶段。

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
| `segment_ac.c / cb.c / bd.c / da.c` | 各段独立参数与逻辑，目前 `implemented=0` 且回调返回 `SEG_UNIMPLEMENTED` |
| `mission_port.h / mission_port_stm32.c` | 硬件边界；主机测试替换这一层，仍执行正式调度器和速度控制代码 |

共用驱动位于 `Core/Src/Modules/`，保留电机、编码器、速度 PI、按键消抖、灰度 UART、黑白校准及 Flash 存储、IMU、OLED。蓝牙底层 `bt_uart` 源码保留但不编译、不初始化；蓝牙模式协议和旧任务调度已移除。

旧任务 `car/task2/task4/mode2`、其专用控制及诊断记录、旧计圈/位置停车/巡线任务包装和更早的重复应用源码已删除。删除前的源码归档在本地 `backups/before-mission-refactor-20261026.zip`。既有备份和日志保留；`backups/`、`logs/` 和 `build/` 不纳入版本控制。

## 路段接口与交接规则

- `Enter(ctx, input, params)`：建立路段局部状态，返回第一组有效左右轮目标。只能返回运行中、故障或未实现；不能在入口直接宣告完成。
- `Update(ctx, input, params)`：返回 `SEG_RUNNING / SEG_COMPLETE / SEG_FAULT / SEG_UNIMPLEMENTED`，运行时带左右轮目标，单位为**编码器计数/秒（cps）**。故障可附带路段专用错误码。
- `Exit(ctx, input, reason)`：只清理内部状态，不调用电机、编码器或 IMU。完成、手动停止、故障分别传入不同退出原因，每次激活最多退出一次。
- 每段拥有独立上下文和参数。上下文含入口快照、活动状态及局部阶段；后续新增本段参数和状态应在该段维护，需要扩展共享上下文时显式更新接口。
- 每轮先各更新一次编码器、灰度及 IMU，再生成一份不可修改快照。出段和入段读取同一份快照；路段只计算目标，由任务统一执行。
- 当前统一速度接口接受两轮均 `0 < cps <= 3200` 的前进目标，拒绝零值、负值、NaN 和无穷大。任务结束或故障通过独立停车命令处理。
- 全程启动前检查四段描述符及 `implemented`，单段模式只检查所选路段。任何必需段未实现均在电机启动前拒绝运行。
- 启动时读取入口位置和航向作为任务/路段基准，使用新增 `SpeedControl_StartDifferentialPreserveCounts()` 启动速度环，保留编码器生产者及其累计值。既有 `StartDifferential()` 的重置语义保留。
- 普通切段只调用 `SpeedControl_SetDifferentialTargets()`，不清零编码器、不重置航向、不重启 PI。下一段 `Enter()` 当轮的目标立即接管，底层继续执行原有速度斜坡。
- AC、BD 要求有效航向，不要求空白处存在黑线；CB、DA 要求有效航向和已校准的新鲜灰度数据。校准期间不允许运动；故障停车后需要新的启动操作，速度环锁存故障需复位后重新检查。
- 无定时假切段。后续完成条件必须由各段结合实际传感器和位置判定；入口过渡、捕线和出口过渡也属于路段内部逻辑。

## 启停、显示与到点事件

- USER 按键：PA0，高电平按下；上电时按住必须先释放。空闲短按在消抖释放后请求启动，运行中按下立即请求停车，停车后的释放不会再次启动。
- 空闲按住约 6 秒后释放进入灰度黑白校准；校准中再按一次取消。保留原校准过程及存储格式；校准和等待期间持续更新传感器。
- OLED 显示编译模式、当前路段、任务状态、停止原因、目标 cps、到点次数和灰度校准状态。占位启动后显示 `REJECTED / NOT IMPLEMENTED`。
- 每个路段完成时总调度器发出一次到点事件。全程顺序为 **C、B、D、A**；初始放在 A 不发事件。事件与是否能继续进入下一段分别处理：例如到 C 后下一段传感器失效，仍保留 C 到达事件并停车。
- 到点 LED1（PC2，低有效）非阻塞点亮 150 ms。蜂鸣器未绑定引脚，本轮只预留 `MissionPort_NotifyPoint()` 接口，尚无声音输出。
- `mission_diag` 为新的 96 字节 V1 记录，包含模式、状态、当前段、停止原因、路段错误码、目标、传感器状态和事件。常规更新间隔 20 ms，状态、目标及到点事件变化当轮发布，保留供 SWD 读取的稳定窗口。`arrivals` 为本次任务到点数，`event_count` 为本次上电累计事件数。读取时要求头尾序号相等且为偶数。

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

主机测试在 `build/Mission-Host` 构建，包含纯状态机、五种正式占位配置和可运动模拟路段。运行测试使用正式的任务、按键消抖、速度 PI/斜坡代码，硬件端口及传感器反馈使用替身；模拟路段仅链接进测试，不存在于固件。

测试覆盖全程顺序、单段结束、到点去重、同轮交接、保留累计值及航向、启动预检、无效目标、传感器/速度故障、手动停车、长按校准、上电按住和毫秒回绕。占位配置验证上电、API 启动及按键启动均无驱动输出。

`read_task2`、`read_speed_test`、`read_line_follow`、`read_landmark`、`read_control_trace` 已停止在线使用，仅允许 `-DumpFile` 解码旧记录并明确警告。依赖旧停车状态的 `measure_wheel_distance` 整体停用。灰度、OLED 和 UART 硬件诊断保留，灰度/OLED 读取器默认改为 `build/Mission-Debug`。

## 后续逐段接入

1. 选择一个段，例如 `-Mode AC`，在对应文件实现 `Enter/Update/Exit` 和该段参数，先保持 `implemented=0`。
2. 用模拟输入验证目标限制、完成条件、超时和故障；完成后才将该段 `implemented` 改为 1。不要通过时长或固定返回完成来绕过占位检查。
3. 单段实车校准入口姿态、距离、速度及完成判据。四段分别验证后，再验证 AC→CB、CB→BD、BD→DA 的连续交接。
4. 完成端点几何、传感器安装偏移与到点判据校准，确认蜂鸣器接线后接入非阻塞声音输出，最后验证全程回 A 停车。

图中尺寸不直接换成控制默认值；旧 `odometry_config.h` 的轮距换算比例仅作为历史校准资料保留，当前任务未使用，需针对新赛道和轮胎载荷复核。

验收证据分开记录：本轮为代码结构、主机模拟和 ARM 构建；尚未烧录、未做板卡行为或实车路线验证。
