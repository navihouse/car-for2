# car-for2 下一阶段修改方案（2026-10-27）

依据：`car-for2_AI_HANDOFF_2026-10-27.md`（GPT 交接文档）+ 当天对本仓库的实测复核。
所有行号对应当前工作区文件。

---

## 0. 本次实测复核（先把"没验证"这块补上）

| 项目 | 结果 |
| --- | --- |
| FULL 固件构建 | 通过。RAM 12 632 / 48 KB（25.70%），FLASH 50 736 / 254 KB（19.51%） |
| 新增产物 | `build/Mission-Debug/C10B_Test.hex`（142 798 B）、`.bin`（50 736 B，与 FLASH 占用一致），由 `CMakeLists.txt` 末尾新增的 POST_BUILD `objcopy` 生成 |
| 主机回归 | 12/12 通过，0 失败，5.4 s：`mission_control_test`、`mission_readiness_0..4`、`mission_runtime_test`、`motion_algorithms_test`、`mission_actual_route_test`、`imu_heading_test`、`encoder_wrap_test`、`mission_reader_test` |
| 构建脚本 | 修了 `scripts/build_mission.ps1` 在 PowerShell 7.3+ 下被 CMake 的 stderr 打断的缺陷（native stderr 被提升为终止错误） |

结论：交接文档里"本批 9 项改动只有代码审查、没跑主机测试、没有 ARM 构建"这个最大的空白，已经在主机侧补齐。
**但板子上跑的仍是旧固件**，因此"C 点后不继续、也没听到提示声"这条现象发生在**不含蜂鸣器代码、C 点判据也不同**的旧映像上（旧版只点亮 LED1 150 ms，蜂鸣器当时根本没绑定引脚）。它不能直接当作"新逻辑在 C 点失败"的证据，必须先烧新映像再复测一次。

---

## 1. 问题 → 根因假设

### P1（阻断）C 点后不再继续

新固件停在 C 附近时会落在三种状态，OLED 第 2~4 行和 `mission_diag` 能直接区分：

| 你看到的现象 | `segment` / `phase` | `reason` / `segment_fault` | 含义 | 处置方向 |
| --- | --- | --- | --- | --- |
| 冲过 C 很远（≈1.5 m）才停，蜂鸣器**不响** | AC / STRAIGHT | SEGMENT FAULT / DISTANCE | C 的条纹从未被确认，AC 一直跑到 `point_max` 上限 | 判据窗口、掩码、标定 |
| 停在 C 附近，蜂鸣器**响过** | CB / CAPTURE | SEGMENT FAULT / CAPTURE TIME/DIST | 确认了 C，但窄条纹捕获 1.5 s / 200 mm 内没成 | 放宽 capture |
| 进了 C 之后又动了一小段才停 | CB / FOLLOW | SEGMENT FAULT / NO LINE | 捕获成功、跟线丢线 | 跟线稳健性 |

结构性隐患（无论根因是哪条都要改）：

- 点确认窗口写死在 `Core/Inc/motion_tuning.h:43-50`：`MOTION_AC_POINT_MIN_MM 1000` / `MAX 1550`，而 `MOTION_AC_NOMINAL_MM 1281`（:39）。也就是 −22%/+21%，只要 `MOTION_LEFT_COUNTS_PER_MM 6.730141612`（:10-11）有 5% 偏差、或起步打滑，C 就会掉到窗口边缘。
- capture 是四个条件 AND 在一起：`MOTION_CAPTURE_TIMEOUT_MS 1500`（:110）、`MOTION_CAPTURE_MAX_MM 200`（:111）、`MOTION_CAPTURE_CENTER_LIMIT 1000`（:109）、`MOTION_CAPTURE_HEADING_ACCEPT_DEG 20`（:116）。任何一个偏严，C 处就 1.5 s 内失败。
- 失败即 `MissionControl_Fail` 停机（`Core/Src/Mission/mission.c:112-117`、`:158-162`），**没有降级恢复路径**。而你的要求是"不要卡在那里"。

### P2（体验）车太慢

量化：`MOTION_AC_CRUISE_CPS 900`（`motion_tuning.h:33-36`）÷ 6.7301 ≈ **133.7 mm/s**。
AB 1000 mm ≈ 7.5 s，AC 1281 mm ≈ 9.6 s，四段直线 ≈ 36 s；转向 `MOTION_TURN_RATE_MAX_DPS 50`（:65）→ 180° ≥ 3.6 s，38.66° ≥ 0.8 s，再加 `MOTION_SETTLE_MS 500`（:77）与纠偏。整套路线 ≥ 50 s。

更关键的第二个原因在速度环（`Core/Src/Modules/speed_control.c`）：

- 前馈是**固定线性拟合**、且左右不同：`LEFT_FEED_FORWARD = 50.0f/3346.0f`（:8）、`RIGHT_FEED_FORWARD = 50.0f/2727.5f`（:9）。900 cps 只给约 **13.4 / 16.5** 个 PWM 单位。
- `MOTION_SPEED_INTEGRAL_LIMIT 20`（`motion_tuning.h:19`）把积分卡在 ±20，`MOTION_SPEED_KP 0.010`（:16）意味着 200 cps 的误差只补 2 个 PWM 单位。
- 于是：**如果新车电机/电池/齿比与取值来源不同，前馈偏小 + 积分上限 20 → 稳态 PWM 被卡在 30~40%，cruise 设多大都跑不上去**，观感就是"起步肉、到不了速"。

所以提速不能只调 cruise，必须先重测 cps↔PWM 曲线。

### P3（体验）A 点原地转向慢、过冲后反向纠偏

- 慢：`MOTION_TURN_RATE_MAX_DPS 50`（:65）上限 + `MOTION_TURN_MIN_CPS 350`（:66，约 52 mm/s 轮速）→ 低速段占掉大半时间。
- 过冲再反打：断力点由 `MOTION_TURN_BRAKE_LEAD_S 0.04`（:72）和 `MOTION_TURN_BRAKE_DEG 2.0`（:73）决定，50 dps 下最多提前 2° 断力，剩下的靠惯性滑；`MOTION_TURN_ACCEPT_DEG 3.0`（:74）很紧，滑过 3° 就进 `MOTION_TURN_MAX_CORRECTIONS 3`（:78）的纠偏轮 → 视觉上"快速转过去 → 停 → 反着抖一下"。
- 这是控制律 + 参数问题，不是硬件问题。

---

## 2. 修改方案（四批，每批可独立验收）

### 批次 A：先拿证据（0 行运动代码，今天就能做）

1. 烧 FULL 新固件（第 4 节）。
2. 完整跑一次到停车，读诊断：`.\scripts\read_mission.ps1 -AsJson`
   得到 `state / reason / segment / segment_fault / phase / motion_mode / distance_mm / line_mask / capture_frames`（写作时是 144 B V2；第 9 节已升到 **152 B V3**，多出 `lap` 与 `drift_mdeg`，magic 仍是 `0x4D534E31`，头尾奇偶序号校验，`mission.c:14` 的 `_Static_assert` 锁定 ABI）。
3. 接了 USB-TTL 的话，`MissionPort_Service` 每秒输出一行摘要，可以直接看时间轴。

**产出：一份"停在哪、为什么停"的事实单。在这之前不要改 motion 层判据**，否则又是一层猜测叠在旧猜测上。

### 批次 B：把 C 点失败变成可恢复（固件，优先级最高）

| # | 改动 | 文件 | 验收 |
| --- | --- | --- | --- |
| B1 | 点窗口由名义值推导：`point_min = 0.75*nominal`、`point_max = 1.20*nominal`，在 `Motion_ValidateParams` 校验；现有宏保留为显式覆盖 | `motion_tuning.h:39-50`、`motion_control.c` 的 `Motion_ValidateParams` | 主机用例：名义 1281 时窗口 = 961~1537 |
| B2 | DISTANCE 不再是终点：超上限后进 `PHASE_SEARCH`，≤400 cps 直行 ≤150 mm，要求同一条纹连续 3 新帧；找到 → 记 `recovered` 正常交接；超 150 mm 才报 `MOTION_FAULT_DISTANCE` | `motion_straight.c`、`motion_edge.c`、`mission_segment.h`（加 PHASE_SEARCH） | 注入"晚 200 mm 出现条纹"轨迹，断言不进 FAULT 且 segment 前进 |
| B3 | capture 分级放宽：`TIMEOUT 1500→2500`、`MAX_MM 200→320`、`HEADING_ACCEPT_DEG 20→35`，并按条纹宽度分"窄/宽"两条判据（宽条纹只在 capture 期接受，:112-113 的注释已定了这条规则） | `motion_tuning.h:106-118`、`motion_line.c` | 注入"窄条纹晚 150 mm 出现"，断言捕获成功 |
| B4 | NO LINE 降级：丢线先保持最后位置 300 ms（现在 `MOTION_LINE_UNCERTAIN_MS 100`，:118），再原地小角度扫线，最后才 FAULT | `motion_line.c`、`motion_tuning.h` | 注入"丢线 250 ms 后恢复"，断言不进 FAULT |

新增主机用例 `tests/mission_segment_recovery_test.c`，在 `tests/CMakeLists.txt` 注册；调度器回归用的模拟路段仍只链接进它自己那项测试。

### 批次 C：提速（参数 + 前馈标定，需上板）

1. **重测 cps↔PWM 曲线**：固定占空比扫 20 / 30 / 40 / 50%，记稳态 cps（可用现有 `SpeedControl` 测试模式与 `scripts/read_speed_test.ps1` 的离线解码路径），重算 `LEFT_FEED_FORWARD` / `RIGHT_FEED_FORWARD`，并按实测把 `MOTION_SPEED_INTEGRAL_LIMIT` 放到能覆盖残差的量级。
2. 调参顺序（一次只动一组）：`MOTION_PWM_LIMIT 65→80`（:18）→ `MOTION_AC/BD_CRUISE_CPS 900→1200`（:33-34）→ `MOTION_TARGET_RAMP_CPS_S 5000→8000`（:20）。CB/DA 的 900 **先不动**。
3. 只有 A→C 连续 3 次都能稳定停在窗口内，才提 CB/DA 的 900。注意 `MOTION_LINE_HIGH_SPEED_THRESHOLD_CPS 900`（:102）是高速整形分支的切换点，越过它会同时切 `HIGH_BASE_FLOOR 900 / HIGH_WHEEL_FLOOR 450`（:103-104），要一起复核。

### 批次 D：转向手感

| # | 改动 | 位置 | 说明 |
| --- | --- | --- | --- |
| D1 | `MOTION_TURN_RATE_MAX_DPS 50→80~90`，`MOTION_TURN_MIN_CPS 350` 保留 | `motion_tuning.h:65-66` | 350 是静摩擦下限，调小会原地不动 |
| D2 | 固定 `BRAKE_LEAD_S 0.04` 换成**自适应提前量**：每次 settle 记录"断力后继续转过的角度"，EMA（α≈0.5）更新 lead，clamp [1°, 8°]，断力条件改 `\|err\| ≤ ratedps*lead_s + 1°` | `motion_control.c`（转向）、`motion_tuning.h:72` | 主机完全可测：fake 陀螺给固定角速度 + 固定减速度 |
| D3 | `MOTION_TURN_ACCEPT_DEG 3→4`，第二次纠偏后放宽到 6°（滞环），避免"过冲→反打"来回 | `motion_tuning.h:74`、`motion_control.c` | 断言纠偏次数从 2 降到 ≤1 |
| D4 | `MOTION_SETTLE_MS 500→300`，仍要求满足 `MOTION_STILL_RATE_DPS 3` | `motion_tuning.h:75-77` | 每次转向省 0.2 s，四次省 0.8 s |

---

## 3. 明确不做的事

- 不把 IMU 读取搬进中断（交接文档 2.3 已否决）。
- 不破坏 `MISSION_MODE` 五档结构，不把 HAL 调用渗进 Mission 层（主机可测边界是这轮最大的资产）。
- 不改 `mission_diag` 布局：`mission.c:14` 的 `_Static_assert(sizeof(MissionDiagnostics) == 144U)` 是 ABI 闸门；要加字段必须同时升 `version` 并改 `read_mission.ps1`。
- 在批次 A 的事实单出来之前，不重写 `motion_straight.c` / `motion_line.c` 的判据。

## 4. 烧录

### 4.1 脚本（推荐）

新脚本 `scripts/flash_mission.ps1`，默认就是按你给的那条命令的形态执行，并额外做了四件事：先用 `build_mission.ps1` 构建、确保 `.hex` 不比 `.elf` 旧、失败时自动回退到软件复位再试一次、按输出关键字判定是否真的下载并校验成功。

```powershell
# 构建 + 烧录 + 校验（等价于你给的那条命令）
.\scripts\flash_mission.ps1

# 只烧已经构建好的映像
.\scripts\flash_mission.ps1 -NoBuild

# NRST 没接到 SWD 排针时，改用软件复位连
.\scripts\flash_mission.ps1 -Connect Normal

# 烧单段固件（AC / CB / BD / DA）
.\scripts\flash_mission.ps1 -Mode AC

# 只看将要执行的命令，不碰板子
.\scripts\flash_mission.ps1 -NoBuild -DryRun
```

### 4.2 手动命令（与你的命令逐项对应）

```powershell
& 'C:\Users\21656\AppData\Local\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe' `
  -c port=SWD mode=UR reset=HWrst freq=100 `
  -d 'D:\STM32project\car-for2\build\Mission-Debug\C10B_Test.hex' `
  -v -rst
```

| 参数 | 作用 | 注意 |
| --- | --- | --- |
| `-c port=SWD` | SWD 接口 | 接 ST-LINK 的 SWDIO/SWCLK/GND，板子要单独供电 |
| `mode=UR` | connect under reset | 需要 NRST 接到排针；没接就换 `mode=NORMAL reset=SWrst`（脚本 `-Connect Normal`） |
| `reset=HWrst` | 硬件复位 | 同上 |
| `freq=100` | SWD 时钟 100 kHz | 故意降速，之前 1 MHz 会 "device does not answer"；先低速连上再谈提速 |
| `-d <hex>` | 下载 Intel HEX | **这个项目的 hex 在 `build\Mission-Debug\`，不是 `firmware\T321-C2-OLED\`** |
| `-v` | 下载后校验 | 成功会打印 `Download verified successfully` |
| `-rst` | 下载完复位并运行 | 之后固件停在 MISSION / IDLE，短按 USER_KEY 才启动 |

### 4.3 板子上电后的检查顺序

1. 复位后 OLED 第 1~2 行应是 `MISSION` / `FULL`，第 5 行 `IDLE`，底部提示 `CLICK START HOLD6 CAL`。
2. 短按 USER_KEY 启动；若拒绝启动，第 4 行会给出原因（`GYRO NOT READY` / `LINE NOT READY` / `ENCODER STALE` / `SPEED FAULT`）。
3. 到 A、C、D 点应短鸣 + LED 亮一下（本批新增 PA15 蜂鸣器，`mission_port_stm32.c:17-22`）。
4. 停车后用 `.\scripts\read_mission.ps1 -AsJson` 取诊断快照。

### 4.4 常见失败

| 输出 | 原因 | 处置 |
| --- | --- | --- |
| `No STM32 device found` / `device does not answer` | 供电、SWD 线序，或 STM32CubeIDE 正占用调试口 | 关掉调试会话；确认 3.3V 与 GND；`-Connect Normal` |
| `Error: Cannot access memory` | 目标在低功耗/复位状态 | 用 `mode=UR`；确认 NRST |
| 校验失败 | hex 与芯片型号不符（F103xB vs F103xE） | 确认构建用的是 `STM32F103xx_FLASH.ld` 的 xB 变体 |

---

## 5. 建议的执行顺序

```
今天：批次 A（烧新固件 → 跑一次 → 读诊断）   ← 先拿到事实
      ↓
批次 B（恢复路径）→ 主机用例 → 实车复测 C 点
      ↓
批次 C（前馈标定 + cruise）→ 计时对比
      ↓
批次 D（转向自适应刹车）
```

每一批都保持"主机测试先绿、再上板"的次序；批次 B/C/D 的任何改动都必须让 `.\scripts\test_mission.ps1` 保持 12/12（新增用例后为 13/13、14/14）。

---

## 6. 2026-10-27 实车故障后的落地记录

### 6.1 现场结论（SWD 直接读数）

| 证据 | 数值 | 结论 |
| --- | --- | --- |
| `mission_diag` | `REJECTED` / `SPEED_FAULT`、`Elapsed_ms=0` | 启动被闸门拒绝，不是跑起来才坏 |
| `speed_control.fault` | `1 = LEFT_NO_FEEDBACK` | 左轮无反馈 |
| `speed_fault_data` | `t=16230 ms`、`mode=2`、`left_pwm=-18%`、`left_reference=-140`、`left_idle_ms=709` | A 点转向**纠偏**阶段 18% 占空比 709 ms 零计数 |
| `imu_diag` | `identity=0x68`、`errors=0`、`misses=0`、`bus_fault=0`、198 Hz、静止漂移 -0.8°/20 s、`bias=-23.2 dps` | IMU 正常；`-23 dps` 与 `imu_config.h:17` 既有基线一致；仅 `accel X=-32768` 饱和（仅诊断用） |

根因：本批次同时加了 `speed_control.c` 的摩擦淡出（30%→12%）和 `motion_straight.c:59` 的速度下限淡出（350→120 cps），纠偏占空比降到 18%，低于静摩擦所需；而 `speed_control.c:268` 的 400 ms 转向无反馈判据没动，于是判死。

### 6.2 本次改动（已烧录验证）

| 文件 | 改动 |
| --- | --- |
| `Core/Src/Modules/speed_control.c` | 新增 `left_boost/right_boost`（0..1）：轮子真卡住 ≥ `MOTION_TURN_BREAKAWAY_MS` 时把淡出的 30% 踢一脚补回来，轮子动起来后按 `MOTION_TURN_BOOST_RELEASE_MS` 衰减；`ParkingWheel()` 增加 `stalled_ms`/`boost` 参数，调用点传 `now_ms-left_motion_ms`；转向无反馈判据改用 `MOTION_TURN_NO_FEEDBACK_MS` |
| `Core/Inc/motion_tuning.h` | 新增 `MOTION_TURN_BREAKAWAY_MS 150U`、`MOTION_TURN_BOOST_RELEASE_MS 300U`、`MOTION_TURN_NO_FEEDBACK_MS 700U` |
| `tests/test_speed_control.c` + `tests/CMakeLists.txt` | 新增 `speed_control_test`：卡死时占空比 13%→34%、转起来不升级（≤20%）、breakaway 后回落、700 ms 后仍判 `LEFT_NO_FEEDBACK` |
| `scripts/clear_speed_fault.ps1` | 台架用：SWD 写 `speed_control.fault=0` 解除锁存（不动 flash、不复位、电机不上电） |
| `scripts/read_encoder.ps1` | 读 `encoder_data`，可 `-Count 20 -IntervalMs 300` 边转轮子边看计数 |

实测：FULL 构建 RAM 12640 B（25.72%）、FLASH 50996 B（19.61%）；主机测试 **13/13** 通过；hex 143526 B 已烧录并 `Download verified successfully`。

### 6.3 决策：故障锁存保留（不修）

`mission_control.c:19` 的 `if (in->speed_fault) return MISSION_SPEED_FAULT;` **故意保留**：电机可能已通电而轮子不转，锁存可以避免持续堵转发热烧电机。因此 `LEFT_NO_FEEDBACK` 之后仍必须**断电重启**才能再试；台架上可用 `clear_speed_fault.ps1` 代替断电。不要把这条当成 bug 去"修复"。

---

## 7. 2026-10-27 第二轮：C 点识别成功但进不了 CB

### 7.1 现场结论（SWD 读数）

| 证据 | 数值 | 结论 |
| --- | --- | --- |
| `mission_diag` | `FAULT` / `SPEED_FAULT`、`Segment=CB`、`Phase=CAPTURE`、`Arrivals=1`、`Last_point=C`、`Heading_error=40.12°`、`Capture_frames=0`、`Elapsed_ms=7734` | C 点判到位了，CB 段在捕获阶段被速度故障打断 |
| `speed_fault_data` | `mode=0`、`left_pwm=6%`、`right_pwm=12%`、`left_reference=150`、`right_reference=337`、`left_idle_ms=254` | 反推 `motion_line.c:121-122` 的 `base=243`、`cap=93`，与 150/336 完全吻合 → 故障落在**无巡线回退分支** |

三条根因：

1. **CB 段没有入段转向**：`segment_cb.c:8` 的 `target_yaw_deg=180` 表示入段切线是 `180-180=0°`，而 AC 结束时车头是 `-38.66°`。原来 `motion_line.c:64` 一进段直接 `PHASE_CAPTURE`，而捕获判据要求 `|entry_error| ≤ MOTION_CAPTURE_HEADING_ACCEPT_DEG(20°)`，40° 差一倍 → `Capture_frames` 永远为 0。DA 段同理（入段切线 180°，BD 结束在 218.66°）。
2. **捕获爬行占空比低于静摩擦**：无巡线时用 `0.75 × 上一次目标`，下限 150 cps；前进模式的 `ControlWheel()` 没有摩擦前馈，150 cps 只有 6% 占空比，推不动车。
3. **无反馈判据误判**：前进模式的 `NO_FEEDBACK_MS=250U`，6% 占空比本就不该动，254 ms 后报 `LEFT_NO_FEEDBACK`，`mission_control.c:19` 立刻锁存，连 1500 ms 的捕获超时都等不到。

### 7.2 改动

| 文件 | 改动 |
| --- | --- |
| `Core/Src/Mission/motion_turn.c`（新建） | 从 `motion_straight.c` 抽出共享原地转向律 `MotionTurn_Begin/MotionTurn_Update`（角度→角速度→轮速 + 刹车带 + 方向/打滑/陀螺响应故障判定），直线段与圆弧段共用同一份代码 |
| `Core/Src/Mission/mission_segment.h:11-12` | `MissionPhase` 增加 `PHASE_ALIGN`（插在 `PHASE_STRAIGHT` 之后） |
| `Core/Src/Mission/motion_straight.c` | 改用 `motion_turn.c`，删除本地 `BeginTurn/Turn`；行为不变 |
| `Core/Src/Mission/motion_line.c` | `MotionArc_Enter`：`from_handoff` 且入段切线误差 >20° 时先 `PHASE_ALIGN` 原地转到切线；转到位后切 `PHASE_CAPTURE`（重置 1500 ms 捕获预算）；捕获期无巡线时直接用 `capture_cps` 爬行而非 0.75 倍衰减；切相位时强制 PD 下一帧重算（否则会把转向的 `-x/+x` 或刹车后的 `0/0` 当成前进目标，被判 `BAD_TARGET`） |
| `Core/Src/Modules/speed_control.c:293-296` | 无反馈判据增加占空比门控：只有 `|pwm| ≥ MOTION_STALL_MIN_PWM` 才累计 idle，低速爬行不再被判堵转 |
| `Core/Inc/motion_tuning.h:25` | 新增 `MOTION_STALL_MIN_PWM 25.0f` |
| `tests/` | 新增 `arc_align_test`（C/D 入段必须原地对齐、方向正确、≤20° 才允许捕获）；`mission_actual_route_test` 断言 C、D 交接后进入 `PHASE_ALIGN` 且驱动重新武装转向曲线 |
| `scripts/read_mission.ps1:41` | 相位名表插入 `ALIGN` |

### 7.3 验证

- 主机测试 **14/14** 通过（新增 `arc_align_test`），其中 `mission_actual_route_test` 跑完 A→C→B→D→A 全程，两处入段转向分别消掉 38.66°，车头回到 0°/180° 才进捕获。
- FULL 构建：RAM 12640 B（25.72%）、FLASH 51472 B（19.79%）。
- 尚未上板：本次改动写完时 ST-LINK 未连接（`STM32_Programmer_CLI -l` 报 `No ST-Link detected`），需插上后 `.\scripts\flash_mission.ps1 -Mode FULL` 再实车复测 C 点。

---

## 8. 2026-10-27 第三轮：C/D 入段改为「滚动转弯」，撤销第 7 节的原地对齐

### 8.1 为什么撤销

第 7 节的 `PHASE_ALIGN` 虽然在主机测试里干净利落，但物理上有两个硬伤：

1. **它是一次硬停**。AC 以 1600 cps（≈238 mm/s）巡航判到 C 点，交接帧 `MotionDrive_Command(FORWARD→TURN)` 走到 `SpeedControl_Start()`，内部先 `SpeedControl_Stop()` → `Motor_Brake()`，然后才命令 `∓800 cps`。10 ms 内从满速前进变成反向差速，实车一定甩尾，黑线可能被直接滑过去。
2. **它把静摩擦重新请回场上**。停稳后要重新克服 breakaway（第 6 节那次 `LEFT_NO_FEEDBACK` 的成因），而滚动状态下轮子本来就在转，这个问题根本不存在。

C 点是 38.66° 的折角，几何上**不存在零转向切点**，这 38.66° 必须在 C 附近转掉；能选的只是「停着转」还是「滚着转」。定量看滚动方案完全够用（capture 600 cps）：

```
差速上限 = capture_cps − MOTION_LINE_MIN_WHEEL_CPS = 600 − 150 = 450 cps
入段误差 38.66° → trim = clamp(10 × 38.66, ±450) = 386.6
两轮 = 213 / 987 cps → 差速 773 cps = 114.9 mm/s → 偏航 50.6 °/s
38.66° ÷ 50.6 ≈ 0.76 s，期间行进 ≈ 68 mm（捕获预算 200 mm）
```

### 8.2 改动

| 文件 | 改动 |
| --- | --- |
| `Core/Src/Mission/motion_line.c` | `MotionArc_Enter`：删除 `MotionTurn_Begin(...,PHASE_ALIGN)` 分支，进段即 `PHASE_CAPTURE`，`from_handoff` 时只把 `target_yaw_deg` 置为入段切线；`MotionArc_Update`：删除整段 `PHASE_ALIGN` 处理块 |
| `Core/Src/Mission/motion_line.c` | 差速上限改为动态计算：`heading_cap = min(MOTION_CAPTURE_HEADING_MAX_CPS, capture_cps − MOTION_LINE_MIN_WHEEL_CPS)`，600 cps 时即 450；`align_capture`（`from_handoff` 且仍在 `PHASE_CAPTURE`）期间始终生效 |
| `Core/Inc/motion_tuning.h` | `MOTION_CAPTURE_HEADING_MAX_CPS` 350 → **650**（只作绝对上限，实际由 capture 速度决定） |
| `tests/test_arc_align.c` | 重写为「滚动入线」6 个用例：首帧必须是 FORWARD 且两轮为正、差速用满 450、0.76 s 内收敛到 20°、窄条 3 帧切 FOLLOW、已对齐时无偏置、DA 方向相反 |
| `tests/test_motion_algorithms.c` | `CaptureHandoffTests` 断言由 `PHASE_ALIGN`+`MOTION_TURN` 改为 `PHASE_CAPTURE`+`MOTION_FORWARD`+差速方向 |
| `tests/test_mission_actual_route.c` | C/D 交接断言改为「不停车」：`MotionDrive_Mode()==MOTION_FORWARD`、`fake_starts/fake_motor_stops` 不变、`phase==PHASE_CAPTURE` |

### 8.3 保留的部分

- **P2（捕获兜底用 `capture_cps`）仍然必要**：滚动转弯时内侧轮被压到 150 cps 甚至失速，回退基速若还是 `0.75 ×` 就会掉到推不动的占空比。
- **P3（占空比门控判据）是滚动方案的安全网**：内侧轮在 6% 占空比下停转属于正常现象，`|pwm| ≥ MOTION_STALL_MIN_PWM(25%)` 才累计 idle，因此不会被误判为 `NO_FEEDBACK`；真堵转仍由 25% 以上的占空比暴露。
- `PHASE_ALIGN` 枚举**保留**（诊断编号与 `read_mission.ps1:41` 相位表不动），但 CB/DA 不再进入；A/B 点的原地转向走的是 `PHASE_TURN`。

### 8.4 验证

- 主机测试 **14/14** 通过（`arc_align_test` 已重写，`mission_actual_route_test` 全程仍绿）。
- FULL 构建：RAM 12640 B（25.72%）、FLASH 51228 B（19.70%）。
- 未上板：ST-LINK 未连接（`No ST-Link detected!`）。上板后重点看两件事：① C 点是否在不停车的情况下把车头掰进切线；② 内侧轮失速时 `Speed_fault` 是否保持 0。

---

## 9. 2026-10-27 第四轮：四圈任务（计圈 + A 点漂移清零 + 诊断 V3）

### 9.1 为什么改

比赛任务是**四圈**，而代码只跑一圈就停：`mission_control.c:132` 原来是 `if (m->mode != MISSION_MODE_FULL || m->segment == SEGMENT_DA) { state=MISSION_DONE; }`，`endpoints[]={C,B,D,A}` 走完即完，全工程没有任何计圈变量。四圈 ≈ 4 × 34 s ≈ 136 s，必须显式实现。

多圈在几何上是自洽的：DA 段出线航向 0°，下一圈 AC 要 −38.66°，误差 −38.66°，与第一圈完全一致；`course_origin_deg` 原来只在 `MissionControl_Start` 取一次，本身不会随圈数漂移。

### 9.2 改动

| 文件 | 改动 |
| --- | --- |
| `Core/Src/Mission/mission_config.h` | 新增 `MISSION_FULL_LAPS 4U`（FULL 模式的圈数；单段模式恒为 1 圈） |
| `Core/Src/Mission/mission_control.h` | `MissionControl` 增加 `lap`（已完成的圈数，0 基）、`laps`（本次圈数）、`laps_override`（台架覆盖，0 表示按模式默认）、`drift_mdeg`（各圈在 A 点被抵消的航向残差累计） |
| `Core/Src/Mission/mission_control.c` | `Start`：`laps = laps_override ? laps_override : (FULL ? MISSION_FULL_LAPS : 1)`；`Update`：DA 完成时若 `lap+1 < laps`，先用 `residual = wrap(YAW_SIGN*(yaw − course_origin))` 累计 `drift_mdeg`、再用当前 `yaw_deg` 重取 `course_origin_deg`、`++lap`、`segment=SEGMENT_AC` 并 `Enter`，否则 `MISSION_DONE` |
| `Core/Src/Mission/mission.h` / `mission.c` | 诊断 ABI 144 → **152 B（V3，`version=3`）**：在 `sequence_end` 之前插入 `lap`（word 35）、`drift_mdeg`（word 36），`sequence_end` 移到 word 37；`_Static_assert` 同步为 `152U`；新增 `Mission_SetLaps()` 台架接口（运行中拒绝修改） |
| `scripts/read_mission.ps1` | 读长度 `0x90 → 0x98`（38 字）、版本校验 `2 → 3`、尾字校验索引 `35 → 37`，新增 `Lap` / `Drift_mdeg` 两个输出字段 |
| `tests/test_mission_control.c` | `Init` 默认 `laps_override=1`；新增两圈用例：DA 完成后回卷到 AC、`lap=1`、`arrivals=4` 不清零、`course_origin_deg` 重取为当前 yaw、`drift_mdeg` 精确等于注入的 5° 残差 |
| `tests/test_mission_runtime.c` | `Mission_SetLaps(1)`，保持调度器用例仍是「单圈 + 显式重来」 |
| `tests/test_mission_actual_route.c` | 跑满**四圈**：`arrivals==16`、`lap==3`、`event_count==16`、到点序列按 `arrivals%4` 循环；诊断 dump 改为每行最多 4 字（38 字不再整除 4，避免越界读） |
| `tests/test_mission_reader.ps1` | 校验 V3、`Arrivals=16`、`Lap=3`、`Event_count=16` |

### 9.3 验证

- 主机测试 **14/14** 通过；`mission_actual_route_test` 四圈合成耗时 **137760 ms**，与理论 136 s 吻合（合成模型直接按命令轮速积分，属于交叉验证）。
- FULL 构建：RAM 12664 B（25.76%）、FLASH 51452 B（19.78%）。
- 单圈调试：把 `MISSION_FULL_LAPS` 改成 `1U`，或调 `Mission_SetLaps(1)`；单段固件（`-Mode AC/CB/BD/DA`）不受影响，仍是只跑一段就 `DONE`。

### 9.4 上板要看的三件事

1. `Lap` 是否从 0 递增到 3 后进 `DONE`；每圈在 A 点应有蜂鸣 + LED 提示（`MISSION_POINT_SIGNAL_MS` 300 ms），四圈共 16 次到点提示。
2. `Drift_mdeg` 是每圈在 A 点被抵消掉的航向残差累计。若每圈都往同一方向累加，说明是 IMU 零偏而非随机误差 → 可以把均值反哺 `Core/Src/Modules/imu_config.h` 的 `IMU_Z_BIAS_PRIOR_DPS`；若符号来回跳，则是轮速/打滑造成的，不用动 IMU。
3. 四圈 136 s 里电池压降会让后几圈偏慢，`Elapsed_ms` 与每圈耗时（可对比 `arrivals` 的时间戳）能看出掉速趋势。若第四圈明显慢，先查电压再谈调参。

---

## 10. 2026-10-27 第五轮：缩短停车等待（方案 B）

### 10.1 为什么改

四圈 132.96 s 里，真正被"等待"吃掉的约 9.6 s：路线上一共 8 处刹停（A 点 ×4、B 点 ×4，C/D 已是滚动入线不停车），每次刹停有**两个纯等待窗口**——入段的 `PHASE_STILL` 与转完后的 `PHASE_SETTLE`，原来各 500 ms。

```
每次刹停：刹车 0.2 + STILL 0.5 + TURN 1.35 + SETTLE 0.5 + 重加速 0.3 ≈ 2.85 s
8 次 × (0.5+0.5-0.2-0.2) = 4.8 s        ← 本次要省掉的就是这一块
```

这两个窗口都不改变几何，只是"等它别动"。而"它别动"这件事真正的证据是**角速度 ≤3°/s 且两轮 <60 cps**（`Still()`），编码器 10 ms 一采样，200 ms 已经是 20 个连续样本，比 500 ms 只少一点冗余，不损失任何判据强度。

### 10.2 改动

| 文件 | 改动 |
| --- | --- |
| `Core/Inc/motion_tuning.h:88-95` | 原来是单个 `MOTION_SETTLE_MS 500U` 被两个窗口共用；现在拆成 `MOTION_STILL_MS 200U`（入段静止确认）与 `MOTION_SETTLE_MS 200U`（转到位后停稳确认），并加注释说明"若实车出现还在滚就起转，就把这两个值调回 300" |
| `Core/Src/Mission/motion_straight.c:9-14` | `Settled()` 增加 `uint32_t window_ms` 参数，不再读死宏；`PHASE_STILL` 传 `MOTION_STILL_MS`、`PHASE_SETTLE` 传 `MOTION_SETTLE_MS`，两个窗口因此可以独立调 |
| `README.md:32-33` | 参数表拆成两行（`MOTION_STILL_MS / MOTION_SETTLE_MS` 均 200 ms；`STILL_RATE_DPS / STILL_WHEEL_CPS`） |
| `README.md:88` | "静止条件需连续满足至少 500 ms" 改为引用两个宏，并说明 10 ms 采样下 200 ms 已是 20 个连续样本 |

计划文档第 4 节里"`MOTION_SETTLE_MS 500→300`"（D4）的目标由本节一次性做到，D4 视为完成。

### 10.3 验证

- 主机测试 **14/14** 通过（`test_motion_algorithms.c:181` 用的是宏 `in.now_ms+=MOTION_SETTLE_MS`，自动跟随新值，不需要改断言）。
- `mission_actual_route_test` 四圈合成耗时 **137760 → 132960 ms**，正好少 4.8 s，与"8 次刹停 × 2 个窗口 × 0.3 s"完全对上。
- FULL 构建：RAM 12664 B（25.76%）、FLASH 51456 B（19.78%）。
- 未上板（ST-LINK 未连接）。

### 10.4 上板要看的

1. A/B 折角**起转那一帧**，车是否真的已经停住——若出现"边滚边起转"，把 `MOTION_STILL_MS` 单独调回 300（`MOTION_SETTLE_MS` 可以不动）。
2. 转完进直线时若出现"车还在晃就开始走"，同理调 `MOTION_SETTLE_MS`。
3. 其余指标（`Lap`、`Drift_mdeg`、`Turn_corrections`）应与第 9 节完全一致——本次不改变任何几何。

---

## 11. 速度上限：用合成模型扫出来的真实边界（2026-10-27）

### 11.1 扫描结果

只改 `Core/Inc/motion_tuning.h` 里的巡航/捕获速度，重编 `mission_actual_route_test` 后跑四圈：

| 直道 AC/BD | 循迹 CB/DA | 捕获 | 四圈合成耗时 | 结果 |
| --- | --- | --- | --- | --- |
| 1600 | 1000 | 600 | **132 960 ms** | 通过（当前值） |
| 2000 | 1400 | 600 | **106 560 ms** | 通过 |
| 2400 | 1600 | 600 | **95 520 ms** | 通过 |
| 2800 | 2000 | 800 | **83 520 ms** | 通过（贴边） |
| 3200 | 2400 | 1000 | — | **失败**：5260 ms CB 段 `detail=10`(CAPTURE)、`dist=200` |
| 3200 | 2400 | 800 | — | **失败**：5370 ms 同上、`dist=201` |

耗时基本按"命令轮速积分"线性下降，说明**限制不在时间上，而在捕线预算上**。

### 11.2 失败机制：直道巡航速度会吃掉捕线距离预算

从直道巡航降到捕线速度是斜坡（`MOTION_TARGET_RAMP_CPS_S 5000`），这段斜坡本身就在往前走：

```
斜坡距离 = [c_capture·Δ/5000 + Δ²/10000] / 6.7301   mm   (Δ = c_cruise − c_capture)
2800→800：107 mm      3200→800：143 mm
```

`MOTION_CAPTURE_MAX_MM 200` 只剩 93 mm / 57 mm 给"转掉 38.66°"，而 800 cps 下 57 mm 只有 0.48 s，滚动入段收敛需要约 0.65 s → 必然 `MOTION_FAULT_CAPTURE`。这解释了 2800 能过（贴边）、3200 必挂。

**结论**：想上 2800 以上的直道速度，必须先解决这个耦合，两个办法：
1. C/D 判点前在直道末尾 N mm 内把巡航预降到 `capture_cps`（推荐，顺带让入段更稳）；
2. 或把 `MOTION_CAPTURE_MAX_MM` 从 200 提到 300（简单，但入段搜索距离变长，可能冲出弯道）。

### 11.3 循迹段的天花板不是转向限幅，而是 PD 的稳态偏置

巡线 PD 没有积分项（`motion_line.c:55`：`turn = KP*pos + KD*d − heading_trim`），而圆弧需要**持续的**差速才能维持角速度 `ω = v/R`：

```
需要 turn = 0.5 · (v/R) · 轨道宽 / 6.7301 = 0.1625 · cruise            (R = 400 mm, 轨距 130 mm)
稳态偏置 pos_ss = turn / MOTION_LINE_KP = 0.903 · cruise                (KP = 0.18)
```

即：巡航 1000 cps 时车是**贴着线外侧约 900 位置单位**（量程 ±3500）跑的，巡航 2000 时是 1806 单位。这带来两个后果：

1. 速度被自己扣掉：`base = cruise − |pos|·0.10`，900 单位就是 −90 cps；
2. 提速后余量迅速变薄，线一旦滑到传感器边缘就是 `NO_LINE`。

所以循迹提速的正解不是加大 `MOTION_LINE_STEERING_MAX_CPS`（现在根本没饱和），而是：
- **加曲率前馈**：`turn_ff = 0.1625 · cruise`（半圆弧段 FOLLOW 期间常开），稳态偏置直接归零，PD 只管扰动；
- 或退一步把 `MOTION_LINE_KP` 0.18 提到 0.26~0.30，偏置减半（代价是 1000 单位的量化跳变会放大成 260~300 cps 的 turn 抖动量）；
- 同时把 `MOTION_LINE_PERIOD_MS` 50 → 20，否则 300 mm/s 时每 15 mm 才修正一次。

### 11.4 执行器余量（为什么说"没到理论上限"是对的）

- 实测标定：左轮 50% 占空比 = 3346 cps、右轮 = 2727.5 cps（`speed_control.c:8-9` 就是这两条拟合）。
- `MOTION_PWM_LIMIT 65`：2800 cps 时前馈只有左 42%、右 51%，加积分仍在 65 以内；3200 cps 才真正贴住天花板。
- 所以**电机不是一个近在眼前的墙**，真正的墙依次是：捕线预算（11.2）→ PD 稳态偏置（11.3）→ 传感器条宽度与 10 ms 帧率 → 最后才是 PWM/电池。

## 12. 2026-10-27 第四轮：曲率前馈 + 转向提速（已落地、未烧录）

### 12.1 改了什么

| 文件 | 改动 |
| --- | --- |
| `Core/Inc/motion_tuning.h` | 新增 `MOTION_LINE_CURVATURE_FF (-0.1625f)`；`MOTION_TURN_ANGLE_KP 2.0 → 3.2`、`MOTION_TURN_RATE_MAX_DPS 50 → 65`、`MOTION_TURN_MAX_CPS 1200 → 1400` |
| `Core/Src/Mission/motion_line.c` | `Targets()` 增加 `following` 参数，FOLLOW 期先算 `turn_ff = FF*cruise`，再让 PD 只在 `authority = cap − |turn_ff|` 内工作：`turn = turn_ff + clamp(KP*pos + KD*d − trim, ±authority)` |
| `Core/Src/Mission/motion_control.c` | `Motion_ValidateTuning` 增加 `MOTION_LINE_CURVATURE_FF` 的 `isfinite` 与 `|FF| ≤ 0.5` 校验 |
| `tests/test_motion_algorithms.c` | 居中线断言改为"差速已由前馈给出"（`right−left` 在 250~400 cps）；转向接近段断言改为"高于 `MOTION_TURN_APPROACH_MIN_CPS` 且低于半速"（旧的 `< MOTION_TURN_MIN_CPS` 是在 KP=2 下标定的绝对值） |
| `tests/test_mission_actual_route.c` | 圆弧巡线反馈从"固定偏置"改成"航向相对切线滞后"（`pos = 100 * lag`，`lag = yaw − tangent`）：以前偏置是常量，圆弧转多少圈全看时间×差速，只能靠撞对；现在闭合到航向，能真正验证前馈方向对不对 |

### 12.2 前馈符号（最容易搞错的一处）

赛道逆时针跑：CB 扫 0°→180°、DA 扫 180°→360°，两个半圆都是左转。左转要求右轮领先，即 `left = base + turn` 里的 `turn` 必须为负，所以常量取 **−0.1625**。若把赛道改成顺时针，这个常量要取正号（注释已写在宏上方）。

### 12.3 效果（合成轨迹，非实车）

| 指标 | 改前 | 改后 |
| --- | --- | --- |
| 四圈合成耗时 | 132 960 ms | **128 040 ms**（−4.9 s，每圈约 −1.2 s） |
| 主机测试 | 14/14 | 14/14 |
| FLASH | 51 456 B | 51 540 B（19.82%） |

省下的时间由两块构成：圆弧段 `base` 不再被 `|pos|·0.10` 自扣（稳态偏置归零），以及 A/B 两处原地转向从约 1.35 s 缩到约 0.9 s。

### 12.4 上板要看什么

1. **A 点转向是否还落得准**：KP 3.2 + 65 dps 下，刹车提前量仍是固定 `MOTION_TURN_BRAKE_LEAD_S 0.04`。若 `Turn_corrections` 变成 1（即一次纠偏），说明过冲超过 `ACCEPT_DEG 3°`，把 `MOTION_TURN_ANGLE_KP` 退回 2.8 或把提前量加到 0.05。
2. **圆弧是否贴线**：跑完一圈看 `Line_position` 是否还在 ±1000 以内。前馈正确时车应该基本居中（以前会稳定偏 ~900 单位）；若持续偏向一侧且 `|pos| > 2000`，先怀疑前馈符号或半径假设（赛道半圆半径不是 400 mm 就按比例改 FF）。
3. **捕线是否还稳**：前馈只在 FOLLOW 生效，CAPTURE 期仍是纯 PD + `heading_trim`，行为不变。

### 12.5 一圈的时间预算（合成轨迹逐相位实测，1600 / 1000 / 600）

| 相位 | 一圈耗时 | 说明 |
| --- | --- | --- |
| `PHASE_STRAIGHT` | 11 310 ms | AC + BD 共 2 562 mm @ 237.7 mm/s，含两段斜坡 |
| `PHASE_CAPTURE` | 1 600 ms | 两个弯各约 800 ms：滚着把 38.66° 弯角转掉，直到 `|entry_error| ≤ 20°` 才确认 |
| `PHASE_FOLLOW` | 16 840 ms | 两个半圆 2 514 mm，平均 ≈149 mm/s——**一圈里最大的一块，占 52%** |
| `PHASE_TURN` | 1 610 ms | A、B 各约 805 ms（`ANGLE_KP 3.2` + 65 dps） |
| `PHASE_STILL` + `PHASE_SETTLE` | 420 + 420 ms | 两次刹停 × (200 + 200 ms) |
| 合计 | ≈ 32 000 ms | 四圈 128 040 ms |

一次"刹停 + 转向"的完整开销 = 200(静止) + 805(转) + 200(停稳) ≈ **1.2 s**，已低于 1.5 s 的目标。

### 12.6 提速阶梯（同一套代码、同一测试，逐档实测）

| AC/BD | CB/DA | capture | 四圈 | 一圈 | 结果 |
| --- | --- | --- | --- | --- | --- |
| 1600 | 1000 | 600 | 128 040 ms | 32.0 s | 当前默认 |
| 2000 | 1400 | 600 | 100 200 ms | 25.1 s | 通过，推荐第一档 |
| 2400 | 1600 | 600 | 88 760 ms | 22.2 s | 通过 |
| 2800 | 2000 | 800 | 76 680 ms | 19.2 s | 通过（捕线已需提到 800） |
| 3200 | 2400 | 1000 | — | — | 失败：5 060 ms CB `CAPTURE` `dist=200`，斜坡吃掉 137 mm，只剩 63 mm 转不完 38.66° |

加速的收益几乎全在 `FOLLOW`（1000→2000 时 16 840 → 7 920 ms），直道段则是 11 310 → 6 970 ms。再往上必须先解决 11.2 的捕线预算耦合（C/D 前预降速，或把 `MOTION_CAPTURE_MAX_MM` 提到 300）。

## 13. 2026-10-27 第五轮：巡航提到 2000 / 1400（已落地、未烧录）

| 文件 | 改动 |
| --- | --- |
| `Core/Inc/motion_tuning.h:38-41` | `MOTION_AC/BD_CRUISE_CPS 1600 → 2000`、`MOTION_CB/DA_CRUISE_CPS 1000 → 1400`（capture 仍 600，`STEERING_MAX_CPS` 仍 650，`LINE_PERIOD_MS` 仍 50） |
| `tests/test_motion_algorithms.c` | 圆弧差速断言改成随 `p->cruise_cps` 参数化（`2·|FF|·cruise`），不再写死 250~400 cps，否则改巡航速度就得改测试 |
| `README.md` | 提速阶梯表的 "Existing" 列同步为 2000 / 1400 |

验证：主机 14/14；四圈合成 **128 040 → 100 200 ms**（一圈 32.0 → 25.1 s）；`FOLLOW` 16 840 → 11 840 ms、`STRAIGHT` 11 310 → 9 240 ms；FULL 构建 RAM 12 664 B、FLASH 51 540 B（19.82%）。ST-LINK 未连接，**未烧录**。

### 13.1 探针记录（只测不改，用于回答"弯道还能怎么快"）

| 配置 | 四圈 | `FOLLOW` | 结论 |
| --- | --- | --- | --- |
| 2000 / 1400 / 600，cap 650，loop 50 ms | 100 200 ms | 11 840 | 已采用 |
| 2800 / 2000 / 800，`STEERING_MAX_CPS` 900 | 76 240 ms | 7 840 | 合成轨迹通过，但**合成模型给不出转向余量的收益** |
| 同上 + `MOTION_LINE_PERIOD_MS 20` | 76 320 ms | 7 840 | 时间不变（模型里没有延迟），只能上板看 |

合成轨迹把 `speed_control.left/right_reference_cps` 直接当轮速真值，没有打滑、没有轮胎侧偏，所以"加大转向差速上限"和"加快巡线环"这两项在模型里必然看不到收益——它们是实车量，只能用实车判断。

## 14. 2026-10-27 实车故障：过 C 点后不转向、直行到 NO_LINE

### 14.1 现象

烧录 2000 / 1400 那版后实车跑 A→C 正常，C 点声光提示正常，随后**没有任何转向动作**，直着开过去，显示屏 "no line"。

### 14.2 现场读数（`.\scripts\read_mission.ps1`）

| 字段 | 值 | 含义 |
| --- | --- | --- |
| `State` / `Reason` | `FAULT` / `SEGMENT_FAULT` | 段内故障 |
| `Segment` / `Segment_fault` | `CB` / `11` (`NO_LINE`) | 不是捕线超时，是**丢线** |
| `Distance_mm` | **100** | 进段只走了 100 mm 就结束 |
| `Heading_error_deg` | **39.487°** | 入段该转的 38.66° **一点没转** |
| `Capture_frames` | 0 | 从未确认到窄线 |
| `Line_mask` / `Line_position` | `0x00` / `-3500` | 探头下全白，最后一次有效位置在最左 |
| `Speed_fault` | 0 | 不是堵转，与上一轮的 `LEFT_NO_FEEDBACK` 无关 |

### 14.3 根因（三段串联）

1. **入段速度太高，差速被减速吃光**：C 点是 2000 cps（297 mm/s）巡航交接到 capture 600 cps 的，PI 要用 0.28 s 把目标差速 `150 / 1050` 落下来，这期间两轮都在从 2000 往下刹，**真实轮速差接近 0**，车走直线。而这一转必须在 **58 mm** 内完成，减速斜坡本身就要走 70 mm。
2. **过完黑条后探头下没有线**：C 点后面的黑条是唯一压在探头上的黑色；圆弧线在车体左前方，车又正带着 −38.66° 直行，**前约 80 mm 探头全白**（`Line_mask=0x00`，与 `Distance_mm=100` 吻合）。
3. **丢线判据在 CAPTURE 期只有 350 ms**：`MOTION_CAPTURE_UNCERTAIN_MS 350`，车在第 1 条里没转、在第 2 条里看不到线，350 ms 后直接 `Motion_Fail(NO_LINE)` → 显示屏 "no line"。

**为什么滚动入线的设计没兜住**：`motion_line.c` 的丢线分支只对 `align_capture` 保留 `heading_trim`，时间上却仍受 350 ms 约束；其余分支一律 `recovery_left = recovery_right = capture_cps`（正对称爬行，等于**主动放弃转向**）。

### 14.4 修复

| 文件 | 改动 |
| --- | --- |
| `Core/Inc/motion_tuning.h` | `MOTION_CAPTURE_TIMEOUT_MS 1500 → 2000`、`MOTION_CAPTURE_MAX_MM 200 → 320`（盲区 + 38.66° 旋转都要装得下）；新增 `MOTION_STRAIGHT_HANDOFF_CPS 600`、`MOTION_POINT_APPROACH_MM 140` |
| `Core/Src/Mission/motion_straight.c` | 判点前 140 mm 把巡航线性降到 `MOTION_STRAIGHT_HANDOFF_CPS`，**弯道差速留给转向而不是拿去减速**；判点窗口与判点逻辑不变 |
| `Core/Src/Mission/motion_line.c` | 丢线分支新增 `owing_tangent`：入段航向差仍 > `MOTION_CAPTURE_HEADING_ACCEPT_DEG` 时**不算故障**、按陀螺仪继续转，并把无反馈计时戳一直刷新；只有转到 20° 以内后，丢线才重新开始 350 ms 计时 |
| `tests/test_motion_algorithms.c` | 新增"实车级丢线"回归用例：入段 −38.66°、连续 25 帧全白，断言**仍然 FORWARD 且左右差速 ≥ 300 cps**（不允许直行）、不许故障；转到切线后再全白才允许 `NO_LINE`。同时把捕线超时/距离上限的断言改成引用宏，避免以后改预算又改测试 |

### 14.5 验证

- 主机 **14/14 通过**（新增丢线用例在内）。
- 四圈合成 **100 200 → 120 600 ms**（一圈 30.2 s）：`STRAIGHT` 9 240 → 14 320 ms（判点前降速的代价），`CAPTURE` 1 760 → 1 280 ms，`FOLLOW` 11 840 → 12 360 ms。**仍然比 1600/1000 那版（128 040 ms）快**。
- FULL 构建：RAM 12 664 B（25.76%）、FLASH **51 764 B**（19.90%）。

### 14.6 上板预期与回退

1. 过 C 点应看到**先减速、再持续左转**，2 s / 320 mm 内抓住圆弧线；若 `Segment_fault` 仍是 `NO_LINE` 且 `Distance_mm ≈ 320`，说明 3 条里第 2 条仍成立（探头根本没扫到线），下一步是加大入段差速上限 `MOTION_CAPTURE_HEADING_MAX_CPS` 或把 `MOTION_CAPTURE_HEADING_KP` 从 10 提到 14。
2. 若 `Distance_mm` 停在 ~200 且 `Segment_fault=CAPTURE`，是转过了头、3 帧确认没凑齐 → 把 `MOTION_CAPTURE_CONFIRM_FRAMES` 3 降到 2。
3. 嫌判点前降速太慢（一圈 +5 s），把 `MOTION_STRAIGHT_HANDOFF_CPS` 提到 900、`MOTION_POINT_APPROACH_MM` 保持 140：差速仍有 450 cps 权限，代价约每秒 1 圈 2 s。
4. 完全回退本次改动只需把 `MOTION_CAPTURE_MAX_MM` 改回 200、`MOTION_POINT_APPROACH_MM` 改 0（降速段自动失效），但**不要**回退 `owing_tangent` —— 那是这次"不转向"的直接病根。

## 15. 2026-10-28 第四次上板：判点前降速把车降停了（SPEED_FAULT）

### 15.1 现象

用户实测："小车 AC 段的行驶距离不够，显示屏显示 SPEED FAULT"，现场卷尺量 A→C 约 **125 cm**。

### 15.2 读数

`mission_diag`：`State=FAULT`、`Reason=SPEED_FAULT`、`Segment=AC`、`Segment_fault=2`、`Phase=STRAIGHT`、`Distance_mm=996`、`Elapsed_ms=6971`、`Turn_corrections=1`、`Heading_error_deg=0.733`、`Speed_fault=2`（`RIGHT_NO_FEEDBACK`）。

`speed_fault_data`（0x20000234，13 字）：

| 字段 | 值 | 说明 |
| --- | --- | --- |
| `mode` | 0 | 正常前进模式（不是原地转向） |
| `sample_ms` | 14 214 | 故障发生的速度环时间 |
| `fault` | 2 | `RIGHT_NO_FEEDBACK` |
| `left_idle_ms` / `right_idle_ms` | 340 / 360 | 两轮**同时**零计数超过 250 ms 判据 |
| `left_pwm` / `right_pwm` | 22 / 25 | 占空比只有 22~25% |
| `left_reference` / `right_reference` | 623 / 637 | 正是降速目标（cruise 2000 → handoff 600 的中段） |
| `left_cps` / `right_cps` | 0 / 0 | 轮子确实没转 |
| `integral` | +6.8 / +6.6 | 积分从巡航时的 +20 被负误差放掉，只剩 6.8 |

### 15.3 根因：不是识别窗口，是降速降过头

里程 996 mm 恰好在"判点前 140 mm 降到 `MOTION_STRAIGHT_HANDOFF_CPS`(600)"的斜坡末段。600 cps 的前馈占空比只有 `0.0149*600 ≈ 9%`，加上正在放掉的积分约 22~25%——**低于这辆车破静摩擦所需的 ~34%**（A 点原地转向的实测门槛）。于是减速斜坡把两轮带到 0 之后再也起不来；积分每秒只能涨 `Ki*ref ≈ 18%`，而 250 ms 的 `NO_FEEDBACK` 判据先到。

三条并存的因素：① 低速段占空比不足；② `ControlWheel` 没有破静摩擦助推（只有原地转向的 `ParkingWheel` 有）；③ 降速锚在 `point_min`（窗口下限 1000），比标称点 1281 提前 141 mm 就开始减速。

### 15.4 修复

| 文件 | 改动 | 理由 |
| --- | --- | --- |
| `Core/Inc/motion_tuning.h` | `MOTION_STRAIGHT_HANDOFF_CPS 600 → 1200`（178 mm/s） | 1200 cps 的占空比 ≈ 18% + 积分，稳定在滚动区；滚着转的入段差速仍然够用（capture 1000 时权限 650） |
| `Core/Inc/motion_tuning.h` | `MOTION_CB/DA_CAPTURE_CPS 600 → 1000` | capture 600 同样处在"降下去就起不来"的区间；1000 cps 是这辆车能稳定滚动的下限附近，同时弯道 320 mm 预算仍装得下 38.66° |
| `Core/Inc/motion_tuning.h` | 新增 `MOTION_BREAKAWAY_MIN_CPS 200`、`MOTION_BREAKAWAY_STALL_CPS 60`、`MOTION_BREAKAWAY_PWM 15` | 破静摩擦助推的三个阈值，详见 README 参数表 |
| `Core/Src/Modules/speed_control.c` | `ControlWheel` 末尾：`|reference| ≥ 200 && |measured| < 60` 时给 `sign(reference)*15%` 占空比 | 无状态、轮子一动就消失：滚动段手感完全不变；真堵转仍会被 250 ms 判据抓住（助推不足以让卡死的轮子转起来） |
| `Core/Src/Mission/motion_straight.c` | 降速锚点从 `point_min` 改为 `nominal_mm - MOTION_POINT_APPROACH_MM` | 窗口宽（±250 mm）不代表可以提前减速；应当按**标称长度**来减速 |
| `tests/test_motion_algorithms.c` | 入段差速断言改为按 `entry_trim = min(KP·|entry_error|, min(CAPTURE_HEADING_MAX_CPS, capture − MIN_WHEEL))` 精确比对；降速用例距离改用 `nominal − 20` | `capture 600 → 1000` 后旧断言（写死 `2·(capture − 2·MIN_WHEEL)`）与 `900 mm` 距离都不再成立 |

### 15.5 验证

- 主机 **14/14 通过**。
- FULL 构建：RAM 12 664 B（25.76%）、FLASH **51 952 B**（19.97%）。
- 已烧录：`Download verified successfully` + `MCU Reset`；烧后读回 `State=IDLE`、`Reason=READY`、`Version=3`、`Lap=0`。

### 15.6 关于"把识别窗口改成 1200–1300 mm"

**不建议**，理由：

1. 本次故障发生在 `Distance_mm=996`，此时连窗口下限 1000 都还没进——窗口再宽再窄都不影响这次故障。
2. 窗口 `[1000, 1550]` 的作用是容忍里程误差（轮滑、转向时的位移不计入里程、探头触发延迟 `0.02·v`）。实测 A→C 物理距离约 1250 mm，落在窗口中部；把窗口收窄到 `[1200, 1300]`（±50 mm）后，只要某一圈里程偏 60 mm 就**必然丢点**，而且是整段跑完才丢，代价比现在大得多。
3. 判点本身还要求"连续 3 个新帧同色"，窗口越窄、帧数要求不变，等于把两者的交集压得更小。
4. 若以后确实发现里程系统性偏短（例如 `Distance_mm` 稳定停在 900），正确做法是**同时**下调 `point_min`/`point_max`（例如 `[850, 1400]`），而不是把带宽压窄。

### 15.7 上板要看的三件事

1. AC 段是否跑到 ~1250 mm 且**不再报 SPEED_FAULT**（这是本次修复的直接目标）。
2. 过 C 点后是否**先减速到 1200 cps 附近、再持续左转**；`Capture_frames` 应从 0 涨到 3、`Phase` 进 `FOLLOW`。
3. 若仍报 `RIGHT_NO_FEEDBACK`，读 `speed_fault_data` 的 `left_pwm/right_pwm`：若仍 < 30%，说明这个底盘的破静摩擦门槛比 34% 还高，把 `MOTION_BREAKAWAY_PWM` 从 15 提到 22。

## 16. 2026-10-28 第五次上板：C 点转正了但线不在探头下（CB 抓不到线）

### 16.1 现象

用户实测："AC 段已经完全正常，但是到了 C 点车头回正的时候我觉得小车的灰度识别模块已经难以精确识别到了黑线了，CB 段的程序没有正常运行"。

### 16.2 读数

| 字段 | 值 | 含义 |
| --- | --- | --- |
| `State` / `Reason` | `FAULT` / `SEGMENT_FAULT` | 段内故障 |
| `Segment` / `Segment_fault` | `CB` / `11`(`NO_LINE`) | 又是丢线 |
| `Distance_mm` | **174** | 进段走了 174 mm 才放弃 |
| `Heading_error_deg` | **3.775°** | 38.66° 已经转掉了——**这次转向本身是成功的** |
| `Capture_frames` | 0 | 174 mm 内一帧都没确认 |
| `Line_mask` / `Line_position` | `0x00` / `-3500` | 探头下全白 |
| `Speed_fault` | 0 | 不是堵转 |

结论：转向没问题，是**车被带到了线外**，探头压根没机会看到黑线。

### 16.3 根因（两处，用数值仿真定位）

**① 冻结切线必然积累横向偏差。** 旧捕获律把入段航向误差折算成固定差速（`heading_trim = clamp(10·err0, ±cap)`），车以 148.6 mm/s 沿半径 342 mm 的弧前进——而半圆半径是 400 mm。把 C 点当原点、切线当 +x 建弧做数值积分：

| 沿切线前进 | 车离线横向距离 |
| --- | --- |
| 50 mm | −30 mm |
| 100 mm | −58 mm |
| 150 mm | −84 mm |
| **174 mm** | **−96 mm** |
| 220 mm | −116 mm |
| 300 mm | −147 mm |

负号 = 偏在线外侧。**174 mm 时偏离 96 mm，与现场 `Distance_mm=174`、`mask=0x00` 完全吻合**——8 个探头撑到 ±50 mm 左右，96 mm 已经彻底在阵列之外。

**② 捕获的"中心门限"把边缘帧全丢了。** `MOTION_CAPTURE_CENTER_LIMIT` 原本是 **1000**（中心 ±1 个探头间距）。滚着切入时线本来就从阵列边缘进视野（位置 ±2000~3000），于是每一帧都被判"不在中心"、`capture_frames` 反复清零，永远凑不满 3 帧。

### 16.4 修复：不再"冻结切线"，改成"跟踪已知半圆"

赛道几何是已知的：每个弯就是一个半径 `nominal/π ≈ 400 mm` 的半圆。既然已知，就没有理由盲目地保持一个会越走越偏的航向。

| 文件 | 改动 |
| --- | --- |
| `Core/Src/Mission/mission_segment.h` | `MissionMotionState` 增加 `track_x/track_y/track_mm/track_origin_yaw_deg/have_track`：入段切线坐标系里的死推算位姿（+x 前、+y 左） |
| `Core/Src/Mission/motion_line.c` | 新增 `TrackAdvance()`：每帧按里程增量与相对航向积分位姿（在 `MotionArc_Update` 最开头调用，所有相位都积分）；新增 `CaptureTrim()`：`lateral = R − |车−圆心|`（+ = 线在左）、`progress = atan2f(x, R−y)`、`steer = atan2f(lateral, LOOKAHEAD)`、`error = −steer − heading`，增益按误差分档（>8° 用 40，否则 10），再叠加曲率前馈 |
| `Core/Src/Mission/motion_line.c` | 丢线分支：`align_capture`（= `PHASE_CAPTURE` + 交接入段）时**不再判 `NO_LINE`**，改为持续用 `CaptureTrim` 跟踪半圆，由 2000 ms / 320 mm 的捕获预算兜底报 `MOTION_FAULT_CAPTURE` |
| `Core/Inc/motion_tuning.h` | `MOTION_CAPTURE_CENTER_LIMIT 1000 → 3000`（阵列内任意位置都接受）；新增 `MOTION_CAPTURE_ACQUIRE_DEG 8`、`MOTION_CAPTURE_ACQUIRE_KP 40`、`MOTION_CAPTURE_LOOKAHEAD_MM 80` |
| `tests/test_arc_align.c` | 新增 `TestBlindZoneThenEdgeStripeConfirms`：入段 −38.66°、**连续 60 帧（600 ms）全白**——旧代码 350 ms 就 `NO_LINE`——断言仍 `SEG_RUNNING` 且持续前进；随后给一条**边缘单探头**条纹（位置 +2500），3 个新帧内进 `FOLLOW`（旧的中心门限 1000 会拒绝它）。另把"已对齐入段无偏置"改为"已对齐入段保留曲率前馈偏置"（`right−left = −2·FF·capture`） |
| `tests/test_motion_algorithms.c` | 捕线期丢线的旧期望（`NO_LINE`）改为"预算耗尽才 `MOTION_FAULT_CAPTURE`"；饱和断言改为按速度余量比对 |

**为什么要饱和打满而不是慢慢纠**：`ACQUIRE_KP 40` 在 8° 以外的误差上直接顶到轮速余量（capture 1000 时 850 cps 差速 ≈ 111 dps），把 38.66° 用**最短半径**掰过来——这是几何上能给出的最小鼓包；误差进 8° 后再交给 10 的跟踪增益收横向偏差。同一套仿真下峰值横向偏差 **20 mm**、320 mm 内收敛到 **4 mm**；若全程用 KP=10 不饱和，峰值 43 mm。

**前视距离为什么是 80 mm**：以弦代弧、R=400 时，30 mm 的直线偏差要走到 `x = √(2·30·R) ≈ 155 mm` 才形成，也就是"线进入视野那一刻的偏离"由前视距离决定。150 mm 前视对应 47 mm 提前量（会切进线内侧来回摆），80 mm 对应约 13 mm 提前量，实测最优。

### 16.5 验证

- 主机 **14/14 通过**（含新的 600 ms 盲区用例与边缘条纹确认）。
- 四圈合成 **99 680 ms**（比上一版 100 200 ms 略快）：`CAPTURE` 1 280 ms、`FOLLOW` 11 920 ms。
- FULL 构建：RAM 12 760 B（25.96%）、FLASH **58 484 B**（22.49%）。FLASH 涨了 6.5 KB，是 `CaptureTrim` 引入的 `sqrtf/atan2f/sinf/cosf`（libm 浮点版）——占 254 KB 的 22.5%，仍很宽松。
- 已烧录：`Download verified successfully` + `MCU Reset`；读回 `State=IDLE`、`Reason=READY`、`Version=3`。

### 16.6 上板要看什么

1. 过 C 点应看到**连续左转、不再直冲**；`Capture_frames` 应从 0 涨到 3、`Phase` 进 `FOLLOW`（这是本次修复的直接目标）。
2. 若 `Segment_fault` 变成 `CAPTURE` 且 `Distance_mm ≈ 320`：说明转的半径还不够小或线真的更靠外——先看 `Line_mask` 有没有任何一位变成 1（有 = 线进过视野）。
3. 若仍 `NO_LINE`：**不可能**再由捕获期触发（已改成只报 `CAPTURE`），若出现说明故障来自 FOLLOW 期，需看 `Phase`。
4. 回退方法：把 `MOTION_CAPTURE_CENTER_LIMIT` 改回 1000、删掉 `align_capture` 分支即回到"冻结切线"版本——但那一版已用数值证明 174 mm 偏 96 mm，不建议回退。

## 17. 2026-10-28 第六次上板：B 点左转角多转约 5°

### 17.1 现象与判断

用户实测："AC-CB 段运行得很好，虽然中间有点问题但是瑕不掩瑜。现在问题出现在 BD 段，小车在 B 点停车转向的角度不对，我感觉小车应该向左转向的角度再减小 5° 左右（目测预估）"。

B 点是**停车后原地左转**（`motion_straight.c` 的 `PHASE_STILL → PHASE_TURN → PHASE_SETTLE`），转到的绝对目标航向是 `MOTION_BD_TARGET_DEG = 180 + 38.66 = 218.66°`。转向本身有 `ACCEPT_DEG 3°` 的收敛判据与最多 3 次纠偏，所以"多转 5° 还落得住"说明这不是收敛失败，而是**场地在 B 处的实际折角比标称小**（标称按 AB=1000 mm / AD=BC=800 mm 的矩形推出的 38.66°）。

### 17.2 改动

| 文件 | 改动 |
| --- | --- |
| `Core/Inc/motion_tuning.h:45-52` | 新增 `MOTION_BD_CORNER_OFFSET_DEG (-5.0f)`，`MOTION_BD_TARGET_DEG` 改为 `180.0f + 38.66f + MOTION_BD_CORNER_OFFSET_DEG` → **218.66° → 213.66°**，B 处左转角 38.66° → 33.66° |
| `tests/test_motion_algorithms.c:97` | 原来断言 `heading_error == 48.66f`（写死 218.66−170）；改为按宏推导 `Motion_WrapDegrees(p->target_yaw_deg - 170.0f)`，以后调这个偏移测试不会再挂 |
| `README.md` | 段表 BD 目标航向改 213.66°（等价 −146.34°）并说明修正来源；参数表新增 `MOTION_BD_CORNER_OFFSET_DEG` 一行 |

**为什么只改 BD 一处**：AC 的 −38.66°、C/D 的出弯切线（CB 0°、DA 180°）、A 点转向都没有被用户报告偏差；把偏移做成独立宏而不是去改公共的 38.66，可以让每一处独立标定。

### 17.3 验证

- 主机 **14/14 通过**。
- 四圈合成 **99 400 ms**（B 点少转 5° 省 0.28 s/圈）：`TURN` 1 530 ms、`STRAIGHT` 9 550 ms、`CAPTURE` 1 160 ms、`FOLLOW` 12 040 ms。
- FULL 构建：RAM 12 760 B（25.96%）、FLASH 58 484 B（22.49%）——宏改动不占空间，数值未变。
- 已烧录：`Download verified successfully` + `MCU Reset`。

### 17.4 上板与后续

1. 若 B 点现在**正好对正**，说明 −5° 就是场地误差，保留。
2. 若变成**反方向差 2~3°**（车头偏右），把 `MOTION_BD_CORNER_OFFSET_DEG` 改成 −3；若还偏左就继续往 −7 试。每次只动这一个数，一次上板只验证一件事。
3. 改写后 D 点入弯几何同步变化：BD 走完后车头是 213.66°，DA 的入段切线是 180°，折角 33.66°——与 B 点对称，属于预期。
4. 若日后确认**场地整体**与标称矩形不符（C 点也在偏），正确做法是把 `AB/AD/BC` 尺寸重新量一遍、按 `atan2` 重算 38.66°，而不是逐个点加偏移。

## 18. 2026-10-28 第七次上板：回到 A 点后转向把车拐进 CB 弧内侧

### 18.1 现象

用户实测："能够成功运行一圈了，但是在 DA 段 A 点转向的时候出现了巨大问题，A 点的转向直接转到 CB 段的弧线中间去了……起始点 A 和后续点 A 的转向角度不能套用，现在转向角度应该偏右 10°~15°"。

### 18.2 读数（跑完一圈后卡在 CB）

| 字段 | 值 | 含义 |
| --- | --- | --- |
| `State` / `Reason` | `FAULT` / `SEGMENT_FAULT` | — |
| `Segment` / `Segment_fault` | `CB` / `10`(`CAPTURE`) | 第二圈的 CB 捕线超限，不是丢线 |
| `Distance_mm` | 290 | 捕获预算 320 几乎用尽 |
| `Heading_error_deg` | **−44.133** | 第二圈到达 C 时，车头比 CB 入段切线差了 −44°（第一圈是 +38.66°） |
| `Capture_frames` | 0 | 一个有效帧都没确认 |
| `Lap` / `Arrivals` | **1** / **5** | 第一圈 4 个点全部命中（Arrivals=4），第二圈又到了一次 C（第 5 个） |
| `Drift_mdeg` | **+21 253** | 回到 A 点时，课程航向比 0° 多了 **+21.25°** |

### 18.3 根因：每圈在 A 点重置"课程原点"，把 21° 的姿态误差焊进了下一圈

上一版（§9）为了消 IMU 漂移，在 DA 结束、回到 A 时执行 `m->course_origin_deg = in->yaw_deg`。问题在于：

1. A 点判点确认的那一刻，车并不是精确对着"0° 方向"的——实测它已经多转了 21.25°（`Drift_mdeg`）。
2. 重置把这个**姿态**当成了"0°"。于是车"以为"自己在 0°，实际却偏了 21°。
3. 下一圈 AC 仍然按绝对目标 −38.66° 转向，转出来的实际方向就比真实对角线偏了 21°。
4. AC 段是"保持航向直行"，PD 会一直和这 21° 的偏置较劲，`trim` 打满 ±150 → 车走成一条弧线，**拐进 CB 弧内侧**——与用户描述完全一致。
5. 到了 C 点，入段切线误差从 +38.66° 变成 −44°（符号都翻了），滚动捕获按标称切线 0° 去追，追的是错方向，320 mm 用完 `CAPTURE`。

**为什么"起始 A 和后续 A 不能套用同一个角度"**：起始 A 时车是静止的、真正对准 0° 的；后续 A 时车带着 21° 的姿态误差。用同一个绝对目标，物理结果自然不同。

### 18.4 修复：课程坐标系整场只锚定一次

| 文件 | 改动 |
| --- | --- |
| `Core/Src/Mission/mission_config.h:20-27` | 新增 `MISSION_LAP_ORIGIN_RESET 0`：默认**不**在 A 点重置原点，并写明理由（重置会把判点瞬间的姿态误差锚成 0°） |
| `Core/Src/Mission/mission_control.c:139-152` | 圈回卷时**只**累加 `drift_mdeg` 作为观测，不再改 `course_origin_deg`（保留 `#if MISSION_LAP_ORIGIN_RESET` 分支，便于回退对比） |
| `tests/test_mission_control.c:81-86` | 断言改为"原点保持不变、漂移照样累计" |

**为什么这样能让后续 A 自动转对**：航向目标仍是绝对值 −38.66°，车从多少度出发就转多少度——残余 21° 由转向自己补掉（该圈 A 点会右转约 60°），转完仍精确指向真实对角线。误差不再累积，第二、三、四圈的 A 点转角和第一圈自然不同，这正是物理事实。

**IMU 漂移怎么办**：实测偏置漂移约 −0.04 dps，四圈约 140 s 累计约 −5.6°；相对"每圈 20° 级"的重置误差，这是更小的一项。`Drift_mdeg` 继续记录每圈残余，用于判断漂移是否恶化。

### 18.5 验证与工具

- 主机 **14/14 通过**（含四圈合成 route 测试）。
- FULL 构建：RAM 12 760 B（25.96%）、FLASH **58 476 B**（22.48%）。
- 已烧录：`Download verified successfully` + `MCU Reset`。
- 新增 `scripts/watch_mission.ps1`：100 ms 轮询同一份快照，**只在状态/段/相位/圈/里程(200 mm 分桶)/航向(10° 分桶)变化时打印一行**，一次跑四圈也装得下。用法 `.\scripts\watch_mission.ps1`（`-DurationSec 180` 可自动停）。这是下一轮定位 A 点转向的主要手段——快照只能看到"停在哪儿"，看不到"怎么转过去的"。

### 18.6 上板要看什么

1. 第一圈回到 A 后，第二圈的 A 点转向应当**明显比第一圈转得多**（多转约等于 `Drift_mdeg/1000` 度），转完沿真实对角线走，不再拐进弧内侧。
2. 连续跑四圈，`Drift_mdeg` 每次回 A 都应该只有个位数度（残余误差）；若某一圈突然变大（>15°），说明那一段的判点或捕获有问题。
3. 若第二圈 A 点**转过头**（车头偏右），说明残余被重复补偿了——检查是否误开了 `MISSION_LAP_ORIGIN_RESET`。
4. 若要复现旧行为做对比：把 `MISSION_LAP_ORIGIN_RESET` 改 1 重新烧录。
