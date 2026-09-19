# 2026-09-19 联盟赛 3V3 决策与离线模拟实现记录

## 1. 范围

本次只处理任务二、任务三所需的 3V3（高校联盟赛）哨兵决策与离线模拟，不扩展任务四、任务五。实现原则是复用现有 behavior_tree、导航语义和 simulator 链路，新增模块保持独立可测。

## 2. 已核实事实

- RMUL 2026 3V3 场地为 1200cm x 800cm。
- 0x0101 的 bit23-24 是 center_gain_point_status：0=未占领，1=己方占领，2=对方占领，3=双方占领。
- 现有 SetPositionLeagueSimple() 原本只有 MainGoal/PatrolGoals 固定点逻辑，无法表达控制区变化。
- 现有 simulator 能回放 DecisionTrace，但原 mock 输入没有发布 center_gain_point_status，离线实时决策只能看到未占领。
- 现有导航目标链在 NaviSetting.UseXY=false 时发布 /ly/navi/goal 语义 ID；蓝方通过现有 ResolveGoalId() 加 LocationCount 偏移。

## 3. 决策实现

核心策略文件：

- src/behavior_tree/include/League3v3Policy.hpp
- src/behavior_tree/src/League3v3Policy.cpp
- src/behavior_tree/test/test_league3v3_policy.cpp

策略输入来自：

- SelfHp：己方哨兵血量。
- EventDataFresh + CenterStatus：控制区状态。
- Friend/Enemy 快照：英雄、步兵、哨兵的血量与位置新鲜度。
- EnemyNearCenter：敌方新鲜位置是否进入控制区。

策略输出语义：

- 控制区未占领：去 CenterAnchor。
- 己方占领：血量健康时围绕中心巡逻；敌方英雄弱且可跟踪时，追击状态落到 EnemyStartApproach；敌方接近中心时回中心防守。
- 敌方占领：按己方与队友血量选择掩护、退到队友身后或争夺中心。
- 双方争夺：回 CenterAnchor，优先保持可交战位置。
- 血量低于 RecoveryHp=250：回 SelfBase，并锁存到 RecoveryExitHp=400 才退出回补。
- 控制区状态过期：回 CenterAnchor，不把过期裁判数据当成真实占领状态。

目标点 ID：

0 SelfBase
1 CenterAnchor
2 EnemyStartApproach
3 CenterOwnSide
4 CenterTop
5 CenterEnemySide

红蓝镜像由 League3v3GoalPoint() 统一处理。敌方启动区只到外围 EnemyStartApproach，不进入对方启动区/补给禁区。

2026-09-20 调整：EnemyStartApproach 红方坐标由 (950,100) 移到对面家门口对应的镜像点 (1050,630)，蓝方对应为 (150,170)，用于对敌方侧展开压制的战术落点。

## 4. 现有链路接入

- SubscribeMessage.cpp 消费 /ly/game/event_data 的 center_gain_point_status。
- GameLoop.cpp 新增 BuildLeague3v3Input()，在 League3v3.Enable=true 时优先运行 League3v3Policy。
- DecisionIntent 新增 League3v3=27，输出 league_3v3。
- DecisionTrace 输出 3V3 场地图尺寸、语义目标名和 event_center_gain_point_status。
- WaitBeforeGame.cpp 在 3V3 开局等待时保持 SelfBase，避免继续使用区域赛 Home 的 (393,810) 坐标。
- /ly/position/data 的 Y 轴转换增加了 3V3 场高 800，区域赛仍使用 1500。

## 5. 离线模拟

新增/更新：

- src/simulator/config/league_3v3.yaml
- tools/maps/basemaps/RMUL2026_3V3_topdown_field.png
- tools/maps/plugins/RMUL2026_3V3.json
- scripts/simulator/league_3v3_demo.sh
- src/simulator/sample/mock_sequences/league_3v3_center_status.json

league_3v3_demo.sh 默认做两件与区域赛 demo 不同的事：

1. 使用 league_3v3.yaml 加载 1200x800 地图和 3V3 goal 表。
2. 通过 control-bus 序列依次切换控制区状态，并关闭 runtime start gate，确保实时离线 trace 能进入 3V3 策略。

代码层同时在 3V3 profile 下保留 JSON 的 Task.Outpost=false，避免区域赛 Task.yaml 的 Outpost=true 抢先接管。

启动方式：

cd /home/milu/rm_ws/rm_project
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 42s bash scripts/simulator/league_3v3_demo.sh --trace /tmp/league3v3_trace.jsonl --every 2

浏览器 Tactical Board 默认地址：

http://127.0.0.1:9011/

## 6. 验证结果

- 全工作区 colcon build：5 个包全部成功。
- 全工作区 colcon test：505 tests，0 errors，0 failures，11 skipped。
- 实时 3V3 序列 trace 观察结果：

控制区状态 1 -> center_patrol -> center_own_side
控制区状态 2 -> contest_center -> center_enemy_side
控制区状态 3 -> defend_center -> center_anchor
控制区状态 1 -> center_patrol -> center_own_side
控制区状态 0 -> go_center -> center_anchor

- 离线 validation：errors=0；剩余 WARN 来自 mock 姿态反馈和 match-time 跳变，不是决策/坐标错误。

## 7. 仍需上机或跨仓库确认

1. 实机导航侧必须增加这套 0..5 的 3V3 goal 表和对应坐标/位姿转换。
2. 若实机 /ly/position/data 不使用联盟赛场地原点/尺度，需以机器人实际上传协议为准再次核对 Y 轴转换。
3. Chase.Enable=false 时，弱英雄追击只落到 EnemyStartApproach 语义点；现有 Chase 子系统启用后才会进一步覆盖具体追击目标。
4. 当前 mock 姿态反馈没有完整闭环，validation 会因为 posture pending 报 WARN，不影响 3V3 决策回放。
