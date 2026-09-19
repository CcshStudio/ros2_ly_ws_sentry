// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

// [BT v4] 
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/loggers/bt_minitrace_logger.h>
// bt_zmq_publisher.h 在 BT.CPP v4 已移除，改用 groot2_publisher
#include <behaviortree_cpp/loggers/groot2_publisher.h>

// [ROS 2]
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <deque>
#include <array>
#include <cstdint>
#include <fstream>
#include <string>
#include <string_view>
#include <atomic>
#include <optional>
#include <thread>
#include <mutex>
#include <utility>

#include "Utils/Logger.hpp"

#include "../module/BasicTypes.hpp"
#include "../module/json.hpp"
#include "../module/Rate.hpp"
#include "../module/Counter.hpp"
#include "../module/Random.hpp"
#include "../module/PassiveGimbalMotion.hpp"
#include "../module/Area.hpp"
#include "../module/ROSTools.hpp"

#include "Node.hpp"
#include "Topic.hpp"
#include "Robot.hpp"
#include "AimSource.hpp"
#include "AreaManager.hpp"
#include "DefaultStrategyManager.hpp"
#include "DecisionExplain.hpp"
#include "DecisionIntent.hpp"
#include "EventManager.hpp"
#include "FaceModeManager.hpp"
#include "MapCommandTask.hpp"
#include "NearGoalArrivalConfirm.hpp"
#include "PostureManager.hpp"
#include "OutpostEngagementLock.hpp"
#include "StrategyManager.hpp"
#include "TacticalProtectionPolicy.hpp"
#include "League3v3Policy.hpp"

using namespace BT;
using namespace LangYa;
using namespace Utils::Logger;
using json = nlohmann::json;

namespace BehaviorTree {

enum class RuntimeFaultCode : std::uint8_t {
    None = 0,
    LoopStall = 1,
    TickStall = 2,
    TreeEmpty = 3,
    TreeException = 4
};

enum class StrategyMode : std::uint8_t {
    LeagueSimple = 4,
    Regional = 5
};

inline const char* StrategyModeToString(const StrategyMode mode) {
    switch (mode) {
        case StrategyMode::LeagueSimple: return "LeagueSimple";
        case StrategyMode::Regional: return "Regional";
        default: return "Unknown";
    }
}

enum class CompetitionProfile : std::uint8_t {
    Regional = 0,
    League = 1
};

inline const char* CompetitionProfileToString(const CompetitionProfile profile) {
    switch (profile) {
        case CompetitionProfile::Regional: return "regional";
        case CompetitionProfile::League: return "league";
        default: return "regional";
    }
}

enum class GoalReachStatus : std::uint8_t {
    Unknown = 0,
    Traveling = 1,
    Reached = 2,
    Unreachable = 3,
    Timeout = 4
};

inline const char* GoalReachStatusToString(const GoalReachStatus status) {
    switch (status) {
        case GoalReachStatus::Unknown: return "unknown";
        case GoalReachStatus::Traveling: return "traveling";
        case GoalReachStatus::Reached: return "reached";
        case GoalReachStatus::Unreachable: return "unreachable";
        case GoalReachStatus::Timeout: return "timeout";
        default: return "unknown";
    }
}

enum class GoalReachReason : std::uint8_t {
    None = 0,
    ExternalReached = 1,
    ExternalUnreachable = 2,
    PositionDistance = 3,
    GraceActive = 4,
    PositionStale = 5,
    InvalidGoal = 6,
    Timeout = 7,
    NearGoalConfirmPending = 8
};

inline const char* GoalReachReasonToString(const GoalReachReason reason) {
    switch (reason) {
        case GoalReachReason::None: return "none";
        case GoalReachReason::ExternalReached: return "external_reached";
        case GoalReachReason::ExternalUnreachable: return "external_unreachable";
        case GoalReachReason::PositionDistance: return "position_distance";
        case GoalReachReason::GraceActive: return "grace_active";
        case GoalReachReason::PositionStale: return "position_stale";
        case GoalReachReason::InvalidGoal: return "invalid_goal";
        case GoalReachReason::Timeout: return "timeout";
        case GoalReachReason::NearGoalConfirmPending: return "near_goal_confirm_pending";
        default: return "none";
    }
}

struct GoalReachState {
    std::uint8_t GoalId{0};
    std::uint8_t BaseGoalId{LangYa::Home.ID};
    Area::Point<std::uint16_t> GoalPosition{};
    std::chrono::steady_clock::time_point GoalStartTime{};
    rclcpp::Time GoalStartStamp{};
    int GoalAgeMs{-1};
    GoalReachStatus Status{GoalReachStatus::Unknown};
    GoalReachReason Reason{GoalReachReason::None};
    std::optional<bool> ExternalReach{};
    std::optional<bool> ExternalReachable{};
    bool PositionFresh{false};
    bool HasPosition{false};
    int SelfX{0};
    int SelfY{0};
    double DistanceCm{-1.0};
    int ArriveDistanceCm{0};
    int FaceDistanceCm{0};
    bool DistanceFallbackAllowed{false};
    bool WithinArriveDistance{false};
    bool WithinFaceDistance{false};
    bool NearGoalConfirmPending{false};
    int NearGoalConfirmElapsedMs{0};
    bool Timeout{false};
};

enum class ControlOutputSnapshotSource : std::uint8_t {
    Normal = 0,
    SafeControl = 1,
};

enum class ControlTrajectoryUnavailableReason : std::uint8_t {
    None = 0,
    NotRecorded = 1,
    PublisherUnavailable = 2,
    InvalidDynamics = 3,
    SafeControl = 4,
};

struct TraceFireCodeSnapshot {
    std::uint8_t FieldMask{0};
    std::uint8_t Raw{0};
    std::uint8_t FireStatus{0};
    std::uint8_t CapState{0};
    bool FollowMode{false};
    bool AimMode{false};
    std::uint8_t Rotate{0};
};

struct GimbalFeedbackTraceSnapshot {
    bool Available{false};
    TraceFireCodeSnapshot FireCode{};
    std::chrono::steady_clock::time_point ReceivedAt{};
};

struct ControlOutputAnglesTraceSnapshot {
    bool Published{false};
    float Yaw{0.0F};
    float Pitch{0.0F};
};

struct ControlOutputFireCodeTraceSnapshot {
    bool Published{false};
    TraceFireCodeSnapshot FireCode{};
};

struct ControlOutputTrajectoryTraceSnapshot {
    bool Published{false};
    bool Available{false};
    ControlTrajectoryUnavailableReason UnavailableReason{
        ControlTrajectoryUnavailableReason::NotRecorded};
    float Yaw{0.0F};
    float Pitch{0.0F};
    float YawOmega{0.0F};
    float PitchOmega{0.0F};
    float YawAlpha{0.0F};
    float PitchAlpha{0.0F};
};

struct ControlOutputTraceSnapshot {
    bool Available{false};
    std::uint64_t Sequence{0};
    std::chrono::steady_clock::time_point PublishedAt{};
    ControlOutputSnapshotSource Source{ControlOutputSnapshotSource::Normal};
    ControlOutputAnglesTraceSnapshot Angles{};
    ControlOutputFireCodeTraceSnapshot FireCode{};
    ControlOutputTrajectoryTraceSnapshot Trajectory{};
};

inline TraceFireCodeSnapshot MakeTraceFireCodeSnapshot(
    const gimbal_driver::msg::FireCode& message) noexcept {
    return TraceFireCodeSnapshot{
        .FieldMask = message.field_mask,
        .Raw = message.raw,
        .FireStatus = message.fire_status,
        .CapState = message.cap_state,
        .FollowMode = message.follow_mode,
        .AimMode = message.aim_mode,
        .Rotate = message.rotate,
    };
}

inline ControlOutputTraceSnapshot MakeControlOutputTraceSnapshot(
    const std::uint64_t sequence,
    const std::chrono::steady_clock::time_point published_at,
    const ControlOutputSnapshotSource source,
    const gimbal_driver::msg::GimbalAngles& angles,
    const bool angles_published,
    const gimbal_driver::msg::FireCode& fire_code,
    const bool fire_code_published,
    const std::optional<gimbal_driver::msg::GimbalTrajectory>& trajectory,
    const bool trajectory_published,
    const ControlTrajectoryUnavailableReason trajectory_unavailable_reason) noexcept {
    ControlOutputTraceSnapshot snapshot;
    snapshot.Available = angles_published || fire_code_published || trajectory_published;
    snapshot.Sequence = sequence;
    snapshot.PublishedAt = published_at;
    snapshot.Source = source;
    snapshot.Angles.Published = angles_published;
    if (angles_published) {
        snapshot.Angles.Yaw = angles.yaw;
        snapshot.Angles.Pitch = angles.pitch;
    }
    snapshot.FireCode.Published = fire_code_published;
    if (fire_code_published) {
        snapshot.FireCode.FireCode = MakeTraceFireCodeSnapshot(fire_code);
    }
    snapshot.Trajectory.Published = trajectory_published;
    snapshot.Trajectory.Available = trajectory_published && trajectory.has_value();
    snapshot.Trajectory.UnavailableReason = snapshot.Trajectory.Available
        ? ControlTrajectoryUnavailableReason::None
        : trajectory_unavailable_reason;
    if (snapshot.Trajectory.Available) {
        snapshot.Trajectory.Yaw = trajectory->yaw;
        snapshot.Trajectory.Pitch = trajectory->pitch;
        snapshot.Trajectory.YawOmega = trajectory->yaw_omega;
        snapshot.Trajectory.PitchOmega = trajectory->pitch_omega;
        snapshot.Trajectory.YawAlpha = trajectory->yaw_alpha;
        snapshot.Trajectory.PitchAlpha = trajectory->pitch_alpha;
    }
    return snapshot;
}

enum class RegionalDefenseSearchKind : std::uint8_t {
    None = 0,
    OwnBase = 1,
    OwnHighland = 2,
    OwnRoadCorridor = 3,
    OwnHighlandRoadCorridor = 4,
    CommonCentral = 5,
    EnemySideSoft = 6,
    OwnFortressGainPoint = 7
};

enum class RegionalDefenseSelection : std::uint8_t {
    ProtectCastle = 0,
    CommonCentral = 1,
};

inline constexpr const char* RegionalDefenseSearchKindToString(
    const RegionalDefenseSearchKind kind) noexcept {
    switch (kind) {
        case RegionalDefenseSearchKind::OwnBase: return "own_base";
        case RegionalDefenseSearchKind::OwnHighland: return "own_highland";
        case RegionalDefenseSearchKind::OwnRoadCorridor: return "own_road_corridor";
        case RegionalDefenseSearchKind::OwnHighlandRoadCorridor: return "own_highland_road_corridor";
        case RegionalDefenseSearchKind::CommonCentral: return "common_central";
        case RegionalDefenseSearchKind::EnemySideSoft: return "enemy_side_soft";
        case RegionalDefenseSearchKind::OwnFortressGainPoint: return "own_fortress_gain_point";
        default: return "none";
    }
}

    #define SET_POSITION(area, team) \
    do { \
        naviCommandGoal = LangYa::area(team); \
        naviGoalPosition = BehaviorTree::Area::area(team); \
        naviGoalPublishAllowed_ = true; \
    } while(0)


class Application {
    friend class StrategyManager;
public:
    inline static constexpr const char nodeName[] = "behavior_tree";
    
    // [修改] 改為 std::string，在 .cpp 構造函數中賦值，解決相對路徑問題
    std::string behavior_tree_file_;
    std::string config_file_;

private:
    // [ROS 2] 節點指針
    std::shared_ptr<rclcpp::Node> node_;

    int buff_shoot_count = 0; // 打符的击打次数

    /// 决策需要的数据
    UnitTeam team{UnitTeam::Red}; // 当前队伍颜色
    std::uint16_t enemyOutpostHealth{0}; // 敌方前哨站血量
    std::uint16_t selfOutpostHealth{0}; // 我方前哨站血量
    bool hasReceivedEnemyOutpostHealth_{false};
    std::chrono::steady_clock::time_point lastEnemyOutpostHealthRxTime_{};
    bool hasReceivedSelfOutpostHealth_{false};
    std::chrono::steady_clock::time_point lastSelfOutpostHealthRxTime_{};
    std::chrono::steady_clock::time_point lastObservedSelfOutpostHealthRxTime_{};
    ProtectOutpostState protectOutpostState_{};
    std::uint16_t enemyBaseHealth{0};  // 基地血量
    std::uint16_t selfBaseHealth{0};
    bool hasReceivedSelfBaseHealth_{false};
    std::chrono::steady_clock::time_point lastSelfBaseHealthRxTime_{};
    BaseDamageWindowState selfBaseDamageWindow_{};
    std::uint16_t ammoLeft{0}; // 剩余子弹数
    std::uint16_t timeLeft{0}; // 比赛剩余时间
    std::uint16_t myselfHealth{0}; // 自己的血量
    bool hasReceivedMyselfHealth_{false};
    bool hasReceivedAmmoLeft_{false};
    bool hasReceivedGameStartFlag_{false};
    std::chrono::steady_clock::time_point lastMyselfHealthRxTime{};
    std::chrono::steady_clock::time_point lastAmmoLeftRxTime{};
    std::chrono::steady_clock::time_point lastGameStartRxTime{};
    
    Robots friendRobots; // 己方机器人的信息
    Robots enemyRobots; // 敌方机器人的信息
    bool hasReceivedEnemyHealth_{false};
    std::array<std::chrono::steady_clock::time_point, 10> lastEnemyHealthRxTime_{};
    std::array<std::chrono::steady_clock::time_point, 10> lastFriendHealthRxTime_{};
    std::array<bool, 10> enemyZeroHealthObserved_{};
    std::array<std::chrono::steady_clock::time_point, 10> enemyZeroHealthSince_{};
    std::array<bool, 10> enemyHealthConfirmedDead_{};
    std::array<std::chrono::steady_clock::time_point, 10> lastEnemyConfirmedDeadTime_{};
    std::chrono::steady_clock::time_point lastAimTargetSelectTime_{};
    std::chrono::steady_clock::time_point lastAimTargetCandidateSeenTime_{};
    BuffType teamBuff{0}; // 当前的增益情况
    std::uint32_t rfidStatus{0}; // 0x0209 rfid_status（低32位）
    bool hasRfidStatus2{false}; // 0x0209 rfid_status_2 是否已由下位机提供
    std::uint8_t rfidStatus2{0}; // 0x0209 rfid_status_2 预留扩展字节
    RfidMatchState rfidMatchState{};
    bool hasReceivedRfidStatus_{false};
    std::chrono::steady_clock::time_point lastRfidStatusRxTime_{};
    bool selfFortressRfidActive_{false};
    gimbal_driver::msg::BulletInfo bulletInfo{};
    bool hasReceivedBulletInfo_{false};
    std::chrono::steady_clock::time_point lastBulletInfoRxTime_{};
    gimbal_driver::msg::MapCommand mapCommand{};
    bool hasReceivedMapCommand_{false};
    std::chrono::steady_clock::time_point lastMapCommandRxTime_{};
    std::uint64_t mapCommandRxSequence_{0};
    std::uint64_t handledMapCommandRxSequence_{0};
    std::uint32_t extEventData{};
    bool hasReceivedEventData_{false};
    std::uint8_t eventCenterGainPointStatus_{0};
    std::uint8_t eventSelfSmallEnergyStatus_{0};
    std::uint8_t eventSelfLargeEnergyStatus_{0};
    std::uint8_t eventSelfFortressGainPointStatus_{0};
    std::uint8_t eventSelfOutpostGainPointStatus_{0};
    bool eventSelfBaseGainPointStatus_{false};
    std::chrono::steady_clock::time_point lastEventDataRxTime_{};
    CastleCaptureAttributionState castleCaptureAttribution_{};
    mutable CastleOccupancyResolution lastCastleOccupancy_{};
    mutable std::chrono::steady_clock::time_point castleReachedGraceUntil_{};
    bool sentryCanActivateEnergyMechanism_{false};
    bool hasReceivedSentryInfo_{false};
    std::chrono::steady_clock::time_point lastSentryInfoRxTime_{};
    PostureRefereeTimer postureRefereeTimer_{};
    std::array<ArmorData, 10> armorList; // 外部 /ly/aim/armor_targets 目标序列
    bool is_game_begin{false}; // 比赛开始的标志
    FireCodeType RecFireCode{}; // 云台的火控数据
    std::uint8_t postureState{0}; // 云台/下位机回传姿态: 0=未知, 1=进攻, 2=防御, 3=移动
    bool hasReceivedPostureState_{false};
    std::chrono::steady_clock::time_point lastPostureStateRxTime_{};
    std::uint8_t capV{0};
    std::uint8_t naviLowerHead{0};


    /// 决策修改控制数据需要的前置数据
    std::chrono::steady_clock::time_point gameStartTime{std::chrono::steady_clock::now()};
    std::chrono::steady_clock::time_point lastFoundEnemyTime{std::chrono::steady_clock::now()}; 
    GimbalAnglesType gimbalAngles{0, 0}; 
    std::int16_t gimbalYawVelRaw{0};   // from /ly/gimbal/chassis.angular_velocity * 100
    float gimbalYawVelDegPerSec{0.0f};
    std::int16_t gimbalYawAngleRaw{0}; // from /ly/gimbal/chassis.steer_angle * 100
    float gimbalYawAngleDeg{0.0f};
    int patrolScanDirection_{1}; // +1 向右, -1 向左（mode=2 使用）
    float patrolScanCenterYaw_{0.0f};
    float patrolScanOffsetYaw_{0.0f};
    float patrolScanPhaseRad_{0.0f};
    bool patrolScanCenterInitialized_{false};
    int patrolScanActiveMode_{0}; // 0=无, 1/2=当前扫描模式
    bool patrolScanResumeSmoothing_{false};
    std::chrono::steady_clock::time_point patrolScanResumeStartedAt_{};
    PassiveGimbalMotion passiveGimbalMotion_{};
    std::atomic<bool> hasReceivedGimbalAngles_{false};
    std::chrono::steady_clock::time_point lastGimbalAnglesRxTime{};
    std::vector<UnitType> reliableEnemyPosuition; 
    std::vector<UnitType> hitableTargets; 

    // ==========================================
    // [恢復] 核心決策控制變數
    // ==========================================
    /// 决策的控制数据: 云台，火控, 导航, 击打目标
    AimMode aimMode{AimMode::RotateScan};
    // ArmorType targetArmor{ArmorType::Hero}; // 目标装甲板
    ArmorData targetArmor{}; // 目标装甲板，包括距离
    AimData externalAimData{}; // 外部 sentry_msgs/AimResult follow/角度/开火门控
    AimData faceModeData{}; // 接收 FaceMode 解算出来的固定点朝向角
    struct FaceModeSolverStatusState {
        bool Received{false};
        bool Function{false};
        bool ManualTarget{false};
        std::uint32_t TargetUpdateCount{0};
        std::chrono::steady_clock::time_point LastRx{};
    } faceModeSolverStatus{};
    GimbalControlData gimbalControlData{}; /// 发送给云台的角度控制数据，火控数据等
    std::uint8_t postureCommand{0}; // 姿态控制指令: 0=不下发, 1=进攻, 2=防御, 3=移动
    std::atomic<bool> isFindTargetAtomic{false}; // 在回调函数中，每接收一次消息就会被置为true，然后在发送完控制数据之后置为false
    std::chrono::steady_clock::time_point lastTargetSeenTime{}; // 最近一次收到目标回调
    std::chrono::steady_clock::time_point lastDamageTime{};     // 最近一次检测到掉血
    struct DamageSample {
        std::chrono::steady_clock::time_point Time{};
        std::uint16_t Delta{0};
    };
    std::deque<DamageSample> postureRecentDamageSamples_{};
    std::chrono::steady_clock::time_point lastDamageBurstTime_{};
    bool postureHealthInitialized_{false};
    std::uint16_t postureLastHealth_{0};
    SentryPosture postureLastDesired_{SentryPosture::Unknown};
    std::string postureLastReason_{"init"};
    TaskPostureIntentState postureTaskIntent_{};
    bool energyActivateConfirmPulseActive_{false};
    std::chrono::steady_clock::time_point energyActivateConfirmPulseUntil_{};
    std::chrono::steady_clock::time_point nextEnergyActivateConfirmTime_{};
    std::chrono::steady_clock::time_point lastEnergyActivateConfirmTime_{};
    bool buffTaskLocked_{false};
    std::chrono::steady_clock::time_point buffTaskStartTime_{};
    std::chrono::steady_clock::time_point buffTaskDamageAbortUntil_{};
    std::chrono::steady_clock::time_point outpostTaskDamageAbortUntil_{};
    std::chrono::steady_clock::time_point outpostVisualScoutStartTime_{};
    std::chrono::steady_clock::time_point outpostVisualScoutCooldownUntil_{};
    std::chrono::steady_clock::time_point outpostPostArmorFaceSearchUntil_{};
    bool outpostVisualScoutNavigationActive_{false};
    bool outpostArmorInterruptActive_{false};
    std::array<ExternalAimTargetCache, 9> externalAimTargets_{};
    bool hasExternalAimTargets_{false};
    std::chrono::steady_clock::time_point lastExternalAimTargetsRxTime_{};
    std::chrono::steady_clock::time_point lastExternalAimResultRxTime_{};

    std::uint8_t naviCommandGoal{0}; // 导航目标
    Area::Point<std::uint16_t> naviGoalPosition{}; // 导航定位目标
    bool naviGoalPublishAllowed_{true};
    MapCommandTask mapCommandTask_{};
    std::optional<MapCommandRawGoal> activeMapCommandGoal_{};
    bool mapCommandGoalPublishPending_{false};
    std::uint8_t lastNaviComnamdGoal{0}; // 上一次导航目标
    VelocityType naviVelocityInput{0, 0}; /// 外部导航输入速度（/ly/navi/vel）
    VelocityType naviVelocity{0, 0}; /// 定义回调，接收导航的速度控制数据
    std::chrono::steady_clock::time_point lastNaviVelocityRxTime_{};
    bool naviReach{false}; // /ly/navi/reached: 当前导航目标是否已到达
    bool naviReachable{true}; // /ly/navi/reachable: 当前导航目标是否有有效路径
    bool naviIsRotate{true}; // /ly/navi/should_rotate: 外部导航是否允许正常小陀螺
    bool hasReceivedNaviReach_{false};
    bool hasReceivedNaviReachable_{false};
    bool hasReceivedNaviIsRotate_{false};
    bool naviExternalStatusGoalInitialized_{false};
    std::chrono::steady_clock::time_point lastNaviReachRxTime_{};
    std::chrono::steady_clock::time_point lastNaviReachableRxTime_{};
    std::chrono::steady_clock::time_point lastNaviIsRotateRxTime_{};
    std::chrono::steady_clock::time_point naviExternalStatusGoalStartTime_{};
    rclcpp::Time naviExternalStatusGoalStartRosTime_{};
    std::uint8_t naviExternalStatusGoalId_{0};
    Area::Point<std::uint16_t> naviExternalStatusGoalPosition_{};
    mutable NearGoalArrivalConfirm nearGoalArrivalConfirm_{};
    bool regionalRecoveryProbeActive_{false};
    std::size_t regionalRecoveryProbeIndex_{0};
    bool regionalRecoveryMonitorActive_{false};
    Area::Point<std::uint16_t> regionalRecoveryMonitorGoal_{};
    std::chrono::steady_clock::time_point regionalRecoveryMonitorStartTime_{};
    std::uint16_t regionalRecoveryMonitorHealth_{0};
    std::uint16_t regionalRecoveryMonitorAmmo_{0};
    bool specialPatrolHoldActive_{false};
    std::uint8_t specialPatrolHoldBaseGoal_{LangYa::CentralLeftA.ID};
    std::chrono::steady_clock::time_point specialPatrolHoldStartTime_{};
    bool naviRelativeTargetValid{false};
    float naviRelativeTargetX{0.0F};
    float naviRelativeTargetY{0.0F};
    float naviRelativeTargetZ{0.0F};
    float naviRelativeTargetDistance{0.0F};
    float naviRelativeTargetYawErrorDeg{0.0F};
    float naviRelativeTargetPitchErrorDeg{0.0F};
    std::uint8_t naviRelativeTargetArmorType{0U};
    std::uint8_t naviRelativeTargetAimMode{0U};
    std::string naviRelativeTargetFrameId{};
    bool naviChaseOfficialTargetValid{false};
    std::uint8_t naviChaseOfficialTargetArmorType{0U};
    bool chaseTacticalAllowed_{false};
    bool naviChaseVelocityActive_{false};
    VelocityType naviChaseVelocity{0, 0};
    std::chrono::steady_clock::time_point lastOfficialChaseAreaLimitLogTime_{};
    TimerClock naviCommandIntervalClock{Seconds{10}}; // 控制间隔
    std::uint8_t speedLevel{1}; // 0 没电, 1 正常, 2 快速
    StrategyMode strategyMode_{StrategyMode::Regional}; // 当前策略
    CompetitionProfile competitionProfile_{CompetitionProfile::Regional};
    std::string competitionProfileOverride_{};
    bool debugBypassGameStart_{false};
    bool runtimeRearmStartGate_{false};
    bool publishNaviGoal_{true};
    int waitForGameStartTimeoutSec_{0};
    int leagueRefereeStaleTimeoutMs_{0};
    bool runtimeStartGateActive_{false};
    std::chrono::steady_clock::time_point runtimeStartGateLastLogTime_{};
    bool leagueRecoveryActive_{false};
    std::chrono::steady_clock::time_point leagueRecoveryStartTime_{};
    std::chrono::steady_clock::time_point leagueRecoveryReach350Time_{};
    std::chrono::steady_clock::time_point leagueRecoveryLastIncreaseTime_{};
    std::chrono::steady_clock::time_point leagueRecoveryCooldownUntil_{};
    std::uint16_t leagueRecoveryEntryHealth_{0};
    std::uint16_t leagueRecoveryPeakHealth_{0};
    League3v3Policy league3v3Policy_{};
    League3v3Decision league3v3Decision_{};
    bool league3v3PolicyConfigured_{false};
    std::size_t leaguePatrolGoalIndex_{0};
    bool leaguePatrolGoalInitialized_{false};
    std::size_t showcasePatrolGoalIndex_{0};
    bool showcasePatrolGoalInitialized_{false};
    std::size_t naviDebugGoalIndex_{0};
    bool naviDebugGoalInitialized_{false};
    bool leagueRouteCompatAfterGatePending_{false};
    bool leagueRouteCompatActive_{false};
    std::chrono::steady_clock::time_point leagueRouteCompatUntil_{};
    bool leagueRouteCompatHasPendingGoal_{false};
    std::uint8_t leagueRouteCompatPendingBaseGoal_{LangYa::Home.ID};
    int leagueRouteCompatPendingHoldSec_{1};
    std::chrono::steady_clock::time_point lastLeagueRecoveryGuardLogTime_{};
    std::chrono::steady_clock::time_point lastPositionDataGuardLogTime_{};
    struct SentryPositionSourceCache {
        bool Valid{false};
        int X{0};
        int Y{0};
        std::chrono::steady_clock::time_point LastRx{};
        rclcpp::Time Stamp{};
    };
    struct UnitInfoStampCache {
        rclcpp::Time Stamp{};
    };
    struct UnitPositionState {
        bool HasPosition{false};
        bool Fresh{false};
        int X{0};
        int Y{0};
        std::string Source{"none"};
        rclcpp::Time Stamp{};
        std::int64_t AgeMs{-1};
    };
    SentryPositionSourceCache sentryUwbPositionSource_{};
    SentryPositionSourceCache sentryPositionDataSource_{};
    SentryPositionSourceCache sentryNaviPositionSource_{};
    bool hasReceivedSentryPosition_{false};
    std::chrono::steady_clock::time_point lastSentryPositionRxTime_{};
    std::chrono::steady_clock::time_point lastSentryRadarPositionRxTime_{};
    std::chrono::steady_clock::time_point lastSentryPositionFusionLogTime_{};
    std::string sentryPositionFusionSource_{"none"};
    std::array<std::chrono::steady_clock::time_point, 10> lastEnemyPositionRxTime_{};
    std::array<std::chrono::steady_clock::time_point, 10> lastFriendPositionRxTime_{};
    std::array<UnitInfoStampCache, 10> lastEnemyPositionStamp_{};
    std::array<UnitInfoStampCache, 10> lastFriendPositionStamp_{};
    std::array<UnitInfoStampCache, 10> lastEnemyHealthStamp_{};
    std::array<UnitInfoStampCache, 10> lastFriendHealthStamp_{};
    std::array<std::string, 10> lastEnemyPositionSource_{};

    // ==========================================
    // Runtime Guard (L1/L2)
    // ==========================================
    std::thread runtimeGuardThread_{};
    std::atomic<bool> runtimeGuardStop_{false};
    std::atomic<bool> runtimeRecoveryRequested_{false};
    std::atomic<RuntimeFaultCode> runtimeFaultCode_{RuntimeFaultCode::None};
    std::atomic<bool> runtimeTickInProgress_{false};
    std::atomic<std::int64_t> runtimeLastLoopBeatNs_{0};
    std::atomic<std::int64_t> runtimeTickStartNs_{0};
    std::atomic<std::int64_t> runtimeTickEndNs_{0};
    std::atomic<std::int64_t> runtimeLastSafePublishNs_{0};
    std::mutex runtimeRecoveryMutex_{};
    bool runtimeRecovering_{false};
    std::chrono::steady_clock::time_point runtimeRecoveryWindowStart_{};
    std::chrono::steady_clock::time_point runtimeLastSoftRecoverTime_{};
    int runtimeRecoveryCountInWindow_{0};
    static constexpr int kRuntimeLoopStallMs = 1500;
    static constexpr int kRuntimeTickStallMs = 800;
    static constexpr int kRuntimeGimbalStaleMs = 1200;
    static constexpr int kRuntimeRecoveryWindowSec = 60;
    static constexpr int kRuntimeRecoveryLimit = 3;
    static constexpr int kRuntimeRecoveryMinIntervalMs = 1500;
    static constexpr int kRuntimeSafePublishMinIntervalMs = 200;
    static constexpr int kNaviExternalStatusTimeoutMs = 2000;

    BT::Blackboard::Ptr GlobalBlackboard_ = BT::Blackboard::create(); // 跨 tick 持久黑板
    BT::Blackboard::Ptr TickBlackboard_ = BT::Blackboard::create();   // 每次 tick 中间黑板
    BT::BehaviorTreeFactory Factory{}; // 行为树工厂
    BT::Tree BTree{}; // 行为树
    std::unique_ptr<BT::StdCoutLogger> btCoutLogger_; // bt_cout_logger
    std::unique_ptr<BT::FileLogger2> btFileLogger_; // bt_file_logger_v2
    std::unique_ptr<BT::MinitraceLogger> btMinitraceLogger_; // bt_minitrace_logger
    std::unique_ptr<BT::Groot2Publisher> btGrootPublisher_; // Groot2
    // ==========================================

    std::shared_ptr<Logger> LoggerPtr; // 日志

    Config config{}; // 配置文件
    AreaManager areaManager_{};
    DefaultStrategyManager defaultStrategyManager_{};
    DecisionIntent lastDecisionIntent_{};
    bool decisionConfigurationLogged_{false};
    std::optional<DecisionExplain::Fingerprint> lastDecisionExplainFingerprint_{};
    EventManager eventManager_{};
    EventSnapshot eventSnapshot_{};
    PostureManager postureManager_{};
    OutpostEngagementLock outpostEngagementLock_{};
    OutpostEngagementDecision outpostEngagementDecision_{};
    StrategyManager strategyManager_{};
    RegionalDefenseSearchKind regionalDefenseSearchKind_{RegionalDefenseSearchKind::None};
    std::size_t regionalDefenseSearchIndex_{0};
    std::uint8_t regionalDefenseSearchBaseGoal_{LangYa::Home.ID};
    std::chrono::steady_clock::time_point regionalDefenseSearchStartTime_{};
    std::chrono::steady_clock::time_point regionalDefenseSearchArrivedTime_{};
    int regionalDefenseSearchDirection_{1};
    int fortressGainPointEnemyCount_{0};
    std::chrono::steady_clock::time_point fortressGainPointNoContactSince_{};
    std::chrono::steady_clock::time_point fortressGainPointDegradedUntil_{};
    bool protectHeroActive_{false};
    std::chrono::steady_clock::time_point protectHeroLastEnemySeenTime_{};
    bool protectHeroEnhancedDefenseUnavailable_{false};
    bool protectHeroEnhancedDefenseRecoveryDeferred_{false};
    bool enhancedRecoveryMoveUnavailable_{false};
    bool enhancedMoveHealthInitialized_{false};
    std::uint16_t enhancedMoveLastHealth_{0};
    std::chrono::steady_clock::time_point enhancedMoveRespawnSuppressUntil_{};
    FaceModeManager faceModeManager_{};
    FaceModeManager::Decision lastFaceModeDecision_{};
    std::chrono::steady_clock::time_point lastUpdateBlackboardLogTime_{};
    std::chrono::steady_clock::time_point lastTreeTickLogTime_{};
    std::chrono::steady_clock::time_point lastTransportLogTime_{};
    std::ofstream decisionTraceStream_{};
    std::string decisionTraceFile_{};
    bool decisionTraceRequested_{false};
    bool decisionTraceEnabled_{false};
    int decisionTraceEveryTicks_{5};
    std::uint64_t decisionTraceTickCount_{0};
    std::uint64_t decisionTraceWriteCount_{0};
    std::mutex decisionTraceSnapshotMutex_{};
    GimbalFeedbackTraceSnapshot gimbalFeedbackTraceSnapshot_{};
    ControlOutputTraceSnapshot controlOutputTraceSnapshot_{};
    std::uint64_t controlOutputTraceSequence_{0};
    void CaptureGimbalFeedbackTraceSnapshot(
        const gimbal_driver::msg::FireCode& message,
        std::chrono::steady_clock::time_point received_at) noexcept;
    void CaptureControlOutputTraceSnapshot(
        const gimbal_driver::msg::GimbalAngles& angles,
        bool angles_published,
        const gimbal_driver::msg::FireCode& fire_code,
        bool fire_code_published,
        const std::optional<gimbal_driver::msg::GimbalTrajectory>& trajectory,
        bool trajectory_published,
        ControlTrajectoryUnavailableReason trajectory_unavailable_reason,
        ControlOutputSnapshotSource source,
        std::chrono::steady_clock::time_point published_at) noexcept;

    RateClock fireRateClock{20}, treeTickRateClock{100}, naviCommandRateClock{2}; // 频率控制
    TimerClock rotateTimerClock{Seconds{2}}; // 旋转时间
    DescentDetector<std::uint16_t>  healthDecreaseDetector{400}; // 血量丢失检测器


    // ==========================================
    // [ROS 2] 通訊管理
    // ==========================================
    // 必須保存指針以防斷連
    std::vector<std::shared_ptr<void>> subscribers_;
    std::vector<std::shared_ptr<void>> publishers_;

    // [保留] 回调管理 (為了接口兼容性保留)
    MultiCallback<Application&> callbacks{*this};

    // [ROS 2] 訂閱生成器 — 接受兩參數 lambda: [](Application& app, MsgSharedPtr msg){}
    template<typename TTopic>
    void GenSub(std::function<void(Application&, typename TTopic::CallbackArg)> callback) {
        GenSubWithQoS<TTopic>(rclcpp::QoS(10), std::move(callback));
    }

    template<typename TTopic>
    void GenSubWithQoS(
        const rclcpp::QoS& qos,
        std::function<void(Application&, typename TTopic::CallbackArg)> callback) {
        using MsgType = typename TTopic::Msg;
        std::string topic_name = TTopic::Name;

        auto sub = node_->create_subscription<MsgType>(
            topic_name,
            qos,
            [this, callback](const typename MsgType::SharedPtr msg) {
                callback(*this, msg);
            }
        );
        subscribers_.push_back(sub);
    }
    
    // [新增] 發布生成器
    template<typename TTopic>
    auto GenPub() {
        using MsgType = typename TTopic::Type;
        auto pub = node_->create_publisher<MsgType>(TTopic::Name, rclcpp::QoS(10));
        publishers_.push_back(pub); 
        return pub;
    }

    // [ROS 2] 發布者指針 (明確類型)
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_vision_mode_;

    rclcpp::Publisher<gimbal_driver::msg::GimbalAngles>::SharedPtr pub_gimbal_control_;
    rclcpp::Publisher<gimbal_driver::msg::GimbalTrajectory>::SharedPtr pub_gimbal_trajectory_;
    rclcpp::Publisher<gimbal_driver::msg::FireCode>::SharedPtr pub_gimbal_firecode_;
    rclcpp::Publisher<gimbal_driver::msg::SentryCmd>::SharedPtr pub_control_posture_;
    rclcpp::Publisher<gimbal_driver::msg::SentryCmd>::SharedPtr pub_control_sentry_cmd_;
    rclcpp::Publisher<gimbal_driver::msg::ControlVelocity>::SharedPtr pub_gimbal_vel_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_gimbal_capV_;

#ifdef LY_ENABLE_SENTRY_MSGS
    rclcpp::Publisher<sentry_msgs::msg::AimTarget>::SharedPtr pub_external_aim_select_target_;
#endif
    
    rclcpp::Publisher<gimbal_driver::msg::Vel>::SharedPtr pub_navi_vel_;
    rclcpp::Publisher<auto_aim_common::msg::RelativeTarget>::SharedPtr pub_navi_target_rel_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_navi_goal_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_navi_goal_pos_raw_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_navi_goal_pos_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_navi_goal_pose_;
    rclcpp::Publisher<auto_aim_common::msg::GoalReach>::SharedPtr pub_navi_reach_state_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_face_mode_target_raw_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_navi_speed_level_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_navi_lower_head_;
    
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_bt_target_;
    rclcpp::Publisher<gimbal_driver::msg::UnitInfoArray>::SharedPtr pub_friend_info_;
    rclcpp::Publisher<gimbal_driver::msg::UnitInfoArray>::SharedPtr pub_enemy_info_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_sentry_position_;

    void RecordDamageSample(std::chrono::steady_clock::time_point now, std::uint16_t damage);
    DecisionIntent MakeDecisionIntent(
        DecisionReason reason,
        std::uint8_t base_goal_id,
        UnitTeam goal_team,
        bool apply_team_offset,
        const char* detail = nullptr) const;
    void RecordDecisionIntent(DecisionIntent intent);
    void LogDecisionConfigurationOnce();
    void MaybeLogNavigationDecision();
    void MaybeLogRelativeTargetDecision();
    void MaybeLogManualOutpostGoalPoseDecision(double x_meter, double y_meter, double z_meter);
    void LogNavigationDecisionIfChanged(
        const DecisionExplain::NavigationObservation& observation);
    void UpdateSentryPositionFusion(std::chrono::steady_clock::time_point now);
    UnitPositionState GetSentryPositionState(std::chrono::steady_clock::time_point now) const;
    UnitPositionState GetSentryPositionState(std::chrono::steady_clock::time_point now, int fresh_ms) const;
    UnitPositionState GetFriendPositionState(
        UnitType unit_type,
        int fresh_ms,
        std::chrono::steady_clock::time_point now) const;
    UnitPositionState GetEnemyPositionState(
        UnitType unit_type,
        int fresh_ms,
        std::chrono::steady_clock::time_point now) const;
    bool IsSentryPositionFresh(std::chrono::steady_clock::time_point now) const;
    gimbal_driver::msg::UnitInfoArray MakeFriendInfoMsg();
    gimbal_driver::msg::UnitInfoArray MakeEnemyInfoMsg();


public:
    void SubscribeMessageAll();
    void PrintMessageAll();

    // 发布消息
    void PublishMessageAll();
    void PubAimModeEnableData();
    void PubGimbalControlData();
    void PubPostureControlData();
    void PubEnergyActivateConfirmData(bool confirm);
    void UpdateEnergyActivateConfirmCommand(bool should_confirm);
    void PubAimTargetData();
    void PubExternalAimTargetData();
    void PubNaviControlData();
    void PubNaviRelativeTarget();
    void PubNaviGoal();
    void PubNaviGoalPos();
    void PubMapCommandGoalPos();
    void PubNaviSpeedLevel(std::uint8_t requested_level);
    void PubNaviReachState();
    bool PubManualOutpostGoalPose(const char* reason);
    void PubFriendInfo();
    void PubEnemyInfo();
    void PubSentryPosition();


    // 等待比赛开始
    void WaitForGameStart();
    void WaitBeforeGame();

    //  比赛循环
    void GameLoop();

    /**
     * @brief 从黑板获取数据 \n
     */
    template<typename T>
    T GetInfoFromBlackBoard(const std::string &key) {
        T value{};
        bool ValueExist = GlobalBlackboard_->get<T>(key, value);
        if (!ValueExist) {
            if(LoggerPtr) LoggerPtr->Error("Blackboard key {} not found", key.c_str());
            else RCLCPP_ERROR(node_->get_logger(), "Blackboard key %s not found", key.c_str());
        }
        return value;
    }
    void UpdateBlackBoard();
    void UpdateEventSnapshot();
    void TransportData();
    void PublishTogether();
    void TreeTick();
    void TreeTickGuarded();
    void ProcessData();
    bool CheckPositionRecovery();
    void SetPositionRepeat();
    bool StrategyLayerHandled() const noexcept { return strategyManager_.Handled(); }
    void ResetChaseTacticalAuthorization() noexcept {
        chaseTacticalAllowed_ = false;
        naviChaseVelocityActive_ = false;
        naviChaseVelocity = VelocityType{0, 0};
        naviRelativeTargetValid = false;
        naviRelativeTargetX = 0.0F;
        naviRelativeTargetY = 0.0F;
        naviRelativeTargetZ = 0.0F;
        naviRelativeTargetDistance = 0.0F;
        naviRelativeTargetYawErrorDeg = 0.0F;
        naviRelativeTargetPitchErrorDeg = 0.0F;
        naviRelativeTargetArmorType = 0U;
        naviRelativeTargetAimMode = static_cast<std::uint8_t>(aimMode);
        naviRelativeTargetFrameId.clear();
        naviChaseOfficialTargetValid = false;
        naviChaseOfficialTargetArmorType = 0U;
    }
    void SetChaseTacticalAllowed(bool allowed) noexcept { chaseTacticalAllowed_ = allowed; }
    bool IsChaseTacticalAllowed() const noexcept { return chaseTacticalAllowed_; }
    bool CanAuthorizeChaseTactical() const noexcept;
    bool TryApplyChaseTactical(
        std::optional<AreaKey> explicit_allowed_area = std::nullopt,
        const char* decision_detail = "chase");
    bool ShouldSuppressChaseForSpecialPatrol() const noexcept;
    bool RunStrategyLayerHard();
    bool RunStrategyLayerDefault();
    bool RunStrategyLayerTask();
    bool RunStrategyLayerTactical();
    bool RunStrategyLayerSpecial();
    bool RunStrategyLayerFinalizer();
    bool TrySetMapCommandGoal();
    void CancelMapCommandTask() noexcept;
    void SetPositionLeagueSimple();
    void SetPositionShowcasePatrol();
    void SetPositionNaviDebugPlan();
    bool IsLeagueRouteCompatEnabled() const noexcept;
    bool IsLeagueGoalSwitchBetween2And3(
        std::uint8_t from_goal_id,
        std::uint8_t to_goal_id,
        UnitTeam goal_team,
        bool apply_team_offset) const noexcept;
    bool TickLeagueRouteCompat(UnitTeam goal_team, bool apply_team_offset);
    void StartLeagueRouteCompat(
        std::uint8_t pending_base_goal,
        int pending_hold_sec,
        UnitTeam goal_team,
        bool apply_team_offset,
        const char* reason);
    bool IsNaviGoalAreaScopeEnabled() const noexcept;
    bool IsNaviGoalAllowedByAreaScope(
        std::uint8_t base_goal_id,
        UnitTeam goal_team,
        UnitTeam my_team,
        UnitTeam enemy_team) const;
    bool IsHighlandCompatEnabled() const noexcept;
    bool IsHighlandCompatTarget(std::uint8_t base_goal_id, UnitTeam goal_team) const;
    bool IsSelfInMainArea(UnitTeam area_team, Area::MainAreaKind kind) const;
    bool IsNaviExternalStatusFreshForGoal(
        std::chrono::steady_clock::time_point last_rx,
        std::uint8_t goal_id,
        Area::Point<std::uint16_t> goal_position) const;
    std::optional<bool> GetExternalNaviReachForGoal(
        std::uint8_t goal_id,
        Area::Point<std::uint16_t> goal_position) const;
    std::optional<bool> GetExternalNaviReachableForGoal(
        std::uint8_t goal_id,
        Area::Point<std::uint16_t> goal_position) const;
    void UpdateNaviExternalStatusGoal(
        std::uint8_t goal_id,
        Area::Point<std::uint16_t> goal_position);
    GoalReachState EvaluateNaviGoalReach(
        std::uint8_t goal_id,
        Area::Point<std::uint16_t> goal_position,
        int arrive_distance_cm,
        int face_distance_cm = 0,
        std::uint8_t base_goal_id = LangYa::Home.ID,
        int timeout_sec = 0) const;
    GoalReachState EvaluateBaseGoalReach(
        std::uint8_t base_goal_id,
        UnitTeam goal_team,
        bool apply_team_offset = true,
        int face_distance_cm = 0,
        int timeout_sec = 0) const;
    int GoalReachTimeoutSecForBaseGoal(std::uint8_t base_goal_id) const;
    bool IsNaviGoalPositionArrived(
        std::uint8_t goal_id,
        Area::Point<std::uint16_t> goal_position) const;
    bool IsBaseGoalArrived(
        std::uint8_t base_goal_id,
        UnitTeam goal_team,
        bool apply_team_offset = true) const;
    bool IsBaseGoalWithinDistance(
        std::uint8_t base_goal_id,
        UnitTeam goal_team,
        int distance_cm) const;
    bool IsBaseGoalExternallyUnreachable(
        std::uint8_t base_goal_id,
        UnitTeam goal_team,
        bool apply_team_offset = true) const;
    bool IsHighlandCompatArrived(UnitTeam goal_team) const;
    void ResetRegionalAreaControlOverride() noexcept;
    void ApplyAimModeFaceTarget(UnitTeam target_team);
    void RefreshAimModeFaceControl();
    bool TrySetAimModeTaskGoal(UnitTeam my_team, UnitTeam enemy_team, const char* reason);
    bool IsOutpostVisualScoutNavigationActive() const noexcept { return outpostVisualScoutNavigationActive_; }
    bool IsOutpostOpeningHighPriorityActive() const noexcept;
    bool ShouldSuppressChaseForOutpostTask() const noexcept;
    bool TrySetOutpostVisualScoutTravelGoal(UnitTeam my_team, UnitTeam enemy_team, const char* reason);
    void ApplyRegionalAreaTaskControl(const RegionalAreaTaskTickResult& result);
    bool RequestReadyRoadlandSafeReturn(const char* reason);
    bool TickRegionalAreaTask(UnitTeam my_team, UnitTeam enemy_team);
    bool TryStartRegionalAreaTaskForGoal(
        std::uint8_t base_goal_id,
        UnitTeam goal_team,
        UnitTeam my_team,
        bool apply_team_offset,
        const char* reason,
        RegionalAreaTaskOrigin origin);
    bool TickNaviAreaTransition();
    bool TryStartNaviAreaTransition(
        std::uint8_t base_goal_id,
        UnitTeam goal_team,
        UnitTeam my_team,
        bool apply_team_offset,
        const char* reason);
    bool IsRegionalDefenseAimSuppressActive() const noexcept;
    bool IsFortressGainPointEnemyOccupiedEventRawFresh(int referee_fresh_ms) const noexcept;
    bool IsProtectCastleRfidStayActive(int referee_fresh_ms) const noexcept;
    bool IsProtectCastleBaseDamageActive(
        std::chrono::steady_clock::time_point now) const noexcept;
    CastleOccupancyResolution ResolveProtectCastleOccupancy(
        std::chrono::steady_clock::time_point now) const;
    bool IsFortressGainPointEnemyOccupiedEventFresh(int referee_fresh_ms) const noexcept;
    League3v3Input BuildLeague3v3Input(std::chrono::steady_clock::time_point now) const;
    bool IsFriendPositionFresh(UnitType unit_type, int fresh_ms) const;
    bool IsFriendHealthFresh(UnitType unit_type, int fresh_ms) const;
    bool IsEnemyPositionFresh(UnitType unit_type, int fresh_ms) const;
    std::optional<RegionalDefenseThreat> EvaluateRegionalDefenseThreat(
        UnitTeam my_team,
        UnitTeam enemy_team) const;
    bool TrySetRegionalDefenseGoal(
        UnitTeam my_team,
        UnitTeam enemy_team,
        RegionalDefenseSelection selection = RegionalDefenseSelection::ProtectCastle);
    bool TryAdvanceRegionalDefenseCommonCentralSearch(
        const NaviProgressWatchdogDecision& decision,
        UnitTeam my_team,
        std::chrono::steady_clock::time_point now);
    void UpdateProtectOutpostState(std::chrono::steady_clock::time_point now);
    bool TrySetProtectOutpostGoal(UnitTeam my_team, UnitTeam enemy_team);
    bool TrySetProtectHeroGoal(UnitTeam my_team, UnitTeam enemy_team);
    bool TrySetSpecialPatrolGoal(UnitTeam my_team, UnitTeam enemy_team);
    bool TickNaviProgressWatchdog(UnitTeam my_team, UnitTeam enemy_team);
    bool IsDefaultRegionalDecisionReady(UnitTeam my_team, UnitTeam enemy_team) const;
    bool TrySetDefaultRegionalGoal(UnitTeam my_team, UnitTeam enemy_team);
    bool TrySetDefaultRegionalAreaTaskGoal(UnitTeam my_team, UnitTeam enemy_team);
    bool TrySetRegionalIdlePatrolGoal(UnitTeam my_team, UnitTeam enemy_team);
    const DecisionIntent& LastDecisionIntent() const noexcept { return lastDecisionIntent_; }
    void UpdateNaviProgressWatchdogGoal(
        std::uint8_t base_goal_id,
        UnitTeam goal_team,
        bool apply_team_offset);
    bool TrySetScopedPositionByBaseGoal(
        std::uint8_t base_goal_id,
        UnitTeam goal_team,
        UnitTeam my_team,
        UnitTeam enemy_team,
        bool apply_team_offset = true,
        const char* reason = nullptr,
        RegionalAreaTaskOrigin origin = RegionalAreaTaskOrigin::ScopedGoal);
    bool TrySetRandomScopedPositionByBaseGoal(
        const std::vector<std::pair<std::uint8_t, UnitTeam>>& goals,
        UnitTeam my_team,
        UnitTeam enemy_team,
        const char* reason = nullptr);
    void SetPositionByBaseGoal(std::uint8_t base_goal_id, UnitTeam team, bool apply_team_offset = true);
    std::uint8_t ResolveGoalId(std::uint8_t base_goal_id, UnitTeam team, bool apply_team_offset = true) const noexcept;
    void SetAimTarget();
    void RefreshOutpostEngagementLock();
    bool HasFreshExternalAimTarget(ArmorType armor_type) const noexcept;
    bool HasFreshSelectedExternalAimTarget() const noexcept;
    bool SelectFreshExternalAimTarget();
    void SetAimTargetNormal();
    bool TrySetAimTargetByAutonomy();
    void SetAimMode();
    void SelectStrategyMode();
    bool IsDecisionAutonomyModuleEnabled(std::string_view module) const;
    void CheckDebug();
    void UpdatePostureCommand(bool has_target);
    SentryPosture SelectDesiredPosture(bool has_target) const;
    TaskPostureIntentState ResolveTaskPostureIntent(
        std::chrono::steady_clock::time_point now) const;
    bool HasRecentTarget() const;
    AimSourceView CurrentAimSource() const noexcept;
    const AimData& CurrentAimData() const noexcept;
    bool AutoAimFreshAndValid() const noexcept;
    bool CurrentAimFreshAndValid() const noexcept;
    bool CurrentAimFreshOrLatched(std::chrono::steady_clock::time_point now, int hold_ms) const noexcept;
    bool CurrentAimTargetForAngles(
        bool callback_seen,
        std::chrono::steady_clock::time_point now,
        int hold_ms,
        bool* fresh_target = nullptr,
        bool* latched_target = nullptr) const noexcept;
    bool BuffAimTargetLocked() const noexcept;
    bool BuffAimFreshAndFireReady() const noexcept;
    bool OutpostAimFreshAndValid() const noexcept;
    bool OutpostAimFreshOrLatched(std::chrono::steady_clock::time_point now, int hold_ms) const noexcept;
    bool IsUnderFireRecent() const;
    bool IsUnderFireBurst() const;
    bool IsDamageBurst(int window_ms, int threshold) const;

    // 行为树初始化
    bool LoadBehaviorTree() noexcept;
    bool RegisterTreeNodes();

    // 初始化地图
    void InitMap();

    // 日志初始化
    bool InitLogger();

    // 获取配置文件
    bool ConfigurationInit();
    void ApplyTaskParameterOverrides();
    void ApplyChaseParameterOverrides();
    void ApplyChasePolicyParameterOverrides();
    void ApplyAreaManagerParameterOverrides();
    void ApplySpecialParameterOverrides();
    void ApplyStartGateParameterOverrides();
    void ApplyNaviControlParameterOverrides();
    void ApplyTacticalParameterOverrides();
    void ApplyPatrolScanParameterOverrides();
    void ApplyFaceModeParameterOverrides();
    void ApplyExternalAimParameterOverrides();
    void ApplyAimParameterOverrides();
    bool InitDecisionTrace();
    void WriteDecisionTrace(std::string_view event) noexcept;
    void CloseDecisionTrace();
    void StartRuntimeGuard();
    void StopRuntimeGuard();
    void RuntimeGuardLoop();
    void RequestSoftRecovery(RuntimeFaultCode code) noexcept;
    bool TryHandleSoftRecovery();
    bool TrySoftReloadBehaviorTree(RuntimeFaultCode code);
    bool IsCriticalInputStale() const;
    void PublishSafeControl(const char* reason, bool from_guard_thread = false) noexcept;
    void MarkLoopBeat() noexcept;
    void MarkTickStart() noexcept;
    void MarkTickEnd() noexcept;
    static std::int64_t NowSteadyNs() noexcept;
    static const char* RuntimeFaultCodeToString(RuntimeFaultCode code) noexcept;

    StrategyMode GetStrategyMode() const noexcept { return strategyMode_; }
    void SetStrategyMode(const StrategyMode mode) noexcept { strategyMode_ = mode; }
    CompetitionProfile GetCompetitionProfile() const noexcept { return competitionProfile_; }
    void SetCompetitionProfile(const CompetitionProfile profile) noexcept { competitionProfile_ = profile; }
    bool IsLeagueProfile() const noexcept { return competitionProfile_ == CompetitionProfile::League; }
    bool IsShowcasePatrolEnabled() const noexcept { return config.ShowcasePatrolSettings.Enable; }
    bool IsNaviDebugEnabled() const noexcept { return config.NaviDebugSettings.Enable; }
    AimMode GetAimMode() const noexcept { return aimMode; }
    BT::Blackboard::Ptr GetGlobalBlackboard() const noexcept { return GlobalBlackboard_; }
    BT::Blackboard::Ptr GetTickBlackboard() const noexcept { return TickBlackboard_; }
    void ResetTickBlackboard() {
        TickBlackboard_ = BT::Blackboard::create();
        if (GlobalBlackboard_) {
            GlobalBlackboard_->set("TickBlackboard", TickBlackboard_);
        }
    }
    int ElapsedSeconds() const {
        return static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - gameStartTime).count());
    }
    std::vector<UnitType> GetHitableTargetsCopy() const { return hitableTargets; }
    std::vector<UnitType> GetReliableEnemyPositionsCopy() const { return reliableEnemyPosuition; }
    ArmorData GetTargetArmorCopy() const { return targetArmor; }
    std::uint8_t GetPostureCommand() const noexcept { return postureCommand; }
    std::uint8_t GetPostureState() const noexcept { return postureState; }
    const PostureRuntime& GetPostureRuntime() const noexcept { return postureManager_.Runtime(); }

    Application(int argc, char **argv);
    ~Application();
    void Run();
};
}
