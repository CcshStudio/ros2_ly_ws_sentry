// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#pragma once

#include <chrono>
#include <cstdint>
#include <array>
#include <numbers>
#include <string>
#include <unordered_map>
#include <vector>

#pragma region Enums
namespace LangYa
{

#pragma region RobamasterTypes
    enum class UnitType : std::uint8_t
    {
        Unknown = 0,
        Hero = Unknown + 1,
        Engineer = Hero + 1,
        Infantry1 = Engineer + 1,
        Infantry2 = Infantry1 + 1,
        Infantry3 = Infantry2 + 1,
        Drone = Infantry3 + 1,
        Sentry = Drone + 1,
        Dart = Sentry + 1,
        Radar = Dart + 1
    };
    static std::vector<UnitType> RobotLists = {UnitType::Hero, UnitType::Engineer, UnitType::Infantry1, UnitType::Infantry2, UnitType::Sentry};

    /// @brief 团队类型
    enum class UnitTeam : std::uint8_t
    {
        Unknown = 0,
        Red = Unknown + 1,
        Blue = Red + 1,
        Other = Blue + 1
    };

    /// @brief 与串口协议中相同的单位类型数值
    enum class AllUnitType : std::uint8_t
    {
        Unknown = 0,
        RedOffset = 0,
        BlueOffset = 100,

        // sc_X macro for unit definition
#define sc_unit_list 	\
		sc_X(Hero) 		\
		sc_X(Engineer) 	\
		sc_X(Infantry1) \
		sc_X(Infantry2) \
		sc_X(Infantry3) \
		sc_X(Drone) 	\
		sc_X(Sentry) 	\
		sc_X(Radar)
#define sc_team_unit(team, type) team##type = team##Offset + static_cast<std::uint8_t>(UnitType::type),  // NOLINT(bugprone-macro-parentheses)
#define sc_X(type) sc_team_unit(Red, type)
        sc_unit_list
#undef sc_X
#define sc_X(type) sc_team_unit(Blue, type)
        sc_unit_list
#undef sc_X
#undef sc_team_unit
#undef sc_uint_list
    };

#pragma endregion RobomasterTypes

#pragma region Armor
    /// @brief 机器学习中的类型--地面作战单位
    enum class ArmorType : std::uint8_t
    {
        Base = 0,
        Hero = Base + 1,
        Engineer = Hero + 1,
        Infantry1 = Engineer + 1,
        Infantry2 = Infantry1 + 1,
        Infantry3 = Infantry2 + 1,
        Sentry = Infantry3 + 1,
        Outpost = Sentry + 1,
        UnKnown = Outpost + 1
    };

#pragma endregion Armor

#pragma region Gimbaldata
    using AngleType = float;
    using Angle100Type = std::int16_t;
    using RadianType = float;

    inline RadianType ToRadian(const AngleType angle) noexcept { return angle * std::numbers::pi_v<float> / 180.0f; }

    inline AngleType ToAngle(const RadianType radian) noexcept { return radian * 180.0f / std::numbers::pi_v<float>; }
#pragma pack(push, 1)
    struct GimbalAnglesType {
        AngleType Yaw;
        AngleType Pitch;
    };

    struct VelocityType {
        std::int8_t X;
        std::int8_t Y;
    };

    struct UWBPositionType {
        std::int16_t X{0}; // 自己的x位置，单位cm
        std::int16_t Y{0}; // 自己的y位置，单位cm
    };

    struct FireCodeType
    {
        /// @brief 开火状态，所有位反转表示开火 0b00 <-> 0b11
        std::uint8_t FireStatus : 2 = 0;

        /// @brief 电容状态， 00:不用 01:轻度使用 10:重度使用
        std::uint8_t CapState : 2 = 0;

        /// @brief 跟随模式，1 表示启用，0 表示禁用
        std::uint8_t FollowMode : 1 = 0;
        /// @brief 辅瞄模式，1 表示启用，0 表示禁用
        std::uint8_t AimMode : 1 = 0;

        /// @brief 小陀螺状态，共四档，0 表示无速，1 表示低速， 2 表示中速， 3 表示高速
        std::uint8_t Rotate : 2 = 0;

        /// @brief 翻转开火标志位
        void FlipFireStatus() noexcept { FireStatus = FireStatus == 0 ? 0b11 : 0b00; }
    };

    /// @note 这里类型要求使用 @c std::uint16_t ，是因为在非 g++ 编译器上，此结构体的大小可能不符合预期
    struct GameCodeType {
        std::uint16_t IsGameBegin        : 1 = 0; // 比赛是否开始，0表示未开始，1表示开始
        std::uint16_t HeroPrecaution     : 1 = 0; // 防御英雄，暂时没有使用
        std::uint16_t IsMyTeamRed        : 1 = 0; // 我方颜色，0表示蓝方，1表示红方
        std::uint16_t EnemyOutpostHealth : 6 = 60; // 我方前哨站血量
        std::uint16_t SelfOutpostHealth  : 6 = 60; // 敌方前哨站血量
        std::uint16_t IsReturnedHome     : 1 = 0; // 是否返回基地，暂时没有使用
    };

    /// 场地事件数据（0x0101），按 RM2026 V1.3.0 定义，共4个字节
    struct ExtEventDataType
    {
        std::uint32_t SelfSupplyStatus : 3 = 0;            // bit0-2
        std::uint32_t SelfSmallEnergyStatus : 2 = 0;       // bit3-4
        std::uint32_t SelfLargeEnergyStatus : 2 = 0;       // bit5-6
        std::uint32_t SelfCentralHighlandStatus : 2 = 0;   // bit7-8
        std::uint32_t SelfTrapezoidHighlandStatus : 2 = 0; // bit9-10
        std::uint32_t EnemyLastDartHitTime : 9 = 0;        // bit11-19
        std::uint32_t EnemyLastDartHitTarget : 3 = 0;      // bit20-22
        std::uint32_t CenterGainPointStatus : 2 = 0;       // bit23-24
        std::uint32_t SelfFortressGainPointStatus : 2 = 0; // bit25-26
        std::uint32_t SelfOutpostGainPointStatus : 2 = 0;  // bit27-28
        std::uint32_t SelfBaseGainPointStatus : 1 = 0;     // bit29
        std::uint32_t Reserved : 2 = 0;                    // bit30-31
    };

    struct GimbalData {
        static constexpr auto TypeID = 0;

        GimbalAnglesType GimbalAngles;  // 8个字节
        VelocityType Velocity; // 2个字节
        FireCodeType FireCode; // 1个字节
        std::uint8_t CapV; // 1个字节
    };

    struct GameData {
        static constexpr auto TypeID = 1;

        GameCodeType GameCode{};  // 2个字节
        std::uint16_t AmmoLeft{}; // 2个字节 // 剩余弹药
        std::uint16_t TimeLeft{};  // 2个字节 // 比赛剩余时间
        std::uint16_t SelfHealth{};  // 2个字节 // 自己的血量
        std::uint32_t ExtEventData{};  // 4个字节 // 场地数据
    };

    // 不要再结构体里面调用构造函数
    struct HealthMyselfData {
        static constexpr auto TypeID = 2;

        std::uint16_t HeroMyself{200};
        std::uint16_t EngineerMyself{};
        std::uint16_t Infantry1Myself{200};
        std::uint16_t Infantry2Myself{};
        std::uint16_t BaseMyself{};
        std::uint16_t SentryMyself{};
    };

    struct HealthEnemyData {
        static constexpr auto TypeID = 3;

        std::uint16_t HeroEnemy{};
        std::uint16_t EngineerEnemy{};
        std::uint16_t Infantry1Enemy{};
        std::uint16_t Infantry2Enemy{};
        std::uint16_t BaseEnemy{}; // 基地血量，分度值100，0-50
        std::uint16_t SentryEnemy{};
    };
    //0x0204
    struct BuffType{
        std::uint8_t reserve;
        std::uint8_t RecoveryBuff; // 回血增益，百分比
        std::uint8_t CoolingBuff; // 热量冷却倍率，直接值
        std::uint8_t DefenceBuff; // 防御增益，百分比
        std::uint8_t VulnerabilityBuff; // 负防御增益，百分比
        std::uint16_t AttackBuff; // 攻击增益，百分比
        std::uint8_t RemainingEnergy; // 剩余能量值反馈
                                      // 十六进制，小于50%反馈，默认反馈0x32
                                      // 50%以上反馈0x1F，0b11111
                                      // 30%以上反馈0x1E，0b11110
                                      // 25%以上反馈0x1C，0b11100
                                      // 5%以上反馈0x18，0b11000
                                      // 1%以上反馈0x10，0b10000
    };

    struct RfidMatchState {
        bool Fresh{false};
        bool Any{false};
        std::uint32_t Raw{0};
        bool HasRfidStatus2{false};
        std::uint8_t RfidStatus2Raw{0};

        bool SelfBaseGainPoint{false};
        bool SelfSupply{false};
        bool SelfNonResourceSupply{false};
        bool SelfResourceSupply{false};
        bool SelfHighlandGainPoint{false};
        bool EnemyHighlandGainPoint{false};
        bool SelfRoadCrossing{false};
        bool EnemyRoadCrossing{false};
        bool SelfCentralHighlandCrossing{false};
        bool EnemyCentralHighlandCrossing{false};
        bool SelfTunnel{false};
        bool EnemyTunnel{false};
        bool Tunnel{false};
        bool CenterGainPoint{false};
        bool SelfFortressGainPoint{false};
        bool EnemyFortressGainPoint{false};
        bool SelfOutpostGainPoint{false};
        bool EnemyOutpostGainPoint{false};
        bool SelfAssemblyGainPoint{false};
        bool EnemyAssemblyGainPoint{false};
        bool SelfFlyRamp{false};
        bool EnemyFlyRamp{false};
        bool OnSelfSideRfid{false};
        bool OnEnemySideRfid{false};
    };

    struct RFIDAndBuffData{
        static constexpr auto TypeID = 4;

        BuffType BuffStatus;
        // 对齐 RM2026 V1.3.0 的 0x0209 rfid_status（低 32 位，bit0-31）
        std::uint32_t RFIDStatus;
    };
    static_assert(sizeof(RFIDAndBuffData) == sizeof(GimbalData), "TypeID=4 payload must stay 12B");

    struct PositionType{
        uint8_t CarId; /// 兵种id，x表示我方，100+x表示敌方
        UWBPositionType Position; // 4个字节
    };

    struct PositionData{
        static constexpr auto TypeID = 5;

        PositionType Friend; // 5字节
        PositionType Enemy; // 5字节
        uint16_t reserve;
    };

    struct GimbalControlData {
        std::uint8_t HeadFlag{'!'};
        GimbalAnglesType GimbalAngles;
        FireCodeType FireCode;
        std::uint8_t Tail{0};
    };
#pragma pack(pop)


#pragma endregion Gimbaldata

#pragma region Navigation
    using namespace std::chrono_literals;

    struct TeamedLocation
    {
        static constexpr auto LocationCount = 50;

        std::uint8_t ID;

        std::uint8_t operator()(const UnitTeam team) const
        {
            return team == UnitTeam::Blue ? ID + LocationCount : ID;
        }
    };
    static constexpr TeamedLocation Home{ 0 };
    static constexpr TeamedLocation Base{ 1 };
    static constexpr TeamedLocation Recovery{ 2 };
    static constexpr TeamedLocation BuffShoot{ 3 };
    static constexpr TeamedLocation LeftHighLand{ 4 };
    static constexpr TeamedLocation CastleLeft1{ 5 };
    static constexpr TeamedLocation Castle{ 6 };
    static constexpr TeamedLocation CastleRight1{ 7 };
    static constexpr TeamedLocation CastleRight2{ 8 };
    static constexpr TeamedLocation FlyRoad{ 9 };
    static constexpr TeamedLocation OutpostArea{ 10 };
    static constexpr TeamedLocation MidShoot{ 11 };
    static constexpr TeamedLocation LeftShoot{ 12 };
    static constexpr TeamedLocation OutpostShoot{ 13 };
    static constexpr TeamedLocation BuffAround1{ 14 };
    static constexpr TeamedLocation BuffAround2{ 15 };
    static constexpr TeamedLocation RightShoot{ 16 };
    static constexpr TeamedLocation HoleRoad{ 17 };
    static constexpr TeamedLocation OccupyArea{ 18 };
    static constexpr TeamedLocation Highland{ 19 };
    static constexpr TeamedLocation CastleLeft2{ 20 };
    static constexpr TeamedLocation BaseToCentral{ 21 };
    static constexpr TeamedLocation CentralToBase{ 22 };
    static constexpr TeamedLocation BuffOutpost{ 23 };
    static constexpr TeamedLocation OutpostGuard{ 24 };
    // 保持导航 BaseGoalId=25，作为正式 PreRoadland 目标点。
    static constexpr TeamedLocation PreRoadland{ 25 };
    static constexpr TeamedLocation CentralLeftA{ 26 };
    static constexpr TeamedLocation CentralLeftB{ 27 };
    // Tactical ProtectOutpost uses the official C3/C4 defense positions.
    static constexpr TeamedLocation ProtectOutpost{ 28 };
    // Tactical CommonCentral points. Keep CentralLeft.A/B (26/27) unchanged
    // for the legacy CentralLeft special patrol.
    static constexpr TeamedLocation CentralHigh{ 29 };
    static constexpr TeamedLocation CentralLow{ 30 };

    /// @brief 团队类型
    enum class NaviTeam : std::uint8_t
    {
        Unknown = 0,
        Myself = Unknown + 1,
        Enemy = Myself + 1,
        Other = Enemy + 1
    };

    // 行为树用于记录导航信息
    struct NaviPosition
    {
        TeamedLocation Location{ Home };
        NaviTeam Team{ NaviTeam::Unknown };
    };

#pragma pack(push, 1)
    struct LocatorMessage final {
        struct RotationType {
            float W;
            float X;
            float Y;
            float Z;
        };

        RotationType Rotation{};

        struct LocationType {
            float X;
            float Y;
            float Z;
        };

        LocationType Location{};
        std::int8_t Neighbor{};
    };

    struct NaviCommandMessage {
        std::uint8_t Head{'!'};
        std::uint8_t DestinationID{};
        std::uint8_t CRC{0};
    };

    struct NaviControlMessage {
        std::uint8_t Head{'!'};
        std::uint8_t LocationID{0};
        VelocityType Velocity{};
        LocatorMessage Locator{};
        std::uint8_t CRC{0};
    };
#pragma pack(pop)

#pragma endregion Navigation

#pragma region Aim

     struct ArmorData { // 接收来自辅瞄的装甲板序列
        ArmorType Type{ArmorType::UnKnown}; // 装甲板类型
        float Distance{30.0}; // 距离, 单位：米
    };
    /**
     * @brief 决策发送给辐瞄的数据
     */
    struct AimTargetData {
        std::uint8_t Head{'!'};
        ArmorType Target{ArmorType::Hero}; // 击打目标
        std::uint8_t CRC{0};
    };

    struct AimData { // 打符或辅瞄模式接收的数据
        bool FireStatus{false}; // true 表示开火
        bool BuffFollow{false};  // 用于打符的跟随
        bool Valid{false};       // 当前模式可直接使用的完整 yaw/pitch 对
        bool Fresh{false};       // 本循环内是否收到新的一帧
        bool HasLatchedAngles{false}; // 是否有上一帧可继续保持的有效锁角
        std::chrono::steady_clock::time_point LastValidTime{}; // 上一次收到有效锁角的时间
        GimbalAnglesType Angles; // 云台的控制角度
        float YawOmega{0.0F}; // 目标 yaw 角速度，deg/s
        float PitchOmega{0.0F}; // 目标 pitch 角速度，deg/s
        float YawAlpha{0.0F}; // 目标 yaw 角加速度，deg/s^2
        float PitchAlpha{0.0F}; // 目标 pitch 角加速度，deg/s^2
    };

    struct ExternalAimTargetCache {
        bool Valid{false};
        float X{0.0f};
        float Y{0.0f};
        float Z{0.0f};
        float Distance{30.0f};
        std::string FrameId{};
        std::chrono::steady_clock::time_point LastSeen{};
    };

    enum class AimMode : std::uint8_t { // 瞄准模式
        None = 0,
        AutoAim = 1,
        RotateScan = 2,
        Buff = 3,
        Outpost = 4,
        FaceMode = 5
    };
#pragma endregion Aim


#pragma region Configuration

    // 辅瞄调试
    struct AimDebug {
        bool StopFire{false};
        bool StopRotate{false};
        bool StopScan{false};
        bool ForceOutpost{false};
        bool ForceBuff{false};
        bool HitCar{false};
        bool FireRequireTargetStatus{true};
        bool ReuseLatchedAnglesOnNoTarget{true};
        int LatchedTargetHoldMs{100};
    };

    // 巡逻扫描配置
    struct PatrolScanSetting {
        int Mode{1}; // 1=原始单向/受击抖扫, 2=左右摆头, 3=慢速高位单向扫
        double Mode1YawStepDegPerTick{9.0};
        double Mode1YawBoostStepDegPerTick{10.0};
        double Mode1PitchCenterDeg{5.0};
        double Mode1PitchHalfRangeDeg{15.0};
        double Mode1PitchPeriodMs{2000.0};
        double Mode2YawStepDegPerTick{1.0};
        double Mode2YawBoostStepDegPerTick{1.1};
        double Mode2YawHalfRangeDeg{30.0};
        double Mode2CenterDriftPerCycleDeg{-70.0};
        double Mode2PitchCenterDeg{0.0};
        double Mode2PitchHalfRangeDeg{13.0};
        double Mode2PitchPeriodMs{500.0};
        double Mode2ResumeYawToleranceDeg{1.0};
        int Mode2ResumeMaxMs{800};
        double Mode3YawStepDegPerTick{6.0};
        double Mode3PitchOffsetDeg{0.0};
        double Mode3PitchHalfRangeDeg{12.0};
        double Mode3PitchPeriodMs{2000.0};
        double PassiveYawRateDegPerSec{120.0};
        double PassivePitchRateDegPerSec{60.0};
        int PassiveMaxIntervalMs{25};
        bool FaceModeFallbackEnable{true};
        int FaceModeFallbackMode{2};
        int OutpostFaceModeFallbackMode{2};
        int OutpostDamageAbortMode{2};
        double StartGatePitchOffsetDeg{10.0};
        bool StartGatePitchOffsetApplyToMode3{false};
        double OutpostPitchOffsetDeg{15.0};
        bool OutpostPitchOffsetApplyToMode3{true};
        bool FaceModeFallbackEnableProvided{false};
        bool FaceModeFallbackModeProvided{false};
        bool OutpostFaceModeFallbackModeProvided{false};
    };

    // 频率相关
    struct Rate {
        int FireRate{20};
        int TreeTickRate{100};
        int NaviCommandRate{1};
    };
    struct BuffTimerSetting {
        bool Enable{false};
        int StartSec{0};
        int EndSec{25};
        int MaxShootCount{15};
    };

    struct BuffConfirmSetting {
        int RefereeFreshTimeoutMs{2000};
        int PulseMs{500};
        int RetryIntervalMs{2000};
        int PostConfirmGraceMs{3000};
        int TaskHoldTimeoutMs{30000};
        int DamageAbortThreshold{30};
        int DamageAbortWindowMs{1000};
        int DamageAbortHoldMs{5000};
    };

    struct OutpostConfirmSetting {
        int RefereeFreshTimeoutMs{2000};
        bool TrustEnemyOutpostHp{true};
        bool EnhancedAttackOnEnemyHpDrop{true};
        int NormalAttackLockExitHp{200};
        int EnhancedAttackLockExitHp{250};
        int MaxGameTimeSec{120};
        int MinSelfHp{150};
        int MinAmmo{30};
        bool VisualScoutWithoutHp{true};
        int VisualScoutHoldMs{10000};
        int VisualScoutCooldownMs{15000};
        int VisualScoutFaceDistanceCm{300};
        bool PostWindowScoutEnable{true};
        int PostWindowScoutIntervalSec{60};
        int PostWindowScoutHoldMs{5000};
        int ArmorWarningDistanceCm{1000};
        int ArmorInterruptMaxDistanceCm{1000};
        int PostArmorFaceSearchMs{5000};
        int DamageAbortThreshold{30};
        int DamageAbortWindowMs{1000};
        int DamageAbortHoldMs{3000};
        bool OpeningHighPriority{true};
        int OpeningHoldSec{120};
        bool OpeningHoldUntilWindowEnd{true};
        bool SuppressChaseWhileActive{true};
        bool ManualGoalEnable{false};
        double ManualGoalMapXM{0.0};
        double ManualGoalMapYM{0.0};
        double ManualGoalMapZM{0.0};
    };

    struct MapCommandSetting {
        bool Enable{true};
        int HoldSec{45};
        int DedupDistanceCm{20};
    };

    struct TaskSetting {
        bool Buff{false};
        bool Outpost{false};
        BuffTimerSetting BuffTimer{};
        BuffConfirmSetting BuffConfirm{};
        OutpostConfirmSetting OutpostConfirm{};
        MapCommandSetting MapCommand{};
    };

    struct DamageOpenGateSetting {
        bool Enable{false};
        std::uint16_t HealthDropThreshold{30};
    };

    struct StartGateSetting {
        bool AllowGimbalPatrolBeforeStart{false};
        std::string GimbalStrategy{"patrol"};
        int FaceModeStatusFreshMs{500};
    };

    struct NaviSetting {
        bool UseXY{true};
        // UseXY=true 时：
        // true  -> 交给导航链路：/ly/navi/target_rel 或 /ly/navi/goal_pos_raw 经 navi_tf_bridge 输出 /goal_pose
        // false -> behavior_tree 直接发布地图绝对坐标到 /ly/navi/goal_pos（不走 tf bridge）
        bool ToNavi{true};
    };

    struct FaceModeSetting {
        bool Enable{false};
        int LostTargetHoldMs{300};
        bool SuppressFire{true};
        bool FallbackToPatrolScanMode2{true};
        int FallbackPatrolScanMode{2};
        int OutpostFallbackPatrolScanMode{2};
        bool FallbackToPatrolScanMode2Provided{false};
        bool FallbackPatrolScanModeProvided{false};
        bool OutpostFallbackPatrolScanModeProvided{false};
    };

    struct ExternalAimSetting {
        bool Enable{true};
        int ResultFreshTimeoutMs{300};
        int TargetFreshTimeoutMs{500};
        bool UseTargetArrayAsArmorList{true};
        bool PublishSelectTarget{true};
        std::string TargetDefaultFrame{"gimbal_world"};
    };

    struct NaviControlSetting {
        bool Enable{false};
        // Publish the selected navigation speed level beside each outgoing navigation command.
        bool IsPubNaviSpeedLevel{false};
        int FreshTimeoutMs{500};
        bool DefaultIsRotate{true};
        bool ForceFollowModeWhenFalse{true};
        bool ClearFollowModeWhenTrue{true};
        bool ClearRegionalFaceModeWhenTrue{true};
        bool StopRotateWhenFalse{true};
        // 新鲜 should_rotate=false 时，只覆盖本 tick 的目标姿态为 Move；冷却期间信号解除不会补切。
        bool SetPostureToMoveWhenFalse{false};
    };

    struct DamageRotateSetting {
        std::uint8_t DefaultGear{0};
        int NoHitTimeoutMs{1800};
        int Gear0HoldMs{220};
        int Gear1HoldMs{220};
        int Gear2HoldMs{220};
        int ScanBoostWindowMs{1300};
        int ScanYawPhaseMs{160};
    };

    struct ProtectHeroEnhancedDefenseSetting {
        bool Enable{false};
        int DamageWindowMs{1500};
        int DamageThresholdHp{30};
    };

    struct ProtectHeroSetting {
        bool Enable{true};
        // Defaults off so legacy JSON-only profiles retain their prior behavior.
        ProtectHeroEnhancedDefenseSetting EnhancedDefense{};
        // YAML-enabled regional policy: hold Highland before an enemy enters.
        // Keep the legacy baseline false for JSON-only profiles.
        bool ProactiveHoldWhenHeroInHighland{false};
        int StartElapsedSec{120};
        int HoldSec{30};
        int NoEnemyReleaseSec{8};
        int FriendPositionFreshMs{2500};
        int FriendHealthFreshMs{2500};
        std::uint8_t GoalBaseId{Highland.ID};
    };

    // Legacy JSON used HeroProtection. Keep its type and Config field so old
    // competition profiles remain valid; Tactical.ProtectHero is the runtime owner.
    using HeroProtectionSetting = ProtectHeroSetting;

    struct TacticalPrioritySetting {
        int ProtectCastle{1};
        int ProtectOutpost{2};
        int CommonCentral{3};
        int ProtectHero{4};
        int Chase{5};
    };

    struct TacticalRegionalDefenseCommonCentralSetting {
        // Tactical CommonCentral is separate from Default AreaManager.CommonCentral.
        bool Enable{true};
        int HoldSec{15};
    };

    struct TacticalRegionalDefenseSetting {
        TacticalRegionalDefenseCommonCentralSetting CommonCentral{};
    };

    struct ProtectOutpostSetting {
        bool Enable{true};
        int HealthFreshMs{2000};
        int DamageWindowMs{2000};
        int DamageThresholdHp{20};
        int SearchHoldSec{30};
        int UnreachableCooldownSec{10};
    };

    struct ProtectCastleSetting {
        bool Enable{true};
        bool RFID{true};
        bool EnemyPos{true};
        bool StayWhenRfid{false};
        bool Base{true};
        int OccupancyPositionFreshMs{2500};
        int CastlePositionMarginCm{60};
        int ArrivalConfirmGraceMs{3000};
        int RfidCaptureTransitionWindowMs{3000};
    };

    struct EnhancedRecoveryMoveSetting {
        bool Enable{true};
        int HealthThresholdHp{80};
        int RespawnSuppressSec{30};
    };

    struct EnhancedPostureSetting {
        int ContradictionGraceMs{500};
        EnhancedRecoveryMoveSetting RecoveryMove{};
    };

    struct TacticalSetting {
        DamageRotateSetting DamageRotate{};
        TacticalPrioritySetting Priority{};
        TacticalRegionalDefenseSetting RegionalDefense{};
        EnhancedPostureSetting EnhancedPosture{};
        ProtectCastleSetting ProtectCastle{};
        ProtectOutpostSetting ProtectOutpost{};
        ProtectHeroSetting ProtectHero{};
    };

    struct SentryPositionFusionSourceSetting {
        bool Enable{true};
        int Priority{0};
        double Weight{1.0};
        int FreshTimeoutMs{0}; // <=0 uses SentryPositionFusionSetting::FreshTimeoutMs
    };

    struct SentryPositionFusionSetting {
        bool Enable{true};
        std::string Mode{"priority"}; // priority or weighted
        int FreshTimeoutMs{2000};
        SentryPositionFusionSourceSetting Uwb{true, 0, 1.0, 0};
        SentryPositionFusionSourceSetting PositionData{true, 2, 0.7, 0};
        SentryPositionFusionSourceSetting Navi{true, 1, 0.8, 0};
    };

    struct League3v3Setting {
        bool Enable{false};
        bool AssumeCoordination{true};
        std::uint16_t RecoveryHp{250};
        std::uint16_t RecoveryExitHp{400};
        std::uint16_t SelfWeakHp{275};
        std::uint16_t TeammateWeakHp{150};
        std::uint16_t EnemyWeakHp{150};
        int CenterPatrolHoldSec{5};
        int CenterPatrolTimeoutSec{10};
        int PositionFreshTimeoutMs{2500};
        int HealthFreshTimeoutMs{1000};
    };
    struct LeagueStrategySetting {
        bool EnableRouteCompat{true};
        bool UseHealthRecovery{true};
        std::uint16_t HealthRecoveryThreshold{100};
        bool UseAmmoRecovery{true};
        std::uint16_t AmmoRecoveryThreshold{30};
        bool DamageScanBoostEnable{true};
        std::uint16_t HealthRecoveryExitMin{350};
        std::uint16_t HealthRecoveryExitPreferred{400};
        int HealthRecoveryPlateauSec{2};
        int HealthRecoveryExitStableSec{1};
        int HealthRecoveryMaxHoldSec{12};
        int HealthRecoveryCooldownSec{20};
        League3v3Setting League3v3{};
        std::uint8_t MainGoal{OccupyArea.ID};
        std::vector<std::uint8_t> PatrolGoals{};
        int GoalHoldSec{15};
    };

    struct ShowcasePatrolSetting {
        bool Enable{false};
        std::vector<std::uint8_t> Goals{};
        int GoalHoldSec{5};
        bool Random{false};
        bool DisableTeamOffset{false};
        bool IgnoreRecovery{false};
    };

    struct NaviDebugSetting {
        bool Enable{false};
        std::string PlanFile{};
        std::string ActivePlan{};
        std::vector<std::uint8_t> Goals{};
        int GoalHoldSec{5};
        bool Random{false};
        bool DisableTeamOffset{false};
        bool IgnoreRecovery{true};
        std::uint8_t SpeedLevel{1};
    };

    struct ChaseAreaLimitSetting {
        bool Enable{false};
        int BoundaryMarginCm{30};
        bool ChaseEnableCrossArea{false};
        bool HoldWhenNoIntersection{true};
    };

    // 底盘追击配置
    struct ChaseSetting {
        bool Enable{false};
        bool FollowAimTarget{true};
        bool ToNavi{false}; // true: publish /ly/navi/target_rel and let navi own speed control
        bool UseOfficialPositionSource{true}; // true: chase can use /ly/position/data official-map target positions
        bool PreferOfficialPositionSource{false}; // false: AimTargetArray/target_rel is preferred, official position is fallback
        bool EnableNaviTargetOfficialFallback{true}; // camera/TF official-map position fallback into BT
        int OfficialPositionFreshMs{500};
        bool EnableInAutoAim{true};
        bool EnableInRotateScan{true};
        bool EnableInOutpostMode{false};
        bool EnableInBuffMode{false};
        bool StopWhenNoTarget{false};
        int LostTargetHoldMs{200};

        // 与目标保持的最佳距离（单位：cm）
        int PreferredDistanceCm{100};
        int DistanceDeadbandCm{5};
        ChaseAreaLimitSetting AreaLimit{};
        int MinValidDistanceCm{80};
        int MaxValidDistanceCm{1200};

        // 追击速度控制
        double DistanceKp{0.3};
        int MaxForwardSpeed{150};
        int MaxBackwardSpeed{95};

        // 侧向修正（基于云台 yaw 误差）
        bool UseYawStrafe{true};
        double YawKp{1.8};
        int YawDeadbandDeg{1};
        int MaxStrafeSpeed{90};
        bool InvertStrafeDirection{false};
    };

    // Regional 追击的区域所有权开关；距离和速度仍由 ChaseSetting 管理。
    struct ChasePolicySetting {
        bool Enable{false};
        bool MyBase{false};
        bool MyHighland{false};
        bool MyPreRoadland{false};
        bool MyReadyRoadland{false};
        bool CommonCentral{false};
    };

    // 姿态模块配置
    struct PostureSetting {
        bool Enable{true};
        int SwitchCooldownSec{5};     // 规则: 姿态切换冷却
        int MaxSinglePostureSec{180}; // 规则: 单姿态累计超过该值会降档
        int EarlyRotateSec{165};      // 接近降档前提前轮换
        int RefereeInfo3FreshMs{1500}; // 0x020D sentry_info_3 新鲜时优先使用裁判剩余秒数
        int FeedbackFreshMs{1000};    // /ly/gimbal/posture 回读最大有效年龄
        int RefereeRemainWarnSec{20}; // 裁判姿态剩余秒数不高于此值时开始降低候选分数
        int RefereeRemainPenalty{5};  // 剩余秒数进入预警区后的最大扣分
        int RefereeZeroRemainPenalty{20}; // 裁判剩余秒数为0时的候选扣分
        int EnhancedCurrentPostureBonus{3}; // 当前为强化姿态时，同类攻/防/移的保持加分
        int MinHoldSec{10};           // 防抖: 最短保持时间
        int PendingAckTimeoutMs{600}; // 等待回读超时
        int RetryIntervalMs{300};     // 重试间隔
        int MaxRetryCount{3};         // 最大重试次数
        int EnhancedPendingAckTimeoutMs{800}; // 强化姿态等待回读超时
        int EnhancedRetryIntervalMs{300};     // 强化姿态重试间隔
        int EnhancedMaxRetryCount{5};         // 强化姿态最大重试次数
        bool OptimisticAck{true};     // 回读缺失时是否乐观确认
        int TargetKeepMs{800};        // 目标短时丢失容忍，防止姿态抖动
        int DamageKeepSec{4};         // 最近受击保持时间窗口
        int DamageBurstWindowMs{0};   // 短时间受击统计窗口，0=关闭
        int DamageBurstThreshold{0};  // 窗口内累计掉血阈值，0=关闭
        int DamageBurstDefenseHoldSec{0}; // 触发短时重受击后的防守保持时长
        int LowHealthThreshold{120};  // 低血阈值
        int VeryLowHealthThreshold{80}; // 极低血阈值
        int LowAmmoThreshold{30};     // 低弹阈值
        int ScoreHysteresis{2};       // 姿态切换分差迟滞
        bool DynamicTransitReserveEnable{true}; // 按当前导航 ETA 动态保留 Move 额度
        int TransitVelocityFreshMs{1500}; // /ly/navi/vel 可用于 ETA 的最大年龄
        double TransitNominalSpeedMps{0.8}; // 速度回读不可用时的保守估计
        double TransitSafetyFactor{1.5}; // ETA 安全系数
        int TransitArrivalBufferSec{8}; // 到点确认、减速和重规划缓冲
        int TransitMinReserveSec{30}; // 无论距离多短，Move 最低保留
        int TransitMaxReserveSec{120}; // 动态保留上限，避免无限期锁住 Move
        int TransitFallbackReserveSec{45}; // 位置失鲜/无目标时的保守保留
    };

    struct NaviGoalAutonomySetting {
        bool UseAreaScope{false};
        std::vector<std::string> MyArea{};
        std::vector<std::string> EnemyArea{};
        std::vector<std::string> CommonArea{};
        bool HighlandCompatEnable{false};
        bool HighlandCompatDisableRotate{false};
        int HighlandCompatArriveDistanceCm{20};
        int HighlandCompatTimeoutSec{6};
        bool BuffOutpostCompatEnable{false};
        int BuffOutpostCompatTimeoutSec{6};
        int DistanceFallbackGraceMs{3000};
        int NearGoalConfirmWaitMs{1500};
    };

    struct RegionalDefenseSetting {
        bool Enable{true};
        bool EnableSoftEnemySideThreat{true};
        int EnemyPositionFreshMs{2500};
        int HardHoldSec{5};
        int SoftHoldSec{8};
        int SearchHoldSec{4};
        int SearchNoTargetSec{4};
        int FortressStandEnemyCountMin{2};
        int FortressNoContactDegradeSec{8};
        int FortressDegradeCooldownSec{6};
        std::uint16_t StrongHealthMin{250};
        std::uint16_t StrongAmmoMin{40};
        int MultiEnemyBaseCount{2};
    };

    struct NaviProgressWatchdogSetting {
        bool Enable{false};
        int MoveProgressCm{80};
        int NoMoveTimeoutSec{14};
        int FallbackHoldSec{5};
        int FallbackCooldownSec{12};
    };

    struct RegionalIdlePatrolSetting {
        bool Enable{false};
        int GoalHoldSec{8};
        bool GoalEnableProvided{false};
        std::vector<std::uint8_t> Goals{
            LangYa::HoleRoad.ID,
            LangYa::Castle.ID,
            LangYa::CastleRight2.ID,
            LangYa::CastleRight1.ID,
            LangYa::CastleLeft1.ID,
            LangYa::CastleLeft2.ID
        };
    };

    struct SpecialPatrolSetting {
        bool Enable{false};
        int GoalHoldSec{0};
        int SpeedLevel{1};
        bool SuppressChase{true};
        bool StopOnTarget{true};
    };

    struct SpecialSetting {
        SpecialPatrolSetting Patrol{};
    };

    struct MyHighlandAreaTaskSetting {
        bool Enable{true};
        bool UseFaceMode{false};
        int ApproachTimeoutSec{8};
        int HighlandPatrolHoldSec{15};
        int BuffShootTravelTimeoutSec{8};
        int BuffShootHoldSec{15};
        int LeaveTimeoutSec{8};
    };

    struct PatrolGoalSetting {
        std::uint8_t BaseGoalId{LangYa::CastleLeft2.ID};
        double Weight{10.0};
    };

    struct PatrolGoalSelectionSetting {
        double DistancePenaltyPerMeter{0.4};
        double CurrentGoalPenalty{5.0};
        bool AvoidCurrentGoal{true};
        double UnvisitedBonus{12.0};
        double FreshnessBonusMax{12.0};
        int FreshnessTimeoutSec{120};
        double RecentVisitPenalty{8.0};
        int RecentVisitPenaltySec{30};
    };

    using MyBasePatrolGoalSetting = PatrolGoalSetting;

    struct MyBaseAreaTaskSetting {
        bool Enable{true};
        int TravelTimeoutSec{12};
        int CommandHoldSec{1};
        int GoalHoldSec{15};
        int MaxPatrolSteps{4};
        std::vector<PatrolGoalSetting> PatrolGoals{
            {LangYa::CastleLeft1.ID, 1.0},
            {LangYa::CastleLeft2.ID, 1.0},
            {LangYa::CastleRight2.ID, 1.0},
            {LangYa::CastleRight1.ID, 1.0}
        };
    };

    struct MyReadyRoadlandAreaTaskSetting {
        bool Enable{true};
        bool UseFaceMode{false};
        int TravelTimeoutSec{12};
        int CrossTimeoutSec{8};
        int CommandHoldSec{1};
        int GuardHoldSec{15};
        int FaceTargetZCm{100};
        int HealthyHpMin{300};
        int HealthyAmmoMin{50};
    };

    struct MyPreRoadlandAreaTaskSetting {
        bool Enable{true};
        int TravelTimeoutSec{12};
        int GoalHoldSec{15};
        int CommandHoldSec{1};
        int SpeedLevel{1};
    };

    struct CommonCentralAreaTaskSetting {
        bool Enable{true};
        int TravelTimeoutSec{12};
        int GoalHoldSec{15};
        int CommandHoldSec{1};
        int MaxPatrolSteps{8};
        int HealthyHpMin{300};
        int HealthyAmmoMin{50};
    };

    struct DefaultPolicyHealthSetting {
        int MyAreaHpMin{250};
        int CommonCentralHpMin{300};
        int EnemyAreaHpMin{350};
        int LowResourceFallbackHp{250};
    };

    struct DefaultPolicyAmmoSetting {
        int MyAreaAmmoMin{50};
        int CommonCentralAmmoMin{50};
        int EnemyAreaAmmoMin{80};
        int LowResourceFallbackAmmo{30};
    };

    struct DefaultPolicyScoreSetting {
        double WeightMyBase{10.0};
        double WeightMyHighland{8.0};
        double WeightMyPreRoadland{7.0};
        double WeightMyReadyRoadland{7.0};
        double WeightCommonCentral{6.0};
        double WeightEnemyBase{4.0};
        double WeightEnemyHighland{4.0};
        double WeightEnemyReadyRoadland{4.0};
        double DistancePenaltyPerMeter{0.4};
        double CurrentAreaPenalty{2.0};
        double LastAreaPenalty{1.0};
        double AfterHighlandMyBaseBonus{5.0};
        double AfterHighlandMyReadyRoadlandBonus{3.0};
        double LowResourceMyBaseBonus{4.0};
    };

    struct DefaultPolicyRetrySetting {
        int CompleteCooldownSec{2};
        int FailureCooldownSec{8};
        int UnreachableCooldownSec{12};
        int MaxRetry{2};
    };

    struct DefaultPolicySetting {
        bool Enable{true};
        DefaultPolicyHealthSetting Health{};
        DefaultPolicyAmmoSetting Ammo{};
        DefaultPolicyScoreSetting Score{};
        DefaultPolicyRetrySetting Retry{};
    };

    struct RegionalAreaTaskSetting {
        bool Enable{true};
        bool IgnoreRecovery{false};
        PatrolGoalSelectionSetting PatrolSelection{};
        MyHighlandAreaTaskSetting MyHighland{};
        MyBaseAreaTaskSetting MyBase{};
        MyPreRoadlandAreaTaskSetting MyPreRoadland{};
        MyReadyRoadlandAreaTaskSetting MyReadyRoadland{};
        CommonCentralAreaTaskSetting CommonCentral{};
        DefaultPolicySetting DefaultPolicy{};
    };

    struct AimTargetAutonomySetting {
        bool Enable{false};
        double PriorityWeight{1.0};
        double DistanceWeight{0.8};
        double LowHealthWeight{0.6};
        double CurrentTargetBonus{0.3};
        double HeroBonus{0.2};
        double SentryBonus{0.1};
        int HealthFreshTimeoutMs{800};
        int DeadHealthConfirmMs{500};
        int DeadHealthHoldMs{1200};
        int RespawnTransitionTimeoutMs{3000};
        int LostTargetHoldMs{200};
        int MinSwitchIntervalMs{500};
        double SwitchScoreMargin{0.25};
        int RespawnInvulnerableSec{30};
        int SentryRespawnInvulnerableSec{30};
    };

    struct DecisionAutonomySetting {
        bool Enable{false};
        std::vector<std::string> EnabledModules{"aim_target"};
        std::vector<std::string> HardRuleModules{"recovery", "aim_mode", "fire_safety"};
        NaviGoalAutonomySetting NaviGoal{};
        AimTargetAutonomySetting AimTarget{};
    };

    // Main
    struct Config {
        AimDebug AimDebugSettings{};
        PatrolScanSetting PatrolScanSettings{};
        Rate RateSettings{};
        bool SwitchPoint{false};
        TaskSetting TaskSettings{};
        DamageOpenGateSetting DamageOpenGateSettings{};
        StartGateSetting StartGateSettings{};
        NaviSetting NaviSettings{};
        FaceModeSetting FaceModeSettings{};
        ExternalAimSetting ExternalAimSettings{};
        NaviControlSetting NaviControlSettings{};
        TacticalSetting TacticalSettings{};
        SentryPositionFusionSetting SentryPositionFusionSettings{};
        LeagueStrategySetting LeagueStrategySettings{};
        ShowcasePatrolSetting ShowcasePatrolSettings{};
        NaviDebugSetting NaviDebugSettings{};
        RegionalDefenseSetting RegionalDefenseSettings{};
        HeroProtectionSetting HeroProtectionSettings{};
        NaviProgressWatchdogSetting NaviProgressWatchdogSettings{};
        RegionalIdlePatrolSetting RegionalIdlePatrolSettings{};
        SpecialSetting SpecialSettings{};
        RegionalAreaTaskSetting RegionalAreaTaskSettings{};
        std::vector<int> AimTargetPriority{
            static_cast<int>(ArmorType::Hero),
            static_cast<int>(ArmorType::Infantry1),
            static_cast<int>(ArmorType::Infantry2),
            static_cast<int>(ArmorType::Sentry),
            static_cast<int>(ArmorType::Engineer)
        };
        std::vector<int> AimTargetIgnore{};
        DecisionAutonomySetting DecisionAutonomySettings{};
        ChaseSetting ChaseSettings{};
        ChasePolicySetting ChasePolicySettings{};
        PostureSetting PostureSettings{};
        int ScanCounter{1};  /// 扫描模式计数器，一定值后Yaw动一次
        std::string CompetitionProfile{"regional"};
    };

#pragma endregion Configuration
}
#pragma endregion

//數據框架定義
