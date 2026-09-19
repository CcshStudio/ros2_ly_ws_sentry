// AUTO-COMMENT: file overview
// Optional JSONL decision trace writer for offline visualization tools.

#include "../include/Application.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <stdexcept>

using namespace LangYa;
using namespace BehaviorTree;

namespace BehaviorTree {

namespace {

constexpr int kTraceFieldWidthCm = 2800;
constexpr int kTraceFieldHeightCm = 1500;
constexpr int kLeague3v3TraceFieldWidthCm = 1200;
constexpr int kLeague3v3TraceFieldHeightCm = 800;
constexpr float kTraceVelocityRawToMps = 0.025f;

const char* UnitTeamToString(const UnitTeam team) noexcept {
    switch (team) {
        case UnitTeam::Red: return "red";
        case UnitTeam::Blue: return "blue";
        case UnitTeam::Other: return "other";
        default: return "unknown";
    }
}

const char* AimModeToString(const AimMode mode) noexcept {
    switch (mode) {
        case AimMode::None: return "None";
        case AimMode::AutoAim: return "AutoAim";
        case AimMode::RotateScan: return "RotateScan";
        case AimMode::Buff: return "Buff";
        case AimMode::Outpost: return "Outpost";
        default: return "Unknown";
    }
}

const char* ArmorTypeToString(const ArmorType type) noexcept {
    switch (type) {
        case ArmorType::Base: return "Base";
        case ArmorType::Hero: return "Hero";
        case ArmorType::Engineer: return "Engineer";
        case ArmorType::Infantry1: return "Infantry1";
        case ArmorType::Infantry2: return "Infantry2";
        case ArmorType::Infantry3: return "Infantry3";
        case ArmorType::Sentry: return "Sentry";
        case ArmorType::Outpost: return "Outpost";
        default: return "Unknown";
    }
}

const char* UnitTypeToString(const UnitType type) noexcept {
    switch (type) {
        case UnitType::Hero: return "Hero";
        case UnitType::Engineer: return "Engineer";
        case UnitType::Infantry1: return "Infantry1";
        case UnitType::Infantry2: return "Infantry2";
        case UnitType::Infantry3: return "Infantry3";
        case UnitType::Drone: return "Drone";
        case UnitType::Sentry: return "Sentry";
        case UnitType::Dart: return "Dart";
        case UnitType::Radar: return "Radar";
        default: return "Unknown";
    }
}

const char* GoalName(const int base_goal_id, const bool league_3v3) noexcept {
    if (league_3v3) {
        return League3v3GoalName(static_cast<League3v3Goal>(base_goal_id));
    }
    switch (base_goal_id) {
        case LangYa::Home.ID: return "Home";
        case LangYa::Base.ID: return "Base";
        case LangYa::Recovery.ID: return "Recovery";
        case LangYa::BuffShoot.ID: return "BuffShoot";
        case LangYa::LeftHighLand.ID: return "LeftHighLand";
        case LangYa::CastleLeft1.ID: return "CastleLeft1";
        case LangYa::Castle.ID: return "Castle";
        case LangYa::CastleRight1.ID: return "CastleRight1";
        case LangYa::CastleRight2.ID: return "CastleRight2";
        case LangYa::FlyRoad.ID: return "FlyRoad";
        case LangYa::OutpostArea.ID: return "OutpostArea";
        case LangYa::MidShoot.ID: return "MidShoot";
        case LangYa::LeftShoot.ID: return "LeftShoot";
        case LangYa::OutpostShoot.ID: return "OutpostShoot";
        case LangYa::BuffAround1.ID: return "BuffAround1";
        case LangYa::BuffAround2.ID: return "BuffAround2";
        case LangYa::RightShoot.ID: return "RightShoot";
        case LangYa::HoleRoad.ID: return "HoleRoad";
        case LangYa::OccupyArea.ID: return "OccupyArea";
        case LangYa::Highland.ID: return "Highland";
        case LangYa::CastleLeft2.ID: return "CastleLeft2";
        case LangYa::BaseToCentral.ID: return "BaseToCentral";
        case LangYa::CentralToBase.ID: return "CentralToBase";
        case LangYa::BuffOutpost.ID: return "BuffOutpost";
        case LangYa::OutpostGuard.ID: return "OutpostGuard";
        case LangYa::PreRoadland.ID: return "PreRoadland";
        case LangYa::CentralLeftA.ID: return "CentralLeft.A";
        case LangYa::CentralLeftB.ID: return "CentralLeft.B";
        case LangYa::ProtectOutpost.ID: return "ProtectOutpost";
        case LangYa::CentralHigh.ID: return "CentralHigh";
        case LangYa::CentralLow.ID: return "CentralLow";
        default: return "Unknown";
    }
}

int GoalBaseId(const std::uint8_t goal_id) noexcept {
    return goal_id >= LangYa::TeamedLocation::LocationCount
        ? static_cast<int>(goal_id - LangYa::TeamedLocation::LocationCount)
        : static_cast<int>(goal_id);
}

const char* GoalSide(const std::uint8_t goal_id) noexcept {
    return goal_id >= LangYa::TeamedLocation::LocationCount ? "blue" : "red";
}

json FiniteFloat(const float value) {
    if (std::isfinite(value)) {
        return value;
    }
    return nullptr;
}

json AgeMsOrNull(
    const std::chrono::steady_clock::time_point now,
    const std::chrono::steady_clock::time_point stamp,
    const bool valid) {
    if (!valid || stamp.time_since_epoch().count() == 0) {
        return nullptr;
    }
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - stamp).count();
    return static_cast<int>(std::max<std::chrono::milliseconds::rep>(0, age_ms));
}

json UnitListToJson(const std::vector<UnitType>& units) {
    json out = json::array();
    for (const auto unit : units) {
        out.push_back({
            {"id", static_cast<int>(unit)},
            {"name", UnitTypeToString(unit)},
        });
    }
    return out;
}

json RobotToJson(const Robot& robot, const UnitType type, const char* side) {
    return {
        {"type_id", static_cast<int>(type)},
        {"type", UnitTypeToString(type)},
        {"side", side},
        {"team_id", static_cast<int>(robot.team_)},
        {"hp", static_cast<int>(robot.currentHealth_)},
        {"max_hp", static_cast<int>(robot.maxHealth_)},
        {"distance_m", FiniteFloat(robot.distance_)},
        {"position_cm", {
            {"x", static_cast<int>(robot.position_.X)},
            {"y", static_cast<int>(robot.position_.Y)},
        }},
    };
}

json RobotsToJson(const Robots& robots, const char* side, const bool league_3v3) {
    static constexpr std::array<UnitType, 6> kTraceUnits{
        UnitType::Hero,
        UnitType::Engineer,
        UnitType::Infantry1,
        UnitType::Infantry2,
        UnitType::Infantry3,
        UnitType::Sentry,
    };
    static constexpr std::array<UnitType, 3> kLeague3v3Units{
        UnitType::Hero,
        UnitType::Infantry1,
        UnitType::Sentry,
    };

    json out = json::array();
    const auto append_unit = [&](const UnitType type) {
        const auto& robot = robots[type];
        if (league_3v3 && robot.position_.X == 0 && robot.position_.Y == 0) {
            return;
        }
        out.push_back(RobotToJson(robot, type, side));
    };
    if (league_3v3) {
        for (const auto type : kLeague3v3Units) {
            append_unit(type);
        }
    } else {
        for (const auto type : kTraceUnits) {
            append_unit(type);
        }
    }
    return out;
}

json UnitInfoToJson(const gimbal_driver::msg::UnitInfoArray& info) {
    json out = json::array();
    for (const auto& unit : info.units) {
        out.push_back({
            {"car_id", static_cast<int>(unit.car_id)},
            {"hp", static_cast<int>(unit.hp)},
            {"has_hp", unit.has_hp},
            {"hp_fresh", unit.hp_fresh},
            {"position_cm", {
                {"x", static_cast<int>(unit.position_x)},
                {"y", static_cast<int>(unit.position_y)},
            }},
            {"has_position", unit.has_position},
            {"position_fresh", unit.position_fresh},
            {"position_source", unit.position_source},
            {"area_id", static_cast<int>(unit.area_id)},
            {"area_name", unit.area_name},
            {"area_used_nearest_fallback", unit.area_used_nearest_fallback},
        });
    }
    return out;
}

json RfidMatchToJson(const RfidMatchState& state) {
    return {
        {"fresh", state.Fresh},
        {"any", state.Fresh && state.Any},
        {"raw", state.Raw},
        {"has_rfid_status_2", state.HasRfidStatus2},
        {"rfid_status_2_raw", static_cast<int>(state.RfidStatus2Raw)},
        {"self_base_gain_point", state.Fresh && state.SelfBaseGainPoint},
        {"self_supply", state.Fresh && state.SelfSupply},
        {"self_non_resource_supply", state.Fresh && state.SelfNonResourceSupply},
        {"self_resource_supply", state.Fresh && state.SelfResourceSupply},
        {"self_highland_gain_point", state.Fresh && state.SelfHighlandGainPoint},
        {"enemy_highland_gain_point", state.Fresh && state.EnemyHighlandGainPoint},
        {"self_road_crossing", state.Fresh && state.SelfRoadCrossing},
        {"enemy_road_crossing", state.Fresh && state.EnemyRoadCrossing},
        {"self_central_highland_crossing", state.Fresh && state.SelfCentralHighlandCrossing},
        {"enemy_central_highland_crossing", state.Fresh && state.EnemyCentralHighlandCrossing},
        {"self_tunnel", state.Fresh && state.SelfTunnel},
        {"enemy_tunnel", state.Fresh && state.EnemyTunnel},
        {"tunnel", state.Fresh && state.Tunnel},
        {"center_gain_point", state.Fresh && state.CenterGainPoint},
        {"self_fortress_gain_point", state.Fresh && state.SelfFortressGainPoint},
        {"enemy_fortress_gain_point", state.Fresh && state.EnemyFortressGainPoint},
        {"self_outpost_gain_point", state.Fresh && state.SelfOutpostGainPoint},
        {"enemy_outpost_gain_point", state.Fresh && state.EnemyOutpostGainPoint},
        {"self_assembly_gain_point", state.Fresh && state.SelfAssemblyGainPoint},
        {"enemy_assembly_gain_point", state.Fresh && state.EnemyAssemblyGainPoint},
        {"self_fly_ramp", state.Fresh && state.SelfFlyRamp},
        {"enemy_fly_ramp", state.Fresh && state.EnemyFlyRamp},
        {"on_self_side", state.Fresh && state.OnSelfSideRfid},
        {"on_enemy_side", state.Fresh && state.OnEnemySideRfid},
    };
}

json PostureValueJson(const std::uint8_t value) {
    const auto posture = ToPosture(value);
    return {
        {"id", static_cast<int>(value)},
        {"name", PostureToString(posture)},
    };
}

json PostureEnumJson(const SentryPosture posture) {
    return {
        {"id", static_cast<int>(ToPostureValue(posture))},
        {"name", PostureToString(posture)},
    };
}

const char* ControlOutputSnapshotSourceToString(
    const ControlOutputSnapshotSource source) noexcept {
    switch (source) {
        case ControlOutputSnapshotSource::Normal: return "normal";
        case ControlOutputSnapshotSource::SafeControl: return "safe_control";
        default: return "unknown";
    }
}

const char* ControlTrajectoryUnavailableReasonToString(
    const ControlTrajectoryUnavailableReason reason) noexcept {
    switch (reason) {
        case ControlTrajectoryUnavailableReason::None: return "";
        case ControlTrajectoryUnavailableReason::NotRecorded: return "not_recorded";
        case ControlTrajectoryUnavailableReason::PublisherUnavailable: return "publisher_unavailable";
        case ControlTrajectoryUnavailableReason::InvalidDynamics: return "invalid_dynamics";
        case ControlTrajectoryUnavailableReason::SafeControl: return "safe_control";
        default: return "unknown";
    }
}

json TraceFireCodeToJson(const TraceFireCodeSnapshot& fire_code, const bool available) {
    if (!available) {
        return {
            {"field_mask", nullptr},
            {"raw", nullptr},
            {"fire_status", nullptr},
            {"cap_state", nullptr},
            {"follow_mode", nullptr},
            {"aim_mode", nullptr},
            {"rotate", nullptr},
        };
    }
    return {
        {"field_mask", static_cast<int>(fire_code.FieldMask)},
        {"raw", static_cast<int>(fire_code.Raw)},
        {"fire_status", static_cast<int>(fire_code.FireStatus)},
        {"cap_state", static_cast<int>(fire_code.CapState)},
        {"follow_mode", fire_code.FollowMode},
        {"aim_mode", fire_code.AimMode},
        {"rotate", static_cast<int>(fire_code.Rotate)},
    };
}

json ControlOutputTraceToJson(
    const ControlOutputTraceSnapshot& snapshot,
    const std::chrono::steady_clock::time_point now) {
    const bool snapshot_available = snapshot.Available;
    const bool angles_published = snapshot_available && snapshot.Angles.Published;
    const bool fire_code_published = snapshot_available && snapshot.FireCode.Published;
    const bool trajectory_published = snapshot_available && snapshot.Trajectory.Published;
    const bool trajectory_available = snapshot_available && snapshot.Trajectory.Available;
    return {
        {"available", snapshot_available},
        {"sequence", snapshot_available ? json(snapshot.Sequence) : json(nullptr)},
        {"age_ms", AgeMsOrNull(now, snapshot.PublishedAt, snapshot_available)},
        {"source", snapshot_available
            ? ControlOutputSnapshotSourceToString(snapshot.Source)
            : "not_recorded"},
        {"angles", {
            {"published", angles_published},
            {"yaw", angles_published ? FiniteFloat(snapshot.Angles.Yaw) : json(nullptr)},
            {"pitch", angles_published ? FiniteFloat(snapshot.Angles.Pitch) : json(nullptr)},
        }},
        {"fire_code", {
            {"published", fire_code_published},
            {"field_mask", fire_code_published
                ? json(static_cast<int>(snapshot.FireCode.FireCode.FieldMask)) : json(nullptr)},
            {"raw", fire_code_published
                ? json(static_cast<int>(snapshot.FireCode.FireCode.Raw)) : json(nullptr)},
            {"fire_status", fire_code_published
                ? json(static_cast<int>(snapshot.FireCode.FireCode.FireStatus)) : json(nullptr)},
            {"cap_state", fire_code_published
                ? json(static_cast<int>(snapshot.FireCode.FireCode.CapState)) : json(nullptr)},
            {"follow_mode", fire_code_published
                ? json(snapshot.FireCode.FireCode.FollowMode) : json(nullptr)},
            {"aim_mode", fire_code_published
                ? json(snapshot.FireCode.FireCode.AimMode) : json(nullptr)},
            {"rotate", fire_code_published
                ? json(static_cast<int>(snapshot.FireCode.FireCode.Rotate)) : json(nullptr)},
        }},
        {"trajectory", {
            {"published", trajectory_published},
            {"available", trajectory_available},
            {"unavailable_reason", ControlTrajectoryUnavailableReasonToString(
                snapshot_available
                    ? snapshot.Trajectory.UnavailableReason
                    : ControlTrajectoryUnavailableReason::NotRecorded)},
            {"yaw", trajectory_available ? FiniteFloat(snapshot.Trajectory.Yaw) : json(nullptr)},
            {"pitch", trajectory_available ? FiniteFloat(snapshot.Trajectory.Pitch) : json(nullptr)},
            {"yaw_omega", trajectory_available ? FiniteFloat(snapshot.Trajectory.YawOmega) : json(nullptr)},
            {"pitch_omega", trajectory_available ? FiniteFloat(snapshot.Trajectory.PitchOmega) : json(nullptr)},
            {"yaw_alpha", trajectory_available ? FiniteFloat(snapshot.Trajectory.YawAlpha) : json(nullptr)},
            {"pitch_alpha", trajectory_available ? FiniteFloat(snapshot.Trajectory.PitchAlpha) : json(nullptr)},
        }},
    };
}

}  // namespace

void Application::CaptureGimbalFeedbackTraceSnapshot(
    const gimbal_driver::msg::FireCode& message,
    const std::chrono::steady_clock::time_point received_at) noexcept {
    try {
        std::lock_guard<std::mutex> lock(decisionTraceSnapshotMutex_);
        gimbalFeedbackTraceSnapshot_.Available = true;
        gimbalFeedbackTraceSnapshot_.FireCode = MakeTraceFireCodeSnapshot(message);
        gimbalFeedbackTraceSnapshot_.ReceivedAt = received_at;
    } catch (...) {
    }
}

void Application::CaptureControlOutputTraceSnapshot(
    const gimbal_driver::msg::GimbalAngles& angles,
    const bool angles_published,
    const gimbal_driver::msg::FireCode& fire_code,
    const bool fire_code_published,
    const std::optional<gimbal_driver::msg::GimbalTrajectory>& trajectory,
    const bool trajectory_published,
    const ControlTrajectoryUnavailableReason trajectory_unavailable_reason,
    const ControlOutputSnapshotSource source,
    const std::chrono::steady_clock::time_point published_at) noexcept {
    try {
        std::lock_guard<std::mutex> lock(decisionTraceSnapshotMutex_);
        const auto next_sequence = controlOutputTraceSequence_ + 1;
        const auto snapshot = MakeControlOutputTraceSnapshot(
            next_sequence,
            published_at,
            source,
            angles,
            angles_published,
            fire_code,
            fire_code_published,
            trajectory,
            trajectory_published,
            trajectory_unavailable_reason);
        if (!snapshot.Available) {
            return;
        }
        controlOutputTraceSequence_ = next_sequence;
        controlOutputTraceSnapshot_ = snapshot;
    } catch (...) {
    }
}

bool Application::InitDecisionTrace() {
    if (!decisionTraceRequested_) {
        return false;
    }
    if (decisionTraceFile_.empty()) {
        if (LoggerPtr) {
            LoggerPtr->Warning("Decision trace requested but decision_trace_file is empty; trace stays disabled.");
        }
        return false;
    }

    try {
        std::filesystem::path trace_path(decisionTraceFile_);
        if (!trace_path.is_absolute()) {
            trace_path = std::filesystem::current_path() / trace_path;
        }
        trace_path = trace_path.lexically_normal();

        const auto parent = trace_path.parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent);
        }

        decisionTraceStream_.open(trace_path, std::ios::out | std::ios::trunc);
        if (!decisionTraceStream_.is_open()) {
            decisionTraceEnabled_ = false;
            if (LoggerPtr) {
                LoggerPtr->Warning("Decision trace disabled: cannot open {}", trace_path.string());
            }
            return false;
        }

        decisionTraceFile_ = trace_path.string();
        decisionTraceEnabled_ = true;
        decisionTraceTickCount_ = 0;
        decisionTraceWriteCount_ = 0;
        if (LoggerPtr) {
            LoggerPtr->Info("Decision trace enabled: {}", decisionTraceFile_);
        }
        return true;
    } catch (const std::exception& ex) {
        decisionTraceEnabled_ = false;
        if (LoggerPtr) {
            LoggerPtr->Warning("Decision trace disabled: {}", ex.what());
        }
        return false;
    }
}

void Application::CloseDecisionTrace() {
    if (decisionTraceStream_.is_open()) {
        decisionTraceStream_.flush();
        decisionTraceStream_.close();
    }
    decisionTraceEnabled_ = false;
}

void Application::WriteDecisionTrace(const std::string_view event) noexcept {
    if (!decisionTraceEnabled_ || !decisionTraceStream_.is_open()) {
        return;
    }

    try {

    const bool is_tick_event = event == "tick";
    std::uint64_t trace_tick = decisionTraceTickCount_;
    if (is_tick_event) {
        trace_tick = decisionTraceTickCount_++;
        if (decisionTraceEveryTicks_ > 1 &&
            (trace_tick % static_cast<std::uint64_t>(decisionTraceEveryTicks_)) != 0) {
            return;
        }
    }

    const auto now = std::chrono::steady_clock::now();
    const auto wall_now = std::chrono::system_clock::now();
    const double elapsed_sec = std::chrono::duration<double>(now - gameStartTime).count();
    const auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        wall_now.time_since_epoch()).count();
    GimbalFeedbackTraceSnapshot gimbal_feedback_snapshot;
    ControlOutputTraceSnapshot control_output_snapshot;
    {
        std::lock_guard<std::mutex> lock(decisionTraceSnapshotMutex_);
        gimbal_feedback_snapshot = gimbalFeedbackTraceSnapshot_;
        control_output_snapshot = controlOutputTraceSnapshot_;
    }

    const int goal_base_id = GoalBaseId(naviCommandGoal);
    const auto current_goal_reach = EvaluateNaviGoalReach(
        naviCommandGoal,
        naviGoalPosition,
        std::max(1, config.DecisionAutonomySettings.NaviGoal.HighlandCompatArriveDistanceCm),
        0,
        static_cast<std::uint8_t>(goal_base_id),
        GoalReachTimeoutSecForBaseGoal(static_cast<std::uint8_t>(goal_base_id)));
    const json reach_distance_cm =
        current_goal_reach.DistanceCm >= 0.0 && std::isfinite(current_goal_reach.DistanceCm)
            ? json(current_goal_reach.DistanceCm)
            : json(nullptr);
    const auto posture_runtime = postureManager_.Runtime();
    const bool navi_should_rotate_fresh =
        hasReceivedNaviIsRotate_ &&
        lastNaviIsRotateRxTime_.time_since_epoch().count() != 0 &&
        now - lastNaviIsRotateRxTime_ <=
            std::chrono::milliseconds(config.NaviControlSettings.FreshTimeoutMs);
    const bool effective_navi_should_rotate =
        navi_should_rotate_fresh
            ? naviIsRotate
            : config.NaviControlSettings.DefaultIsRotate;
    const bool enable_chase_to_navi =
        chaseTacticalAllowed_ &&
        config.ChaseSettings.Enable && config.ChaseSettings.ToNavi;
    const bool uses_chase_official_goal = enable_chase_to_navi && naviChaseOfficialTargetValid;
    const bool uses_chase_relative_target =
        enable_chase_to_navi &&
        !uses_chase_official_goal &&
        (naviRelativeTargetValid || config.ChaseSettings.StopWhenNoTarget);
    const bool uses_goal_pos = uses_chase_official_goal ||
        (config.NaviSettings.UseXY && !uses_chase_relative_target);
    const bool uses_goal_pos_bridge = uses_goal_pos && config.NaviSettings.ToNavi;
    const bool uses_any_to_navi = uses_chase_relative_target || uses_goal_pos_bridge;
    const char* output_kind = "goal_id";
    const char* output_topic = ly_navi_goal::Name;
    const char* final_goal_pos_topic = "";
    if (uses_chase_official_goal) {
        output_kind = config.NaviSettings.ToNavi ? "chase_goal_pos_raw_bridge" : "chase_goal_pos";
        output_topic = config.NaviSettings.ToNavi ? ly_navi_goal_pos_raw::Name : ly_navi_goal_pos::Name;
        final_goal_pos_topic = config.NaviSettings.ToNavi ? "/goal_pose" : ly_navi_goal_pos::Name;
    } else if (uses_chase_relative_target) {
        output_kind = "relative_target_bridge";
        output_topic = ly_navi_target_rel::Name;
        final_goal_pos_topic = "/goal_pose";
    } else if (uses_goal_pos) {
        output_kind = uses_goal_pos_bridge ? "goal_pos_raw_bridge" : "goal_pos";
        output_topic = uses_goal_pos_bridge ? ly_navi_goal_pos_raw::Name : ly_navi_goal_pos::Name;
        final_goal_pos_topic = uses_goal_pos_bridge ? "/goal_pose" : ly_navi_goal_pos::Name;
    }

    // Tactical trace evidence is intentionally captured in BT from the same
    // policy helpers that make the decision. The simulator consumes these
    // facts as observations and never reconstructs ProtectCastle from UI data.
    const auto& tactical_settings = config.TacticalSettings;
    const auto tactical_now = std::chrono::steady_clock::now();
    const int tactical_referee_fresh_ms = std::max(
        std::max(0, config.TaskSettings.BuffConfirm.RefereeFreshTimeoutMs),
        std::max(0, config.TaskSettings.OutpostConfirm.RefereeFreshTimeoutMs));
    const bool protect_castle_rfid_raw_active =
        IsFortressGainPointEnemyOccupiedEventRawFresh(tactical_referee_fresh_ms);
    const bool protect_castle_rfid_stay_active =
        IsProtectCastleRfidStayActive(tactical_referee_fresh_ms);
    const bool protect_castle_rfid_event_active =
        IsFortressGainPointEnemyOccupiedEventFresh(tactical_referee_fresh_ms);
    const bool protect_castle_base_damage_active =
        IsProtectCastleBaseDamageActive(tactical_now);
    const auto castle_occupancy = ResolveProtectCastleOccupancy(tactical_now);
    const bool self_base_hp_fresh = hasReceivedSelfBaseHealth_ &&
        lastSelfBaseHealthRxTime_.time_since_epoch().count() != 0 &&
        tactical_now - lastSelfBaseHealthRxTime_ <= std::chrono::milliseconds(2000);
    const bool self_outpost_hp_fresh = hasReceivedSelfOutpostHealth_ &&
        lastSelfOutpostHealthRxTime_.time_since_epoch().count() != 0 &&
        tactical_now - lastSelfOutpostHealthRxTime_ <=
            std::chrono::milliseconds(std::max(0, tactical_settings.ProtectOutpost.HealthFreshMs));
    const bool self_outpost_destroyed = self_outpost_hp_fresh && selfOutpostHealth == 0;
    const UnitTeam tactical_enemy_team =
        team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
    const auto tactical_regional_threat =
        EvaluateRegionalDefenseThreat(team, tactical_enemy_team);
    const int tactical_own_base_enemy_count = tactical_regional_threat.has_value()
        ? tactical_regional_threat->OwnBaseCount
        : 0;
    const int tactical_own_highland_enemy_count = tactical_regional_threat.has_value()
        ? tactical_regional_threat->OwnHighlandCount
        : 0;
    const bool protect_castle_enemy_pos_active =
        tactical_settings.ProtectCastle.Enable &&
        tactical_settings.ProtectCastle.EnemyPos &&
        tactical_own_base_enemy_count > 0;
    const auto& protect_hero = tactical_settings.ProtectHero;
    const auto hero_position = GetFriendPositionState(
        UnitType::Hero,
        protect_hero.FriendPositionFreshMs,
        tactical_now);
    const bool protect_hero_position_valid = hero_position.Fresh &&
        hero_position.X > 0 && hero_position.Y > 0 &&
        hero_position.X <= kTraceFieldWidthCm && hero_position.Y <= kTraceFieldHeightCm;
    const bool protect_hero_in_highland = protect_hero_position_valid &&
        Area::IsPointInsideMainArea(
            team,
            Area::MainAreaKind::Highland,
            hero_position.X,
            hero_position.Y);
    const bool protect_hero_in_area = protect_hero_position_valid &&
        Area::IsPointInsideProtectHeroArea(team, hero_position.X, hero_position.Y);
    const bool protect_hero_health_fresh =
        IsFriendHealthFresh(UnitType::Hero, protect_hero.FriendHealthFreshMs);
    const bool protect_hero_known_dead = protect_hero_health_fresh &&
        friendRobots[UnitType::Hero].currentHealth_ == 0;
    const bool protect_hero_threat_ready = tactical_own_base_enemy_count > 0 &&
        tactical_own_highland_enemy_count > 0;
    const bool protect_hero_mode_gate_ready = BehaviorTree::ShouldHoldProtectHero(
        protect_hero.ProactiveHoldWhenHeroInHighland,
        protect_hero_threat_ready);
    const bool protect_hero_profile_ready =
        !IsLeagueProfile() && !IsShowcasePatrolEnabled();
    const bool protect_hero_elapsed_ready =
        ElapsedSeconds() >= protect_hero.StartElapsedSec;

    json record;
    record["schema"] = "ly_decision_trace_v1";
    record["schema_version"] = 4;
    record["event"] = std::string(event);
    record["tick"] = trace_tick;
    record["t"] = elapsed_sec;
    record["elapsed_sec"] = ElapsedSeconds();
    record["wall_time_ms"] = wall_ms;

    const bool league_3v3_trace = config.LeagueStrategySettings.League3v3.Enable;
    record["field_cm"] = {
        {"width", league_3v3_trace ? kLeague3v3TraceFieldWidthCm : kTraceFieldWidthCm},
        {"height", league_3v3_trace ? kLeague3v3TraceFieldHeightCm : kTraceFieldHeightCm},
        {"frame", "left_bottom_origin_cm"},
    };
    record["competition_profile"] = CompetitionProfileToString(competitionProfile_);
    record["strategy_mode"] = StrategyModeToString(strategyMode_);
    record["strategy_mode_id"] = static_cast<int>(strategyMode_);
    record["team"] = UnitTeamToString(team);
    record["team_id"] = static_cast<int>(team);

    record["aim_mode"] = AimModeToString(aimMode);
    record["aim_mode_id"] = static_cast<int>(aimMode);
    record["aim_enable"] = {
        {"auto_aim", aimMode == AimMode::AutoAim || aimMode == AimMode::RotateScan},
        {"buff", aimMode == AimMode::Buff},
        {"outpost", aimMode == AimMode::Outpost},
    };
    record["target_armor"] = {
        {"id", static_cast<int>(targetArmor.Type)},
        {"name", ArmorTypeToString(targetArmor.Type)},
        {"distance_m", FiniteFloat(targetArmor.Distance)},
    };
    const auto aim_source = CurrentAimSource();
    record["target_state"] = {
        {"has_recent_target", HasRecentTarget()},
        {"external_aim_active", aim_source.ExternalAimActive},
        {"fresh_current_aim", aim_source.Active != nullptr && AimFreshAndValid(*aim_source.Active)},
        {"fresh_auto_aim", aim_source.AutoAim != nullptr && AimFreshAndValid(*aim_source.AutoAim)},
        {"fresh_buff", aim_source.Buff != nullptr &&
            AimBuffTargetLocked(*aim_source.Buff, aim_source.ExternalAimActive)},
        {"fresh_outpost", aim_source.Outpost != nullptr && AimFreshAndValid(*aim_source.Outpost)},
        {"hitable_targets", UnitListToJson(hitableTargets)},
        {"reliable_enemy_positions", UnitListToJson(reliableEnemyPosuition)},
    };
    record["events"] = {
        {"event_data_fresh", eventSnapshot_.EventDataFresh},
        {"sentry_info_fresh", eventSnapshot_.SentryInfoFresh},
        {"buff_task_enabled", eventSnapshot_.BuffTaskEnabled},
        {"buff_can_activate", eventSnapshot_.BuffCanActivate},
        {"buff_activating", eventSnapshot_.BuffActivating},
        {"buff_activated", eventSnapshot_.BuffActivated},
        {"outpost_task_enabled", eventSnapshot_.OutpostTaskEnabled},
        {"enemy_outpost_alive", eventSnapshot_.EnemyOutpostAlive},
        {"outpost_attack_window_open", eventSnapshot_.OutpostAttackWindowOpen},
        {"self_low_hp", eventSnapshot_.SelfLowHp},
        {"self_low_ammo", eventSnapshot_.SelfLowAmmo},
        {"recent_damage_over_30", eventSnapshot_.RecentDamageOver30},
        {"armor_target_visible", eventSnapshot_.ArmorTargetVisible},
        {"buff_target_locked", eventSnapshot_.BuffTargetLocked},
        {"outpost_target_locked", eventSnapshot_.OutpostTargetLocked},
        {"goal_reached", eventSnapshot_.GoalReached},
        {"goal_unreachable", eventSnapshot_.GoalUnreachable},
        {"regional_defense_active", eventSnapshot_.RegionalDefenseActive},
        {"center_gain_point_status", static_cast<int>(eventCenterGainPointStatus_)},
        {"self_fortress_gain_point_status", static_cast<int>(eventSnapshot_.SelfFortressGainPointStatus)},
        {"self_outpost_gain_point_status", static_cast<int>(eventSnapshot_.SelfOutpostGainPointStatus)},
        {"self_base_gain_point_status", eventSnapshot_.SelfBaseGainPointStatus},
    };
    record["tactical"] = {
        {"available", true},
        {"protect_castle", {
            {"enabled", tactical_settings.ProtectCastle.Enable},
            {"priority", tactical_settings.Priority.ProtectCastle},
            {"base_enabled", tactical_settings.ProtectCastle.Base},
            {"base_damage_active", protect_castle_base_damage_active},
            {"base_hp_fresh", self_base_hp_fresh},
            {"base_damage_remaining_ms", selfBaseDamageWindow_.ActiveUntil > tactical_now
                ? std::chrono::duration_cast<std::chrono::milliseconds>(
                    selfBaseDamageWindow_.ActiveUntil - tactical_now).count()
                : 0},
            {"rfid_enabled", tactical_settings.ProtectCastle.RFID},
            {"stay_when_rfid_enabled", tactical_settings.ProtectCastle.StayWhenRfid},
            {"stay_when_rfid_active", protect_castle_rfid_stay_active},
            {"enemy_pos_enabled", tactical_settings.ProtectCastle.EnemyPos},
            {"rfid_event_raw_active", protect_castle_rfid_raw_active},
            {"rfid_event_active", protect_castle_rfid_event_active},
            {"enemy_pos_active", protect_castle_enemy_pos_active},
            {"referee_fortress_status", static_cast<int>(eventSelfFortressGainPointStatus_)},
            {"referee_fortress_fresh", hasReceivedEventData_ &&
                lastEventDataRxTime_.time_since_epoch().count() != 0 &&
                tactical_now - lastEventDataRxTime_ <= std::chrono::milliseconds(tactical_referee_fresh_ms)},
            {"occupancy_action", CastleOccupancyActionToString(castle_occupancy.Action)},
            {"self_likely_at_castle", castle_occupancy.SelfLikelyAtCastle},
            {"teammate_likely_at_castle", castle_occupancy.TeammateLikelyAtCastle},
            {"team_or_ambiguous_occupant", castle_occupancy.TeamOrAmbiguousOccupant},
        }},
        {"protect_hero", {
            {"enabled", protect_hero.Enable},
            {"proactive_hold_when_hero_in_highland",
             protect_hero.ProactiveHoldWhenHeroInHighland},
            {"enhanced_defense_enabled", protect_hero.EnhancedDefense.Enable},
            {"enhanced_defense_damage_window_ms", protect_hero.EnhancedDefense.DamageWindowMs},
            {"enhanced_defense_damage_threshold_hp", protect_hero.EnhancedDefense.DamageThresholdHp},
            {"enhanced_defense_request_unavailable", protectHeroEnhancedDefenseUnavailable_},
            {"enhanced_defense_recovery_deferred", protectHeroEnhancedDefenseRecoveryDeferred_},
            {"active", protectHeroActive_},
            {"priority", tactical_settings.Priority.ProtectHero},
            {"profile_ready", protect_hero_profile_ready},
            {"elapsed_ready", protect_hero_elapsed_ready},
            {"position_fresh", hero_position.Fresh},
            {"position_valid", protect_hero_position_valid},
            {"hero_x_cm", hero_position.X},
            {"hero_y_cm", hero_position.Y},
            {"in_highland", protect_hero_in_highland},
            {"in_protect_area", protect_hero_in_area},
            {"health_fresh", protect_hero_health_fresh},
            {"known_dead", protect_hero_known_dead},
            {"own_base_enemy_count", tactical_own_base_enemy_count},
            {"own_highland_enemy_count", tactical_own_highland_enemy_count},
            {"threat_ready", protect_hero_threat_ready},
            {"mode_gate_ready", protect_hero_mode_gate_ready},
            {"start_elapsed_sec", protect_hero.StartElapsedSec},
            {"hold_sec", protect_hero.HoldSec},
            {"no_enemy_release_sec", protect_hero.NoEnemyReleaseSec},
            {"position_fresh_ms", protect_hero.FriendPositionFreshMs},
            {"health_fresh_ms", protect_hero.FriendHealthFreshMs},
            {"goal_base_id", static_cast<int>(protect_hero.GoalBaseId)},
        }},
        {"protect_outpost", {
            {"enabled", tactical_settings.ProtectOutpost.Enable},
            {"hp", static_cast<int>(selfOutpostHealth)},
            {"hp_fresh", self_outpost_hp_fresh},
            {"destroyed", self_outpost_destroyed},
            {"phase", ProtectOutpostPhaseToString(protectOutpostState_.Phase)},
            {"event_generation", protectOutpostState_.ActiveEventGeneration},
            {"damage_window_ms", tactical_settings.ProtectOutpost.DamageWindowMs},
            {"damage_threshold_hp", tactical_settings.ProtectOutpost.DamageThresholdHp},
            {"search_hold_sec", tactical_settings.ProtectOutpost.SearchHoldSec},
            {"priority", tactical_settings.Priority.ProtectOutpost},
        }},
        {"regional_defense", {
            {"threat_active", tactical_regional_threat.has_value()},
            {"search_kind", RegionalDefenseSearchKindToString(regionalDefenseSearchKind_)},
            {"common_central_enabled", tactical_settings.RegionalDefense.CommonCentral.Enable},
            {"common_central_hold_sec", tactical_settings.RegionalDefense.CommonCentral.HoldSec},
            {"common_central_priority", tactical_settings.Priority.CommonCentral},
            {"fortress_enemy_count", fortressGainPointEnemyCount_},
            {"own_base_enemy_count", tactical_own_base_enemy_count},
        }},
    };
    record["goal_reach_state"] = {
        {"status", GoalReachStatusToString(current_goal_reach.Status)},
        {"status_id", static_cast<int>(current_goal_reach.Status)},
        {"reason", GoalReachReasonToString(current_goal_reach.Reason)},
        {"reason_id", static_cast<int>(current_goal_reach.Reason)},
        {"goal_id", static_cast<int>(current_goal_reach.GoalId)},
        {"base_goal_id", static_cast<int>(current_goal_reach.BaseGoalId)},
        {"goal_age_ms", current_goal_reach.GoalAgeMs},
        {"external_reach_fresh", current_goal_reach.ExternalReach.has_value()},
        {"external_reach", current_goal_reach.ExternalReach.value_or(false)},
        {"external_reachable_fresh", current_goal_reach.ExternalReachable.has_value()},
        {"external_reachable", current_goal_reach.ExternalReachable.value_or(true)},
        {"position_fresh", current_goal_reach.PositionFresh},
        {"has_position", current_goal_reach.HasPosition},
        {"distance_cm", reach_distance_cm},
        {"arrive_distance_cm", current_goal_reach.ArriveDistanceCm},
        {"face_distance_cm", current_goal_reach.FaceDistanceCm},
        {"distance_fallback_allowed", current_goal_reach.DistanceFallbackAllowed},
        {"within_arrive_distance", current_goal_reach.WithinArriveDistance},
        {"within_face_distance", current_goal_reach.WithinFaceDistance},
        {"near_goal_confirm_pending", current_goal_reach.NearGoalConfirmPending},
        {"near_goal_confirm_elapsed_ms", current_goal_reach.NearGoalConfirmElapsedMs},
        {"timeout", current_goal_reach.Timeout},
    };
    record["units"] = {
        {"friend", RobotsToJson(friendRobots, "friend", league_3v3_trace)},
        {"enemy", RobotsToJson(enemyRobots, "enemy", league_3v3_trace)},
    };
    record["unit_info"] = {
        {"friend", UnitInfoToJson(MakeFriendInfoMsg())},
        {"enemy", UnitInfoToJson(MakeEnemyInfoMsg())},
    };

    record["decision_output"] = {
        {"kind", output_kind},
        {"source", "behavior_tree"},
        {"publish_allowed", naviGoalPublishAllowed_},
        {"publish_enabled", publishNaviGoal_},
        {"uses_goal_pos", uses_goal_pos},
        {"uses_to_navi", uses_any_to_navi},
        {"relative_target_valid", naviRelativeTargetValid},
        {"chase_official_target_valid", naviChaseOfficialTargetValid},
        {"chase_official_armor_type", static_cast<int>(naviChaseOfficialTargetArmorType)},
        {"output_topic", output_topic},
        {"output_frame", "map"},
        {"final_goal_pos_topic", final_goal_pos_topic},
        {"goal_id", static_cast<int>(naviCommandGoal)},
        {"goal_base_id", goal_base_id},
        {"goal_name", GoalName(goal_base_id, config.LeagueStrategySettings.League3v3.Enable)},
        {"goal_side", GoalSide(naviCommandGoal)},
        {"speed_level", static_cast<int>(speedLevel)},
        {"goal_pos_cm", {
            {"x", static_cast<int>(naviGoalPosition.x)},
            {"y", static_cast<int>(naviGoalPosition.y)},
        }},
    };
    record["decision_intent"] = {
        {"layer", DecisionLayerToString(lastDecisionIntent_.Layer)},
        {"reason", DecisionReasonToString(lastDecisionIntent_.Reason)},
        {"base_goal_id", static_cast<int>(lastDecisionIntent_.BaseGoalId)},
        {"resolved_goal_id", static_cast<int>(lastDecisionIntent_.ResolvedGoalId)},
        {"goal_team", UnitTeamToString(lastDecisionIntent_.GoalTeam)},
        {"apply_team_offset", lastDecisionIntent_.ApplyTeamOffset},
        {"priority", lastDecisionIntent_.Priority},
        {"detail", lastDecisionIntent_.Detail},
    };

    record["navi_goal"] = {
        {"id", static_cast<int>(naviCommandGoal)},
        {"base_id", goal_base_id},
        {"name", GoalName(goal_base_id, config.LeagueStrategySettings.League3v3.Enable)},
        {"side", GoalSide(naviCommandGoal)},
        {"publish_allowed", naviGoalPublishAllowed_},
        {"publish_enabled", publishNaviGoal_},
        {"speed_level", static_cast<int>(speedLevel)},
        {"position_cm", {
            {"x", static_cast<int>(naviGoalPosition.x)},
            {"y", static_cast<int>(naviGoalPosition.y)},
        }},
    };
    record["navi_velocity"] = {
        {"input_x", static_cast<int>(naviVelocityInput.X)},
        {"input_y", static_cast<int>(naviVelocityInput.Y)},
        {"output_x", static_cast<int>(naviVelocity.X)},
        {"output_y", static_cast<int>(naviVelocity.Y)},
        {"raw_to_mps", kTraceVelocityRawToMps},
    };
    record["navi_status"] = {
        {"should_rotate", effective_navi_should_rotate},
        {"should_rotate_fresh", navi_should_rotate_fresh},
        {"reached", naviReach},
        {"reached_fresh", current_goal_reach.ExternalReach.has_value()},
        {"reachable", naviReachable},
        {"reachable_fresh", current_goal_reach.ExternalReachable.has_value()},
    };
    const auto face_mode_source = [](const FaceModeManager::Source source) {
        switch (source) {
            case FaceModeManager::Source::Regional: return "regional";
            case FaceModeManager::Source::Buff: return "buff";
            case FaceModeManager::Source::Outpost: return "outpost";
            case FaceModeManager::Source::StartGate: return "start_gate";
            default: return "none";
        }
    };
    record["face_mode"] = {
        {"requested", lastFaceModeDecision_.Requested},
        {"active", lastFaceModeDecision_.Active},
        {"patrol_fallback", lastFaceModeDecision_.UsePatrolFallback},
        {"suppress_fire", lastFaceModeDecision_.SuppressFire},
        {"source", face_mode_source(lastFaceModeDecision_.RequestSource)},
        {"phase", RegionalAreaTaskPhaseToString(lastFaceModeDecision_.Phase)},
        {"has_angles", lastFaceModeDecision_.Angles.has_value()},
        {"yaw", lastFaceModeDecision_.Angles.has_value()
            ? json(lastFaceModeDecision_.Angles->Yaw) : json(nullptr)},
        {"pitch", lastFaceModeDecision_.Angles.has_value()
            ? json(lastFaceModeDecision_.Angles->Pitch) : json(nullptr)},
    };
    record["navi_relative_target"] = {
        {"valid", naviRelativeTargetValid},
        {"frame_id", naviRelativeTargetFrameId},
        {"x", FiniteFloat(naviRelativeTargetX)},
        {"y", FiniteFloat(naviRelativeTargetY)},
        {"z", FiniteFloat(naviRelativeTargetZ)},
        {"distance", FiniteFloat(naviRelativeTargetDistance)},
        {"yaw_error_deg", FiniteFloat(naviRelativeTargetYawErrorDeg)},
        {"pitch_error_deg", FiniteFloat(naviRelativeTargetPitchErrorDeg)},
        {"armor_type", static_cast<int>(naviRelativeTargetArmorType)},
        {"aim_mode", static_cast<int>(naviRelativeTargetAimMode)},
        {"official_target_valid", naviChaseOfficialTargetValid},
        {"official_armor_type", static_cast<int>(naviChaseOfficialTargetArmorType)},
    };

    record["posture"] = {
        {"command", PostureValueJson(postureCommand)},
        {"state", PostureValueJson(postureState)},
        {"last_desired", PostureEnumJson(postureLastDesired_)},
        {"last_reason", postureLastReason_},
        {"task_intent", TaskPostureIntentToString(postureTaskIntent_.Intent)},
        {"task_source", postureTaskIntent_.Source},
        {"task_owns_current_goal", postureTaskIntent_.OwnsCurrentGoal},
        {"under_fire_recent", IsUnderFireRecent()},
        {"under_fire_burst", IsUnderFireBurst()},
        {"runtime", {
            {"current", PostureEnumJson(posture_runtime.Current.Base)},
            {"desired", PostureEnumJson(posture_runtime.Desired.Base)},
            {"pending", PostureEnumJson(posture_runtime.Pending.Base)},
            {"has_pending", posture_runtime.HasPending},
            {"feedback_stale", posture_runtime.FeedbackStale},
            {"retry_count", posture_runtime.RetryCount},
            {"referee_timer_fresh", posture_runtime.RefereeTimerFresh},
            {"referee_enhanced_posture", posture_runtime.RefereeEnhancedPosture},
            {"using_referee_timer", posture_runtime.UsingRefereeTimer},
            {"referee_remain_warn_sec", config.PostureSettings.RefereeRemainWarnSec},
            {"referee_remain_penalty", config.PostureSettings.RefereeRemainPenalty},
            {"referee_zero_remain_penalty", config.PostureSettings.RefereeZeroRemainPenalty},
            {"enhanced_current_posture_bonus", config.PostureSettings.EnhancedCurrentPostureBonus},
            {"accum_sec", {
                {"attack", posture_runtime.AccumSec[static_cast<int>(SentryPosture::Attack)]},
                {"defense", posture_runtime.AccumSec[static_cast<int>(SentryPosture::Defense)]},
                {"move", posture_runtime.AccumSec[static_cast<int>(SentryPosture::Move)]},
            }},
            {"degraded", {
                {"attack", posture_runtime.Degraded[static_cast<int>(SentryPosture::Attack)]},
                {"defense", posture_runtime.Degraded[static_cast<int>(SentryPosture::Defense)]},
                {"move", posture_runtime.Degraded[static_cast<int>(SentryPosture::Move)]},
            }},
            {"referee_remaining_sec", {
                {"attack", posture_runtime.RefereeRemainingSec[static_cast<int>(SentryPosture::Attack)]},
                {"defense", posture_runtime.RefereeRemainingSec[static_cast<int>(SentryPosture::Defense)]},
                {"move", posture_runtime.RefereeRemainingSec[static_cast<int>(SentryPosture::Move)]},
            }},
            {"referee_enhanced_remaining_sec", {
                {"attack", posture_runtime.RefereeEnhancedRemainingSec[static_cast<int>(SentryPosture::Attack)]},
                {"defense", posture_runtime.RefereeEnhancedRemainingSec[static_cast<int>(SentryPosture::Defense)]},
                {"move", posture_runtime.RefereeEnhancedRemainingSec[static_cast<int>(SentryPosture::Move)]},
            }},
        }},
    };
    record["outpost_engagement_lock"] = {
        {"active", outpostEngagementDecision_.Active},
        {"hold_target", outpostEngagementDecision_.HoldTarget},
        {"enhanced_armed", outpostEngagementDecision_.EnhancedArmed},
        {"enhanced_pending", outpostEngagementDecision_.EnhancedPending},
        {"enhanced_active", outpostEngagementDecision_.EnhancedActive},
        {"enhanced_unavailable", outpostEngagementDecision_.EnhancedUnavailable},
        {"exit_reason", static_cast<int>(outpostEngagementDecision_.ExitReason)},
        {"normal_exit_hp", config.TaskSettings.OutpostConfirm.NormalAttackLockExitHp},
        {"enhanced_exit_hp", config.TaskSettings.OutpostConfirm.EnhancedAttackLockExitHp},
    };

    record["referee"] = {
        {"self_hp", static_cast<int>(myselfHealth)},
        {"self_outpost_hp", static_cast<int>(selfOutpostHealth)},
        {"enemy_outpost_hp", static_cast<int>(enemyOutpostHealth)},
        {"self_base_hp", static_cast<int>(selfBaseHealth)},
        {"self_base_hp_fresh", self_base_hp_fresh},
        {"enemy_base_hp", static_cast<int>(enemyBaseHealth)},
        {"ammo", static_cast<int>(ammoLeft)},
        {"time_left", static_cast<int>(timeLeft)},
        {"is_game_begin", is_game_begin},
        {"has_self_hp", hasReceivedMyselfHealth_},
        {"has_ammo", hasReceivedAmmoLeft_},
        {"has_game_start", hasReceivedGameStartFlag_},
        {"rfid_status", rfidStatus},
        {"has_rfid_status_2", hasRfidStatus2},
        {"rfid_status_2", static_cast<int>(rfidStatus2)},
        {"rfid_match", RfidMatchToJson(rfidMatchState)},
        {"ext_event_data", extEventData},
        {"has_event_data", hasReceivedEventData_},
        {"event_center_gain_point_status", static_cast<int>(eventCenterGainPointStatus_)},
        {"event_self_small_energy_status", static_cast<int>(eventSelfSmallEnergyStatus_)},
        {"event_self_large_energy_status", static_cast<int>(eventSelfLargeEnergyStatus_)},
        {"event_self_fortress_gain_point_status", static_cast<int>(eventSelfFortressGainPointStatus_)},
        {"event_self_outpost_gain_point_status", static_cast<int>(eventSelfOutpostGainPointStatus_)},
        {"event_self_base_gain_point_status", eventSelfBaseGainPointStatus_},
        {"has_sentry_info", hasReceivedSentryInfo_},
        {"sentry_can_activate_energy", sentryCanActivateEnergyMechanism_},
        {"energy_activate_confirm_pulse", energyActivateConfirmPulseActive_},
        {"buff_task_locked", buffTaskLocked_},
        {"team_buff", {
            {"recovery", static_cast<int>(teamBuff.RecoveryBuff)},
            {"cooling", static_cast<int>(teamBuff.CoolingBuff)},
            {"defence", static_cast<int>(teamBuff.DefenceBuff)},
            {"vulnerability", static_cast<int>(teamBuff.VulnerabilityBuff)},
            {"attack", static_cast<int>(teamBuff.AttackBuff)},
            {"remaining_energy", static_cast<int>(teamBuff.RemainingEnergy)},
        }},
    };

    record["gimbal"] = {
        {"yaw_deg", FiniteFloat(gimbalAngles.Yaw)},
        {"pitch_deg", FiniteFloat(gimbalAngles.Pitch)},
        {"yaw_vel_raw", static_cast<int>(gimbalYawVelRaw)},
        {"yaw_vel_deg_per_sec", FiniteFloat(gimbalYawVelDegPerSec)},
        {"yaw_angle_raw", static_cast<int>(gimbalYawAngleRaw)},
        {"yaw_angle_deg", FiniteFloat(gimbalYawAngleDeg)},
        {"cap_v", static_cast<int>(capV)},
        {"navi_lower_head", static_cast<int>(naviLowerHead)},
        {"fire_code", {
            {"fire_status", static_cast<int>(RecFireCode.FireStatus)},
            {"cap_state", static_cast<int>(RecFireCode.CapState)},
            {"follow_mode", static_cast<int>(RecFireCode.FollowMode)},
            {"aim_mode", static_cast<int>(RecFireCode.AimMode)},
            {"rotate", static_cast<int>(RecFireCode.Rotate)},
        }},
    };
    record["gimbal_feedback"] = {
        {"available", gimbal_feedback_snapshot.Available},
        {"age_ms", AgeMsOrNull(
            now,
            gimbal_feedback_snapshot.ReceivedAt,
            gimbal_feedback_snapshot.Available)},
        {"fire_code", TraceFireCodeToJson(
            gimbal_feedback_snapshot.FireCode,
            gimbal_feedback_snapshot.Available)},
    };
    record["control_output"] = ControlOutputTraceToJson(control_output_snapshot, now);

    record["bullet_info"] = {
        {"has_received", hasReceivedBulletInfo_},
        {"age_ms", AgeMsOrNull(now, lastBulletInfoRxTime_, hasReceivedBulletInfo_)},
        {"has_initial_speed", bulletInfo.has_initial_speed},
        {"initial_speed", bulletInfo.has_initial_speed ? FiniteFloat(bulletInfo.initial_speed) : json(nullptr)},
        {"has_shoot_data", bulletInfo.has_shoot_data},
        {"bullet_type", static_cast<int>(bulletInfo.bullet_type)},
        {"shooter_number", static_cast<int>(bulletInfo.shooter_number)},
        {"launching_frequency", static_cast<int>(bulletInfo.launching_frequency)},
        {"has_projectile_allowance", bulletInfo.has_projectile_allowance},
        {"projectile_allowance_17mm", static_cast<int>(bulletInfo.projectile_allowance_17mm)},
        {"projectile_allowance_42mm", static_cast<int>(bulletInfo.projectile_allowance_42mm)},
        {"remaining_gold_coin", static_cast<int>(bulletInfo.remaining_gold_coin)},
        {"projectile_allowance_fortress_17mm", static_cast<int>(bulletInfo.projectile_allowance_fortress_17mm)},
    };

    record["runtime_guard"] = {
        {"fault", RuntimeFaultCodeToString(runtimeFaultCode_.load(std::memory_order_relaxed))},
        {"recovery_requested", runtimeRecoveryRequested_.load(std::memory_order_relaxed)},
        {"recovering", runtimeRecovering_},
    };

    decisionTraceStream_ << record.dump() << '\n';
    ++decisionTraceWriteCount_;
    if (!is_tick_event || (decisionTraceWriteCount_ % 20) == 0) {
        decisionTraceStream_.flush();
    }
    if (!decisionTraceStream_) {
        throw std::runtime_error("trace write failed");
    }
    } catch (const std::exception& ex) {
        decisionTraceEnabled_ = false;
        if (decisionTraceStream_.is_open()) {
            decisionTraceStream_.close();
        }
        std::fprintf(stderr, "Decision trace disabled after write failure: %s\n", ex.what());
    } catch (...) {
        decisionTraceEnabled_ = false;
        if (decisionTraceStream_.is_open()) {
            decisionTraceStream_.close();
        }
        std::fprintf(stderr, "Decision trace disabled after an unknown write failure\n");
    }
}

}  // namespace BehaviorTree
