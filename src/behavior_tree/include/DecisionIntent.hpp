// AUTO-COMMENT: file overview
// Typed decision intent metadata for behavior_tree navigation decisions.

#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include "../module/BasicTypes.hpp"

namespace BehaviorTree {

enum class DecisionLayer : std::uint8_t {
    Unknown = 0,
    Hard = 1,
    Default = 2,
    Task = 3,
    Tactical = 4,
    RegionalDefense = 5,
    AimMode = 6,
    Watchdog = 7,
    IdlePatrol = 8,
    Special = 9
};

enum class DecisionReason : std::uint8_t {
    Unknown = 0,
    AreaScopeBlocked = 1,
    DefaultAreaPolicy = 2,
    RegionalDefense = 3,
    OwnBaseMulti = 4,
    OwnBaseChase = 5,
    OwnBaseGuard = 6,
    OwnHighlandRoadCorridor = 7,
    OwnRoadCorridor = 8,
    OwnHighland = 9,
    CommonCentral = 10,
    EnemyReadyRoadlandSoft = 11,
    EnemyHighlandSoft = 12,
    AimModeBuff = 13,
    AimModeOutpost = 14,
    NaviProgressWatchdog = 15,
    RegionalIdlePatrol = 16,
    AreaTask = 17,
    NaviTransition = 18,
    Start = 19,
    OwnFortressGainPointEnemy = 20,
    ProtectHero = 21,
    SpecialPatrol = 22,
    MapCommand = 23,
    Chase = 24,
    Recovery = 25,
    ProtectOutpost = 26,
    League3v3 = 27
};

struct DecisionIntent {
    DecisionLayer Layer{DecisionLayer::Unknown};
    DecisionReason Reason{DecisionReason::Unknown};
    std::uint8_t BaseGoalId{LangYa::Home.ID};
    std::uint8_t ResolvedGoalId{0};
    LangYa::UnitTeam GoalTeam{LangYa::UnitTeam::Unknown};
    bool ApplyTeamOffset{true};
    int Priority{0};
    std::string Detail{};
};

inline const char* DecisionLayerToString(const DecisionLayer layer) noexcept {
    switch (layer) {
        case DecisionLayer::Hard: return "hard";
        case DecisionLayer::Default: return "default";
        case DecisionLayer::Task: return "task";
        case DecisionLayer::Tactical: return "tactical";
        case DecisionLayer::RegionalDefense: return "regional_defense";
        case DecisionLayer::AimMode: return "aim_mode";
        case DecisionLayer::Watchdog: return "watchdog";
        case DecisionLayer::IdlePatrol: return "idle_patrol";
        case DecisionLayer::Special: return "special";
        default: return "unknown";
    }
}

inline const char* DecisionReasonToString(const DecisionReason reason) noexcept {
    switch (reason) {
        case DecisionReason::AreaScopeBlocked: return "area_scope_blocked";
        case DecisionReason::DefaultAreaPolicy: return "default_area_policy";
        case DecisionReason::RegionalDefense: return "regional_defense";
        case DecisionReason::OwnBaseMulti: return "own_base_multi";
        case DecisionReason::OwnBaseChase: return "own_base_chase";
        case DecisionReason::OwnBaseGuard: return "own_base_guard";
        case DecisionReason::OwnHighlandRoadCorridor: return "own_highland_road_corridor";
        case DecisionReason::OwnRoadCorridor: return "own_road_corridor";
        case DecisionReason::OwnHighland: return "own_highland";
        case DecisionReason::CommonCentral: return "common_central";
        case DecisionReason::EnemyReadyRoadlandSoft: return "enemy_road_corridor_soft";
        case DecisionReason::EnemyHighlandSoft: return "enemy_highland_soft";
        case DecisionReason::AimModeBuff: return "task_buff";
        case DecisionReason::AimModeOutpost: return "task_outpost";
        case DecisionReason::NaviProgressWatchdog: return "navi_progress_watchdog";
        case DecisionReason::RegionalIdlePatrol: return "regional_idle_patrol";
        case DecisionReason::AreaTask: return "area_task";
        case DecisionReason::NaviTransition: return "navi_transition";
        case DecisionReason::Start: return "start";
        case DecisionReason::OwnFortressGainPointEnemy: return "own_fortress_gain_point_enemy";
        case DecisionReason::ProtectHero: return "protect_hero";
        case DecisionReason::SpecialPatrol: return "special_patrol";
        case DecisionReason::MapCommand: return "map_command";
        case DecisionReason::Chase: return "chase";
        case DecisionReason::Recovery: return "recovery";
        case DecisionReason::ProtectOutpost: return "protect_outpost";
        case DecisionReason::League3v3: return "league_3v3";
        default: return "unknown";
    }
}

inline DecisionReason DecisionReasonFromString(const std::string_view reason) noexcept {
    if (reason == "area_scope" || reason == "area_scope_blocked") return DecisionReason::AreaScopeBlocked;
    if (reason == "default_area_policy") return DecisionReason::DefaultAreaPolicy;
    if (reason == "regional_defense") return DecisionReason::RegionalDefense;
    if (reason == "own_base_multi") return DecisionReason::OwnBaseMulti;
    if (reason == "own_base_chase") return DecisionReason::OwnBaseChase;
    if (reason == "own_base_guard") return DecisionReason::OwnBaseGuard;
    if (reason == "own_highland_road_corridor") return DecisionReason::OwnHighlandRoadCorridor;
    if (reason == "own_road_corridor") return DecisionReason::OwnRoadCorridor;
    if (reason == "own_highland") return DecisionReason::OwnHighland;
    if (reason == "common_central") return DecisionReason::CommonCentral;
    if (reason == "enemy_road_corridor_soft") return DecisionReason::EnemyReadyRoadlandSoft;
    if (reason == "enemy_highland_soft") return DecisionReason::EnemyHighlandSoft;
    if (reason == "task_buff" || reason == "regional_task_buff" ||
        reason == "regional_tactical_buff_mode") {
        return DecisionReason::AimModeBuff;
    }
    if (reason == "task_outpost" ||
        reason == "regional_task_outpost_aim" ||
        reason == "regional_task_opening_outpost_scout_travel" ||
        reason == "regional_tactical_aim_mode" ||
        reason == "regional_tactical_opening_outpost_aim" ||
        reason == "regional_tactical_opening_outpost_scout_travel" ||
        reason == "regional_tactical_outpost_scout_travel") {
        return DecisionReason::AimModeOutpost;
    }
    if (reason == "navi_progress_watchdog") return DecisionReason::NaviProgressWatchdog;
    if (reason == "regional_idle_patrol") return DecisionReason::RegionalIdlePatrol;
    if (reason == "area_task") return DecisionReason::AreaTask;
    if (reason == "navi_transition") return DecisionReason::NaviTransition;
    if (reason == "start") return DecisionReason::Start;
    if (reason == "own_fortress_gain_point_enemy") return DecisionReason::OwnFortressGainPointEnemy;
    if (reason == "protect_hero") return DecisionReason::ProtectHero;
    if (reason == "special_patrol") return DecisionReason::SpecialPatrol;
    if (reason == "map_command") return DecisionReason::MapCommand;
    if (reason == "chase") return DecisionReason::Chase;
    if (reason == "recovery") return DecisionReason::Recovery;
    if (reason == "protect_outpost") return DecisionReason::ProtectOutpost;
    if (reason == "league_3v3") return DecisionReason::League3v3;
    return DecisionReason::Unknown;
}

inline DecisionLayer DecisionLayerForReason(const DecisionReason reason) noexcept {
    switch (reason) {
        case DecisionReason::Recovery:
            return DecisionLayer::Hard;
        case DecisionReason::DefaultAreaPolicy:
            return DecisionLayer::Default;
        case DecisionReason::OwnBaseMulti:
        case DecisionReason::OwnBaseChase:
        case DecisionReason::OwnBaseGuard:
        case DecisionReason::OwnHighlandRoadCorridor:
        case DecisionReason::OwnRoadCorridor:
        case DecisionReason::OwnHighland:
        case DecisionReason::CommonCentral:
        case DecisionReason::EnemyReadyRoadlandSoft:
        case DecisionReason::EnemyHighlandSoft:
        case DecisionReason::RegionalDefense:
        case DecisionReason::OwnFortressGainPointEnemy:
            return DecisionLayer::RegionalDefense;
        case DecisionReason::AimModeBuff:
        case DecisionReason::AimModeOutpost:
            return DecisionLayer::Task;
        case DecisionReason::NaviProgressWatchdog:
            return DecisionLayer::Watchdog;
        case DecisionReason::RegionalIdlePatrol:
            return DecisionLayer::IdlePatrol;
        case DecisionReason::AreaTask:
        case DecisionReason::NaviTransition:
        case DecisionReason::MapCommand:
            return DecisionLayer::Task;
        case DecisionReason::AreaScopeBlocked:
        case DecisionReason::ProtectHero:
        case DecisionReason::Chase:
        case DecisionReason::ProtectOutpost:
        case DecisionReason::League3v3:
            return DecisionLayer::Tactical;
        case DecisionReason::SpecialPatrol:
            return DecisionLayer::Special;
        default:
            return DecisionLayer::Unknown;
    }
}

inline int DecisionPriorityForReason(const DecisionReason reason) noexcept {
    if (reason == DecisionReason::Recovery) return 400;
    if (reason == DecisionReason::MapCommand) return 390;
    if (reason == DecisionReason::AimModeBuff || reason == DecisionReason::AimModeOutpost) return 300;
    if (reason == DecisionReason::AreaTask || reason == DecisionReason::NaviTransition) return 280;
    switch (DecisionLayerForReason(reason)) {
        case DecisionLayer::Hard: return 400;
        case DecisionLayer::Task: return 200;
        case DecisionLayer::Special: return 140;
        case DecisionLayer::Default: return 120;
        case DecisionLayer::Tactical: return 100;
        case DecisionLayer::RegionalDefense: return 100;
        case DecisionLayer::AimMode: return 100;
        case DecisionLayer::Watchdog: return 90;
        case DecisionLayer::IdlePatrol: return 40;
        default: return 0;
    }
}

}  // namespace BehaviorTree
