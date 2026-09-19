// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"
#include "../include/DamageRotatePolicy.hpp"
#include "../include/ChasePolicy.hpp"
#include "../include/ExternalAimTargetPolicy.hpp"
#include "../include/RegionalDefenseSearchPolicy.hpp"
#include "../module/Mode2PatrolResume.hpp"
#include "../include/OutpostOpeningHold.hpp"
#include "../include/TacticalProtectionPolicy.hpp"

#include <array>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

using namespace LangYa;

namespace BehaviorTree {


    float normalize_angle_0_360(float angle) {
        float normalized = fmod(angle, 360.0f);
        if (normalized < 0)
            normalized += 360.0f;
        return normalized;
    }

    float normalize_angle_near(float angle, float reference) {
        return reference + static_cast<float>(std::remainder(angle - reference, 360.0f));
    }

    namespace {
    constexpr std::uint8_t kLeagueRouteCompatViaGoalBaseId = LangYa::LeftHighLand.ID;  // base goal id=4
    constexpr int kLeagueRouteCompatViaHoldSec = 5;
    constexpr int kOfficialFieldWidthCm = 2800;
    constexpr int kOfficialFieldHeightCm = 1500;
    constexpr auto kRfidFreshTimeout = std::chrono::milliseconds(1000);
    // 丢 1~2 帧时保留锁角，避免抖动；时间过长会让云台“粘住旧目标”。
    constexpr auto kLostTargetHold = std::chrono::milliseconds(200);

    struct FaceModeRuntimeLogState {
        FaceModeManager::Source Source{FaceModeManager::Source::None};
        int Phase{0};
        bool Requested{false};
        bool Active{false};
        bool Fallback{false};
        bool AnglesAvailable{false};
        bool SuppressFire{false};
        bool VisualPriority{false};
        bool NavigationRelease{false};

        bool operator==(const FaceModeRuntimeLogState&) const = default;
    };

    std::string NormalizeConfigToken(std::string value) {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
            if (c == '-' || c == ' ') {
                return '_';
            }
            return static_cast<char>(std::tolower(c));
        });
        return value;
    }

    std::optional<ArmorType> ArmorTypeFromPriorityId(const int armor_type_id) {
        switch (static_cast<ArmorType>(armor_type_id)) {
            case ArmorType::Hero:
            case ArmorType::Engineer:
            case ArmorType::Infantry1:
            case ArmorType::Infantry2:
            case ArmorType::Sentry:
                return static_cast<ArmorType>(armor_type_id);
            default:
                return std::nullopt;
        }
    }

    std::optional<UnitType> UnitTypeFromArmorType(const ArmorType armor_type) {
        switch (armor_type) {
            case ArmorType::Hero:
                return UnitType::Hero;
            case ArmorType::Engineer:
                return UnitType::Engineer;
            case ArmorType::Infantry1:
                return UnitType::Infantry1;
            case ArmorType::Infantry2:
                return UnitType::Infantry2;
            case ArmorType::Sentry:
                return UnitType::Sentry;
            default:
                return std::nullopt;
        }
    }

    std::optional<ArmorType> ArmorTypeFromUnitType(const UnitType unit_type) {
        switch (unit_type) {
            case UnitType::Hero:
                return ArmorType::Hero;
            case UnitType::Engineer:
                return ArmorType::Engineer;
            case UnitType::Infantry1:
                return ArmorType::Infantry1;
            case UnitType::Infantry2:
                return ArmorType::Infantry2;
            case UnitType::Sentry:
                return ArmorType::Sentry;
            default:
                return std::nullopt;
        }
    }

    bool IsIgnoredArmorType(const std::vector<int>& ignore_list, const ArmorType armor_type) {
        return std::find(ignore_list.begin(),
                         ignore_list.end(),
                         static_cast<int>(armor_type)) != ignore_list.end();
    }

    bool IsIgnoredUnitType(const std::vector<int>& ignore_list, const UnitType unit_type) {
        const auto armor_type = ArmorTypeFromUnitType(unit_type);
        return armor_type.has_value() && IsIgnoredArmorType(ignore_list, *armor_type);
    }

    int ClampToInt8(const int value) {
        return std::clamp(value, -128, 127);
    }

    bool IsOfficialFieldPointValid(const int x, const int y) {
        return x > 0 && y > 0 && x <= kOfficialFieldWidthCm && y <= kOfficialFieldHeightCm;
    }

    std::uint8_t BaseGoalIdFromResolvedGoal(const std::uint8_t goal_id) noexcept {
        return goal_id >= LangYa::TeamedLocation::LocationCount
            ? static_cast<std::uint8_t>(goal_id - LangYa::TeamedLocation::LocationCount)
            : goal_id;
    }

    Area::Point<std::uint16_t> BuildOfficialChaseGoal(
        const int self_x,
        const int self_y,
        const int target_x,
        const int target_y,
        const int preferred_distance_cm,
        const int distance_deadband_cm) {
        int goal_x = self_x;
        int goal_y = self_y;
        const double dx = static_cast<double>(target_x - self_x);
        const double dy = static_cast<double>(target_y - self_y);
        const double distance = std::hypot(dx, dy);
        const double preferred = static_cast<double>(std::max(0, preferred_distance_cm));
        const double deadband = static_cast<double>(std::max(0, distance_deadband_cm));

        if (distance > 1e-6 && std::abs(distance - preferred) > deadband && distance > preferred) {
            const double scale = (distance - preferred) / distance;
            goal_x = static_cast<int>(std::lround(static_cast<double>(self_x) + dx * scale));
            goal_y = static_cast<int>(std::lround(static_cast<double>(self_y) + dy * scale));
        }

        goal_x = std::clamp(goal_x, 0, kOfficialFieldWidthCm);
        goal_y = std::clamp(goal_y, 0, kOfficialFieldHeightCm);
        return Area::Point<std::uint16_t>{
            static_cast<std::uint16_t>(goal_x),
            static_cast<std::uint16_t>(goal_y)};
    }

    struct OfficialChaseAreaLimitResult {
        Area::Point<std::uint16_t> Goal{};
        bool Limited{false};
        bool Held{false};
        const char* Status{"disabled"};
        const char* AreaName{"unknown"};
    };

    struct MainAreaBoundaryView {
        const char* Name;
        UnitTeam Team;
        Area::MainAreaKind Kind;
        const std::vector<Area::Point<int>>* Boundary;
    };

    std::array<MainAreaBoundaryView, 9> MainAreaBoundaries() {
        return {{
            {"red_base", UnitTeam::Red, Area::MainAreaKind::Base,
             &Area::MainAreaBoundary(UnitTeam::Red, Area::MainAreaKind::Base)},
            {"red_highland", UnitTeam::Red, Area::MainAreaKind::Highland,
             &Area::MainAreaBoundary(UnitTeam::Red, Area::MainAreaKind::Highland)},
            {"red_pre_roadland", UnitTeam::Red, Area::MainAreaKind::PreRoadland,
             &Area::MainAreaBoundary(UnitTeam::Red, Area::MainAreaKind::PreRoadland)},
            {"red_ready_roadland", UnitTeam::Red, Area::MainAreaKind::ReadyRoadland,
             &Area::MainAreaBoundary(UnitTeam::Red, Area::MainAreaKind::ReadyRoadland)},
            {"common_central", UnitTeam::Unknown, Area::MainAreaKind::Central,
             &Area::MainAreaBoundary(UnitTeam::Red, Area::MainAreaKind::Central)},
            {"blue_base", UnitTeam::Blue, Area::MainAreaKind::Base,
             &Area::MainAreaBoundary(UnitTeam::Blue, Area::MainAreaKind::Base)},
            {"blue_highland", UnitTeam::Blue, Area::MainAreaKind::Highland,
             &Area::MainAreaBoundary(UnitTeam::Blue, Area::MainAreaKind::Highland)},
            {"blue_pre_roadland", UnitTeam::Blue, Area::MainAreaKind::PreRoadland,
             &Area::MainAreaBoundary(UnitTeam::Blue, Area::MainAreaKind::PreRoadland)},
            {"blue_ready_roadland", UnitTeam::Blue, Area::MainAreaKind::ReadyRoadland,
             &Area::MainAreaBoundary(UnitTeam::Blue, Area::MainAreaKind::ReadyRoadland)},
        }};
    }

    UnitTeam MainAreaLookupTeam(const MainAreaBoundaryView& area) {
        return area.Team == UnitTeam::Unknown ? UnitTeam::Red : area.Team;
    }

    const MainAreaBoundaryView* FindContainingMainArea(const int x, const int y) {
        static const auto boundaries = MainAreaBoundaries();
        for (const auto& boundary : boundaries) {
            if (Area::IsPointInsideMainArea(MainAreaLookupTeam(boundary), boundary.Kind, x, y)) {
                return &boundary;
            }
        }
        return nullptr;
    }

    bool AreaScopeContains(
        const std::vector<std::string>& scope,
        const Area::MainAreaKind kind) {
        for (const auto& token : scope) {
            const auto parsed = AreaManager::MainAreaKindFromToken(token);
            if (parsed.has_value() && *parsed == kind) {
                return true;
            }
        }
        return false;
    }

    bool IsMainAreaAllowedForCrossChase(
        const MainAreaBoundaryView& area,
        const UnitTeam my_team,
        const UnitTeam enemy_team,
        const NaviGoalAutonomySetting& navi_goal) {
        if (!navi_goal.UseAreaScope) {
            return true;
        }
        if (area.Kind == Area::MainAreaKind::Central) {
            return AreaScopeContains(navi_goal.CommonArea, Area::MainAreaKind::Central);
        }
        if (area.Team == my_team) {
            return AreaScopeContains(navi_goal.MyArea, area.Kind);
        }
        if (area.Team == enemy_team) {
            return AreaScopeContains(navi_goal.EnemyArea, area.Kind);
        }
        return false;
    }

    bool IsAreaKeyAllowedForChaseTarget(
        const AreaKey& area,
        const NaviGoalAutonomySetting& navi_goal) {
        if (!navi_goal.UseAreaScope) {
            return true;
        }
        if (area.Side == AreaSide::Common) {
            return AreaScopeContains(navi_goal.CommonArea, area.Kind);
        }
        if (area.Side == AreaSide::My) {
            return AreaScopeContains(navi_goal.MyArea, area.Kind);
        }
        if (area.Side == AreaSide::Enemy) {
            return AreaScopeContains(navi_goal.EnemyArea, area.Kind);
        }
        return false;
    }

    double Cross2d(
        const double ax,
        const double ay,
        const double bx,
        const double by) {
        return ax * by - ay * bx;
    }

    bool SegmentIntersectionT(
        const double start_x,
        const double start_y,
        const double end_x,
        const double end_y,
        const Area::Point<int>& edge_start,
        const Area::Point<int>& edge_end,
        double& t_out) {
        constexpr double kEpsilon = 1e-9;
        const double rx = end_x - start_x;
        const double ry = end_y - start_y;
        const double sx = static_cast<double>(edge_end.x - edge_start.x);
        const double sy = static_cast<double>(edge_end.y - edge_start.y);
        const double denom = Cross2d(rx, ry, sx, sy);
        if (std::abs(denom) < kEpsilon) {
            return false;
        }

        const double qpx = static_cast<double>(edge_start.x) - start_x;
        const double qpy = static_cast<double>(edge_start.y) - start_y;
        const double t = Cross2d(qpx, qpy, sx, sy) / denom;
        const double u = Cross2d(qpx, qpy, rx, ry) / denom;
        if (t < -kEpsilon || t > 1.0 + kEpsilon || u < -kEpsilon || u > 1.0 + kEpsilon) {
            return false;
        }
        t_out = std::clamp(t, 0.0, 1.0);
        return true;
    }

    bool FirstBoundaryIntersectionT(
        const std::vector<Area::Point<int>>& boundary,
        const double start_x,
        const double start_y,
        const double end_x,
        const double end_y,
        double& t_out) {
        if (boundary.size() < 3) {
            return false;
        }

        constexpr double kEpsilon = 1e-9;
        double best_t = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0, j = boundary.size() - 1; i < boundary.size(); j = i++) {
            double t = 0.0;
            if (SegmentIntersectionT(start_x, start_y, end_x, end_y, boundary[j], boundary[i], t) &&
                t > kEpsilon) {
                best_t = std::min(best_t, t);
            }
        }

        if (!std::isfinite(best_t)) {
            return false;
        }
        t_out = best_t;
        return true;
    }

    OfficialChaseAreaLimitResult ApplyOfficialChaseAreaLimit(
        const int self_x,
        const int self_y,
        const Area::Point<std::uint16_t>& goal,
        const UnitTeam my_team,
        const UnitTeam enemy_team,
        const NaviGoalAutonomySetting& navi_goal,
        const ChaseAreaLimitSetting& config) {
        OfficialChaseAreaLimitResult result{.Goal = goal};
        if (!config.Enable) {
            return result;
        }

        const auto* self_area = FindContainingMainArea(self_x, self_y);
        if (self_area == nullptr || self_area->Boundary == nullptr) {
            result.Status = "unknown_area";
            result.Goal = Area::Point<std::uint16_t>{
                static_cast<std::uint16_t>(std::clamp(self_x, 0, kOfficialFieldWidthCm)),
                static_cast<std::uint16_t>(std::clamp(self_y, 0, kOfficialFieldHeightCm))};
            result.Held = true;
            return result;
        }

        result.AreaName = self_area->Name;
        if (config.ChaseEnableCrossArea &&
            !IsMainAreaAllowedForCrossChase(*self_area, my_team, enemy_team, navi_goal)) {
            result.Status = "area_scope_blocked";
            result.Goal = Area::Point<std::uint16_t>{
                static_cast<std::uint16_t>(std::clamp(self_x, 0, kOfficialFieldWidthCm)),
                static_cast<std::uint16_t>(std::clamp(self_y, 0, kOfficialFieldHeightCm))};
            result.Held = true;
            return result;
        }

        const int goal_x = static_cast<int>(goal.x);
        const int goal_y = static_cast<int>(goal.y);
        if (config.ChaseEnableCrossArea) {
            const auto* goal_area = FindContainingMainArea(goal_x, goal_y);
            if (goal_area != nullptr &&
                goal_area->Boundary != nullptr &&
                IsMainAreaAllowedForCrossChase(*goal_area, my_team, enemy_team, navi_goal)) {
                result.Status = "inside_allowed_area";
                result.AreaName = goal_area->Name;
                return result;
            }
        } else {
            if (Area::IsPointInsideMainArea(MainAreaLookupTeam(*self_area), self_area->Kind, goal_x, goal_y)) {
                result.Status = "inside";
                return result;
            }
        }

        double t_hit = 0.0;
        if (!FirstBoundaryIntersectionT(
                *self_area->Boundary,
                static_cast<double>(self_x),
                static_cast<double>(self_y),
                static_cast<double>(goal_x),
                static_cast<double>(goal_y),
                t_hit)) {
            result.Status = "no_intersection";
            if (config.HoldWhenNoIntersection) {
                result.Goal = Area::Point<std::uint16_t>{
                    static_cast<std::uint16_t>(std::clamp(self_x, 0, kOfficialFieldWidthCm)),
                    static_cast<std::uint16_t>(std::clamp(self_y, 0, kOfficialFieldHeightCm))};
                result.Held = true;
            }
            return result;
        }

        const double dx = static_cast<double>(goal_x - self_x);
        const double dy = static_cast<double>(goal_y - self_y);
        const double length_cm = std::hypot(dx, dy);
        double t_limit = t_hit;
        if (length_cm > 1e-6 && config.BoundaryMarginCm > 0) {
            t_limit = std::max(0.0, t_hit - static_cast<double>(config.BoundaryMarginCm) / length_cm);
        }

        const int limited_x = std::clamp(
            static_cast<int>(std::lround(static_cast<double>(self_x) + dx * t_limit)),
            0,
            kOfficialFieldWidthCm);
        const int limited_y = std::clamp(
            static_cast<int>(std::lround(static_cast<double>(self_y) + dy * t_limit)),
            0,
            kOfficialFieldHeightCm);
        result.Goal = Area::Point<std::uint16_t>{
            static_cast<std::uint16_t>(limited_x),
            static_cast<std::uint16_t>(limited_y)};
        result.Limited = true;
        result.Status = "clamped";
        return result;
    }

    std::string NormalizeDecisionModule(std::string_view module) {
        std::string normalized(module);
        std::transform(normalized.begin(), normalized.end(), normalized.begin(),
            [](unsigned char c) {
                if (c == '-' || c == ' ') {
                    return '_';
                }
                return static_cast<char>(std::tolower(c));
            });
        return normalized;
    }

    bool HasAutonomyToken(const std::vector<std::string>& tokens, std::string_view expected) {
        const auto normalized_expected = NormalizeDecisionModule(expected);
        return std::find(tokens.begin(), tokens.end(), normalized_expected) != tokens.end();
    }

    }  // namespace

    DecisionIntent Application::MakeDecisionIntent(
        const DecisionReason reason,
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const bool apply_team_offset,
        const char* detail) const {
        DecisionIntent intent{};
        intent.Layer = DecisionLayerForReason(reason);
        intent.Reason = reason;
        intent.BaseGoalId = base_goal_id;
        intent.ResolvedGoalId = ResolveGoalId(base_goal_id, goal_team, apply_team_offset);
        intent.GoalTeam = goal_team;
        intent.ApplyTeamOffset = apply_team_offset;
        intent.Priority = DecisionPriorityForReason(reason);
        intent.Detail = detail != nullptr && detail[0] != '\0'
            ? detail
            : DecisionReasonToString(reason);
        return intent;
    }

    void Application::RecordDecisionIntent(DecisionIntent intent) {
        lastDecisionIntent_ = std::move(intent);
    }

    Application::UnitPositionState Application::GetSentryPositionState(
        const std::chrono::steady_clock::time_point now) const {
        return GetSentryPositionState(
            now,
            std::max(1, config.SentryPositionFusionSettings.FreshTimeoutMs));
    }

    Application::UnitPositionState Application::GetSentryPositionState(
        const std::chrono::steady_clock::time_point now,
        const int fresh_ms) const {
        UnitPositionState state;
        state.X = static_cast<int>(friendRobots[UnitType::Sentry].position_.X);
        state.Y = static_cast<int>(friendRobots[UnitType::Sentry].position_.Y);
        state.Source = sentryPositionFusionSource_.empty()
            ? "unknown"
            : sentryPositionFusionSource_;
        const auto sentry_index = static_cast<std::size_t>(UnitType::Sentry);
        if (sentry_index < lastFriendPositionStamp_.size()) {
            state.Stamp = lastFriendPositionStamp_[sentry_index].Stamp;
        }
        state.HasPosition =
            hasReceivedSentryPosition_ &&
            lastSentryPositionRxTime_.time_since_epoch().count() != 0;
        if (!state.HasPosition) {
            state.Source = "none";
            return state;
        }
        state.AgeMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - lastSentryPositionRxTime_).count();
        state.Fresh = state.AgeMs <= std::max(1, fresh_ms);
        return state;
    }

    Application::UnitPositionState Application::GetFriendPositionState(
        const UnitType unit_type,
        const int fresh_ms,
        const std::chrono::steady_clock::time_point now) const {
        if (unit_type == UnitType::Sentry) {
            return GetSentryPositionState(now, fresh_ms);
        }
        UnitPositionState state;
        const auto index = static_cast<std::size_t>(unit_type);
        if (index >= lastFriendPositionRxTime_.size()) {
            return state;
        }
        const auto& robot = friendRobots[unit_type];
        state.X = static_cast<int>(robot.position_.X);
        state.Y = static_cast<int>(robot.position_.Y);
        state.Source = "position_data";
        state.Stamp = lastFriendPositionStamp_[index].Stamp;
        const auto& last_rx = lastFriendPositionRxTime_[index];
        state.HasPosition = last_rx.time_since_epoch().count() != 0;
        if (!state.HasPosition) {
            state.Source = "none";
            return state;
        }
        state.AgeMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_rx).count();
        state.Fresh = state.AgeMs <= std::max(1, fresh_ms);
        return state;
    }

    Application::UnitPositionState Application::GetEnemyPositionState(
        const UnitType unit_type,
        const int fresh_ms,
        const std::chrono::steady_clock::time_point now) const {
        UnitPositionState state;
        const auto index = static_cast<std::size_t>(unit_type);
        if (index >= lastEnemyPositionRxTime_.size()) {
            return state;
        }
        const auto& robot = enemyRobots[unit_type];
        state.X = static_cast<int>(robot.position_.X);
        state.Y = static_cast<int>(robot.position_.Y);
        state.Source = lastEnemyPositionSource_[index].empty()
            ? "position_data"
            : lastEnemyPositionSource_[index];
        state.Stamp = lastEnemyPositionStamp_[index].Stamp;
        const auto& last_rx = lastEnemyPositionRxTime_[index];
        state.HasPosition = last_rx.time_since_epoch().count() != 0;
        if (!state.HasPosition) {
            state.Source = "none";
            return state;
        }
        state.AgeMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_rx).count();
        state.Fresh = state.AgeMs <= std::max(1, fresh_ms);
        return state;
    }

    bool Application::IsSentryPositionFresh(
        const std::chrono::steady_clock::time_point now) const {
        if (!hasReceivedSentryPosition_ ||
            lastSentryPositionRxTime_.time_since_epoch().count() == 0) {
            return false;
        }
        return GetSentryPositionState(now).Fresh;
    }

    void Application::UpdateSentryPositionFusion(
        const std::chrono::steady_clock::time_point now) {
        const auto& setting = config.SentryPositionFusionSettings;
        struct Candidate {
            const char* Name;
            const SentryPositionSourceCache* Cache;
            const SentryPositionFusionSourceSetting* Setting;
        };

        const std::array<Candidate, 3> candidates{{
            {"uwb", &sentryUwbPositionSource_, &setting.Uwb},
            {"position_data", &sentryPositionDataSource_, &setting.PositionData},
            {"navi", &sentryNaviPositionSource_, &setting.Navi},
        }};

        auto source_timeout_ms = [&](const SentryPositionFusionSourceSetting& source) {
            if (source.FreshTimeoutMs > 0) {
                return source.FreshTimeoutMs;
            }
            return std::max(1, setting.FreshTimeoutMs);
        };
        auto source_fresh = [&](const Candidate& candidate) {
            if (candidate.Cache == nullptr || candidate.Setting == nullptr ||
                (setting.Enable && !candidate.Setting->Enable) ||
                !candidate.Cache->Valid ||
                candidate.Cache->LastRx.time_since_epoch().count() == 0 ||
                candidate.Cache->X <= 0 || candidate.Cache->Y <= 0) {
                return false;
            }
            return now - candidate.Cache->LastRx <=
                std::chrono::milliseconds(std::max(1, source_timeout_ms(*candidate.Setting)));
        };
        auto apply_result = [&](const int x,
                                const int y,
                                const std::chrono::steady_clock::time_point last_rx,
                                const rclcpp::Time& stamp,
                                const char* source) {
            const int clamped_x = std::clamp(
                x,
                0,
                static_cast<int>(std::numeric_limits<std::int16_t>::max()));
            const int clamped_y = std::clamp(
                y,
                0,
                static_cast<int>(std::numeric_limits<std::int16_t>::max()));
            const auto sentry_index = static_cast<std::size_t>(UnitType::Sentry);
            friendRobots[UnitType::Sentry].position_.X = static_cast<std::int16_t>(clamped_x);
            friendRobots[UnitType::Sentry].position_.Y = static_cast<std::int16_t>(clamped_y);
            hasReceivedSentryPosition_ = true;
            lastSentryPositionRxTime_ = last_rx;
            if (sentry_index < lastFriendPositionRxTime_.size()) {
                lastFriendPositionRxTime_[sentry_index] = last_rx;
                lastFriendPositionStamp_[sentry_index].Stamp = stamp;
            }

            const std::string next_source = source == nullptr ? "unknown" : source;
            const bool source_changed = next_source != sentryPositionFusionSource_;
            sentryPositionFusionSource_ = next_source;
            if (LoggerPtr &&
                (source_changed ||
                 now - lastSentryPositionFusionLogTime_ > std::chrono::seconds(2))) {
                LoggerPtr->Debug(
                    "Sentry position fusion: mode={} source={} x={} y={}",
                    setting.Enable ? NormalizeConfigToken(setting.Mode) : "priority",
                    sentryPositionFusionSource_,
                    clamped_x,
                    clamped_y);
                lastSentryPositionFusionLogTime_ = now;
            }
        };

        const auto mode = setting.Enable
            ? NormalizeConfigToken(setting.Mode)
            : std::string{"priority"};
        if (mode == "weighted" || mode == "weight" || mode == "weighted_fit") {
            double sum_weight = 0.0;
            double sum_x = 0.0;
            double sum_y = 0.0;
            std::chrono::steady_clock::time_point latest_rx{};
            rclcpp::Time latest_stamp{};
            int used_count = 0;
            for (const auto& candidate : candidates) {
                if (!source_fresh(candidate)) {
                    continue;
                }
                const double weight = std::max(0.0, candidate.Setting->Weight);
                if (weight <= 0.0) {
                    continue;
                }
                sum_weight += weight;
                sum_x += static_cast<double>(candidate.Cache->X) * weight;
                sum_y += static_cast<double>(candidate.Cache->Y) * weight;
                if (latest_rx.time_since_epoch().count() == 0 ||
                    candidate.Cache->LastRx > latest_rx) {
                    latest_rx = candidate.Cache->LastRx;
                    latest_stamp = candidate.Cache->Stamp;
                }
                ++used_count;
            }
            if (sum_weight > 0.0 && used_count > 0) {
                const int fused_x = static_cast<int>(std::lround(sum_x / sum_weight));
                const int fused_y = static_cast<int>(std::lround(sum_y / sum_weight));
                apply_result(
                    fused_x,
                    fused_y,
                    latest_rx,
                    latest_stamp,
                    used_count == 1 ? "weighted_single" : "weighted");
            }
            return;
        }

        const Candidate* best = nullptr;
        for (const auto& candidate : candidates) {
            if (!source_fresh(candidate)) {
                continue;
            }
            if (best == nullptr ||
                candidate.Setting->Priority < best->Setting->Priority ||
                (candidate.Setting->Priority == best->Setting->Priority &&
                 candidate.Cache->LastRx > best->Cache->LastRx)) {
                best = &candidate;
            }
        }
        if (best != nullptr) {
            apply_result(best->Cache->X, best->Cache->Y, best->Cache->LastRx, best->Cache->Stamp, best->Name);
        }
    }

     /**
     * @brief 更新黑板数据 \n
     * @brief  更新数据从上到下依次是：我方颜色，敌方哨站血量，我方哨站血量，剩余弹药，比赛剩余时间 \n
     * @brief  自身血量，己方英雄血量，己方3号步兵血量，视野中的装甲板序列，是否找到目标 \n
     */
    void Application::UpdateBlackBoard() {

        std::uint16_t SelfHealth = myselfHealth;
        faceModeManager_.BeginCycle();
        // 外部 aim 是唯一正式视觉目标源。
        // 注意这里是“本拍是否有新鲜目标”，不是长期跟踪状态。
        const bool has_external_target =
            config.ExternalAimSettings.Enable && externalAimData.Fresh && externalAimData.Valid;
        const bool IsFindTarget = has_external_target;
        const auto now = std::chrono::steady_clock::now();
        rfidMatchState.Fresh =
            hasReceivedRfidStatus_ &&
            lastRfidStatusRxTime_.time_since_epoch().count() != 0 &&
            now - lastRfidStatusRxTime_ <= kRfidFreshTimeout;
        const auto enemy_team = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        UpdateSentryPositionFusion(now);
        const auto self_position = GetSentryPositionState(now);
        areaManager_.TickSelfArea(
            now,
            self_position.Fresh,
            self_position.X,
            self_position.Y,
            team,
            enemy_team);

        if (!GlobalBlackboard_) {
            GlobalBlackboard_ = BT::Blackboard::create();
        }

        // 将数据写入黑板
        GlobalBlackboard_->set<UnitTeam>("MyTeam", team);
        GlobalBlackboard_->set<std::uint16_t>("TimeLeft", timeLeft);
        GlobalBlackboard_->set<std::uint16_t>("SelfHealth", SelfHealth);
        GlobalBlackboard_->set<std::uint16_t>("AmmoLeft", ammoLeft);
        GlobalBlackboard_->set<Robots>("FriendRobots", friendRobots);
        GlobalBlackboard_->set<Robots>("EnemyRobots", enemyRobots);
        GlobalBlackboard_->set<gimbal_driver::msg::UnitInfoArray>("FriendInfo", MakeFriendInfoMsg());
        GlobalBlackboard_->set<gimbal_driver::msg::UnitInfoArray>("EnemyInfo", MakeEnemyInfoMsg());
        GlobalBlackboard_->set<std::uint16_t>("EnemyOutpostHealth", enemyOutpostHealth);
        GlobalBlackboard_->set<std::uint16_t>("SelfOutpostHealth", selfOutpostHealth);
        GlobalBlackboard_->set<std::uint16_t>("SelfBaseHealth", selfBaseHealth);
        GlobalBlackboard_->set<std::uint16_t>("EnemyBaseHealth", enemyBaseHealth);
        GlobalBlackboard_->set<std::uint32_t>("RfidStatus", rfidStatus);
        GlobalBlackboard_->set<bool>("HasRfidStatus2", hasRfidStatus2);
        GlobalBlackboard_->set<std::uint8_t>("RfidStatus2", rfidStatus2);
        GlobalBlackboard_->set<RfidMatchState>("RfidMatch", rfidMatchState);
        GlobalBlackboard_->set<bool>("RfidFresh", rfidMatchState.Fresh);
        GlobalBlackboard_->set<bool>("RfidAny", rfidMatchState.Fresh && rfidMatchState.Any);
        GlobalBlackboard_->set<bool>("RfidSelfSupply", rfidMatchState.Fresh && rfidMatchState.SelfSupply);
        GlobalBlackboard_->set<bool>("RfidSelfBaseGainPoint", rfidMatchState.Fresh && rfidMatchState.SelfBaseGainPoint);
        GlobalBlackboard_->set<bool>("RfidSelfHighlandGainPoint", rfidMatchState.Fresh && rfidMatchState.SelfHighlandGainPoint);
        GlobalBlackboard_->set<bool>("RfidEnemyHighlandGainPoint", rfidMatchState.Fresh && rfidMatchState.EnemyHighlandGainPoint);
        GlobalBlackboard_->set<bool>("RfidSelfRoadCrossing", rfidMatchState.Fresh && rfidMatchState.SelfRoadCrossing);
        GlobalBlackboard_->set<bool>("RfidEnemyRoadCrossing", rfidMatchState.Fresh && rfidMatchState.EnemyRoadCrossing);
        GlobalBlackboard_->set<bool>("RfidSelfTunnel", rfidMatchState.Fresh && rfidMatchState.SelfTunnel);
        GlobalBlackboard_->set<bool>("RfidEnemyTunnel", rfidMatchState.Fresh && rfidMatchState.EnemyTunnel);
        GlobalBlackboard_->set<bool>("RfidTunnel", rfidMatchState.Fresh && rfidMatchState.Tunnel);
        GlobalBlackboard_->set<bool>("RfidCenterGainPoint", rfidMatchState.Fresh && rfidMatchState.CenterGainPoint);
        GlobalBlackboard_->set<bool>("RfidOnSelfSide", rfidMatchState.Fresh && rfidMatchState.OnSelfSideRfid);
        GlobalBlackboard_->set<bool>("RfidOnEnemySide", rfidMatchState.Fresh && rfidMatchState.OnEnemySideRfid);
        GlobalBlackboard_->set<std::uint32_t>("ExtEventData", extEventData);
        GlobalBlackboard_->set<bool>("HasEventData", hasReceivedEventData_);
        GlobalBlackboard_->set<std::uint8_t>("EventSelfSmallEnergyStatus", eventSelfSmallEnergyStatus_);
        GlobalBlackboard_->set<std::uint8_t>("EventSelfLargeEnergyStatus", eventSelfLargeEnergyStatus_);
        GlobalBlackboard_->set<std::uint8_t>("EventSelfFortressGainPointStatus", eventSelfFortressGainPointStatus_);
        GlobalBlackboard_->set<std::uint8_t>("EventSelfOutpostGainPointStatus", eventSelfOutpostGainPointStatus_);
        GlobalBlackboard_->set<bool>("EventSelfBaseGainPointStatus", eventSelfBaseGainPointStatus_);
        GlobalBlackboard_->set("ArmorList", armorList);
        GlobalBlackboard_->set("TeamBuff", teamBuff);
        GlobalBlackboard_->set<std::uint8_t>("AimMode", static_cast<std::uint8_t>(aimMode));
        GlobalBlackboard_->set<std::uint8_t>("NaviGoal", naviCommandGoal);
        GlobalBlackboard_->set<int>("BuffShootCount", buff_shoot_count);
        GlobalBlackboard_->set<std::uint8_t>("StrategyMode", static_cast<std::uint8_t>(strategyMode_));
        GlobalBlackboard_->set<std::chrono::steady_clock::time_point>("GameStartTime", gameStartTime);
        GlobalBlackboard_->set<bool>("IsFindTarget", IsFindTarget);
        GlobalBlackboard_->set<std::uint8_t>("PostureState", postureState);
        GlobalBlackboard_->set<std::uint8_t>("PostureCommand", postureCommand);
        GlobalBlackboard_->set<bool>("PostureUnderFireRecent", IsUnderFireRecent());
        GlobalBlackboard_->set<bool>("PostureUnderFireBurst", IsUnderFireBurst());
        GlobalBlackboard_->set<std::int16_t>("GimbalYawVelRaw", gimbalYawVelRaw);
        GlobalBlackboard_->set<float>("GimbalYawVelDegPerSec", gimbalYawVelDegPerSec);
        GlobalBlackboard_->set<std::int16_t>("GimbalYawAngleRaw", gimbalYawAngleRaw);
        GlobalBlackboard_->set<float>("GimbalYawAngleDeg", gimbalYawAngleDeg);
        GlobalBlackboard_->set<std::string>("DecisionIntentLayer", DecisionLayerToString(lastDecisionIntent_.Layer));
        GlobalBlackboard_->set<std::string>("DecisionIntentReason", DecisionReasonToString(lastDecisionIntent_.Reason));
        GlobalBlackboard_->set<std::uint8_t>("DecisionIntentBaseGoal", lastDecisionIntent_.BaseGoalId);
        GlobalBlackboard_->set<std::uint8_t>("DecisionIntentResolvedGoal", lastDecisionIntent_.ResolvedGoalId);
        GlobalBlackboard_->set<int>("DecisionIntentPriority", lastDecisionIntent_.Priority);
        GlobalBlackboard_->set<std::string>("DecisionIntentDetail", lastDecisionIntent_.Detail);
        if (TickBlackboard_) {
            TickBlackboard_->set<std::string>("DecisionIntentLayer", DecisionLayerToString(lastDecisionIntent_.Layer));
            TickBlackboard_->set<std::string>("DecisionIntentReason", DecisionReasonToString(lastDecisionIntent_.Reason));
            TickBlackboard_->set<std::uint8_t>("DecisionIntentBaseGoal", lastDecisionIntent_.BaseGoalId);
            TickBlackboard_->set<std::uint8_t>("DecisionIntentResolvedGoal", lastDecisionIntent_.ResolvedGoalId);
            TickBlackboard_->set<int>("DecisionIntentPriority", lastDecisionIntent_.Priority);
            TickBlackboard_->set<std::string>("DecisionIntentDetail", lastDecisionIntent_.Detail);
        }

        if (now - lastUpdateBlackboardLogTime_ > std::chrono::seconds(2)) {
            LoggerPtr->Debug("Blackboard updated: TimeLeft={}, SelfHealth={}, AmmoLeft={}, EnemyOutpostHealth={}, SelfOutpostHealth={}",
                timeLeft, SelfHealth, ammoLeft, enemyOutpostHealth, selfOutpostHealth);
            lastUpdateBlackboardLogTime_ = now;
        }
    }

    void Application::UpdateEventSnapshot() {
        const auto now = std::chrono::steady_clock::now();
        const auto my_team = team;
        const auto enemy_team = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        const int referee_fresh_ms = std::max(
            std::max(0, config.TaskSettings.BuffConfirm.RefereeFreshTimeoutMs),
            std::max(0, config.TaskSettings.OutpostConfirm.RefereeFreshTimeoutMs));
        const bool recent_damage_over_30 = [&]() {
            for (auto it = postureRecentDamageSamples_.rbegin();
                 it != postureRecentDamageSamples_.rend();
                 ++it) {
                if ((now - it->Time) > std::chrono::milliseconds(1000)) {
                    break;
                }
                if (it->Delta > 30U) {
                    return true;
                }
            }
            return false;
        }();
        const auto current_goal_reach = EvaluateNaviGoalReach(
            naviCommandGoal,
            naviGoalPosition,
            std::max(1, config.DecisionAutonomySettings.NaviGoal.HighlandCompatArriveDistanceCm),
            0,
            BaseGoalIdFromResolvedGoal(naviCommandGoal),
            GoalReachTimeoutSecForBaseGoal(BaseGoalIdFromResolvedGoal(naviCommandGoal)));
        UpdateProtectOutpostState(now);

        eventSnapshot_ = eventManager_.Evaluate(
            EventEvaluateInput{
                .Now = now,
                .RefereeFreshTimeoutMs = referee_fresh_ms,
                .HasEventData = hasReceivedEventData_,
                .LastEventDataRxTime = lastEventDataRxTime_,
                .SelfSmallEnergyStatus = eventSelfSmallEnergyStatus_,
                .SelfLargeEnergyStatus = eventSelfLargeEnergyStatus_,
                .SelfFortressGainPointStatus = eventSelfFortressGainPointStatus_,
                .SelfOutpostGainPointStatus = eventSelfOutpostGainPointStatus_,
                .SelfBaseGainPointStatus = eventSelfBaseGainPointStatus_,
                .HasSentryInfo = hasReceivedSentryInfo_,
                .LastSentryInfoRxTime = lastSentryInfoRxTime_,
                .SentryCanActivateEnergyMechanism = sentryCanActivateEnergyMechanism_,
                .BuffTaskEnabled = config.TaskSettings.Buff,
                .OutpostTaskEnabled = config.TaskSettings.Outpost,
                .OutpostMaxGameTimeSec = config.TaskSettings.OutpostConfirm.MaxGameTimeSec,
                .ElapsedGameSec = ElapsedSeconds(),
                .HasEnemyOutpostHealth = hasReceivedEnemyOutpostHealth_,
                .LastEnemyOutpostHealthRxTime = lastEnemyOutpostHealthRxTime_,
                .EnemyOutpostHealth = enemyOutpostHealth,
                .HasSelfHealth = hasReceivedMyselfHealth_,
                .LastSelfHealthRxTime = lastMyselfHealthRxTime,
                .SelfHealth = myselfHealth,
                .LowHpThreshold = config.LeagueStrategySettings.HealthRecoveryThreshold,
                .HasAmmo = hasReceivedAmmoLeft_,
                .LastAmmoRxTime = lastAmmoLeftRxTime,
                .Ammo = ammoLeft,
                .LowAmmoThreshold = config.LeagueStrategySettings.AmmoRecoveryThreshold,
                .RecentDamageOver30 = recent_damage_over_30,
                .ArmorTargetVisible = AutoAimFreshAndValid(),
                .BuffTargetLocked = BuffAimTargetLocked(),
                .OutpostTargetLocked = OutpostAimFreshAndValid(),
                .NaviStatusFreshTimeoutMs = kNaviExternalStatusTimeoutMs,
                .HasNaviReach = hasReceivedNaviReach_,
                .NaviReach = naviReach,
                .LastNaviReachRxTime = lastNaviReachRxTime_,
                .HasNaviReachable = hasReceivedNaviReachable_,
                .NaviReachable = naviReachable,
                .LastNaviReachableRxTime = lastNaviReachableRxTime_,
                .CompositeGoalReached = current_goal_reach.Status == GoalReachStatus::Reached,
                .CompositeGoalUnreachable = current_goal_reach.Status == GoalReachStatus::Unreachable,
                .RegionalDefense = EvaluateRegionalDefenseThreat(my_team, enemy_team)
            });

        if (!GlobalBlackboard_) {
            GlobalBlackboard_ = BT::Blackboard::create();
        }
        GlobalBlackboard_->set<EventSnapshot>("EventSnapshot", eventSnapshot_);
        GlobalBlackboard_->set<std::uint8_t>(
            "NaviGoalReachStatus",
            static_cast<std::uint8_t>(current_goal_reach.Status));
        GlobalBlackboard_->set<std::uint8_t>(
            "NaviGoalReachReason",
            static_cast<std::uint8_t>(current_goal_reach.Reason));
        GlobalBlackboard_->set<std::string>(
            "NaviGoalReachStatusName",
            GoalReachStatusToString(current_goal_reach.Status));
        GlobalBlackboard_->set<std::string>(
            "NaviGoalReachReasonName",
            GoalReachReasonToString(current_goal_reach.Reason));
        GlobalBlackboard_->set<double>("NaviGoalReachDistanceCm", current_goal_reach.DistanceCm);
        GlobalBlackboard_->set<bool>("NaviGoalReachWithinArriveDistance", current_goal_reach.WithinArriveDistance);
        GlobalBlackboard_->set<bool>("NaviGoalReachWithinFaceDistance", current_goal_reach.WithinFaceDistance);
        GlobalBlackboard_->set<bool>("NaviGoalReachTimeout", current_goal_reach.Timeout);
        GlobalBlackboard_->set<bool>("EventBuffCanActivate", eventSnapshot_.BuffCanActivate);
        GlobalBlackboard_->set<bool>("EventBuffActivating", eventSnapshot_.BuffActivating);
        GlobalBlackboard_->set<bool>("EventBuffActivated", eventSnapshot_.BuffActivated);
        GlobalBlackboard_->set<bool>("EventEnemyOutpostAlive", eventSnapshot_.EnemyOutpostAlive);
        GlobalBlackboard_->set<bool>("EventRegionalDefenseActive", eventSnapshot_.RegionalDefenseActive);
        GlobalBlackboard_->set<bool>("EventRecentDamageOver30", eventSnapshot_.RecentDamageOver30);
        GlobalBlackboard_->set<bool>("EventGoalReached", eventSnapshot_.GoalReached);
        GlobalBlackboard_->set<bool>("EventGoalUnreachable", eventSnapshot_.GoalUnreachable);
        GlobalBlackboard_->set<std::uint8_t>(
            "EventSelfFortressGainPointStatus",
            eventSnapshot_.SelfFortressGainPointStatus);
    }

    /**
     * @brief 从黑板获取数据,处理ros队列的消息并发布 \n
     * @brief 从黑板获取的有辐瞄击打目标，导航目的地
     */
    void Application::TransportData() {
        // targetArmor.Type = GetInfoFromBlackBoard<ArmorType>("AimTarget");
        // naviCommandGoal = GetInfoFromBlackBoard<std::uint8_t>("naviCommandGoal");
        // LoggerPtr->Info("AimTarget: {}, NaviGoal: {}", static_cast<int>(targetArmor.Type), static_cast<int>(naviCommandGoal));

        
        // targetArmor = ArmorType::Infantry1;
        // 设置数据内容
        PublishTogether();
        const auto now = std::chrono::steady_clock::now();
        if (now - lastTransportLogTime_ > std::chrono::seconds(1)) {
            PrintMessageAll();
            LoggerPtr->Debug("TransportData: published control data.");
            lastTransportLogTime_ = now;
        }
    }

     /**
     * @brief 实现PublishTogether \n
     * @brief 判断是否找到目标， 找到目标就发送目标数据 \n
     * @brief 否则经过一定时间之后， 将gimbalControlData的GimbalAngles均匀变化
     */
    void Application::PublishTogether() {

        // int now_time = 420 - timeLeft;
        int now_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gameStartTime).count();
        static constexpr auto delta_yaw = 1.0f; //bt每tick單位
        static constexpr auto buff_yaw = -50.0f + 360.0f;
        const auto& patrol_scan = config.PatrolScanSettings;
        static constexpr auto kTwoPi = 6.2831853071795864769f;
        const auto& damage_rotate_setting = config.TacticalSettings.DamageRotate;
        bool navi_rotate_control_stop_request = false;
        bool navi_rotate_control_release_request = false;
        bool navi_rotate_control_follow_output = false;
        if (config.NaviControlSettings.Enable) {
            bool external_is_rotate = config.NaviControlSettings.DefaultIsRotate;
            const auto rotate_control_now = std::chrono::steady_clock::now();
            const bool external_is_rotate_fresh =
                hasReceivedNaviIsRotate_ &&
                lastNaviIsRotateRxTime_.time_since_epoch().count() != 0 &&
                rotate_control_now - lastNaviIsRotateRxTime_ <=
                    std::chrono::milliseconds(config.NaviControlSettings.FreshTimeoutMs);
            if (external_is_rotate_fresh) {
                external_is_rotate = naviIsRotate;
            }
            navi_rotate_control_stop_request = !external_is_rotate;
            navi_rotate_control_release_request =
                external_is_rotate_fresh &&
                external_is_rotate &&
                config.NaviControlSettings.ClearFollowModeWhenTrue;
            if (!navi_rotate_control_release_request &&
                navi_rotate_control_stop_request &&
                config.NaviControlSettings.ForceFollowModeWhenFalse) {
                navi_rotate_control_follow_output = true;
            }
        }
        
        // 小陀螺策略：
        // 1) 平时使用 Tactical 全局默认档；
        // 2) 受击后按 0 -> 1 -> 2 -> 3 递进换档；
        // 3) 到 3 档后持续保持；
        // 4) 仅在一段时间未受击后，回落到 Tactical 全局默认档。
        const auto rotate_now = std::chrono::steady_clock::now();
        static auto last_damage_rotate_time = std::chrono::steady_clock::time_point{};
        static auto rotate_ramp_start_time = std::chrono::steady_clock::time_point{};
        static bool rotate_under_fire = false;

        const bool damage_detected_this_tick = healthDecreaseDetector.trigger(myselfHealth); // 血量减少
        if (damage_detected_this_tick) {
            last_damage_rotate_time = rotate_now;
            if (!rotate_under_fire) {
                rotate_under_fire = true;
                rotate_ramp_start_time = rotate_now;
            }
            rotateTimerClock.tick();
        }

        bool in_damage_rotate_window = false;
        int damage_rotate_elapsed_ms = -1;
        const auto current_base_goal_id = BaseGoalIdFromResolvedGoal(naviCommandGoal);
        const std::uint8_t default_rotate_gear = damage_rotate_setting.DefaultGear;
        std::uint8_t rotate_gear = default_rotate_gear;
        std::uint8_t damage_rotate_gear = default_rotate_gear;

        if (last_damage_rotate_time.time_since_epoch().count() != 0) {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                rotate_now - last_damage_rotate_time).count();
            damage_rotate_elapsed_ms = static_cast<int>(elapsed_ms);
            in_damage_rotate_window = elapsed_ms <= damage_rotate_setting.ScanBoostWindowMs;
        }

        if (rotate_under_fire) {
            const auto no_hit_ms = (last_damage_rotate_time.time_since_epoch().count() == 0)
                ? damage_rotate_setting.NoHitTimeoutMs + 1
                : static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                    rotate_now - last_damage_rotate_time).count());

            if (no_hit_ms > damage_rotate_setting.NoHitTimeoutMs) {
                rotate_under_fire = false;
                rotate_gear = default_rotate_gear;
            } else {
                const auto ramp_ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                    rotate_now - rotate_ramp_start_time).count());
                damage_rotate_gear = ResolveDamageRotateGear(
                    damage_rotate_setting, default_rotate_gear, ramp_ms);
                rotate_gear = damage_rotate_gear;
            }
        }

        const bool highland_compat_disable_rotate_active =
            areaManager_.HighlandTransitionActive() &&
            config.DecisionAutonomySettings.NaviGoal.HighlandCompatDisableRotate &&
            !navi_rotate_control_release_request;
        const int regional_referee_fresh_ms = std::max(
            std::max(0, config.TaskSettings.BuffConfirm.RefereeFreshTimeoutMs),
            std::max(0, config.TaskSettings.OutpostConfirm.RefereeFreshTimeoutMs));
        const bool fortress_defense_search_active =
            regionalDefenseSearchKind_ == RegionalDefenseSearchKind::OwnFortressGainPoint &&
            current_base_goal_id != LangYa::Recovery.ID;
        const bool fortress_defense_control_allowed =
            !highland_compat_disable_rotate_active;
        const bool fortress_defense_target_locked =
            aimMode != AimMode::Buff &&
            aimMode != AimMode::Outpost &&
            fortress_defense_search_active &&
            (IsProtectCastleRfidStayActive(regional_referee_fresh_ms) ||
             IsFortressGainPointEnemyOccupiedEventFresh(regional_referee_fresh_ms)) &&
            isFindTargetAtomic.load(std::memory_order_relaxed);
        const bool fortress_defense_stand_still =
            fortress_defense_target_locked &&
            fortress_defense_control_allowed &&
            !config.AimDebugSettings.StopFire &&
            fortressGainPointEnemyCount_ >=
                std::max(1, config.RegionalDefenseSettings.FortressStandEnemyCountMin);
        if (fortress_defense_stand_still &&
            !config.AimDebugSettings.StopRotate &&
            fortress_defense_control_allowed) {
            rotate_gear = 3;
        }
        const std::uint8_t computed_rotate_gear = rotate_gear;
        const bool follow_mode_output =
            gimbalControlData.FireCode.FollowMode != 0 || navi_rotate_control_follow_output;
        const auto rotate_resolution = ResolveFinalRotateGear(
            computed_rotate_gear,
            config.AimDebugSettings.StopRotate,
            highland_compat_disable_rotate_active,
            navi_rotate_control_stop_request && config.NaviControlSettings.StopRotateWhenFalse,
            follow_mode_output);
        gimbalControlData.FireCode.Rotate = rotate_resolution.Gear;

        static auto last_rotate_log = std::chrono::steady_clock::time_point{};
        if (now_time >= 0) {
            const auto log_now = std::chrono::steady_clock::now();
            if (log_now - last_rotate_log > std::chrono::seconds(2)) {
                LoggerPtr->Debug(
                    "Rotate Gear: {} (base_goal={} tactical_default={} damage_detected={} damage_active={} damage_gear={} computed_gear={} suppressed_by={} damage_elapsed_ms={} no_hit_timeout_ms={})",
                    gimbalControlData.FireCode.Rotate,
                    static_cast<int>(current_base_goal_id),
                    static_cast<int>(default_rotate_gear),
                    damage_detected_this_tick ? 1 : 0,
                    rotate_under_fire ? 1 : 0,
                    static_cast<int>(damage_rotate_gear),
                    static_cast<int>(computed_rotate_gear),
                    RotateSuppressionSourceToString(rotate_resolution.SuppressedBy),
                    damage_rotate_elapsed_ms,
                    damage_rotate_setting.NoHitTimeoutMs);
                last_rotate_log = log_now;
            }
        }

        /*----------云台----------*/
        auto now = std::chrono::steady_clock::now();
        const bool outpost_damage_abort_scan_active =
            outpostTaskDamageAbortUntil_.time_since_epoch().count() != 0 &&
            now < outpostTaskDamageAbortUntil_;
        const bool external_aim_active = config.ExternalAimSettings.Enable;
        if (external_aim_active &&
            externalAimData.LastValidTime.time_since_epoch().count() != 0 &&
            now - externalAimData.LastValidTime >
                std::chrono::milliseconds(std::max(1, config.ExternalAimSettings.ResultFreshTimeoutMs))) {
            externalAimData.Valid = false;
            externalAimData.Fresh = false;
            externalAimData.FireStatus = false;
            externalAimData.HasLatchedAngles = false;
        }
        const AimData& activeAimData = CurrentAimData();
        GimbalAnglesType nextAngles = gimbalAngles;
        VelocityType nextVelocity = naviVelocityInput;
        bool passive_gimbal_motion_active = false;
        const bool find_target_callback = isFindTargetAtomic.load(std::memory_order_relaxed);
        bool find_target = false;
        bool has_recent_latched_target = false;
        const bool has_target_for_angles = CurrentAimTargetForAngles(
            find_target_callback,
            now,
            config.AimDebugSettings.LatchedTargetHoldMs,
            &find_target,
            &has_recent_latched_target);
        const bool visual_target_has_face_priority =
            has_target_for_angles &&
            (external_aim_active || aimMode == AimMode::Buff || aimMode == AimMode::Outpost);
        const auto face_mode_decision = faceModeManager_.Resolve(
            faceModeData,
            config.FaceModeSettings,
            config.PatrolScanSettings,
            visual_target_has_face_priority,
            navi_rotate_control_release_request,
            config.NaviControlSettings.ClearRegionalFaceModeWhenTrue,
            now);
        lastFaceModeDecision_ = face_mode_decision;
        const bool face_mode_requested = face_mode_decision.Requested;
        const bool face_mode_fallback_patrol_scan = face_mode_decision.UsePatrolFallback;
        const bool face_mode_active = face_mode_decision.Active;
        auto reset_patrol_scan_state = [this]() {
            patrolScanDirection_ = 1;
            patrolScanCenterYaw_ = 0.0f;
            patrolScanOffsetYaw_ = 0.0f;
            patrolScanPhaseRad_ = 0.0f;
            patrolScanCenterInitialized_ = false;
            patrolScanActiveMode_ = 0;
            patrolScanResumeSmoothing_ = false;
            patrolScanResumeStartedAt_ = {};
        };
        const auto begin_patrol_scan_resume_smoothing = [this]() {
            if (patrolScanActiveMode_ == 2 && !patrolScanResumeSmoothing_) {
                patrolScanResumeSmoothing_ = true;
                // The two-second post-target hold is intentionally outside the
                // resume deadline. Start timing only when Mode 2 actually resumes.
                patrolScanResumeStartedAt_ = {};
            }
        };
        const auto face_mode_source_name = [](const FaceModeManager::Source source) {
            switch (source) {
            case FaceModeManager::Source::Regional:
                return "regional";
            case FaceModeManager::Source::Buff:
                return "buff";
            case FaceModeManager::Source::Outpost:
                return "outpost";
            case FaceModeManager::Source::StartGate:
                return "start_gate";
            case FaceModeManager::Source::None:
                return "none";
            }
            return "unknown";
        };
        const bool navigation_releases_regional_face_mode =
            navi_rotate_control_release_request &&
            config.NaviControlSettings.ClearRegionalFaceModeWhenTrue &&
            face_mode_decision.RequestSource == FaceModeManager::Source::Regional;
        const char* face_mode_outcome = "not_requested";
        if (face_mode_active) {
            face_mode_outcome = "accepted";
        } else if (!face_mode_requested &&
                   face_mode_decision.RequestSource != FaceModeManager::Source::None &&
                   !config.FaceModeSettings.Enable) {
            face_mode_outcome = "face_mode_disabled";
        } else if (visual_target_has_face_priority) {
            face_mode_outcome = "visual_target_priority";
        } else if (navigation_releases_regional_face_mode) {
            face_mode_outcome = "navigation_release";
        } else if (face_mode_fallback_patrol_scan) {
            face_mode_outcome = "angles_missing_or_stale";
        } else if (face_mode_requested) {
            face_mode_outcome = "angles_unavailable_fallback_disabled";
        }
        const FaceModeRuntimeLogState face_mode_log_state{
            face_mode_decision.RequestSource,
            static_cast<int>(face_mode_decision.Phase),
            face_mode_requested,
            face_mode_active,
            face_mode_fallback_patrol_scan,
            face_mode_decision.Angles.has_value(),
            face_mode_decision.SuppressFire,
            visual_target_has_face_priority,
            navigation_releases_regional_face_mode};
        static std::optional<FaceModeRuntimeLogState> last_face_mode_log_state;
        if (!last_face_mode_log_state || face_mode_log_state != *last_face_mode_log_state) {
            LoggerPtr->Info(
                "FaceMode runtime: source={} requested={} active={} fallback={} outcome={} "
                "angles_available={} suppress_fire={} visual_priority={} navi_release={} phase={}",
                face_mode_source_name(face_mode_decision.RequestSource),
                face_mode_requested ? 1 : 0,
                face_mode_active ? 1 : 0,
                face_mode_fallback_patrol_scan ? 1 : 0,
                face_mode_outcome,
                face_mode_decision.Angles.has_value() ? 1 : 0,
                face_mode_decision.SuppressFire ? 1 : 0,
                visual_target_has_face_priority ? 1 : 0,
                navigation_releases_regional_face_mode ? 1 : 0,
                static_cast<int>(face_mode_decision.Phase));
            last_face_mode_log_state = face_mode_log_state;
        }
        if (face_mode_active) {
            passive_gimbal_motion_active = true;
            // Mode 2 resumes its prior center/phase after a temporary fixed-angle hold.
            begin_patrol_scan_resume_smoothing();
            gimbalControlData.FireCode.AimMode = 0;
            if (face_mode_decision.SuppressFire) {
                gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
            }
            nextAngles = face_mode_decision.Angles.value_or(gimbalAngles);

        } else if (has_target_for_angles) {
            // Preserve the Mode 2 trajectory while visual aiming temporarily owns the gimbal.
            begin_patrol_scan_resume_smoothing();
            LoggerPtr->Debug(
                "Aim target active, AimMode={}, fresh={}, held={}",
                static_cast<int>(aimMode),
                find_target ? 1 : 0,
                has_recent_latched_target ? 1 : 0);
            if (find_target && !config.AimDebugSettings.StopFire){
                if(aimMode == AimMode::Buff) { // 打符模式
                    const int referee_fresh_ms =
                        std::max(0, config.TaskSettings.BuffConfirm.RefereeFreshTimeoutMs);
                    const bool event_data_fresh =
                        hasReceivedEventData_ &&
                        lastEventDataRxTime_.time_since_epoch().count() != 0 &&
                        now - lastEventDataRxTime_ <= std::chrono::milliseconds(referee_fresh_ms);
                    const bool energy_activating =
                        event_data_fresh &&
                        (eventSelfSmallEnergyStatus_ == 2 || eventSelfLargeEnergyStatus_ == 2);
                    const int post_confirm_grace_ms =
                        std::max(0, config.TaskSettings.BuffConfirm.PostConfirmGraceMs);
                    const bool confirm_grace_active =
                        lastEnergyActivateConfirmTime_.time_since_epoch().count() != 0 &&
                        now - lastEnergyActivateConfirmTime_ <= std::chrono::milliseconds(post_confirm_grace_ms);
                    const bool buff_fire_allowed =
                        config.TaskSettings.BuffTimer.Enable || energy_activating || confirm_grace_active;
                    if(buff_fire_allowed && activeAimData.FireStatus){
                        /// 立刻响应不需要tick
                        RecFireCode.FlipFireStatus();
                        gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
                        externalAimData.FireStatus = false;
                        buff_shoot_count++;
                    } else {
                        gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
                    }
                } else if (external_aim_active) {
                    if (activeAimData.FireStatus) {
                        RecFireCode.FlipFireStatus();
                        gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
                        externalAimData.FireStatus = false;
                    } else {
                        gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
                    }
                } else { // 非打符模式（打前哨和打车），沿用老代码：收到回调就按频率开火
                    if(fireRateClock.trigger()){
                        fireRateClock.tick();
                        RecFireCode.FlipFireStatus();
                        gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
                    }
                }
            } else {
                gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
            }
            gimbalControlData.FireCode.AimMode = 1;
            if (find_target) {
                lastFoundEnemyTime = now;
            }
            
            nextAngles = activeAimData.Angles;
            if (aimMode != AimMode::Buff && aimMode != AimMode::Outpost) {
                LoggerPtr->Debug(
                    "AutoAim Angles -> Pitch: {}, Yaw: {}",
                    activeAimData.Angles.Pitch,
                    activeAimData.Angles.Yaw);
            }
        }
        else { // 未识别到目标
            gimbalControlData.FireCode.AimMode = 0;
            
            if(aimMode != AimMode::Buff || face_mode_fallback_patrol_scan) {
                if (!config.AimDebugSettings.StopScan && now - lastFoundEnemyTime > std::chrono::milliseconds(2000)) {
                    static auto last_searching_log = std::chrono::steady_clock::time_point{};
                    const bool outpost_face_mode_fallback =
                        face_mode_fallback_patrol_scan && aimMode == AimMode::Outpost;
                    int patrol_mode = config.PatrolScanSettings.Mode;
                    if (outpost_damage_abort_scan_active) {
                        patrol_mode = config.PatrolScanSettings.OutpostDamageAbortMode;
                    } else if (outpost_face_mode_fallback) {
                        patrol_mode = config.PatrolScanSettings.OutpostFaceModeFallbackMode;
                    } else if (face_mode_fallback_patrol_scan) {
                        patrol_mode = config.PatrolScanSettings.FaceModeFallbackMode;
                    }
                    const bool boost_patrol_scan =
                        aimMode == AimMode::RotateScan &&
                        damage_rotate_elapsed_ms >= 0 &&
                        damage_rotate_elapsed_ms <= damage_rotate_setting.ScanBoostWindowMs;
                    float yaw_scan_step = static_cast<float>(boost_patrol_scan
                        ? patrol_scan.Mode1YawBoostStepDegPerTick
                        : patrol_scan.Mode1YawStepDegPerTick);
                    int yaw_scan_direction = 1;

                    if (patrol_mode == 2) {
                        yaw_scan_step = static_cast<float>(boost_patrol_scan
                            ? patrol_scan.Mode2YawBoostStepDegPerTick
                            : patrol_scan.Mode2YawStepDegPerTick);

                        if (!patrolScanCenterInitialized_ || patrolScanActiveMode_ != patrol_mode) {
                            patrolScanCenterInitialized_ = true;
                            patrolScanActiveMode_ = patrol_mode;
                            patrolScanCenterYaw_ = gimbalAngles.Yaw;
                            patrolScanOffsetYaw_ = 0.0f;
                            patrolScanPhaseRad_ = 0.0f;
                            patrolScanDirection_ = 1; // 新一轮巡逻默认先向右
                            patrolScanResumeSmoothing_ = false;
                            patrolScanResumeStartedAt_ = {};
                        }

                        if (!patrolScanResumeSmoothing_) {
                            const float half_range = static_cast<float>(patrol_scan.Mode2YawHalfRangeDeg);
                            const float phase_step = yaw_scan_step / std::max(half_range, 1.0f);
                            const float center_drift_step =
                                static_cast<float>(patrol_scan.Mode2CenterDriftPerCycleDeg) *
                                phase_step / kTwoPi;

                            // mode2: 讓中心點每完成一個正弦掃描週期固定右偏同樣角度。
                            patrolScanCenterYaw_ = normalize_angle_near(
                                patrolScanCenterYaw_ + center_drift_step,
                                gimbalAngles.Yaw);

                            patrolScanPhaseRad_ = std::fmod(patrolScanPhaseRad_ + phase_step, kTwoPi);
                            patrolScanOffsetYaw_ = half_range * std::sin(patrolScanPhaseRad_);
                            patrolScanDirection_ = (std::cos(patrolScanPhaseRad_) >= 0.0f) ? 1 : -1;
                            if (patrolScanPhaseRad_ < 0.0f) {
                                patrolScanPhaseRad_ += kTwoPi;
                            }
                        }
                        yaw_scan_direction = patrolScanDirection_;
                    } else if (patrol_mode == 3) {
                        yaw_scan_step = static_cast<float>(patrol_scan.Mode3YawStepDegPerTick);
                        if (patrolScanActiveMode_ != patrol_mode || patrolScanCenterInitialized_) {
                            reset_patrol_scan_state();
                            patrolScanActiveMode_ = patrol_mode;
                        }
                        yaw_scan_direction = 1;
                    } else {
                        if (patrolScanActiveMode_ != patrol_mode || patrolScanCenterInitialized_) {
                            reset_patrol_scan_state();
                            patrolScanActiveMode_ = patrol_mode;
                        }
                        yaw_scan_direction = boost_patrol_scan
                            ? (((damage_rotate_elapsed_ms / damage_rotate_setting.ScanYawPhaseMs) % 2 == 0) ? 1 : -1)
                            : 1;
                    }
                    if (now - last_searching_log > std::chrono::seconds(2)) {
                        LoggerPtr->Debug(
                            "Searching Target... patrol_mode={} yaw_step={} dir={} (damage_boost={} elapsed_ms={} face_fallback={})",
                            patrol_mode,
                            yaw_scan_step,
                            yaw_scan_direction,
                            boost_patrol_scan ? 1 : 0,
                            damage_rotate_elapsed_ms,
                            face_mode_fallback_patrol_scan ? 1 : 0);
                        last_searching_log = now;
                        gimbalControlData.FireCode.AimMode = 0;
                    }
                    const auto current_time = std::chrono::steady_clock::now();
                    const float next_scan_yaw = (patrol_mode == 2 && patrolScanCenterInitialized_)
                        ? normalize_angle_near(patrolScanCenterYaw_ + patrolScanOffsetYaw_, gimbalAngles.Yaw)
                        : static_cast<float>(gimbalAngles.Yaw + yaw_scan_direction * yaw_scan_step);
                    const auto pitch_elapsed_ms = static_cast<float>(
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            current_time - gameStartTime).count());
                    float pitch_center = static_cast<float>(patrol_scan.Mode1PitchCenterDeg);
                    float pitch_half_range = static_cast<float>(patrol_scan.Mode1PitchHalfRangeDeg);
                    float pitch_period_ms = static_cast<float>(patrol_scan.Mode1PitchPeriodMs);
                    if (patrol_mode == 2) {
                        pitch_center = static_cast<float>(patrol_scan.Mode2PitchCenterDeg);
                        pitch_half_range = static_cast<float>(patrol_scan.Mode2PitchHalfRangeDeg);
                        pitch_period_ms = static_cast<float>(patrol_scan.Mode2PitchPeriodMs);
                    } else if (patrol_mode == 3) {
                        pitch_center = static_cast<float>(patrol_scan.Mode3PitchOffsetDeg);
                        pitch_half_range = static_cast<float>(patrol_scan.Mode3PitchHalfRangeDeg);
                        pitch_period_ms = static_cast<float>(patrol_scan.Mode3PitchPeriodMs);
                    }
                    const float next_scan_pitch =
                        pitch_center +
                        pitch_half_range *
                            std::sin(pitch_elapsed_ms * kTwoPi / std::max(pitch_period_ms, 1.0f));
                    nextAngles = GimbalAnglesType{
                        static_cast<AngleType>(next_scan_yaw),
                        static_cast<AngleType>(next_scan_pitch)
                    };

                    if (patrol_mode == 2 && patrolScanResumeSmoothing_) {
                        const float yaw_error =
                            normalize_angle_near(next_scan_yaw, gimbalAngles.Yaw) - gimbalAngles.Yaw;
                        if (patrolScanResumeStartedAt_.time_since_epoch().count() == 0) {
                            patrolScanResumeStartedAt_ = now;
                        }
                        const int resume_elapsed_ms = static_cast<int>(
                            std::chrono::duration_cast<std::chrono::milliseconds>(
                                now - patrolScanResumeStartedAt_).count());
                        const auto resume = EvaluateMode2PatrolResume(
                            yaw_error,
                            std::max(0, resume_elapsed_ms),
                            static_cast<float>(patrol_scan.Mode2ResumeYawToleranceDeg),
                            patrol_scan.Mode2ResumeMaxMs);
                        passive_gimbal_motion_active = true;
                        if (!resume.ContinueSmoothing) {
                            patrolScanResumeSmoothing_ = false;
                            patrolScanResumeStartedAt_ = {};
                            if (resume.RebaseFromFeedback) {
                                patrolScanCenterYaw_ = gimbalAngles.Yaw;
                                patrolScanOffsetYaw_ = 0.0f;
                                patrolScanPhaseRad_ = 0.0f;
                                patrolScanDirection_ = 1;
                                nextAngles.Yaw = gimbalAngles.Yaw;
                                passive_gimbal_motion_active = false;
                                LoggerPtr->Info(
                                    "Mode2 patrol resume timeout -> rebase: yaw_error={} elapsed_ms={} center={}",
                                    yaw_error,
                                    resume_elapsed_ms,
                                    patrolScanCenterYaw_);
                            } else {
                                LoggerPtr->Info(
                                    "Mode2 patrol resume synchronized: yaw_error={} center={} phase={}",
                                    yaw_error,
                                    patrolScanCenterYaw_,
                                    patrolScanPhaseRad_);
                            }
                        }
                    }

                    if (aimMode == AimMode::Outpost &&
                        (patrol_mode != 3 || patrol_scan.OutpostPitchOffsetApplyToMode3)) {
                        nextAngles.Pitch += static_cast<AngleType>(patrol_scan.OutpostPitchOffsetDeg);
                    }
                } else {
                    // Keep a paused Mode 2 center/phase so scan resumes on the shifted
                    // trajectory instead of restarting at its old center after target hold.
                    if (patrolScanActiveMode_ != 2) {
                        reset_patrol_scan_state();
                    }
                    nextAngles = gimbalAngles;
                    LoggerPtr->Debug(
                        "Hold current gimbal angles (latched reuse disabled or unavailable) -> Pitch: {}, Yaw: {}",
                        gimbalAngles.Pitch,
                        gimbalAngles.Yaw);
                }
            }else { // 打符模式
                reset_patrol_scan_state();
                if (now_time < 5) {
                    LoggerPtr->Info("Set Angles, Buff Mode, 10 min!");
                    // Yaw
                    float current_yaw = normalize_angle_0_360(gimbalAngles.Yaw);
                    float delta = buff_yaw - current_yaw;
                    if (delta > 180.0f) delta -= 360.0f; // 角度差大于180，反向旋转
                    if (delta < -180.0f) delta += 360.0f;
                    int opt = delta > 0 ? 1 : -1;
                    int target_yaw = gimbalAngles.Yaw + delta;
                    nextAngles = gimbalAngles;

                    if (std::abs(delta) > 10 * delta_yaw) {
                        nextAngles.Yaw = static_cast<AngleType>(gimbalAngles.Yaw + delta_yaw * opt);
                    } else {
                        nextAngles.Yaw = static_cast<AngleType>(target_yaw);
                    }
                    nextAngles.Pitch = 19.0f;
                }else {
                    nextAngles = AimFreshAndValid(activeAimData)
                        ? activeAimData.Angles
                        : gimbalAngles;
                }
            }
            gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
        }

        if (naviChaseVelocityActive_) {
            nextVelocity = naviChaseVelocity;
        }

        if (fortress_defense_stand_still) {
            nextVelocity = VelocityType{0, 0};
            static auto last_fortress_stand_log = std::chrono::steady_clock::time_point{};
            if (now - last_fortress_stand_log > std::chrono::seconds(2)) {
                LoggerPtr->Info(
                    "Fortress defense stand-fire: enemy_count={} threshold={}, lock=1, fire_enabled=1.",
                    fortressGainPointEnemyCount_,
                    config.RegionalDefenseSettings.FortressStandEnemyCountMin);
                last_fortress_stand_log = now;
            }
        }

        // lower_head 只在未锁目标且没有 FaceMode 接管时生效，避免覆盖固定点朝向/Outpost fallback。
        if(naviLowerHead && !has_target_for_angles && !face_mode_requested) {
            nextAngles = GimbalAnglesType{gimbalAngles.Yaw, -15.0f}; //-22.5 - 26.0
            passive_gimbal_motion_active = false;
        }
        if (passive_gimbal_motion_active) {
            nextAngles = passiveGimbalMotion_.Step(
                gimbalAngles,
                nextAngles,
                now,
                config.PatrolScanSettings.PassiveYawRateDegPerSec,
                config.PatrolScanSettings.PassivePitchRateDegPerSec,
                config.PatrolScanSettings.PassiveMaxIntervalMs);
        } else {
            passiveGimbalMotion_.Reset();
        }
        gimbalControlData.GimbalAngles = nextAngles;
        naviVelocity = nextVelocity;

        const auto follow_mode_before_navi_rotate_control_publish =
            gimbalControlData.FireCode.FollowMode;
        if (navi_rotate_control_follow_output) {
            gimbalControlData.FireCode.FollowMode = 1;
        }
        PublishMessageAll();
        gimbalControlData.FireCode.FollowMode =
            follow_mode_before_navi_rotate_control_publish;
        externalAimData.Fresh = false;
        faceModeData.Fresh = false;
        isFindTargetAtomic = false;
    }

    /**
     * @brief 决策进程主循环 \n
     * @brief 1. 设置黑板数据 \n
     * @brief 2. 休眠 \n
     * @brief 3. 更新黑板数据 \n
     * @brief 4. 处理行为树 \n
     */
    void Application::GameLoop() {

        if (!GlobalBlackboard_) {
            GlobalBlackboard_ = BT::Blackboard::create();
        }
        ResetTickBlackboard();

        GlobalBlackboard_->set<UnitTeam>("MyTeam", team); // 队伍颜色
        GlobalBlackboard_->set<std::chrono::steady_clock::time_point>(
            "LastCommandTime", std::chrono::steady_clock::now()); // 上次发送命令的时间
        GlobalBlackboard_->set<std::chrono::seconds>("CommandInterval", std::chrono::seconds{0}); // 命令间隔
        GlobalBlackboard_->set<ArmorType>("AimTarget", ArmorType::Hero); // 辅瞄击打目标
        GlobalBlackboard_->set<std::uint8_t>("naviCommandGoal", Home(team)); // 导航目的地
        GlobalBlackboard_->set<std::shared_ptr<Logger>>("LoggerPtr", LoggerPtr);
        GlobalBlackboard_->set("TickBlackboard", TickBlackboard_);
        UpdateBlackBoard();

        while (rclcpp::ok()) {
            MarkLoopBeat();
            rclcpp::spin_some(node_); // 处理回调函数

            if (TryHandleSoftRecovery()) {
                treeTickRateClock.sleep();
                continue;
            }

            if (IsCriticalInputStale()) {
                static auto last_stale_log = std::chrono::steady_clock::time_point{};
                const auto now = std::chrono::steady_clock::now();
                if (LoggerPtr && (now - last_stale_log > std::chrono::seconds(2))) {
                    LoggerPtr->Warning("Critical input stale: gimbal angles > {} ms, publish safe-control.",
                                       kRuntimeGimbalStaleMs);
                    last_stale_log = now;
                }
                PublishSafeControl("gimbal_stale");
                treeTickRateClock.sleep();
                continue;
            }

            if (runtimeRearmStartGate_ && !is_game_begin) {
                const auto now = std::chrono::steady_clock::now();
                if (!runtimeStartGateActive_) {
                    runtimeStartGateActive_ = true;
                    runtimeStartGateLastLogTime_ = now;
                    LoggerPtr->Info("Runtime start gate armed: waiting /ly/game/is_start=true.");
                } else if (LoggerPtr && (now - runtimeStartGateLastLogTime_ > std::chrono::seconds(2))) {
                    LoggerPtr->Debug("Runtime start gate waiting for /ly/game/is_start=true...");
                    runtimeStartGateLastLogTime_ = now;
                }

                SET_POSITION(Home, team);
                if (publishNaviGoal_ && naviCommandRateClock.trigger()) {
                    naviCommandRateClock.tick();
                    const bool navi_to_navi =
                        config.NaviSettings.UseXY &&
                        config.NaviSettings.ToNavi &&
                        config.ChaseSettings.Enable &&
                        config.ChaseSettings.ToNavi;
                    if (config.NaviSettings.UseXY && !navi_to_navi) {
                        PubNaviGoalPos();
                    } else {
                        PubNaviGoal();
                    }
                }

                naviVelocityInput = VelocityType{0, 0};
                naviVelocity = VelocityType{0, 0};
                postureCommand = 0;
                PublishSafeControl("runtime_start_gate");
                UpdateBlackBoard();
                if (decisionTraceEnabled_) {
                    WriteDecisionTrace("tick");
                }
                treeTickRateClock.sleep();
                continue;
            }
            if (runtimeRearmStartGate_ && runtimeStartGateActive_ && is_game_begin) {
                runtimeStartGateActive_ = false;
                gameStartTime = std::chrono::steady_clock::now();
                if (LoggerPtr) {
                    LoggerPtr->Info("Runtime start gate opened: resume decision loop.");
                }
                if (decisionTraceEnabled_) {
                    WriteDecisionTrace("game_start");
                }
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - lastTreeTickLogTime_ > std::chrono::seconds(2)) {
                LoggerPtr->Debug("BehaviorTree Root Tick...");
                lastTreeTickLogTime_ = now;
            }
            TreeTickGuarded();
            if (decisionTraceEnabled_) {
                WriteDecisionTrace("tick");
            }
            treeTickRateClock.sleep();
        }
    }

    void Application::TreeTickGuarded() {
        MarkTickStart();
        try {
            TreeTick();
        } catch (const std::exception& ex) {
            if (LoggerPtr) {
                LoggerPtr->Error("BehaviorTree tick exception: {}", ex.what());
            }
            RequestSoftRecovery(RuntimeFaultCode::TreeException);
        } catch (...) {
            if (LoggerPtr) {
                LoggerPtr->Error("BehaviorTree tick exception: <unknown>");
            }
            RequestSoftRecovery(RuntimeFaultCode::TreeException);
        }
        MarkTickEnd();
    }

    void Application::TreeTick() {
        if (BTree.subtrees.empty()) {
            LoggerPtr->Error("BehaviorTree is empty, skip tick.");
            RequestSoftRecovery(RuntimeFaultCode::TreeEmpty);
            return;
        }

        const auto status = BTree.tickWhileRunning(std::chrono::milliseconds(1));
        if (status == BT::NodeStatus::FAILURE) {
            LoggerPtr->Warning("BehaviorTree tick returned FAILURE.");
        }
    }

    bool Application::IsDecisionAutonomyModuleEnabled(std::string_view module) const {
        const auto& autonomy = config.DecisionAutonomySettings;
        if (!autonomy.Enable) {
            return false;
        }
        const auto normalized_module = NormalizeDecisionModule(module);
        if (HasAutonomyToken(autonomy.HardRuleModules, normalized_module)) {
            return false;
        }
        if (HasAutonomyToken(autonomy.EnabledModules, "all")) {
            return true;
        }
        return HasAutonomyToken(autonomy.EnabledModules, normalized_module);
    }

    void Application::SelectStrategyMode() {
        if (GetCompetitionProfile() == CompetitionProfile::League) {
            SetStrategyMode(StrategyMode::LeagueSimple);
            return;
        }

        // Regional uses AreaManager/default-task policy only; old point-table
        // strategies stay out of the live regional chain.
        SetStrategyMode(StrategyMode::Regional);
    }

    void Application::SetAimMode() {
        // int now_time = 420 - timeLeft;
        int now_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - gameStartTime).count();
        LoggerPtr->Info("SetAimMode - now_time: {}", now_time);
        if(config.TaskSettings.Buff) { // 打符
            outpostVisualScoutNavigationActive_ = false;
            const auto now = std::chrono::steady_clock::now();
            const int referee_fresh_ms =
                std::max(0, config.TaskSettings.BuffConfirm.RefereeFreshTimeoutMs);
            const int post_confirm_grace_ms =
                std::max(0, config.TaskSettings.BuffConfirm.PostConfirmGraceMs);
            const int task_hold_timeout_ms =
                std::max(0, config.TaskSettings.BuffConfirm.TaskHoldTimeoutMs);
            const int damage_abort_threshold =
                std::max(0, config.TaskSettings.BuffConfirm.DamageAbortThreshold);
            const int damage_abort_window_ms =
                std::max(0, config.TaskSettings.BuffConfirm.DamageAbortWindowMs);
            const int damage_abort_hold_ms =
                std::max(0, config.TaskSettings.BuffConfirm.DamageAbortHoldMs);
            const bool event_data_fresh =
                hasReceivedEventData_ &&
                lastEventDataRxTime_.time_since_epoch().count() != 0 &&
                now - lastEventDataRxTime_ <= std::chrono::milliseconds(referee_fresh_ms);
            const bool sentry_info_fresh =
                hasReceivedSentryInfo_ &&
                lastSentryInfoRxTime_.time_since_epoch().count() != 0 &&
                now - lastSentryInfoRxTime_ <= std::chrono::milliseconds(referee_fresh_ms);
            const bool energy_activated =
                event_data_fresh &&
                (eventSelfSmallEnergyStatus_ == 1 || eventSelfLargeEnergyStatus_ == 1);
            const bool energy_activating =
                event_data_fresh &&
                (eventSelfSmallEnergyStatus_ == 2 || eventSelfLargeEnergyStatus_ == 2);
            const bool can_activate_energy =
                sentry_info_fresh && sentryCanActivateEnergyMechanism_;
            const bool energy_done_without_next =
                energy_activated && !energy_activating && !can_activate_energy;
            const bool confirm_grace_active =
                lastEnergyActivateConfirmTime_.time_since_epoch().count() != 0 &&
                now - lastEnergyActivateConfirmTime_ <= std::chrono::milliseconds(post_confirm_grace_ms);
            const bool damage_abort_active =
                buffTaskDamageAbortUntil_.time_since_epoch().count() != 0 &&
                now < buffTaskDamageAbortUntil_;
            const bool recent_damage_abort = [&]() {
                if (damage_abort_threshold <= 0 || damage_abort_window_ms <= 0) {
                    return false;
                }
                for (auto it = postureRecentDamageSamples_.rbegin();
                     it != postureRecentDamageSamples_.rend();
                     ++it) {
                    if ((now - it->Time) > std::chrono::milliseconds(damage_abort_window_ms)) {
                        break;
                    }
                    if (it->Delta > static_cast<std::uint16_t>(damage_abort_threshold)) {
                        return true;
                    }
                }
                return false;
            }();

            if (recent_damage_abort) {
                buffTaskLocked_ = false;
                buffTaskStartTime_ = {};
                buffTaskDamageAbortUntil_ = now + std::chrono::milliseconds(damage_abort_hold_ms);
                aimMode = AimMode::RotateScan;
                LoggerPtr->Info(
                    "Buff interrupted by damage > {}. Hold armor mode for {} ms.",
                    damage_abort_threshold,
                    damage_abort_hold_ms);
            } else if (damage_abort_active) {
                aimMode = AimMode::RotateScan;
            } else if (energy_done_without_next) {
                buffTaskLocked_ = false;
                buffTaskStartTime_ = {};
                LoggerPtr->Info(
                    "Buff energy active and no next activation window. event_fresh={} sentry_fresh={} small_status={} large_status={} can_activate={}",
                    event_data_fresh ? 1 : 0,
                    sentry_info_fresh ? 1 : 0,
                    static_cast<int>(eventSelfSmallEnergyStatus_),
                    static_cast<int>(eventSelfLargeEnergyStatus_),
                    can_activate_energy ? 1 : 0);
                aimMode = AimMode::RotateScan;
            } else if (config.TaskSettings.BuffTimer.Enable) {
                const int timer_start = config.TaskSettings.BuffTimer.StartSec;
                const int timer_end = config.TaskSettings.BuffTimer.EndSec;
                const int max_shoot_count = config.TaskSettings.BuffTimer.MaxShootCount;
                const bool in_timer_window = now_time >= timer_start && now_time < timer_end;
                const bool shoot_count_allowed =
                    max_shoot_count < 0 || buff_shoot_count <= max_shoot_count;
                if (in_timer_window && shoot_count_allowed) {
                    if (!buffTaskLocked_) {
                        buffTaskStartTime_ = now;
                    }
                    buffTaskLocked_ = true;
                    aimMode = AimMode::Buff;
                } else {
                    buffTaskLocked_ = false;
                    buffTaskStartTime_ = {};
                    LoggerPtr->Info(
                        "Buff timer gate closed: now={} window=[{}, {}) shoot_count={} max={}",
                        now_time,
                        timer_start,
                        timer_end,
                        buff_shoot_count,
                        max_shoot_count);
                    aimMode = AimMode::RotateScan;
                }
            } else {
                const bool task_hold_timeout =
                    buffTaskLocked_ &&
                    task_hold_timeout_ms > 0 &&
                    buffTaskStartTime_.time_since_epoch().count() != 0 &&
                    now - buffTaskStartTime_ > std::chrono::milliseconds(task_hold_timeout_ms);
                const bool official_state_keeps_task =
                    energy_activating || can_activate_energy || confirm_grace_active;
                if (task_hold_timeout && !official_state_keeps_task) {
                    buffTaskLocked_ = false;
                    buffTaskStartTime_ = {};
                    LoggerPtr->Warning(
                        "Buff task timeout without fresh active state: timeout_ms={} event_fresh={} sentry_fresh={} small_status={} large_status={} can_activate={}",
                        task_hold_timeout_ms,
                        event_data_fresh ? 1 : 0,
                        sentry_info_fresh ? 1 : 0,
                        static_cast<int>(eventSelfSmallEnergyStatus_),
                        static_cast<int>(eventSelfLargeEnergyStatus_),
                        can_activate_energy ? 1 : 0);
                    aimMode = AimMode::RotateScan;
                } else if (official_state_keeps_task || buffTaskLocked_) {
                    if (!buffTaskLocked_) {
                        buffTaskStartTime_ = now;
                    }
                    buffTaskLocked_ = true;
                    LoggerPtr->Info(
                        "Buff state gate open: event_fresh={} sentry_fresh={} small_status={} large_status={} can_activate={} confirm_grace={} locked={}",
                        event_data_fresh ? 1 : 0,
                        sentry_info_fresh ? 1 : 0,
                        static_cast<int>(eventSelfSmallEnergyStatus_),
                        static_cast<int>(eventSelfLargeEnergyStatus_),
                        can_activate_energy ? 1 : 0,
                        confirm_grace_active ? 1 : 0,
                        buffTaskLocked_ ? 1 : 0);
                    aimMode = AimMode::Buff;
                } else {
                    LoggerPtr->Info(
                        "Buff state gate closed: event_fresh={} sentry_fresh={} small_status={} large_status={} can_activate={} confirm_grace={}",
                        event_data_fresh ? 1 : 0,
                        sentry_info_fresh ? 1 : 0,
                        static_cast<int>(eventSelfSmallEnergyStatus_),
                        static_cast<int>(eventSelfLargeEnergyStatus_),
                        can_activate_energy ? 1 : 0,
                        confirm_grace_active ? 1 : 0);
                    aimMode = AimMode::RotateScan;
                }
            }
        }else if(config.TaskSettings.Outpost) { // 打前哨站
            const auto& outpost_confirm = config.TaskSettings.OutpostConfirm;
            const auto now = std::chrono::steady_clock::now();
            const int referee_fresh_ms = std::max(0, outpost_confirm.RefereeFreshTimeoutMs);
            const bool outpost_opening_high_priority = IsOutpostOpeningHighPriorityActive();
            const bool regional_defense_can_preempt_opening =
                CanRegionalDefensePreemptOutpostOpening(outpost_opening_high_priority);
            if (regional_defense_can_preempt_opening && IsRegionalDefenseAimSuppressActive()) {
                outpostVisualScoutNavigationActive_ = false;
                outpostPostArmorFaceSearchUntil_ = {};
                outpostArmorInterruptActive_ = false;
                aimMode = AimMode::RotateScan;
                LoggerPtr->Info(
                    "Regional defense active: suppress Outpost aim mode outside opening Task ownership.");
                return;
            }
            if (areaManager_.RegionalAreaTaskActive() &&
                areaManager_.RegionalAreaTask().Type == RegionalAreaTaskType::MyReadyRoadland &&
                !areaManager_.RegionalAreaTaskCanYieldToHigherPriority() &&
                !outpost_opening_high_priority) {
                outpostVisualScoutNavigationActive_ = false;
                outpostPostArmorFaceSearchUntil_ = {};
                outpostArmorInterruptActive_ = false;
                aimMode = AimMode::RotateScan;
                LoggerPtr->Info("ReadyRoadland hard crossing active: suppress Outpost aim mode.");
                return;
            }

            const bool enemy_outpost_hp_fresh =
                hasReceivedEnemyOutpostHealth_ &&
                lastEnemyOutpostHealthRxTime_.time_since_epoch().count() != 0 &&
                now - lastEnemyOutpostHealthRxTime_ <= std::chrono::milliseconds(referee_fresh_ms);
            const bool enemy_outpost_hp_trusted =
                outpost_confirm.TrustEnemyOutpostHp && enemy_outpost_hp_fresh;
            const bool self_hp_fresh =
                hasReceivedMyselfHealth_ &&
                lastMyselfHealthRxTime.time_since_epoch().count() != 0 &&
                now - lastMyselfHealthRxTime <= std::chrono::milliseconds(referee_fresh_ms);
            const bool ammo_fresh =
                hasReceivedAmmoLeft_ &&
                lastAmmoLeftRxTime.time_since_epoch().count() != 0 &&
                now - lastAmmoLeftRxTime <= std::chrono::milliseconds(referee_fresh_ms);
            const bool self_hp_ready =
                outpost_confirm.MinSelfHp <= 0 || (self_hp_fresh && myselfHealth >= outpost_confirm.MinSelfHp);
            const bool ammo_ready =
                outpost_confirm.MinAmmo <= 0 || (ammo_fresh && ammoLeft >= outpost_confirm.MinAmmo);
            const bool opening_window_limited = outpost_confirm.MaxGameTimeSec > 0;
            const bool in_time_window =
                !opening_window_limited || now_time < outpost_confirm.MaxGameTimeSec;
            const int opening_hold_sec = std::max(0, outpost_confirm.OpeningHoldSec);
            const bool opening_hard_hold_active = IsOutpostOpeningHoldActive(
                config.TaskSettings.Outpost,
                outpost_confirm,
                now_time);
            const bool post_window_scout_time =
                opening_window_limited &&
                now_time >= outpost_confirm.MaxGameTimeSec &&
                !opening_hard_hold_active &&
                outpost_confirm.PostWindowScoutEnable;
            const bool outpost_time_gate_open =
                in_time_window || opening_hard_hold_active || post_window_scout_time;
            const int visual_scout_face_distance_cm =
                std::max(0, outpost_confirm.VisualScoutFaceDistanceCm);
            const auto outpost_goal_reach = EvaluateBaseGoalReach(
                LangYa::BuffOutpost.ID,
                team,
                true,
                visual_scout_face_distance_cm,
                GoalReachTimeoutSecForBaseGoal(LangYa::BuffOutpost.ID));
            const bool outpost_goal_unreachable =
                outpost_goal_reach.Status == GoalReachStatus::Unreachable;
            const bool outpost_visual_scout_point_reached =
                outpost_goal_reach.Status == GoalReachStatus::Reached;
            const bool outpost_visual_scout_face_ready =
                outpost_visual_scout_point_reached ||
                outpost_goal_reach.WithinFaceDistance;
            const int armor_warning_distance_cm =
                std::max(0, outpost_confirm.ArmorWarningDistanceCm);
            const auto nearest_armor_distance_cm = [&]() -> std::optional<double> {
                std::optional<double> nearest;
                for (const auto& armor : armorList) {
                    if (armor.Type == ArmorType::UnKnown ||
                        armor.Type == ArmorType::Outpost ||
                        !std::isfinite(armor.Distance) ||
                        armor.Distance <= 0.0F) {
                        continue;
                    }
                    const double distance_cm = static_cast<double>(armor.Distance) * 100.0;
                    if (!nearest.has_value() || distance_cm < *nearest) {
                        nearest = distance_cm;
                    }
                }
                return nearest;
            }();
            const bool armor_target_too_far =
                nearest_armor_distance_cm.has_value() &&
                armor_warning_distance_cm > 0 &&
                *nearest_armor_distance_cm > static_cast<double>(armor_warning_distance_cm);
            const bool external_target_list_fresh =
                hasExternalAimTargets_ &&
                lastExternalAimTargetsRxTime_.time_since_epoch().count() != 0 &&
                now - lastExternalAimTargetsRxTime_ <=
                    std::chrono::milliseconds(std::max(1, config.ExternalAimSettings.TargetFreshTimeoutMs));
            const bool armor_target_visible =
                external_target_list_fresh &&
                nearest_armor_distance_cm.has_value() &&
                !armor_target_too_far;
            const int damage_abort_threshold = std::max(0, outpost_confirm.DamageAbortThreshold);
            const int damage_abort_window_ms = std::max(0, outpost_confirm.DamageAbortWindowMs);
            const int damage_abort_hold_ms = std::max(0, outpost_confirm.DamageAbortHoldMs);
            const bool damage_abort_active =
                outpostTaskDamageAbortUntil_.time_since_epoch().count() != 0 &&
                now < outpostTaskDamageAbortUntil_;
            const bool recent_damage_abort = [&]() {
                if (damage_abort_threshold <= 0 || damage_abort_window_ms <= 0) {
                    return false;
                }
                for (auto it = postureRecentDamageSamples_.rbegin();
                     it != postureRecentDamageSamples_.rend();
                     ++it) {
                    if ((now - it->Time) > std::chrono::milliseconds(damage_abort_window_ms)) {
                        break;
                    }
                    if (it->Delta > static_cast<std::uint16_t>(damage_abort_threshold)) {
                        return true;
                    }
                }
                return false;
            }();
            const bool outpost_visual_recent =
                targetArmor.Type == ArmorType::Outpost &&
                externalAimData.HasLatchedAngles &&
                externalAimData.LastValidTime.time_since_epoch().count() != 0 &&
                now - externalAimData.LastValidTime <=
                    std::chrono::milliseconds(std::max(0, config.AimDebugSettings.LatchedTargetHoldMs));
            const int visual_scout_hold_ms = std::max(0, outpost_confirm.VisualScoutHoldMs);
            const int visual_scout_cooldown_ms = std::max(0, outpost_confirm.VisualScoutCooldownMs);
            const int post_window_scout_hold_ms =
                std::max(0, outpost_confirm.PostWindowScoutHoldMs);
            const int post_window_scout_interval_ms =
                std::max(0, outpost_confirm.PostWindowScoutIntervalSec) * 1000;
            const bool post_window_scout_mode =
                post_window_scout_time && !in_time_window;
            const int active_visual_scout_hold_ms = post_window_scout_mode
                ? post_window_scout_hold_ms
                : visual_scout_hold_ms;
            const int active_visual_scout_cooldown_ms = post_window_scout_mode
                ? post_window_scout_interval_ms
                : visual_scout_cooldown_ms;
            const int post_armor_face_search_ms =
                std::max(0, outpost_confirm.PostArmorFaceSearchMs);
            const bool outpost_visual_scout_cooling_down =
                outpostVisualScoutCooldownUntil_.time_since_epoch().count() != 0 &&
                now < outpostVisualScoutCooldownUntil_;
            const bool outpost_base_gate_allowed =
                self_hp_ready &&
                ammo_ready &&
                outpost_time_gate_open &&
                !outpost_goal_unreachable &&
                !(enemy_outpost_hp_trusted && enemyOutpostHealth == 0);
            if (!armor_target_visible && outpostArmorInterruptActive_) {
                outpostArmorInterruptActive_ = false;
                if (outpost_base_gate_allowed && post_armor_face_search_ms > 0) {
                    outpostPostArmorFaceSearchUntil_ =
                        now + std::chrono::milliseconds(post_armor_face_search_ms);
                    outpostVisualScoutCooldownUntil_ = {};
                    outpostVisualScoutStartTime_ = {};
                    LoggerPtr->Info(
                        "Outpost armor interrupt released: search outpost with FaceMode for {} ms.",
                        post_armor_face_search_ms);
                }
            }
            const bool post_armor_face_search_active =
                outpost_base_gate_allowed &&
                outpostPostArmorFaceSearchUntil_.time_since_epoch().count() != 0 &&
                now < outpostPostArmorFaceSearchUntil_;
            const bool outpost_visual_scout_available =
                outpost_confirm.VisualScoutWithoutHp &&
                active_visual_scout_hold_ms > 0 &&
                (!outpost_visual_scout_cooling_down || opening_hard_hold_active) &&
                (!post_window_scout_mode || outpost_visual_scout_face_ready);
            const bool outpost_visual_scout_candidate_allowed =
                outpost_base_gate_allowed &&
                (opening_hard_hold_active ||
                 (enemy_outpost_hp_trusted && enemyOutpostHealth > 0) ||
                 outpost_visual_recent ||
                 outpost_visual_scout_available ||
                 post_armor_face_search_active);
            auto clear_outpost_visual_scout_attempt = [&]() {
                outpostVisualScoutStartTime_ = {};
            };
            auto reset_outpost_visual_scout_state = [&]() {
                outpostVisualScoutStartTime_ = {};
                outpostVisualScoutCooldownUntil_ = {};
                outpostPostArmorFaceSearchUntil_ = {};
                outpostVisualScoutNavigationActive_ = false;
                outpostArmorInterruptActive_ = false;
            };
            outpostVisualScoutNavigationActive_ = false;

            if (recent_damage_abort) {
                reset_outpost_visual_scout_state();
                outpostTaskDamageAbortUntil_ = now + std::chrono::milliseconds(damage_abort_hold_ms);
                aimMode = AimMode::RotateScan;
                LoggerPtr->Info(
                    "Outpost interrupted by damage > {}. Hold armor mode for {} ms.",
                    damage_abort_threshold,
                    damage_abort_hold_ms);
            } else if (damage_abort_active) {
                reset_outpost_visual_scout_state();
                aimMode = AimMode::RotateScan;
            } else if (armor_target_visible) {
                clear_outpost_visual_scout_attempt();
                outpostPostArmorFaceSearchUntil_ = {};
                outpostArmorInterruptActive_ = outpost_base_gate_allowed;
                outpostVisualScoutNavigationActive_ = outpost_visual_scout_candidate_allowed;
                aimMode = AimMode::RotateScan;
                LoggerPtr->Info(
                    "Armor target visible: interrupt Outpost aim mode. nearest_cm={} warning_cm={} keep_scout_nav={}.",
                    nearest_armor_distance_cm.value_or(-1.0),
                    armor_warning_distance_cm,
                    outpostVisualScoutNavigationActive_ ? 1 : 0);
            } else if (!self_hp_ready || !ammo_ready) {
                reset_outpost_visual_scout_state();
                aimMode = AimMode::RotateScan;
                LoggerPtr->Info(
                    "Outpost resource gate closed: hp={} fresh={} min={} ammo={} fresh={} min={}.",
                    myselfHealth,
                    self_hp_fresh ? 1 : 0,
                    outpost_confirm.MinSelfHp,
                    ammoLeft,
                    ammo_fresh ? 1 : 0,
                    outpost_confirm.MinAmmo);
            } else if (!outpost_time_gate_open) {
                reset_outpost_visual_scout_state();
                aimMode = AimMode::RotateScan;
                LoggerPtr->Info(
                    "Outpost time gate closed: now={} max={}.",
                    now_time,
                    outpost_confirm.MaxGameTimeSec);
            } else if (outpost_goal_unreachable) {
                reset_outpost_visual_scout_state();
                aimMode = AimMode::RotateScan;
                LoggerPtr->Info("Outpost task canceled: BuffOutpost goal externally unreachable.");
            } else if (enemy_outpost_hp_trusted && enemyOutpostHealth == 0) {
                reset_outpost_visual_scout_state();
                LoggerPtr->Info("Enemy Outpost HP interface says destroyed; skip Outpost task.");
                aimMode = AimMode::RotateScan;
            } else if (enemy_outpost_hp_trusted && enemyOutpostHealth > 0) {
                reset_outpost_visual_scout_state();
                LoggerPtr->Info("Enemy Outpost HP interface says alive: {}", enemyOutpostHealth);
                outpostVisualScoutNavigationActive_ = true;
                aimMode = outpost_visual_scout_face_ready ? AimMode::Outpost : AimMode::RotateScan;
                LoggerPtr->Info(
                    "Outpost HP alive: point_reached={} face_distance_ready={} face_distance_cm={}.",
                    outpost_visual_scout_point_reached ? 1 : 0,
                    outpost_visual_scout_face_ready ? 1 : 0,
                    visual_scout_face_distance_cm);
            } else if (opening_hard_hold_active) {
                outpostVisualScoutStartTime_ = {};
                outpostVisualScoutCooldownUntil_ = {};
                outpostPostArmorFaceSearchUntil_ = {};
                outpostVisualScoutNavigationActive_ = true;
                aimMode = outpost_visual_scout_face_ready ? AimMode::Outpost : AimMode::RotateScan;
                LoggerPtr->Debug(
                    "Outpost opening hard hold active: elapsed={}s hold={}s point_reached={} face_ready={}.",
                    now_time,
                    opening_hold_sec,
                    outpost_visual_scout_point_reached ? 1 : 0,
                    outpost_visual_scout_face_ready ? 1 : 0);
            } else if (outpost_visual_recent) {
                reset_outpost_visual_scout_state();
                outpostVisualScoutNavigationActive_ = true;
                LoggerPtr->Info(
                    "Keep Outpost task by recent visual target. point_reached={}",
                    outpost_visual_scout_point_reached ? 1 : 0);
                aimMode = outpost_visual_scout_face_ready ? AimMode::Outpost : AimMode::RotateScan;
            } else if (post_armor_face_search_active) {
                outpostVisualScoutNavigationActive_ = true;
                clear_outpost_visual_scout_attempt();
                if (outpost_visual_scout_face_ready) {
                    if (!outpost_visual_scout_point_reached) {
                        outpostPostArmorFaceSearchUntil_ =
                            now + std::chrono::milliseconds(post_armor_face_search_ms);
                    }
                    aimMode = AimMode::Outpost;
                    LoggerPtr->Info(
                        "Outpost post-armor search: FaceMode active for {} ms, point_reached={}.",
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            outpostPostArmorFaceSearchUntil_ - now).count(),
                        outpost_visual_scout_point_reached ? 1 : 0);
                } else {
                    aimMode = AimMode::RotateScan;
                    LoggerPtr->Info(
                        "Outpost post-armor search: travel to BuffOutpost before FaceMode. face_distance_cm={}",
                        visual_scout_face_distance_cm);
                }
            } else if (post_window_scout_mode &&
                       outpost_confirm.VisualScoutWithoutHp &&
                       active_visual_scout_hold_ms > 0 &&
                       !outpost_visual_scout_cooling_down &&
                       !outpost_visual_scout_face_ready) {
                clear_outpost_visual_scout_attempt();
                aimMode = AimMode::RotateScan;
                LoggerPtr->Debug(
                    "Outpost post-window visual scout waits for default patrol: face_distance_ready=0 face_distance_cm={}.",
                    visual_scout_face_distance_cm);
            } else if (outpost_confirm.VisualScoutWithoutHp &&
                       active_visual_scout_hold_ms > 0) {
                if (outpost_visual_scout_cooling_down) {
                    aimMode = AimMode::RotateScan;
                    LoggerPtr->Info(
                        "Outpost visual scout cooling down: cooldown_left_ms={}.",
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            outpostVisualScoutCooldownUntil_ - now).count());
                } else {
                    outpostVisualScoutNavigationActive_ = true;
                    if (!outpost_visual_scout_face_ready) {
                        outpostVisualScoutStartTime_ = {};
                        aimMode = AimMode::RotateScan;
                        LoggerPtr->Info(
                            "Outpost visual scout travel: go to BuffOutpost in armor mode before face distance. face_distance_cm={}",
                            visual_scout_face_distance_cm);
                    } else if (!post_window_scout_mode && !outpost_visual_scout_point_reached) {
                        outpostVisualScoutStartTime_ = {};
                        aimMode = AimMode::Outpost;
                        LoggerPtr->Info(
                            "Outpost visual scout approach: within face distance, use FaceMode before BuffOutpost reached.");
                    } else if (outpostVisualScoutStartTime_.time_since_epoch().count() == 0) {
                        outpostVisualScoutStartTime_ = now;
                        LoggerPtr->Info(
                            "Start Outpost visual scout without HP for {} ms. post_window={}",
                            active_visual_scout_hold_ms,
                            post_window_scout_mode ? 1 : 0);
                        aimMode = AimMode::Outpost;
                    } else {
                        const auto scout_elapsed =
                            std::chrono::duration_cast<std::chrono::milliseconds>(
                                now - outpostVisualScoutStartTime_);
                        if (scout_elapsed <= std::chrono::milliseconds(active_visual_scout_hold_ms)) {
                            aimMode = AimMode::Outpost;
                        } else {
                            outpostVisualScoutStartTime_ = {};
                            const int scout_cooldown_ms = active_visual_scout_cooldown_ms;
                            outpostVisualScoutCooldownUntil_ =
                                now + std::chrono::milliseconds(scout_cooldown_ms);
                            outpostVisualScoutNavigationActive_ = false;
                            aimMode = AimMode::RotateScan;
                            LoggerPtr->Warning(
                                "Outpost visual scout timeout without target: hold_ms={} cooldown_ms={}.",
                                active_visual_scout_hold_ms,
                                scout_cooldown_ms);
                        }
                    }
                }
            } else {
                clear_outpost_visual_scout_attempt();
                aimMode = AimMode::RotateScan;
                LoggerPtr->Info(
                    "Outpost visual scout disabled and HP interface unavailable: trust={} fresh={} hp={}.",
                    outpost_confirm.TrustEnemyOutpostHp ? 1 : 0,
                    enemy_outpost_hp_fresh ? 1 : 0,
                    enemyOutpostHealth);
            }
        }else { // 普通模式
            outpostVisualScoutNavigationActive_ = false;
            outpostPostArmorFaceSearchUntil_ = {};
            outpostArmorInterruptActive_ = false;
            if (IsRegionalDefenseAimSuppressActive()) {
                LoggerPtr->Info("Regional defense active: keep armor vision mode.");
            } else {
                LoggerPtr->Info("AimMode: RotateScan!");
            }
            aimMode = AimMode::RotateScan;
        }

    }
    // 提前处理坐标等数据
    void Application::ProcessData() {
        const auto now = std::chrono::steady_clock::now();
        int now_time = 420 - timeLeft;
        // 处理坐标数据
        reliableEnemyPosuition.clear();
        const int reliable_enemy_position_fresh_ms = std::max(
            1,
            std::max(
                config.RegionalDefenseSettings.EnemyPositionFreshMs,
                config.ChaseSettings.OfficialPositionFreshMs));
        for(auto robot : RobotLists) {
            const auto enemy_position = GetEnemyPositionState(
                robot,
                reliable_enemy_position_fresh_ms,
                now);
            if(enemy_position.Fresh &&
               IsOfficialFieldPointValid(enemy_position.X, enemy_position.Y) &&
               enemy_position.X > 100 && enemy_position.Y > 100) {
                reliableEnemyPosuition.push_back(robot);
            }
        }
        LoggerPtr->Info("> reliableEnemyPosuition <");
        for(auto robot : reliableEnemyPosuition) {
            const auto enemy_position = GetEnemyPositionState(
                robot,
                reliable_enemy_position_fresh_ms,
                now);
            LoggerPtr->Info("ID: {}, X: {}, Y:{}", static_cast<int>(robot), enemy_position.X, enemy_position.Y);
        }

        // 处理距离和无敌状态的数据
        hitableTargets.clear();
        const auto& aim_target_setting = config.DecisionAutonomySettings.AimTarget;
        const auto has_fresh_external_target = [&](const ArmorType armor_type) {
            if (!config.ExternalAimSettings.Enable ||
                !config.ExternalAimSettings.UseTargetArrayAsArmorList ||
                !hasExternalAimTargets_) {
                return false;
            }
            const auto target_index = static_cast<std::size_t>(armor_type);
            if (target_index >= externalAimTargets_.size()) {
                return false;
            }
            const auto& cached = externalAimTargets_[target_index];
            const int fresh_ms = std::max(1, config.ExternalAimSettings.TargetFreshTimeoutMs);
            return cached.Valid &&
                cached.LastSeen.time_since_epoch().count() != 0 &&
                now - cached.LastSeen <= std::chrono::milliseconds(fresh_ms);
        };
        const auto is_confirmed_dead_hold = [&](const UnitType unit_type) {
            const auto index = static_cast<std::size_t>(unit_type);
            if (index >= enemyHealthConfirmedDead_.size()) {
                return false;
            }
            if (!enemyHealthConfirmedDead_[index]) {
                return false;
            }
            const auto last_confirmed_dead = lastEnemyConfirmedDeadTime_[index];
            if (last_confirmed_dead.time_since_epoch().count() == 0) {
                return false;
            }
            const int dead_hold_ms = std::max(0, aim_target_setting.DeadHealthHoldMs);
            return dead_hold_ms == 0 ||
                now - last_confirmed_dead <= std::chrono::milliseconds(dead_hold_ms);
        };
        const auto is_invulnerable = [&](const UnitType unit_type) {
            int hold_seconds = aim_target_setting.RespawnInvulnerableSec;
            if (unit_type == UnitType::Sentry) {
                hold_seconds = aim_target_setting.SentryRespawnInvulnerableSec;
            }
            const auto armor_type = ArmorTypeFromUnitType(unit_type);
            if (armor_type.has_value() && has_fresh_external_target(*armor_type)) {
                return false;
            }
            return is_confirmed_dead_hold(unit_type) ||
                enemyRobots[unit_type].isInvulnerable(hold_seconds);
        };
        const auto add_hitable_target = [&](const UnitType unit_type) {
            if (std::find(hitableTargets.begin(), hitableTargets.end(), unit_type) == hitableTargets.end()) {
                hitableTargets.push_back(unit_type);
            }
        };
        for (auto Armor : armorList) {
            if (Armor.Type == ArmorType::UnKnown) continue;
            if(Armor.Type == ArmorType::Hero) {
                enemyRobots[UnitType::Hero].distance_ = Armor.Distance;
                if(!is_invulnerable(UnitType::Hero)) add_hitable_target(UnitType::Hero);
            }else if(Armor.Type == ArmorType::Engineer) {
                enemyRobots[UnitType::Engineer].distance_ = Armor.Distance;
                if(!is_invulnerable(UnitType::Engineer) && now_time > 60) add_hitable_target(UnitType::Engineer);
            }else if(Armor.Type == ArmorType::Infantry1) {
                enemyRobots[UnitType::Infantry1].distance_ = Armor.Distance;
                if(!is_invulnerable(UnitType::Infantry1)) add_hitable_target(UnitType::Infantry1);
            }else if(Armor.Type == ArmorType::Infantry2) {
                enemyRobots[UnitType::Infantry2].distance_ = Armor.Distance;
                if(!is_invulnerable(UnitType::Infantry2)) add_hitable_target(UnitType::Infantry2);
            }else if(Armor.Type == ArmorType::Sentry) {
                enemyRobots[UnitType::Sentry].distance_ = Armor.Distance;
                if(!is_invulnerable(UnitType::Sentry)) add_hitable_target(UnitType::Sentry);
            }
        }
        for(auto robot : hitableTargets) {
            LoggerPtr->Info("ID{}", static_cast<int>(robot));
        }
    }

    bool Application::HasFreshExternalAimTarget(const ArmorType armor_type) const noexcept {
        if (!config.ExternalAimSettings.Enable ||
            !hasExternalAimTargets_ ||
            !IsSelectableExternalAimArmor(armor_type)) {
            return false;
        }
        const auto target_index = static_cast<std::size_t>(armor_type);
        if (target_index >= externalAimTargets_.size()) {
            return false;
        }
        const auto& cached = externalAimTargets_[target_index];
        const int fresh_ms = std::max(1, config.ExternalAimSettings.TargetFreshTimeoutMs);
        return cached.Valid &&
            cached.LastSeen.time_since_epoch().count() != 0 &&
            std::chrono::steady_clock::now() - cached.LastSeen <= std::chrono::milliseconds(fresh_ms);
    }

    bool Application::HasFreshSelectedExternalAimTarget() const noexcept {
        return ShouldSelectFreshExternalAimTarget(
            targetArmor.Type,
            HasFreshExternalAimTarget(targetArmor.Type),
            IsIgnoredArmorType(config.AimTargetIgnore, targetArmor.Type));
    }

    bool Application::SelectFreshExternalAimTarget() {
        std::optional<ArmorType> selected_armor;
        float selected_distance = std::numeric_limits<float>::infinity();

        for (std::uint8_t raw_armor = static_cast<std::uint8_t>(ArmorType::Base);
             raw_armor <= static_cast<std::uint8_t>(ArmorType::Outpost);
             ++raw_armor) {
            const auto armor_type = static_cast<ArmorType>(raw_armor);
            if (!ShouldSelectFreshExternalAimTarget(
                    armor_type,
                    HasFreshExternalAimTarget(armor_type),
                    IsIgnoredArmorType(config.AimTargetIgnore, armor_type))) {
                continue;
            }
            const auto& candidate = externalAimTargets_[static_cast<std::size_t>(armor_type)];
            if (!selected_armor.has_value() || candidate.Distance < selected_distance) {
                selected_armor = armor_type;
                selected_distance = candidate.Distance;
            }
        }

        if (!selected_armor.has_value()) {
            return false;
        }
        targetArmor.Type = *selected_armor;
        targetArmor.Distance = selected_distance;
        return true;
    }

    void Application::SetAimTarget() {
        UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        const auto self_position = GetSentryPositionState(std::chrono::steady_clock::now());
        const bool has_self_position =
            self_position.Fresh && self_position.X > 0 && self_position.Y > 0;
        const auto is_ignored_armor = [this](const ArmorType armor_type) -> bool {
            return IsIgnoredArmorType(config.AimTargetIgnore, armor_type);
        };
        const auto set_outpost_target = [&]() -> bool {
            if (is_ignored_armor(ArmorType::Outpost)) {
                return false;
            }
            targetArmor.Type = ArmorType::Outpost;
            targetArmor.Distance = 30.0F;
            if (HasFreshExternalAimTarget(ArmorType::Outpost)) {
                targetArmor.Distance = externalAimTargets_[static_cast<std::size_t>(ArmorType::Outpost)].Distance;
            }
            return true;
        };
        if (outpostEngagementDecision_.HoldTarget && set_outpost_target()) {
            return;
        }
        if (SelectFreshExternalAimTarget()) {
            return;
        }
        if(aimMode == AimMode::Buff) { // 打符，修改为默认值
            if(has_self_position &&
               BehaviorTree::Area::BuffOutpost.near(self_position.X, self_position.Y, 100, MyTeam) &&
               !is_ignored_armor(ArmorType::Hero)) {
                targetArmor.Type = ArmorType::Hero;
            } else {
                SetAimTargetNormal();
            }
        }else if(aimMode == AimMode::Outpost ||
                 (outpostVisualScoutNavigationActive_ && !outpostArmorInterruptActive_)) { // 打前哨站
            const int outpost_select_distance_cm = std::max(
                100,
                std::max(0, config.TaskSettings.OutpostConfirm.VisualScoutFaceDistanceCm));
            if (set_outpost_target()) {
                // Keep selecting Outpost during the scout/navigation task so far armor targets do not
                // steal /ly/aim/result before they enter ArmorWarningDistanceCm.
            } else if((IsBaseGoalArrived(LangYa::BuffOutpost.ID, MyTeam, true) ||
                IsBaseGoalWithinDistance(
                    LangYa::BuffOutpost.ID,
                    MyTeam,
                    outpost_select_distance_cm)) &&
               !is_ignored_armor(ArmorType::Outpost)) {
                targetArmor.Type = ArmorType::Outpost;
            } else {
                SetAimTargetNormal();
            }
        }else { // 普通模式
            if(naviCommandGoal == LangYa::HoleRoad(EnemyTeam)) { // 英雄点位1
                if(has_self_position &&
                   BehaviorTree::Area::HoleRoad.near(self_position.X, self_position.Y, 100, MyTeam) &&
                   !is_ignored_armor(ArmorType::Hero)) {
                    targetArmor.Type = ArmorType::Hero;
                    targetArmor.Distance = enemyRobots[UnitType::Hero].distance_;
                } else {
                    SetAimTargetNormal();
                }
            } else {
                SetAimTargetNormal();
            }
            LoggerPtr->Info("Target: {}", static_cast<int>(targetArmor.Type));
        }

    }

    void Application::RefreshOutpostEngagementLock() {
        const auto now = std::chrono::steady_clock::now();
        const auto& outpost = config.TaskSettings.OutpostConfirm;
        const int fresh_ms = std::max(0, outpost.RefereeFreshTimeoutMs);
        const bool enemy_hp_fresh = lastEnemyOutpostHealthRxTime_.time_since_epoch().count() != 0 &&
            now - lastEnemyOutpostHealthRxTime_ <= std::chrono::milliseconds(fresh_ms);
        const bool self_hp_fresh = hasReceivedSentryInfo_ &&
            lastSentryInfoRxTime_.time_since_epoch().count() != 0 &&
            now - lastSentryInfoRxTime_ <= std::chrono::milliseconds(fresh_ms);
        auto referee_timer = postureRefereeTimer_;
        if (referee_timer.HasInfo3 && referee_timer.AgeMeasuredAt.time_since_epoch().count() != 0) {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - referee_timer.AgeMeasuredAt).count();
            referee_timer.Fresh = static_cast<std::uint64_t>(referee_timer.AgeMs) +
                static_cast<std::uint64_t>(std::max<decltype(elapsed_ms)>(elapsed_ms, 0)) <=
                static_cast<std::uint64_t>(std::max(0, config.PostureSettings.RefereeInfo3FreshMs));
        }
        const auto reach = EvaluateBaseGoalReach(LangYa::BuffOutpost.ID, team, true);
        outpostEngagementLock_.Configure({
            .Enable = outpost.TrustEnemyOutpostHp,
            .EnhancedAttackOnEnemyHpDrop = outpost.EnhancedAttackOnEnemyHpDrop,
            .NormalAttackLockExitHp = static_cast<std::uint16_t>(std::clamp(outpost.NormalAttackLockExitHp, 0, 400)),
            .EnhancedAttackLockExitHp = static_cast<std::uint16_t>(std::clamp(outpost.EnhancedAttackLockExitHp, 0, 400)),
            .TargetLostGraceMs = std::max(0, config.PostureSettings.TargetKeepMs),
        });
        outpostEngagementDecision_ = outpostEngagementLock_.Tick(now, {
            .Target7Fresh = OutpostAimFreshAndValid(),
            .SelectedTarget7 = targetArmor.Type == ArmorType::Outpost,
            .EnemyHpFresh = enemy_hp_fresh,
            .EnemyHp = enemyOutpostHealth,
            .SelfHpFresh = self_hp_fresh,
            .SelfHp = myselfHealth,
            .NavigationReachable = reach.Status != GoalReachStatus::Unreachable,
            .Posture = postureManager_.Runtime(),
            .PostureCooldownReady = postureManager_.IsSwitchCooldownReady(now),
            .EnhancedAttackRemainingFresh = referee_timer.Fresh,
            .EnhancedAttackRemainingSec = referee_timer.EnhancedRemainingSec[static_cast<std::size_t>(SentryPosture::Attack)],
        });
        if (outpostEngagementDecision_.CancelPending) {
            postureManager_.CancelPending();
        }
    }

    bool Application::TrySetAimTargetByAutonomy() {
        const auto& autonomy = config.DecisionAutonomySettings.AimTarget;
        if (!autonomy.Enable && !IsDecisionAutonomyModuleEnabled("aim_target")) {
            return false;
        }

        struct AimCandidate {
            ArmorType Armor{ArmorType::UnKnown};
            UnitType Unit{UnitType::Unknown};
            float Distance{0.0f};
            std::uint16_t Health{0U};
            bool HealthFresh{false};
            double Score{0.0};
        };

        const auto now = std::chrono::steady_clock::now();
        const int health_fresh_timeout_ms = std::max(0, autonomy.HealthFreshTimeoutMs);
        const auto is_health_fresh = [&](const UnitType unit_type) {
            const auto index = static_cast<std::size_t>(unit_type);
            if (!hasReceivedEnemyHealth_ || index >= lastEnemyHealthRxTime_.size()) {
                return false;
            }
            const auto rx_time = lastEnemyHealthRxTime_[index];
            if (rx_time.time_since_epoch().count() == 0) {
                return false;
            }
            if (health_fresh_timeout_ms == 0) {
                return true;
            }
            return now - rx_time <= std::chrono::milliseconds(health_fresh_timeout_ms);
        };
        const auto hold_current_if_recent = [&]() {
            const int lost_target_hold_ms = std::max(0, autonomy.LostTargetHoldMs);
            if (lost_target_hold_ms <= 0 || targetArmor.Type == ArmorType::UnKnown ||
                IsIgnoredArmorType(config.AimTargetIgnore, targetArmor.Type) ||
                lastAimTargetCandidateSeenTime_.time_since_epoch().count() == 0 ||
                now - lastAimTargetCandidateSeenTime_ > std::chrono::milliseconds(lost_target_hold_ms)) {
                return false;
            }
            const auto unit_type = UnitTypeFromArmorType(targetArmor.Type);
            if (unit_type.has_value()) {
                targetArmor.Distance = enemyRobots[*unit_type].distance_;
            }
            return true;
        };

        std::vector<AimCandidate> candidates;
        candidates.reserve(hitableTargets.size());
        std::uint16_t max_health = 1U;
        for (const auto unit_type : hitableTargets) {
            if (IsIgnoredUnitType(config.AimTargetIgnore, unit_type)) {
                continue;
            }
            const auto armor_type = ArmorTypeFromUnitType(unit_type);
            if (!armor_type.has_value()) {
                continue;
            }
            const auto& robot = enemyRobots[unit_type];
            const bool health_fresh = is_health_fresh(unit_type);
            candidates.push_back(AimCandidate{
                .Armor = *armor_type,
                .Unit = unit_type,
                .Distance = robot.distance_,
                .Health = robot.currentHealth_,
                .HealthFresh = health_fresh
            });
            if (health_fresh) {
                max_health = std::max(max_health, robot.currentHealth_);
            }
        }
        if (candidates.empty()) {
            return hold_current_if_recent();
        }
        lastAimTargetCandidateSeenTime_ = now;

        const auto get_priority_rank = [this](const ArmorType armor_type) {
            const int armor_id = static_cast<int>(armor_type);
            const auto it = std::find(config.AimTargetPriority.begin(), config.AimTargetPriority.end(), armor_id);
            if (it == config.AimTargetPriority.end()) {
                return static_cast<int>(config.AimTargetPriority.size());
            }
            return static_cast<int>(std::distance(config.AimTargetPriority.begin(), it));
        };
        const auto score_candidate = [&](AimCandidate& candidate) {
            const int rank = get_priority_rank(candidate.Armor);
            const double priority_score = config.AimTargetPriority.empty()
                ? 0.0
                : static_cast<double>(config.AimTargetPriority.size() - rank) /
                    static_cast<double>(config.AimTargetPriority.size());
            const double distance_score = (std::isfinite(candidate.Distance) && candidate.Distance > 0.0f)
                ? 1.0 / (0.1 + static_cast<double>(candidate.Distance))
                : 0.0;
            const double health_score = candidate.HealthFresh
                ? 1.0 - static_cast<double>(candidate.Health) /
                    static_cast<double>(std::max<std::uint16_t>(1U, max_health))
                : 0.0;

            double score = 0.0;
            score += autonomy.PriorityWeight * priority_score;
            score += autonomy.DistanceWeight * distance_score;
            score += autonomy.LowHealthWeight * health_score;
            if (candidate.Armor == targetArmor.Type) {
                score += autonomy.CurrentTargetBonus;
            }
            if (candidate.Armor == ArmorType::Hero) {
                score += autonomy.HeroBonus;
            } else if (candidate.Armor == ArmorType::Sentry) {
                score += autonomy.SentryBonus;
            }
            candidate.Score = score;
            return score;
        };

        double best_score = -std::numeric_limits<double>::infinity();
        std::optional<AimCandidate> best_candidate;
        std::optional<AimCandidate> current_candidate;
        for (auto& candidate : candidates) {
            const double score = score_candidate(candidate);
            if (score > best_score) {
                best_score = score;
                best_candidate = candidate;
            }
            if (candidate.Armor == targetArmor.Type) {
                current_candidate = candidate;
            }
        }

        if (!best_candidate.has_value()) {
            return hold_current_if_recent();
        }

        auto selected_candidate = *best_candidate;
        const int min_switch_interval_ms = std::max(0, autonomy.MinSwitchIntervalMs);
        const bool switch_too_soon =
            selected_candidate.Armor != targetArmor.Type &&
            current_candidate.has_value() &&
            min_switch_interval_ms > 0 &&
            lastAimTargetSelectTime_.time_since_epoch().count() != 0 &&
            now - lastAimTargetSelectTime_ < std::chrono::milliseconds(min_switch_interval_ms);
        const bool switch_margin_too_small =
            selected_candidate.Armor != targetArmor.Type &&
            current_candidate.has_value() &&
            selected_candidate.Score <
                current_candidate->Score + std::max(0.0, autonomy.SwitchScoreMargin);
        if (switch_too_soon || switch_margin_too_small) {
            selected_candidate = *current_candidate;
        }

        if (selected_candidate.Armor != targetArmor.Type ||
            lastAimTargetSelectTime_.time_since_epoch().count() == 0) {
            lastAimTargetSelectTime_ = now;
        }
        targetArmor.Type = selected_candidate.Armor;
        targetArmor.Distance = selected_candidate.Distance;
        return true;
    }

    void Application::SetAimTargetNormal() {
        auto set_target = [&](const ArmorType armor_type, const UnitType unit_type) {
            targetArmor.Type = armor_type;
            targetArmor.Distance = enemyRobots[unit_type].distance_;
        };
        const auto is_armor_ignored = [this](const ArmorType armor_type) -> bool {
            return IsIgnoredArmorType(config.AimTargetIgnore, armor_type);
        };
        const auto has_target = [&](const UnitType unit_type) -> bool {
            return !IsIgnoredUnitType(config.AimTargetIgnore, unit_type) &&
                   std::find(hitableTargets.begin(), hitableTargets.end(), unit_type) != hitableTargets.end();
        };
        const bool infantry1_find = has_target(UnitType::Infantry1);
        const bool infantry2_find = has_target(UnitType::Infantry2);

        if (TrySetAimTargetByAutonomy()) {
            return;
        }

        if (!hitableTargets.empty()) {
            for (const auto armor_id : config.AimTargetPriority) {
                const auto armor_type = ArmorTypeFromPriorityId(armor_id);
                if (!armor_type.has_value()) {
                    continue;
                }
                if (is_armor_ignored(*armor_type)) {
                    continue;
                }
                if ((*armor_type == ArmorType::Infantry1 || *armor_type == ArmorType::Infantry2) &&
                    infantry1_find && infantry2_find) {
                    // 兼容旧逻辑：步兵1/2同时可打时，优先近距离；距离接近时看血量更低者。
                    const auto distance1 = enemyRobots[UnitType::Infantry1].distance_;
                    const auto distance2 = enemyRobots[UnitType::Infantry2].distance_;
                    const auto delta_distance = distance1 - distance2;
                    if (std::fabs(delta_distance) > 1.0f) {
                        if (distance1 < distance2) {
                            set_target(ArmorType::Infantry1, UnitType::Infantry1);
                        } else {
                            set_target(ArmorType::Infantry2, UnitType::Infantry2);
                        }
                    } else {
                        const auto health1 = enemyRobots[UnitType::Infantry1].currentHealth_;
                        const auto health2 = enemyRobots[UnitType::Infantry2].currentHealth_;
                        if (health1 < health2) {
                            set_target(ArmorType::Infantry1, UnitType::Infantry1);
                        } else {
                            set_target(ArmorType::Infantry2, UnitType::Infantry2);
                        }
                    }
                    return;
                }
                const auto unit_type = UnitTypeFromArmorType(*armor_type);
                if (!unit_type.has_value()) {
                    continue;
                }
                if (!has_target(*unit_type)) {
                    continue;
                }
                set_target(*armor_type, *unit_type);
                return;
            }

            // 配置优先级没有命中时，回退到最近目标
            std::optional<UnitType> nearest_unit;
            for (const auto unit_type : hitableTargets) {
                if (IsIgnoredUnitType(config.AimTargetIgnore, unit_type)) {
                    continue;
                }
                if (!nearest_unit.has_value() ||
                    enemyRobots[unit_type].distance_ < enemyRobots[*nearest_unit].distance_) {
                    nearest_unit = unit_type;
                }
            }
            if (nearest_unit.has_value()) {
                const auto armor_type = ArmorTypeFromUnitType(*nearest_unit);
                if (armor_type.has_value()) {
                    set_target(*armor_type, *nearest_unit);
                    return;
                }
            }
        }

        for (const auto armor_id : config.AimTargetPriority) {
            const auto armor_type = ArmorTypeFromPriorityId(armor_id);
            if (!armor_type.has_value() || is_armor_ignored(*armor_type)) {
                continue;
            }
            targetArmor.Type = *armor_type;
            targetArmor.Distance = 30;
            return;
        }
        targetArmor.Type = ArmorType::UnKnown;
        targetArmor.Distance = 30;
    }

    void Application::CheckDebug() {
        if (config.AimDebugSettings.ForceBuff) aimMode = AimMode::Buff;
        else if(config.AimDebugSettings.ForceOutpost) aimMode = AimMode::Outpost;
        /*------------打印日志---------*/
        if(aimMode == AimMode::AutoAim) LoggerPtr->Info("AimMode: AutoAim");
        else if(aimMode == AimMode::Buff) LoggerPtr->Info("AimMode: Buff");
        else if(aimMode == AimMode::Outpost) LoggerPtr->Info("AimMode: Outpost");
        else if(aimMode == AimMode::RotateScan) LoggerPtr->Info("AimMode: RotateScan");
    }
}


namespace BehaviorTree {

    bool Application::IsLeagueRouteCompatEnabled() const noexcept {
        return IsLeagueProfile() &&
            !config.NaviSettings.UseXY &&
            config.LeagueStrategySettings.EnableRouteCompat;
    }

    bool Application::IsLeagueGoalSwitchBetween2And3(
        const std::uint8_t from_goal_id,
        const std::uint8_t to_goal_id,
        const UnitTeam goal_team,
        const bool apply_team_offset) const noexcept {
        const auto goal_2 = ResolveGoalId(LangYa::Recovery.ID, goal_team, apply_team_offset);
        const auto goal_3 = ResolveGoalId(LangYa::BuffShoot.ID, goal_team, apply_team_offset);
        return (from_goal_id == goal_2 && to_goal_id == goal_3) ||
            (from_goal_id == goal_3 && to_goal_id == goal_2);
    }

    bool Application::TickLeagueRouteCompat(
        const UnitTeam goal_team,
        const bool apply_team_offset) {
        if (!IsLeagueRouteCompatEnabled() || !leagueRouteCompatActive_) {
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now < leagueRouteCompatUntil_) {
            SetPositionByBaseGoal(kLeagueRouteCompatViaGoalBaseId, goal_team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{1});
            speedLevel = 1;
            return true;
        }

        leagueRouteCompatActive_ = false;
        leagueRouteCompatUntil_ = std::chrono::steady_clock::time_point{};
        if (!leagueRouteCompatHasPendingGoal_) {
            return false;
        }

        const auto pending_base_goal = leagueRouteCompatPendingBaseGoal_;
        const int pending_hold_sec = std::max(1, leagueRouteCompatPendingHoldSec_);
        leagueRouteCompatHasPendingGoal_ = false;

        SetPositionByBaseGoal(pending_base_goal, goal_team, apply_team_offset);
        naviCommandIntervalClock.reset(Seconds{pending_hold_sec});
        speedLevel = 1;
        LoggerPtr->Info(
            "League route compat finished: via goal done, continue goal={} hold={}s.",
            static_cast<int>(naviCommandGoal),
            pending_hold_sec);
        return true;
    }

    void Application::StartLeagueRouteCompat(
        const std::uint8_t pending_base_goal,
        const int pending_hold_sec,
        const UnitTeam goal_team,
        const bool apply_team_offset,
        const char* reason) {
        if (!IsLeagueRouteCompatEnabled()) {
            return;
        }
        const int safe_pending_hold_sec = std::max(1, pending_hold_sec);
        leagueRouteCompatActive_ = true;
        leagueRouteCompatUntil_ = std::chrono::steady_clock::now() + std::chrono::seconds(kLeagueRouteCompatViaHoldSec);
        leagueRouteCompatHasPendingGoal_ = true;
        leagueRouteCompatPendingBaseGoal_ = pending_base_goal;
        leagueRouteCompatPendingHoldSec_ = safe_pending_hold_sec;

        SetPositionByBaseGoal(kLeagueRouteCompatViaGoalBaseId, goal_team, apply_team_offset);
        naviCommandIntervalClock.reset(Seconds{1});
        speedLevel = 1;
        LoggerPtr->Info(
            "League route compat {}: via goal={} for {}s, then goal={} hold={}s.",
            reason ? reason : "start",
            static_cast<int>(naviCommandGoal),
            kLeagueRouteCompatViaHoldSec,
            static_cast<int>(ResolveGoalId(pending_base_goal, goal_team, apply_team_offset)),
            safe_pending_hold_sec);
    }

    std::uint8_t Application::ResolveGoalId(
        const std::uint8_t base_goal_id,
        const UnitTeam team,
        const bool apply_team_offset) const noexcept {
        return AreaManager::ResolveGoalId(base_goal_id, team, apply_team_offset);
    }

    bool Application::IsNaviGoalAreaScopeEnabled() const noexcept {
        return areaManager_.IsGoalAreaScopeEnabled();
    }

    bool Application::IsNaviGoalAllowedByAreaScope(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const UnitTeam my_team,
        const UnitTeam enemy_team) const {
        const auto result = areaManager_.CheckGoalAreaScope(
            base_goal_id,
            goal_team,
            my_team,
            enemy_team);
        if (result.Allowed) {
            return true;
        }
        if (result.ResolvedArea.has_value() &&
            result.ResolvedArea->UsedNearestFallback &&
            LoggerPtr) {
            LoggerPtr->Debug(
                "DecisionAutonomy[navi_goal_area]: goal={} team={} mapped to nearest area='{}'.",
                static_cast<int>(base_goal_id),
                result.ScopeName,
                Area::MainAreaKindName(result.ResolvedArea->Kind));
        }
        return false;
    }

    bool Application::IsHighlandCompatEnabled() const noexcept {
        return areaManager_.IsHighlandCompatEnabled();
    }

    bool Application::IsHighlandCompatTarget(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team) const {
        return areaManager_.IsHighlandCompatTarget(base_goal_id, goal_team);
    }

    bool Application::IsSelfInMainArea(
        const UnitTeam area_team,
        const Area::MainAreaKind kind) const {
        if (area_team != UnitTeam::Red && area_team != UnitTeam::Blue) {
            return false;
        }
        const auto self_position = GetSentryPositionState(std::chrono::steady_clock::now());
        if (!self_position.Fresh) {
            return false;
        }
        if (self_position.X <= 0 || self_position.Y <= 0) {
            return false;
        }
        return AreaManager::IsPositionInMainArea(area_team, kind, self_position.X, self_position.Y);
    }

    bool Application::IsNaviExternalStatusFreshForGoal(
        const std::chrono::steady_clock::time_point last_rx,
        const std::uint8_t goal_id,
        const Area::Point<std::uint16_t> goal_position) const {
        if (!naviExternalStatusGoalInitialized_ ||
            naviExternalStatusGoalId_ != goal_id ||
            naviExternalStatusGoalPosition_.x != goal_position.x ||
            naviExternalStatusGoalPosition_.y != goal_position.y ||
            naviCommandGoal != goal_id ||
            last_rx.time_since_epoch().count() == 0 ||
            last_rx < naviExternalStatusGoalStartTime_) {
            return false;
        }
        return std::chrono::steady_clock::now() - last_rx <=
            std::chrono::milliseconds(kNaviExternalStatusTimeoutMs);
    }

    std::optional<bool> Application::GetExternalNaviReachForGoal(
        const std::uint8_t goal_id,
        const Area::Point<std::uint16_t> goal_position) const {
        if (!hasReceivedNaviReach_ ||
            !IsNaviExternalStatusFreshForGoal(lastNaviReachRxTime_, goal_id, goal_position)) {
            return std::nullopt;
        }
        return naviReach;
    }

    std::optional<bool> Application::GetExternalNaviReachableForGoal(
        const std::uint8_t goal_id,
        const Area::Point<std::uint16_t> goal_position) const {
        if (!hasReceivedNaviReachable_ ||
            !IsNaviExternalStatusFreshForGoal(lastNaviReachableRxTime_, goal_id, goal_position)) {
            return std::nullopt;
        }
        return naviReachable;
    }

    void Application::UpdateNaviExternalStatusGoal(
        const std::uint8_t goal_id,
        const Area::Point<std::uint16_t> goal_position) {
        const bool same_goal =
            naviExternalStatusGoalInitialized_ &&
            naviExternalStatusGoalId_ == goal_id &&
            naviExternalStatusGoalPosition_.x == goal_position.x &&
            naviExternalStatusGoalPosition_.y == goal_position.y;
        if (same_goal) {
            return;
        }
        naviExternalStatusGoalInitialized_ = true;
        naviExternalStatusGoalId_ = goal_id;
        naviExternalStatusGoalPosition_ = goal_position;
        naviExternalStatusGoalStartTime_ = std::chrono::steady_clock::now();
        naviExternalStatusGoalStartRosTime_ = node_ ? node_->now() : rclcpp::Time{};
    }

    GoalReachState Application::EvaluateNaviGoalReach(
        const std::uint8_t goal_id,
        const Area::Point<std::uint16_t> goal_position,
        const int arrive_distance_cm,
        const int face_distance_cm,
        const std::uint8_t base_goal_id,
        const int timeout_sec) const {
        GoalReachState state;
        state.GoalId = goal_id;
        state.BaseGoalId = base_goal_id;
        state.GoalPosition = goal_position;
        state.ArriveDistanceCm = std::max(1, arrive_distance_cm);
        state.FaceDistanceCm = std::max(0, face_distance_cm);
        state.Status = GoalReachStatus::Traveling;
        state.Reason = GoalReachReason::None;

        const auto now = std::chrono::steady_clock::now();
        const bool external_goal_matches =
            naviExternalStatusGoalInitialized_ &&
            naviExternalStatusGoalId_ == goal_id &&
            naviExternalStatusGoalPosition_.x == goal_position.x &&
            naviExternalStatusGoalPosition_.y == goal_position.y &&
            naviExternalStatusGoalStartTime_.time_since_epoch().count() != 0;
        if (external_goal_matches) {
            state.GoalStartTime = naviExternalStatusGoalStartTime_;
            state.GoalStartStamp = naviExternalStatusGoalStartRosTime_;
        }

        const auto& runtime = areaManager_.ProgressWatchdogRuntime();
        const bool runtime_goal_matches =
            runtime.Active &&
            runtime.GoalId == goal_id &&
            runtime.GoalPosition.x == goal_position.x &&
            runtime.GoalPosition.y == goal_position.y &&
            runtime.GoalStartTime.time_since_epoch().count() != 0;
        if (state.GoalStartTime.time_since_epoch().count() == 0 && runtime_goal_matches) {
            state.GoalStartTime = runtime.GoalStartTime;
        }
        if (state.GoalStartTime.time_since_epoch().count() != 0) {
            state.GoalAgeMs = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                now - state.GoalStartTime).count());
        }

        const auto external_reachable = GetExternalNaviReachableForGoal(goal_id, goal_position);
        state.ExternalReachable = external_reachable;
        const auto external_reach = GetExternalNaviReachForGoal(goal_id, goal_position);
        state.ExternalReach = external_reach;
        const bool is_active_command_goal =
            goal_id == naviCommandGoal &&
            goal_position.x == naviGoalPosition.x &&
            goal_position.y == naviGoalPosition.y;

        const int distance_fallback_grace_ms =
            std::max(0, config.DecisionAutonomySettings.NaviGoal.DistanceFallbackGraceMs);
        const bool grace_active =
            distance_fallback_grace_ms > 0 &&
            runtime_goal_matches &&
            now - runtime.GoalStartTime < std::chrono::milliseconds(distance_fallback_grace_ms);
        state.DistanceFallbackAllowed = !grace_active;

        if (timeout_sec > 0 &&
            state.GoalStartTime.time_since_epoch().count() != 0 &&
            now - state.GoalStartTime >= std::chrono::seconds(timeout_sec)) {
            state.Timeout = true;
        }

        const auto self_position = GetSentryPositionState(now);
        state.PositionFresh = self_position.Fresh;
        state.SelfX = self_position.X;
        state.SelfY = self_position.Y;
        state.HasPosition = self_position.HasPosition && state.SelfX > 0 && state.SelfY > 0;
        const bool position_usable_for_distance = state.HasPosition && state.PositionFresh;
        if (position_usable_for_distance) {
            const double distance_sq = AreaManager::DistanceSq(
                state.SelfX,
                state.SelfY,
                static_cast<int>(goal_position.x),
                static_cast<int>(goal_position.y));
            state.DistanceCm = std::sqrt(distance_sq);
            state.WithinArriveDistance =
                distance_sq <= static_cast<double>(state.ArriveDistanceCm) *
                    static_cast<double>(state.ArriveDistanceCm);
            state.WithinFaceDistance =
                state.FaceDistanceCm > 0 &&
                distance_sq <= static_cast<double>(state.FaceDistanceCm) *
                    static_cast<double>(state.FaceDistanceCm);
        }

        if (external_reachable.has_value() && !*external_reachable) {
            if (is_active_command_goal) {
                nearGoalArrivalConfirm_.Reset();
            }
            state.Status = GoalReachStatus::Unreachable;
            state.Reason = GoalReachReason::ExternalUnreachable;
            return state;
        }
        if (external_reach.has_value() && *external_reach) {
            if (is_active_command_goal) {
                nearGoalArrivalConfirm_.Reset();
            }
            state.Status = GoalReachStatus::Reached;
            state.Reason = GoalReachReason::ExternalReached;
            return state;
        }
        if (distance_fallback_grace_ms > 0 &&
            grace_active) {
            if (is_active_command_goal) {
                nearGoalArrivalConfirm_.Reset();
            }
            state.Status = GoalReachStatus::Traveling;
            state.Reason = GoalReachReason::GraceActive;
            return state;
        }
        const bool use_position_distance = position_usable_for_distance && state.WithinArriveDistance;
        bool near_goal_confirmed = use_position_distance;
        if (is_active_command_goal) {
            const int near_goal_confirm_wait_ms =
                std::max(0, config.DecisionAutonomySettings.NaviGoal.NearGoalConfirmWaitMs);
            near_goal_confirmed = nearGoalArrivalConfirm_.Observe(
                goal_id,
                goal_position,
                use_position_distance,
                now,
                near_goal_confirm_wait_ms);
            state.NearGoalConfirmPending = use_position_distance && !near_goal_confirmed;
            state.NearGoalConfirmElapsedMs = nearGoalArrivalConfirm_.ElapsedMs(now);
        }
        if (state.NearGoalConfirmPending) {
            state.DistanceFallbackAllowed = false;
            state.Status = GoalReachStatus::Traveling;
            state.Reason = GoalReachReason::NearGoalConfirmPending;
            return state;
        }
        if (near_goal_confirmed) {
            state.Status = GoalReachStatus::Reached;
            state.Reason = GoalReachReason::PositionDistance;
            return state;
        }
        if (state.Timeout) {
            state.Status = GoalReachStatus::Timeout;
            state.Reason = GoalReachReason::Timeout;
            return state;
        }
        if (!position_usable_for_distance) {
            state.Status = GoalReachStatus::Traveling;
            state.Reason = GoalReachReason::PositionStale;
            return state;
        }
        return state;
    }

    GoalReachState Application::EvaluateBaseGoalReach(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const bool apply_team_offset,
        const int face_distance_cm,
        const int timeout_sec) const {
        const auto arrive_distance = static_cast<std::uint16_t>(
            std::max(1, config.DecisionAutonomySettings.NaviGoal.HighlandCompatArriveDistanceCm));
        if (!AreaManager::IsValidBaseGoalId(base_goal_id)) {
            GoalReachState state;
            state.BaseGoalId = base_goal_id;
            state.ArriveDistanceCm = arrive_distance;
            state.FaceDistanceCm = std::max(0, face_distance_cm);
            state.Status = GoalReachStatus::Unknown;
            state.Reason = GoalReachReason::InvalidGoal;
            return state;
        }
        const auto goal_id = ResolveGoalId(base_goal_id, goal_team, apply_team_offset);
        const auto goal_point = AreaManager::GoalPointByBaseId(base_goal_id, goal_team);
        return EvaluateNaviGoalReach(
            goal_id,
            goal_point,
            arrive_distance,
            face_distance_cm,
            base_goal_id,
            timeout_sec);
    }

    int Application::GoalReachTimeoutSecForBaseGoal(const std::uint8_t base_goal_id) const {
        if (base_goal_id == LangYa::BuffOutpost.ID) {
            return std::max(0, config.DecisionAutonomySettings.NaviGoal.BuffOutpostCompatTimeoutSec);
        }
        if (base_goal_id == LangYa::Highland.ID || base_goal_id == LangYa::LeftHighLand.ID) {
            return std::max(0, config.DecisionAutonomySettings.NaviGoal.HighlandCompatTimeoutSec);
        }
        return 0;
    }

    bool Application::IsNaviGoalPositionArrived(
        const std::uint8_t goal_id,
        const Area::Point<std::uint16_t> goal_position) const {
        const auto arrive_distance = static_cast<std::uint16_t>(
            std::max(1, config.DecisionAutonomySettings.NaviGoal.HighlandCompatArriveDistanceCm));
        return EvaluateNaviGoalReach(goal_id, goal_position, arrive_distance).Status ==
            GoalReachStatus::Reached;
    }

    bool Application::IsBaseGoalArrived(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const bool apply_team_offset) const {
        return EvaluateBaseGoalReach(base_goal_id, goal_team, apply_team_offset).Status ==
            GoalReachStatus::Reached;
    }

    bool Application::IsBaseGoalWithinDistance(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const int distance_cm) const {
        if (!AreaManager::IsValidBaseGoalId(base_goal_id) || distance_cm <= 0) {
            return false;
        }
        return EvaluateBaseGoalReach(
            base_goal_id,
            goal_team,
            true,
            distance_cm).WithinFaceDistance;
    }

    bool Application::IsBaseGoalExternallyUnreachable(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const bool apply_team_offset) const {
        return EvaluateBaseGoalReach(base_goal_id, goal_team, apply_team_offset).Status ==
            GoalReachStatus::Unreachable;
    }

    bool Application::IsHighlandCompatArrived(const UnitTeam goal_team) const {
        return IsBaseGoalArrived(LangYa::Highland.ID, goal_team);
    }

    void Application::ResetRegionalAreaControlOverride() noexcept {
        faceModeManager_.BeginCycle();
    }

    void Application::ApplyAimModeFaceTarget(const UnitTeam target_team) {
        if (!config.FaceModeSettings.Enable) {
            return;
        }
        (void)faceModeManager_.RequestAimTarget(
            aimMode,
            target_team,
            pub_face_mode_target_raw_);
    }

    void Application::RefreshAimModeFaceControl() {
        if (aimMode != AimMode::Buff && aimMode != AimMode::Outpost) {
            return;
        }
        const auto enemy_team = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        const auto target_team = aimMode == AimMode::Outpost ? enemy_team : team;
        ApplyAimModeFaceTarget(target_team);
    }

    bool Application::TrySetAimModeTaskGoal(
        const UnitTeam my_team,
        const UnitTeam enemy_team,
        const char* reason) {
        if (aimMode != AimMode::Buff && aimMode != AimMode::Outpost) {
            return false;
        }

        TrySetScopedPositionByBaseGoal(
            LangYa::BuffOutpost.ID,
            my_team,
            my_team,
            enemy_team,
            true,
            reason);
        const auto face_target_team = aimMode == AimMode::Outpost ? enemy_team : my_team;
        ApplyAimModeFaceTarget(face_target_team);
        naviCommandIntervalClock.reset(Seconds{2});
        speedLevel = 1;
        return true;
    }

    bool Application::TrySetOutpostVisualScoutTravelGoal(
        const UnitTeam my_team,
        const UnitTeam enemy_team,
        const char* reason) {
        if (!outpostVisualScoutNavigationActive_) {
            return false;
        }

        TrySetScopedPositionByBaseGoal(
            LangYa::BuffOutpost.ID,
            my_team,
            my_team,
            enemy_team,
            true,
            reason);
        naviCommandIntervalClock.reset(Seconds{2});
        speedLevel = 1;
        return true;
    }

    void Application::ApplyRegionalAreaTaskControl(const RegionalAreaTaskTickResult& result) {
        (void)faceModeManager_.RequestRegionalTask(result, pub_face_mode_target_raw_);
        if (!config.NaviControlSettings.Enable) {
            gimbalControlData.FireCode.FollowMode = result.FollowMode ? 1 : 0;
        }
        if (result.SuppressFire) {
            gimbalControlData.FireCode.FireStatus = RecFireCode.FireStatus;
        }
    }

    bool Application::RequestReadyRoadlandSafeReturn(const char* reason) {
        if (!areaManager_.RegionalAreaTaskActive() ||
            areaManager_.RegionalAreaTask().Type != RegionalAreaTaskType::MyReadyRoadland) {
            return false;
        }
        const auto before_phase = areaManager_.RegionalAreaTask().Phase;
        areaManager_.RequestReadyRoadlandReturnToBase(std::chrono::steady_clock::now());
        const auto after_phase = areaManager_.RegionalAreaTask().Phase;
        const bool changed = before_phase != after_phase;
        if (changed && LoggerPtr) {
            LoggerPtr->Info(
                "RegionalAreaTask[MyReadyRoadland] safe return requested: {}.",
                reason ? reason : "higher priority");
        }
        return changed;
    }

    bool Application::TickRegionalAreaTask(
        const UnitTeam my_team,
        const UnitTeam enemy_team) {
        if (!areaManager_.RegionalAreaTaskActive()) {
            return false;
        }

        const auto active_task_type = areaManager_.RegionalAreaTask().Type;
        const auto now = std::chrono::steady_clock::now();
        const bool active_low_priority_task =
            active_task_type == RegionalAreaTaskType::MyBase ||
            active_task_type == RegionalAreaTaskType::MyPreRoadland ||
            active_task_type == RegionalAreaTaskType::MyReadyRoadland ||
            active_task_type == RegionalAreaTaskType::CommonCentral;
        const bool active_ready_roadland_task = active_task_type == RegionalAreaTaskType::MyReadyRoadland;
        const bool can_yield_to_higher_priority = areaManager_.RegionalAreaTaskCanYieldToHigherPriority();
        const bool aim_task_has_higher_priority =
            aimMode == AimMode::Buff ||
            aimMode == AimMode::Outpost ||
            outpostVisualScoutNavigationActive_;
        if (active_low_priority_task && aim_task_has_higher_priority) {
            if (!can_yield_to_higher_priority) {
                // ReadyRoadland crossing is a bound control segment; do not release regional control
                // until the far endpoint or timeout protection completes it.
            } else if (active_ready_roadland_task) {
                RequestReadyRoadlandSafeReturn("aim task has higher priority");
            } else {
                areaManager_.ClearRegionalAreaTask();
                defaultStrategyManager_.RecordRegionalAreaResult(
                    active_task_type,
                    "preempted",
                    now,
                    config.RegionalAreaTaskSettings.DefaultPolicy);
                ResetRegionalAreaControlOverride();
                gimbalControlData.FireCode.FollowMode = 0;
                if (LoggerPtr) {
                    LoggerPtr->Info("RegionalAreaTask canceled: aim task has higher priority.");
                }
                return false;
            }
        }
        if (active_low_priority_task) {
            if (active_ready_roadland_task) {
                const auto threat = EvaluateRegionalDefenseThreat(my_team, enemy_team);
                if (threat.has_value() && can_yield_to_higher_priority) {
                    const bool soft_threat_blocked =
                        !threat->HardThreat &&
                        (aimMode == AimMode::Buff || aimMode == AimMode::Outpost ||
                         !naviCommandIntervalClock.trigger());
                    if (!soft_threat_blocked) {
                        RequestReadyRoadlandSafeReturn("regional defense has higher priority");
                    }
                }
            } else if (
                TrySetRegionalDefenseGoal(my_team, enemy_team) ||
                TrySetRegionalDefenseGoal(
                    my_team,
                    enemy_team,
                    RegionalDefenseSelection::CommonCentral)) {
                areaManager_.ClearRegionalAreaTask();
                defaultStrategyManager_.RecordRegionalAreaResult(
                    active_task_type,
                    "preempted",
                    now,
                    config.RegionalAreaTaskSettings.DefaultPolicy);
                ResetRegionalAreaControlOverride();
                gimbalControlData.FireCode.FollowMode = 0;
                if (LoggerPtr) {
                    LoggerPtr->Info("RegionalAreaTask canceled: regional defense has higher priority.");
                }
                return true;
            }
        }

        const bool apply_team_offset = areaManager_.RegionalAreaTask().ApplyTeamOffset;
        const auto goal_team = areaManager_.RegionalAreaTask().GoalTeam;
        const auto current_base_goal = areaManager_.RegionalAreaTask().CurrentBaseGoal;
        auto referee_value_fresh = [&](const bool received, const std::chrono::steady_clock::time_point last_rx) {
            return received &&
                last_rx.time_since_epoch().count() != 0 &&
                now - last_rx <= std::chrono::seconds(2);
        };
        const auto& ready_roadland_setting = config.RegionalAreaTaskSettings.MyReadyRoadland;
        const auto& central_setting = config.RegionalAreaTaskSettings.CommonCentral;
        const bool ready_roadland_health_known =
            referee_value_fresh(hasReceivedMyselfHealth_, lastMyselfHealthRxTime);
        const bool ready_roadland_ammo_known =
            referee_value_fresh(hasReceivedAmmoLeft_, lastAmmoLeftRxTime);
        const bool ready_roadland_data_unhealthy =
            ready_roadland_health_known &&
            ready_roadland_ammo_known &&
            (myselfHealth < static_cast<std::uint16_t>(std::max(0, ready_roadland_setting.HealthyHpMin)) ||
             ammoLeft < static_cast<std::uint16_t>(std::max(0, ready_roadland_setting.HealthyAmmoMin)));
        const bool central_health_known =
            referee_value_fresh(hasReceivedMyselfHealth_, lastMyselfHealthRxTime);
        const bool central_ammo_known =
            referee_value_fresh(hasReceivedAmmoLeft_, lastAmmoLeftRxTime);
        const bool central_data_unhealthy =
            central_health_known &&
            central_ammo_known &&
            (myselfHealth < static_cast<std::uint16_t>(std::max(0, central_setting.HealthyHpMin)) ||
             ammoLeft < static_cast<std::uint16_t>(std::max(0, central_setting.HealthyAmmoMin)));
        const auto self_position = GetSentryPositionState(now);
        const bool has_self_position =
            self_position.Fresh && self_position.X > 0 && self_position.Y > 0;
        const bool hold_base_patrol_for_armor = [&]() {
            if (active_task_type != RegionalAreaTaskType::MyBase ||
                targetArmor.Type == ArmorType::UnKnown ||
                targetArmor.Type == ArmorType::Outpost) {
                return false;
            }
            return CurrentAimTargetForAngles(
                isFindTargetAtomic.load(std::memory_order_relaxed),
                now,
                config.AimDebugSettings.LatchedTargetHoldMs);
        }();
        const auto result = areaManager_.TickRegionalAreaTask(
            RegionalAreaTaskTickInput{
                .Setting = config.RegionalAreaTaskSettings,
                .Now = now,
                .HighlandArrived = IsBaseGoalArrived(LangYa::Highland.ID, goal_team, apply_team_offset),
                .HighlandUnreachable = IsBaseGoalExternallyUnreachable(LangYa::Highland.ID, goal_team, apply_team_offset),
                .BuffShootArrived = IsBaseGoalArrived(LangYa::BuffShoot.ID, goal_team, apply_team_offset),
                .BuffShootUnreachable = IsBaseGoalExternallyUnreachable(LangYa::BuffShoot.ID, goal_team, apply_team_offset),
                .HoleRoadArrived = IsBaseGoalArrived(LangYa::HoleRoad.ID, goal_team, apply_team_offset),
                .HoleRoadUnreachable = IsBaseGoalExternallyUnreachable(LangYa::HoleRoad.ID, goal_team, apply_team_offset),
                .IsCurrentGoalArrived = IsBaseGoalArrived(current_base_goal, goal_team, apply_team_offset),
                .IsCurrentGoalUnreachable = IsBaseGoalExternallyUnreachable(current_base_goal, goal_team, apply_team_offset),
                .ReadyRoadlandCentralToBaseArrived = IsBaseGoalArrived(LangYa::CentralToBase.ID, goal_team, apply_team_offset),
                .ReadyRoadlandCentralToBaseUnreachable = IsBaseGoalExternallyUnreachable(LangYa::CentralToBase.ID, goal_team, apply_team_offset),
                .ReadyRoadlandBaseToCentralArrived = IsBaseGoalArrived(LangYa::BaseToCentral.ID, goal_team, apply_team_offset),
                .ReadyRoadlandBaseToCentralUnreachable = IsBaseGoalExternallyUnreachable(LangYa::BaseToCentral.ID, goal_team, apply_team_offset),
                .ReadyRoadlandShouldLeave = active_ready_roadland_task && ready_roadland_data_unhealthy,
                .CentralShouldLeave =
                    active_task_type == RegionalAreaTaskType::CommonCentral && central_data_unhealthy,
                .HoldCurrentBaseGoal = hold_base_patrol_for_armor,
                .HasSelfPosition = has_self_position,
                .SelfX = has_self_position ? self_position.X : 0,
                .SelfY = has_self_position ? self_position.Y : 0
            });

        if (result.Completed) {
            ApplyRegionalAreaTaskControl(result);
            defaultStrategyManager_.RecordRegionalAreaResult(
                result.Type,
                result.Reason.empty() ? std::string_view{"done"} : std::string_view{result.Reason},
                now,
                config.RegionalAreaTaskSettings.DefaultPolicy);
            if (LoggerPtr) {
                LoggerPtr->Info(
                    "RegionalAreaTask completed type={} phase={} reason={}, follow_mode off.",
                    RegionalAreaTaskTypeToString(result.Type),
                    RegionalAreaTaskPhaseToString(result.Phase),
                    result.Reason.empty() ? "done" : result.Reason.c_str());
            }
            naviCommandIntervalClock.reset(Seconds{1});
            speedLevel = 1;
            return true;
        }

        if (!result.Active || !result.SetGoal) {
            return false;
        }

        const auto previous_goal_id = naviCommandGoal;
        const auto previous_goal_position = naviGoalPosition;
        ApplyRegionalAreaTaskControl(result);
        SetPositionByBaseGoal(result.BaseGoalId, result.GoalTeam, result.ApplyTeamOffset);
        const bool goal_changed =
            naviCommandGoal != previous_goal_id ||
            naviGoalPosition.x != previous_goal_position.x ||
            naviGoalPosition.y != previous_goal_position.y;
        if (goal_changed) {
            const std::string detail =
                "area_task=" + std::string{RegionalAreaTaskTypeToString(result.Type)} +
                " phase=" + RegionalAreaTaskPhaseToString(result.Phase);
            RecordDecisionIntent(MakeDecisionIntent(
                DecisionReason::DefaultAreaPolicy,
                result.BaseGoalId,
                result.GoalTeam,
                result.ApplyTeamOffset,
                detail.c_str()));
        }
        if (result.ResetNaviHold) {
            naviCommandIntervalClock.reset(Seconds{std::max(1, result.NaviHoldSec)});
        }
        speedLevel = static_cast<std::uint8_t>(std::clamp(result.SpeedLevel, 0, 255));
        return true;
    }

    bool Application::TryStartRegionalAreaTaskForGoal(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const UnitTeam my_team,
        const bool apply_team_offset,
        const char* reason,
        const RegionalAreaTaskOrigin origin) {
        if (!config.RegionalAreaTaskSettings.Enable ||
            (!config.RegionalAreaTaskSettings.MyHighland.Enable &&
             !config.RegionalAreaTaskSettings.MyBase.Enable &&
             !config.RegionalAreaTaskSettings.MyPreRoadland.Enable &&
             !config.RegionalAreaTaskSettings.MyReadyRoadland.Enable &&
             !config.RegionalAreaTaskSettings.CommonCentral.Enable) ||
            areaManager_.HighlandTransitionActive() ||
            areaManager_.RegionalAreaTaskActive()) {
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        const auto self_position = GetSentryPositionState(now);
        const bool has_self_position =
            self_position.Fresh && self_position.X > 0 && self_position.Y > 0;
        auto plan = areaManager_.PlanRegionalAreaTaskForGoal(
            base_goal_id,
            goal_team,
            my_team,
            apply_team_offset,
            has_self_position,
            has_self_position ? self_position.X : 0,
            has_self_position ? self_position.Y : 0,
            config.RegionalAreaTaskSettings.MyBase,
            config.RegionalAreaTaskSettings.PatrolSelection,
            now,
            IsSelfInMainArea(my_team, Area::MainAreaKind::Highland));
        if (!plan.has_value()) {
            return false;
        }
        plan->Origin = origin;
        if (plan->Type == RegionalAreaTaskType::MyReadyRoadland ||
            plan->Type == RegionalAreaTaskType::CommonCentral) {
            auto referee_value_fresh = [&](const bool received, const std::chrono::steady_clock::time_point last_rx) {
                return received &&
                    last_rx.time_since_epoch().count() != 0 &&
                    now - last_rx <= std::chrono::seconds(2);
            };
            const int healthy_hp_min = plan->Type == RegionalAreaTaskType::MyReadyRoadland
                ? config.RegionalAreaTaskSettings.MyReadyRoadland.HealthyHpMin
                : config.RegionalAreaTaskSettings.CommonCentral.HealthyHpMin;
            const int healthy_ammo_min = plan->Type == RegionalAreaTaskType::MyReadyRoadland
                ? config.RegionalAreaTaskSettings.MyReadyRoadland.HealthyAmmoMin
                : config.RegionalAreaTaskSettings.CommonCentral.HealthyAmmoMin;
            const bool data_healthy =
                referee_value_fresh(hasReceivedMyselfHealth_, lastMyselfHealthRxTime) &&
                referee_value_fresh(hasReceivedAmmoLeft_, lastAmmoLeftRxTime) &&
                myselfHealth >= static_cast<std::uint16_t>(std::max(0, healthy_hp_min)) &&
                ammoLeft >= static_cast<std::uint16_t>(std::max(0, healthy_ammo_min));
            if (!data_healthy) {
                return false;
            }
        }
        if ((plan->Type == RegionalAreaTaskType::MyHighland &&
             !config.RegionalAreaTaskSettings.MyHighland.Enable) ||
            (plan->Type == RegionalAreaTaskType::MyBase &&
             !config.RegionalAreaTaskSettings.MyBase.Enable) ||
            (plan->Type == RegionalAreaTaskType::MyPreRoadland &&
             !config.RegionalAreaTaskSettings.MyPreRoadland.Enable) ||
            (plan->Type == RegionalAreaTaskType::MyReadyRoadland &&
             !config.RegionalAreaTaskSettings.MyReadyRoadland.Enable) ||
            (plan->Type == RegionalAreaTaskType::CommonCentral &&
             !config.RegionalAreaTaskSettings.CommonCentral.Enable)) {
            return false;
        }

        areaManager_.StartRegionalAreaTask(*plan, now);
        if (LoggerPtr) {
            if (plan->Type == RegionalAreaTaskType::MyBase) {
                LoggerPtr->Info(
                    "RegionalAreaTask[MyBase] start from goal={} reason={}: start_base_goal={} selection=weighted_evaluation.",
                    static_cast<int>(ResolveGoalId(base_goal_id, goal_team, apply_team_offset)),
                    reason ? reason : "area_task",
                    static_cast<int>(plan->InitialBaseGoal));
            } else if (plan->Type == RegionalAreaTaskType::MyPreRoadland) {
                LoggerPtr->Info(
                    "RegionalAreaTask[MyPreRoadland] start from goal={} reason={}: goal=PreRoadland (BaseGoalId=25).",
                    static_cast<int>(ResolveGoalId(base_goal_id, goal_team, apply_team_offset)),
                    reason ? reason : "area_task");
            } else if (plan->Type == RegionalAreaTaskType::MyReadyRoadland) {
                LoggerPtr->Info(
                    "RegionalAreaTask[MyReadyRoadland] start from goal={} reason={}: CentralToBase -> BaseToCentral guarded crossing.",
                    static_cast<int>(ResolveGoalId(base_goal_id, goal_team, apply_team_offset)),
                    reason ? reason : "area_task");
            } else if (plan->Type == RegionalAreaTaskType::CommonCentral) {
                LoggerPtr->Info(
                    "RegionalAreaTask[CommonCentral] start from goal={} reason={}: start_base_goal={} team={} patrol=own OutpostArea->RightShoot->BuffAround2->LeftShoot->OutpostShoot->enemy RightShoot->OccupyArea->OutpostShoot.",
                    static_cast<int>(ResolveGoalId(base_goal_id, goal_team, apply_team_offset)),
                    reason ? reason : "area_task",
                    static_cast<int>(ResolveGoalId(plan->InitialBaseGoal, plan->InitialGoalTeam, apply_team_offset)),
                    plan->InitialGoalTeam == my_team ? "my" : "enemy");
            } else {
                LoggerPtr->Info(
                    "RegionalAreaTask[MyHighland] start from goal={} reason={}: Highland -> BuffShoot hold={}s -> HoleRoad.",
                    static_cast<int>(ResolveGoalId(base_goal_id, goal_team, apply_team_offset)),
                    reason ? reason : "area_task",
                    config.RegionalAreaTaskSettings.MyHighland.BuffShootHoldSec);
            }
        }
        return TickRegionalAreaTask(my_team, UnitTeam::Unknown);
    }

    bool Application::TickNaviAreaTransition() {
        if (!areaManager_.HighlandTransitionActive()) {
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        const auto& runtime = areaManager_.TransitionRuntime();
        const auto via_base_goal = AreaManager::IsValidBaseGoalId(runtime.ViaBaseGoal)
            ? runtime.ViaBaseGoal
            : LangYa::Highland.ID;
        const auto via_goal_reach = EvaluateBaseGoalReach(
            via_base_goal,
            runtime.GoalTeam,
            runtime.ApplyTeamOffset);
        const bool route_unreachable = via_goal_reach.Status == GoalReachStatus::Unreachable;
        const bool arrived = via_goal_reach.Status == GoalReachStatus::Reached;
        const auto tick = areaManager_.TickHighlandTransition(now, route_unreachable, arrived);
        gimbalControlData.FireCode.FollowMode = tick.FollowMode ? 1 : 0;

        if (tick.Action == NaviAreaTransitionTickAction::ContinueVia) {
            SetPositionByBaseGoal(
                tick.ViaBaseGoal,
                tick.GoalTeam,
                tick.ApplyTeamOffset);
            return true;
        }

        if (tick.Action == NaviAreaTransitionTickAction::FinishNoPending) {
            if (LoggerPtr) {
                LoggerPtr->Info(
                    "Area transition {} {}: via goal={} done, follow_mode off.",
                    NaviAreaTransitionKindToString(tick.Kind),
                    tick.CompletionReason,
                    static_cast<int>(tick.ViaBaseGoal));
            }
            return false;
        }

        if (tick.Action == NaviAreaTransitionTickAction::FinishWithPending) {
            SetPositionByBaseGoal(tick.PendingBaseGoal, tick.GoalTeam, tick.ApplyTeamOffset);
            if (LoggerPtr) {
                LoggerPtr->Info(
                    "Area transition {} {}: via goal={} done, continue goal={}, follow_mode off.",
                    NaviAreaTransitionKindToString(tick.Kind),
                    tick.CompletionReason,
                    static_cast<int>(tick.ViaBaseGoal),
                    static_cast<int>(naviCommandGoal));
            }
            return true;
        }
        return false;
    }

    bool Application::TryStartNaviAreaTransition(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const UnitTeam my_team,
        const bool apply_team_offset,
        const char* reason) {
        if (areaManager_.HighlandTransitionActive() ||
            !areaManager_.IsNaviAreaTransitionCompatEnabled() ||
            !AreaManager::IsValidBaseGoalId(base_goal_id) ||
            goal_team == UnitTeam::Unknown ||
            my_team == UnitTeam::Unknown) {
            return false;
        }
        const auto plan = areaManager_.PlanHighlandTransition(
            base_goal_id,
            goal_team,
            my_team,
            apply_team_offset,
            naviCommandGoal,
            IsHighlandCompatArrived(goal_team),
            IsBaseGoalArrived(LangYa::HoleRoad.ID, my_team, apply_team_offset),
            IsBaseGoalArrived(LangYa::BuffOutpost.ID, my_team, apply_team_offset),
            IsSelfInMainArea(my_team, Area::MainAreaKind::Highland));
        if (!plan.has_value()) {
            return false;
        }
        if (plan->CheckViaAlreadyArrived &&
            IsBaseGoalArrived(plan->ViaBaseGoal, plan->GoalTeam, plan->ApplyTeamOffset)) {
            return false;
        }

        areaManager_.StartHighlandTransition(*plan, std::chrono::steady_clock::now());
        gimbalControlData.FireCode.FollowMode = 1;

        SetPositionByBaseGoal(plan->ViaBaseGoal, plan->GoalTeam, plan->ApplyTeamOffset);
        if (LoggerPtr) {
            LoggerPtr->Info(
                "Area transition {} {}: via goal={} then goal={} follow_mode on.",
                NaviAreaTransitionKindToString(plan->Kind),
                reason ? reason : "start",
                static_cast<int>(naviCommandGoal),
                plan->HasPendingGoal
                    ? static_cast<int>(ResolveGoalId(plan->PendingBaseGoal, plan->GoalTeam, plan->ApplyTeamOffset))
                    : -1);
        }
        return true;
    }

    bool Application::IsRegionalDefenseAimSuppressActive() const noexcept {
        return areaManager_.IsRegionalDefenseAimSuppressActive(std::chrono::steady_clock::now());
    }

    bool Application::IsOutpostOpeningHighPriorityActive() const noexcept {
        const auto& outpost = config.TaskSettings.OutpostConfirm;
        return IsOutpostOpeningPriorityActive(
            config.TaskSettings.Outpost,
            outpost,
            ElapsedSeconds());
    }

    bool Application::ShouldSuppressChaseForOutpostTask() const noexcept {
        return config.TaskSettings.Outpost &&
            config.TaskSettings.OutpostConfirm.SuppressChaseWhileActive &&
            (outpostVisualScoutNavigationActive_ || aimMode == AimMode::Outpost);
    }

    bool Application::ShouldSuppressChaseForSpecialPatrol() const noexcept {
        return config.SpecialSettings.Patrol.Enable &&
            config.SpecialSettings.Patrol.SuppressChase &&
            GetStrategyMode() == StrategyMode::Regional &&
            !IsLeagueProfile() &&
            !IsShowcasePatrolEnabled();
    }

    bool Application::TryApplyChaseTactical(
        std::optional<AreaKey> explicit_allowed_area,
        const char* decision_detail) {
        if (!CanAuthorizeChaseTactical()) {
            return false;
        }

        const auto maybe_target_unit = UnitTypeFromArmorType(targetArmor.Type);
        if (!maybe_target_unit.has_value()) {
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        const UnitTeam my_team = team;
        const UnitTeam enemy_team = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        const int official_position_fresh_ms =
            std::max(1, config.ChaseSettings.OfficialPositionFreshMs);
        const auto target_position = GetEnemyPositionState(
            *maybe_target_unit,
            official_position_fresh_ms,
            now);
        std::optional<ResolvedAreaKey> target_area;
        if (target_position.Fresh &&
            IsOfficialFieldPointValid(target_position.X, target_position.Y)) {
            const auto exact_target_area = AreaManager::ResolveAreaKeyForPoint(
                my_team,
                enemy_team,
                target_position.X,
                target_position.Y);
            if (exact_target_area.has_value()) {
                target_area = ResolvedAreaKey{
                    .Key = *exact_target_area,
                    .UsedNearestFallback = false};
            }
        }

        const bool regional_policy_profile =
            GetStrategyMode() == StrategyMode::Regional && !IsShowcasePatrolEnabled();
        if (!regional_policy_profile && target_position.Fresh &&
            IsOfficialFieldPointValid(target_position.X, target_position.Y)) {
            const auto legacy_target_area = AreaManager::ResolveAreaKeyForPointWithNearest(
                my_team,
                enemy_team,
                target_position.X,
                target_position.Y);
            if (legacy_target_area.has_value() &&
                !IsAreaKeyAllowedForChaseTarget(
                    legacy_target_area->Key,
                    config.DecisionAutonomySettings.NaviGoal)) {
                if (LoggerPtr &&
                    now - lastOfficialChaseAreaLimitLogTime_ > std::chrono::seconds(2)) {
                    LoggerPtr->Info(
                        "Chase blocked by legacy area scope: target={} pos=({}, {}) side={} area={} nearest={}.",
                        static_cast<int>(targetArmor.Type),
                        target_position.X,
                        target_position.Y,
                        static_cast<int>(legacy_target_area->Key.Side),
                        Area::MainAreaKindName(legacy_target_area->Key.Kind),
                        legacy_target_area->UsedNearestFallback ? 1 : 0);
                    lastOfficialChaseAreaLimitLogTime_ = now;
                }
                return false;
            }
        }
        const ChasePolicyResult chase_policy = EvaluateRegionalChasePolicy(
            config.ChasePolicySettings,
            ChasePolicyContext{
                .RegionalProfile = regional_policy_profile,
                .YieldablePlan =
                    areaManager_.RegionalAreaTaskActive() &&
                    areaManager_.RegionalAreaTaskCanYieldToHigherPriority(),
                .Plan = areaManager_.RegionalAreaTaskActive()
                    ? std::optional<RegionalAreaTaskRuntime>{areaManager_.RegionalAreaTask()}
                    : std::nullopt,
                .ExplicitAllowedArea = explicit_allowed_area,
                .TargetPositionFresh = target_position.Fresh,
                .TargetArea = target_area});
        if (!chase_policy.Allowed) {
            if (LoggerPtr && now - lastOfficialChaseAreaLimitLogTime_ > std::chrono::seconds(2)) {
                LoggerPtr->Info(
                    "ChasePolicy blocked target={} reason={} position_fresh={} target_area_resolved={}.",
                    static_cast<int>(targetArmor.Type),
                    ChasePolicyReasonToString(chase_policy.Reason),
                    target_position.Fresh ? 1 : 0,
                    target_area.has_value() ? 1 : 0);
                lastOfficialChaseAreaLimitLogTime_ = now;
            }
            return false;
        }
        const bool external_aim_active = config.ExternalAimSettings.Enable;
        if (external_aim_active &&
            externalAimData.LastValidTime.time_since_epoch().count() != 0 &&
            now - externalAimData.LastValidTime >
                std::chrono::milliseconds(std::max(1, config.ExternalAimSettings.ResultFreshTimeoutMs))) {
            externalAimData.Valid = false;
            externalAimData.Fresh = false;
            externalAimData.FireStatus = false;
            externalAimData.HasLatchedAngles = false;
        }

        const AimData& active_aim_data = CurrentAimData();

        const bool find_target_callback = isFindTargetAtomic.load(std::memory_order_relaxed);
        bool find_target = false;
        bool has_recent_latched_target = false;
        const bool has_target_for_angles = CurrentAimTargetForAngles(
            find_target_callback,
            now,
            config.AimDebugSettings.LatchedTargetHoldMs,
            &find_target,
            &has_recent_latched_target);
        const auto chase_angles = has_target_for_angles ? active_aim_data.Angles : gimbalAngles;

        bool has_external_target_point = false;
        ExternalAimTargetCache external_target_point{};
        if (config.ExternalAimSettings.Enable &&
            hasExternalAimTargets_ &&
            targetArmor.Type != ArmorType::UnKnown) {
            const auto target_index = static_cast<std::size_t>(targetArmor.Type);
            const int fresh_ms = std::max(1, config.ExternalAimSettings.TargetFreshTimeoutMs);
            if (target_index < externalAimTargets_.size()) {
                const auto& cached = externalAimTargets_[target_index];
                has_external_target_point =
                    cached.Valid &&
                    cached.LastSeen.time_since_epoch().count() != 0 &&
                    now - cached.LastSeen <= std::chrono::milliseconds(fresh_ms) &&
                    std::isfinite(cached.X) &&
                    std::isfinite(cached.Y) &&
                    std::isfinite(cached.Z) &&
                    std::isfinite(cached.Distance) &&
                    cached.Distance > 0.0f &&
                    !cached.FrameId.empty();
                if (has_external_target_point) {
                    external_target_point = cached;
                }
            }
        }

        const bool chase_to_navi = config.ChaseSettings.ToNavi;
        const bool navi_to_navi = config.NaviSettings.UseXY && config.NaviSettings.ToNavi;
        bool has_chase_target = has_target_for_angles || has_external_target_point;
        if (!has_chase_target &&
            config.ChaseSettings.LostTargetHoldMs > 0 &&
            active_aim_data.HasLatchedAngles &&
            lastTargetSeenTime.time_since_epoch().count() != 0) {
            has_chase_target = (now - lastTargetSeenTime) <=
                std::chrono::milliseconds(config.ChaseSettings.LostTargetHoldMs);
        }

        bool chase_distance_valid = false;
        float distance_cm = 0.0f;
        if (has_external_target_point) {
            distance_cm = external_target_point.Distance * 100.0f;
            chase_distance_valid =
                distance_cm >= static_cast<float>(config.ChaseSettings.MinValidDistanceCm) &&
                distance_cm <= static_cast<float>(config.ChaseSettings.MaxValidDistanceCm);
        } else if (std::isfinite(targetArmor.Distance) && targetArmor.Distance > 0.0f) {
            distance_cm = targetArmor.Distance * 100.0f;
            chase_distance_valid =
                distance_cm >= static_cast<float>(config.ChaseSettings.MinValidDistanceCm) &&
                distance_cm <= static_cast<float>(config.ChaseSettings.MaxValidDistanceCm);
        }

        bool chase_output_active = false;
        if (has_chase_target &&
            chase_distance_valid &&
            targetArmor.Type != ArmorType::UnKnown) {
            const auto yaw_error_deg = static_cast<double>(
                std::remainder(chase_angles.Yaw - gimbalAngles.Yaw, 360.0f));
            const auto pitch_error_deg = static_cast<double>(
                std::remainder(chase_angles.Pitch - gimbalAngles.Pitch, 360.0f));
            const double distance_m = static_cast<double>(distance_cm) * 0.01;
            constexpr double kDegToRad = 0.017453292519943295;
            const double yaw_rad = yaw_error_deg * kDegToRad;
            const double pitch_rad = pitch_error_deg * kDegToRad;
            const double cos_pitch = std::cos(pitch_rad);

            naviRelativeTargetValid = true;
            if (has_external_target_point) {
                naviRelativeTargetX = external_target_point.X;
                naviRelativeTargetY = external_target_point.Y;
                naviRelativeTargetZ = external_target_point.Z;
                naviRelativeTargetDistance = external_target_point.Distance;
                naviRelativeTargetFrameId = external_target_point.FrameId;
            } else {
                naviRelativeTargetX = static_cast<float>(distance_m * cos_pitch * std::cos(yaw_rad));
                naviRelativeTargetY = static_cast<float>(distance_m * cos_pitch * std::sin(yaw_rad));
                naviRelativeTargetZ = static_cast<float>(distance_m * std::sin(pitch_rad));
                naviRelativeTargetDistance = static_cast<float>(distance_m);
                naviRelativeTargetFrameId.clear();
            }
            naviRelativeTargetYawErrorDeg = static_cast<float>(yaw_error_deg);
            naviRelativeTargetPitchErrorDeg = static_cast<float>(pitch_error_deg);
            naviRelativeTargetArmorType = static_cast<std::uint8_t>(targetArmor.Type);
            naviRelativeTargetAimMode = static_cast<std::uint8_t>(aimMode);
            chase_output_active = true;

            if (chase_to_navi) {
                if (config.NaviSettings.UseXY && !navi_to_navi) {
                    if (target_position.Fresh &&
                        target_position.X >= 0 &&
                        target_position.Y >= 0) {
                        naviGoalPosition.x = static_cast<std::uint16_t>(
                            std::clamp(target_position.X, 0, 65535));
                        naviGoalPosition.y = static_cast<std::uint16_t>(
                            std::clamp(target_position.Y, 0, 65535));
                    }
                }
            } else {
                const double distance_error_cm =
                    static_cast<double>(distance_cm) -
                    static_cast<double>(config.ChaseSettings.PreferredDistanceCm);

                int chase_vx = 0;
                if (std::abs(distance_error_cm) >
                    static_cast<double>(config.ChaseSettings.DistanceDeadbandCm)) {
                    chase_vx = static_cast<int>(std::lround(config.ChaseSettings.DistanceKp * distance_error_cm));
                    chase_vx = std::clamp(
                        chase_vx,
                        -config.ChaseSettings.MaxBackwardSpeed,
                        config.ChaseSettings.MaxForwardSpeed);
                }

                int chase_vy = 0;
                if (config.ChaseSettings.UseYawStrafe &&
                    std::abs(yaw_error_deg) > static_cast<double>(config.ChaseSettings.YawDeadbandDeg)) {
                    chase_vy = static_cast<int>(std::lround(config.ChaseSettings.YawKp * yaw_error_deg));
                    if (config.ChaseSettings.InvertStrafeDirection) {
                        chase_vy = -chase_vy;
                    }
                    chase_vy = std::clamp(
                        chase_vy,
                        -config.ChaseSettings.MaxStrafeSpeed,
                        config.ChaseSettings.MaxStrafeSpeed);
                }

                naviChaseVelocity = VelocityType{
                    static_cast<std::int8_t>(ClampToInt8(chase_vx)),
                    static_cast<std::int8_t>(ClampToInt8(chase_vy))
                };
                naviChaseVelocityActive_ = true;
            }
        } else if (!chase_to_navi && config.ChaseSettings.StopWhenNoTarget) {
            naviChaseVelocity = VelocityType{0, 0};
            naviChaseVelocityActive_ = true;
            chase_output_active = true;
        } else if (chase_to_navi &&
                   config.NaviSettings.UseXY &&
                   !navi_to_navi &&
                   config.ChaseSettings.StopWhenNoTarget) {
            const auto self_position = GetSentryPositionState(now);
            if (self_position.Fresh &&
                self_position.X >= 0 &&
                self_position.Y >= 0) {
                naviGoalPosition.x = static_cast<std::uint16_t>(
                    std::clamp(self_position.X, 0, 65535));
                naviGoalPosition.y = static_cast<std::uint16_t>(
                    std::clamp(self_position.Y, 0, 65535));
            }
            chase_output_active = true;
        } else if (chase_to_navi && config.ChaseSettings.StopWhenNoTarget) {
            chase_output_active = true;
        }

        if (chase_to_navi && config.ChaseSettings.UseOfficialPositionSource) {
            const bool should_try_official =
                !has_external_target_point &&
                (config.ChaseSettings.PreferOfficialPositionSource || !naviRelativeTargetValid);
            const auto self_position = GetSentryPositionState(now, official_position_fresh_ms);

            if (should_try_official && target_position.Fresh) {
                if (IsOfficialFieldPointValid(target_position.X, target_position.Y) &&
                    self_position.Fresh &&
                    IsOfficialFieldPointValid(self_position.X, self_position.Y)) {
                    const auto official_chase_goal = BuildOfficialChaseGoal(
                        self_position.X,
                        self_position.Y,
                        target_position.X,
                        target_position.Y,
                        config.ChaseSettings.PreferredDistanceCm,
                        config.ChaseSettings.DistanceDeadbandCm);
                    const auto limited_chase_goal = ApplyOfficialChaseAreaLimit(
                        self_position.X,
                        self_position.Y,
                        official_chase_goal,
                        my_team,
                        enemy_team,
                        config.DecisionAutonomySettings.NaviGoal,
                        config.ChaseSettings.AreaLimit);
                    naviGoalPosition = limited_chase_goal.Goal;
                    if ((limited_chase_goal.Limited || limited_chase_goal.Held) &&
                        now - lastOfficialChaseAreaLimitLogTime_ > std::chrono::seconds(2)) {
                        LoggerPtr->Info(
                            "Official chase area limit {} area={} self=({}, {}) raw_goal=({}, {}) limited=({}, {}).",
                            limited_chase_goal.Status,
                            limited_chase_goal.AreaName,
                            self_position.X,
                            self_position.Y,
                            static_cast<int>(official_chase_goal.x),
                            static_cast<int>(official_chase_goal.y),
                            static_cast<int>(naviGoalPosition.x),
                            static_cast<int>(naviGoalPosition.y));
                        lastOfficialChaseAreaLimitLogTime_ = now;
                    }
                    naviChaseOfficialTargetValid = true;
                    naviChaseOfficialTargetArmorType =
                        static_cast<std::uint8_t>(targetArmor.Type);
                    chase_output_active = true;
                }
            }
        }

        if (chase_output_active) {
            chaseTacticalAllowed_ = true;
            if (chase_to_navi) {
                RecordDecisionIntent(MakeDecisionIntent(
                    DecisionReason::Chase,
                    BaseGoalIdFromResolvedGoal(naviCommandGoal),
                    team,
                    true,
                    decision_detail ? decision_detail : "chase"));
            }
        }
        return chase_output_active;
    }

    bool Application::IsFortressGainPointEnemyOccupiedEventRawFresh(const int referee_fresh_ms) const noexcept {
        const auto& protect_castle = config.TacticalSettings.ProtectCastle;
        return IsProtectCastleRfidEventEnabled(
            protect_castle.Enable,
            protect_castle.RFID,
            hasReceivedEventData_ &&
            lastEventDataRxTime_.time_since_epoch().count() != 0 &&
            std::chrono::steady_clock::now() - lastEventDataRxTime_ <=
                std::chrono::milliseconds(std::max(0, referee_fresh_ms)) &&
            (eventSelfFortressGainPointStatus_ == 2U ||
             eventSelfFortressGainPointStatus_ == 3U));
    }

    bool Application::IsProtectCastleRfidStayActive(const int referee_fresh_ms) const noexcept {
        const auto& protect_castle = config.TacticalSettings.ProtectCastle;
        return IsProtectCastleRfidStayEnabled(
            protect_castle.Enable,
            protect_castle.RFID,
            protect_castle.StayWhenRfid,
            IsFortressGainPointEnemyOccupiedEventRawFresh(referee_fresh_ms));
    }

    bool Application::IsProtectCastleBaseDamageActive(
        const std::chrono::steady_clock::time_point now) const noexcept {
        constexpr auto kBaseHealthFreshTimeout = std::chrono::milliseconds(2000);
        const auto& protect_castle = config.TacticalSettings.ProtectCastle;
        const bool base_health_fresh = hasReceivedSelfBaseHealth_ &&
            lastSelfBaseHealthRxTime_.time_since_epoch().count() != 0 &&
            now - lastSelfBaseHealthRxTime_ <= kBaseHealthFreshTimeout;
        return BehaviorTree::IsProtectCastleBaseDamageActive(
            protect_castle.Enable,
            protect_castle.Base,
            base_health_fresh,
            selfBaseDamageWindow_,
            now);
    }

    void Application::UpdateProtectOutpostState(
        const std::chrono::steady_clock::time_point now) {
        const auto& setting = config.TacticalSettings.ProtectOutpost;
        if (!setting.Enable || IsLeagueProfile() || IsShowcasePatrolEnabled()) {
            protectOutpostState_ = {};
            lastObservedSelfOutpostHealthRxTime_ = {};
            return;
        }

        const bool health_fresh = hasReceivedSelfOutpostHealth_ &&
            lastSelfOutpostHealthRxTime_.time_since_epoch().count() != 0 &&
            now - lastSelfOutpostHealthRxTime_ <=
                std::chrono::milliseconds(std::max(0, setting.HealthFreshMs));
        const auto target = Area::ProtectOutpost(team);
        const auto target_goal_id = ResolveGoalId(LangYa::ProtectOutpost.ID, team, true);
        const bool owns_navigation_goal =
            naviCommandGoal == target_goal_id &&
            naviGoalPosition.x == target.x &&
            naviGoalPosition.y == target.y;
        const auto reach = owns_navigation_goal
            ? EvaluateNaviGoalReach(
                target_goal_id,
                target,
                std::max(1, config.DecisionAutonomySettings.NaviGoal.HighlandCompatArriveDistanceCm),
                0,
                LangYa::ProtectOutpost.ID,
                0)
            : GoalReachState{};

        const auto old_phase = protectOutpostState_.Phase;
        const auto old_generation = protectOutpostState_.ActiveEventGeneration;
        const bool has_new_health_sample = health_fresh &&
            lastSelfOutpostHealthRxTime_ != lastObservedSelfOutpostHealthRxTime_;
        const bool outpost_destroyed = has_new_health_sample && selfOutpostHealth == 0;
        if (has_new_health_sample) {
            lastObservedSelfOutpostHealthRxTime_ = lastSelfOutpostHealthRxTime_;
            (void)ObserveProtectOutpostHealth(
                protectOutpostState_,
                selfOutpostHealth,
                true,
                lastSelfOutpostHealthRxTime_,
                std::chrono::milliseconds(std::max(1, setting.DamageWindowMs)),
                std::max(1, setting.DamageThresholdHp));
        }
        if (outpost_destroyed && owns_navigation_goal) {
            naviGoalPublishAllowed_ = false;
            naviExternalStatusGoalInitialized_ = false;
            areaManager_.ClearProgressWatchdog();
        }
        TickProtectOutpost(
            protectOutpostState_,
            health_fresh,
            owns_navigation_goal && reach.Status == GoalReachStatus::Reached,
            owns_navigation_goal && reach.Status == GoalReachStatus::Unreachable,
            now,
            std::chrono::seconds(std::max(0, setting.SearchHoldSec)),
            std::chrono::seconds(std::max(0, setting.UnreachableCooldownSec)));

        if (LoggerPtr &&
            (old_phase != protectOutpostState_.Phase ||
             old_generation != protectOutpostState_.ActiveEventGeneration)) {
            LoggerPtr->Info(
                "ProtectOutpost: phase={} generation={} hp={} fresh={} target=({}, {}).",
                ProtectOutpostPhaseToString(protectOutpostState_.Phase),
                protectOutpostState_.ActiveEventGeneration,
                selfOutpostHealth,
                health_fresh ? 1 : 0,
                static_cast<int>(target.x),
                static_cast<int>(target.y));
        }
        if (LoggerPtr && outpost_destroyed && owns_navigation_goal) {
            LoggerPtr->Info("ProtectOutpost: own outpost destroyed, revoke C3/C4 navigation.");
        }
    }

    bool Application::TrySetProtectOutpostGoal(
        const UnitTeam my_team,
        const UnitTeam enemy_team) {
        (void)enemy_team;
        const auto& setting = config.TacticalSettings.ProtectOutpost;
        if (!setting.Enable ||
            GetStrategyMode() != StrategyMode::Regional ||
            IsLeagueProfile() ||
            IsShowcasePatrolEnabled() ||
            (protectOutpostState_.Phase != ProtectOutpostPhase::Travel &&
             protectOutpostState_.Phase != ProtectOutpostPhase::SearchHold)) {
            return false;
        }

        const auto target = Area::ProtectOutpost(my_team);
        const auto target_goal_id = ResolveGoalId(LangYa::ProtectOutpost.ID, my_team, true);
        const bool goal_changed = naviCommandGoal != target_goal_id ||
            naviGoalPosition.x != target.x ||
            naviGoalPosition.y != target.y ||
            !naviGoalPublishAllowed_;
        if (goal_changed) {
            naviCommandGoal = target_goal_id;
            naviGoalPosition = target;
            naviGoalPublishAllowed_ = true;
            naviExternalStatusGoalInitialized_ = false;
            UpdateNaviProgressWatchdogGoal(LangYa::ProtectOutpost.ID, my_team, true);
            naviCommandIntervalClock.reset(Seconds{1});
        }
        speedLevel = 1;

        std::string detail = "phase=";
        detail += ProtectOutpostPhaseToString(protectOutpostState_.Phase);
        detail += " generation=" + std::to_string(protectOutpostState_.ActiveEventGeneration);
        detail += " priority=" + std::to_string(config.TacticalSettings.Priority.ProtectOutpost);
        RecordDecisionIntent(MakeDecisionIntent(
            DecisionReason::ProtectOutpost,
            LangYa::ProtectOutpost.ID,
            my_team,
            true,
            detail.c_str()));
        return true;
    }

    CastleOccupancyResolution Application::ResolveProtectCastleOccupancy(
        const std::chrono::steady_clock::time_point now) const {
        const auto& protect_castle = config.TacticalSettings.ProtectCastle;
        const int position_fresh_ms = std::max(1, protect_castle.OccupancyPositionFreshMs);
        const int castle_margin_cm = std::max(0, protect_castle.CastlePositionMarginCm);
        const int arrival_grace_ms = std::max(0, protect_castle.ArrivalConfirmGraceMs);
        const int referee_fresh_ms = std::max(
            std::max(0, config.TaskSettings.BuffConfirm.RefereeFreshTimeoutMs),
            std::max(0, config.TaskSettings.OutpostConfirm.RefereeFreshTimeoutMs));
        const bool referee_fresh = hasReceivedEventData_ &&
            lastEventDataRxTime_.time_since_epoch().count() != 0 &&
            now - lastEventDataRxTime_ <= std::chrono::milliseconds(referee_fresh_ms);
        const bool self_rfid_at_castle = hasReceivedRfidStatus_ &&
            lastRfidStatusRxTime_.time_since_epoch().count() != 0 &&
            now - lastRfidStatusRxTime_ <= kRfidFreshTimeout &&
            rfidMatchState.SelfFortressGainPoint;
        const auto self_position = GetSentryPositionState(now, position_fresh_ms);
        const bool self_position_at_castle = self_position.Fresh &&
            Area::IsPointInsideCastleAreaWithMargin(
                team, self_position.X, self_position.Y, castle_margin_cm);

        bool teammate_at_castle = false;
        for (const auto unit_type : RobotLists) {
            if (unit_type == UnitType::Sentry) {
                continue;
            }
            const auto teammate = GetFriendPositionState(unit_type, position_fresh_ms, now);
            if (teammate.Fresh && Area::IsPointInsideCastleAreaWithMargin(
                    team, teammate.X, teammate.Y, castle_margin_cm)) {
                teammate_at_castle = true;
                break;
            }
        }

        const bool reached_castle = hasReceivedNaviReach_ && naviReach &&
            lastNaviReachRxTime_.time_since_epoch().count() != 0 &&
            now - lastNaviReachRxTime_ <= std::chrono::milliseconds(arrival_grace_ms) &&
            BaseGoalIdFromResolvedGoal(naviCommandGoal) == LangYa::Castle.ID;
        if (reached_castle) {
            castleReachedGraceUntil_ = now + std::chrono::milliseconds(arrival_grace_ms);
        }
        const bool reached_castle_grace =
            castleReachedGraceUntil_.time_since_epoch().count() != 0 &&
            now < castleReachedGraceUntil_;

        lastCastleOccupancy_ = ResolveCastleOccupancy({
            .RefereeFresh = referee_fresh,
            .RefereeStatus = eventSelfFortressGainPointStatus_,
            .SelfRfidAtCastle = self_rfid_at_castle,
            .SelfPositionAtCastle = self_position_at_castle,
            .SelfCaptureConfirmed = protect_castle.Enable && protect_castle.RFID &&
                castleCaptureAttribution_.SelfCaptureConfirmed && self_rfid_at_castle,
            .ReachedCastleGrace = reached_castle_grace,
            .TeammatePositionAtCastle = teammate_at_castle,
        });
        return lastCastleOccupancy_;
    }

    bool Application::IsFortressGainPointEnemyOccupiedEventFresh(const int referee_fresh_ms) const noexcept {
        const auto now = std::chrono::steady_clock::now();
        if (fortressGainPointDegradedUntil_.time_since_epoch().count() != 0 &&
            now < fortressGainPointDegradedUntil_) {
            return false;
        }
        return IsFortressGainPointEnemyOccupiedEventRawFresh(referee_fresh_ms);
    }

    bool Application::IsFriendPositionFresh(
        const UnitType unit_type,
        const int fresh_ms) const {
        return GetFriendPositionState(
            unit_type,
            fresh_ms,
            std::chrono::steady_clock::now()).Fresh;
    }

    bool Application::IsFriendHealthFresh(
        const UnitType unit_type,
        const int fresh_ms) const {
        const auto index = static_cast<std::size_t>(unit_type);
        if (index >= lastFriendHealthRxTime_.size()) {
            return false;
        }
        const auto& last_rx = lastFriendHealthRxTime_[index];
        if (last_rx.time_since_epoch().count() == 0) {
            return false;
        }
        return std::chrono::steady_clock::now() - last_rx <=
            std::chrono::milliseconds(std::max(1, fresh_ms));
    }

    bool Application::IsEnemyPositionFresh(
        const UnitType unit_type,
        const int fresh_ms) const {
        return GetEnemyPositionState(
            unit_type,
            fresh_ms,
            std::chrono::steady_clock::now()).Fresh;
    }

    std::optional<RegionalDefenseThreat> Application::EvaluateRegionalDefenseThreat(
        const UnitTeam my_team,
        const UnitTeam enemy_team) const {
        const auto& defense = config.RegionalDefenseSettings;
        if (!defense.Enable || IsLeagueProfile() || IsShowcasePatrolEnabled()) {
            return std::nullopt;
        }

        std::vector<RegionalDefenseEnemyPosition> fresh_enemies;
        const auto now = std::chrono::steady_clock::now();
        for (const auto unit_type : RobotLists) {
            const auto enemy_position = GetEnemyPositionState(
                unit_type,
                defense.EnemyPositionFreshMs,
                now);
            if (!enemy_position.Fresh) {
                continue;
            }
            if (enemy_position.X <= 0 || enemy_position.Y <= 0) {
                continue;
            }
            // /ly/position/data is normalized into official-field centimeters in SubscribeMessage.cpp.
            // Keep regional defense area tests in that frame; do not mix map/odom coordinates here.
            fresh_enemies.push_back(RegionalDefenseEnemyPosition{
                .X = enemy_position.X,
                .Y = enemy_position.Y});
        }

        const auto& protect_castle = config.TacticalSettings.ProtectCastle;
        const bool protect_castle_enemy_position = IsProtectCastleEnemyPositionEnabled(
            protect_castle.Enable,
            protect_castle.EnemyPos);
        auto threat = areaManager_.AnalyzeRegionalDefenseThreat(
            my_team,
            enemy_team,
            defense.EnableSoftEnemySideThreat,
            protect_castle_enemy_position,
            fresh_enemies);

        const int referee_fresh_ms = std::max(
            std::max(0, config.TaskSettings.BuffConfirm.RefereeFreshTimeoutMs),
            std::max(0, config.TaskSettings.OutpostConfirm.RefereeFreshTimeoutMs));
        const bool protect_castle_rfid_stay_active =
            IsProtectCastleRfidStayActive(referee_fresh_ms);
        const bool protect_castle_base_damage_active =
            IsProtectCastleBaseDamageActive(now);
        if (protect_castle_rfid_stay_active ||
            IsFortressGainPointEnemyOccupiedEventFresh(referee_fresh_ms)) {
            threat.OwnFortressGainPointEnemyOccupied = true;
            threat.HardThreat = true;
        }
        if (protect_castle_base_damage_active) {
            threat.OwnBaseDamageActive = true;
            threat.HardThreat = true;
        }

        if (!threat.HardThreat && !threat.SoftEnemySideThreat) {
            return std::nullopt;
        }
        return threat;
    }

    bool Application::TrySetRegionalDefenseGoal(
        const UnitTeam my_team,
        const UnitTeam enemy_team,
        const RegionalDefenseSelection selection) {
        const auto& defense = config.RegionalDefenseSettings;
        const auto& protect_castle = config.TacticalSettings.ProtectCastle;
        const auto maybe_threat = EvaluateRegionalDefenseThreat(my_team, enemy_team);
        if (!maybe_threat.has_value()) {
            const int referee_fresh_ms = std::max(
                std::max(0, config.TaskSettings.BuffConfirm.RefereeFreshTimeoutMs),
                std::max(0, config.TaskSettings.OutpostConfirm.RefereeFreshTimeoutMs));
            const auto now = std::chrono::steady_clock::now();
            const bool raw_fortress_event =
                IsFortressGainPointEnemyOccupiedEventRawFresh(referee_fresh_ms);
            const bool visual_contact_recent =
                isFindTargetAtomic.load(std::memory_order_relaxed) ||
                (lastTargetSeenTime.time_since_epoch().count() != 0 &&
                 now - lastTargetSeenTime <=
                    std::chrono::seconds(std::max(1, defense.SearchNoTargetSec)));
            if (!raw_fortress_event) {
                fortressGainPointNoContactSince_ = {};
                fortressGainPointDegradedUntil_ = {};
            } else if (visual_contact_recent) {
                fortressGainPointDegradedUntil_ = {};
            }
            fortressGainPointEnemyCount_ = 0;
            regionalDefenseSearchKind_ = RegionalDefenseSearchKind::None;
            regionalDefenseSearchIndex_ = 0U;
            regionalDefenseSearchBaseGoal_ = LangYa::Home.ID;
            regionalDefenseSearchStartTime_ = {};
            regionalDefenseSearchArrivedTime_ = {};
            regionalDefenseSearchDirection_ = 1;
            return false;
        }
        const auto threat = *maybe_threat;
        const auto now = std::chrono::steady_clock::now();
        const int referee_fresh_ms = std::max(
            std::max(0, config.TaskSettings.BuffConfirm.RefereeFreshTimeoutMs),
            std::max(0, config.TaskSettings.OutpostConfirm.RefereeFreshTimeoutMs));
        const bool protect_castle_rfid_stay_active =
            IsProtectCastleRfidStayActive(referee_fresh_ms);
        if (!threat.OwnFortressGainPointEnemyOccupied &&
            threat.OwnBaseCount > 0 &&
            IsFortressGainPointEnemyOccupiedEventRawFresh(referee_fresh_ms)) {
            fortressGainPointDegradedUntil_ = {};
        }

        if (threat.OwnFortressGainPointEnemyOccupied) {
            fortressGainPointEnemyCount_ = threat.OwnBaseCount;
            if (protect_castle_rfid_stay_active) {
                fortressGainPointNoContactSince_ = {};
            } else {
                const bool visual_contact_recent =
                    isFindTargetAtomic.load(std::memory_order_relaxed) ||
                    (lastTargetSeenTime.time_since_epoch().count() != 0 &&
                     now - lastTargetSeenTime <=
                        std::chrono::seconds(std::max(1, defense.SearchNoTargetSec)));
                const bool has_fortress_contact =
                    threat.OwnBaseCount > 0 ||
                    visual_contact_recent;
                if (has_fortress_contact) {
                    fortressGainPointNoContactSince_ = {};
                } else {
                    if (fortressGainPointNoContactSince_.time_since_epoch().count() == 0) {
                        fortressGainPointNoContactSince_ = now;
                    } else if (now - fortressGainPointNoContactSince_ >=
                               std::chrono::seconds(std::max(1, defense.FortressNoContactDegradeSec))) {
                        fortressGainPointDegradedUntil_ =
                            now + std::chrono::seconds(std::max(1, defense.FortressDegradeCooldownSec));
                        fortressGainPointNoContactSince_ = {};
                        fortressGainPointEnemyCount_ = 0;
                        regionalDefenseSearchKind_ = RegionalDefenseSearchKind::None;
                        regionalDefenseSearchIndex_ = 0U;
                        regionalDefenseSearchBaseGoal_ = LangYa::Home.ID;
                        regionalDefenseSearchStartTime_ = {};
                        regionalDefenseSearchArrivedTime_ = {};
                        regionalDefenseSearchDirection_ = 1;
                        if (LoggerPtr) {
                            LoggerPtr->Warning(
                                "Fortress gain-point event degraded: status={} no own-base enemy position and no visual target for {}s; cooldown={}s.",
                                static_cast<int>(eventSelfFortressGainPointStatus_),
                                std::max(1, defense.FortressNoContactDegradeSec),
                                std::max(1, defense.FortressDegradeCooldownSec));
                        }
                        return false;
                    }
                }
            }
        } else {
            fortressGainPointEnemyCount_ = 0;
            fortressGainPointNoContactSince_ = {};
        }

        if (!threat.HardThreat) {
            if (aimMode == AimMode::Buff || aimMode == AimMode::Outpost ||
                !naviCommandIntervalClock.trigger()) {
                return false;
            }
        }

        const bool strong_resource =
            myselfHealth >= defense.StrongHealthMin &&
            ammoLeft >= defense.StrongAmmoMin;

        RegionalDefenseSearchKind search_kind = RegionalDefenseSearchKind::None;
        const char* reason = "regional_defense";
        int hold_sec = defense.HardHoldSec;
        std::vector<std::uint8_t> candidates;
        auto order_nearest_base_candidates = [&](std::vector<std::uint8_t> goals) {
            const auto self_position = GetSentryPositionState(now);
            if (!self_position.Fresh || self_position.X <= 0 || self_position.Y <= 0) {
                return goals;
            }
            std::stable_sort(
                goals.begin(),
                goals.end(),
                [&](const std::uint8_t lhs, const std::uint8_t rhs) {
                    const auto lhs_point = AreaManager::GoalPointByBaseId(lhs, my_team);
                    const auto rhs_point = AreaManager::GoalPointByBaseId(rhs, my_team);
                    return AreaManager::DistanceSq(
                        self_position.X,
                        self_position.Y,
                        static_cast<int>(lhs_point.x),
                        static_cast<int>(lhs_point.y)) <
                        AreaManager::DistanceSq(
                            self_position.X,
                            self_position.Y,
                            static_cast<int>(rhs_point.x),
                            static_cast<int>(rhs_point.y));
                });
            return goals;
        };

        const bool protect_castle_enemy_position_active =
            protect_castle.EnemyPos && threat.OwnBaseCount > 0;
        const bool protect_castle_source_active =
            threat.OwnFortressGainPointEnemyOccupied ||
            threat.OwnBaseDamageActive ||
            protect_castle_enemy_position_active;
        const auto castle_occupancy = protect_castle_source_active
            ? ResolveProtectCastleOccupancy(now)
            : CastleOccupancyResolution{};

        if (threat.HardThreat) {
            if (selection == RegionalDefenseSelection::CommonCentral) {
                if (threat.CommonCentralCount > 0 &&
                    config.TacticalSettings.RegionalDefense.CommonCentral.Enable) {
                    search_kind = RegionalDefenseSearchKind::CommonCentral;
                    reason = "common_central";
                    hold_sec = std::max(
                        1,
                        config.TacticalSettings.RegionalDefense.CommonCentral.HoldSec);
                    candidates = {
                        LangYa::HoleRoad.ID,
                        LangYa::CentralHigh.ID,
                        LangYa::CentralLow.ID,
                        LangYa::OutpostGuard.ID};
                }
            } else if (protect_castle_source_active) {
                search_kind = RegionalDefenseSearchKind::OwnFortressGainPoint;
                reason = CastleOccupancyActionToString(castle_occupancy.Action);
                if (castle_occupancy.Action == CastleOccupancyAction::ApproachCastle ||
                    castle_occupancy.Action == CastleOccupancyAction::HoldCastle) {
                    candidates = {LangYa::Castle.ID};
                } else {
                    candidates = order_nearest_base_candidates({
                        LangYa::CastleLeft1.ID,
                        LangYa::CastleLeft2.ID,
                        LangYa::CastleRight1.ID,
                        LangYa::CastleRight2.ID
                    });
                }
            } else if (threat.OwnBaseCount > 0) {
                search_kind = RegionalDefenseSearchKind::OwnBase;
                reason = threat.OwnBaseCount >= defense.MultiEnemyBaseCount
                    ? "own_base_multi"
                    : (strong_resource ? "own_base_chase" : "own_base_guard");
                candidates = {
                    LangYa::Castle.ID,
                    LangYa::CastleRight2.ID,
                    LangYa::CastleLeft2.ID,
                    LangYa::CastleRight1.ID,
                    LangYa::CastleLeft1.ID
                };
            } else if (threat.OwnHighlandCount > 0 &&
                       (threat.OwnPreRoadlandCount > 0 || threat.OwnReadyRoadlandCount > 0)) {
                search_kind = RegionalDefenseSearchKind::OwnHighlandRoadCorridor;
                reason = "own_highland_road_corridor";
                candidates = {LangYa::Castle.ID, LangYa::HoleRoad.ID, LangYa::CastleRight2.ID};
            } else if (threat.OwnPreRoadlandCount > 0 || threat.OwnReadyRoadlandCount > 0) {
                search_kind = RegionalDefenseSearchKind::OwnRoadCorridor;
                reason = "own_road_corridor";
                candidates = {LangYa::CastleRight2.ID, LangYa::CastleRight1.ID, LangYa::Castle.ID};
            } else if (threat.OwnHighlandCount > 0) {
                search_kind = RegionalDefenseSearchKind::OwnHighland;
                reason = "own_highland";
                candidates = {LangYa::HoleRoad.ID, LangYa::Highland.ID, LangYa::Castle.ID};
            }
        } else {
            search_kind = RegionalDefenseSearchKind::EnemySideSoft;
            reason = (threat.EnemyPreRoadlandCount > 0 || threat.EnemyReadyRoadlandCount > 0)
                ? "enemy_road_corridor_soft"
                : "enemy_highland_soft";
            hold_sec = defense.SoftHoldSec;
            candidates = {LangYa::HoleRoad.ID, LangYa::Highland.ID, LangYa::Castle.ID};
        }

        if (candidates.empty()) {
            // CommonCentral is now an independent Tactical candidate. The
            // higher-priority ProtectCastle probe runs first every tick, so
            // it must not reset an active four-point search merely because
            // the central threat was intentionally deferred to that candidate.
            if (selection == RegionalDefenseSelection::ProtectCastle &&
                regionalDefenseSearchKind_ == RegionalDefenseSearchKind::CommonCentral) {
                return false;
            }
            regionalDefenseSearchKind_ = RegionalDefenseSearchKind::None;
            regionalDefenseSearchIndex_ = 0U;
            regionalDefenseSearchBaseGoal_ = LangYa::Home.ID;
            regionalDefenseSearchStartTime_ = {};
            regionalDefenseSearchArrivedTime_ = {};
            regionalDefenseSearchDirection_ = 1;
            return false;
        }

        if (threat.HardThreat) {
            areaManager_.StartRegionalDefenseSuppress(now, defense.HardHoldSec);
            aimMode = AimMode::RotateScan;
        }

        const bool same_search = regionalDefenseSearchKind_ == search_kind;
        if (!same_search) {
            regionalDefenseSearchKind_ = search_kind;
            regionalDefenseSearchIndex_ = 0U;
            regionalDefenseSearchDirection_ = 1;
            if (search_kind == RegionalDefenseSearchKind::CommonCentral) {
                const auto self_position = GetSentryPositionState(now);
                if (self_position.Fresh && self_position.X > 0 && self_position.Y > 0) {
                    double nearest_distance = std::numeric_limits<double>::infinity();
                    for (std::size_t index = 0; index < candidates.size(); ++index) {
                        const auto point = AreaManager::GoalPointByBaseId(candidates[index], my_team);
                        const auto distance = AreaManager::DistanceSq(
                            self_position.X,
                            self_position.Y,
                            static_cast<int>(point.x),
                            static_cast<int>(point.y));
                        if (distance < nearest_distance) {
                            nearest_distance = distance;
                            regionalDefenseSearchIndex_ = index;
                        }
                    }
                    regionalDefenseSearchDirection_ =
                        regionalDefenseSearchIndex_ + 1U >= candidates.size() ? -1 : 1;
                }
            }
            regionalDefenseSearchBaseGoal_ = candidates[regionalDefenseSearchIndex_];
            regionalDefenseSearchStartTime_ = now;
            regionalDefenseSearchArrivedTime_ = {};
        } else if (regionalDefenseSearchIndex_ >= candidates.size()) {
            regionalDefenseSearchIndex_ = 0U;
            regionalDefenseSearchDirection_ = 1;
            regionalDefenseSearchBaseGoal_ = candidates.front();
            regionalDefenseSearchStartTime_ = now;
            regionalDefenseSearchArrivedTime_ = {};
        } else {
            regionalDefenseSearchBaseGoal_ = candidates[regionalDefenseSearchIndex_];
        }

        auto visual_target_recently_seen = [&]() {
            return CurrentAimFreshOrLatched(
                now,
                std::max(1, defense.SearchNoTargetSec) * 1000);
        };

        const auto current_goal_reach = EvaluateBaseGoalReach(
            regionalDefenseSearchBaseGoal_, my_team, true);
        const bool current_search_done =
            current_goal_reach.Status == GoalReachStatus::Reached ||
            current_goal_reach.Status == GoalReachStatus::Unreachable;
        const bool common_central_search =
            search_kind == RegionalDefenseSearchKind::CommonCentral;
        const int search_hold_sec = common_central_search
            ? hold_sec
            : defense.SearchHoldSec;
        if (common_central_search &&
            current_goal_reach.Status == GoalReachStatus::Reached &&
            regionalDefenseSearchArrivedTime_.time_since_epoch().count() == 0) {
            regionalDefenseSearchArrivedTime_ = now;
        }

        const auto search_hold_start = common_central_search
            ? regionalDefenseSearchArrivedTime_
            : regionalDefenseSearchStartTime_;
        const bool search_held_long_enough =
            search_hold_start.time_since_epoch().count() != 0 &&
            now - search_hold_start >=
                std::chrono::seconds(std::max(1, search_hold_sec));
        const bool should_advance_search = common_central_search
            ? same_search &&
                candidates.size() > 1U &&
                ShouldAdvanceCommonCentralSearch({
                    .GoalReached = current_goal_reach.Status == GoalReachStatus::Reached,
                    .GoalUnreachable = current_goal_reach.Status == GoalReachStatus::Unreachable,
                    .HoldElapsed = search_held_long_enough,
                    .VisualTargetRecentlySeen = visual_target_recently_seen()})
            : same_search &&
                candidates.size() > 1U &&
                (current_search_done || search_held_long_enough) &&
                !visual_target_recently_seen();

        const bool protect_castle_perimeter_defense =
            protect_castle_source_active &&
            castle_occupancy.Action == CastleOccupancyAction::PerimeterDefense;
        const auto try_protect_castle_perimeter_chase = [&]() {
            if (!protect_castle_perimeter_defense) {
                return false;
            }
            const auto active_output_reach = EvaluateNaviGoalReach(
                naviCommandGoal,
                naviGoalPosition,
                std::max(1, config.DecisionAutonomySettings.NaviGoal.HighlandCompatArriveDistanceCm),
                0,
                BaseGoalIdFromResolvedGoal(naviCommandGoal),
                GoalReachTimeoutSecForBaseGoal(BaseGoalIdFromResolvedGoal(naviCommandGoal)));
            if (active_output_reach.Status == GoalReachStatus::Unreachable) {
                return false;
            }
            return TryApplyChaseTactical(
                AreaKey{
                    .Side = AreaSide::My,
                    .Kind = Area::MainAreaKind::Base,
                    .Team = my_team},
                "protect_castle_perimeter_chase");
        };

        if (should_advance_search) {
            const auto previous_goal = regionalDefenseSearchBaseGoal_;
            if (common_central_search) {
                regionalDefenseSearchDirection_ = ReverseCommonCentralSearchDirection(
                    regionalDefenseSearchIndex_ % kCommonCentralSearchGoals.size(),
                    regionalDefenseSearchDirection_);
                regionalDefenseSearchIndex_ = NextCommonCentralSearchIndex(
                    regionalDefenseSearchIndex_ % kCommonCentralSearchGoals.size(),
                    regionalDefenseSearchDirection_);
            } else {
                regionalDefenseSearchIndex_ =
                    (regionalDefenseSearchIndex_ + 1U) % candidates.size();
            }
            regionalDefenseSearchBaseGoal_ = candidates[regionalDefenseSearchIndex_];
            regionalDefenseSearchStartTime_ = now;
            regionalDefenseSearchArrivedTime_ = {};
            if (LoggerPtr) {
                const char* trigger = current_goal_reach.Status == GoalReachStatus::Unreachable
                    ? "external_unreachable"
                    : (current_goal_reach.Status == GoalReachStatus::Reached
                        ? GoalReachReasonToString(current_goal_reach.Reason)
                        : "search_hold_timeout");
                LoggerPtr->Info(
                    "Regional defense switch: {} -> {} trigger={} reach_status={} reach_reason={} external_reachable={} external_reached={}.",
                    static_cast<int>(previous_goal),
                    static_cast<int>(regionalDefenseSearchBaseGoal_),
                    trigger,
                    GoalReachStatusToString(current_goal_reach.Status),
                    GoalReachReasonToString(current_goal_reach.Reason),
                    current_goal_reach.ExternalReachable.has_value()
                        ? (*current_goal_reach.ExternalReachable ? "true" : "false")
                        : "stale",
                    current_goal_reach.ExternalReach.has_value()
                        ? (*current_goal_reach.ExternalReach ? "true" : "false")
                        : "stale");
            }
        }

        const auto current_goal_id =
            ResolveGoalId(regionalDefenseSearchBaseGoal_, my_team, true);
        auto set_defense_goal = [&](const std::uint8_t goal_id) {
            if (!AreaManager::IsValidBaseGoalId(goal_id)) {
                return false;
            }
            if (!IsNaviGoalAllowedByAreaScope(goal_id, my_team, my_team, enemy_team)) {
                naviGoalPublishAllowed_ = false;
                return false;
            }
            if (!TryStartNaviAreaTransition(goal_id, my_team, my_team, true, reason)) {
                SetPositionByBaseGoal(goal_id, my_team, true);
            }
            RecordDecisionIntent(MakeDecisionIntent(
                DecisionReasonFromString(reason),
                goal_id,
                my_team,
                true,
                reason));
            naviCommandIntervalClock.reset(Seconds{std::max(1, hold_sec)});
            speedLevel = 1;
            if (LoggerPtr) {
                LoggerPtr->Info(
                    "Regional defense {} kind={} search_index={} goal={} own_fortress_gain_point_enemy={} base_damage={} fortress_status={} castle_action={} self_likely_at_castle={} teammate_likely_at_castle={} fortress_enemy_count={} own_base={} own_highland={} own_pre_roadland={} own_ready_roadland={} common_central={} enemy_highland={} enemy_pre_roadland={} enemy_ready_roadland={}",
                    reason,
                    RegionalDefenseSearchKindToString(search_kind),
                    regionalDefenseSearchIndex_,
                    static_cast<int>(naviCommandGoal),
                    threat.OwnFortressGainPointEnemyOccupied ? 1 : 0,
                    threat.OwnBaseDamageActive ? 1 : 0,
                    static_cast<int>(eventSelfFortressGainPointStatus_),
                    CastleOccupancyActionToString(castle_occupancy.Action),
                    castle_occupancy.SelfLikelyAtCastle ? 1 : 0,
                    castle_occupancy.TeammateLikelyAtCastle ? 1 : 0,
                    fortressGainPointEnemyCount_,
                    threat.OwnBaseCount,
                    threat.OwnHighlandCount,
                    threat.OwnPreRoadlandCount,
                    threat.OwnReadyRoadlandCount,
                    threat.CommonCentralCount,
                    threat.EnemyHighlandCount,
                    threat.EnemyPreRoadlandCount,
                    threat.EnemyReadyRoadlandCount);
            }
            return true;
        };

        if (same_search &&
            !should_advance_search &&
            regionalDefenseSearchBaseGoal_ != LangYa::Home.ID &&
            naviCommandGoal == current_goal_id) {
            if (!try_protect_castle_perimeter_chase()) {
                // An official-coordinate Chase may have overwritten this point
                // on the prior tick. Reapply the perimeter fallback and reset
                // the external reach-status binding only when it differs.
                const auto perimeter_point = AreaManager::GoalPointByBaseId(
                    regionalDefenseSearchBaseGoal_, my_team);
                if (naviGoalPosition.x != perimeter_point.x ||
                    naviGoalPosition.y != perimeter_point.y) {
                    set_defense_goal(regionalDefenseSearchBaseGoal_);
                }
            }
            return true;
        }

        for (std::size_t attempt = 0; attempt < candidates.size(); ++attempt) {
            const auto index = (regionalDefenseSearchIndex_ + attempt) % candidates.size();
            regionalDefenseSearchIndex_ = index;
            regionalDefenseSearchBaseGoal_ = candidates[index];
            if (set_defense_goal(regionalDefenseSearchBaseGoal_)) {
                regionalDefenseSearchStartTime_ = now;
                regionalDefenseSearchArrivedTime_ = {};
                try_protect_castle_perimeter_chase();
                return true;
            }
        }

        // A valid ProtectCastle occupancy must not fall through to generic
        // Chase when the area scope refuses every fixed defense point.
        return protect_castle_source_active;
    }

    bool Application::TrySetProtectHeroGoal(
        const UnitTeam my_team,
        const UnitTeam enemy_team) {
        const auto& protection = config.TacticalSettings.ProtectHero;
        if (!protection.Enable ||
            IsLeagueProfile() ||
            IsShowcasePatrolEnabled() ||
            ElapsedSeconds() < protection.StartElapsedSec) {
            protectHeroActive_ = false;
            return false;
        }

        constexpr UnitType hero_unit = UnitType::Hero;
        const auto hero_position = GetFriendPositionState(
            hero_unit,
            protection.FriendPositionFreshMs,
            std::chrono::steady_clock::now());
        if (!hero_position.Fresh) {
            protectHeroActive_ = false;
            return false;
        }
        const bool hero_in_highland =
            Area::IsPointInsideMainArea(
                my_team,
                Area::MainAreaKind::Highland,
                hero_position.X,
                hero_position.Y);
        const bool hero_in_protect_hero =
            Area::IsPointInsideProtectHeroArea(my_team, hero_position.X, hero_position.Y);
        if (!IsOfficialFieldPointValid(hero_position.X, hero_position.Y) ||
            (!hero_in_highland && !hero_in_protect_hero)) {
            protectHeroActive_ = false;
            return false;
        }

        if (IsFriendHealthFresh(hero_unit, protection.FriendHealthFreshMs) &&
            friendRobots[hero_unit].currentHealth_ == 0) {
            protectHeroActive_ = false;
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        const auto maybe_threat = EvaluateRegionalDefenseThreat(my_team, enemy_team);
        const bool has_enemy_threat = maybe_threat.has_value();
        const bool legacy_threat_ready = has_enemy_threat &&
            maybe_threat->OwnBaseCount > 0 && maybe_threat->OwnHighlandCount > 0;
        const bool should_start_protect = BehaviorTree::ShouldHoldProtectHero(
            protection.ProactiveHoldWhenHeroInHighland,
            legacy_threat_ready);
        if (should_start_protect) {
            protectHeroActive_ = true;
            if (has_enemy_threat) {
                protectHeroLastEnemySeenTime_ = now;
            }
        } else if (protectHeroActive_ && has_enemy_threat) {
            protectHeroLastEnemySeenTime_ = now;
        } else if (!protectHeroActive_) {
            return false;
        } else {
            const auto release_after =
                std::chrono::seconds(std::max(1, protection.NoEnemyReleaseSec));
            if (protectHeroLastEnemySeenTime_.time_since_epoch().count() == 0 ||
                now - protectHeroLastEnemySeenTime_ >= release_after) {
                protectHeroActive_ = false;
                protectHeroLastEnemySeenTime_ = {};
                return false;
            }
        }

        const std::uint8_t goal_base_id = AreaManager::IsValidBaseGoalId(protection.GoalBaseId)
            ? protection.GoalBaseId
            : LangYa::Highland.ID;
        constexpr bool apply_team_offset = true;
        const char* reason = "protect_hero";
        if (!IsNaviGoalAllowedByAreaScope(goal_base_id, my_team, my_team, enemy_team)) {
            naviGoalPublishAllowed_ = false;
            RecordDecisionIntent(MakeDecisionIntent(
                DecisionReason::AreaScopeBlocked,
                goal_base_id,
                my_team,
                apply_team_offset,
                reason));
            return false;
        }

        const auto resolved_goal_id = ResolveGoalId(goal_base_id, my_team, apply_team_offset);
        if (naviCommandGoal == resolved_goal_id && !naviCommandIntervalClock.trigger()) {
            speedLevel = 1;
            return true;
        }

        if (!TryStartNaviAreaTransition(goal_base_id, my_team, my_team, apply_team_offset, reason)) {
            SetPositionByBaseGoal(goal_base_id, my_team, apply_team_offset);
        }
        RecordDecisionIntent(MakeDecisionIntent(
            DecisionReason::ProtectHero,
            goal_base_id,
            my_team,
            apply_team_offset,
            reason));
        naviCommandIntervalClock.reset(Seconds{std::max(1, protection.HoldSec)});
        speedLevel = 1;
        if (LoggerPtr) {
            LoggerPtr->Info(
                "ProtectHero: elapsed={}s hero=({}, {}) in_highland={} in_protect_hero={} proactive_hold={} legacy_threat_ready={} goal={} hold={}s no_enemy_release={}s.",
                ElapsedSeconds(),
                hero_position.X,
                hero_position.Y,
                hero_in_highland ? 1 : 0,
                hero_in_protect_hero ? 1 : 0,
                protection.ProactiveHoldWhenHeroInHighland ? 1 : 0,
                legacy_threat_ready ? 1 : 0,
                static_cast<int>(naviCommandGoal),
                std::max(1, protection.HoldSec),
                std::max(1, protection.NoEnemyReleaseSec));
        }
        return true;
    }

    bool Application::TrySetSpecialPatrolGoal(
        const UnitTeam my_team,
        const UnitTeam enemy_team) {
        (void)enemy_team;
        const auto& patrol = config.SpecialSettings.Patrol;
        if (!patrol.Enable ||
            IsLeagueProfile() ||
            IsShowcasePatrolEnabled() ||
            GetStrategyMode() != StrategyMode::Regional ||
            my_team == UnitTeam::Unknown) {
            specialPatrolHoldActive_ = false;
            return false;
        }

        if (areaManager_.RegionalAreaTaskActive()) {
            if (areaManager_.RegionalAreaTask().Type == RegionalAreaTaskType::MyReadyRoadland &&
                !areaManager_.RegionalAreaTaskCanYieldToHigherPriority()) {
                return false;
            }
            const auto canceled_task_type = areaManager_.RegionalAreaTask().Type;
            areaManager_.ClearRegionalAreaTask();
            defaultStrategyManager_.RecordRegionalAreaResult(
                canceled_task_type,
                "preempted",
                std::chrono::steady_clock::now(),
                config.RegionalAreaTaskSettings.DefaultPolicy);
            ResetRegionalAreaControlOverride();
            gimbalControlData.FireCode.FollowMode = 0;
            if (LoggerPtr) {
                LoggerPtr->Info("RegionalAreaTask canceled: Special Patrol has higher priority.");
            }
        }

        constexpr bool apply_team_offset = true;
        auto is_patrol_goal = [&](const std::uint8_t base_goal) {
            return base_goal == LangYa::CentralLeftA.ID ||
                   base_goal == LangYa::CentralLeftB.ID;
        };
        auto other_goal = [](const std::uint8_t base_goal) {
            return base_goal == LangYa::CentralLeftA.ID
                ? LangYa::CentralLeftB.ID
                : LangYa::CentralLeftA.ID;
        };
        auto nearest_goal = [&]() {
            const auto now = std::chrono::steady_clock::now();
            const auto self_position = GetSentryPositionState(now);
            if (!self_position.Fresh) {
                return LangYa::CentralLeftA.ID;
            }
            if (self_position.X <= 0 || self_position.Y <= 0) {
                return LangYa::CentralLeftA.ID;
            }
            const auto point_a = AreaManager::GoalPointByBaseId(LangYa::CentralLeftA.ID, my_team);
            const auto point_b = AreaManager::GoalPointByBaseId(LangYa::CentralLeftB.ID, my_team);
            const double dist_a = AreaManager::DistanceSq(
                self_position.X,
                self_position.Y,
                static_cast<int>(point_a.x),
                static_cast<int>(point_a.y));
            const double dist_b = AreaManager::DistanceSq(
                self_position.X,
                self_position.Y,
                static_cast<int>(point_b.x),
                static_cast<int>(point_b.y));
            return dist_b < dist_a ? LangYa::CentralLeftB.ID : LangYa::CentralLeftA.ID;
        };

        const auto current_base_goal = BaseGoalIdFromResolvedGoal(naviCommandGoal);
        std::uint8_t target_base_goal =
            is_patrol_goal(current_base_goal) ? current_base_goal : nearest_goal();

        const AimData& active_aim_data = CurrentAimData();
        const bool target_locked =
            patrol.StopOnTarget &&
            targetArmor.Type != ArmorType::UnKnown &&
            isFindTargetAtomic.load(std::memory_order_relaxed) &&
            AimFreshAndValid(active_aim_data);
        if (target_locked) {
            const auto now = std::chrono::steady_clock::now();
            const auto self_position = GetSentryPositionState(now);
            if (self_position.Fresh && self_position.X > 0 && self_position.Y > 0) {
                naviCommandGoal = ResolveGoalId(target_base_goal, my_team, apply_team_offset);
                naviGoalPosition = Area::Point<std::uint16_t>{
                    static_cast<std::uint16_t>(std::clamp(self_position.X, 0, kOfficialFieldWidthCm)),
                    static_cast<std::uint16_t>(std::clamp(self_position.Y, 0, kOfficialFieldHeightCm))
                };
                naviGoalPublishAllowed_ = true;
                naviCommandIntervalClock.reset(Seconds{1});
                speedLevel = 1;
                UpdateNaviProgressWatchdogGoal(target_base_goal, my_team, apply_team_offset);
                specialPatrolHoldActive_ = false;
                if (LoggerPtr) {
                    LoggerPtr->Info(
                        "Special Patrol: target locked, hold current position=({}, {}) goal={}.",
                        self_position.X,
                        self_position.Y,
                        static_cast<int>(naviCommandGoal));
                }
                RecordDecisionIntent(MakeDecisionIntent(
                    DecisionReason::SpecialPatrol,
                    target_base_goal,
                    my_team,
                    apply_team_offset,
                    "special_patrol_target_hold"));
                return true;
            }
        }

        if (is_patrol_goal(current_base_goal) &&
            IsBaseGoalArrived(current_base_goal, my_team, apply_team_offset)) {
            const auto now = std::chrono::steady_clock::now();
            if (!specialPatrolHoldActive_ ||
                specialPatrolHoldBaseGoal_ != current_base_goal) {
                specialPatrolHoldActive_ = true;
                specialPatrolHoldBaseGoal_ = current_base_goal;
                specialPatrolHoldStartTime_ = now;
            }
            const int hold_sec = std::max(0, patrol.GoalHoldSec);
            if (hold_sec == 0 ||
                now - specialPatrolHoldStartTime_ >= std::chrono::seconds(hold_sec)) {
                target_base_goal = other_goal(current_base_goal);
                specialPatrolHoldActive_ = false;
            }
        } else if (!is_patrol_goal(current_base_goal)) {
            specialPatrolHoldActive_ = false;
        }

        const auto target_position = AreaManager::GoalPointByBaseId(target_base_goal, my_team);
        const auto resolved_goal_id = ResolveGoalId(target_base_goal, my_team, apply_team_offset);
        const bool position_mismatch =
            naviGoalPosition.x != target_position.x ||
            naviGoalPosition.y != target_position.y;
        const bool should_refresh_goal =
            naviCommandGoal != resolved_goal_id ||
            position_mismatch ||
            !naviGoalPublishAllowed_ ||
            naviCommandIntervalClock.trigger();
        if (should_refresh_goal) {
            SetPositionByBaseGoal(target_base_goal, my_team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{std::max(1, patrol.GoalHoldSec)});
            speedLevel = static_cast<std::uint8_t>(std::clamp(patrol.SpeedLevel, 0, 255));
            if (LoggerPtr) {
                LoggerPtr->Info(
                    "Special Patrol: goal={} point=({}, {}) hold={}s speed={}.",
                    static_cast<int>(naviCommandGoal),
                    static_cast<int>(target_position.x),
                    static_cast<int>(target_position.y),
                    std::max(0, patrol.GoalHoldSec),
                    static_cast<int>(speedLevel));
            }
        }

        RecordDecisionIntent(MakeDecisionIntent(
            DecisionReason::SpecialPatrol,
            target_base_goal,
            my_team,
            apply_team_offset,
            "special_patrol"));
        return true;
    }

    void Application::UpdateNaviProgressWatchdogGoal(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const bool apply_team_offset) {
        const std::uint8_t goal_id = ResolveGoalId(base_goal_id, goal_team, apply_team_offset);
        const auto now = std::chrono::steady_clock::now();
        const auto self_position = GetSentryPositionState(now);
        const bool has_self_position =
            self_position.Fresh && self_position.X > 0 && self_position.Y > 0;
        areaManager_.UpdateProgressWatchdogGoal(
            goal_id,
            base_goal_id,
            goal_team,
            apply_team_offset,
            naviGoalPosition,
            has_self_position ? self_position.X : 0,
            has_self_position ? self_position.Y : 0,
            now);
    }

    bool Application::TryAdvanceRegionalDefenseCommonCentralSearch(
        const NaviProgressWatchdogDecision& decision,
        const UnitTeam my_team,
        const std::chrono::steady_clock::time_point now) {
        if (!config.TacticalSettings.RegionalDefense.CommonCentral.Enable ||
            regionalDefenseSearchKind_ != RegionalDefenseSearchKind::CommonCentral ||
            !decision.NeedFallback ||
            decision.OriginalGoalTeam != my_team ||
            !decision.OriginalApplyTeamOffset ||
            decision.OriginalBaseGoal != regionalDefenseSearchBaseGoal_ ||
            decision.OriginalGoalId != ResolveGoalId(
                regionalDefenseSearchBaseGoal_, my_team, true)) {
            return false;
        }

        // CommonCentral owns a fixed four-point search. Keep its watchdog
        // transition inside the same tactical state machine instead of
        // replacing it with AreaManager's generic area fallback list.
        const auto current_index = regionalDefenseSearchIndex_ % kCommonCentralSearchGoals.size();
        if (kCommonCentralSearchGoals[current_index] != regionalDefenseSearchBaseGoal_) {
            return false;
        }

        const auto previous_goal = regionalDefenseSearchBaseGoal_;
        regionalDefenseSearchDirection_ = ReverseCommonCentralSearchDirection(
            current_index,
            regionalDefenseSearchDirection_);
        regionalDefenseSearchIndex_ = NextCommonCentralSearchIndex(
            current_index,
            regionalDefenseSearchDirection_);
        regionalDefenseSearchBaseGoal_ = kCommonCentralSearchGoals[regionalDefenseSearchIndex_];
        regionalDefenseSearchStartTime_ = now;
        regionalDefenseSearchArrivedTime_ = {};
        if (LoggerPtr) {
            LoggerPtr->Warning(
                "Regional defense switch: {} -> {} trigger={} search_kind=common_central.",
                static_cast<int>(previous_goal),
                static_cast<int>(regionalDefenseSearchBaseGoal_),
                decision.GoalUnreachable ? "external_unreachable" : "watchdog_no_progress");
        }
        return true;
    }

    bool Application::TickNaviProgressWatchdog(
        const UnitTeam my_team,
        const UnitTeam enemy_team) {
        const auto& watchdog = config.NaviProgressWatchdogSettings;
        const auto now = std::chrono::steady_clock::now();
        const auto& runtime = areaManager_.ProgressWatchdogRuntime();
        if (outpostVisualScoutNavigationActive_) {
            const auto buff_outpost_goal_id =
                ResolveGoalId(LangYa::BuffOutpost.ID, my_team, true);
            const auto buff_outpost_goal_position =
                AreaManager::GoalPointByBaseId(LangYa::BuffOutpost.ID, my_team);
            const bool watchdog_matches_outpost_goal =
                runtime.Active &&
                runtime.GoalId == buff_outpost_goal_id &&
                runtime.GoalPosition.x == buff_outpost_goal_position.x &&
                runtime.GoalPosition.y == buff_outpost_goal_position.y;
            if (!watchdog_matches_outpost_goal) {
                return false;
            }
        }
        const auto self_position = GetSentryPositionState(now);
        const bool has_self_position =
            self_position.Fresh && self_position.X > 0 && self_position.Y > 0;
        const auto current_goal_reach = EvaluateNaviGoalReach(
            runtime.GoalId,
            runtime.GoalPosition,
            std::max(1, config.DecisionAutonomySettings.NaviGoal.HighlandCompatArriveDistanceCm),
            0,
            runtime.BaseGoal);

        const auto decision = areaManager_.TickProgressWatchdog(
            NaviProgressWatchdogInput{
                .Enabled = watchdog.Enable,
                .BlockedByAreaTransition = areaManager_.HighlandTransitionActive(),
                .HasSelfPosition = has_self_position,
                .SelfX = has_self_position ? self_position.X : 0,
                .SelfY = has_self_position ? self_position.Y : 0,
                .IsCurrentGoalArrived = current_goal_reach.Status == GoalReachStatus::Reached,
                .IsCurrentGoalUnreachable = current_goal_reach.Status == GoalReachStatus::Unreachable,
                .Setting = watchdog,
                .Now = now
            });
        if (!decision.NeedFallback) {
            return false;
        }

        if (TryAdvanceRegionalDefenseCommonCentralSearch(decision, my_team, now)) {
            // Let Tactical materialize the next CommonCentral goal in this
            // tick. The generic watchdog must not consume this transition.
            return false;
        }

        for (const auto fallback_goal : decision.FallbackCandidates) {
            if (TrySetScopedPositionByBaseGoal(
                    fallback_goal,
                    decision.OriginalGoalTeam,
                    my_team,
                    enemy_team,
                    decision.OriginalApplyTeamOffset,
                    "navi_progress_watchdog")) {
                naviCommandIntervalClock.reset(Seconds{std::max(1, watchdog.FallbackHoldSec)});
                speedLevel = 1;
                areaManager_.CommitProgressWatchdogFallback(
                    now,
                    watchdog.FallbackCooldownSec,
                    decision.OriginalBaseGoal,
                    decision.OriginalGoalTeam);
                if (LoggerPtr) {
                    LoggerPtr->Warning(
                        "Navi progress watchdog: goal={} {}, fallback goal={}.",
                        static_cast<int>(decision.OriginalGoalId),
                        decision.GoalUnreachable ? "unreachable" : "no movement",
                        static_cast<int>(naviCommandGoal));
                }
                return true;
            }
        }

        areaManager_.MarkProgressWatchdogFallbackFailed(
            now,
            has_self_position,
            has_self_position ? self_position.X : 0,
            has_self_position ? self_position.Y : 0);
        return false;
    }

    bool Application::TrySetRegionalIdlePatrolGoal(
        const UnitTeam my_team,
        const UnitTeam enemy_team) {
        const auto& patrol = config.RegionalIdlePatrolSettings;
        if (!patrol.Enable || IsLeagueProfile() || IsShowcasePatrolEnabled()) {
            return false;
        }
        if (aimMode == AimMode::Buff || aimMode == AimMode::Outpost ||
            outpostVisualScoutNavigationActive_) {
            return false;
        }
        if (patrol.Goals.empty()) {
            naviCommandIntervalClock.reset(Seconds{std::max(1, patrol.GoalHoldSec)});
            if (LoggerPtr) {
                LoggerPtr->Warning("Regional idle patrol enabled but no goals are enabled.");
            }
            return true;
        }

        constexpr bool apply_team_offset = true;
        const int hold_sec = std::max(1, patrol.GoalHoldSec);
        const auto candidates = areaManager_.BuildRegionalIdlePatrolCandidates(
            patrol.Goals,
            naviCommandGoal,
            my_team);
        for (const auto& candidate : candidates) {
            const auto base_goal_id = candidate.BaseGoalId;
            if (!AreaManager::IsValidBaseGoalId(base_goal_id)) {
                continue;
            }
            const auto previous_goal_id = naviCommandGoal;
            const auto previous_goal_position = naviGoalPosition;
            const auto previous_intent = lastDecisionIntent_;
            if (TrySetScopedPositionByBaseGoal(
                    base_goal_id,
                    my_team,
                    my_team,
                    enemy_team,
                    apply_team_offset,
                    "regional_idle_patrol")) {
                if (naviCommandGoal != previous_goal_id ||
                    naviGoalPosition.x != previous_goal_position.x ||
                    naviGoalPosition.y != previous_goal_position.y) {
                    lastDecisionIntent_.Detail =
                        "index=" + std::to_string(candidate.Index) +
                        " hold_sec=" + std::to_string(hold_sec);
                } else {
                    lastDecisionIntent_ = previous_intent;
                }
                areaManager_.CommitRegionalIdlePatrolCandidate(candidate.Index);
                naviCommandIntervalClock.reset(Seconds{hold_sec});
                speedLevel = 1;
                if (LoggerPtr) {
                    LoggerPtr->Info(
                        "Regional idle patrol: base_goal={} goal={} hold={}s.",
                        static_cast<int>(base_goal_id),
                        static_cast<int>(naviCommandGoal),
                        hold_sec);
                }
                return true;
            }
        }

        areaManager_.ResetRegionalIdlePatrol();
        naviCommandIntervalClock.reset(Seconds{hold_sec});
        if (LoggerPtr) {
            LoggerPtr->Warning("Regional idle patrol: all configured goals blocked by area scope.");
        }
        return true;
    }

    bool Application::IsDefaultRegionalDecisionReady(
        const UnitTeam my_team,
        const UnitTeam enemy_team) const {
        return !IsLeagueProfile() &&
            !IsShowcasePatrolEnabled() &&
            GetStrategyMode() == StrategyMode::Regional &&
            aimMode != AimMode::Buff &&
            aimMode != AimMode::Outpost &&
            !outpostVisualScoutNavigationActive_ &&
            !areaManager_.RegionalAreaTaskActive() &&
            !areaManager_.HighlandTransitionActive() &&
            !EvaluateRegionalDefenseThreat(my_team, enemy_team).has_value();
    }

    bool Application::TrySetDefaultRegionalAreaTaskGoal(
        const UnitTeam my_team,
        const UnitTeam enemy_team) {
        const auto& task = config.RegionalAreaTaskSettings;
        if (!task.Enable ||
            IsLeagueProfile() ||
            IsShowcasePatrolEnabled() ||
            aimMode == AimMode::Buff ||
            aimMode == AimMode::Outpost ||
            outpostVisualScoutNavigationActive_) {
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        auto referee_value_fresh = [&](const bool received, const std::chrono::steady_clock::time_point last_rx) {
            return received &&
                last_rx.time_since_epoch().count() != 0 &&
                now - last_rx <= std::chrono::seconds(2);
        };
        const auto self_position = GetSentryPositionState(now);
        const bool has_self_position =
            self_position.Fresh && self_position.X > 0 && self_position.Y > 0;
        const auto candidates =
            defaultStrategyManager_.BuildRegionalAreaCandidates(
                DefaultRegionalPolicyInput{
                    .Config = &config,
                    .MyTeam = my_team,
                    .EnemyTeam = enemy_team,
                    .HealthFresh = referee_value_fresh(hasReceivedMyselfHealth_, lastMyselfHealthRxTime),
                    .AmmoFresh = referee_value_fresh(hasReceivedAmmoLeft_, lastAmmoLeftRxTime),
                    .Health = myselfHealth,
                    .Ammo = ammoLeft,
                    .HasSelfPosition = has_self_position,
                    .SelfX = has_self_position ? self_position.X : 0,
                    .SelfY = has_self_position ? self_position.Y : 0,
                    .SelfArea = areaManager_.SelfAreaRuntime(),
                    .Now = now
                });
        if (candidates.empty()) {
            defaultStrategyManager_.ResetRegionalPolicy();
            return false;
        }

        for (const auto& candidate : candidates) {
            const auto previous_goal_id = naviCommandGoal;
            const auto previous_goal_position = naviGoalPosition;
            const auto previous_intent = lastDecisionIntent_;
            if (!TrySetScopedPositionByBaseGoal(
                    candidate.BaseGoalId,
                    candidate.GoalTeam,
                    my_team,
                    enemy_team,
                    true,
                    "default_area_policy",
                    RegionalAreaTaskOrigin::DefaultPolicy)) {
                continue;
            }

            if (naviCommandGoal != previous_goal_id ||
                naviGoalPosition.x != previous_goal_position.x ||
                naviGoalPosition.y != previous_goal_position.y) {
                lastDecisionIntent_.Detail = "area=" + std::string{candidate.Name};
            } else {
                lastDecisionIntent_ = previous_intent;
            }
            defaultStrategyManager_.CommitRegionalAreaSelection(candidate, now);
            naviCommandIntervalClock.reset(Seconds{1});
            speedLevel = 1;
            if (LoggerPtr) {
                LoggerPtr->Info(
                    "Default regional policy: area={} score={:.2f} base_goal={} goal={}.",
                    candidate.Name,
                    candidate.Score,
                    static_cast<int>(candidate.BaseGoalId),
                    static_cast<int>(naviCommandGoal));
            }
            return true;
        }

        return false;
    }

    bool Application::TrySetDefaultRegionalGoal(
        const UnitTeam my_team,
        const UnitTeam enemy_team) {
        if (TrySetDefaultRegionalAreaTaskGoal(my_team, enemy_team)) {
            speedLevel = 1;
            return true;
        }
        return false;
    }

    bool Application::TrySetScopedPositionByBaseGoal(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const UnitTeam my_team,
        const UnitTeam enemy_team,
        const bool apply_team_offset,
        const char* reason,
        const RegionalAreaTaskOrigin origin) {
        const auto intent_reason = reason != nullptr
            ? DecisionReasonFromString(reason)
            : DecisionReason::Unknown;
        if (!IsNaviGoalAllowedByAreaScope(base_goal_id, goal_team, my_team, enemy_team)) {
            naviGoalPublishAllowed_ = false;
            RecordDecisionIntent(MakeDecisionIntent(
                DecisionReason::AreaScopeBlocked,
                base_goal_id,
                goal_team,
                apply_team_offset,
                reason));
            if (LoggerPtr) {
                LoggerPtr->Info(
                    "DecisionAutonomy[navi_goal_area]: block goal={} team={} reason={}",
                    static_cast<int>(ResolveGoalId(base_goal_id, goal_team, apply_team_offset)),
                    goal_team == my_team ? "my" : "enemy",
                    reason ? reason : "area_scope");
            }
            return false;
        }

        if (TryStartRegionalAreaTaskForGoal(
                base_goal_id, goal_team, my_team, apply_team_offset, reason, origin)) {
            RecordDecisionIntent(MakeDecisionIntent(
                intent_reason,
                base_goal_id,
                goal_team,
                apply_team_offset,
                reason));
            return true;
        }

        if (TryStartNaviAreaTransition(base_goal_id, goal_team, my_team, apply_team_offset, reason)) {
            RecordDecisionIntent(MakeDecisionIntent(
                intent_reason,
                base_goal_id,
                goal_team,
                apply_team_offset,
                reason));
            return true;
        }

        SetPositionByBaseGoal(base_goal_id, goal_team, apply_team_offset);
        RecordDecisionIntent(MakeDecisionIntent(
            intent_reason,
            base_goal_id,
            goal_team,
            apply_team_offset,
            reason));
        return true;
    }

    bool Application::TrySetRandomScopedPositionByBaseGoal(
        const std::vector<std::pair<std::uint8_t, UnitTeam>>& goals,
        const UnitTeam my_team,
        const UnitTeam enemy_team,
        const char* reason) {
        std::vector<std::pair<std::uint8_t, UnitTeam>> allowed_goals;
        allowed_goals.reserve(goals.size());
        for (const auto& goal : goals) {
            if (!AreaManager::IsValidBaseGoalId(goal.first)) {
                continue;
            }
            if (IsNaviGoalAllowedByAreaScope(goal.first, goal.second, my_team, enemy_team)) {
                allowed_goals.push_back(goal);
            }
        }
        if (allowed_goals.empty()) {
            if (IsNaviGoalAreaScopeEnabled()) {
                naviGoalPublishAllowed_ = false;
                if (LoggerPtr) {
                    LoggerPtr->Info(
                        "DecisionAutonomy[navi_goal_area]: block all random goals reason={}",
                        reason ? reason : "area_scope");
                }
            }
            return false;
        }

        Random random;
        const auto index = random.Get(0, static_cast<int>(allowed_goals.size()) - 1);
        const auto& selected = allowed_goals[static_cast<std::size_t>(index)];
        return TrySetScopedPositionByBaseGoal(
            selected.first,
            selected.second,
            my_team,
            enemy_team,
            true,
            reason);
    }

    void Application::SetPositionByBaseGoal(
        const std::uint8_t base_goal_id,
        const UnitTeam goal_team,
        const bool apply_team_offset) {
        auto assign_position = [&](const auto& goal_location, const auto& area_location) {
            naviCommandGoal = apply_team_offset ? goal_location(goal_team) : goal_location.ID;
            naviGoalPosition = area_location(goal_team);
            naviGoalPublishAllowed_ = true;
        };
        auto assign_point = [&](const auto& goal_location, const Area::Point<std::uint16_t>& point) {
            naviCommandGoal = apply_team_offset ? goal_location(goal_team) : goal_location.ID;
            naviGoalPosition = point;
            naviGoalPublishAllowed_ = true;
        };
        std::uint8_t effective_base_goal_id = base_goal_id;

        switch (base_goal_id) {
            case LangYa::Home.ID: assign_position(LangYa::Home, BehaviorTree::Area::Home); break;
            case LangYa::Base.ID: assign_position(LangYa::Base, BehaviorTree::Area::Base); break;
            case LangYa::Recovery.ID: assign_position(LangYa::Recovery, BehaviorTree::Area::Recovery); break;
            case LangYa::BuffShoot.ID: assign_position(LangYa::BuffShoot, BehaviorTree::Area::BuffShoot); break;
            case LangYa::LeftHighLand.ID: assign_position(LangYa::LeftHighLand, BehaviorTree::Area::LeftHighLand); break;
            case LangYa::CastleLeft1.ID: assign_position(LangYa::CastleLeft1, BehaviorTree::Area::CastleLeft1); break;
            case LangYa::CastleLeft2.ID: assign_position(LangYa::CastleLeft2, BehaviorTree::Area::CastleLeft2); break;
            case LangYa::Castle.ID: assign_position(LangYa::Castle, BehaviorTree::Area::Castle); break;
            case LangYa::CastleRight1.ID: assign_position(LangYa::CastleRight1, BehaviorTree::Area::CastleRight1); break;
            case LangYa::CastleRight2.ID: assign_position(LangYa::CastleRight2, BehaviorTree::Area::CastleRight2); break;
            case LangYa::FlyRoad.ID: assign_position(LangYa::FlyRoad, BehaviorTree::Area::FlyRoad); break;
            case LangYa::OutpostArea.ID: assign_position(LangYa::OutpostArea, BehaviorTree::Area::OutpostArea); break;
            case LangYa::MidShoot.ID: assign_position(LangYa::MidShoot, BehaviorTree::Area::MidShoot); break;
            case LangYa::LeftShoot.ID: assign_position(LangYa::LeftShoot, BehaviorTree::Area::LeftShoot); break;
            case LangYa::OutpostShoot.ID: assign_position(LangYa::OutpostShoot, BehaviorTree::Area::OutpostShoot); break;
            case LangYa::BuffAround1.ID: assign_position(LangYa::BuffAround1, BehaviorTree::Area::BuffAround1); break;
            case LangYa::BuffAround2.ID: assign_position(LangYa::BuffAround2, BehaviorTree::Area::BuffAround2); break;
            case LangYa::RightShoot.ID: assign_position(LangYa::RightShoot, BehaviorTree::Area::RightShoot); break;
            case LangYa::HoleRoad.ID: assign_position(LangYa::HoleRoad, BehaviorTree::Area::HoleRoad); break;
            case LangYa::OccupyArea.ID: assign_position(LangYa::OccupyArea, BehaviorTree::Area::OccupyArea); break;
            case LangYa::Highland.ID: assign_position(LangYa::Highland, BehaviorTree::Area::Highland); break;
            case LangYa::BaseToCentral.ID: assign_position(LangYa::BaseToCentral, BehaviorTree::Area::BaseToCentral); break;
            case LangYa::CentralToBase.ID: assign_position(LangYa::CentralToBase, BehaviorTree::Area::CentralToBase); break;
            case LangYa::BuffOutpost.ID: assign_position(LangYa::BuffOutpost, BehaviorTree::Area::BuffOutpost); break;
            case LangYa::OutpostGuard.ID: assign_position(LangYa::OutpostGuard, BehaviorTree::Area::OutpostGuard); break;
            case LangYa::PreRoadland.ID: assign_position(LangYa::PreRoadland, BehaviorTree::Area::PreRoadland); break;
            case LangYa::CentralLeftA.ID: assign_point(LangYa::CentralLeftA, BehaviorTree::Area::CentralLeft.A(goal_team)); break;
            case LangYa::CentralLeftB.ID: assign_point(LangYa::CentralLeftB, BehaviorTree::Area::CentralLeft.B(goal_team)); break;
            case LangYa::ProtectOutpost.ID: assign_position(LangYa::ProtectOutpost, BehaviorTree::Area::ProtectOutpost); break;
            case LangYa::CentralHigh.ID: assign_position(LangYa::CentralHigh, BehaviorTree::Area::CentralHigh); break;
            case LangYa::CentralLow.ID: assign_position(LangYa::CentralLow, BehaviorTree::Area::CentralLow); break;
            default:
                LoggerPtr->Warning("Unknown base goal id={}, fallback to Home.", static_cast<int>(base_goal_id));
                effective_base_goal_id = LangYa::Home.ID;
                assign_position(LangYa::Home, BehaviorTree::Area::Home);
                break;
        }
        UpdateNaviProgressWatchdogGoal(effective_base_goal_id, goal_team, apply_team_offset);
    }

    void Application::SetPositionRepeat() {
        if (IsLeagueProfile()) {
            SetPositionLeagueSimple();
            return;
        }
        if (IsShowcasePatrolEnabled()) {
            SetPositionShowcasePatrol();
            return;
        }
        const UnitTeam my_team = team;
        const UnitTeam enemy_team = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        (void)TrySetDefaultRegionalGoal(my_team, enemy_team);
    }

    League3v3Input Application::BuildLeague3v3Input(
        const std::chrono::steady_clock::time_point now) const {
        const auto& setting = config.LeagueStrategySettings.League3v3;
        const int position_fresh_ms = std::max(1, setting.PositionFreshTimeoutMs);
        const int health_fresh_ms = std::max(1, setting.HealthFreshTimeoutMs);

        const auto age_within = [&](const std::chrono::steady_clock::time_point& last_rx,
                                    const int timeout_ms) {
            return last_rx.time_since_epoch().count() != 0 &&
                now - last_rx <= std::chrono::milliseconds(timeout_ms);
        };
        const auto external_visible = [&](const ArmorType type) {
            const auto index = static_cast<std::size_t>(type);
            if (index >= externalAimTargets_.size()) {
                return false;
            }
            const auto& cache = externalAimTargets_[index];
            return cache.Valid &&
                cache.LastSeen.time_since_epoch().count() != 0 &&
                now - cache.LastSeen <= std::chrono::milliseconds(health_fresh_ms);
        };
        const auto friend_snapshot = [&](const UnitType type) {
            const auto index = static_cast<std::size_t>(type);
            League3v3UnitSnapshot snapshot;
            snapshot.HpFresh = IsFriendHealthFresh(type, health_fresh_ms);
            snapshot.Hp = friendRobots[type].currentHealth_;
            snapshot.PositionFresh = IsFriendPositionFresh(type, position_fresh_ms);
            snapshot.Visible = false;
            return snapshot;
        };
        const auto enemy_snapshot = [&](const UnitType type, const ArmorType armor) {
            const auto index = static_cast<std::size_t>(type);
            League3v3UnitSnapshot snapshot;
            snapshot.HpFresh =
                index < lastEnemyHealthRxTime_.size() &&
                age_within(lastEnemyHealthRxTime_[index], health_fresh_ms);
            snapshot.Hp = enemyHealthConfirmedDead_[index]
                ? 0U
                : static_cast<std::uint16_t>(enemyRobots[type].currentHealth_);
            snapshot.PositionFresh = IsEnemyPositionFresh(type, position_fresh_ms);
            snapshot.Visible = external_visible(armor);
            return snapshot;
        };

        League3v3Input input;
        input.GameRunning = true;
        input.SelfHpFresh =
            hasReceivedMyselfHealth_ &&
            age_within(lastMyselfHealthRxTime, health_fresh_ms);
        input.SelfHp = static_cast<std::uint16_t>(std::max(0, static_cast<int>(myselfHealth)));
        input.EventDataFresh =
            hasReceivedEventData_ &&
            age_within(lastEventDataRxTime_, health_fresh_ms);
        input.CenterStatus = static_cast<League3v3CenterStatus>(
            std::clamp(static_cast<int>(eventCenterGainPointStatus_), 0, 3));
        input.Friend.Hero = friend_snapshot(UnitType::Hero);
        input.Friend.Sentry = friend_snapshot(UnitType::Sentry);
        input.Friend.Infantry = friend_snapshot(UnitType::Infantry1);
        input.Enemy.Hero = enemy_snapshot(UnitType::Hero, ArmorType::Hero);
        input.Enemy.Sentry = enemy_snapshot(UnitType::Sentry, ArmorType::Sentry);
        input.Enemy.Infantry = enemy_snapshot(UnitType::Infantry1, ArmorType::Infantry1);
        input.NowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();

        const std::array<std::pair<UnitType, bool>, 3> enemy_units{{
            {UnitType::Hero, input.Enemy.Hero.PositionFresh},
            {UnitType::Sentry, input.Enemy.Sentry.PositionFresh},
            {UnitType::Infantry1, input.Enemy.Infantry.PositionFresh}}};
        for (const auto& [unit, fresh] : enemy_units) {
            if (!fresh) {
                continue;
            }
            if (IsLeague3v3ControlAreaPoint(
                    static_cast<int>(enemyRobots[unit].position_.X),
                    static_cast<int>(enemyRobots[unit].position_.Y))) {
                input.EnemyNearCenter = true;
                break;
            }
        }
        return input;
    }
    void Application::SetPositionLeagueSimple() {
        const auto& league = config.LeagueStrategySettings;
        const int hold_sec = std::max(1, league.GoalHoldSec);

        if (league.League3v3.Enable) {
            if (!league3v3PolicyConfigured_) {
                league3v3Policy_ = League3v3Policy(league.League3v3);
                league3v3PolicyConfigured_ = true;
            }
            const auto now = std::chrono::steady_clock::now();
            league3v3Decision_ = league3v3Policy_.Decide(BuildLeague3v3Input(now));
            const auto league3v3_base_goal = league3v3Decision_.Goal == League3v3Goal::None
                ? static_cast<std::uint8_t>(League3v3Goal::SelfBase)
                : static_cast<std::uint8_t>(league3v3Decision_.Goal);
            const std::string league3v3_detail =
                std::string("state=") + League3v3StateName(league3v3Decision_.State) +
                " reason=" + league3v3Decision_.Reason +
                " target=" + League3v3TargetName(league3v3Decision_.Target);
            RecordDecisionIntent(MakeDecisionIntent(
                DecisionReason::League3v3,
                league3v3_base_goal,
                team,
                true,
                league3v3_detail.c_str()));
            if (league3v3Decision_.Goal == League3v3Goal::None) {
                naviGoalPublishAllowed_ = false;
                speedLevel = 1;
                return;
            }
            naviCommandGoal = ResolveGoalId(
                static_cast<std::uint8_t>(league3v3Decision_.Goal),
                team,
                true);
            const auto league_goal = League3v3GoalPoint(league3v3Decision_.Goal, team);
            naviGoalPosition = Area::Point<std::uint16_t>{league_goal.X, league_goal.Y};
            naviGoalPublishAllowed_ = true;
            speedLevel = 1;
            return;
        }
        constexpr bool apply_team_offset = true;
        if (IsLeagueRouteCompatEnabled() && leagueRouteCompatAfterGatePending_) {
            leagueRouteCompatAfterGatePending_ = false;
            leagueRouteCompatActive_ = true;
            leagueRouteCompatUntil_ =
                std::chrono::steady_clock::now() + std::chrono::seconds(kLeagueRouteCompatViaHoldSec);
            leagueRouteCompatHasPendingGoal_ = false;
            leagueRouteCompatPendingBaseGoal_ = LangYa::Home.ID;
            leagueRouteCompatPendingHoldSec_ = 1;
            SetPositionByBaseGoal(kLeagueRouteCompatViaGoalBaseId, team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{1});
            speedLevel = 1;
            LoggerPtr->Info(
                "League route compat after_gate: via goal={} for {}s.",
                static_cast<int>(naviCommandGoal),
                kLeagueRouteCompatViaHoldSec);
            return;
        }

        if (TickLeagueRouteCompat(team, apply_team_offset)) {
            return;
        }

        // 联赛模式优先做回补判定；命中后直接返回，不再切换巡航点。
        if (CheckPositionRecovery()) {
            LoggerPtr->Info("League profile recovery: health={} ammo={}", myselfHealth, ammoLeft);
            leaguePatrolGoalIndex_ = 0;
            leaguePatrolGoalInitialized_ = false;
            return;
        }

        std::vector<std::uint8_t> plan;
        plan.reserve(1 + league.PatrolGoals.size());
        // 计划路径由 MainGoal + PatrolGoals 去重组成。
        // 这里保证非法点位不会进入运行态。
        auto append_goal = [&](const std::uint8_t goal_id) {
            if (!AreaManager::IsValidBaseGoalId(goal_id)) {
                LoggerPtr->Warning("Skip invalid league goal id={}.", static_cast<int>(goal_id));
                return;
            }
            if (std::find(plan.begin(), plan.end(), goal_id) == plan.end()) {
                plan.push_back(goal_id);
            }
        };
        append_goal(league.MainGoal);
        for (const auto goal_id : league.PatrolGoals) {
            append_goal(goal_id);
        }
        if (plan.empty()) {
            append_goal(LangYa::OccupyArea.ID);
        }

        if (!leaguePatrolGoalInitialized_) {
            leaguePatrolGoalIndex_ = 0;
            const auto init_goal_id = ResolveGoalId(plan[leaguePatrolGoalIndex_], team, apply_team_offset);
            if (IsLeagueRouteCompatEnabled()) {
                if (IsLeagueGoalSwitchBetween2And3(
                        naviCommandGoal,
                        init_goal_id,
                        team,
                        apply_team_offset)) {
                    leaguePatrolGoalInitialized_ = true;
                    StartLeagueRouteCompat(
                        plan[leaguePatrolGoalIndex_],
                        hold_sec,
                        team,
                        apply_team_offset,
                        "init_2_3_switch");
                    return;
                }
            }
            SetPositionByBaseGoal(plan[leaguePatrolGoalIndex_], team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{hold_sec});
            speedLevel = 1;
            leaguePatrolGoalInitialized_ = true;
            LoggerPtr->Info("League profile init goal={}", static_cast<int>(naviCommandGoal));
            return;
        }

        const auto current_it = std::find_if(plan.begin(), plan.end(),
            [this](const std::uint8_t goal_id) {
                return naviCommandGoal == ResolveGoalId(goal_id, team);
            });
        if (current_it == plan.end()) {
            leaguePatrolGoalIndex_ = 0;
            const auto reset_goal_id = ResolveGoalId(plan[leaguePatrolGoalIndex_], team, apply_team_offset);
            if (IsLeagueRouteCompatEnabled() &&
                IsLeagueGoalSwitchBetween2And3(
                    naviCommandGoal,
                    reset_goal_id,
                    team,
                    apply_team_offset)) {
                StartLeagueRouteCompat(
                    plan[leaguePatrolGoalIndex_],
                    hold_sec,
                    team,
                    apply_team_offset,
                    "reset_2_3_switch");
                return;
            }
            SetPositionByBaseGoal(plan[leaguePatrolGoalIndex_], team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{hold_sec});
            speedLevel = 1;
            LoggerPtr->Info("League profile reset goal={}", static_cast<int>(naviCommandGoal));
            return;
        }
        leaguePatrolGoalIndex_ = static_cast<std::size_t>(std::distance(plan.begin(), current_it));

        if (plan.size() == 1U) {
            speedLevel = 1;
            return;
        }

        if (!naviCommandIntervalClock.trigger()) {
            speedLevel = 1;
            return;
        }

        if (plan.size() > 1U) {
            leaguePatrolGoalIndex_ = (leaguePatrolGoalIndex_ + 1U) % plan.size();
        }
        const auto target_base_goal = plan[leaguePatrolGoalIndex_];
        const auto target_goal_id = ResolveGoalId(target_base_goal, team, apply_team_offset);
        if (IsLeagueRouteCompatEnabled() &&
            IsLeagueGoalSwitchBetween2And3(
                naviCommandGoal,
                target_goal_id,
                team,
                apply_team_offset)) {
            StartLeagueRouteCompat(
                target_base_goal,
                hold_sec,
                team,
                apply_team_offset,
                "switch_2_3");
            return;
        }
        SetPositionByBaseGoal(target_base_goal, team, apply_team_offset);
        naviCommandIntervalClock.reset(Seconds{hold_sec});
        speedLevel = 1;
        LoggerPtr->Info("League profile switch goal={}", static_cast<int>(naviCommandGoal));
    }

    void Application::SetPositionShowcasePatrol() {
        const auto& showcase = config.ShowcasePatrolSettings;
        const bool apply_team_offset = !showcase.DisableTeamOffset;
        const int hold_sec = std::max(1, showcase.GoalHoldSec);
        const auto& plan = showcase.Goals;

        auto choose_index = [&](const bool initialize) -> std::size_t {
            if (plan.empty()) {
                return 0U;
            }
            if (!showcase.Random || plan.size() == 1U) {
                return initialize ? 0U : (showcasePatrolGoalIndex_ + 1U) % plan.size();
            }
            Random random;
            const auto upper_bound = static_cast<int>(plan.size()) - 1;
            std::size_t next_index = initialize
                ? static_cast<std::size_t>(random.Get(0, upper_bound))
                : showcasePatrolGoalIndex_;
            while (!initialize && next_index == showcasePatrolGoalIndex_) {
                next_index = static_cast<std::size_t>(random.Get(0, upper_bound));
            }
            return next_index;
        };

        auto apply_goal = [&](const std::size_t goal_index, const char* reason) {
            showcasePatrolGoalIndex_ = goal_index;
            SetPositionByBaseGoal(plan[goal_index], team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{hold_sec});
            speedLevel = 1;
            LoggerPtr->Info(
                "Showcase patrol {} goal_id={} raw_base_goal={}",
                reason,
                static_cast<int>(naviCommandGoal),
                static_cast<int>(plan[goal_index]));
        };

        if (!showcase.IgnoreRecovery && CheckPositionRecovery()) {
            LoggerPtr->Info("Showcase patrol recovery: health={} ammo={}", myselfHealth, ammoLeft);
            showcasePatrolGoalIndex_ = 0;
            showcasePatrolGoalInitialized_ = false;
            return;
        }

        if (plan.empty()) {
            SetPositionByBaseGoal(LangYa::OccupyArea.ID, team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{hold_sec});
            speedLevel = 1;
            LoggerPtr->Warning("Showcase patrol has empty goal plan, fallback to OccupyArea.");
            return;
        }

        if (!showcasePatrolGoalInitialized_) {
            apply_goal(choose_index(true), "init");
            showcasePatrolGoalInitialized_ = true;
            return;
        }

        const auto current_it = std::find_if(plan.begin(), plan.end(),
            [&](const std::uint8_t goal_id) {
                return naviCommandGoal == ResolveGoalId(goal_id, team, apply_team_offset);
            });
        if (current_it == plan.end()) {
            apply_goal(choose_index(true), "reset");
            showcasePatrolGoalInitialized_ = true;
            return;
        }
        showcasePatrolGoalIndex_ = static_cast<std::size_t>(std::distance(plan.begin(), current_it));

        if (plan.size() == 1U) {
            speedLevel = 1;
            return;
        }

        if (!naviCommandIntervalClock.trigger()) {
            speedLevel = 1;
            return;
        }

        apply_goal(choose_index(false), "switch");
    }

    void Application::SetPositionNaviDebugPlan() {
        const auto& navi_debug = config.NaviDebugSettings;
        const bool apply_team_offset = !navi_debug.DisableTeamOffset;
        const int hold_sec = std::max(1, navi_debug.GoalHoldSec);
        const auto& plan = navi_debug.Goals;

        auto choose_index = [&](const bool initialize) -> std::size_t {
            if (plan.empty()) {
                return 0U;
            }
            if (!navi_debug.Random || plan.size() == 1U) {
                return initialize ? 0U : (naviDebugGoalIndex_ + 1U) % plan.size();
            }
            Random random;
            const auto upper_bound = static_cast<int>(plan.size()) - 1;
            std::size_t next_index = initialize
                ? static_cast<std::size_t>(random.Get(0, upper_bound))
                : naviDebugGoalIndex_;
            while (!initialize && next_index == naviDebugGoalIndex_) {
                next_index = static_cast<std::size_t>(random.Get(0, upper_bound));
            }
            return next_index;
        };

        auto apply_goal = [&](const std::size_t goal_index, const char* reason) {
            naviDebugGoalIndex_ = goal_index;
            SetPositionByBaseGoal(plan[goal_index], team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{hold_sec});
            speedLevel = navi_debug.SpeedLevel;
            LoggerPtr->Info(
                "NaviDebug {} goal_id={} raw_base_goal={} speed_level={}",
                reason,
                static_cast<int>(naviCommandGoal),
                static_cast<int>(plan[goal_index]),
                static_cast<int>(speedLevel));
        };

        if (!navi_debug.IgnoreRecovery && CheckPositionRecovery()) {
            LoggerPtr->Info("NaviDebug recovery: health={} ammo={}", myselfHealth, ammoLeft);
            naviDebugGoalIndex_ = 0;
            naviDebugGoalInitialized_ = false;
            return;
        }

        if (plan.empty()) {
            SetPositionByBaseGoal(LangYa::OccupyArea.ID, team, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{hold_sec});
            speedLevel = navi_debug.SpeedLevel;
            LoggerPtr->Warning("NaviDebug has empty goal plan, fallback to OccupyArea.");
            return;
        }

        if (!naviDebugGoalInitialized_) {
            apply_goal(choose_index(true), "init");
            naviDebugGoalInitialized_ = true;
            return;
        }

        const auto current_it = std::find_if(plan.begin(), plan.end(),
            [&](const std::uint8_t goal_id) {
                return naviCommandGoal == ResolveGoalId(goal_id, team, apply_team_offset);
            });
        if (current_it == plan.end()) {
            apply_goal(choose_index(true), "reset");
            naviDebugGoalInitialized_ = true;
            return;
        }
        naviDebugGoalIndex_ = static_cast<std::size_t>(std::distance(plan.begin(), current_it));

        if (plan.size() == 1U) {
            speedLevel = navi_debug.SpeedLevel;
            return;
        }

        if (!naviCommandIntervalClock.trigger()) {
            speedLevel = navi_debug.SpeedLevel;
            return;
        }

        apply_goal(choose_index(false), "switch");
    }

    bool Application::CheckPositionRecovery() {
        if (config.LeagueStrategySettings.League3v3.Enable) {
            // League 3v3 recovery is owned by League3v3Policy so its goal IDs
            // cannot collide with the legacy regional Recovery goal.
            return false;
        }
        if (config.RegionalAreaTaskSettings.IgnoreRecovery) {
            return false;
        }

        UnitTeam MyTeam = team, EnemyTeam = team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
        int now_time = 420 - timeLeft;
        auto cancel_regional_area_task_for_recovery = [&]() -> bool {
            if (areaManager_.RegionalAreaTaskActive()) {
                if (areaManager_.RegionalAreaTask().Type == RegionalAreaTaskType::MyReadyRoadland) {
                    RequestReadyRoadlandSafeReturn("recovery has higher priority");
                    return TickRegionalAreaTask(MyTeam, EnemyTeam);
                }
                const auto canceled_task_type = areaManager_.RegionalAreaTask().Type;
                areaManager_.ClearRegionalAreaTask();
                defaultStrategyManager_.RecordRegionalAreaResult(
                    canceled_task_type,
                    "canceled",
                    std::chrono::steady_clock::now(),
                    config.RegionalAreaTaskSettings.DefaultPolicy);
                ResetRegionalAreaControlOverride();
                gimbalControlData.FireCode.FollowMode = 0;
                if (LoggerPtr) {
                    LoggerPtr->Info("RegionalAreaTask canceled: recovery has higher priority.");
                }
            }
            return false;
        };
        const bool disable_team_offset_for_debug =
            (config.ShowcasePatrolSettings.Enable && config.ShowcasePatrolSettings.DisableTeamOffset) ||
            (config.NaviDebugSettings.Enable && config.NaviDebugSettings.DisableTeamOffset);
        const bool apply_team_offset = IsLeagueProfile()
            ? true
            : !disable_team_offset_for_debug;
        const auto recovery_goal_id = ResolveGoalId(LangYa::Recovery.ID, MyTeam, apply_team_offset);
        auto set_recovery_position = [&](const char* reason) {
            if (areaManager_.HighlandTransitionActive()) {
                const auto& runtime = areaManager_.TransitionRuntime();
                if (runtime.HasPendingGoal &&
                    runtime.PendingBaseGoal == LangYa::Recovery.ID &&
                    runtime.GoalTeam == MyTeam &&
                    runtime.ApplyTeamOffset == apply_team_offset &&
                    TickNaviAreaTransition()) {
                    naviCommandIntervalClock.reset(Seconds{1});
                    speedLevel = 2;
                    return true;
                }
            }
            if (TryStartNaviAreaTransition(
                    LangYa::Recovery.ID,
                    MyTeam,
                    MyTeam,
                    apply_team_offset,
                    reason)) {
                naviCommandIntervalClock.reset(Seconds{1});
                speedLevel = 2;
                return true;
            }
            SetPositionByBaseGoal(LangYa::Recovery.ID, MyTeam, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{1});
            speedLevel = 2;
            return true;
        };
        if (IsLeagueProfile()) {
            // 联赛回补策略核心：
            // - 以裁判输入（自身血量/弹药）作为唯一触发源
            // - 带 stale 检查，避免旧数据误触发回补
            // - 回补失败后带 cooldown，防止频繁抖动
            const auto& league = config.LeagueStrategySettings;
            const auto now = std::chrono::steady_clock::now();
            if (TickLeagueRouteCompat(MyTeam, apply_team_offset)) {
                return true;
            }
            const std::uint16_t recovery_exit_min = league.HealthRecoveryExitMin;
            const std::uint16_t recovery_exit_preferred = league.HealthRecoveryExitPreferred;
            const auto recovery_plateau = std::chrono::seconds(league.HealthRecoveryPlateauSec);
            const auto recovery_max_hold = std::chrono::seconds(league.HealthRecoveryMaxHoldSec);
            const auto recovery_cooldown = std::chrono::seconds(league.HealthRecoveryCooldownSec);
            auto reset_league_recovery_state = [&]() {
                leagueRecoveryActive_ = false;
                leagueRecoveryStartTime_ = std::chrono::steady_clock::time_point{};
                leagueRecoveryReach350Time_ = std::chrono::steady_clock::time_point{};
                leagueRecoveryLastIncreaseTime_ = std::chrono::steady_clock::time_point{};
                leagueRecoveryEntryHealth_ = 0;
                leagueRecoveryPeakHealth_ = 0;
            };
            auto is_referee_value_ready = [&](const bool has_received,
                                              const std::chrono::steady_clock::time_point& last_rx_time) {
                if (!has_received) {
                    return false;
                }
                if (leagueRefereeStaleTimeoutMs_ <= 0) {
                    return true;
                }
                if (last_rx_time.time_since_epoch().count() == 0) {
                    return false;
                }
                const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_rx_time).count();
                return age_ms <= leagueRefereeStaleTimeoutMs_;
            };

            const bool health_ready = is_referee_value_ready(hasReceivedMyselfHealth_, lastMyselfHealthRxTime);
            const bool health_low = league.UseHealthRecovery && health_ready &&
                myselfHealth < league.HealthRecoveryThreshold;
            const bool missing_health_input = league.UseHealthRecovery && !health_ready;
            const bool health_recovery_cooldown_active =
                leagueRecoveryCooldownUntil_.time_since_epoch().count() != 0 &&
                now < leagueRecoveryCooldownUntil_;
            const bool effective_health_low = health_low && !health_recovery_cooldown_active;

            if (missing_health_input &&
                (now - lastLeagueRecoveryGuardLogTime_ > std::chrono::seconds(2))) {
                LoggerPtr->Warning(
                    "League recovery guard: skip invalid referee inputs (health_ready={} stale_timeout_ms={})",
                    health_ready ? 1 : 0,
                    leagueRefereeStaleTimeoutMs_);
                lastLeagueRecoveryGuardLogTime_ = now;
            }
            if (health_recovery_cooldown_active &&
                (now - lastLeagueRecoveryGuardLogTime_ > std::chrono::seconds(2))) {
                const auto cooldown_left_sec = std::chrono::duration_cast<std::chrono::seconds>(
                    leagueRecoveryCooldownUntil_ - now).count();
                LoggerPtr->Warning(
                    "League recovery cooldown active: skip health-triggered Recovery for {}s.",
                    cooldown_left_sec > 0 ? cooldown_left_sec : 0);
                lastLeagueRecoveryGuardLogTime_ = now;
            }

            if (effective_health_low && !leagueRecoveryActive_) {
                // 进入“血量回补状态机”
                leagueRecoveryActive_ = true;
                leagueRecoveryStartTime_ = now;
                leagueRecoveryReach350Time_ = std::chrono::steady_clock::time_point{};
                leagueRecoveryLastIncreaseTime_ = now;
                leagueRecoveryEntryHealth_ = myselfHealth;
                leagueRecoveryPeakHealth_ = myselfHealth;
                LoggerPtr->Info(
                    "League health recovery activated: hp={} < threshold={}",
                    myselfHealth,
                    league.HealthRecoveryThreshold);
            }

            if (leagueRecoveryActive_) {
                if (health_ready && myselfHealth > leagueRecoveryPeakHealth_) {
                    leagueRecoveryPeakHealth_ = myselfHealth;
                    leagueRecoveryLastIncreaseTime_ = now;
                }

                const bool reach_preferred =
                    health_ready && myselfHealth >= recovery_exit_preferred;
                const bool reach_min_plateau =
                    health_ready &&
                    myselfHealth >= recovery_exit_min &&
                    leagueRecoveryLastIncreaseTime_.time_since_epoch().count() != 0 &&
                    (now - leagueRecoveryLastIncreaseTime_) >= recovery_plateau;
                const bool recovery_timeout =
                    leagueRecoveryStartTime_.time_since_epoch().count() != 0 &&
                    (now - leagueRecoveryStartTime_) >= recovery_max_hold;

                if (reach_preferred || reach_min_plateau) {
                    LoggerPtr->Info(
                        "League health recovery completed: hp={} (peak={}), return to normal strategy.",
                        myselfHealth,
                        leagueRecoveryPeakHealth_);
                    reset_league_recovery_state();
                    return false;
                }

                if (recovery_timeout) {
                    LoggerPtr->Warning(
                        "League health recovery timeout: entry_hp={} peak_hp={} current_hp={}, fallback to normal strategy.",
                        leagueRecoveryEntryHealth_,
                        leagueRecoveryPeakHealth_,
                        myselfHealth);
                    reset_league_recovery_state();
                    leagueRecoveryCooldownUntil_ = now + recovery_cooldown;
                    return false;
                }

                // 回补进行中：持续锁定 Recovery 点位，缩短导航重发间隔。
                if (IsLeagueRouteCompatEnabled() &&
                    IsLeagueGoalSwitchBetween2And3(
                        naviCommandGoal,
                        recovery_goal_id,
                        MyTeam,
                        apply_team_offset)) {
                    StartLeagueRouteCompat(
                        LangYa::Recovery.ID,
                        1,
                        MyTeam,
                        apply_team_offset,
                        "recovery_2_3_switch");
                    return true;
                }
                return set_recovery_position("league_recovery");
            }

            if (naviCommandGoal == recovery_goal_id &&
                (effective_health_low || missing_health_input)) {
                naviCommandIntervalClock.reset(Seconds{1});
                return true;
            }

            if (effective_health_low) {
                if (IsLeagueRouteCompatEnabled() &&
                    IsLeagueGoalSwitchBetween2And3(
                        naviCommandGoal,
                        recovery_goal_id,
                        MyTeam,
                        apply_team_offset)) {
                    StartLeagueRouteCompat(
                        LangYa::Recovery.ID,
                        1,
                        MyTeam,
                        apply_team_offset,
                        "recovery_2_3_switch");
                    return true;
                }
                return set_recovery_position("league_recovery");
            }
            return false;
        }
        constexpr std::uint16_t kRegionalRecoveryHealthEnter = 150;
        constexpr std::uint16_t kRegionalRecoveryHealthExit = 380;
        constexpr std::uint16_t kRegionalRecoveryAmmoThreshold = 30;
        constexpr auto kRegionalRecoveryProbeDelay = std::chrono::seconds(3);
        const auto recovery_default_position =
            AreaManager::GoalPointByBaseId(LangYa::Recovery.ID, MyTeam);
        const auto& recovery_probe_points = Area::RecoveryProbePoints(MyTeam);
        auto same_recovery_point = [](const Area::Point<std::uint16_t>& a,
                                      const Area::Point<std::uint16_t>& b) {
            return a.x == b.x && a.y == b.y;
        };
        auto reset_regional_recovery_probe = [&]() {
            regionalRecoveryProbeActive_ = false;
            regionalRecoveryProbeIndex_ = 0;
            regionalRecoveryMonitorActive_ = false;
            regionalRecoveryMonitorGoal_ = {};
            regionalRecoveryMonitorStartTime_ = std::chrono::steady_clock::time_point{};
            regionalRecoveryMonitorHealth_ = 0;
            regionalRecoveryMonitorAmmo_ = 0;
        };
        auto command_recovery_position = [&](const Area::Point<std::uint16_t>& point,
                                             const char* reason) {
            const bool changed =
                naviCommandGoal != recovery_goal_id ||
                !same_recovery_point(naviGoalPosition, point);
            naviCommandGoal = recovery_goal_id;
            naviGoalPosition = point;
            naviGoalPublishAllowed_ = true;
            speedLevel = 2;
            UpdateNaviProgressWatchdogGoal(LangYa::Recovery.ID, MyTeam, apply_team_offset);
            naviCommandIntervalClock.reset(Seconds{1});
            if (changed) {
                regionalRecoveryMonitorActive_ = false;
                const std::string detail =
                    std::string{reason} +
                    " hp=" + std::to_string(myselfHealth) +
                    " ammo=" + std::to_string(ammoLeft);
                RecordDecisionIntent(MakeDecisionIntent(
                    DecisionReason::Recovery,
                    LangYa::Recovery.ID,
                    MyTeam,
                    apply_team_offset,
                    detail.c_str()));
                if (LoggerPtr) {
                    LoggerPtr->Info(
                        "Regional recovery target: reason={} point=({}, {}) hp={} ammo={}.",
                        reason,
                        static_cast<int>(point.x),
                        static_cast<int>(point.y),
                        myselfHealth,
                        ammoLeft);
                }
            }
        };
        auto tick_regional_recovery_position = [&]() {
            if (!config.NaviSettings.UseXY && regionalRecoveryProbeActive_) {
                reset_regional_recovery_probe();
            }
            if (!regionalRecoveryProbeActive_) {
                if (areaManager_.HighlandTransitionActive()) {
                    const auto& runtime = areaManager_.TransitionRuntime();
                    if (runtime.HasPendingGoal &&
                        runtime.PendingBaseGoal == LangYa::Recovery.ID &&
                        runtime.GoalTeam == MyTeam &&
                        runtime.ApplyTeamOffset == apply_team_offset &&
                        TickNaviAreaTransition()) {
                        naviCommandIntervalClock.reset(Seconds{1});
                        speedLevel = 2;
                        return true;
                    }
                }
                if (naviCommandGoal != recovery_goal_id &&
                    TryStartNaviAreaTransition(
                        LangYa::Recovery.ID,
                        MyTeam,
                        MyTeam,
                        apply_team_offset,
                        "regional_recovery")) {
                    naviCommandIntervalClock.reset(Seconds{1});
                    speedLevel = 2;
                    return true;
                }
            }
            Area::Point<std::uint16_t> target_position = recovery_default_position;
            if (config.NaviSettings.UseXY &&
                regionalRecoveryProbeActive_ &&
                !recovery_probe_points.empty()) {
                regionalRecoveryProbeIndex_ %= recovery_probe_points.size();
                target_position = recovery_probe_points[regionalRecoveryProbeIndex_];
            }

            if (naviCommandGoal != recovery_goal_id ||
                !same_recovery_point(naviGoalPosition, target_position)) {
                command_recovery_position(
                    target_position,
                    regionalRecoveryProbeActive_ ? "sync_probe" : "sync_default");
                return true;
            }

            if (!IsNaviGoalPositionArrived(recovery_goal_id, target_position)) {
                regionalRecoveryMonitorActive_ = false;
                naviCommandIntervalClock.reset(Seconds{1});
                return true;
            }

            const auto now = std::chrono::steady_clock::now();
            if (!regionalRecoveryMonitorActive_ ||
                !same_recovery_point(regionalRecoveryMonitorGoal_, target_position)) {
                regionalRecoveryMonitorActive_ = true;
                regionalRecoveryMonitorGoal_ = target_position;
                regionalRecoveryMonitorStartTime_ = now;
                regionalRecoveryMonitorHealth_ = myselfHealth;
                regionalRecoveryMonitorAmmo_ = ammoLeft;
                naviCommandIntervalClock.reset(Seconds{1});
                return true;
            }

            const bool resource_improved =
                myselfHealth > regionalRecoveryMonitorHealth_ ||
                ammoLeft > regionalRecoveryMonitorAmmo_;
            if (resource_improved) {
                regionalRecoveryMonitorStartTime_ = now;
                regionalRecoveryMonitorHealth_ = myselfHealth;
                regionalRecoveryMonitorAmmo_ = ammoLeft;
                naviCommandIntervalClock.reset(Seconds{1});
                return true;
            }

            if (config.NaviSettings.UseXY &&
                !recovery_probe_points.empty() &&
                now - regionalRecoveryMonitorStartTime_ >= kRegionalRecoveryProbeDelay) {
                if (!regionalRecoveryProbeActive_) {
                    regionalRecoveryProbeActive_ = true;
                    regionalRecoveryProbeIndex_ = 0;
                } else {
                    regionalRecoveryProbeIndex_ =
                        (regionalRecoveryProbeIndex_ + 1U) % recovery_probe_points.size();
                }
                regionalRecoveryMonitorActive_ = false;
                command_recovery_position(
                    recovery_probe_points[regionalRecoveryProbeIndex_],
                    "no_resource_change_probe");
                return true;
            }

            naviCommandIntervalClock.reset(Seconds{1});
            return true;
        };
        const bool regional_recovery_needed =
            myselfHealth < kRegionalRecoveryHealthEnter ||
            ammoLeft <= kRegionalRecoveryAmmoThreshold;
        const bool regional_recovery_ready =
            myselfHealth >= kRegionalRecoveryHealthExit &&
            ammoLeft > kRegionalRecoveryAmmoThreshold;

        if (regional_recovery_ready) {
            reset_regional_recovery_probe();
        }

        // 复活/回补保持：进入 Recovery 后，血量和弹量都恢复才释放。
        if(naviCommandGoal == recovery_goal_id) {
            if(!regional_recovery_ready) {
                if (cancel_regional_area_task_for_recovery()) {
                    return true;
                }
                return tick_regional_recovery_position();
            }
        }
        // 回家
        // 条件为：血量低于进入阈值，或者弹药低于进入阈值。
        if(regional_recovery_needed) {
            if (cancel_regional_area_task_for_recovery()) {
                return true;
            }
            reset_regional_recovery_probe();
            command_recovery_position(recovery_default_position, "enter_default");
            return true;
        }
        reset_regional_recovery_probe();
        return false;
    }


}
