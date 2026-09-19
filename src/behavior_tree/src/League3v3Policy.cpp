#include "../include/League3v3Policy.hpp"

#include <algorithm>
#include <array>

namespace BehaviorTree {
namespace {

bool IsVisibleTarget(const League3v3UnitSnapshot& unit) noexcept {
    return unit.Visible && (!unit.HpFresh || unit.Hp > 0U);
}

bool IsFreshHpTarget(const League3v3UnitSnapshot& unit, const bool assume_coordination) noexcept {
    return assume_coordination && unit.HpFresh && unit.Hp > 0U;
}

}  // namespace

League3v3Point League3v3GoalPoint(const League3v3Goal goal, const LangYa::UnitTeam team) noexcept {
    const bool blue = team == LangYa::UnitTeam::Blue;
    switch (goal) {
        case League3v3Goal::SelfBase: return blue ? League3v3Point{1125, 100} : League3v3Point{75, 700};
        case League3v3Goal::CenterAnchor: return League3v3Point{600, 400};
        case League3v3Goal::EnemyStartApproach: return blue ? League3v3Point{150, 170} : League3v3Point{1050, 630};
        case League3v3Goal::CenterOwnSide: return blue ? League3v3Point{700, 400} : League3v3Point{500, 400};
        case League3v3Goal::CenterTop: return blue ? League3v3Point{600, 300} : League3v3Point{600, 500};
        case League3v3Goal::CenterEnemySide: return blue ? League3v3Point{500, 400} : League3v3Point{700, 400};
        case League3v3Goal::None: return {};
    }
    return {};
}

bool IsLeague3v3ControlAreaPoint(const int x_cm, const int y_cm) noexcept {
    return x_cm >= 435 && x_cm <= 765 && y_cm >= 250 && y_cm <= 550;
}
League3v3Policy::League3v3Policy(LangYa::League3v3Setting setting) noexcept
: setting_(setting) {}

const char* League3v3StateName(const League3v3State state) noexcept {
    switch (state) {
        case League3v3State::HoldStart: return "hold_start";
        case League3v3State::RecoverAtSupply: return "recover_at_supply";
        case League3v3State::GoCenter: return "go_center";
        case League3v3State::HoldCenter: return "hold_center";
        case League3v3State::DefendCenter: return "defend_center";
        case League3v3State::CoverWeakAlly: return "cover_weak_ally";
        case League3v3State::RetreatBehindAlly: return "retreat_behind_ally";
        case League3v3State::CenterPatrol: return "center_patrol";
        case League3v3State::ContestCenter: return "contest_center";
        case League3v3State::ChaseEnemyHero: return "chase_enemy_hero";
    }
    return "unknown";
}

const char* League3v3GoalName(const League3v3Goal goal) noexcept {
    switch (goal) {
        case League3v3Goal::None: return "none";
        case League3v3Goal::SelfBase: return "self_base";
        case League3v3Goal::CenterAnchor: return "center_anchor";
        case League3v3Goal::EnemyStartApproach: return "enemy_start_approach";
        case League3v3Goal::CenterOwnSide: return "center_own_side";
        case League3v3Goal::CenterTop: return "center_top";
        case League3v3Goal::CenterEnemySide: return "center_enemy_side";
    }
    return "unknown";
}

const char* League3v3TargetName(const League3v3Target target) noexcept {
    switch (target) {
        case League3v3Target::None: return "none";
        case League3v3Target::EnemyHero: return "enemy_hero";
        case League3v3Target::EnemySentry: return "enemy_sentry";
        case League3v3Target::EnemyInfantry: return "enemy_infantry";
    }
    return "unknown";
}

bool League3v3Policy::UnitTrackable(const League3v3UnitSnapshot& unit) const noexcept {
    if (unit.HpFresh && unit.Hp == 0U) {
        return false;
    }
    return IsVisibleTarget(unit) ||
        IsFreshHpTarget(unit, setting_.AssumeCoordination) ||
        (setting_.AssumeCoordination && unit.PositionFresh);
}

League3v3Target League3v3Policy::SelectTarget(const League3v3Input& input) const noexcept {
    if (UnitTrackable(input.Enemy.Hero)) {
        return League3v3Target::EnemyHero;
    }
    if (UnitTrackable(input.Enemy.Sentry)) {
        return League3v3Target::EnemySentry;
    }
    if (UnitTrackable(input.Enemy.Infantry)) {
        return League3v3Target::EnemyInfantry;
    }
    return League3v3Target::None;
}

bool League3v3Policy::FriendWeak(const League3v3Input& input) const noexcept {
    const auto weak = [this](const League3v3UnitSnapshot& unit) {
        return unit.HpFresh && unit.Hp <= setting_.TeammateWeakHp;
    };
    return weak(input.Friend.Hero) || weak(input.Friend.Infantry);
}

bool League3v3Policy::FriendPositionFresh(const League3v3Input& input) const noexcept {
    return (input.Friend.Hero.PositionFresh && input.Friend.Hero.HpFresh) ||
        (input.Friend.Infantry.PositionFresh && input.Friend.Infantry.HpFresh);
}

bool League3v3Policy::EnemyHeroWeakAndTrackable(const League3v3Input& input) const noexcept {
    if (!setting_.AssumeCoordination && !input.Enemy.Hero.Visible) {
        return false;
    }
    const bool weak = input.Enemy.Hero.HpFresh &&
        input.Enemy.Hero.Hp > 0U &&
        input.Enemy.Hero.Hp <= setting_.EnemyWeakHp;
    const bool trackable = input.Enemy.Hero.PositionFresh || input.Enemy.Hero.Visible;
    return weak && trackable;
}

void League3v3Policy::ResetCenterPatrolState() noexcept {
    center_patrol_index_ = 0U;
    center_patrol_start_ms_ = 0;
    center_patrol_switch_ms_ = 0;
    center_patrol_started_ = false;
    center_patrol_timed_out_ = false;
}

League3v3Goal League3v3Policy::CenterPatrolGoal(const std::int64_t now_ms) noexcept {
    constexpr std::array<League3v3Goal, 3> kPatrolGoals{
        League3v3Goal::CenterOwnSide,
        League3v3Goal::CenterTop,
        League3v3Goal::CenterAnchor};
    const auto hold_ms = static_cast<std::int64_t>(
        std::max(1, setting_.CenterPatrolHoldSec)) * 1000;
    const auto timeout_ms = static_cast<std::int64_t>(
        std::max(1, setting_.CenterPatrolTimeoutSec)) * 1000;

    if (!center_patrol_started_) {
        center_patrol_started_ = true;
        center_patrol_start_ms_ = now_ms;
        center_patrol_switch_ms_ = now_ms;
        return kPatrolGoals[center_patrol_index_];
    }
    if (center_patrol_timed_out_) {
        return League3v3Goal::CenterAnchor;
    }
    if (now_ms >= center_patrol_start_ms_ &&
        now_ms - center_patrol_start_ms_ >= timeout_ms) {
        center_patrol_timed_out_ = true;
        return League3v3Goal::CenterAnchor;
    }
    if (now_ms >= center_patrol_switch_ms_ &&
        now_ms - center_patrol_switch_ms_ >= hold_ms) {
        center_patrol_switch_ms_ = now_ms;
        center_patrol_index_ = (center_patrol_index_ + 1U) % kPatrolGoals.size();
    }
    return kPatrolGoals[center_patrol_index_];
}

League3v3Decision League3v3Policy::Decide(const League3v3Input& input) noexcept {
    if (!input.GameRunning) {
        recovery_latched_ = false;
        ResetCenterPatrolState();
        return {League3v3State::HoldStart, League3v3Goal::SelfBase, League3v3Target::None, "not_running"};
    }

    if (input.SelfHpFresh && input.SelfHp <= setting_.RecoveryHp) {
        recovery_latched_ = true;
    }
    if (recovery_latched_) {
        if (input.SelfHpFresh && input.SelfHp >= setting_.RecoveryExitHp) {
            recovery_latched_ = false;
        } else {
            ResetCenterPatrolState();
            return {League3v3State::RecoverAtSupply, League3v3Goal::SelfBase, League3v3Target::None, "recover"};
        }
    }

    if (!input.EventDataFresh) {
        ResetCenterPatrolState();
        return {
            League3v3State::DefendCenter,
            League3v3Goal::CenterAnchor,
            SelectTarget(input),
            "center_status_stale"};
    }

    const bool self_weak = input.SelfHpFresh && input.SelfHp <= setting_.SelfWeakHp;
    const bool friend_weak = FriendWeak(input);
    const bool friend_position_fresh = FriendPositionFresh(input);
    const auto target = SelectTarget(input);

    switch (input.CenterStatus) {
        case League3v3CenterStatus::Unoccupied:
            ResetCenterPatrolState();
            return {League3v3State::GoCenter, League3v3Goal::CenterAnchor, target, "center_unoccupied"};

        case League3v3CenterStatus::Ours:
            if (!self_weak && EnemyHeroWeakAndTrackable(input)) {
                ResetCenterPatrolState();
                return {
                    League3v3State::ChaseEnemyHero,
                    League3v3Goal::EnemyStartApproach,
                    League3v3Target::EnemyHero,
                    "weak_enemy_hero"};
            }
            if (input.EnemyNearCenter) {
                ResetCenterPatrolState();
                return {League3v3State::DefendCenter, League3v3Goal::CenterAnchor, target, "enemy_near_center"};
            }
            return {
                League3v3State::CenterPatrol,
                CenterPatrolGoal(input.NowMs),
                target,
                "hold_and_patrol_center"};

        case League3v3CenterStatus::Enemy:
            ResetCenterPatrolState();
            if (!self_weak && friend_weak && friend_position_fresh) {
                return {
                    League3v3State::CoverWeakAlly,
                    League3v3Goal::CenterOwnSide,
                    target,
                    "cover_weak_ally"};
            }
            if (self_weak && !friend_weak && friend_position_fresh) {
                return {
                    League3v3State::RetreatBehindAlly,
                    League3v3Goal::CenterOwnSide,
                    target,
                    "retreat_behind_ally"};
            }
            return {
                League3v3State::ContestCenter,
                League3v3Goal::CenterEnemySide,
                target,
                "contest_enemy_center"};

        case League3v3CenterStatus::Contested:
            ResetCenterPatrolState();
            return {
                League3v3State::DefendCenter,
                League3v3Goal::CenterAnchor,
                target,
                "center_contested"};
    }

    ResetCenterPatrolState();
    return {League3v3State::HoldCenter, League3v3Goal::CenterAnchor, target, "fallback"};
}

}  // namespace BehaviorTree
