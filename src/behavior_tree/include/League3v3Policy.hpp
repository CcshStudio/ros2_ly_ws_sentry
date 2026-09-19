#pragma once

#include <cstddef>
#include <cstdint>

#include "../module/BasicTypes.hpp"

namespace BehaviorTree {

enum class League3v3State : std::uint8_t {
    HoldStart = 0,
    RecoverAtSupply,
    GoCenter,
    HoldCenter,
    DefendCenter,
    CoverWeakAlly,
    RetreatBehindAlly,
    CenterPatrol,
    ContestCenter,
    ChaseEnemyHero
};

enum class League3v3Goal : std::uint8_t {
    None = 0xFF,
    SelfBase = 0,
    CenterAnchor = 1,
    EnemyStartApproach = 2,
    CenterOwnSide = 3,
    CenterTop = 4,
    CenterEnemySide = 5
};

enum class League3v3Target : std::uint8_t {
    None = 0,
    EnemyHero,
    EnemySentry,
    EnemyInfantry
};

enum class League3v3CenterStatus : std::uint8_t {
    Unoccupied = 0,
    Ours = 1,
    Enemy = 2,
    Contested = 3
};

struct League3v3UnitSnapshot {
    bool HpFresh{false};
    std::uint16_t Hp{0};
    bool PositionFresh{false};
    bool Visible{false};
};

struct League3v3TeamSnapshot {
    League3v3UnitSnapshot Hero{};
    League3v3UnitSnapshot Sentry{};
    League3v3UnitSnapshot Infantry{};
};

struct League3v3Point {
    std::uint16_t X{0};
    std::uint16_t Y{0};
};

struct League3v3Input {
    bool GameRunning{false};
    bool SelfHpFresh{false};
    std::uint16_t SelfHp{0};
    bool EventDataFresh{false};
    League3v3CenterStatus CenterStatus{League3v3CenterStatus::Unoccupied};
    League3v3TeamSnapshot Friend{};
    League3v3TeamSnapshot Enemy{};
    bool EnemyNearCenter{false};
    std::int64_t NowMs{0};
};

struct League3v3Decision {
    League3v3State State{League3v3State::HoldStart};
    League3v3Goal Goal{League3v3Goal::SelfBase};
    League3v3Target Target{League3v3Target::None};
    const char* Reason{"hold_start"};
};

class League3v3Policy {
public:
    explicit League3v3Policy(LangYa::League3v3Setting setting = {}) noexcept;

    League3v3Decision Decide(const League3v3Input& input) noexcept;

private:
    League3v3Goal CenterPatrolGoal(std::int64_t now_ms) noexcept;
    void ResetCenterPatrolState() noexcept;
    League3v3Target SelectTarget(const League3v3Input& input) const noexcept;
    bool FriendWeak(const League3v3Input& input) const noexcept;
    bool FriendPositionFresh(const League3v3Input& input) const noexcept;
    bool EnemyHeroWeakAndTrackable(const League3v3Input& input) const noexcept;
    bool UnitTrackable(const League3v3UnitSnapshot& unit) const noexcept;

    LangYa::League3v3Setting setting_{};
    bool recovery_latched_{false};
    std::size_t center_patrol_index_{0};
    std::int64_t center_patrol_start_ms_{0};
    std::int64_t center_patrol_switch_ms_{0};
    bool center_patrol_started_{false};
    bool center_patrol_timed_out_{false};
};

const char* League3v3StateName(League3v3State state) noexcept;
const char* League3v3GoalName(League3v3Goal goal) noexcept;
const char* League3v3TargetName(League3v3Target target) noexcept;
League3v3Point League3v3GoalPoint(League3v3Goal goal, LangYa::UnitTeam team) noexcept;
bool IsLeague3v3ControlAreaPoint(int x_cm, int y_cm) noexcept;

}  // namespace BehaviorTree
