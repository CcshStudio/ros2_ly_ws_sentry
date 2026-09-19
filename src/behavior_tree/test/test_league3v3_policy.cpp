#include <gtest/gtest.h>

#include "League3v3Policy.hpp"

namespace {

using BehaviorTree::League3v3CenterStatus;
using BehaviorTree::League3v3Goal;
using BehaviorTree::League3v3Input;
using BehaviorTree::League3v3Policy;
using BehaviorTree::League3v3State;
using BehaviorTree::League3v3Target;
using BehaviorTree::League3v3UnitSnapshot;

League3v3Input RunningInput() {
    League3v3Input input;
    input.GameRunning = true;
    input.SelfHpFresh = true;
    input.SelfHp = 400;
    input.EventDataFresh = true;
    input.CenterStatus = League3v3CenterStatus::Unoccupied;
    return input;
}

League3v3UnitSnapshot WeakHero() {
    League3v3UnitSnapshot unit;
    unit.HpFresh = true;
    unit.Hp = 100;
    unit.PositionFresh = true;
    return unit;
}

League3v3UnitSnapshot HealthyInfantry() {
    League3v3UnitSnapshot unit;
    unit.HpFresh = true;
    unit.Hp = 300;
    unit.PositionFresh = true;
    return unit;
}

}  // namespace

TEST(League3v3PolicyTest, HoldsStartBeforeGame) {
    League3v3Policy policy;
    const auto decision = policy.Decide(League3v3Input{});

    EXPECT_EQ(decision.State, League3v3State::HoldStart);
    EXPECT_EQ(decision.Goal, League3v3Goal::SelfBase);
}

TEST(League3v3PolicyTest, RecoversAtSupplyAndLatchesUntilFull) {
    League3v3Policy policy;
    auto input = RunningInput();
    input.SelfHp = 250;

    auto decision = policy.Decide(input);
    ASSERT_EQ(decision.State, League3v3State::RecoverAtSupply);
    EXPECT_EQ(decision.Goal, League3v3Goal::SelfBase);

    input.SelfHp = 399;
    decision = policy.Decide(input);
    EXPECT_EQ(decision.State, League3v3State::RecoverAtSupply);

    input.SelfHp = 400;
    decision = policy.Decide(input);
    EXPECT_NE(decision.State, League3v3State::RecoverAtSupply);
}

TEST(League3v3PolicyTest, GoesCenterWhenCenterUnoccupied) {
    League3v3Policy policy;
    auto input = RunningInput();
    input.CenterStatus = League3v3CenterStatus::Unoccupied;

    const auto decision = policy.Decide(input);

    EXPECT_EQ(decision.State, League3v3State::GoCenter);
    EXPECT_EQ(decision.Goal, League3v3Goal::CenterAnchor);
}

TEST(League3v3PolicyTest, ChasesWeakEnemyHeroWhenCenterOurs) {
    League3v3Policy policy;
    auto input = RunningInput();
    input.CenterStatus = League3v3CenterStatus::Ours;
    input.Enemy.Hero = WeakHero();

    const auto decision = policy.Decide(input);

    EXPECT_EQ(decision.State, League3v3State::ChaseEnemyHero);
    EXPECT_EQ(decision.Goal, League3v3Goal::EnemyStartApproach);
    EXPECT_EQ(decision.Target, League3v3Target::EnemyHero);
}

TEST(League3v3PolicyTest, CoversWeakAllyWhenEnemyOwnsCenterAndSelfHealthy) {
    League3v3Policy policy;
    auto input = RunningInput();
    input.CenterStatus = League3v3CenterStatus::Enemy;
    input.Friend.Hero = WeakHero();

    const auto decision = policy.Decide(input);

    EXPECT_EQ(decision.State, League3v3State::CoverWeakAlly);
    EXPECT_EQ(decision.Goal, League3v3Goal::CenterOwnSide);
}

TEST(League3v3PolicyTest, RetreatsBehindHealthyAllyWhenSelfWeak) {
    League3v3Policy policy;
    auto input = RunningInput();
    input.SelfHp = 270;
    input.CenterStatus = League3v3CenterStatus::Enemy;
    input.Friend.Hero = HealthyInfantry();

    const auto decision = policy.Decide(input);

    EXPECT_EQ(decision.State, League3v3State::RetreatBehindAlly);
    EXPECT_EQ(decision.Goal, League3v3Goal::CenterOwnSide);
}

TEST(League3v3PolicyTest, ContestedCenterPrioritizesHeroOverSentry) {
    League3v3Policy policy;
    auto input = RunningInput();
    input.CenterStatus = League3v3CenterStatus::Contested;
    input.Enemy.Hero = WeakHero();
    input.Enemy.Sentry = HealthyInfantry();

    const auto decision = policy.Decide(input);

    EXPECT_EQ(decision.State, League3v3State::DefendCenter);
    EXPECT_EQ(decision.Target, League3v3Target::EnemyHero);
}

TEST(League3v3PolicyTest, CoordinationDisabledIgnoresNonVisibleEnemyIntel) {
    LangYa::League3v3Setting setting;
    setting.AssumeCoordination = false;
    League3v3Policy policy(setting);
    auto input = RunningInput();
    input.CenterStatus = League3v3CenterStatus::Ours;
    input.Enemy.Hero = WeakHero();

    const auto decision = policy.Decide(input);

    EXPECT_EQ(decision.State, League3v3State::CenterPatrol);
    EXPECT_EQ(decision.Target, League3v3Target::None);
}

TEST(League3v3PolicyTest, OwnedCenterPatrolUsesCenterGoals) {
    League3v3Policy policy;
    auto input = RunningInput();
    input.CenterStatus = League3v3CenterStatus::Ours;
    input.NowMs = 0;

    auto decision = policy.Decide(input);
    EXPECT_EQ(decision.State, League3v3State::CenterPatrol);
    EXPECT_EQ(decision.Goal, League3v3Goal::CenterOwnSide);

    input.NowMs = 5000;
    decision = policy.Decide(input);
    EXPECT_EQ(decision.Goal, League3v3Goal::CenterTop);

    input.NowMs = 10000;
    decision = policy.Decide(input);
    EXPECT_EQ(decision.Goal, League3v3Goal::CenterAnchor);
}

TEST(League3v3PolicyTest, DeadEnemyWithFreshPositionIsNotTrackable) {
    League3v3Policy policy;
    auto input = RunningInput();
    input.CenterStatus = League3v3CenterStatus::Contested;
    input.Enemy.Hero.HpFresh = true;
    input.Enemy.Hero.Hp = 0;
    input.Enemy.Hero.PositionFresh = true;

    const auto decision = policy.Decide(input);

    EXPECT_EQ(decision.Target, League3v3Target::None);
}

TEST(League3v3PolicyTest, GoalPointsUseTeamMirrorAndKeepEnemyApproachOutsideStart) {
    const auto red_enemy = BehaviorTree::League3v3GoalPoint(
        League3v3Goal::EnemyStartApproach,
        LangYa::UnitTeam::Red);
    const auto blue_enemy = BehaviorTree::League3v3GoalPoint(
        League3v3Goal::EnemyStartApproach,
        LangYa::UnitTeam::Blue);

    EXPECT_EQ(red_enemy.X, 1050);
    EXPECT_EQ(red_enemy.Y, 630);
    EXPECT_EQ(blue_enemy.X, 150);
    EXPECT_EQ(blue_enemy.Y, 170);
    EXPECT_FALSE(BehaviorTree::IsLeague3v3ControlAreaPoint(red_enemy.X, red_enemy.Y));
    EXPECT_FALSE(BehaviorTree::IsLeague3v3ControlAreaPoint(blue_enemy.X, blue_enemy.Y));
}
