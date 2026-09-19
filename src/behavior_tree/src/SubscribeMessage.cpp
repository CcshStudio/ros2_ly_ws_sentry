// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"
#include "../include/EnemyPositionSourcePolicy.hpp"
#include <algorithm>
#include <limits>
#include <cmath>
#include <optional>

using namespace LangYa;

namespace BehaviorTree{

    namespace {
    constexpr int kOfficialFieldWidthCm = 2800;
    constexpr int kOfficialFieldHeightCm = 1500;

    bool IsRawPositionZero(const int x, const int y) {
        return x == 0 && y == 0;
    }

    bool IsOfficialFieldPointValid(const int x, const int y) {
        return x > 0 && y > 0 &&
               x <= kOfficialFieldWidthCm &&
               y <= kOfficialFieldHeightCm;
    }

    std::optional<UnitType> UnitTypeFromArmorTypeId(const std::uint16_t armor_type_id) {
        switch (static_cast<ArmorType>(armor_type_id)) {
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

    RfidMatchState BuildRfidMatchState(const gimbal_driver::msg::RfidStatus& msg) {
        RfidMatchState state;
        state.Fresh = true;
        state.Raw = msg.raw;
        state.HasRfidStatus2 = msg.has_rfid_status_2;
        state.RfidStatus2Raw = msg.rfid_status_2_raw;

        state.SelfBaseGainPoint = msg.friend_base;
        state.SelfNonResourceSupply = msg.friend_supply_noremix;
        state.SelfResourceSupply = msg.friend_supply_remix;
        state.SelfSupply = state.SelfNonResourceSupply || state.SelfResourceSupply;
        state.SelfHighlandGainPoint =
            msg.friend_central || msg.friend_highland;
        state.EnemyHighlandGainPoint =
            msg.enemy_central || msg.enemy_highland;
        state.SelfRoadCrossing = msg.friend_roadland_under || msg.friend_roadland_high;
        state.EnemyRoadCrossing = msg.enemy_roadland_under || msg.enemy_roadland_high;
        state.SelfCentralHighlandCrossing =
            msg.friend_central_under || msg.friend_central_high;
        state.EnemyCentralHighlandCrossing =
            msg.enemy_central_under || msg.enemy_central_high;
        state.SelfTunnel =
            msg.friend_tunnel_roadland_down || msg.friend_tunnel_roadland_mid ||
            msg.friend_tunnel_roadland_up || msg.friend_tunnel_highland_low ||
            msg.friend_tunnel_highland_mid || msg.friend_tunnel_highland_high;
        state.EnemyTunnel =
            msg.enemy_tunnel_roadland_down || msg.enemy_tunnel_roadland_mid ||
            msg.enemy_tunnel_roadland_up || msg.enemy_tunnel_highland_low ||
            msg.enemy_tunnel_highland_mid || msg.enemy_tunnel_highland_high;
        state.Tunnel = state.SelfTunnel || state.EnemyTunnel;
        state.CenterGainPoint = msg.central_rmul;
        state.SelfFortressGainPoint = msg.friend_bastion;
        state.EnemyFortressGainPoint = msg.enemy_bastion;
        state.SelfOutpostGainPoint = msg.friend_outpost;
        state.EnemyOutpostGainPoint = msg.enemy_outpost;
        state.SelfAssemblyGainPoint = msg.friend_armor;
        state.EnemyAssemblyGainPoint = msg.enemy_armor;
        state.SelfFlyRamp = msg.friend_flyroad_front || msg.friend_flyroad_back;
        state.EnemyFlyRamp = msg.enemy_flyroad_front || msg.enemy_flyroad_back;

        state.OnSelfSideRfid =
            state.SelfBaseGainPoint || state.SelfSupply || state.SelfHighlandGainPoint ||
            state.SelfRoadCrossing || state.SelfCentralHighlandCrossing || state.SelfTunnel ||
            state.SelfFortressGainPoint || state.SelfOutpostGainPoint || state.SelfAssemblyGainPoint ||
            state.SelfFlyRamp;
        state.OnEnemySideRfid =
            state.EnemyHighlandGainPoint || state.EnemyRoadCrossing ||
            state.EnemyCentralHighlandCrossing || state.EnemyTunnel || state.EnemyFortressGainPoint ||
            state.EnemyOutpostGainPoint || state.EnemyAssemblyGainPoint || state.EnemyFlyRamp;
        state.Any = state.OnSelfSideRfid || state.OnEnemySideRfid || state.CenterGainPoint;
        return state;
    }
    }  // namespace

    void Application::PrintMessageAll() {
        LoggerPtr->Debug("-------------PrintMessageAll-------------");
        LoggerPtr->Debug("|  Myself Hreo Position: {}, {}", friendRobots[UnitType::Hero].position_.X, friendRobots[UnitType::Hero].position_.Y);
        LoggerPtr->Debug("|  Myself Engineer Position: {}, {}", friendRobots[UnitType::Engineer].position_.X, friendRobots[UnitType::Engineer].position_.Y);
        LoggerPtr->Debug("|  Myself Infantry1 Position: {}, {}", friendRobots[UnitType::Infantry1].position_.X, friendRobots[UnitType::Infantry1].position_.Y);
        LoggerPtr->Debug("|  Myself Infantry2 Position: {}, {}", friendRobots[UnitType::Infantry2].position_.X, friendRobots[UnitType::Infantry2].position_.Y);
        LoggerPtr->Debug("|  Myself Sentry Position: {}, {}", friendRobots[UnitType::Sentry].position_.X, friendRobots[UnitType::Sentry].position_.Y);
        LoggerPtr->Debug("|-----------------------------------------");
        LoggerPtr->Debug("|  Enemy Hero Position: {}, {}", enemyRobots[UnitType::Hero].position_.X, enemyRobots[UnitType::Hero].position_.Y);
        LoggerPtr->Debug("|  Enemy Engineer Position: {}, {}", enemyRobots[UnitType::Engineer].position_.X, enemyRobots[UnitType::Engineer].position_.Y);
        LoggerPtr->Debug("|  Enemy Infantry1 Position: {}, {}", enemyRobots[UnitType::Infantry1].position_.X, enemyRobots[UnitType::Infantry1].position_.Y);
        LoggerPtr->Debug("|  Enemy Infantry2 Position: {}, {}", enemyRobots[UnitType::Infantry2].position_.X, enemyRobots[UnitType::Infantry2].position_.Y);
        LoggerPtr->Debug("|  Enemy Sentry Position: {}, {}", enemyRobots[UnitType::Sentry].position_.X, enemyRobots[UnitType::Sentry].position_.Y);
        LoggerPtr->Debug("|-----------------------------------------");
        LoggerPtr->Debug("|  Myself Hero Health: {}", friendRobots[UnitType::Hero].currentHealth_);
        LoggerPtr->Debug("|  Myself Engineer Health: {}", friendRobots[UnitType::Engineer].currentHealth_);
        LoggerPtr->Debug("|  Myself Infantry1 Health: {}", friendRobots[UnitType::Infantry1].currentHealth_);
        LoggerPtr->Debug("|  Myself Infantry2 Health: {}", friendRobots[UnitType::Infantry2].currentHealth_);
        LoggerPtr->Debug("|  Myself Sentry Health: {}", friendRobots[UnitType::Sentry].currentHealth_);
        LoggerPtr->Debug("|-----------------------------------------");
        LoggerPtr->Debug("|  Enemy Hero Health: {}", enemyRobots[UnitType::Hero].currentHealth_);
        LoggerPtr->Debug("|  Enemy Engineer Health: {}", enemyRobots[UnitType::Engineer].currentHealth_);
        LoggerPtr->Debug("|  Enemy Infantry1 Health: {}", enemyRobots[UnitType::Infantry1].currentHealth_);
        LoggerPtr->Debug("|  Enemy Infantry2 Health: {}", enemyRobots[UnitType::Infantry2].currentHealth_);
        LoggerPtr->Debug("|  Enemy Sentry Health: {}", enemyRobots[UnitType::Sentry].currentHealth_);
        LoggerPtr->Debug("|------------------------------------");
        LoggerPtr->Debug("|  DefenceBuff: {}", int(teamBuff.DefenceBuff));
        LoggerPtr->Debug("|  RecoveryBuff: {}", int(teamBuff.RecoveryBuff));
        LoggerPtr->Debug("|  RemainingEnergy: {}", int(teamBuff.RemainingEnergy));
        LoggerPtr->Debug("|  VulnerabilityBuff: {}", int(teamBuff.VulnerabilityBuff));
        LoggerPtr->Debug("|-----------------------------------------");
        LoggerPtr->Debug("|  CapV: {}", int(capV));
        LoggerPtr->Debug("-----------End PrintMessageAll-----------");
    }
    void Application::SubscribeMessageAll() {
        auto stamp_or_now = [](Application& app, const auto& header)
            -> rclcpp::Time {
            if (header.stamp.sec != 0 || header.stamp.nanosec != 0) {
                return rclcpp::Time(header.stamp);
            }
            return app.node_->now();
        };

        // 输入分组说明：
        // 1) 云台与火控回读
        // 2) 裁判/比赛态数据（血量、弹药、时间、开赛标志）
        // 3) 外部 aim 结果与目标候选。
        // 4) 导航与定位（速度、位置、低头标志）
        // ly_gimbal_angles
        GenSub<ly_gimbal_angles>([](Application& app, auto msg) {
            app.gimbalAngles = GimbalAnglesType{
                static_cast<AngleType>(msg->yaw),
                static_cast<AngleType>(msg->pitch)
            }; // ROS2改成小寫
            app.hasReceivedGimbalAngles_ = true;
            app.lastGimbalAnglesRxTime = std::chrono::steady_clock::now();
        });

        // ly_gimbal_firecode
        GenSub<ly_gimbal_firecode>([](Application& app, auto msg) {
            app.CaptureGimbalFeedbackTraceSnapshot(*msg, std::chrono::steady_clock::now());
            app.RecFireCode.FireStatus = msg->fire_status & 0b11;
            app.RecFireCode.CapState = msg->cap_state & 0b11;
            app.RecFireCode.FollowMode = msg->follow_mode ? 1 : 0;
            app.RecFireCode.AimMode = msg->aim_mode ? 1 : 0;
            app.RecFireCode.Rotate = msg->rotate & 0b11;
        });

        // ly_gimbal_chassis
        GenSub<ly_gimbal_chassis>([](Application& app, auto msg) {
            app.gimbalYawVelDegPerSec = static_cast<float>(msg->angular_velocity);
            app.gimbalYawAngleDeg = static_cast<float>(msg->steer_angle);
            app.gimbalYawVelRaw = static_cast<std::int16_t>(std::lround(msg->angular_velocity * 100.0f));
            app.gimbalYawAngleRaw = static_cast<std::int16_t>(std::lround(msg->steer_angle * 100.0f));
        });

        // ly_gimbal_posture
        GenSubWithQoS<ly_gimbal_posture>(rclcpp::QoS(1), [](Application& app, auto msg) {
            app.postureState = msg->data;
            app.hasReceivedPostureState_ = true;
            app.lastPostureStateRxTime_ = std::chrono::steady_clock::now();
        });

        // ly_gimbal_capV
        GenSub<ly_gimbal_capV>([](Application& app, auto msg) {
            app.capV = msg->data;
        });

        // ly_game_event_data: semantic referee 0x0101 event_data topic.
        GenSub<ly_game_event_data>([](Application& app, auto msg) {
            const auto now = std::chrono::steady_clock::now();
            app.extEventData = msg->raw;
            app.eventCenterGainPointStatus_ = msg->center_gain_point_status;
            app.eventSelfSmallEnergyStatus_ = msg->self_small_energy_status;
            app.eventSelfLargeEnergyStatus_ = msg->self_large_energy_status;
            app.eventSelfFortressGainPointStatus_ = msg->self_fortress_gain_point_status;
            app.eventSelfOutpostGainPointStatus_ = msg->self_outpost_gain_point_status;
            app.eventSelfBaseGainPointStatus_ = msg->self_base_gain_point_status;
            app.hasReceivedEventData_ = true;
            app.lastEventDataRxTime_ = now;
            const auto& protect_castle = app.config.TacticalSettings.ProtectCastle;
            if (protect_castle.Enable && protect_castle.RFID) {
                ObserveCastleCaptureAttribution(
                    app.castleCaptureAttribution_,
                    app.eventSelfFortressGainPointStatus_,
                    app.selfFortressRfidActive_,
                    now,
                    std::chrono::milliseconds(std::max(
                        1, protect_castle.RfidCaptureTransitionWindowMs)));
            } else {
                app.castleCaptureAttribution_ = {};
            }
        });

        // ly_game_sentry_info: referee 0x020D sentry_info/sentry_info_2 semantic topic.
        GenSub<ly_game_sentry_info>([](Application& app, auto msg) {
            app.sentryCanActivateEnergyMechanism_ = msg->can_activate_energy_mechanism;
            app.postureRefereeTimer_.HasInfo3 = msg->has_sentry_info_3 &&
                msg->sentry_info_3_age_ms != std::numeric_limits<std::uint32_t>::max();
            app.postureRefereeTimer_.AgeMs = msg->sentry_info_3_age_ms;
            app.postureRefereeTimer_.AgeMeasuredAt = std::chrono::steady_clock::now();
            app.postureRefereeTimer_.Enhanced = msg->enhanced_posture;
            app.postureRefereeTimer_.RemainingSec = {
                0,
                msg->attack_posture_remaining_s,
                msg->defense_posture_remaining_s,
                msg->move_posture_remaining_s};
            app.postureRefereeTimer_.EnhancedRemainingSec = {
                0,
                msg->enhanced_attack_posture_remaining_s,
                msg->enhanced_defense_posture_remaining_s,
                msg->enhanced_move_posture_remaining_s};
            app.hasReceivedSentryInfo_ = true;
            app.lastSentryInfoRxTime_ = std::chrono::steady_clock::now();
        });

        // Preserve each receipt so the task policy can deduplicate 0x0303 repeats.
        GenSub<ly_game_map_command>([](Application& app, auto msg) {
            app.mapCommand = *msg;
            app.hasReceivedMapCommand_ = true;
            app.lastMapCommandRxTime_ = std::chrono::steady_clock::now();
            ++app.mapCommandRxSequence_;
        });

        // ly_friend_is_team_red
        GenSub<ly_friend_is_team_red>([](Application& app, auto msg) {
            app.team = msg->data ? UnitTeam::Red : UnitTeam::Blue;
        });

        // ly_game_all
        GenSub<ly_game_all>([](Application& app, auto msg) {
            // 0 HP 是裁判有效状态，必须保留以识别正常读条复活的 0 -> 正血转换。
            const auto now = std::chrono::steady_clock::now();
            if (IsNormalRespawnHealthTransition(
                    app.enhancedMoveHealthInitialized_,
                    app.enhancedMoveLastHealth_,
                    msg->selfhealth)) {
                const int suppress_sec = std::max(
                    0,
                    app.config.TacticalSettings.EnhancedPosture.RecoveryMove.RespawnSuppressSec);
                app.enhancedMoveRespawnSuppressUntil_ = now + std::chrono::seconds(suppress_sec);
                if (app.LoggerPtr) {
                    app.LoggerPtr->Info(
                        "Enhanced Recovery Move suppressed for {}s after normal respawn (hp={}).",
                        suppress_sec,
                        msg->selfhealth);
                }
            }
            app.enhancedMoveLastHealth_ = msg->selfhealth;
            app.enhancedMoveHealthInitialized_ = true;
            app.myselfHealth = msg->selfhealth;
            app.hasReceivedMyselfHealth_ = true;
            app.lastMyselfHealthRxTime = now;
        });

        // ly_enemy_op_hp
        GenSub<ly_enemy_op_hp>([](Application& app, auto msg) {
            app.enemyOutpostHealth = msg->data;
            app.hasReceivedEnemyOutpostHealth_ = true;
            app.lastEnemyOutpostHealthRxTime_ = std::chrono::steady_clock::now();
        });

        // ly_friend_op_hp
        GenSub<ly_friend_op_hp>([](Application& app, auto msg) {
            app.selfOutpostHealth = msg->data;
            app.hasReceivedSelfOutpostHealth_ = true;
            app.lastSelfOutpostHealthRxTime_ = std::chrono::steady_clock::now();
        });

        // ly_friend_base_hp
        GenSub<ly_friend_base_hp>([](Application& app, auto msg) {
            const auto now = std::chrono::steady_clock::now();
            app.selfBaseHealth = msg->data;
            app.hasReceivedSelfBaseHealth_ = true;
            app.lastSelfBaseHealthRxTime_ = now;
            ObserveBaseHealthForProtection(app.selfBaseDamageWindow_, msg->data, now);
        });

        // ly_enemy_base_hp
        GenSub<ly_enemy_base_hp>([](Application& app, auto msg) {
            app.enemyBaseHealth = msg->data;
        });

        // ly_friend_ammo_left
        GenSub<ly_friend_ammo_left>([](Application& app, auto msg) {
            app.ammoLeft = msg->data;
            app.hasReceivedAmmoLeft_ = true;
            app.lastAmmoLeftRxTime = std::chrono::steady_clock::now();
        });

        // ly_game_time_left
        GenSub<ly_game_time_left>([](Application& app, auto msg) {
            app.timeLeft = msg->data;
        });

        // ly_game_is_start
        GenSub<ly_game_is_start>([](Application& app, auto msg) {
            app.is_game_begin = msg->data;
            app.hasReceivedGameStartFlag_ = true;
            app.lastGameStartRxTime = std::chrono::steady_clock::now();
        });

        // ly_navi_vel
        GenSub<ly_navi_vel>([](Application& app, auto msg) {
            app.naviVelocityInput.X = msg->x;
            app.naviVelocityInput.Y = msg->y;
            app.lastNaviVelocityRxTime_ = std::chrono::steady_clock::now();
            // 兼容旧语义：保留原始转发链路变量
            app.naviVelocity.X = msg->x;
            app.naviVelocity.Y = msg->y;
        });

        // ly_navi_lower_head
        GenSub<ly_navi_lower_head>([](Application& app, auto msg) {
            app.naviLowerHead = msg->data;
        });

        // ly_navi_reached
        GenSub<ly_navi_reached>([](Application& app, auto msg) {
            const auto now = std::chrono::steady_clock::now();
            app.naviReach = msg->data;
            app.hasReceivedNaviReach_ = true;
            app.lastNaviReachRxTime_ = now;
        });

        // ly_navi_reachable
        GenSub<ly_navi_reachable>([](Application& app, auto msg) {
            const auto now = std::chrono::steady_clock::now();
            app.naviReachable = msg->data;
            app.hasReceivedNaviReachable_ = true;
            app.lastNaviReachableRxTime_ = now;
        });

        // ly_navi_should_rotate
        GenSub<ly_navi_should_rotate>([](Application& app, auto msg) {
            app.naviIsRotate = msg->data;
            app.hasReceivedNaviIsRotate_ = true;
            app.lastNaviIsRotateRxTime_ = std::chrono::steady_clock::now();
        });

        // ly_team_buff
        GenSub<ly_team_buff>([](Application& app, auto msg) {
            app.teamBuff.RecoveryBuff = msg->recoverybuff;
            app.teamBuff.CoolingBuff = msg->coolingbuff;
            app.teamBuff.DefenceBuff = msg->defencebuff;
            app.teamBuff.VulnerabilityBuff = msg->vulnerabilitybuff;
            app.teamBuff.AttackBuff = msg->attackbuff;
            app.teamBuff.RemainingEnergy = msg->remainingenergy;
        });

        // ly_game_rfid
        GenSub<ly_game_rfid>([](Application& app, auto msg) {
            app.rfidStatus = msg->raw;
            app.hasRfidStatus2 = msg->has_rfid_status_2;
            app.rfidStatus2 = msg->rfid_status_2_raw;
            app.rfidMatchState = BuildRfidMatchState(*msg);
            app.selfFortressRfidActive_ = app.rfidMatchState.SelfFortressGainPoint;
            if (!app.selfFortressRfidActive_) {
                app.castleCaptureAttribution_.SelfCaptureConfirmed = false;
            }
            app.hasReceivedRfidStatus_ = true;
            app.lastRfidStatusRxTime_ = std::chrono::steady_clock::now();
        });

        // ly_game_bullet
        // 先缓存 TypeID 7/8 合并后的弹速/发射事件/允许发弹量状态；当前决策仍使用旧 ammo/speed 输入。
        GenSub<ly_game_bullet>([](Application& app, auto msg) {
            app.bulletInfo = *msg;
            app.hasReceivedBulletInfo_ = true;
            app.lastBulletInfoRxTime_ = std::chrono::steady_clock::now();
        });

        // ly_navi_position
        // Navigation/TF-derived self position in official-map centimeters: [x, y].
        GenSub<ly_navi_position>([stamp_or_now](Application& app, auto msg) {
            const auto now = std::chrono::steady_clock::now();
            const auto stamp = stamp_or_now(app, msg->header);
            if (msg->data.size() < 2) {
                if (now - app.lastPositionDataGuardLogTime_ > std::chrono::seconds(2)) {
                    if (app.LoggerPtr) {
                        app.LoggerPtr->Warning(
                            "Ignore invalid /ly/navi/position: data size={} (need >=2)",
                            msg->data.size());
                    }
                    app.lastPositionDataGuardLogTime_ = now;
                }
                return;
            }
            app.sentryNaviPositionSource_.Valid = true;
            app.sentryNaviPositionSource_.X = static_cast<int>(msg->data[0]);
            app.sentryNaviPositionSource_.Y = static_cast<int>(msg->data[1]);
            app.sentryNaviPositionSource_.LastRx = now;
            app.sentryNaviPositionSource_.Stamp = stamp;
            app.UpdateSentryPositionFusion(now);
            app.PubFriendInfo();
        });

        // ly_friend_uwb_pos
        // Dedicated radar/UWB self position.
        GenSub<ly_friend_uwb_pos>([stamp_or_now](Application& app, auto msg) {
            const auto now = std::chrono::steady_clock::now();
            const auto stamp = stamp_or_now(app, msg->header);
            if (msg->data.size() < 2) {
                if (now - app.lastPositionDataGuardLogTime_ > std::chrono::seconds(2)) {
                    if (app.LoggerPtr) {
                        app.LoggerPtr->Warning(
                            "Ignore invalid /ly/friend/uwb_pos: data size={} (need >=2)",
                            msg->data.size());
                    }
                    app.lastPositionDataGuardLogTime_ = now;
                }
                return;
            }

            app.sentryUwbPositionSource_.Valid = true;
            app.sentryUwbPositionSource_.X = static_cast<int>(msg->data[0]);
            app.sentryUwbPositionSource_.Y = 1500 - static_cast<int>(msg->data[1]);
            app.sentryUwbPositionSource_.LastRx = now;
            app.sentryUwbPositionSource_.Stamp = stamp;
            app.lastSentryRadarPositionRxTime_ = now;
            app.UpdateSentryPositionFusion(now);
            app.PubFriendInfo();
        });

        // ly_position_data
        GenSub<ly_position_data>([stamp_or_now](Application& app, auto msg) {
            const auto stamp = stamp_or_now(app, msg->header);
            const int official_field_height_cm =
                app.config.LeagueStrategySettings.League3v3.Enable ? 800 : 1500;
            int FriendCarId = msg->friendcarid;
            auto in_range = [](const int idx) { return idx >= 0 && idx < 10; };
            auto maybe_warn_invalid_id = [&](const char* side, const int raw_id) {
                const auto now = std::chrono::steady_clock::now();
                if (now - app.lastPositionDataGuardLogTime_ > std::chrono::seconds(2)) {
                    app.LoggerPtr->Warning(
                        "Ignore invalid {} car id from /ly/position/data: {}",
                        side, raw_id);
                    app.lastPositionDataGuardLogTime_ = now;
                }
            };
            if (in_range(FriendCarId)) {
                const auto now = std::chrono::steady_clock::now();
                const auto friend_index = static_cast<std::size_t>(FriendCarId);
                const bool friend_is_sentry = FriendCarId == static_cast<int>(UnitType::Sentry);
                const bool friend_position_valid =
                    !IsRawPositionZero(msg->friendx, msg->friendy);
                if (!friend_position_valid) {
                    // 下位机/裁判系统未收到位置时会持续给 0,0；不要把它当真实场地点。
                } else if (friend_is_sentry) {
                    app.sentryPositionDataSource_.Valid = true;
                    app.sentryPositionDataSource_.X = static_cast<int>(msg->friendx);
                    app.sentryPositionDataSource_.Y = official_field_height_cm - static_cast<int>(msg->friendy);
                    app.sentryPositionDataSource_.LastRx = now;
                    app.sentryPositionDataSource_.Stamp = stamp;
                    app.UpdateSentryPositionFusion(now);
                } else {
                    app.friendRobots[FriendCarId].position_.X = msg->friendx;
                    app.friendRobots[FriendCarId].position_.Y = official_field_height_cm - msg->friendy;
                    app.lastFriendPositionRxTime_[friend_index] = now;
                    app.lastFriendPositionStamp_[friend_index].Stamp = stamp;
                }
            } else {
                maybe_warn_invalid_id("friend", FriendCarId);
            }
            int EnemyCarId = msg->enemycarid;
            EnemyCarId = EnemyCarId % 100;
            if (in_range(EnemyCarId)) {
                if (!IsRawPositionZero(msg->enemyx, msg->enemyy)) {
                    app.enemyRobots[EnemyCarId].position_.X = msg->enemyx;
                    app.enemyRobots[EnemyCarId].position_.Y = official_field_height_cm - msg->enemyy;
                    const auto enemy_index = static_cast<std::size_t>(EnemyCarId);
                    app.lastEnemyPositionRxTime_[enemy_index] = std::chrono::steady_clock::now();
                    app.lastEnemyPositionStamp_[enemy_index].Stamp = stamp;
                    app.lastEnemyPositionSource_[enemy_index] = "position_data";
                }
            } else {
                maybe_warn_invalid_id("enemy", EnemyCarId);
            }
            app.PubFriendInfo();
            app.PubEnemyInfo();
        });

        // ly_navi_target_official
        // navi_tf_bridge 将当前追击相对目标 TF 到 map 后，再用同一套矩阵反算为 official-map cm。
        // 只作为敌方位置 fallback；新鲜非 0 的 /ly/position/data 优先。
        GenSub<ly_navi_target_official>([stamp_or_now](Application& app, auto msg) {
            const auto stamp = stamp_or_now(app, msg->header);
            const auto now = std::chrono::steady_clock::now();
            if (msg->data.size() < 3) {
                if (now - app.lastPositionDataGuardLogTime_ > std::chrono::seconds(2)) {
                    app.LoggerPtr->Warning(
                        "Ignore invalid /ly/navi/target_official: data size={} (need >=3)",
                        msg->data.size());
                    app.lastPositionDataGuardLogTime_ = now;
                }
                return;
            }

            const int x = static_cast<int>(msg->data[0]);
            const int y = static_cast<int>(msg->data[1]);
            if (!IsOfficialFieldPointValid(x, y)) {
                return;
            }

            const auto maybe_unit = UnitTypeFromArmorTypeId(msg->data[2]);
            if (!maybe_unit.has_value()) {
                return;
            }

            const auto index = static_cast<std::size_t>(*maybe_unit);
            if (index >= app.lastEnemyPositionRxTime_.size()) {
                return;
            }

            const auto& last_rx = app.lastEnemyPositionRxTime_[index];
            const bool has_fresh_position_data =
                app.lastEnemyPositionSource_[index] == "position_data" &&
                last_rx.time_since_epoch().count() != 0 &&
                now - last_rx <= std::chrono::milliseconds(
                    std::max(1, app.config.ChaseSettings.OfficialPositionFreshMs));
            if (!ShouldAcceptNaviTargetOfficialFallback(
                    app.config.ChaseSettings,
                    has_fresh_position_data)) {
                return;
            }

            app.enemyRobots[*maybe_unit].position_.X = static_cast<std::int16_t>(x);
            app.enemyRobots[*maybe_unit].position_.Y = static_cast<std::int16_t>(y);
            app.lastEnemyPositionRxTime_[index] = now;
            app.lastEnemyPositionStamp_[index].Stamp = stamp;
            app.lastEnemyPositionSource_[index] = "navi_target_official";
            app.PubEnemyInfo();
        });

#ifdef LY_ENABLE_SENTRY_MSGS
        // ly_aim_armor_targets: external sentry.aim target candidates.
        GenSubWithQoS<ly_aim_armor_targets>(rclcpp::SensorDataQoS(), [](Application& app, auto msg) {
            if (!app.config.ExternalAimSettings.Enable) {
                return;
            }
            const auto now = std::chrono::steady_clock::now();
            std::fill(app.externalAimTargets_.begin(),
                      app.externalAimTargets_.end(),
                      ExternalAimTargetCache{});
            if (app.config.ExternalAimSettings.UseTargetArrayAsArmorList) {
                std::fill(app.armorList.begin(), app.armorList.end(), ArmorData{ArmorType::UnKnown, 30});
            }

            std::size_t armor_index = 0;
            for (const auto& target : msg->aim_targets) {
                const auto target_id = static_cast<std::size_t>(target.id);
                if (target_id >= app.externalAimTargets_.size()) {
                    continue;
                }
                const auto x = static_cast<float>(target.position.x);
                const auto y = static_cast<float>(target.position.y);
                const auto z = static_cast<float>(target.position.z);
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                    continue;
                }
                std::string frame_id =
                    !target.header.frame_id.empty()
                        ? target.header.frame_id
                        : msg->header.frame_id;
                if (frame_id.empty()) {
                    frame_id = app.config.ExternalAimSettings.TargetDefaultFrame;
                }
                const float distance = std::hypot(x, y, z);
                app.externalAimTargets_[target_id] = ExternalAimTargetCache{
                    .Valid = true,
                    .X = x,
                    .Y = y,
                    .Z = z,
                    .Distance = std::isfinite(distance) && distance > 0.0f ? distance : 30.0f,
                    .FrameId = frame_id,
                    .LastSeen = now
                };
                if (app.config.ExternalAimSettings.UseTargetArrayAsArmorList &&
                    armor_index < app.armorList.size()) {
                    app.armorList[armor_index++] = ArmorData{
                        static_cast<ArmorType>(target.id),
                        app.externalAimTargets_[target_id].Distance
                    };
                }
            }
            app.hasExternalAimTargets_ = true;
            app.lastExternalAimTargetsRxTime_ = now;
        });

        // ly_aim_result: external sentry.aim final angle, dynamics, and fire gate.
        GenSub<ly_aim_result>([](Application& app, auto msg) {
            if (!app.config.ExternalAimSettings.Enable) {
                return;
            }
            const auto yaw = static_cast<AngleType>(msg->yaw);
            const auto pitch = static_cast<AngleType>(msg->pitch);
            const auto now = std::chrono::steady_clock::now();
            if constexpr (!(requires {
                msg->yaw_omega;
                msg->pitch_omega;
                msg->yaw_alpha;
                msg->pitch_alpha;
            })) {
                // Offline builds may use the legacy AimResult. It has no
                // dynamics, so it can never authorize trajectory output.
                app.externalAimData.Angles = GimbalAnglesType{yaw, pitch};
                app.externalAimData.YawOmega = 0.0F;
                app.externalAimData.PitchOmega = 0.0F;
                app.externalAimData.YawAlpha = 0.0F;
                app.externalAimData.PitchAlpha = 0.0F;
                app.externalAimData.BuffFollow = false;
                app.externalAimData.FireStatus = false;
                app.externalAimData.Valid = false;
                app.externalAimData.Fresh = false;
                app.lastExternalAimResultRxTime_ = now;
                return;
            } else {

                const auto yaw_omega = static_cast<float>(msg->yaw_omega);
                const auto pitch_omega = static_cast<float>(msg->pitch_omega);
                const auto yaw_alpha = static_cast<float>(msg->yaw_alpha);
                const auto pitch_alpha = static_cast<float>(msg->pitch_alpha);
                const bool finite_result =
                    std::isfinite(yaw) && std::isfinite(pitch) &&
                    std::isfinite(yaw_omega) && std::isfinite(pitch_omega) &&
                    std::isfinite(yaw_alpha) && std::isfinite(pitch_alpha);
                bool target_context_valid = true;
                if (app.config.ExternalAimSettings.UseTargetArrayAsArmorList) {
                    target_context_valid = false;
                    const auto target_id = static_cast<std::size_t>(app.targetArmor.Type);
                    const int fresh_ms = std::max(1, app.config.ExternalAimSettings.TargetFreshTimeoutMs);
                    if (target_id < app.externalAimTargets_.size()) {
                        const auto& cached = app.externalAimTargets_[target_id];
                        target_context_valid =
                            cached.Valid &&
                            cached.LastSeen.time_since_epoch().count() != 0 &&
                            now - cached.LastSeen <= std::chrono::milliseconds(fresh_ms);
                    }
                }
                const bool result_valid = msg->follow && finite_result && target_context_valid;
                app.externalAimData.Angles = GimbalAnglesType{yaw, pitch};
                app.externalAimData.YawOmega = yaw_omega;
                app.externalAimData.PitchOmega = pitch_omega;
                app.externalAimData.YawAlpha = yaw_alpha;
                app.externalAimData.PitchAlpha = pitch_alpha;
                app.externalAimData.BuffFollow = false;
                app.externalAimData.FireStatus = result_valid && msg->fire;
                app.externalAimData.Valid = result_valid;
                app.externalAimData.Fresh = result_valid;
                app.lastExternalAimResultRxTime_ = now;
                if (result_valid) {
                    app.externalAimData.HasLatchedAngles = true;
                    app.externalAimData.LastValidTime = now;
                    app.isFindTargetAtomic = true;
                    app.lastTargetSeenTime = now;
                } else {
                    app.externalAimData.FireStatus = false;
                    app.externalAimData.HasLatchedAngles = false;
                }
            }
        });
#endif

        // ly_face_mode_angles
        GenSub<ly_face_mode_angles>([](Application& app, auto msg) {
            app.faceModeManager_.CacheAngles(
                app.faceModeData,
                msg->yaw,
                msg->pitch,
                std::chrono::steady_clock::now());
        });

        // FaceMode angles do not identify the target. StartGate accepts them only
        // when this solver status confirms the requested fixed outpost target.
        GenSub<ly_gimbal_face_mode_status>([](Application& app, auto msg) {
            app.faceModeSolverStatus.Received = true;
            app.faceModeSolverStatus.Function = msg->function;
            app.faceModeSolverStatus.ManualTarget = msg->manual_target;
            app.faceModeSolverStatus.TargetUpdateCount = msg->target_update_count;
            app.faceModeSolverStatus.LastRx = std::chrono::steady_clock::now();
        });

        // ly_enemy_hp
        GenSub<ly_enemy_hp>([stamp_or_now](Application& app, auto msg) {
            const auto now = std::chrono::steady_clock::now();
            const auto stamp = stamp_or_now(app, msg->header);
            const int respawn_transition_timeout_ms =
                std::max(0, app.config.DecisionAutonomySettings.AimTarget.RespawnTransitionTimeoutMs);
            const auto update_health = [&](const UnitType unit, const std::uint16_t health) {
                if (health == 0) {
                    return false;
                }
                const auto index = static_cast<std::size_t>(unit);
                app.lastEnemyHealthRxTime_[index] = now;
                app.lastEnemyHealthStamp_[index].Stamp = stamp;

                const auto last_confirmed_dead = app.lastEnemyConfirmedDeadTime_[index];
                const bool confirmed_dead_recent =
                    app.enemyHealthConfirmedDead_[index] &&
                    last_confirmed_dead.time_since_epoch().count() != 0 &&
                    (respawn_transition_timeout_ms == 0 ||
                     now - last_confirmed_dead <=
                         std::chrono::milliseconds(respawn_transition_timeout_ms));
                app.enemyZeroHealthObserved_[index] = false;
                app.enemyZeroHealthSince_[index] = {};
                app.enemyHealthConfirmedDead_[index] = false;
                if (confirmed_dead_recent) {
                    app.enemyRobots[unit].markInvulnerableStartNow();
                }
                app.enemyRobots[unit].setCurrentHealth(health, false);
                return true;
            };
            bool updated = false;
            updated |= update_health(UnitType::Hero, static_cast<std::uint16_t>(msg->hero));
            updated |= update_health(UnitType::Engineer, static_cast<std::uint16_t>(msg->engineer));
            updated |= update_health(UnitType::Infantry1, static_cast<std::uint16_t>(msg->infantry1));
            updated |= update_health(UnitType::Infantry2, static_cast<std::uint16_t>(msg->infantry2));
            updated |= update_health(UnitType::Sentry, static_cast<std::uint16_t>(msg->sentry));
            if (updated) {
                app.hasReceivedEnemyHealth_ = true;
                app.PubEnemyInfo();
            }
        });

        // ly_friend_hp
        GenSub<ly_friend_hp>([stamp_or_now](Application& app, auto msg) {
            const auto now = std::chrono::steady_clock::now();
            const auto stamp = stamp_or_now(app, msg->header);
            const auto update_health = [&](const UnitType unit, const std::uint16_t health) {
                if (health == 0) {
                    return false;
                }
                const auto index = static_cast<std::size_t>(unit);
                app.friendRobots[unit].setCurrentHealth(health);
                if (index < app.lastFriendHealthRxTime_.size()) {
                    app.lastFriendHealthRxTime_[index] = now;
                    app.lastFriendHealthStamp_[index].Stamp = stamp;
                }
                return true;
            };
            bool updated = false;
            updated |= update_health(UnitType::Hero, static_cast<std::uint16_t>(msg->hero));
            updated |= update_health(UnitType::Engineer, static_cast<std::uint16_t>(msg->engineer));
            updated |= update_health(UnitType::Infantry1, static_cast<std::uint16_t>(msg->infantry1));
            updated |= update_health(UnitType::Infantry2, static_cast<std::uint16_t>(msg->infantry2));
            updated |= update_health(UnitType::Sentry, static_cast<std::uint16_t>(msg->sentry));
            if (updated) {
                app.PubFriendInfo();
            }
        });
    }    

}
