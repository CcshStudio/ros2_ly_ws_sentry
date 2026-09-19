// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

#include "../include/Application.hpp"
#include <algorithm>
#include <cmath>
#include <optional>

using namespace LangYa;

namespace BehaviorTree {
    namespace {
    constexpr float kGatePatrolTwoPi = 6.2831853071795864769f;

    struct GateFaceModeLogState {
        bool TargetPublish{false};
        bool Active{false};
        bool Fallback{false};
        bool StatusReceived{false};
        bool StatusFresh{false};
        bool Function{false};
        bool ManualTarget{false};
        bool TargetAdvanced{false};
        bool AnglesUsable{false};
        bool AnglesAvailable{false};
        FaceModeManager::StartGateOutpostReadiness Readiness{
            FaceModeManager::StartGateOutpostReadiness::StatusMissing};

        bool operator==(const GateFaceModeLogState&) const = default;
    };

    float NormalizeGatePatrolAngleNear(const float angle, const float reference) {
        return reference + static_cast<float>(std::remainder(angle - reference, 360.0f));
    }
    }  // namespace

    /**
     * @brief 等待比赛开始前的预操作 \n
     * @brief 1. 等待云台数据 \n
     * @brief 2. 等待比赛开始 \n
     * @brief 3. 设置云台角度 \n
     * @brief 4. 设置导航目标 \n
     * @brief 5. 设置底盘速度 \n
     */
    void Application::WaitForGameStart() {
        /// 设置云台角度
        gimbalControlData.GimbalAngles.Yaw = gimbalAngles.Yaw;
        gimbalControlData.GimbalAngles.Pitch = AngleType{0};
        gimbalControlData.FireCode.FireStatus = 0;
        gimbalControlData.FireCode.Rotate = 0;
        gimbalControlData.FireCode.FollowMode = 0;
        gimbalControlData.FireCode.AimMode = 0;
        naviVelocityInput = VelocityType{0, 0};
        naviVelocity = VelocityType{0, 0};
        postureCommand = 0;

        LoggerPtr->Info("Waiting For Game Start!");

        const auto wait_begin = std::chrono::steady_clock::now();
        auto last_wait_log = wait_begin;
        auto last_damage_gate_log = wait_begin;
        auto last_gate_patrol_log = wait_begin;
        bool gate_patrol_center_initialized = false;
        PassiveGimbalMotion gate_passive_gimbal_motion;
        float gate_patrol_center_yaw = 0.0f;
        float gate_patrol_last_yaw = 0.0f;
        float gate_patrol_phase_rad = 0.0f;
        bool gate_face_mode_target_generation_initialized = false;
        std::uint32_t gate_face_mode_target_generation_floor = 0;
        std::optional<GateFaceModeLogState> last_gate_face_mode_log_state;
        bool gate_face_mode_disabled_logged = false;
        bool bypass_logged = false;
        const bool gate_gimbal_patrol_enabled =
            config.StartGateSettings.AllowGimbalPatrolBeforeStart &&
            !config.AimDebugSettings.StopScan;
        const bool gate_face_mode_outpost_enabled =
            config.StartGateSettings.GimbalStrategy == "face_mode_outpost" &&
            !config.AimDebugSettings.StopScan &&
            !debugBypassGameStart_;
        const bool gate_patrol_strategy =
            config.StartGateSettings.GimbalStrategy == "patrol";
        const bool damage_open_gate_enabled = config.DamageOpenGateSettings.Enable;
        const std::uint16_t damage_open_gate_threshold =
            std::max<std::uint16_t>(1, config.DamageOpenGateSettings.HealthDropThreshold);
        bool damage_health_baseline_initialized = false;
        std::uint16_t damage_health_peak = 0;

        const auto set_start_hold_position = [&]() {
            if (config.LeagueStrategySettings.League3v3.Enable) {
                const auto goal = League3v3Goal::SelfBase;
                naviCommandGoal = ResolveGoalId(static_cast<std::uint8_t>(goal), team, true);
                const auto point = League3v3GoalPoint(goal, team);
                naviGoalPosition = Area::Point<std::uint16_t>{point.X, point.Y};
                naviGoalPublishAllowed_ = true;
                RecordDecisionIntent(MakeDecisionIntent(
                    DecisionReason::League3v3,
                    static_cast<std::uint8_t>(goal),
                    team,
                    true,
                    "prestart_hold_start"));
                return;
            }
            SET_POSITION(Home, team);
        };

        if (damage_open_gate_enabled) {
            LoggerPtr->Info(
                "Damage start gate enabled: open by health drop >= {}.",
                static_cast<int>(damage_open_gate_threshold));
        }
        if (gate_gimbal_patrol_enabled) {
            LoggerPtr->Info(
                "Start gate gimbal patrol enabled: chassis velocity/rotate stay zero before game start.");
        }
        if (gate_face_mode_outpost_enabled) {
            LoggerPtr->Info(
                "Start gate FaceMode enabled: face enemy outpost; stale/missing angles fall back to outpost patrol.");
        }

        // [ROS 2] 不再依賴文件系統判斷，直接等待 is_game_begin 標誌
        while (rclcpp::ok()) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds{10});
            const auto now_steady = std::chrono::steady_clock::now();

            set_start_hold_position();

            if(publishNaviGoal_ && naviCommandRateClock.trigger()) {
                naviCommandRateClock.tick();
                const bool navi_to_navi =
                    config.NaviSettings.UseXY &&
                    config.NaviSettings.ToNavi &&
                    config.ChaseSettings.Enable &&
                    config.ChaseSettings.ToNavi;
                if(config.NaviSettings.UseXY && !navi_to_navi) PubNaviGoalPos();
                else PubNaviGoal();
            }

            // 开赛门控期间持续压零速度，避免下位机沿用上一拍底盘控制量。
            naviVelocityInput.X = 0;
            naviVelocityInput.Y = 0;
            naviVelocity.X = 0;
            naviVelocity.Y = 0;
            gimbalControlData.FireCode.FireStatus = 0;
            gimbalControlData.FireCode.Rotate = 0;
            gimbalControlData.FireCode.FollowMode = 0;
            gimbalControlData.FireCode.AimMode = 0;
            bool start_gate_face_mode_active = false;
            bool start_gate_face_mode_fallback = false;
            if (gate_face_mode_outpost_enabled) {
                if (config.FaceModeSettings.Enable) {
                    if (!gate_face_mode_target_generation_initialized) {
                        gate_face_mode_target_generation_floor =
                            faceModeSolverStatus.Received
                                ? faceModeSolverStatus.TargetUpdateCount
                                : 0;
                        gate_face_mode_target_generation_initialized = true;
                    }
                    faceModeManager_.BeginCycle();
                    const auto enemy_team =
                        team == UnitTeam::Blue ? UnitTeam::Red : UnitTeam::Blue;
                    const bool face_mode_target_published = faceModeManager_.RequestStartGateOutpost(
                        enemy_team,
                        pub_face_mode_target_raw_);
                    const auto face_mode_decision = faceModeManager_.Resolve(
                        faceModeData,
                        config.FaceModeSettings,
                        config.PatrolScanSettings,
                        false,
                        false,
                        false,
                        now_steady);
                    lastFaceModeDecision_ = face_mode_decision;
                    const bool status_fresh =
                        faceModeSolverStatus.Received &&
                        faceModeSolverStatus.LastRx.time_since_epoch().count() != 0 &&
                        now_steady - faceModeSolverStatus.LastRx <=
                            std::chrono::milliseconds(
                                config.StartGateSettings.FaceModeStatusFreshMs);
                    const bool face_mode_angles_usable =
                        FaceModeManager::AnglesUsable(
                            faceModeData,
                            config.FaceModeSettings,
                            now_steady);
                    const auto solver_readiness =
                        FaceModeManager::DiagnoseStartGateOutpostSolution(
                            faceModeSolverStatus.Received,
                            status_fresh,
                            faceModeSolverStatus.Function,
                            faceModeSolverStatus.ManualTarget,
                            faceModeSolverStatus.TargetUpdateCount,
                            gate_face_mode_target_generation_floor,
                            face_mode_angles_usable);
                    const bool solver_accepts_start_gate_target =
                        solver_readiness == FaceModeManager::StartGateOutpostReadiness::Ready;
                    // StartGate face_mode_outpost has an explicit safety contract: if
                    // there is no usable fixed-target angle, always scan the configured
                    // outpost fallback. It intentionally does not inherit the generic
                    // task fallback-disable switch.
                    start_gate_face_mode_active =
                        face_mode_decision.Angles.has_value() && solver_accepts_start_gate_target;
                    start_gate_face_mode_fallback = !start_gate_face_mode_active;
                    const char* outcome = start_gate_face_mode_active
                        ? "accepted"
                        : (solver_accepts_start_gate_target
                            ? "angles_unavailable"
                            : FaceModeManager::StartGateOutpostReadinessName(solver_readiness));
                    const GateFaceModeLogState log_state{
                        face_mode_target_published,
                        start_gate_face_mode_active,
                        start_gate_face_mode_fallback,
                        faceModeSolverStatus.Received,
                        status_fresh,
                        faceModeSolverStatus.Function,
                        faceModeSolverStatus.ManualTarget,
                        faceModeSolverStatus.TargetUpdateCount > gate_face_mode_target_generation_floor,
                        face_mode_angles_usable,
                        face_mode_decision.Angles.has_value(),
                        solver_readiness};
                    if (!last_gate_face_mode_log_state ||
                        log_state != *last_gate_face_mode_log_state) {
                        LoggerPtr->Info(
                            "StartGate FaceMode: source=start_gate requested=1 target_publish={} active={} "
                            "fallback=patrol_mode_{} outcome={} status(received={} fresh={} function={} "
                            "manual={} target_updates={} floor={}) angles(usable={} available={}).",
                            face_mode_target_published ? 1 : 0,
                            start_gate_face_mode_active ? 1 : 0,
                            config.PatrolScanSettings.OutpostFaceModeFallbackMode,
                            outcome,
                            faceModeSolverStatus.Received ? 1 : 0,
                            status_fresh ? 1 : 0,
                            faceModeSolverStatus.Function ? 1 : 0,
                            faceModeSolverStatus.ManualTarget ? 1 : 0,
                            faceModeSolverStatus.TargetUpdateCount,
                            gate_face_mode_target_generation_floor,
                            face_mode_angles_usable ? 1 : 0,
                            face_mode_decision.Angles.has_value() ? 1 : 0);
                        last_gate_face_mode_log_state = log_state;
                    }
                    if (start_gate_face_mode_active) {
                        gimbalControlData.GimbalAngles =
                            face_mode_decision.Angles.value_or(gimbalAngles);
                    }
                } else {
                    // A selected FaceMode strategy without its solver enabled must not leave
                    // the opening gimbal idle; use the configured patrol fallback instead.
                    start_gate_face_mode_fallback = true;
                    if (!gate_face_mode_disabled_logged) {
                        LoggerPtr->Info(
                            "StartGate FaceMode: source=start_gate requested=0 active=0 "
                            "fallback=patrol_mode_{} outcome=face_mode_disabled.",
                            config.PatrolScanSettings.OutpostFaceModeFallbackMode);
                        gate_face_mode_disabled_logged = true;
                    }
                }
                // CacheAngles marks a callback fresh for one control cycle. LastValidTime
                // remains available for the configured LostTargetHoldMs window.
                faceModeData.Fresh = false;
            }

            const bool run_start_gate_patrol =
                !debugBypassGameStart_ &&
                ((gate_patrol_strategy && gate_gimbal_patrol_enabled) ||
                 (gate_face_mode_outpost_enabled && start_gate_face_mode_fallback));
            if (run_start_gate_patrol) {
                if (!gate_patrol_center_initialized) {
                    gate_patrol_center_initialized = true;
                    gate_patrol_center_yaw = gimbalAngles.Yaw;
                    gate_patrol_last_yaw = gimbalAngles.Yaw;
                }

                const int patrol_mode = start_gate_face_mode_fallback
                    ? config.PatrolScanSettings.OutpostFaceModeFallbackMode
                    : config.PatrolScanSettings.Mode;
                const auto& patrol_scan = config.PatrolScanSettings;
                float next_yaw = gate_patrol_last_yaw;
                if (patrol_mode == 2) {
                    const float half_range = static_cast<float>(patrol_scan.Mode2YawHalfRangeDeg);
                    const float phase_step =
                        static_cast<float>(patrol_scan.Mode2YawStepDegPerTick) /
                        std::max(half_range, 1.0f);
                    gate_patrol_phase_rad = std::fmod(gate_patrol_phase_rad + phase_step, kGatePatrolTwoPi);
                    if (gate_patrol_phase_rad < 0.0f) {
                        gate_patrol_phase_rad += kGatePatrolTwoPi;
                    }
                    next_yaw = NormalizeGatePatrolAngleNear(
                        gate_patrol_center_yaw +
                            half_range * std::sin(gate_patrol_phase_rad),
                        gimbalAngles.Yaw);
                } else {
                    const float yaw_step =
                        patrol_mode == 3
                            ? static_cast<float>(patrol_scan.Mode3YawStepDegPerTick)
                            : static_cast<float>(patrol_scan.Mode1YawStepDegPerTick);
                    next_yaw = NormalizeGatePatrolAngleNear(
                        gate_patrol_last_yaw + yaw_step,
                        gimbalAngles.Yaw);
                }
                gate_patrol_last_yaw = next_yaw;

                const float pitch_elapsed_ms = static_cast<float>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        now_steady - wait_begin).count());
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
                const float opening_pitch_offset =
                    (patrol_mode != 3 || patrol_scan.StartGatePitchOffsetApplyToMode3)
                        ? static_cast<float>(patrol_scan.StartGatePitchOffsetDeg)
                        : 0.0f;
                const float next_pitch =
                    pitch_center +
                    opening_pitch_offset +
                    pitch_half_range *
                        std::sin(
                            pitch_elapsed_ms * kGatePatrolTwoPi /
                            std::max(pitch_period_ms, 1.0f));
                gimbalControlData.GimbalAngles = GimbalAnglesType{
                    static_cast<AngleType>(next_yaw),
                    static_cast<AngleType>(next_pitch)};

                if (now_steady - last_gate_patrol_log > std::chrono::seconds(2)) {
                    LoggerPtr->Debug(
                        "Start gate gimbal patrol: mode={} yaw={} pitch={}.",
                        patrol_mode,
                        gimbalControlData.GimbalAngles.Yaw,
                        gimbalControlData.GimbalAngles.Pitch);
                    last_gate_patrol_log = now_steady;
                }
            } else if (!start_gate_face_mode_active) {
                gimbalControlData.GimbalAngles.Yaw = gimbalAngles.Yaw;
                gimbalControlData.GimbalAngles.Pitch = AngleType{0};
            }
            if (start_gate_face_mode_active || run_start_gate_patrol) {
                gimbalControlData.GimbalAngles = gate_passive_gimbal_motion.Step(
                    gimbalAngles,
                    gimbalControlData.GimbalAngles,
                    now_steady,
                    config.PatrolScanSettings.PassiveYawRateDegPerSec,
                    config.PatrolScanSettings.PassivePitchRateDegPerSec,
                    config.PatrolScanSettings.PassiveMaxIntervalMs);
            } else {
                gate_passive_gimbal_motion.Reset();
            }
            PubNaviControlData();
            PubGimbalControlData();

            if (debugBypassGameStart_) {
                if (!bypass_logged) {
                    LoggerPtr->Warning(
                        "debug_bypass_is_start=true, skip waiting for /ly/game/is_start.");
                    bypass_logged = true;
                }
                break;
            }

            if (waitForGameStartTimeoutSec_ > 0 &&
                (now_steady - wait_begin) > std::chrono::seconds(waitForGameStartTimeoutSec_)) {
                LoggerPtr->Warning(
                    "WaitForGameStart timeout after {}s, continue without is_start gate.",
                    waitForGameStartTimeoutSec_);
                break;
            }

            if (now_steady - last_wait_log > std::chrono::seconds(2)) {
                if (!hasReceivedGameStartFlag_) {
                    LoggerPtr->Warning("Waiting /ly/game/is_start message...");
                } else {
                    LoggerPtr->Debug("Waiting /ly/game/is_start=true...");
                }
                last_wait_log = now_steady;
            }

            if (is_game_begin) {
                LoggerPtr->Info("!!!!Game !! start!!!!");
                break;
            }

            if (damage_open_gate_enabled) {
                const auto is_health_input_ready = [&]() -> bool {
                    if (!hasReceivedMyselfHealth_) {
                        return false;
                    }
                    if (lastMyselfHealthRxTime.time_since_epoch().count() == 0) {
                        return false;
                    }
                    if (leagueRefereeStaleTimeoutMs_ <= 0) {
                        return true;
                    }
                    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now_steady - lastMyselfHealthRxTime).count();
                    return age_ms <= leagueRefereeStaleTimeoutMs_;
                };

                if (is_health_input_ready()) {
                    if (!damage_health_baseline_initialized) {
                        damage_health_peak = myselfHealth;
                        damage_health_baseline_initialized = true;
                        LoggerPtr->Info(
                            "Damage gate baseline health captured: {}.",
                            static_cast<int>(damage_health_peak));
                    } else {
                        if (myselfHealth > damage_health_peak) {
                            damage_health_peak = myselfHealth;
                        }
                        const std::uint16_t health_drop = damage_health_peak > myselfHealth
                            ? static_cast<std::uint16_t>(damage_health_peak - myselfHealth)
                            : 0;
                        if (health_drop >= damage_open_gate_threshold) {
                            LoggerPtr->Warning(
                                "Damage gate opened by health drop: peak={} current={} drop={} threshold={}.",
                                static_cast<int>(damage_health_peak),
                                static_cast<int>(myselfHealth),
                                static_cast<int>(health_drop),
                                static_cast<int>(damage_open_gate_threshold));
                            break;
                        }
                    }
                } else if (now_steady - last_damage_gate_log > std::chrono::seconds(2)) {
                    LoggerPtr->Warning(
                        "Damage gate waiting for fresh /ly/game/all.selfhealth (stale_timeout_ms={}).",
                        leagueRefereeStaleTimeoutMs_);
                    last_damage_gate_log = now_steady;
                }
            }
        }
        LoggerPtr->Info("Stop Waiting For Game");
    }

    void Application::WaitBeforeGame() {
        LoggerPtr->Info("Waiting Before Game");
        PublishSafeControl("wait_before_game_reset");

        /// 取得第一个云台数据包（不能用 yaw/pitch 非 0 判定，0 也是合法角度）
        const auto wait_begin = std::chrono::steady_clock::now();
        auto last_wait_log = wait_begin;
        constexpr auto kMaxWait = std::chrono::seconds(10);
        while (rclcpp::ok()) {
            rclcpp::spin_some(node_);

            if (hasReceivedGimbalAngles_.load()) {
                LoggerPtr->Info("First gimbal message received.");
                break;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - wait_begin > kMaxWait) {
                LoggerPtr->Warning("No gimbal message within {}s, continue with current angles (Yaw={}, Pitch={}).",
                    std::chrono::duration_cast<std::chrono::seconds>(kMaxWait).count(),
                    gimbalAngles.Yaw, gimbalAngles.Pitch);
                break;
            }
            if (now - last_wait_log > std::chrono::seconds(2)) {
                LoggerPtr->Warning("Waiting for first gimbal message...");
                last_wait_log = now;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        LoggerPtr->Info("Stop waiting for first gimbal data.");
        
        WaitForGameStart();

        LoggerPtr->Info("Stop Waiting Before Game");
    }
}
