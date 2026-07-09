// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

/*
 * gimbal_driver 主节点
 *
 * 核心职责：
 * 1) 订阅 /ly/control/*（上位机控制指令）
 * 2) 与下位机串口设备交互（真实设备或虚拟设备）
 * 3) 发布 /ly/gimbal/* 与 /ly/game/*（回传状态）
 *
 * 备注：
 * - /ly/control/sentry_cmd 为完整裁判命令输入，/ly/control/posture 为姿态命令输入。
 * - /ly/gimbal/posture 发布下位机/裁判姿态回读状态，不镜像上位机命令。
 */
#include <chrono>
#include <thread>
#include <algorithm>
#include <array>
#include <bit>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <mutex>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/executors.hpp>
#include <sstream>

#include "gimbal_driver/msg/gimbal_angles.hpp"
#include "gimbal_driver/msg/chassis.hpp"
#include "gimbal_driver/msg/control_velocity.hpp"
#include "gimbal_driver/msg/event_data.hpp"
#include "gimbal_driver/msg/fire_code.hpp"
#include "gimbal_driver/msg/rfid_status.hpp"
#include "gimbal_driver/msg/gimbal_raw_frame.hpp"
#include "gimbal_driver/msg/uwb_pos.hpp"
#include "gimbal_driver/msg/vel.hpp"
#include "gimbal_driver/msg/health.hpp"
#include "gimbal_driver/msg/game_data.hpp"
#include "gimbal_driver/msg/map_command.hpp"
#include "gimbal_driver/msg/buff_data.hpp"
#include "gimbal_driver/msg/bullet_info.hpp"
#include "gimbal_driver/msg/position_data.hpp"
#include "gimbal_driver/msg/sentry_cmd.hpp"
#include "gimbal_driver/msg/sentry_info.hpp"
#include "gimbal_driver/msg/stamped_u_int16_multi_array.hpp"

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "module/BasicTypes.hpp"
#include "module/DownlinkFrame.hpp"
#include "module/IODevice.hpp"
#include "module/ROSTools.hpp"

using namespace LangYa;

namespace
{
    LY_DEF_ROS_TOPIC(ly_control_angles, "/ly/control/angles", gimbal_driver::msg::GimbalAngles);
    LY_DEF_ROS_TOPIC(ly_control_firecode, "/ly/control/firecode", gimbal_driver::msg::FireCode);
    LY_DEF_ROS_TOPIC(ly_control_vel, "/ly/control/vel", gimbal_driver::msg::ControlVelocity);
    LY_DEF_ROS_TOPIC(ly_control_posture, "/ly/control/posture", gimbal_driver::msg::SentryCmd);
    LY_DEF_ROS_TOPIC(ly_control_sentry_cmd, "/ly/control/sentry_cmd", gimbal_driver::msg::SentryCmd);

    LY_DEF_ROS_TOPIC(ly_gimbal_angles, "/ly/gimbal/angles", gimbal_driver::msg::GimbalAngles);
    LY_DEF_ROS_TOPIC(ly_gimbal_firecode, "/ly/gimbal/firecode", gimbal_driver::msg::FireCode);
    LY_DEF_ROS_TOPIC(ly_gimbal_vel, "/ly/gimbal/vel", gimbal_driver::msg::Vel);
    LY_DEF_ROS_TOPIC(ly_gimbal_chassis, "/ly/gimbal/chassis", gimbal_driver::msg::Chassis);
    LY_DEF_ROS_TOPIC(ly_gimbal_big_yaw_angles, "/ly/gimbal/big_yaw_angles", std_msgs::msg::Float32);
    LY_DEF_ROS_TOPIC(ly_gimbal_posture, "/ly/gimbal/posture", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_gimbal_capV, "/ly/gimbal/capV", std_msgs::msg::UInt8);
    LY_DEF_ROS_TOPIC(ly_game_eventdata, "ly/gimbal/eventdata", std_msgs::msg::UInt32);
    LY_DEF_ROS_TOPIC(ly_game_event_data, "/ly/game/event_data", gimbal_driver::msg::EventData);

    LY_DEF_ROS_TOPIC(ly_friend_is_precaution, "/ly/friend/is_precaution", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_friend_is_at_home, "/ly/friend/is_at_home", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_friend_is_team_red, "/ly/friend/is_team_red", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_friend_hp, "/ly/friend/hp", gimbal_driver::msg::Health);
    LY_DEF_ROS_TOPIC(ly_friend_op_hp, "/ly/friend/op_hp", std_msgs::msg::UInt16);
    LY_DEF_ROS_TOPIC(ly_friend_base_hp, "/ly/friend/base_hp", std_msgs::msg::UInt16);

    LY_DEF_ROS_TOPIC(ly_friend_ammo_left, "/ly/friend/ammo_left", std_msgs::msg::UInt16);
    LY_DEF_ROS_TOPIC(ly_friend_uwb_pos, "/ly/friend/uwb_pos", gimbal_driver::msg::StampedUInt16MultiArray);
    LY_DEF_ROS_TOPIC(ly_friend_uwb_yaw, "/ly/friend/uwb_yaw", std_msgs::msg::UInt16);

    LY_DEF_ROS_TOPIC(ly_game_is_start, "/ly/game/is_start", std_msgs::msg::Bool);
    LY_DEF_ROS_TOPIC(ly_game_time_left, "/ly/game/time_left", std_msgs::msg::UInt16);

    LY_DEF_ROS_TOPIC(ly_enemy_hp, "/ly/enemy/hp", gimbal_driver::msg::Health);
    LY_DEF_ROS_TOPIC(ly_enemy_op_hp, "/ly/enemy/op_hp", std_msgs::msg::UInt16);
    LY_DEF_ROS_TOPIC(ly_enemy_base_hp, "/ly/enemy/base_hp", std_msgs::msg::UInt16);

    LY_DEF_ROS_TOPIC(ly_game_all, "/ly/game/all", gimbal_driver::msg::GameData);
    LY_DEF_ROS_TOPIC(ly_bullet_speed, "/ly/bullet/speed", std_msgs::msg::Float32);

    LY_DEF_ROS_TOPIC(ly_team_buff, "/ly/team/buff", gimbal_driver::msg::BuffData);
    LY_DEF_ROS_TOPIC(ly_game_rfid, "/ly/game/rfid", gimbal_driver::msg::RfidStatus);
    LY_DEF_ROS_TOPIC(ly_position_data, "/ly/position/data", gimbal_driver::msg::PositionData);
    LY_DEF_ROS_TOPIC(ly_game_sentry_info, "/ly/game/sentry/info", gimbal_driver::msg::SentryInfo);
    LY_DEF_ROS_TOPIC(ly_game_bullet, "/ly/game/bullet", gimbal_driver::msg::BulletInfo);
    LY_DEF_ROS_TOPIC(ly_game_map_command, "/ly/game/map_command", gimbal_driver::msg::MapCommand);
        

    using namespace std::chrono_literals;
    class Application
    {
    public:
        inline static constexpr const char Name[] = "gimbal_driver";

    private:
        ROSNode<Name> Node;
        std::atomic_bool DeviceError{ false };
        IODevice<TypedMessage<sizeof(GimbalData)>, GimbalControlFrame> Device{};
        MultiCallback<GimbalControlFrame> CallbackGenerator;
        GimbalControlFrame controlShadow_{};
        std::uint8_t postureCommand_{0}; // 0=不控制, 1=进攻, 2=防御, 3=移动
        std::uint8_t postureState_{0};   // 0=未知, 1=进攻, 2=防御, 3=移动
        int postureTxRepeatCount_{3};
        std::chrono::milliseconds postureTxInterval_{20};
        std::chrono::milliseconds firecodePartialHold_{100};
        float sentryCoordX_{0.0f};
        float sentryCoordY_{0.0f};
        bool sentryCoordPending_{false};
        bool sentryCoordValid_{false};
        std::chrono::steady_clock::time_point sentryCoordNextSendTime_{std::chrono::steady_clock::time_point::min()};
        std::chrono::milliseconds sentryCoordSendInterval_{100};
        std::chrono::steady_clock::time_point sentryCoordLastRxTime_{};
        int sentryCoordFieldWidthX_{2800};
        int sentryCoordFieldWidthY_{1500};
        int sentryCoordFreshTimeoutMs_{2000};
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_sentry_position_;

        float velocityRawToMps_{0.025f};
        std::uint8_t posturePendingToSend_{0};
        std::uint8_t postureLastSent_{0};
        int posturePendingRepeat_{0};
        std::array<std::chrono::steady_clock::time_point, 5> firecodeLastUpdate_{};
        std::chrono::steady_clock::time_point postureNextSendTime_{
            std::chrono::steady_clock::time_point::min()
        };
        std::uint32_t latestRfidStatusRaw_{0};
        bool hasRfidStatusRaw_{false};
        std::uint8_t latestRfidStatus2_{0};
        bool hasRfidStatus2_{false};
        float latestBulletInitialSpeed_{0.0f};
        bool hasBulletInitialSpeed_{false};
        BulletDataAndRfid2 latestBulletDataAndRfid2_{};
        bool hasBulletDataAndRfid2_{false};
        bool rawSerialLogEnable_{false};
        bool rawSerialLogUplink_{true};
        bool rawSerialLogDownlink_{true};
        bool rawSerialLogScreen_{false};
        bool rawSerialLogFlush_{true};
        std::string rawSerialLogDir_{"~/Log/GimbalRaw"};
        std::string rawSerialLogTypeIds_{"all"};
        std::array<bool, 256> rawSerialLogTypeIdEnabled_{};
        std::ofstream rawSerialLogFile_{};
        std::mutex rawSerialLogMutex_{};
        bool rawSerialTopicEnable_{false};
        bool rawSerialTopicUplink_{true};
        bool rawSerialTopicDownlink_{true};
        std::string rawSerialTopicTypeIds_{"all"};
        std::array<bool, 256> rawSerialTopicTypeIdEnabled_{};
        rclcpp::Publisher<gimbal_driver::msg::GimbalRawFrame>::SharedPtr rawSerialRxPublisher_{};
        rclcpp::Publisher<gimbal_driver::msg::GimbalRawFrame>::SharedPtr rawSerialTxPublisher_{};

        enum FireCodeFieldIndex : std::size_t {
            kFireStatusField = 0,
            kCapStateField,
            kFollowModeField,
            kAimModeField,
            kRotateField,
        };

        static bool IsValidPosture(std::uint8_t posture) noexcept {
            return posture >= 1 && posture <= 3;
        }

        static std::uint8_t ClampU2(std::uint8_t value) noexcept {
            return static_cast<std::uint8_t>(value & 0x03u);
        }

        static std::uint8_t ClampU8Bits(std::uint8_t value, unsigned width) noexcept {
            const std::uint8_t mask = static_cast<std::uint8_t>((1u << width) - 1u);
            return static_cast<std::uint8_t>(value & mask);
        }

        static std::uint16_t ClampU16Bits(std::uint16_t value, unsigned width) noexcept {
            const std::uint16_t mask = static_cast<std::uint16_t>((1u << width) - 1u);
            return static_cast<std::uint16_t>(value & mask);
        }

        static std::int8_t ClampInt8(int value) noexcept {
            return static_cast<std::int8_t>(std::clamp(value, -128, 127));
        }

        static bool Bit(const std::uint32_t raw, const unsigned shift) noexcept {
            return ((raw >> shift) & 0x1u) != 0u;
        }

        static std::uint8_t BitsU8(const std::uint32_t raw, const unsigned shift, const unsigned width) noexcept {
            const std::uint32_t mask = (1u << width) - 1u;
            return static_cast<std::uint8_t>((raw >> shift) & mask);
        }

        static std::uint16_t BitsU16(const std::uint32_t raw, const unsigned shift, const unsigned width) noexcept {
            const std::uint32_t mask = (1u << width) - 1u;
            return static_cast<std::uint16_t>((raw >> shift) & mask);
        }

        static std::uint8_t FireCodeRaw(const FireCodeType& firecode) noexcept {
            return *reinterpret_cast<const std::uint8_t*>(&firecode);
        }

        static FireCodeType FireCodeFromRaw(std::uint8_t raw) noexcept {
            FireCodeType firecode{};
            *reinterpret_cast<std::uint8_t*>(&firecode) = raw;
            return firecode;
        }

        static gimbal_driver::msg::EventData ToEventDataMsg(std::uint32_t raw) {
            gimbal_driver::msg::EventData msg;
            msg.raw = raw;
            msg.self_supply_status = BitsU8(raw, 0, 3);
            msg.self_supply_occupied = Bit(raw, 0);
            msg.self_supply_reserved = Bit(raw, 1);
            msg.self_rmul_supply_occupied = Bit(raw, 2);
            msg.self_small_energy_status = BitsU8(raw, 3, 2);
            msg.self_large_energy_status = BitsU8(raw, 5, 2);
            msg.self_central_highland_status = BitsU8(raw, 7, 2);
            msg.self_trapezoid_highland_status = BitsU8(raw, 9, 2);
            msg.enemy_last_dart_hit_time = BitsU16(raw, 11, 9);
            msg.enemy_last_dart_hit_target = BitsU8(raw, 20, 3);
            msg.center_gain_point_status = BitsU8(raw, 23, 2);
            msg.self_fortress_gain_point_status = BitsU8(raw, 25, 2);
            msg.self_outpost_gain_point_status = BitsU8(raw, 27, 2);
            msg.self_base_gain_point_status = Bit(raw, 29);
            msg.reserved = BitsU8(raw, 30, 2);
            return msg;
        }

        static const char* TypeIdName(std::uint8_t type_id) noexcept {
            switch (type_id) {
                case GimbalData::TypeID: return "GimbalData";
                case GameData::TypeID: return "GameData";
                case HealthMyselfData::TypeID: return "HealthMyselfData";
                case HealthEnemyData::TypeID: return "HealthEnemyData";
                case RFIDAndBuffData::TypeID: return "RFIDAndBuffData";
                case PositionData::TypeID: return "PositionData";
                case ChassisData::TypeID: return "ChassisData";
                case SentryData::TypeID: return "SentryData";
                case BulletDataAndRfid2::TypeID: return "BulletDataAndRfid2";
                case MapCommandData::TypeID: return "MapCommandData";
                default: return "Unknown";
            }
        }

        static std::string ExpandUserPath(std::string path) {
            if (path == "~") {
                if (const char* home = std::getenv("HOME")) {
                    return std::string{home};
                }
            } else if (path.rfind("~/", 0) == 0) {
                if (const char* home = std::getenv("HOME")) {
                    return std::string{home} + path.substr(1);
                }
            }
            return path;
        }

        static std::string TimestampForFileName() {
            const auto now = std::chrono::system_clock::now();
            const std::time_t time = std::chrono::system_clock::to_time_t(now);
            std::tm local_time{};
            if (const auto* tm_ptr = std::localtime(&time)) {
                local_time = *tm_ptr;
            }
            std::ostringstream oss;
            oss << std::put_time(&local_time, "%Y%m%d_%H%M%S");
            return oss.str();
        }

        static std::uint64_t WallTimeNs() {
            return static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());
        }

        template<typename T>
        static std::string BytesToHex(const T& item) {
            const auto* bytes = reinterpret_cast<const std::uint8_t*>(&item);
            std::ostringstream oss;
            oss << std::hex << std::setfill('0');
            for (std::size_t i = 0; i < sizeof(T); ++i) {
                if (i != 0) {
                    oss << ' ';
                }
                oss << std::setw(2) << static_cast<unsigned>(bytes[i]);
            }
            return oss.str();
        }

        static void ParseRawTypeIdFilter(
            const std::string& type_ids,
            std::array<bool, 256>& enabled,
            const char* label) {
            enabled.fill(false);
            auto normalized = type_ids.empty() ? std::string{"all"} : type_ids;
            std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
                return static_cast<char>(std::tolower(ch));
            });
            if (normalized == "all" || normalized == "*") {
                enabled.fill(true);
                return;
            }

            std::stringstream ss{normalized};
            std::string token;
            while (std::getline(ss, token, ',')) {
                token.erase(
                    std::remove_if(token.begin(), token.end(), [](unsigned char ch) {
                        return std::isspace(ch) != 0;
                    }),
                    token.end());
                if (token.empty()) {
                    continue;
                }
                try {
                    const auto value = std::stoi(token);
                    if (value >= 0 && value <= 255) {
                        enabled[static_cast<std::size_t>(value)] = true;
                    }
                } catch (const std::exception& ex) {
                    roslog::warn("Invalid %s type id token '%s': %s",
                                 label,
                                 token.c_str(),
                                 ex.what());
                }
            }
        }

        template<typename T>
        static void AssignRawBytes(gimbal_driver::msg::GimbalRawFrame& msg, const T& item) {
            const auto* bytes = reinterpret_cast<const std::uint8_t*>(&item);
            msg.data.assign(bytes, bytes + sizeof(T));
        }

        void ConfigureRawSerialLog(
            bool enable,
            bool uplink,
            bool downlink,
            bool screen,
            bool flush,
            const std::string& dir,
            const std::string& type_ids) {
            std::lock_guard lock{rawSerialLogMutex_};
            rawSerialLogEnable_ = enable;
            rawSerialLogUplink_ = uplink;
            rawSerialLogDownlink_ = downlink;
            rawSerialLogScreen_ = screen;
            rawSerialLogFlush_ = flush;
            rawSerialLogDir_ = dir.empty() ? std::string{"~/Log/GimbalRaw"} : dir;
            rawSerialLogTypeIds_ = type_ids.empty() ? std::string{"all"} : type_ids;
            ParseRawTypeIdFilter(rawSerialLogTypeIds_, rawSerialLogTypeIdEnabled_, "gimbal raw file log");

            if (!rawSerialLogEnable_) {
                rawSerialLogFile_.close();
                return;
            }

            const auto expanded_dir = ExpandUserPath(rawSerialLogDir_);
            std::error_code ec;
            std::filesystem::create_directories(expanded_dir, ec);
            if (ec) {
                roslog::error("Cannot create gimbal raw log dir '%s': %s",
                              expanded_dir.c_str(),
                              ec.message().c_str());
                rawSerialLogEnable_ = false;
                return;
            }

            const auto log_path = std::filesystem::path(expanded_dir) /
                ("gimbal_raw_" + TimestampForFileName() + ".log");
            rawSerialLogFile_.open(log_path, std::ios::out | std::ios::app);
            if (!rawSerialLogFile_) {
                roslog::error("Cannot open gimbal raw log file '%s'", log_path.string().c_str());
                rawSerialLogEnable_ = false;
                return;
            }
            rawSerialLogFile_
                << "# gimbal_driver raw serial log\n"
                << "# time_ns direction type size hex extra\n";
            if (rawSerialLogFlush_) {
                rawSerialLogFile_.flush();
            }
            roslog::warn("gimbal raw serial log enabled: file=%s uplink=%s downlink=%s type_ids=%s",
                         log_path.string().c_str(),
                         rawSerialLogUplink_ ? "true" : "false",
                         rawSerialLogDownlink_ ? "true" : "false",
                         rawSerialLogTypeIds_.c_str());
        }

        void ConfigureRawSerialTopic(bool enable, bool uplink, bool downlink, const std::string& type_ids) {
            rawSerialTopicEnable_ = enable;
            rawSerialTopicUplink_ = uplink;
            rawSerialTopicDownlink_ = downlink;
            rawSerialTopicTypeIds_ = type_ids.empty() ? std::string{"all"} : type_ids;
            ParseRawTypeIdFilter(rawSerialTopicTypeIds_, rawSerialTopicTypeIdEnabled_, "gimbal raw topic log");

            if (!rawSerialTopicEnable_) {
                rawSerialRxPublisher_.reset();
                rawSerialTxPublisher_.reset();
                return;
            }

            auto node = Node.GetNode();
            if (rawSerialTopicUplink_) {
                rawSerialRxPublisher_ =
                    node->create_publisher<gimbal_driver::msg::GimbalRawFrame>(
                        "/ly/log/gimbal_raw_rx",
                        rclcpp::SensorDataQoS());
            }
            if (rawSerialTopicDownlink_) {
                rawSerialTxPublisher_ =
                    node->create_publisher<gimbal_driver::msg::GimbalRawFrame>(
                        "/ly/log/gimbal_raw_tx",
                        rclcpp::SensorDataQoS());
            }
        }

        template<typename T>
        void WriteRawSerialLogLine(
            const char* direction,
            const char* type,
            const T& item,
            const std::string& extra = {}) {
            std::lock_guard lock{rawSerialLogMutex_};
            if (!rawSerialLogEnable_) {
                return;
            }

            std::ostringstream line;
            line << WallTimeNs()
                 << " " << direction
                 << " " << type
                 << " size=" << sizeof(T)
                 << " hex=\"" << BytesToHex(item) << "\"";
            if (!extra.empty()) {
                line << " " << extra;
            }

            if (rawSerialLogFile_) {
                rawSerialLogFile_ << line.str() << '\n';
                if (rawSerialLogFlush_) {
                    rawSerialLogFile_.flush();
                }
            }
            if (rawSerialLogScreen_) {
                roslog::info("%s", line.str().c_str());
            }
        }

        void PublishRawRxTopic(const TypedMessage<sizeof(GimbalData)>& message) {
            if (!rawSerialRxPublisher_ || rawSerialRxPublisher_->get_subscription_count() == 0) {
                return;
            }
            gimbal_driver::msg::GimbalRawFrame msg;
            msg.header.stamp = Node.GetNode()->now();
            msg.direction = gimbal_driver::msg::GimbalRawFrame::DIRECTION_RX;
            msg.type_id = message.TypeID;
            AssignRawBytes(msg, message);
            rawSerialRxPublisher_->publish(msg);
        }

        void PublishRawTxTopic(const GimbalControlFrame& data) {
            if (!rawSerialTxPublisher_ || rawSerialTxPublisher_->get_subscription_count() == 0) {
                return;
            }
            gimbal_driver::msg::GimbalRawFrame msg;
            msg.header.stamp = Node.GetNode()->now();
            msg.direction = gimbal_driver::msg::GimbalRawFrame::DIRECTION_TX;
            msg.type_id = gimbal_driver::msg::GimbalRawFrame::TYPE_ID_TX_CONTROL;
            AssignRawBytes(msg, data);
            msg.firecode_raw = FireCodeRaw(data.FireCode);
            msg.sentry_cmd_raw = std::bit_cast<std::uint32_t>(data.SentryCmd);
            rawSerialTxPublisher_->publish(msg);
        }

        void LogUplinkRaw(const TypedMessage<sizeof(GimbalData)>& message) {
            if (rawSerialLogEnable_ &&
                rawSerialLogUplink_ &&
                rawSerialLogTypeIdEnabled_[static_cast<std::size_t>(message.TypeID)]) {
                std::ostringstream extra;
                extra << "name=" << TypeIdName(message.TypeID);
                const auto type_id = std::to_string(message.TypeID);
                WriteRawSerialLogLine("rx", type_id.c_str(), message, extra.str());
            }

            if (rawSerialTopicEnable_ &&
                rawSerialTopicUplink_ &&
                rawSerialTopicTypeIdEnabled_[static_cast<std::size_t>(message.TypeID)]) {
                PublishRawRxTopic(message);
            }
        }

        void LogDownlinkRaw(const GimbalControlFrame& data, const char* reason) {
            if (rawSerialLogEnable_ && rawSerialLogDownlink_) {
                const auto sentry_cmd_raw = std::bit_cast<std::uint32_t>(data.SentryCmd);
                std::ostringstream extra;
                extra << "reason=" << (reason ? reason : "control")
                      << " firecode_raw=" << static_cast<unsigned>(FireCodeRaw(data.FireCode))
                      << " sentry_cmd_raw=" << sentry_cmd_raw;
                WriteRawSerialLogLine("tx", "control", data, extra.str());
            }

            if (rawSerialTopicEnable_ && rawSerialTopicDownlink_) {
                PublishRawTxTopic(data);
            }
        }

        static gimbal_driver::msg::RfidStatus ToRfidStatusMsg(
            std::uint32_t raw,
            bool has_status2 = false,
            std::uint8_t raw_status2 = 0
        ) {
            gimbal_driver::msg::RfidStatus msg;
            msg.raw = raw;
            msg.friend_base = Bit(raw, 0);
            msg.friend_central = Bit(raw, 1);
            msg.enemy_central = Bit(raw, 2);
            msg.friend_highland = Bit(raw, 3);
            msg.enemy_highland = Bit(raw, 4);
            msg.friend_flyroad_front = Bit(raw, 5);
            msg.friend_flyroad_back = Bit(raw, 6);
            msg.enemy_flyroad_front = Bit(raw, 7);
            msg.enemy_flyroad_back = Bit(raw, 8);
            msg.friend_central_under = Bit(raw, 9);
            msg.friend_central_high = Bit(raw, 10);
            msg.enemy_central_under = Bit(raw, 11);
            msg.enemy_central_high = Bit(raw, 12);
            msg.friend_roadland_under = Bit(raw, 13);
            msg.friend_roadland_high = Bit(raw, 14);
            msg.enemy_roadland_under = Bit(raw, 15);
            msg.enemy_roadland_high = Bit(raw, 16);
            msg.friend_bastion = Bit(raw, 17);
            msg.friend_outpost = Bit(raw, 18);
            msg.friend_supply_noremix = Bit(raw, 19);
            msg.friend_supply_remix = Bit(raw, 20);
            msg.friend_armor = Bit(raw, 21);
            msg.enemy_armor = Bit(raw, 22);
            msg.central_rmul = Bit(raw, 23);
            msg.enemy_bastion = Bit(raw, 24);
            msg.enemy_outpost = Bit(raw, 25);
            msg.friend_tunnel_roadland_down = Bit(raw, 26);
            msg.friend_tunnel_roadland_mid = Bit(raw, 27);
            msg.friend_tunnel_roadland_up = Bit(raw, 28);
            msg.friend_tunnel_highland_low = Bit(raw, 29);
            msg.friend_tunnel_highland_mid = Bit(raw, 30);
            msg.friend_tunnel_highland_high = Bit(raw, 31);
            msg.has_rfid_status_2 = has_status2;
            msg.rfid_status_2_raw = has_status2 ? raw_status2 : 0;
            msg.enemy_tunnel_roadland_down = has_status2 && Bit(raw_status2, 0);
            msg.enemy_tunnel_roadland_mid = has_status2 && Bit(raw_status2, 1);
            msg.enemy_tunnel_roadland_up = has_status2 && Bit(raw_status2, 2);
            msg.enemy_tunnel_highland_low = has_status2 && Bit(raw_status2, 3);
            msg.enemy_tunnel_highland_mid = has_status2 && Bit(raw_status2, 4);
            msg.enemy_tunnel_highland_high = has_status2 && Bit(raw_status2, 5);
            msg.rfid_status_2_reserved = has_status2 ? BitsU8(raw_status2, 6, 2) : 0;
            return msg;
        }

        static gimbal_driver::msg::SentryInfo ToSentryInfoMsg(const SentryData& data) {
            gimbal_driver::msg::SentryInfo msg;
            msg.sentry_info_raw = data.SentryInfo;
            msg.sentry_info_2_raw = data.SentryInfo2;
            msg.reserved = data.Reserved;

            msg.exchanged_projectile_allowance = BitsU16(data.SentryInfo, 0, 11);
            msg.remote_projectile_exchange_count = BitsU8(data.SentryInfo, 11, 4);
            msg.remote_hp_exchange_count = BitsU8(data.SentryInfo, 15, 4);
            msg.can_confirm_free_revive = Bit(data.SentryInfo, 19);
            msg.can_exchange_immediate_revive = Bit(data.SentryInfo, 20);
            msg.immediate_revive_cost = BitsU16(data.SentryInfo, 21, 10);
            msg.sentry_info_reserved = Bit(data.SentryInfo, 31);

            msg.out_of_combat = Bit(data.SentryInfo2, 0);
            msg.remaining_exchangeable_17mm = BitsU16(data.SentryInfo2, 1, 11);
            msg.posture = BitsU8(data.SentryInfo2, 12, 2);
            msg.can_activate_energy_mechanism = Bit(data.SentryInfo2, 14);
            msg.sentry_info_2_reserved = Bit(data.SentryInfo2, 15);
            return msg;
        }

        static gimbal_driver::msg::FireCode ToFireCodeMsg(const FireCodeType& firecode) {
            gimbal_driver::msg::FireCode msg;
            msg.field_mask = gimbal_driver::msg::FireCode::FIELD_ALL;
            msg.fire_status = firecode.FireStatus;
            msg.cap_state = firecode.CapState;
            msg.follow_mode = firecode.FollowMode != 0;
            msg.aim_mode = firecode.AimMode != 0;
            msg.rotate = firecode.Rotate;
            msg.raw = FireCodeRaw(firecode);
            return msg;
        }

        std::int8_t EncodeVelocityRaw(float meters_per_second) const noexcept {
            if (velocityRawToMps_ <= 0.0f) {
                return 0;
            }
            return ClampInt8(static_cast<int>(std::lround(meters_per_second / velocityRawToMps_)));
        }

        float DecodeVelocityRaw(std::int8_t raw) const noexcept {
            return static_cast<float>(raw) * velocityRawToMps_;
        }

        static std::uint8_t DecodePostureFromU16(std::uint16_t posture_raw) noexcept {
            // 兼容两种编码：
            // 1) 老约定：高8位是姿态值
            // 2) 新约定：低8位直接是姿态值
            const auto low8 = static_cast<std::uint8_t>(posture_raw & 0xFFu);
            if (IsValidPosture(low8)) {
                return low8;
            }
            const auto high8 = static_cast<std::uint8_t>((posture_raw >> 8) & 0xFFu);
            return IsValidPosture(high8) ? high8 : 0u;
        }

        static std::int16_t DecodeI16(std::uint16_t raw) noexcept {
            return std::bit_cast<std::int16_t>(raw);
        }

        static float DecodeI16WithScale(std::uint16_t raw, float divisor) noexcept {
            return static_cast<float>(DecodeI16(raw)) / divisor;
        }

        void ArmPostureTx(std::uint8_t posture) {
            if (!IsValidPosture(posture)) {
                return;
            }
            if (posture == postureLastSent_ && posturePendingRepeat_ == 0) {
                return;
            }
            posturePendingToSend_ = posture;
            posturePendingRepeat_ = std::max(1, postureTxRepeatCount_);
            postureNextSendTime_ = std::chrono::steady_clock::now();
            roslog::info("Posture TX armed: cmd=%u, repeat=%d",
                         posturePendingToSend_, posturePendingRepeat_);
        }

        void MaybeSendPostureTx() {
            if (posturePendingRepeat_ <= 0) {
                return;
            }
            const auto now = std::chrono::steady_clock::now();
            if (now < postureNextSendTime_) {
                return;
            }

            auto tx = controlShadow_;
            tx.SentryCmd.Posture = posturePendingToSend_;
            if (!Device.Write(tx)) {
                DeviceError = true;
                return;
            }
            LogDownlinkRaw(tx, "posture_repeat");
            controlShadow_ = tx;

            posturePendingRepeat_--;
            postureNextSendTime_ = now + postureTxInterval_;
            if (posturePendingRepeat_ == 0) {
                postureLastSent_ = posturePendingToSend_;
            }
        }

        void PublishPosture(std::uint8_t posture) {
            using topic = ly_gimbal_posture;
            topic::Msg msg;
            msg.data = posture;
            Node.Publisher<topic>()->publish(msg);
        }

        void DegradeStaleFireCode(FireCodeType& firecode, const std::chrono::steady_clock::time_point now) const {
            auto stale = [&](FireCodeFieldIndex field) {
                const auto stamp = firecodeLastUpdate_[static_cast<std::size_t>(field)];
                return stamp.time_since_epoch().count() == 0 ||
                       (now - stamp) > firecodePartialHold_;
            };

            if (stale(kFireStatusField)) firecode.FireStatus = 0;
            if (stale(kCapStateField)) firecode.CapState = 0;
            if (stale(kFollowModeField)) firecode.FollowMode = 0;
            if (stale(kAimModeField)) firecode.AimMode = 0;
            if (stale(kRotateField)) firecode.Rotate = 0;
        }

        void ApplyFireCodeCommand(GimbalControlFrame& g, const gimbal_driver::msg::FireCode& m) {
            const auto now = std::chrono::steady_clock::now();
            const bool full_snapshot = (m.field_mask == 0) ||
                ((m.field_mask & gimbal_driver::msg::FireCode::FIELD_ALL) == gimbal_driver::msg::FireCode::FIELD_ALL);

            auto mark = [&](FireCodeFieldIndex field) {
                firecodeLastUpdate_[static_cast<std::size_t>(field)] = now;
            };

            if (full_snapshot || (m.field_mask & gimbal_driver::msg::FireCode::FIELD_FIRE_STATUS)) {
                g.FireCode.FireStatus = ClampU2(m.fire_status);
                mark(kFireStatusField);
            }
            if (full_snapshot || (m.field_mask & gimbal_driver::msg::FireCode::FIELD_CAP_STATE)) {
                g.FireCode.CapState = ClampU2(m.cap_state);
                mark(kCapStateField);
            }
            if (full_snapshot || (m.field_mask & gimbal_driver::msg::FireCode::FIELD_FOLLOW_MODE)) {
                g.FireCode.FollowMode = m.follow_mode ? 1 : 0;
                mark(kFollowModeField);
            }
            if (full_snapshot || (m.field_mask & gimbal_driver::msg::FireCode::FIELD_AIM_MODE)) {
                g.FireCode.AimMode = m.aim_mode ? 1 : 0;
                mark(kAimModeField);
            }
            if (full_snapshot || (m.field_mask & gimbal_driver::msg::FireCode::FIELD_ROTATE)) {
                g.FireCode.Rotate = ClampU2(m.rotate);
                mark(kRotateField);
            }

            if (!full_snapshot) {
                DegradeStaleFireCode(g.FireCode, now);
            }
        }

        void ApplySentryCmdCommand(GimbalControlFrame& g, const gimbal_driver::msg::SentryCmd& m) {
            const bool full_snapshot = (m.field_mask == 0) ||
                ((m.field_mask & gimbal_driver::msg::SentryCmd::FIELD_ALL) == gimbal_driver::msg::SentryCmd::FIELD_ALL);

            auto has_field = [&](const std::uint8_t field) {
                return full_snapshot || ((m.field_mask & field) != 0);
            };

            if (has_field(gimbal_driver::msg::SentryCmd::FIELD_CONFIRM_FREE_REVIVE)) {
                g.SentryCmd.ConfirmFreeRevive = m.confirm_free_revive ? 1 : 0;
            }
            if (has_field(gimbal_driver::msg::SentryCmd::FIELD_CONFIRM_IMMEDIATE_REVIVE)) {
                g.SentryCmd.ConfirmImmediateRevive = m.confirm_immediate_revive ? 1 : 0;
            }
            if (has_field(gimbal_driver::msg::SentryCmd::FIELD_EXCHANGE_PROJECTILE_ALLOWANCE)) {
                g.SentryCmd.ExchangeProjectileAllowance = ClampU16Bits(m.exchange_projectile_allowance, 11);
            }
            if (has_field(gimbal_driver::msg::SentryCmd::FIELD_REMOTE_PROJECTILE_EXCHANGE_COUNT)) {
                g.SentryCmd.RemoteProjectileExchangeCount =
                    ClampU8Bits(m.remote_projectile_exchange_count, 4);
            }
            if (has_field(gimbal_driver::msg::SentryCmd::FIELD_REMOTE_HP_EXCHANGE_COUNT)) {
                g.SentryCmd.RemoteHpExchangeCount = ClampU8Bits(m.remote_hp_exchange_count, 4);
            }
            if (has_field(gimbal_driver::msg::SentryCmd::FIELD_POSTURE)) {
                const auto posture = ClampU2(m.posture);
                if (m.posture != posture) {
                    roslog::warn("Invalid /ly/control/sentry_cmd posture: %u (expect 0/1/2/3)",
                                 m.posture);
                }
                g.SentryCmd.Posture = posture;
                postureCommand_ = posture;
                if (posture == 0) {
                    posturePendingRepeat_ = 0;
                    posturePendingToSend_ = 0;
                } else {
                    ArmPostureTx(posture);
                }
            }
            if (has_field(gimbal_driver::msg::SentryCmd::FIELD_CONFIRM_ENERGY_ACTIVATE)) {
                g.SentryCmd.ConfirmEnergyActivate = m.confirm_energy_activate ? 1 : 0;
            }
        }

        
        void MaybeSendSentryCoordinate() {
            if (!sentryCoordPending_ || !sentryCoordValid_) return;
            const auto now = std::chrono::steady_clock::now();
            if (now < sentryCoordNextSendTime_) return;
            if (sentryCoordLastRxTime_.time_since_epoch().count() != 0 && now - sentryCoordLastRxTime_ > std::chrono::milliseconds(sentryCoordFreshTimeoutMs_)) { sentryCoordValid_ = false; sentryCoordPending_ = false; return; }
            float xc = sentryCoordX_; float yc = sentryCoordY_;
            if (xc < 0.0f) xc = 0.0f;
            if (xc > static_cast<float>(sentryCoordFieldWidthX_)) xc = static_cast<float>(sentryCoordFieldWidthX_);
            if (yc < 0.0f) yc = 0.0f;
            if (yc > static_cast<float>(sentryCoordFieldWidthY_)) yc = static_cast<float>(sentryCoordFieldWidthY_);
            LangYa::SentryCoordinateFrame frame;
            frame.X_cm = static_cast<std::int16_t>(xc);
            frame.Y_cm = static_cast<std::int16_t>(yc);
            frame.CRC8 = CRCChecker::CRC8::calculate_downlink(reinterpret_cast<const std::uint8_t*>(&frame), 16);
            if (Device.WriteRaw(frame)) { sentryCoordPending_ = false; } else { DeviceError = true; return; }
            sentryCoordNextSendTime_ = now + sentryCoordSendInterval_;
        }

        void MaybeApplyFireCodeStaleFallback() {
            const auto now = std::chrono::steady_clock::now();
            auto next = controlShadow_.FireCode;
            const auto before_raw = FireCodeRaw(next);
            DegradeStaleFireCode(next, now);
            if (FireCodeRaw(next) == before_raw) {
                return;
            }
            CallbackGenerator.Modify([&](GimbalControlFrame& g) {
                g.FireCode = next;
            });
        }

        template<typename TTopic>
        void GenSub(auto modifier)
        {
            Node.GenSubscriber<TTopic>(CallbackGenerator.Generate<TTopic>(modifier));
        }

        //TODO disable cooldown timer
        using clock_t = std::chrono::steady_clock;
           
        struct Timer {
            clock_t::duration interval;
            clock_t::time_point head;

            bool check(const clock_t::time_point now) {
                const auto delta_t = now - head;
                if (delta_t > interval) {
                    head = now;
                    return true;
                }
                return false;
            }
        };

        class StateTimer {
            Timer shoot_timer{};
            Timer cooldown_timer{};
            bool is_shoot{false};

            Timer& get_timer() {
                return is_shoot ? shoot_timer : cooldown_timer;
            }

        public:
            bool can_shoot() const noexcept {
                return is_shoot;
            }

            StateTimer(const clock_t::duration shoot, const clock_t::duration cooldown) {
                const auto now = clock_t::now();
                shoot_timer.head = now;
                shoot_timer.interval = shoot;
                cooldown_timer.head = now;
                cooldown_timer.interval = cooldown;
            }

            bool check() {
                const auto now = clock_t::now();
                if (get_timer().check(now)) {
                    is_shoot = !is_shoot;
                    get_timer().head = now;
                }
                
                return can_shoot();
            }

        };

        StateTimer state_timer{std::chrono::milliseconds(100), std::chrono::milliseconds(2000)}; //簡學長寫的timer，使用的話則關掉下面的false &&
        //TODO 

        void GenSubs()
        {
            GenSub<ly_control_angles>([](GimbalControlFrame& g, const gimbal_driver::msg::GimbalAngles& m)
                                        {
                                            g.GimbalAngles.Yaw = static_cast<float>(m.yaw);
                                            g.GimbalAngles.Pitch = static_cast<float>(m.pitch);  
                                        });

            GenSub<ly_control_firecode>([this](GimbalControlFrame& g, const gimbal_driver::msg::FireCode& m)
                                        {
                                            ApplyFireCodeCommand(g, m);
                                            if (false && !state_timer.check()) {
                                                g.FireCode.FireStatus = 0;
                                            }
                                        });

            GenSub<ly_control_vel>([this](GimbalControlFrame& g, const gimbal_driver::msg::ControlVelocity& m)
                                   {
                                       if (m.use_raw) {
                                           g.Velocity.X = m.raw_x;
                                           g.Velocity.Y = m.raw_y;
                                           return;
                                       }
                                       g.Velocity.X = EncodeVelocityRaw(m.x_mps);
                                       g.Velocity.Y = EncodeVelocityRaw(m.y_mps);
                                   });

            GenSub<ly_control_posture>([this](GimbalControlFrame& g, const gimbal_driver::msg::SentryCmd& m)
                                       {
                                           const bool has_posture =
                                               m.field_mask == 0 ||
                                               ((m.field_mask & gimbal_driver::msg::SentryCmd::FIELD_POSTURE) != 0);
                                           if (!has_posture) {
                                               roslog::warn("/ly/control/posture missing FIELD_POSTURE; ignore");
                                               return;
                                           }
                                           const auto cmd = m.posture;
                                           if (cmd != 0 && !IsValidPosture(cmd)) {
                                               roslog::warn("Invalid /ly/control/posture: %u (expect 0/1/2/3)", cmd);
                                               return;
                                           }
                                           postureCommand_ = cmd;
                                           g.SentryCmd.Posture = cmd;
                                           // /ly/gimbal/posture 由下位机/裁判回读驱动，避免命令回环掩盖真实执行状态。
                                           if (cmd == 0) {
                                               posturePendingRepeat_ = 0;
                                               posturePendingToSend_ = 0;
                                               return;
                                           }
                                           ArmPostureTx(cmd);
                                       });

            GenSub<ly_control_sentry_cmd>([this](GimbalControlFrame& g, const gimbal_driver::msg::SentryCmd& m)
                                          {
                                              ApplySentryCmdCommand(g, m);
                                          });
        }

        void PublishRfidStatus(const rclcpp::Time& stamp) {
            if (!hasRfidStatusRaw_ && !hasRfidStatus2_) {
                return;
            }
            using topic = ly_game_rfid;
            auto msg = ToRfidStatusMsg(latestRfidStatusRaw_, hasRfidStatus2_, latestRfidStatus2_);
            msg.header.stamp = stamp;
            Node.Publisher<topic>()->publish(msg);
        }

        void PublishBulletInfo(const rclcpp::Time& stamp) {
            using topic = ly_game_bullet;
            topic::Msg msg;
            msg.header.stamp = stamp;

            msg.has_initial_speed = hasBulletInitialSpeed_;
            msg.initial_speed = hasBulletInitialSpeed_ ? latestBulletInitialSpeed_ : 0.0f;

            msg.has_shoot_data = hasBulletDataAndRfid2_;
            msg.bullet_type = hasBulletDataAndRfid2_ ? latestBulletDataAndRfid2_.BulletType : 0;
            msg.shooter_number = hasBulletDataAndRfid2_ ? latestBulletDataAndRfid2_.ShooterNumber : 0;
            msg.launching_frequency =
                hasBulletDataAndRfid2_ ? latestBulletDataAndRfid2_.LaunchingFrequency : 0;

            msg.has_projectile_allowance = hasBulletDataAndRfid2_;
            msg.projectile_allowance_17mm =
                hasBulletDataAndRfid2_ ? latestBulletDataAndRfid2_.ProjectileAllowance17mm : 0;
            msg.projectile_allowance_42mm =
                hasBulletDataAndRfid2_ ? latestBulletDataAndRfid2_.ProjectileAllowance42mm : 0;
            msg.remaining_gold_coin =
                hasBulletDataAndRfid2_ ? latestBulletDataAndRfid2_.RemainingGoldCoin : 0;
            msg.projectile_allowance_fortress_17mm =
                hasBulletDataAndRfid2_ ? latestBulletDataAndRfid2_.ProjectileAllowanceFortress : 0;
            Node.Publisher<topic>()->publish(msg);
        }

        void  PubGimbalData(const GimbalData& data)
	        {
            {
                using topic = ly_gimbal_angles;
                topic::Msg msg;
                msg.yaw = static_cast<float>(data.GimbalAngles.Yaw);
                msg.pitch = static_cast<float>(data.GimbalAngles.Pitch);
                msg.header.stamp = Node.GetNode()->now();
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_gimbal_firecode;
                auto msg = ToFireCodeMsg(data.FireCode);
                msg.header.stamp = Node.GetNode()->now();
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_gimbal_capV;
                topic::Msg msg;
                msg.data = static_cast<std::uint8_t>(data.CapV);
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void PubGameData(const GameData& data)
        {
            {
                using topic = ly_game_all;
                topic::Msg msg;
                msg.gamecode = *reinterpret_cast<const std::uint16_t*>(&data.GameCode);
                msg.ammoleft = data.AmmoLeft;
                msg.timeleft = data.TimeLeft;
                msg.selfhealth = data.SelfHealth;
                msg.exteventdata = *static_cast<const std::uint32_t*>(&data.ExtEventData);
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_friend_ammo_left;
                topic::Msg msg;
                msg.data = data.AmmoLeft;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_enemy_op_hp;
                topic::Msg msg;
                msg.data = data.GameCode.EnemyOutpostHealth * 25;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_friend_is_precaution;
                topic::Msg msg;
                msg.data = data.GameCode.HeroPrecaution;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_game_is_start;
                topic::Msg msg;
                msg.data = data.GameCode.IsGameBegin;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_friend_is_team_red;
                topic::Msg msg;
                msg.data = data.GameCode.IsMyTeamRed;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_friend_is_at_home;
                topic::Msg msg;
                msg.data = data.GameCode.IsReturnedHome;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_friend_op_hp;
                topic::Msg msg;
                msg.data = data.GameCode.SelfOutpostHealth * 25;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_game_time_left;
                topic::Msg msg;
                msg.data = data.TimeLeft;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_game_eventdata;
                topic::Msg msg;
                msg.data = static_cast<std::uint32_t>(data.ExtEventData);
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_game_event_data;
                auto msg = ToEventDataMsg(data.ExtEventData);
                msg.header.stamp = Node.GetNode()->now();
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void PubRFIDAndBuffData(const RFIDAndBuffData& data){
            latestRfidStatusRaw_ = data.RFIDStatus;
            hasRfidStatusRaw_ = true;
            PublishRfidStatus(Node.GetNode()->now());
            {
                using topic = ly_team_buff;
                topic::Msg msg;
                msg.recoverybuff = data.BuffStatus.RecoveryBuff;
                msg.coolingbuff = data.BuffStatus.CoolingBuff;
                msg.defencebuff = data.BuffStatus.DefenceBuff;
                msg.vulnerabilitybuff = data.BuffStatus.VulnerabilityBuff;
                msg.attackbuff = data.BuffStatus.AttackBuff;
                msg.remainingenergy = data.BuffStatus.RemainingEnergy;
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void PubPositionData(const PositionData& data){
            {
                {
                    using topic = ly_position_data;
                    topic::Msg msg;
                    msg.header.stamp = Node.GetNode()->now();
                    msg.friendcarid = data.Friend.CarId;
                    msg.friendx = data.Friend.X;
                    msg.friendy = data.Friend.Y;
                    msg.enemycarid = data.Enemy.CarId;
                    msg.enemyx = data.Enemy.X;
                    msg.enemyy = data.Enemy.Y;
                    Node.Publisher<topic>()->publish(msg);
                }
                if(data.Friend.CarId == 7) {
                    using topic = ly_friend_uwb_pos;
                    std::vector<std::uint16_t> pos = {
                        static_cast<std::uint16_t>(data.Friend.X),
                        static_cast<std::uint16_t>(data.Friend.Y)
                    };
                    topic::Msg msg;
                    msg.header.stamp = Node.GetNode()->now();
                    msg.header.frame_id = "official_map";
                    msg.data = pos;
                    Node.Publisher<topic>()->publish(msg);
                }
            }
            {
                using topic = ly_bullet_speed;
                topic::Msg msg;
                msg.data = static_cast<float>(data.BulletSpeed) / 100.0f;
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void PubHealthMyselfData(const HealthMyselfData& data){
            {
                using topic = ly_friend_hp;
                topic::Msg msg;
                msg.header.stamp = Node.GetNode()->now();
                msg.hero = data.HeroMyself;
                msg.engineer = data.EngineerMyself;
                msg.infantry1 = data.Infantry1Myself;
                msg.infantry2 = data.Infantry2Myself;
                msg.reserve = data.BaseMyself;
                msg.sentry = data.SentryMyself;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_friend_base_hp;
                topic::Msg msg;
                msg.data = data.BaseMyself;
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void PubHealthEnemyData(const HealthEnemyData& data){
            {
                using topic = ly_enemy_hp;
                topic::Msg msg;
                msg.header.stamp = Node.GetNode()->now();
                msg.hero = data.HeroEnemy;
                msg.engineer = data.EngineerEnemy;
                msg.infantry1 = data.Infantry1Enemy;
                msg.infantry2 = data.Infantry2Enemy;
                msg.reserve = data.BaseEnemy;
                msg.sentry = data.SentryEnemy;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_enemy_base_hp;
                topic::Msg msg;
                msg.data = data.BaseEnemy;
                Node.Publisher<topic>()->publish(msg);
            }
        }

        void PubChassisData(const ChassisData& data) {
            const auto packed1_u32 = static_cast<std::uint32_t>(data.ChassisPacked1);
            const auto packed2_u32 = static_cast<std::uint32_t>(data.ChassisPacked2);

            const auto steer_angle_u16 = static_cast<std::uint16_t>(packed1_u32 & 0xFFFFu);
            const auto angular_vel_u16 = static_cast<std::uint16_t>((packed1_u32 >> 16) & 0xFFFFu);
            const auto vel_x_u16 = static_cast<std::uint16_t>(packed2_u32 & 0xFFFFu);
            const auto vel_y_u16 = static_cast<std::uint16_t>((packed2_u32 >> 16) & 0xFFFFu);
            const auto now = Node.GetNode()->now();

            const auto steer_angle = DecodeI16WithScale(steer_angle_u16, 10.0f);
            const auto angular_velocity = DecodeI16WithScale(angular_vel_u16, 100.0f);
            const auto velocity_x = DecodeI16WithScale(vel_x_u16, 100.0f);
            const auto velocity_y = DecodeI16WithScale(vel_y_u16, 100.0f);

            {
                using topic = ly_friend_uwb_yaw;
                topic::Msg msg;
                msg.data = data.UWBAngleYaw;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_gimbal_chassis;
                topic::Msg msg;
                msg.header.stamp = now;
                msg.steer_angle = steer_angle;
                msg.angular_velocity = angular_velocity;
                msg.velocity_x = velocity_x;
                msg.velocity_y = velocity_y;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_gimbal_big_yaw_angles;
                topic::Msg msg;
                msg.data = steer_angle;
                Node.Publisher<topic>()->publish(msg);
            }
            {
                using topic = ly_gimbal_vel;
                topic::Msg msg;
                msg.header.stamp = now;
                msg.x = velocity_x;
                msg.y = velocity_y;
                Node.Publisher<topic>()->publish(msg);
            }

            const auto posture_raw = DecodePostureFromU16(data.Posture);
            if (IsValidPosture(posture_raw)) {
                postureState_ = posture_raw;
                PublishPosture(postureState_);
            }
        }

        void PubSentryData(const SentryData& data) {
            const auto now = Node.GetNode()->now();
            auto sentry_info_msg = ToSentryInfoMsg(data);
            sentry_info_msg.header.stamp = now;
            {
                using topic = ly_game_sentry_info;
                Node.Publisher<topic>()->publish(sentry_info_msg);
            }
            if (IsValidPosture(sentry_info_msg.posture)) {
                postureState_ = sentry_info_msg.posture;
                PublishPosture(postureState_);
            }

            latestBulletInitialSpeed_ = data.BulletInitialSpeed;
            hasBulletInitialSpeed_ = true;
            PublishBulletInfo(now);
        }

        void PubBulletDataAndRfid2(const BulletDataAndRfid2& data) {
            const auto now = Node.GetNode()->now();
            latestBulletDataAndRfid2_ = data;
            hasBulletDataAndRfid2_ = true;
            latestRfidStatus2_ = data.RfidStatus2;
            hasRfidStatus2_ = true;

            PublishBulletInfo(now);
            PublishRfidStatus(now);
        }

        void PubMapCommandData(const MapCommandData& data) {
            using topic = ly_game_map_command;
            topic::Msg msg;
            msg.header.stamp = Node.GetNode()->now();
            msg.has_target_position = data.TargetRobotId == 0;
            msg.target_position_x_m = data.TargetPositionX;
            msg.target_position_y_m = data.TargetPositionY;
            msg.has_target_robot = data.TargetRobotId != 0;
            msg.target_robot_id = data.TargetRobotId;
            msg.cmd_keyboard = data.CmdKeyboard;
            msg.cmd_source = data.CmdSource;
            Node.Publisher<topic>()->publish(msg);
        }

        void LoopRead()
        {
            Device.LoopRead(DeviceError, [this](const TypedMessage<sizeof(GimbalData)>& m)
            {
                LogUplinkRaw(m);
                switch (m.TypeID)
                {
                    case GimbalData::TypeID:
                        PubGimbalData(m.GetDataAs<GimbalData>());
                        break;

                    case GameData::TypeID:
                    {
                        const auto& game_data = m.GetDataAs<GameData>();
                        PubGameData(game_data);
                        break;
                    }

                    case HealthMyselfData::TypeID:
                        PubHealthMyselfData(m.GetDataAs<HealthMyselfData>());
                        break;

                    case HealthEnemyData::TypeID:
                        PubHealthEnemyData(m.GetDataAs<HealthEnemyData>());
                        break;

                    case RFIDAndBuffData::TypeID:
                        PubRFIDAndBuffData(m.GetDataAs<RFIDAndBuffData>());
                        break;

                    case PositionData::TypeID:
                        PubPositionData(m.GetDataAs<PositionData>());
                        break;
                    case ChassisData::TypeID:
                    {
                        const auto& chassis_data = m.GetDataAs<ChassisData>();
                        PubChassisData(chassis_data);
                        break;
                    }
                    case SentryData::TypeID:
                    {
                        PubSentryData(m.GetDataAs<SentryData>());
                        break;
                    }
                    case BulletDataAndRfid2::TypeID:
                    {
                        PubBulletDataAndRfid2(m.GetDataAs<BulletDataAndRfid2>());
                        break;
                    }
                    case MapCommandData::TypeID:
                    {
                        PubMapCommandData(m.GetDataAs<MapCommandData>());
                        break;
                    }

                    default:
                        roslog::error("Application::LoopRead: invalid type id(%u)",
                                      static_cast<unsigned int>(m.TypeID));
                        break;
                }
            });
        }

        void TestVirtualLoopback(){
            TypedMessage<sizeof(GimbalData)> test_msg{};
            GimbalControlFrame test_msg2{};
            test_msg.TypeID = GimbalData::TypeID;
            test_msg.GetDataAs<GimbalData>().GimbalAngles.Yaw = 45.0f;
            test_msg2.GimbalAngles.Yaw = 30.0f;

            Device.Write(test_msg2);
            
            Device.LoopRead(DeviceError, [&](const TypedMessage<sizeof(GimbalData)>& received){
                if (memcmp(&test_msg, &received, sizeof(test_msg)) == 0) {
                    RCLCPP_INFO(rclcpp::get_logger("test"), "Virtual loopback test passed");
                }
            });
        }

    public:
        Application() noexcept : CallbackGenerator{
                [this](const auto& data)
                {
                    controlShadow_ = data;
                    if (DeviceError) return;
                    if (!Device.Write(data)) {
                        DeviceError = true;
                        return;
                    }
                    LogDownlinkRaw(data, "control_callback");
                }
        }
        {
        }

        void Run(int argc, char** argv)
        {
            Node.Initialize(argc, argv);
            Node.Publisher<ly_gimbal_big_yaw_angles>();
            GenSubs();
            auto node = Node.GetNode();
            rclcpp::Rate rate(250);
            bool useVirtualDevice = false;
            std::string serialDeviceName{"/dev/ttyACM0"};
            int serialBaudRate = 115200;
            auto getParamCompat = [this](const char* slash_key, const char* dot_key, auto& value, const auto& default_value) {
                using T = std::decay_t<decltype(value)>;
                Node.GetParam<T>(slash_key, value, static_cast<T>(default_value));
                T dot_value = value;
                Node.GetParam<T>(dot_key, dot_value, value);
                value = dot_value;
            };
            getParamCompat("io_config/use_virtual_device", "io_config.use_virtual_device", useVirtualDevice, false);
            getParamCompat("io_config/device_name", "io_config.device_name", serialDeviceName, std::string{"/dev/ttyACM0"});
            getParamCompat("io_config/baud_rate", "io_config.baud_rate", serialBaudRate, 115200);
            int postureRepeatCount = postureTxRepeatCount_;
            int postureRepeatIntervalMs = static_cast<int>(postureTxInterval_.count());
            int firecodePartialHoldMs = static_cast<int>(firecodePartialHold_.count());
            double velocityRawToMps = velocityRawToMps_;
            bool rawSerialLogEnable = rawSerialLogEnable_;
            bool rawSerialLogUplink = rawSerialLogUplink_;
            bool rawSerialLogDownlink = rawSerialLogDownlink_;
            bool rawSerialLogScreen = rawSerialLogScreen_;
            bool rawSerialLogFlush = rawSerialLogFlush_;
            std::string rawSerialLogDir = rawSerialLogDir_;
            std::string rawSerialLogTypeIds = rawSerialLogTypeIds_;
            bool rawSerialTopicEnable = rawSerialTopicEnable_;
            bool rawSerialTopicUplink = rawSerialTopicUplink_;
            bool rawSerialTopicDownlink = rawSerialTopicDownlink_;
            std::string rawSerialTopicTypeIds = rawSerialTopicTypeIds_;
            getParamCompat(
                "io_config/posture_repeat_count",
                "io_config.posture_repeat_count",
                postureRepeatCount,
                postureRepeatCount);
            getParamCompat(
                "io_config/posture_repeat_interval_ms",
                "io_config.posture_repeat_interval_ms",
                postureRepeatIntervalMs,
                postureRepeatIntervalMs);
            getParamCompat(
                "io_config/firecode_partial_hold_ms",
                "io_config.firecode_partial_hold_ms",
                firecodePartialHoldMs,
                firecodePartialHoldMs);
            getParamCompat(
                "io_config/velocity_raw_to_mps",
                "io_config.velocity_raw_to_mps",
                velocityRawToMps,
                velocityRawToMps);
            getParamCompat(
                "io_config/raw_serial_log_enable",
                "io_config.raw_serial_log_enable",
                rawSerialLogEnable,
                rawSerialLogEnable);
            getParamCompat(
                "io_config/raw_serial_log_uplink",
                "io_config.raw_serial_log_uplink",
                rawSerialLogUplink,
                rawSerialLogUplink);
            getParamCompat(
                "io_config/raw_serial_log_downlink",
                "io_config.raw_serial_log_downlink",
                rawSerialLogDownlink,
                rawSerialLogDownlink);
            getParamCompat(
                "io_config/raw_serial_log_screen",
                "io_config.raw_serial_log_screen",
                rawSerialLogScreen,
                rawSerialLogScreen);
            getParamCompat(
                "io_config/raw_serial_log_flush",
                "io_config.raw_serial_log_flush",
                rawSerialLogFlush,
                rawSerialLogFlush);
            getParamCompat(
                "io_config/raw_serial_log_dir",
                "io_config.raw_serial_log_dir",
                rawSerialLogDir,
                rawSerialLogDir);
            getParamCompat(
                "io_config/raw_serial_log_type_ids",
                "io_config.raw_serial_log_type_ids",
                rawSerialLogTypeIds,
                rawSerialLogTypeIds);
            getParamCompat(
                "io_config/raw_serial_topic_enable",
                "io_config.raw_serial_topic_enable",
                rawSerialTopicEnable,
                rawSerialTopicEnable);
            getParamCompat(
                "io_config/raw_serial_topic_uplink",
                "io_config.raw_serial_topic_uplink",
                rawSerialTopicUplink,
                rawSerialTopicUplink);
            getParamCompat(
                "io_config/raw_serial_topic_downlink",
                "io_config.raw_serial_topic_downlink",
                rawSerialTopicDownlink,
                rawSerialTopicDownlink);
            getParamCompat(
                "io_config/raw_serial_topic_type_ids",
                "io_config.raw_serial_topic_type_ids",
                rawSerialTopicTypeIds,
                rawSerialTopicTypeIds);

            if (postureRepeatCount <= 0) {
                roslog::warn("Invalid posture_repeat_count=%d, fallback to 3", postureRepeatCount);
                postureRepeatCount = 3;
            }
            if (postureRepeatIntervalMs <= 0) {
                roslog::warn("Invalid posture_repeat_interval_ms=%d, fallback to 20",
                             postureRepeatIntervalMs);
                postureRepeatIntervalMs = 20;
            }
            if (firecodePartialHoldMs <= 0) {
                roslog::warn("Invalid firecode_partial_hold_ms=%d, fallback to 100", firecodePartialHoldMs);
                firecodePartialHoldMs = 100;
            }
            if (velocityRawToMps <= 0.0) {
                roslog::warn("Invalid velocity_raw_to_mps=%f, fallback to 0.025", velocityRawToMps);
                velocityRawToMps = 0.025;
            }

            postureTxRepeatCount_ = postureRepeatCount;
            int sentryCoordSendIntervalMs = static_cast<int>(sentryCoordSendInterval_.count());
            int sentryCoordFieldWidthX = sentryCoordFieldWidthX_;
            int sentryCoordFieldWidthY = sentryCoordFieldWidthY_;
            int sentryCoordFreshTimeoutMs = sentryCoordFreshTimeoutMs_;
            getParamCompat("io_config/sentry_coord_send_interval_ms", "io_config.sentry_coord_send_interval_ms", sentryCoordSendIntervalMs, sentryCoordSendIntervalMs);
            getParamCompat("io_config/sentry_coord_field_width_x", "io_config.sentry_coord_field_width_x", sentryCoordFieldWidthX, sentryCoordFieldWidthX);
            getParamCompat("io_config/sentry_coord_field_width_y", "io_config.sentry_coord_field_width_y", sentryCoordFieldWidthY, sentryCoordFieldWidthY);
            getParamCompat("io_config/sentry_coord_fresh_timeout_ms", "io_config.sentry_coord_fresh_timeout_ms", sentryCoordFreshTimeoutMs, sentryCoordFreshTimeoutMs);
            if (sentryCoordSendIntervalMs < 20) { sentryCoordSendIntervalMs = 20; }
            sentryCoordSendInterval_ = std::chrono::milliseconds(sentryCoordSendIntervalMs);
            sentryCoordFieldWidthX_ = sentryCoordFieldWidthX;
            sentryCoordFieldWidthY_ = sentryCoordFieldWidthY;
            sentryCoordFreshTimeoutMs_ = sentryCoordFreshTimeoutMs;

            postureTxInterval_ = std::chrono::milliseconds(postureRepeatIntervalMs);
            firecodePartialHold_ = std::chrono::milliseconds(firecodePartialHoldMs);
            velocityRawToMps_ = static_cast<float>(velocityRawToMps);
            ConfigureRawSerialLog(
                rawSerialLogEnable,
                rawSerialLogUplink,
                rawSerialLogDownlink,
                rawSerialLogScreen,
                rawSerialLogFlush,
                rawSerialLogDir,
                rawSerialLogTypeIds);
            ConfigureRawSerialTopic(
                rawSerialTopicEnable,
                rawSerialTopicUplink,
                rawSerialTopicDownlink,
                rawSerialTopicTypeIds);
            roslog::warn("posture_tx merged mode: repeat_count=%d repeat_interval_ms=%d",
                         postureTxRepeatCount_,
                         static_cast<int>(postureTxInterval_.count()));
            roslog::warn("semantic control: firecode_partial_hold_ms=%d velocity_raw_to_mps=%.4f",
                         static_cast<int>(firecodePartialHold_.count()),
                         static_cast<double>(velocityRawToMps_));
            roslog::warn("gimbal raw serial log: enable=%s uplink=%s downlink=%s screen=%s dir=%s type_ids=%s",
                         rawSerialLogEnable_ ? "true" : "false",
                         rawSerialLogUplink_ ? "true" : "false",
                         rawSerialLogDownlink_ ? "true" : "false",
                         rawSerialLogScreen_ ? "true" : "false",
                         rawSerialLogDir_.c_str(),
                         rawSerialLogTypeIds_.c_str());
            roslog::warn("gimbal raw serial topic: enable=%s uplink=%s downlink=%s type_ids=%s",
                         rawSerialTopicEnable_ ? "true" : "false",
                         rawSerialTopicUplink_ ? "true" : "false",
                         rawSerialTopicDownlink_ ? "true" : "false",
                         rawSerialTopicTypeIds_.c_str());

            while (rclcpp::ok())
            {
                if (!DeviceError) DeviceError = true;
                std::this_thread::sleep_for(1s);
                if (!Device.Initialize(useVirtualDevice, serialDeviceName, serialBaudRate)) continue;
                DeviceError = false;
                posturePendingRepeat_ = 0;
                postureLastSent_ = 0;
                controlShadow_.SentryCmd.Posture = IsValidPosture(postureCommand_) ? postureCommand_ : 0;
                if (IsValidPosture(postureCommand_)) {
                    ArmPostureTx(postureCommand_);
                }
                
                sub_sentry_position_ = node->create_subscription<geometry_msgs::msg::PointStamped>("/ly/bt/sentry_position", rclcpp::SensorDataQoS(), [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) { if (!msg) return; const auto now = std::chrono::steady_clock::now(); if (!std::isfinite(msg->point.x) || !std::isfinite(msg->point.y)) return; sentryCoordX_ = static_cast<float>(msg->point.x * 100.0); sentryCoordY_ = static_cast<float>(msg->point.y * 100.0); sentryCoordPending_ = true; sentryCoordValid_ = true; sentryCoordLastRxTime_ = now; });

                std::jthread reading{ [this, useVirtualDevice] { useVirtualDevice ? TestVirtualLoopback() : LoopRead(); } };
                while (!DeviceError) {
                    rclcpp::spin_some(node);
                    MaybeApplyFireCodeStaleFallback();
                    MaybeSendPostureTx();
                    MaybeSendSentryCoordinate();
                    rate.sleep();
                }
            }
        }
    };
}

int main(int argc, char** argv) try
{
    RCLCPP_INFO(rclcpp::get_logger("main"), "main: running %s", Application::Name);
    Application app{};
    app.Run(argc, argv);
    return 0;
}
catch (const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("main"), "main: %s", e.what());
    return 1;
}
