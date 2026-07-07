#pragma once

#include <cstdint>
#include <cstddef>
#include "BasicTypes.hpp"

namespace LangYa
{

#pragma pack(push, 1)

// 新通信协议
struct GimbalControlFrame
{
    static constexpr std::uint8_t TYPE_ID = 0x00;
    static constexpr std::uint8_t FRAME_SIZE = 18;

    std::uint8_t HeadFlag{ '!' };
    std::uint8_t TypeID{ TYPE_ID };
    VelocityType Velocity;
    GimbalAnglesType GimbalAngles;
    FireCodeType FireCode;
    // SentryCmd 拆出来发？需要理解一下
    SentryCmdType SentryCmd;
    std::uint8_t Tail{ 0 };
};
static_assert(sizeof(GimbalControlFrame) == 18, "GimbalControlFrame must be 18 bytes");


// 小地图传输 : 不一定要17B, 尝试一次传完
struct SentryCoordinateFrame
{
    static constexpr std::uint8_t TYPE_ID = 0x01;
    static constexpr std::uint8_t FRAME_SIZE = 17;

    std::uint8_t HeadFlag{ '!' };
    std::uint8_t TypeID{ TYPE_ID };
    std::int16_t X_cm{ 0 };
    std::int16_t Y_cm{ 0 };
    std::uint8_t Reserved[10]{ 0 };
    std::uint8_t CRC8{ 0 };
};
static_assert(sizeof(SentryCoordinateFrame) == 17, "SentryCoordinateFrame must be 17 bytes");

#pragma pack(pop)

} // namespace LangYa
