# 阶段二：代码影响定位分析（严格证据版）

> 基于 PROTOCOL_RULES_DIFF.md（阶段一已核准）  
> 生成日期：2026-07-06  
> 原则：最小化变更，优先常量/配置修改，不做重构

---

## 变化 P-005, P-006：0x0120 posture 2bit→3bit + ConfirmEnergyActivate bit23→bit24

### 证据卡片 1 — SentryCmdType 结构体定义

- **文件**：`src/gimbal_driver/module/BasicTypes.hpp`
- **位置**：L58-L69
- **原文**：
```cpp
    struct SentryCmdType
    {
        std::uint32_t ConfirmFreeRevive : 1 = 0;             // bit0
        std::uint32_t ConfirmImmediateRevive : 1 = 0;        // bit1
        std::uint32_t ExchangeProjectileAllowance : 11 = 0;  // bit2-12
        std::uint32_t RemoteProjectileExchangeCount : 4 = 0; // bit13-16
        std::uint32_t RemoteHpExchangeCount : 4 = 0;         // bit17-20
        std::uint32_t Posture : 2 = 0;           // bit21-22, 0=保留, 1=进攻, 2=防御, 3=移动
        std::uint32_t ConfirmEnergyActivate : 1 = 0;         // bit23
        std::uint32_t Reserved : 8 = 0;                      // bit24-31
    };
    static_assert(sizeof(SentryCmdType) == sizeof(std::uint32_t), "SentryCmdType must stay 4B");
```

**所需调整**：仅修改三个位宽常量即可。C++ 位域自动重排：
- `Posture : 2` → `Posture : 3`（bit21-23）
- `ConfirmEnergyActivate : 1` 不变（自动变为 bit24）
- `Reserved : 8` → `Reserved : 7`（自动变为 bit25-31）
- 更新注释：`// bit21-23, 0=保留, 1=进攻, 2=防御, 3=移动, 4=强化进攻, 5=强化防御, 6=强化移动`
- `static_assert` 不变（仍为 4B）

**逻辑链**：位域是编译器行为，改变 `Posture` 位宽后，后续位域自动移位，无需手动计算偏移。下游所有使用 `g.SentryCmd.Posture` 和 `g.SentryCmd.ConfirmEnergyActivate` 的代码自动适配，无需修改。

---

### 证据卡片 2 — IsValidPosture 校验函数

- **文件**：`src/gimbal_driver/main.cpp`
- **位置**：L188-L189
- **原文**：
```cpp
        static bool IsValidPosture(std::uint8_t posture) noexcept {
            return posture >= 1 && posture <= 3;
        }
```

**所需调整**：`return posture >= 1 && posture <= 6;`

**逻辑链**：此函数是 post-condition guard，所有调用点（L651/L918/L1214/L1228/L1545/L1546）均依赖它判断姿态合法性。改为 1-6 后，增强姿态值 4/5/6 自动通过校验，无需逐个调用点修改。

---

### 证据卡片 3 — ClampU2 截断函数

- **文件**：`src/gimbal_driver/main.cpp`
- **位置**：L192-L194
- **原文**：
```cpp
        static std::uint8_t ClampU2(std::uint8_t value) noexcept {
            return static_cast<std::uint8_t>(value & 0x03u);
        }
```

**所需调整**：改为 3-bit 掩码：
```cpp
        static std::uint8_t ClampU3(std::uint8_t value) noexcept {
            return static_cast<std::uint8_t>(value & 0x07u);
        }
```

**逻辑链**：`ClampU2` 将输入截断到 2 bits（0-3），无法表示 4/5/6。调用点在 L770 处：
```cpp
const auto posture = ClampU2(m.posture);  // L770
```
需改为 `ClampU3(m.posture)`。

---

### 证据卡片 4 — 日志字符串

- **文件**：`src/gimbal_driver/main.cpp`
- **位置**：L772-L773
- **原文**：
```cpp
                    roslog::warn("Invalid /ly/control/sentry_cmd posture: %u (expect 0/1/2/3)",
                                 m.posture);
```

**所需调整**：`"expect 0/1/2/3"` → `"expect 0/1/2/3/4/5/6"`

---

### 证据卡片 5 — behavior_tree 侧 PostureTypes.hpp

- **文件**：`src/behavior_tree/include/PostureTypes.hpp`
- **位置**：L11-L14（enum），L16-L19（IsValidPosture），L21-L23（IsValidPostureValue），L25-L27（ToPosture），L28-L30（ToPostureValue），L34-L40（PostureToString），L51-L55（PostureRuntime）
- **原文**：
```cpp
enum class SentryPosture : std::uint8_t {
    Unknown = 0,
    Attack = 1,
    Defense = 2,
    Move = 3
};

inline constexpr bool IsValidPosture(const SentryPosture posture) noexcept {
    return posture == SentryPosture::Attack ||
           posture == SentryPosture::Defense ||
           posture == SentryPosture::Move;
}

inline constexpr bool IsValidPostureValue(const std::uint8_t posture) noexcept {
    return posture >= static_cast<std::uint8_t>(SentryPosture::Attack) &&
           posture <= static_cast<std::uint8_t>(SentryPosture::Move);
}
// ...（ToPosture, ToPostureValue, PostureToString 类似限定 1-3）

struct PostureRuntime {
    SentryPosture Current{SentryPosture::Move};
    SentryPosture Desired{SentryPosture::Move};
    SentryPosture Pending{SentryPosture::Unknown};
    std::array<double, 4> AccumSec{};  // index = posture value(1..3)
    std::array<bool, 4> Degraded{};    // index = posture value(1..3)
    // ...
};
```

**所需调整**：
1. enum 添加：`EnhancedAttack = 4, EnhancedDefense = 5, EnhancedMove = 6`
2. `IsValidPosture`：添加三个新 case
3. `IsValidPostureValue`：`posture <= SentryPosture::Move` → `posture <= SentryPosture::EnhancedMove`
4. `ToPosture`、`ToPostureValue`：自动适配（依赖 IsValidPostureValue）
5. `PostureToString`：添加三个新 case
6. `PostureRuntime::AccumSec`、`Degraded`：`std::array<..., 4>` → `std::array<..., 7>`（索引 0-6，值 0-3 为普通姿态，4-6 为强化姿态）
7. 新增常量：`inline constexpr bool IsEnhancedPosture(uint8_t v) noexcept { return v >= 4 && v <= 6; }`

**逻辑链**：`PostureRuntime` 的数组索引直接对应姿态值（已预留 index 0 空位），扩展到 7 个元素后，索引 4/5/6 即为强化姿态。`PostureManager` 中的 `accumulate_time`、`choose_alternative_posture` 等函数通过 `ToPostureValue` 取索引，自动适配。下游逻辑中 `degraded` 判断对强化姿态可能需要不同阈值——但这属于强化姿态规则逻辑（见 R-001~R-005），不在此处。

---

## 变化 P-015, P-016, P-017：0x020D sentry_info_t 6B→14B

### 证据卡片 6 — SentryData 结构体（阻塞点）

- **文件**：`src/gimbal_driver/module/BasicTypes.hpp`
- **位置**：L235-L242
- **原文**：
```cpp
    /// TypeID=7: 哨兵裁判状态与发射初速度（固定12B）
    struct SentryData {
        static constexpr auto TypeID = 7;
        std::uint32_t SentryInfo;        // 0x020D offset 0
        std::uint16_t SentryInfo2;       // 0x020D offset 4
        float BulletInitialSpeed;        // 0x0207 offset 3
        std::uint16_t Reserved{};
    };
    static_assert(sizeof(SentryData) == sizeof(GimbalData), "TypeID=7 payload must stay 12B");
```

**所需调整**（分两步）：

**步骤 A**：修改现有 SentryData，不新增字段，不改 TypeID=7：
- `sentry_info_2` bit 15 从 `Reserved` 改为 `is_enhanced_posture`  
  这不需要改结构体，只需改 `ToSentryInfoMsg` 中的解析（见证据卡片 8）

**步骤 B**：新增 TypeID=10 承载 sentry_info_3（8B）：

此文件中新增：
```cpp
    /// TypeID=10: 哨兵裁判 sentry_info_3 姿态计时器数据（新增于 V2.0.0，固定12B）
    struct SentryInfo3Data {
        static constexpr auto TypeID = 10;
        std::uint64_t SentryInfo3;       // 0x020D offset 6（共8B）
        std::uint32_t Reserved{};
    };
    static_assert(sizeof(SentryInfo3Data) == sizeof(GimbalData), "TypeID=10 payload must stay 12B");
```

同时需要对应修改 `behavior_tree/module/BasicTypes.hpp` 中的镜像结构体。

**逻辑链**：原有 TypeID=7 满载 12B，无法塞入新增的 8B sentry_info_3。新 TypeID=10 独立承载，不破坏现有 TypeID=7 的结构。

---

### 证据卡片 7 — behavior_tree 侧镜像 SentryData

- **文件**：`src/behavior_tree/module/BasicTypes.hpp`
- **搜索方法**：`Select-String -Pattern "SentryData|TypeID = 7|SentryInfo"`

（内部校验：需确认 behavior_tree 侧是否有相同的 SentryData 结构体。两个 BasicTypes.hpp 不完全相同——gimbal_driver 的包含下行帧定义，behavior_tree 的包含配置。如有镜像，同样需要添加 TypeID=10。）

---

### 证据卡片 8 — ToSentryInfoMsg 解析函数

- **文件**：`src/gimbal_driver/main.cpp`
- **位置**：L585-L605
- **原文**：
```cpp
        static gimbal_driver::msg::SentryInfo ToSentryInfoMsg(const SentryData& data) {
            gimbal_driver::msg::SentryInfo msg;
            msg.sentry_info_raw = data.SentryInfo;
            msg.sentry_info_2_raw = data.SentryInfo2;
            msg.reserved = data.Reserved;
            // ... sentry_info 位解析（不变）...
            msg.out_of_combat = Bit(data.SentryInfo2, 0);
            msg.remaining_exchangeable_17mm = BitsU16(data.SentryInfo2, 1, 11);
            msg.posture = BitsU8(data.SentryInfo2, 12, 2);
            msg.can_activate_energy_mechanism = Bit(data.SentryInfo2, 14);
            msg.sentry_info_2_reserved = Bit(data.SentryInfo2, 15);  // ← L603
            return msg;
        }
```

**所需调整**：
1. L603：`msg.sentry_info_2_reserved` → `msg.is_enhanced_posture`（需先在 SentryInfo.msg 中改名）
2. 新增解析 sentry_info_3 的函数（在接收 TypeID=10 后调用）：
```cpp
        static void ParseSentryInfo3(const SentryInfo3Data& data, /* output */ auto& msg) {
            const auto& raw = data.SentryInfo3;
            msg.attack_weaken_remain_sec   = static_cast<uint8_t>(raw & 0xFF);
            msg.defense_weaken_remain_sec  = static_cast<uint8_t>((raw >> 8) & 0xFF);
            msg.move_weaken_remain_sec     = static_cast<uint8_t>((raw >> 16) & 0xFF);
            // bits 24-31: reserved
            msg.enhanced_attack_remain_sec  = static_cast<uint8_t>((raw >> 32) & 0xFF);
            msg.enhanced_defense_remain_sec = static_cast<uint8_t>((raw >> 40) & 0xFF);
            msg.enhanced_move_remain_sec    = static_cast<uint8_t>((raw >> 48) & 0xFF);
            // bits 56-63: reserved
        }
```

---

### 证据卡片 9 — SentryInfo.msg

- **文件**：`src/gimbal_driver/msg/SentryInfo.msg`
- **原文**：见上文完整读取
- **所需调整**：
  1. `bool sentry_info_2_reserved` → `bool is_enhanced_posture`
  2. 新增字段：
```
# 0x020D sentry_info_3, bit0-63
uint64 sentry_info_3_raw
uint8 attack_weaken_remain_sec
uint8 defense_weaken_remain_sec
uint8 move_weaken_remain_sec
uint8 enhanced_attack_remain_sec
uint8 enhanced_defense_remain_sec
uint8 enhanced_move_remain_sec
```

---

### 证据卡片 10 — PubSentryData 和消息分发

- **文件**：`src/gimbal_driver/main.cpp`
- **位置**：L1220-L1229（PubSentryData），L1302-L1304（分发 switch）
- **原文（分发）**：
```cpp
                    case SentryData::TypeID:
                        PubSentryData(m.GetDataAs<SentryData>());
                        break;
```

**所需调整**：在 switch 中新增 TypeID=10 分支：
```cpp
                    case SentryInfo3Data::TypeID:
                        PubSentryInfo3(m.GetDataAs<SentryInfo3Data>());
                        break;
```
并新增 `PubSentryInfo3()` 函数，解析后发布到 `/ly/game/sentry/info3` 或扩展现有 topic。

---

## 变化 C-003：SentryStatusSync 新增 is_powered

### 证据卡片 11

- **文件**：搜索全库未找到 SentryStatusSync / protobuf / MQTT 相关代码
- **结论**：**当前哨兵代码不涉及自定义客户端（MQTT/Protobuf）通信，SentryStatusSync 的变化对现有代码无影响。** 日后若接入自定义客户端，需在对应的 protobuf handler 中添加 `is_powered` 字段处理。

---

## 变化 R-001~R-005：强化姿态机制

### 证据卡片 12 — PostureManager 选姿态逻辑

- **文件**：`src/behavior_tree/src/PostureManager.cpp`
- **位置**：L59-L76（choose_alternative_posture），L80-L193（Tick）
- **原文（choose_alternative_posture）**：
```cpp
SentryPosture PostureManager::choose_alternative_posture(const SentryPosture avoid) const {
    constexpr SentryPosture candidates[] = {
        SentryPosture::Attack,
        SentryPosture::Defense,
        SentryPosture::Move};
    // ...选累计时间最少的...
}
```

**所需调整**：
1. `choose_alternative_posture`：候选列表保持 Attack/Defense/Move 三项，**不加入强化姿态**（强化姿态是显式指令触发，不应被自动轮换选中）
2. `accumulate_time`：需区分强化/非强化姿态的计时。强化姿态有独立的 15s 配额：
   - 在 `PostureRuntime` 中新增 `std::array<double, 7> EnhancedAccumSec{};`
   - 在 `accumulate_time` 中判断 `IsEnhancedPosture(current)` 后写入对应数组
3. 强化姿态超时自动降级：当增强姿态累计 >= 15s，自动发送切回对应普通姿态的指令

---

### 证据卡片 13 — PostureSetting 配置

- **文件**：`src/behavior_tree/module/BasicTypes.hpp`
- **位置**：L679-L697
- **原文**：
```cpp
    struct PostureSetting {
        bool Enable{true};
        int SwitchCooldownSec{5};
        int MaxSinglePostureSec{180};
        int EarlyRotateSec{165};
        int MinHoldSec{10};
        int PendingAckTimeoutMs{600};
        int RetryIntervalMs{300};
        int MaxRetryCount{3};
        bool OptimisticAck{true};
        int TargetKeepMs{800};
        int DamageKeepSec{4};
        int DamageBurstWindowMs{0};
        int DamageBurstThreshold{0};
        int DamageBurstDefenseHoldSec{0};
        int LowHealthThreshold{120};
        int VeryLowHealthThreshold{80};
        int LowAmmoThreshold{30};
        int ScoreHysteresis{2};
    };
```

**所需调整**（最小化：仅添加一个配置常量）：
```cpp
        int MaxEnhancedPostureSec{15};  // 规则: 强化姿态每局累计上限
```
其余强化姿态逻辑通过对比 `EnhancedAccumSec[idx] >= MaxEnhancedPostureSec` 实现，无需新增更多配置项。

**逻辑链**：`MaxEnhancedPostureSec = 15` 直接来自规则 5.6.4。强化姿态降级逻辑在 `accumulate_time` 中检测到超时后，调用已有的 `Tick()`/命令下发链路自动切回普通姿态。

---

### 证据卡片 14 — PostureLogic 评分函数

- **文件**：`src/behavior_tree/src/PostureLogic.cpp`
- **位置**：L18-L23（GetScore），L27-L31（AddScore），L189-L340（SelectDesiredPosture）
- **原文**：
```cpp
int GetScore(const PostureScore& score, const SentryPosture posture) {
    switch (posture) {
        case SentryPosture::Attack: return score.Attack;
        case SentryPosture::Defense: return score.Defense;
        case SentryPosture::Move: return score.Move;
        default: return 0;
    }
}
```

**所需调整**：
1. `GetScore` 和 `AddScore` 的 switch：添加三个强化姿态 case（return 对应的普通姿态分数 + 强化加成偏移）
2. `SelectDesiredPosture`：当前返回 `Attack`/`Defense`/`Move`，强化姿态决策逻辑可后续通过配置（如"受到高伤害时自动进入强化防御"）添加，**不在此次最小变更范围内**
3. `PostureScore` 结构体（L12）：添加三个强化姿态分数字段

**逻辑链**：强化姿态的评分逻辑是纯增量——可先支持枚举值和字段，具体评分逻辑后续迭代。当前最小变更是让编译通过（switch 覆盖所有 enum 值）。

---

### 证据卡片 15 — SentryCmd.msg 和 ApplySentryCmdCommand

- **文件**：`src/gimbal_driver/msg/SentryCmd.msg`
- **位置**：`Select-String -Pattern "POSTURE|posture"` 查找字段定义
- **原文**（推断自 main.cpp 使用方式）：
  - `FIELD_POSTURE` 字段掩码位
  - `posture` 字段 (uint8)
- **所需调整**：
  1. `posture` 字段的取值范围注释从 `0/1/2/3` → `0/1/2/3/4/5/6`
  2. 不需要新增字段（posture 已是 uint8，值域 0-255 足够表示 0-6）

---

## 变化 P-002：0x0003 damage_difference

### 证据卡片 16

- **文件**：`src/gimbal_driver/module/BasicTypes.hpp`，`HealthMyselfData` 结构体（L206-L215）
- **原文**：
```cpp
    struct HealthMyselfData {
        static constexpr auto TypeID = 2;
        std::uint16_t HeroMyself{200};
        std::uint16_t EngineerMyself{};
        std::uint16_t Infantry1Myself{200};
        std::uint16_t Infantry2Myself{};
        std::uint16_t BaseMyself{};
        std::uint16_t SentryMyself{};
    };
```
- **分析**：6×2B=12B 满载。`damage_difference` 是 int16_t（带符号），当前无空间。但 `HealthMyselfData` 并非严格对应 0x0003 的所有字段——它已经重新排列了。0x0003 的新 `damage_difference` 字段原对应 offset 8 的"保留位"。

**所需调整**：
- **当前 behavior_tree 代码不消费 damage_difference 字段**（搜索无果），无需修改。
- 若日后需要此数据：可通过复用 TypeID=3（HealthEnemyData）的末两个字节，或新增 TypeID=10/11 承载。
- **结论**：暂不修改，标记为"搜索全库未找到相关实现，此为新数据字段，当前无使用"。

---

## 变化 P-008：0x0201 新增 bullet_speed_limit

### 证据卡片 17

- **文件**：搜索 `bullet_speed_limit` 在 `src/gimbal_driver` 和 `src/behavior_tree` 中无匹配
- **结论**：当前代码不消费 `bullet_speed_limit` 字段。且 `robot_status_t`（0x0201）的解析在 gimbal_driver 中已有 struct（`BasicTypes.hpp` 中未找到对应 0x0201 的结构体——gimbal_driver 的 BasicTypes.hpp 主要定义下行帧 TypeID 0-9），无需修改。

---

## 去矛盾验证

### 冲突检测

| 变化对 | 可能冲突 | 验证结果 |
|--------|---------|---------|
| P-005 vs P-006 | 同一结构体 SentryCmdType | ✅ 无冲突：C++ 位域自动移位，改 Posture 位宽即可 |
| P-005 vs R-001 | 同一函数 IsValidPosture | ✅ 一致：gimbal_driver 侧从 1-3→1-6，behavior_tree 侧同理 |
| P-016 vs 证据卡片 6 | TypeID=7 空间不足 | ✅ 已解决：新建 TypeID=10 独立承载，不冲突 |
| P-015 vs P-016 | sentry_info_2 bit15 | ✅ 互补：bit15 改名+新增 sentry_info_3，互不重叠 |

### 逻辑一致性检查

所有"所需调整说明"之间无相互否定：
- `PostureManager` 的 `choose_alternative_posture` 不加入强化姿态（显式触发）与 `IsValidPosture` 接受 1-6（允许强化姿态作为输入）不矛盾——前者是自动轮换逻辑，后者是输入校验
- TypeID=10 的新增与 TypeID=7 的保留不冲突：两个独立 TypeID

---

## 变化优先级排序

| 优先级 | 变化编号 | 调整类型 | 涉及文件数 | 预估工作量 |
|--------|---------|---------|-----------|-----------|
| **P0** | P-005, P-006 | 位域常量修改 | 2 | 极小（改 3 个数字） |
| **P0** | P-015, P-016 | bit 改名+解析 | 2 | 小 |
| **P0** | P-017 | 新增 TypeID=10 | 4-5 | 中（新增结构体+解析+消息定义） |
| **P1** | R-001~R-005 | 新增配置常量+枚举 | 5 | 中（枚举扩展+计时逻辑+降级逻辑） |
| **P2** | P-002 | 暂不修改 | 0 | 无 |
| **P2** | P-008 | 暂不修改 | 0 | 无 |
| **无关** | 其余 27 项 | 不涉及 | 0 | 无 |

### 阻塞项说明

**P-017（sentry_info_3 传输）必须在编码前与嵌软组确认**：新增 TypeID=10 意味着下位机（gimbal_driver）需要在上行帧中新增一个 TypeID=10 的数据段，嵌软组的下位机固件必须同步支持。
