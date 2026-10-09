// Research RC protocol v1. Pure helpers: usable in host tests without ESP-IDF.
// RC研究プロトコルv1 — ESP-IDF無しのホスト試験でも使用可能。
#pragma once
#include <cstdint>
#include <cstring>

namespace sf {
enum class RcResult : uint8_t {
    Accepted = 1, NoSdk = 2, WrongSession = 3, StaleId = 4,
    NotFlying = 5, NoGuidance = 6, BadArguments = 7,
};
enum RcCycleFlags : uint8_t {
    RcIdentified = 1, RcUsed = 2, RcTimedOut = 4,
    RcComputed = 8, RcFirstUseValid = 16,
};
struct ResearchRcEvent {
    uint32_t received_us = 0; // Immediately after recvfrom, not RF arrival.
    uint32_t accepted_us = 0;
    uint64_t session_id = 0;
    uint32_t command_id = 0;
    int8_t axes[4] = {};
    uint8_t result = 0;
    uint8_t flight_state = 0;
    uint8_t axes_valid = 0;
    float velocity[4] = {}; // forward/right/up m/s, clockwise yaw rad/s
};
struct ResearchRcCycle {
    uint32_t timestamp = 0; // Actual compute entry time; independent of IMU.
    uint32_t estimate_us = 0; // Estimator sample clock, retained independently.
    uint32_t first_used_us = 0;
    uint64_t session_id = 0;
    uint32_t command_id = 0;
    uint8_t flags = 0;
};
struct ResearchRcCounters { uint32_t received = 0; };
struct TaggedRc {
    uint64_t session = 0;
    uint32_t id = 0;
    int8_t axes[4] = {};
    bool axes_valid = false;
};
inline void rcSkipSpace(const char*& p) {
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
}
inline bool rcUnsigned(const char*& p, uint64_t maximum, uint64_t& value) {
    rcSkipSpace(p); value = 0;
    if (*p < '0' || *p > '9') return false;
    do {
        const uint64_t digit = static_cast<uint64_t>(*p++ - '0');
        if (value > (maximum - digit) / 10) return false;
        value = value * 10 + digit;
    } while (*p >= '0' && *p <= '9');
    return *p == 0 || *p == ' ' || *p == '\t' || *p == '\r' || *p == '\n';
}
inline bool parseRcBegin(const char* text, uint64_t& session) {
    if (std::strncmp(text, "rcbegin ", 8) != 0) return false;
    const char* p = text + 8;
    if (!rcUnsigned(p, UINT64_MAX, session) || session == 0) return false;
    rcSkipSpace(p); return *p == 0;
}
// Return true once identifiers are valid. Invalid axes still produce a reject
// event with axes_valid=0, rather than a fake accepted/clamped request.
inline bool parseTaggedRc(const char* text, TaggedRc& out) {
    out = {};
    if (std::strncmp(text, "rcid ", 5) != 0) return false;
    const char* p = text + 5;
    uint64_t id = 0;
    if (!rcUnsigned(p, UINT64_MAX, out.session) || out.session == 0 ||
        !rcUnsigned(p, UINT32_MAX, id) || id == 0) return false;
    out.id = static_cast<uint32_t>(id);
    for (int i = 0; i < 4; ++i) {
        rcSkipSpace(p);
        const bool negative = *p == '-';
        if (negative) { ++p; if (*p < '0' || *p > '9') return true; }
        uint64_t axis = 0;
        if (!rcUnsigned(p, 100, axis)) return true;
        out.axes[i] = static_cast<int8_t>(negative ? -static_cast<int>(axis) : axis);
    }
    rcSkipSpace(p); out.axes_valid = *p == 0;
    return true;
}
class ResearchRcSession {
public:
    void begin(uint64_t session, uint32_t ip, uint16_t port) {
        session_ = session; ip_ = ip; port_ = port; last_id_ = 0;
    }
    RcResult classify(const TaggedRc& command, uint32_t ip, uint16_t port,
                      bool sdk, bool flying, bool guidance) {
        if (!sdk) return RcResult::NoSdk;
        if (!session_ || command.session != session_ || ip != ip_ || port != port_)
            return RcResult::WrongSession;
        if (command.id <= last_id_) return RcResult::StaleId;
        last_id_ = command.id; // A rejected ID cannot later be replayed as new.
        if (!command.axes_valid) return RcResult::BadArguments;
        if (!flying) return RcResult::NotFlying;
        if (!guidance) return RcResult::NoGuidance;
        return RcResult::Accepted;
    }
private:
    uint64_t session_ = 0;
    uint32_t ip_ = 0, last_id_ = 0;
    uint16_t port_ = 0;
};
} // namespace sf
