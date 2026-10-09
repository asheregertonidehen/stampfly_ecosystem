// Actual PID computation and API policy, with a deterministic device clock.
#include <cassert>
#include <cstdio>
#include <initializer_list>
#include "research_rc.hpp"
#include "pid_controller.hpp"
#include "data_stream_wire.hpp"
int64_t g_mock_esp_time_us = 1000000;
namespace sf::params {
bool get_float(const char*, float&) { return false; }
bool get_int(const char*, int32_t&) { return false; }
bool set_float(const char*, float) { return false; }
void save() {}
}
int main() {
    using namespace sf;
    TaggedRc rc; uint64_t session;
    assert(parseRcBegin("rcbegin 18446744073709551615", session) && session == UINT64_MAX);
    for (const char* s : {"rcbegin 18446744073709551616", "rcbegin 0", "rcbegin 9 extra"})
        assert(!parseRcBegin(s, session));
    assert(parseTaggedRc("rcid 9 1 -100 100 0 -0", rc) && rc.axes_valid && rc.axes[0] == -100);
    for (const char* s : {"rcid 9 1 - 25 0 0 0", "rcid 9 1 101 0 0 0", "rcid 9 1 1.5 0 0 0", "rcid 9 1 1 0 0", "rcid 9 1 1 0 0 0 extra"})
        assert(parseTaggedRc(s, rc) && !rc.axes_valid);
    assert(!parseTaggedRc("rcid 9 4294967296 0 0 0 0", rc));
    assert(!parseTaggedRc("rcid 0 1 0 0 0 0", rc));
    ResearchRcSession policy; policy.begin(9, 10, 11);
    parseTaggedRc("rcid 9 1 -25 12 0 0", rc);
    assert(policy.classify(rc, 10, 11, false, true, true) == RcResult::NoSdk);
    assert(policy.classify(rc, 10, 12, true, true, true) == RcResult::WrongSession);
    assert(policy.classify(rc, 10, 11, true, true, true) == RcResult::Accepted);
    assert(policy.classify(rc, 10, 11, true, true, true) == RcResult::StaleId);
    rc.id = 2;
    assert(policy.classify(rc, 10, 11, true, false, true) == RcResult::NotFlying);
    assert(policy.classify(rc, 10, 11, true, true, true) == RcResult::StaleId);
    rc.id = 3;
    assert(policy.classify(rc, 10, 11, true, true, false) == RcResult::NoGuidance);
    rc.id = 4; rc.axes_valid = false;
    assert(policy.classify(rc, 10, 11, true, true, true) == RcResult::BadArguments);
    PidController pid; pid.init(); pid.onModeChange(FlightMode::POS_HOLD);
    StateEstimate state{}; state.attitude[0] = 1; state.position[2] = -.5f;
    state.timestamp = 999000; // earlier than acceptance: no unsigned false timeout
    CommandSetpoint sticks{}; GuidanceTarget target{};
    target.mode = 2; target.timestamp = 1000000; target.rc_session = UINT64_MAX; target.rc_id = 7;
    target.vx = .1f; target.vyaw = .2f;
    pid.setGuidanceTarget(target, sticks);
    auto out = pid.compute(state, sticks, .0025f);
    assert(!(out.rc_cycle.flags & RcUsed)); // grounded
    pid.onTakeoffComplete(); g_mock_esp_time_us = 1002500;
    out = pid.compute(state, sticks, .0025f);
    assert(out.rc_cycle.flags == (RcComputed | RcIdentified | RcUsed | RcFirstUseValid));
    assert(out.rc_cycle.first_used_us == 1002500 && out.rc_cycle.command_id == 7);
    assert(out.rate_ref[2] == .2f);
    g_mock_esp_time_us = 1005000; out = pid.compute(state, sticks, .0025f);
    assert(out.rc_cycle.first_used_us == 1002500);
    g_mock_esp_time_us = 1500001; out = pid.compute(state, sticks, .0025f);
    assert(out.rc_cycle.flags & RcTimedOut); assert(out.rate_ref[2] == 0);
    target.timestamp = 1500001; target.rc_id = 8; pid.setGuidanceTarget(target, sticks);
    target.rc_id = 9; pid.setGuidanceTarget(target, sticks); // 8 overwritten before compute
    out = pid.compute(state, sticks, .0025f);
    assert(out.rc_cycle.command_id == 9 && !(out.rc_cycle.flags & RcTimedOut));
    sticks.roll = .9f; out = pid.compute(state, sticks, .0025f);
    assert(!(out.rc_cycle.flags & RcUsed)); // pilot cancellation
    sticks.roll = 0; target.rc_id = 10; target.timestamp = UINT32_MAX - 1000;
    g_mock_esp_time_us = (int64_t{1} << 32);
    pid.setGuidanceTarget(target, sticks); out = pid.compute(state, sticks, .0025f);
    assert(out.rc_cycle.flags & RcFirstUseValid); assert(out.rc_cycle.first_used_us == 0);
    assert(!(out.rc_cycle.flags & RcTimedOut));
    pid.onModeChange(FlightMode::STABILIZE); out = pid.compute(state, sticks, .0025f);
    assert(!(out.rc_cycle.flags & RcUsed));
    LogStreamSample samples[8]{}; datastream::UnifiedPacketBuilder b; b.begin(0, samples);
    datastream::WireRcCycle cycles[8]{}; datastream::WireRcInfo info{};
    datastream::WireDuty400 duty[8]{}; datastream::WireControlOutput400 outputs[8]{};
    datastream::WireControl control{}; datastream::WireCtrlRef ref{}; datastream::WireRcEvent event{};
    assert(b.addEntry(0x4C, cycles, sizeof(cycles)));
    assert(b.addEntry(0x4E, &info, sizeof(info)));
    assert(b.addEntry(0x4A, duty, sizeof(duty)));
    assert(b.addEntry(0x4B, outputs, sizeof(outputs)));
    assert(b.addEntry(0x42, &control, sizeof(control)));
    assert(b.addEntry(0x48, &ref, sizeof(ref)));
    assert(b.addEntry(0x4D, &event, sizeof(event)));
    assert(!b.canAddEntry(sizeof(event))); assert(!b.addEntry(0x4D, &event, sizeof(event)));
    assert(b.finish() == 1438);
    std::puts("Research RC parser/session, actual PID use/timeout/cancel/wrap, packet budget PASS");
}
