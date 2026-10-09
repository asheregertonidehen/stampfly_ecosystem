# Research RC timing extension v1

Custom research instrumentation on the inspected ecosystem checkout. This is
not a claim about an official 1.0/1.1 release or the firmware currently flashed.
Legacy `rc a b c d`, telemetry fixed blocks, and pilot override remain supported.

Before takeoff, enter SDK mode (`command`), query `rcproto?` (must return exactly
`rc-timing-v1`), then send `rcbegin SESSION` (nonzero unsigned 64-bit decimal).
Beginning a session requires IDLE_GROUND or IDLE_HELD and pins the sender's IP
and UDP source port. `rcid SESSION ID a b c d` uses nonzero increasing uint32 IDs
and four strict integer axes in [-100,100]. It sends no UDP reply. Session/order,
SDK, argument, FLYING, and API guidance gates produce accepted/rejected events.
An ordered rejected ID is consumed; retrying it cannot later apply it. Requests
with unparseable identifiers cannot produce an identifiable event. The ordinary
untagged request/reply protocol still uses its existing serial reply assumption.

Receipt is the software clock immediately after recvfrom returns, not RF arrival.
Acceptance is the stamp immediately before publishing the velocity target.
First use is the start of the first PID computation that selects this request,
not the exact instruction time of the target read or physical motor response.
A target overwritten between cycles can be accepted without any observed use.
Clock words are low 32 bits of esp_timer_get_time (microseconds); first-use zero
is valid when its validity bit is set. RC timeout uses this device clock rather
than an older estimator clock; the intended 500 ms timeout is unchanged.

The binary unified stream adds these little-endian variable entries:

- 0x4C: eight 25-byte cycle records, `<IIIQIB`: compute/record cycle us,
  estimator sample us, first-used us, session64, id32, flags8. Bits: 1 identified,
  2 velocity-guidance path used, 4 timed-out zero reference, 8 PID computed,
  16 first-use stamp valid. Disarmed/custom-controller records do not claim PID
  use. First-use persists while the target is latched, including inactive cycles.
- 0x4D: one 44-byte receipt record, `<IIQI4b4B4f`: received us, accepted us,
  session64, id32, axes right/forward/up/CW yaw, result, flight state, axes-valid,
  reserved zero, actual velocity references forward/right/up m/s and CW rad/s.
  Results 1–7: accepted, no SDK, wrong session/peer, stale ID, not flying,
  no API guidance, bad arguments. Acceptance/velocities are zero for rejection.
- 0x4E: 20-byte info, `<4I4B`: telemetry construction us, identifiable receipt
  count, receipt queue overflow, control log ring overflow, protocol version 1,
  three reserved zeros. Counts are cumulative since boot; startup losses are
  expected while capture is off. Compare first/last values, not absolute zero.

ControlTask freezes IMU and estimator snapshots before computation and logs the
same estimator state passed to the controller. They remain asynchronous samples;
the cycle stamp does not turn them into simultaneous measurements. Motor duty is
the software duty after this cycle's mixer, not measured RPM/thrust. The firmware
controller still receives nominal IMU_DT; the stream now records actual scheduling.

The 32-event FreeRTOS queue uses nonblocking sends and reports drop-newest overflow.
Mandatory cycle/info/duty/output/pilot/reference blocks leave enough room for one
receipt per packet. Emit at most one queued receipt, drain flow increments only
while an entry fits, then append absolute sensor snapshots on change. Deferred
absolute stamps advance only on successful insertion. This avoids blocking the
control task, extra command-response traffic, or popping events/flow that cannot
fit. Payload maximum 1472 bytes avoids fragmentation under a 1500-byte IPv4 MTU.
A receipt with two flow samples and ToF is 1471 bytes; barometer/magnetometer wait
for subsequent packets. Usual 10 Hz RC leaves four 50 Hz packets between receipts;
bursts above sustained 50 events/s overflow eventually and remain visible. Sensor
and network loss outside these counters are still assessed by stream timing and
packet sequences; boundary network loss cannot be certified from sequence gaps.

`make -C firmware/vehicle/test test` runs existing native tests plus the actual PID
regression for parser/gates, stale-estimator underflow, first use, replacement,
timeout, pilot cancellation, mode change, wrap-to-zero, and mandatory byte budget.
The research repository additionally compiles the actual appendEntries body and
wire structs, and runs a local UDP capture/export/replay test. No tests flash or
fly hardware. A complete ESP-IDF ESP32-S3 build is also required after changes.
