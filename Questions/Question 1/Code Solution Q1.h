#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

#include "../Questions/assessment_interface.h"

// =====================================================
// Optional external command interface (simple + real)
// =====================================================
// If the harness never calls these, code still runs a demo clamp cycle.
namespace {
std::atomic<bool> g_start_cycle{true};       // default true for demo
std::atomic<bool> g_process_done{false};     // if never set, we hold for a timeout
std::atomic<bool> g_user_goal_valid{false};
std::atomic<int32_t> g_user_goal_counts{0};
} // namespace

void StartClampCycle() { g_start_cycle.store(true, std::memory_order_relaxed); }
void SetProcessDone(bool done) { g_process_done.store(done, std::memory_order_relaxed); }

void SetUserGoalMillimeters(double mm) {
  const double mm_clamped = std::clamp(mm, kMinimumPositionMillimeters, kMaximumPositionMillimeters);
  g_user_goal_counts.store(MillimetersToEncoderCounts(mm_clamped), std::memory_order_relaxed);
  g_user_goal_valid.store(true, std::memory_order_relaxed);
}
bool HasUserGoal() { return g_user_goal_valid.load(std::memory_order_relaxed); }

// =====================================================
// Utilities
// =====================================================
namespace {

inline bool Bit(uint16_t w, uint16_t m) { return (w & m) != 0; }
inline int32_t ClampI32(int32_t v, int32_t lo, int32_t hi) { return std::min(std::max(v, lo), hi); }
inline int32_t AbsI32(int32_t v) { return (v >= 0) ? v : -v; }

inline int16_t ClampI16(int32_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return static_cast<int16_t>(v);
}

inline int Sign(double x) { return (x > 0) - (x < 0); }

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 1st-order IIR low-pass: y <- y + alpha*(x-y)
// alpha = dt/(tau+dt), tau = 1/(2*pi*fc)
inline double LowPassStep(double y, double x, double dt, double cutoff_hz) {
  if (cutoff_hz <= 0.0) return x; // disabled
  const double tau = 1.0 / (2.0 * M_PI * cutoff_hz);
  const double alpha = dt / (tau + dt);
  return y + alpha * (x - y);
}

// Median-of-3 helper (spike rejection, no heavy phase lag)
inline double Median3(double a, double b, double c) {
  if (a > b) std::swap(a, b);
  if (b > c) std::swap(b, c);
  if (a > b) std::swap(a, b);
  return b;
}

// =====================================================
// Parameters (tuneable “real hardware knobs”)
// =====================================================
struct Params {
  // Cycle time design target: 1 ms is fine for this scale.
  // (Harness may choose 250us..2ms; we must handle it.)

  // ---- Geometry / sequence targets (Question 1) ----
  double open_mm  = 40.0;
  double clamp_mm = 20.0;

  // ---- Distance-to-goal phase thresholds ----
  double d_far_mm  = 5.0;   // d > 5mm -> fast
  double d_near_mm = 1.0;   // 1mm..5mm -> medium, <1mm -> creep + contact watch

  // ---- Motion limits (counts units) ----
  // max travel speed allowed by harness is 200 mm/s -> keep under that
  double vmax_fast_mm_s   = 140.0;
  double vmax_med_mm_s    = 50.0;
  double vmax_creep_mm_s  = 10.0;

  double amax_fast_mm_s2  = 800.0;
  double amax_med_mm_s2   = 300.0;
  double amax_creep_mm_s2 = 120.0;

  // Jerk scheduling (online adaptive jerk)
  // far -> high jerk, near -> low jerk, edges -> very low jerk
  double j_far_mm_s3   = 8000.0;
  double j_near_mm_s3  = 2000.0;
  double j_edge_mm_s3  = 1200.0;
  double j_scale_mm    = 8.0; // distance scale for jerk blend

  // Edge safety zone
  double edge_zone_mm  = 1.0; // creep near 0/40mm

  // If drive indicates internal saturation limit, back off aggressiveness
  double internal_limit_scale = 0.6;

  // ---- Contact detection near clamp point ----
  // Detect: torque high AND velocity low AND persists
  double contact_watch_window_mm = 2.0;  // only watch for contact when within +/-2mm of clamp target
  double contact_torque_threshold_mNm = 900.0; // tune
  double contact_vel_small_mm_s = 2.0;
  double contact_persist_s = 0.015; // 15ms

  // ---- Clamp torque target (lead screw torque) ----
  // Header torque is motor torque in mNm.
  // Convert lead screw torque -> motor torque using worm ratio + efficiency.
  double target_leadscrew_torque_Nm = 12.5;

  // Because assignment mentions worm gear but not ratio:
  // make ratio/efficiency explicit knobs and document assumptions.
  double worm_ratio = 6.0;     // motor revs per leadscrew rev torque multiplication
  double worm_eff   = 0.70;    // efficiency

  // ---- Torque ramp behavior (A + B + C) ----
  // A) shaped ramp rate (Nm/s)
  double Tdot_fast_Nm_s = 20.0; // fast when far from target
  double Tdot_slow_Nm_s = 3.0;  // slow near target
  double Tscale_Nm      = 4.0;  // blend scale

  // B) stable qualification
  double torque_tol_Nm       = 0.08; // +/- 0.08 Nm
  double torque_stable_s     = 0.100; // must be stable for 100ms

  // C) overshoot soft-landing
  double torque_overshoot_Nm = 0.20; // if overshoot > 0.20Nm -> small backoff + slow re-approach
  double torque_backoff_Nm   = 0.10;

  // Clamp timeouts (real hardware safety)
  double clamp_ramp_timeout_s = 1.0;   // must reach target within 1s
  double clamp_hold_max_s     = 10.0;  // safety cap unless process_done asserted

  // Release ramp down (Nm/s)
  double release_Tdot_Nm_s = 15.0;

  // ---- Filtering ----
  double vel_lpf_hz    = 30.0;
  double torque_lpf_hz = 30.0;
  bool   torque_use_median3 = true;

  // ---- Jam detection during travel (unexpected obstacle) ----
  double jam_torque_threshold_mNm = 1200.0;
  double jam_vel_small_mm_s = 2.0;
  double jam_persist_s = 0.015;
  double jam_min_dist_mm = 3.0;    // only if still > 3mm away from target
  double jam_backoff_mm = 0.5;

  // ---- Goal update rate limiting (if user spams goals) ----
  double goal_update_min_interval_s = 0.050; // 50ms
  double goal_step_max_mm = 5.0;
};

static const Params P{};

// conversions for params
inline int32_t MmToCounts(double mm) { return MillimetersToEncoderCounts(mm); }
inline double CountsToMm(int32_t c)  { return EncoderCountsToMillimeters(c); }

inline double MmpsToCountsps(double mmps) { return mmps * kEncoderCountsPerMotorRevolution; }
inline double Mmps2ToCountsps2(double mmps2) { return mmps2 * kEncoderCountsPerMotorRevolution; }
inline double Mmps3ToCountsps3(double mmps3) { return mmps3 * kEncoderCountsPerMotorRevolution; }

// Convert desired lead screw torque to motor shaft torque (Nm).
inline double LeadScrewTorqueToMotorTorqueNm(double T_leadscrew_Nm) {
  const double denom = std::max(1e-6, (P.worm_ratio * P.worm_eff));
  return T_leadscrew_Nm / denom;
}

// =====================================================
// Phase (Question 1 full sequence)
// =====================================================
enum class Phase {
  kIdle,
  kCloseToClampPos,     // Position mode, jerk-limited
  kContactDetect,       // Position creep + contact check
  kClampRampTorque,     // Torque mode ramp to target
  kClampHoldTorque,     // Torque mode hold until process done / timeout
  kReleaseTorque,       // Torque mode ramp down to 0
  kOpenToHome,          // Position mode to 40mm
  kJamFault             // unexpected jam behavior
};

// =====================================================
// Persistent controller state
// =====================================================
struct State {
  bool inited = false;
  Phase phase = Phase::kIdle;

  // mode manager
  OperationMode last_req_mode = OperationMode::Position;
  int req_streak = 0;

  // jerk-limited command generator state (counts domain)
  double x_cmd = 0.0;
  double v_cmd = 0.0;
  double a_cmd = 0.0;

  // filtered signals
  double vel_f_counts_s = 0.0;
  double tq_f_mNm = 0.0;

  // median(3) buffers for torque
  double tq_hist0 = 0.0, tq_hist1 = 0.0, tq_hist2 = 0.0;

  // dt timers
  double contact_timer_s = 0.0;
  double torque_stable_timer_s = 0.0;
  double clamp_ramp_timer_s = 0.0;
  double clamp_hold_timer_s = 0.0;
  double jam_timer_s = 0.0;
  double settle_timer_s = 0.0;

  // torque command state (motor torque)
  double torque_cmd_Nm = 0.0;

  // jam behavior
  bool jam_latched = false;
  int32_t jam_backoff_goal_counts = 0;

  // goal management (rate-limited)
  int32_t goal_requested_counts = 0;
  int32_t goal_applied_counts = 0;
  double goal_update_timer_s = 0.0;
};

static State g;

// =====================================================
// Mode confirmation manager
// =====================================================
inline bool ModeConfirmedActive(const DriveInputs& in, DriveOutputs* out, OperationMode desired) {
  if (desired != g.last_req_mode) {
    g.last_req_mode = desired;
    g.req_streak = 1;
  } else {
    g.req_streak++;
  }
  out->operation_mode = static_cast<int8_t>(desired);

  const bool streak_ok = (g.req_streak >= kModeChangeDebounceCycles);
  const bool display_ok = (in.operation_mode_display == static_cast<int8_t>(desired));
  return streak_ok && display_ok;
}

// =====================================================
// Online jerk scheduling (continuous)
// =====================================================
inline double ScheduledJmaxCounts(double dist_to_goal_counts, bool edge_zone, bool internal_limit) {
  const double d_mm = std::abs(dist_to_goal_counts) / static_cast<double>(kEncoderCountsPerMotorRevolution);
  // Blend factor s in [0,1]: far => 1, near => 0
  const double s = std::clamp(d_mm / std::max(1e-3, P.j_scale_mm), 0.0, 1.0);

  double j_mm_s3 = P.j_near_mm_s3 + (P.j_far_mm_s3 - P.j_near_mm_s3) * s;
  if (edge_zone) j_mm_s3 = std::min(j_mm_s3, P.j_edge_mm_s3);
  if (internal_limit) j_mm_s3 *= P.internal_limit_scale;

  return Mmps3ToCountsps3(j_mm_s3);
}

// =====================================================
// Jerk-limited S-curve inspired position command update
// (we command position mode, but generate smooth setpoint)
// =====================================================
inline void UpdateJerkLimitedSetpoint(const DriveInputs& in,
                                      double dt,
                                      int32_t goal_counts) {
  const int32_t pos = ClampI32(in.position_actual_counts, kMinPositionCounts, kMaxPositionCounts);
  const int32_t goal = ClampI32(goal_counts, kMinPositionCounts, kMaxPositionCounts);

  const int32_t d_counts = goal - static_cast<int32_t>(std::llround(g.x_cmd));
  const double d_mm = std::abs(d_counts) / static_cast<double>(kEncoderCountsPerMotorRevolution);

  // Distance-to-goal phases choose v/a
  double vmax_mm_s, amax_mm_s2;
  if (d_mm > P.d_far_mm) {
    vmax_mm_s = P.vmax_fast_mm_s;
    amax_mm_s2 = P.amax_fast_mm_s2;
  } else if (d_mm > P.d_near_mm) {
    vmax_mm_s = P.vmax_med_mm_s;
    amax_mm_s2 = P.amax_med_mm_s2;
  } else {
    vmax_mm_s = P.vmax_creep_mm_s;
    amax_mm_s2 = P.amax_creep_mm_s2;
  }

  // Edge zone if near 0/40 now OR goal near 0/40
  const int32_t edge_zone_counts = MmToCounts(P.edge_zone_mm);
  const bool near_min = (pos - kMinPositionCounts) <= edge_zone_counts;
  const bool near_max = (kMaxPositionCounts - pos) <= edge_zone_counts;
  const bool goal_near_min = (goal - kMinPositionCounts) <= edge_zone_counts;
  const bool goal_near_max = (kMaxPositionCounts - goal) <= edge_zone_counts;
  const bool edge_zone = near_min || near_max || goal_near_min || goal_near_max;

  // Internal limit (drive saturating)
  const bool internal_limit = Bit(in.status_word, StatusWordBit::kInternalLimitActive);

  if (edge_zone) {
    vmax_mm_s = std::min(vmax_mm_s, P.vmax_creep_mm_s);
    amax_mm_s2 = std::min(amax_mm_s2, P.amax_creep_mm_s2);
  }
  if (internal_limit) {
    vmax_mm_s *= P.internal_limit_scale;
    amax_mm_s2 *= P.internal_limit_scale;
  }

  const double vmax = MmpsToCountsps(vmax_mm_s);
  const double amax = Mmps2ToCountsps2(amax_mm_s2);
  const double jmax = ScheduledJmaxCounts(d_counts, edge_zone, internal_limit);

  const double e = static_cast<double>(goal) - g.x_cmd;
  const int dir = Sign(e);

  // Desired velocity points toward goal
  double v_des = (dir == 0) ? 0.0 : (static_cast<double>(dir) * vmax);

  // Convert velocity error into acceleration target (clamped)
  const double v_err = v_des - g.v_cmd;
  double a_target = (dt > 0.0) ? (v_err / dt) : 0.0;
  a_target = std::clamp(a_target, -amax, amax);

  // jerk-limited accel update
  const double da_max = jmax * dt;
  const double da = std::clamp(a_target - g.a_cmd, -da_max, da_max);
  g.a_cmd += da;

  // integrate v and x
  g.v_cmd += g.a_cmd * dt;
  g.v_cmd = std::clamp(g.v_cmd, -vmax, vmax);

  g.x_cmd += g.v_cmd * dt;
  g.x_cmd = std::clamp(g.x_cmd,
                       static_cast<double>(kMinPositionCounts),
                       static_cast<double>(kMaxPositionCounts));
}

// =====================================================
// Torque ramp (A + B + C)
// - target torque is motor torque (Nm)
// - outputs expects motor torque in mNm
// =====================================================
inline double ShapedTorqueRampRate(double eT_Nm) {
  const double a = std::clamp(std::abs(eT_Nm) / std::max(1e-6, P.Tscale_Nm), 0.0, 1.0);
  return P.Tdot_slow_Nm_s + (P.Tdot_fast_Nm_s - P.Tdot_slow_Nm_s) * a;
}

inline void UpdateTorqueCommand(double dt, double T_target_Nm, double T_meas_Nm) {
  // Anti-overshoot soft landing:
  const double overshoot = (T_meas_Nm - T_target_Nm);
  if (overshoot > P.torque_overshoot_Nm) {
    // back off slightly and re-approach slowly
    g.torque_cmd_Nm = std::max(0.0, g.torque_cmd_Nm - P.torque_backoff_Nm);
    return;
  }

  const double eT = T_target_Nm - g.torque_cmd_Nm;

  const double Tdot_max = ShapedTorqueRampRate(eT);
  const double dT = std::clamp(eT, -Tdot_max * dt, Tdot_max * dt);

  g.torque_cmd_Nm += dT;

  // Clamp to drive limits (motor)
  const double T_max = static_cast<double>(kMaxTorqueMilliNewtonMeter) * 1e-3;
  const double T_min = static_cast<double>(kMinTorqueMilliNewtonMeter) * 1e-3;
  g.torque_cmd_Nm = std::clamp(g.torque_cmd_Nm, T_min, T_max);
}

} // namespace

// =====================================================
// Main ControlTick
// =====================================================
void ControlTick(const DriveInputs& inputs, DriveOutputs* outputs, double cycle_period_seconds) {
  // 0) dt clamp (real-time robustness)
  const double dt = std::clamp(cycle_period_seconds, kMinCycleTimeSeconds, kMaxCycleTimeSeconds);

  // 1) Safe defaults (always safe even if we return early)
  outputs->control_word = 0;
  outputs->operation_mode = static_cast<int8_t>(OperationMode::Torque);
  outputs->target_torque_milli_newton_meter = 0;
  outputs->target_velocity_counts_per_second = 0;
  outputs->target_position_counts =
      ClampI32(inputs.position_actual_counts, kMinPositionCounts, kMaxPositionCounts);

  outputs->control_word |= ControlWordBit::kQuickStopInactive;

  // 2) Init (bumpless)
  if (!g.inited) {
    g.inited = true;

    const int32_t pos0 = ClampI32(inputs.position_actual_counts, kMinPositionCounts, kMaxPositionCounts);
    g.x_cmd = static_cast<double>(pos0);
    g.v_cmd = 0.0;
    g.a_cmd = 0.0;

    g.vel_f_counts_s = static_cast<double>(inputs.velocity_actual_counts_per_second);
    g.tq_f_mNm = static_cast<double>(inputs.torque_actual_milli_newton_meter);
    g.tq_hist0 = g.tq_hist1 = g.tq_hist2 = g.tq_f_mNm;

    g.phase = Phase::kIdle;
    g.torque_cmd_Nm = 0.0;

    g.goal_requested_counts = pos0;
    g.goal_applied_counts = pos0;
    g.goal_update_timer_s = P.goal_update_min_interval_s; // allow immediate
  }

  const int32_t pos = ClampI32(inputs.position_actual_counts, kMinPositionCounts, kMaxPositionCounts);

  // 3) Communication fault -> safe hold
  if (inputs.fault_code == static_cast<uint16_t>(FaultCode::Communication)) {
    outputs->operation_mode = static_cast<int8_t>(OperationMode::Position);
    outputs->target_position_counts = pos;
    outputs->target_torque_milli_newton_meter = 0;
    outputs->target_velocity_counts_per_second = 0;
    return;
  }

  // 4) Drive fault bit -> safe + reset request
  const bool fault = Bit(inputs.status_word, StatusWordBit::kFault);
  if (fault) {
    outputs->operation_mode = static_cast<int8_t>(OperationMode::Torque);
    outputs->target_torque_milli_newton_meter = 0;
    outputs->target_position_counts = pos;
    outputs->control_word |= ControlWordBit::kFaultReset;
    outputs->control_word |= ControlWordBit::kQuickStopInactive;
    return;
  }

  // 5) Enable handshake
  outputs->control_word |= ControlWordBit::kEnableVoltage;
  const bool ready = Bit(inputs.status_word, StatusWordBit::kReadyToSwitchOn);
  const bool switched_on = Bit(inputs.status_word, StatusWordBit::kSwitchedOn);
  const bool op_enabled = Bit(inputs.status_word, StatusWordBit::kOperationEnabled);

  if (ready) outputs->control_word |= ControlWordBit::kSwitchOn;
  if (switched_on) outputs->control_word |= ControlWordBit::kEnableOperation;

  if (!op_enabled) return;

  // 6) Filtering (LPF) + optional median spike rejection on torque
  g.vel_f_counts_s = LowPassStep(g.vel_f_counts_s,
                                static_cast<double>(inputs.velocity_actual_counts_per_second),
                                dt, P.vel_lpf_hz);

  // update median history
  g.tq_hist2 = g.tq_hist1;
  g.tq_hist1 = g.tq_hist0;
  g.tq_hist0 = static_cast<double>(inputs.torque_actual_milli_newton_meter);

  double tq_raw = g.tq_hist0;
  if (P.torque_use_median3) {
    tq_raw = Median3(g.tq_hist0, g.tq_hist1, g.tq_hist2);
  }

  g.tq_f_mNm = LowPassStep(g.tq_f_mNm, tq_raw, dt, P.torque_lpf_hz);

  const double vel_f_mm_s = g.vel_f_counts_s / static_cast<double>(kEncoderCountsPerMotorRevolution);
  const double tq_f_Nm = (g.tq_f_mNm * 1e-3); // motor torque estimate in Nm

  // 7) Determine if we should run clamp cycle
  const bool start = g_start_cycle.load(std::memory_order_relaxed);

  // If user provided goal, we can override the open/clamp points,
  // but for Q1 we keep the sequence: open->clamp->hold->open.
  const int32_t open_counts  = MmToCounts(P.open_mm);
  const int32_t clamp_counts = MmToCounts(P.clamp_mm);

  // 8) If idle and start requested -> begin closing
  if (g.phase == Phase::kIdle) {
    if (start) {
      g.phase = Phase::kCloseToClampPos;
      g.contact_timer_s = 0.0;
      g.jam_timer_s = 0.0;
      g.torque_stable_timer_s = 0.0;
      g.clamp_ramp_timer_s = 0.0;
      g.clamp_hold_timer_s = 0.0;
      g.torque_cmd_Nm = 0.0;
    } else {
      // hold in position
      outputs->operation_mode = static_cast<int8_t>(OperationMode::Position);
      outputs->target_position_counts = pos;
      return;
    }
  }

  // 9) JAM detection during travel (unexpected obstacle)
  // Only relevant in close/open motion phases (position travel)
  auto MaybeJamDetect = [&](int32_t goal_counts) {
    const int32_t d_counts = AbsI32(goal_counts - pos);
    const double d_mm = CountsToMm(d_counts);

    const bool far = (d_mm > P.jam_min_dist_mm);
    const bool tq_hi = (std::abs(g.tq_f_mNm) >= P.jam_torque_threshold_mNm);
    const bool v_small = (std::abs(vel_f_mm_s) <= P.jam_vel_small_mm_s);

    if (!g.jam_latched && far && tq_hi && v_small) g.jam_timer_s += dt;
    else g.jam_timer_s = 0.0;

    if (!g.jam_latched && g.jam_timer_s >= P.jam_persist_s) {
      g.jam_latched = true;
      g.phase = Phase::kJamFault;

      const int dir = (goal_counts > pos) ? 1 : -1;
      const int32_t backoff = MmToCounts(P.jam_backoff_mm) * dir;
      g.jam_backoff_goal_counts = ClampI32(pos - backoff, kMinPositionCounts, kMaxPositionCounts);
    }
  };

  // =====================================================
  // Phase machine
  // =====================================================
  switch (g.phase) {

    case Phase::kJamFault: {
      // Back off gently and hold.
      const bool mode_ok = ModeConfirmedActive(inputs, outputs, OperationMode::Position);
      outputs->target_torque_milli_newton_meter = 0;
      outputs->target_velocity_counts_per_second = 0;
      if (!mode_ok) {
        outputs->target_position_counts = pos;
        return;
      }
      UpdateJerkLimitedSetpoint(inputs, dt, g.jam_backoff_goal_counts);
      outputs->target_position_counts = ClampI32(
          static_cast<int32_t>(std::llround(g.x_cmd)),
          kMinPositionCounts, kMaxPositionCounts);
      return;
    }

    case Phase::kCloseToClampPos: {
      // Position mode: jerk-limited close toward clamp position (20mm)
      const bool mode_ok = ModeConfirmedActive(inputs, outputs, OperationMode::Position);
      outputs->target_torque_milli_newton_meter = 0;
      outputs->target_velocity_counts_per_second = 0;
      if (!mode_ok) {
        outputs->target_position_counts = pos;
        return;
      }

      MaybeJamDetect(clamp_counts);

      UpdateJerkLimitedSetpoint(inputs, dt, clamp_counts);
      outputs->target_position_counts = ClampI32(
          static_cast<int32_t>(std::llround(g.x_cmd)),
          kMinPositionCounts, kMaxPositionCounts);

      // Transition: when within contact-watch window, go to contact detect
      const double d_mm = std::abs(CountsToMm(clamp_counts - pos));
      if (d_mm <= P.contact_watch_window_mm) {
        g.phase = Phase::kContactDetect;
        g.contact_timer_s = 0.0;
      }
      return;
    }

    case Phase::kContactDetect: {
      // Creep in position mode while looking for contact signature
      const bool mode_ok = ModeConfirmedActive(inputs, outputs, OperationMode::Position);
      outputs->target_torque_milli_newton_meter = 0;
      outputs->target_velocity_counts_per_second = 0;
      if (!mode_ok) {
        outputs->target_position_counts = pos;
        return;
      }

      // Keep creeping toward clamp position with jerk-limited setpoint
      UpdateJerkLimitedSetpoint(inputs, dt, clamp_counts);
      outputs->target_position_counts = ClampI32(
          static_cast<int32_t>(std::llround(g.x_cmd)),
          kMinPositionCounts, kMaxPositionCounts);

      // Contact detection: torque high + velocity low + persistence
      const bool tq_hi = (std::abs(g.tq_f_mNm) >= P.contact_torque_threshold_mNm);
      const bool v_lo  = (std::abs(vel_f_mm_s) <= P.contact_vel_small_mm_s);

      if (tq_hi && v_lo) g.contact_timer_s += dt;
      else g.contact_timer_s = 0.0;

      if (g.contact_timer_s >= P.contact_persist_s) {
        // Contact detected -> clamp in torque mode
        g.phase = Phase::kClampRampTorque;
        g.torque_cmd_Nm = 0.0;
        g.clamp_ramp_timer_s = 0.0;
        g.torque_stable_timer_s = 0.0;
      }
      return;
    }

    case Phase::kClampRampTorque: {
      // Torque mode: ramp motor torque to target leadscrew torque (converted)
      const bool mode_ok = ModeConfirmedActive(inputs, outputs, OperationMode::Torque);
      outputs->target_position_counts = pos;
      outputs->target_velocity_counts_per_second = 0;
      if (!mode_ok) {
        outputs->target_torque_milli_newton_meter = 0;
        return;
      }

      g.clamp_ramp_timer_s += dt;

      const double T_target_motor_Nm = LeadScrewTorqueToMotorTorqueNm(P.target_leadscrew_torque_Nm);
      UpdateTorqueCommand(dt, T_target_motor_Nm, tq_f_Nm);

      // Command torque (mNm)
      const int32_t cmd_mNm = static_cast<int32_t>(std::llround(g.torque_cmd_Nm * 1000.0));
      outputs->target_torque_milli_newton_meter =
          ClampI16(cmd_mNm, kMinTorqueMilliNewtonMeter, kMaxTorqueMilliNewtonMeter);

      // “Torque until stable” qualification
      const bool in_band = (std::abs(tq_f_Nm - T_target_motor_Nm) <= P.torque_tol_Nm);
      if (in_band) g.torque_stable_timer_s += dt;
      else g.torque_stable_timer_s = 0.0;

      if (g.torque_stable_timer_s >= P.torque_stable_s) {
        g.phase = Phase::kClampHoldTorque;
        g.clamp_hold_timer_s = 0.0;
      }

      // Timeout -> safe fallback (treat like jam/fault)
      if (g.clamp_ramp_timer_s >= P.clamp_ramp_timeout_s) {
        g.phase = Phase::kJamFault;
        g.jam_latched = true;
        g.jam_backoff_goal_counts = ClampI32(pos + MmToCounts(0.5), kMinPositionCounts, kMaxPositionCounts);
      }

      return;
    }

    case Phase::kClampHoldTorque: {
      // Hold target torque; exit when process done or hold timeout
      const bool mode_ok = ModeConfirmedActive(inputs, outputs, OperationMode::Torque);
      outputs->target_position_counts = pos;
      outputs->target_velocity_counts_per_second = 0;
      if (!mode_ok) {
        outputs->target_torque_milli_newton_meter = 0;
        return;
      }

      g.clamp_hold_timer_s += dt;

      const double T_target_motor_Nm = LeadScrewTorqueToMotorTorqueNm(P.target_leadscrew_torque_Nm);
      // Maintain torque using the same shaped ramp update (small corrections)
      UpdateTorqueCommand(dt, T_target_motor_Nm, tq_f_Nm);

      const int32_t cmd_mNm = static_cast<int32_t>(std::llround(g.torque_cmd_Nm * 1000.0));
      outputs->target_torque_milli_newton_meter =
          ClampI16(cmd_mNm, kMinTorqueMilliNewtonMeter, kMaxTorqueMilliNewtonMeter);

      const bool done = g_process_done.load(std::memory_order_relaxed);
      if (done || g.clamp_hold_timer_s >= P.clamp_hold_max_s) {
        g.phase = Phase::kReleaseTorque;
      }
      return;
    }

    case Phase::kReleaseTorque: {
      // Torque mode: ramp down to 0 smoothly
      const bool mode_ok = ModeConfirmedActive(inputs, outputs, OperationMode::Torque);
      outputs->target_position_counts = pos;
      outputs->target_velocity_counts_per_second = 0;
      if (!mode_ok) {
        outputs->target_torque_milli_newton_meter = 0;
        return;
      }

      // Ramp torque toward 0 at release rate
      const double eT = 0.0 - g.torque_cmd_Nm;
      const double dT = std::clamp(eT, -P.release_Tdot_Nm_s * dt, P.release_Tdot_Nm_s * dt);
      g.torque_cmd_Nm += dT;

      // Snap near zero
      if (std::abs(g.torque_cmd_Nm) < 0.01) g.torque_cmd_Nm = 0.0;

      const int32_t cmd_mNm = static_cast<int32_t>(std::llround(g.torque_cmd_Nm * 1000.0));
      outputs->target_torque_milli_newton_meter =
          ClampI16(cmd_mNm, kMinTorqueMilliNewtonMeter, kMaxTorqueMilliNewtonMeter);

      if (g.torque_cmd_Nm == 0.0) {
        // Go open to home (40mm)
        g.phase = Phase::kOpenToHome;
        g.jam_timer_s = 0.0;
      }
      return;
    }

    case Phase::kOpenToHome: {
      // Position mode: open back to 40mm using jerk-limited setpoint
      const bool mode_ok = ModeConfirmedActive(inputs, outputs, OperationMode::Position);
      outputs->target_torque_milli_newton_meter = 0;
      outputs->target_velocity_counts_per_second = 0;
      if (!mode_ok) {
        outputs->target_position_counts = pos;
        return;
      }

      MaybeJamDetect(open_counts);

      UpdateJerkLimitedSetpoint(inputs, dt, open_counts);
      outputs->target_position_counts = ClampI32(
          static_cast<int32_t>(std::llround(g.x_cmd)),
          kMinPositionCounts, kMaxPositionCounts);

      // done when close + low velocity for a little time
      const int32_t err = open_counts - pos;
      const bool pos_close = (AbsI32(err) <= kTargetReachedToleranceCounts);
      const bool vel_small = (std::abs(vel_f_mm_s) <= 5.0);

      if (pos_close && vel_small) g.settle_timer_s += dt;
      else g.settle_timer_s = 0.0;

      if (g.settle_timer_s >= 0.050) { // 50ms settle
        g.phase = Phase::kIdle;
        g_start_cycle.store(false, std::memory_order_relaxed);
        g_process_done.store(false, std::memory_order_relaxed);
        g.jam_latched = false;
        g.settle_timer_s = 0.0;
      }
      return;
    }

    default:
      break;
  }

  // If we fall through unexpectedly, hold safe.
  outputs->operation_mode = static_cast<int8_t>(OperationMode::Position);
  outputs->target_position_counts = pos;
  outputs->target_torque_milli_newton_meter = 0;

  // Re-assert enable bits
  outputs->control_word |= ControlWordBit::kQuickStopInactive;
  outputs->control_word |= ControlWordBit::kEnableVoltage;
  if (ready) outputs->control_word |= ControlWordBit::kSwitchOn;
  if (switched_on) outputs->control_word |= ControlWordBit::kEnableOperation;
}

