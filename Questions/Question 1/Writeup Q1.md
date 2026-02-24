# CODE BREAKDOWN AND ANALYSIS
## Overview

This code is a real-hardware style C++ control module for an EtherCAT-connected servo drive that actuates a mechanical vise through a worm gear & lead screw(as given). Instead of treating the system like an ideal simulator situation, I model the vise operation as an industrial state machine which closes and then detects contact, clamps, holds the object/material being operated on, releases and opens and goes back to the initial position using control strategies that are commonly used in production motion systems like
- smooth jerk-limited motion profiling
- mode switching with confirmation
- noise-robust decision logic for contact/jam detection
- torque stability.

To make the behavior robust on real hardware, the implementation explicitly addresses failure modes you see in actual servo systems like EtherCAT jitter or varying cycle times, sensor noise and single-sample spikes, drive enable/fault handshaking, internal limit saturation, end-stop risk, and unexpected obstacles/jams. The result is a controller that moves aggressively when safe, becomes conservative near contact/end stops, transitions cleanly into a torque-regulated clamp to achieve repeatable force, qualifies success with persistence timers (not one-shot thresholds), and always has safe fallbacks (hold position, ramp down torque, back off on jam).


**In this report/writeup I am going to go through the code and highlight the most important controls and safety concerns I am solving in this code of min and some context/explanations on how I reached these conclusions along with relevant code and diagrams.**

## Low-level utilities

These helpers enforce bounded, deterministic behavior. ClampI32/ClampI16 prevents integer overflow and ensures I never command position/torque outside the controller’s safety limits limits. This is a critical real-hardware property - in a real servo drive, sending out-of-range commands can cause faults, unpredictable behavior, or unsafe motion and mainly lead to noisy feedback. The clamps make the system very robust even if upstream logic has a bug.

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





## Feedback Filters

EtherCAT sensor data is noisy and quantized sometimes, and you can get single-cycle glitches when there is an unexpected misalignment. I use a first-order IIR low-pass (LowPassStep) like most controls engineers would for velocity and torque to smooth readings while keeping latency low. The formulation alpha = dt/(tau+dt) is stable even when dt changes (250 micro-s –2milli-s), which is important because the assesment explicitly allows varying cycle times.

The median-of-3 filter is a targeted robustness tool which rejects single-cycle spikes without adding the heavy phase lag that a very low-cutoff low-pass filter would create and this would significantly affect the detection of contact. This is important for contact detection: you want to detect real contact quickly, but you don’t want one bad sample to falsely trigger “contact detected.” So median-of-3 acts like “spike immunity,” and then the Low pass filter provides a smooth signal for stable decision-making and torque qualification.

    #ifndef M_PI
    #define M_PI 3.14159265358979323846
    #endif
    
    inline double LowPassStep(double y, double x, double dt, double cutoff_hz) {
      if (cutoff_hz <= 0.0) return x; // disabled
      const double tau = 1.0 / (2.0 * M_PI * cutoff_hz);
      const double alpha = dt / (tau + dt);
      return y + alpha * (x - y);
    }
    
    inline double Median3(double a, double b, double c) {
      if (a > b) std::swap(a, b);
      if (b > c) std::swap(b, c);
      if (a > b) std::swap(a, b);
      return b;
    }


## Explicit Hardware Knobs

This is the core of real-time hardware project where all critical behaviors are controlled by explicit parameters rather than hard-coded numbers scattered throughout the code. Real clamp systems require tuning because friction, lubrication, temperature, and load compliance vary. By grouping parameters here, I make the design open to changes and can be calibrated using demo movements(“why 900 mNm for contact threshold?”).

Some important assumptions have also been made explicit like, worm ratio and efficiency are unknown in the prompt, so I expose them as tunable knobs. That is realistic engineering you often start with a nominal estimate (for ex 0.7 efficiency) and then calibrate from measurements. Parameters also encode safety intent - edge zones slow down near end stops, internal-limit scaling backs off if the drive reports saturation, and timeouts prevent the system from applying torque indefinitely if something goes wrong.

Also in this system, I would primarily rely on EtherCAT process data for torque, position, velocity, and status information because modern servo drives internally sample at much higher frequencies (typically 10 – 20 kHz) and expose filtered, calibrated signals over the fieldbus. With a 1 ms EtherCAT cycle, the data latency is acceptable relative to the 15–25 Hz control bandwidth used for clamping. **However, for safety-critical signals such as hard end-stops or emergency stops, I would use direct hardware inputs to a safety-rated controller.** EtherCAT provides sufficient resolution and determinism for control-level decisions, while hardware interlocks provide independent fail-safe protection.

<div align="center">
  <img width="800" height="800" alt="image" src="https://github.com/user-attachments/assets/e6ae4cbc-4b2f-412e-9186-824571ec2432" />
</div>

## Phase definitions

This enum defines the clamp operation as an explicit finite-state machine (FSM), which is the standard industrial approach for sequences involving mode switching, safety transitions, and multiple goals. Each phase has a single responsibility: travel quickly, detect contact, clamp, hold, release, return home. That makes the logic easier to reason about and makes failure handling more deterministic. Separating phases is also important for “real hardware correctness” because different control modes (Position vs Torque) are appropriate at different times. By modeling the phases explicitly, we can ensure the controller never accidentally ramps torque while still in position travel, or keeps creeping forward after contact when it should instead transition into a torque-regulated clamp.

    enum class Phase {
      kIdle,
      kCloseToClampPos,
      kContactDetect,
      kClampRampTorque,
      kClampHoldTorque,
      kReleaseTorque,
      kOpenToHome,
      kJamFault
    };


## Persistent Controlller state

This state structure is what makes the control loop a real control system instead of a purely combinational function. Real controllers must remember phase, timers, filtered signals, and setpoint generator internal states across cycles. For example, the jerk-limited trajectory needs x_cmd/v_cmd/a_cmd to produce a smooth S-curve-like motion; filtering needs previous filter output; contact detection requires persistence timers; torque clamp requires tracking the commanded torque ramp.

    struct State {
      bool inited = false;
      Phase phase = Phase::kIdle;
    
      OperationMode last_req_mode = OperationMode::Position;
      int req_streak = 0;
    
      double x_cmd = 0.0;
      double v_cmd = 0.0;
      double a_cmd = 0.0;
    
      double vel_f_counts_s = 0.0;
      double tq_f_mNm = 0.0;
    
      double tq_hist0 = 0.0, tq_hist1 = 0.0, tq_hist2 = 0.0;
    
      double contact_timer_s = 0.0;
      double torque_stable_timer_s = 0.0;
      double clamp_ramp_timer_s = 0.0;
      double clamp_hold_timer_s = 0.0;
      double jam_timer_s = 0.0;
      double settle_timer_s = 0.0;
    
      double torque_cmd_Nm = 0.0;
    
      bool jam_latched = false;
      int32_t jam_backoff_goal_counts = 0;
    
      int32_t goal_requested_counts = 0;
      int32_t goal_applied_counts = 0;
      double goal_update_timer_s = 0.0;
    };
    
    static State g;

## Mode confirmation manager(very important)

In real drives, switching modes is not an instantaneous and you request a mode and must confirm that the drive has actually entered it before you trust the meaning of commands. If you start sending torque commands while the drive is still in position mode, behavior can be wrong and can mess up the behaviour of the motor. The function enforces a strict policy to always request mode continuously, and only return “confirmed” after both a debounce streak and drive confirmation (operation_mode_display). The streak requirement is important because EtherCAT status words can transiently flicker during enable/disable or while a drive is busy. Requiring consecutive confirmations avoids false positives. This helps protect against issues like bus jitter or brief invalid frames(very commonly experienced in similar robotics applications).

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

## Adaptive jerk optimization

This implements the “optimized and innovative” improvement of instead of using a single jerk limit everywhere, jerk is scheduled continuously based on risk. Far from the target, we allow higher jerk so the system responds quickly(as expected in the assesment Problem Statement); near the target, jerk is reduced so the final approach is gentle and less likely to excite resonances or cause overshoot. This mirrors how many high-end motion controllers behave (“soft landing” motion feel). We also incorporate real-hardware signals into this scheduling: if we’re in an edge zone near end stops, jerk is capped to the most conservative value; if the drive reports an internal limit is active, we scale jerk down further. That means the controller adapts automatically to “warning signs” from hardware and becomes safer without needing a separate fault event add-on!
    
    inline double ScheduledJmaxCounts(double dist_to_goal_counts, bool edge_zone, bool internal_limit) {
      const double d_mm = std::abs(dist_to_goal_counts) / static_cast<double>(kEncoderCountsPerMotorRevolution);
      const double s = std::clamp(d_mm / std::max(1e-3, P.j_scale_mm), 0.0, 1.0);
    
      double j_mm_s3 = P.j_near_mm_s3 + (P.j_far_mm_s3 - P.j_near_mm_s3) * s;
      if (edge_zone) j_mm_s3 = std::min(j_mm_s3, P.j_edge_mm_s3);
      if (internal_limit) j_mm_s3 *= P.internal_limit_scale;
    
      return Mmps3ToCountsps3(j_mm_s3);
    }

## Jerk-limited position setpoint generator (S-curve inspired)

This is the heart of the smooth motion strategy. There are many simple trajectory planning methods like S-curve, trapezoidal etc. From already existing knowledge and current research there are many techniques to plan the trajectory of the vise but using a modified more specific version of some of these trajectory planning algorithms is important! Even though we command the drive in Position Mode, we do not jump the target position abruptly. Instead, we generate a smooth position setpoint(x_cmd) by integrating a jerk-limited acceleration  velocity  position chain. This produces an “S-curve inspired” motion profile: acceleration changes smoothly (limited jerk), velocity ramps up smoothly, and the final approach is stable and gentle.

We also implement the “three phase approach” you wanted: the distance-to-goal thresholds choose different velocity/acceleration limits (fast / medium / creep), and the jerk is continuously scheduled. On top of that, the logic reduces aggressiveness near end stops and when internal limits activate. The result is a controller that moves quickly when it’s safe, but automatically becomes conservative when risk increases — exactly the behavior expected from robust industrial motion systems.

    inline void UpdateJerkLimitedSetpoint(const DriveInputs& in,
                                          double dt,
                                          int32_t goal_counts) {
      const int32_t pos = ClampI32(in.position_actual_counts, kMinPositionCounts, kMaxPositionCounts);
      const int32_t goal = ClampI32(goal_counts, kMinPositionCounts, kMaxPositionCounts);
    
      const int32_t d_counts = goal - static_cast<int32_t>(std::llround(g.x_cmd));
      const double d_mm = std::abs(d_counts) / static_cast<double>(kEncoderCountsPerMotorRevolution);
    
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
    
      const int32_t edge_zone_counts = MmToCounts(P.edge_zone_mm);
      const bool near_min = (pos - kMinPositionCounts) <= edge_zone_counts;
      const bool near_max = (kMaxPositionCounts - pos) <= edge_zone_counts;
      const bool goal_near_min = (goal - kMinPositionCounts) <= edge_zone_counts;
      const bool goal_near_max = (kMaxPositionCounts - goal) <= edge_zone_counts;
      const bool edge_zone = near_min || near_max || goal_near_min || goal_near_max;
    
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
    
      double v_des = (dir == 0) ? 0.0 : (static_cast<double>(dir) * vmax);
    
      const double v_err = v_des - g.v_cmd;
      double a_target = (dt > 0.0) ? (v_err / dt) : 0.0;
      a_target = std::clamp(a_target, -amax, amax);
    
      const double da_max = jmax * dt;
      const double da = std::clamp(a_target - g.a_cmd, -da_max, da_max);
      g.a_cmd += da;
    
      g.v_cmd += g.a_cmd * dt;
      g.v_cmd = std::clamp(g.v_cmd, -vmax, vmax);
    
      g.x_cmd += g.v_cmd * dt;
      g.x_cmd = std::clamp(g.x_cmd,
                           static_cast<double>(kMinPositionCounts),
                           static_cast<double>(kMaxPositionCounts));
    }
    
    

## Torque Ramp Strategy

The ramp rate is automatically high when far from target torque and automatically low near target, which reduces overshoot and improves repeatability across changing friction/compliance. This is much more reusable than a fixed two-stage scheme because it scales to different torque targets without rewriting the logic.

We also implement a protective “soft landing” mechanism: if measured torque overshoots the target by more than a threshold, we immediately stop increasing torque and back off slightly. That prevents dangerous force spikes caused by compliance settling or a sudden change in stiffness. Finally, torque commands are clamped to drive constraints, which is a basic but critical real-hardware safety property.

    inline double ShapedTorqueRampRate(double eT_Nm) {
      const double a = std::clamp(std::abs(eT_Nm) / std::max(1e-6, P.Tscale_Nm), 0.0, 1.0);
      return P.Tdot_slow_Nm_s + (P.Tdot_fast_Nm_s - P.Tdot_slow_Nm_s) * a;
    }
    
    inline void UpdateTorqueCommand(double dt, double T_target_Nm, double T_meas_Nm) {
      const double overshoot = (T_meas_Nm - T_target_Nm);
      if (overshoot > P.torque_overshoot_Nm) {
        g.torque_cmd_Nm = std::max(0.0, g.torque_cmd_Nm - P.torque_backoff_Nm);
        return;
      }
    
      const double eT = T_target_Nm - g.torque_cmd_Nm;
    
      const double Tdot_max = ShapedTorqueRampRate(eT);
      const double dT = std::clamp(eT, -Tdot_max * dt, Tdot_max * dt);
    
      g.torque_cmd_Nm += dT;
    
      const double T_max = static_cast<double>(kMaxTorqueMilliNewtonMeter) * 1e-3;
      const double T_min = static_cast<double>(kMinTorqueMilliNewtonMeter) * 1e-3;
      g.torque_cmd_Nm = std::clamp(g.torque_cmd_Nm, T_min, T_max);
    }
    
## Clamping using time & Safety defaults

This prologue is designed for safe real-hardware behavior. First we clamp 'dt' so even if the scheduler jitters, our filters, ramp increments, and persistence timers remain bounded. Next we set safe defaults before doing anything else, so any early return (faults, not enabled, mode switching) still outputs a safe command (hold position, zero torque/velocity).

The initialization block is “bumpless” because we start the trajectory state (x_cmd) at the measured position. That avoids sudden setpoint jumps that could cause a step command to the drive. We also initialize filter states to the current measured signals to avoid filter startup transients that could mistakenly look like a torque spike or velocity change.

## Fault Handling

This is the real-hardware “don’t trust data” safety layer. If EtherCAT communication fails, input data may be stale or full of errors. The safe policy is to stop motion decisions and simply hold position with zero torque/velocity commands. This prevents runaway motion that could occur if the controller keeps integrating setpoints using old sensor feedback. Also in the code, by returning early if not operation-enabled, we ensure we never try to execute motion logic before the drive is in the correct state. That avoids undefined behavior and makes the control loop deterministic: the high-level phases only run when the hardware is actually ready.
    
      const int32_t pos = ClampI32(inputs.position_actual_counts, kMinPositionCounts, kMaxPositionCounts);
    
      if (inputs.fault_code == static_cast<uint16_t>(FaultCode::Communication)) {
        outputs->operation_mode = static_cast<int8_t>(OperationMode::Position);
        outputs->target_position_counts = pos;
        outputs->target_torque_milli_newton_meter = 0;
        outputs->target_velocity_counts_per_second = 0;
        return;
      }
    
      const bool fault = Bit(inputs.status_word, StatusWordBit::kFault);
      if (fault) {
        outputs->operation_mode = static_cast<int8_t>(OperationMode::Torque);
        outputs->target_torque_milli_newton_meter = 0;
        outputs->target_position_counts = pos;
        outputs->control_word |= ControlWordBit::kFaultReset;
        outputs->control_word |= ControlWordBit::kQuickStopInactive;
        return;
      }
      outputs->control_word |= ControlWordBit::kEnableVoltage;
      const bool ready = Bit(inputs.status_word, StatusWordBit::kReadyToSwitchOn);
      const bool switched_on = Bit(inputs.status_word, StatusWordBit::kSwitchedOn);
      const bool op_enabled = Bit(inputs.status_word, StatusWordBit::kOperationEnabled);
    
      if (ready) outputs->control_word |= ControlWordBit::kSwitchOn;
      if (switched_on) outputs->control_word |= ControlWordBit::kEnableOperation;
    
      if (!op_enabled) return;


## Jam Detection & Jam fault

his is a key real-hardware safety feature: a vise might hit an unexpected obstruction or misaligned part. In that case, continuing to push in position mode can generate excessive force or damage. The jam detector looks for a signature: “still far from goal” but torque is high and velocity is near zero for a persistent duration. That combination indicates that motion has stalled against resistance.

If jam is detected, we latch into a fault behavior and compute a small backoff goal. The backoff reduces force, reduces risk, and prevents the system from “digging in.” In jam fault, we keep behavior simple and safe: go back to Position mode and command a gentle jerk-limited backoff to relieve force. We still use the mode manager to confirm we’re actually in position mode before trusting position commands. That prevents incorrect output interpretation during transitions.

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
        case Phase::kJamFault: {
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
## Approaching the clamping mechanism

This phase handles the “rapid close” requirement in the prompt. We use Position mode but generate a smooth setpoint, so the drive sees a realistic motion profile rather than step-like commands. The motion is fast when far away and becomes conservative as we approach the clamp position due to the distance-based limits and jerk scheduling inside UpdateJerkLimitedSetpoint.

We also integrate safety monitoring during travel: jam detection runs continuously. Finally, we transition to a dedicated contact detection phase once we are within a narrow window around the clamp point, which prevents false contact detection earlier in travel and makes the logic robust and easy to tune.

<div align="center">
  <img width="800" height="800" alt="image" src="https://github.com/user-attachments/assets/0fd0a432-3447-41fa-afe0-7f1d888b8dc9" />
</div>

## Contact Detection(very important)

This phase is designed to detect the moment the vise contacts the plates. Contact detection is done using a robust industrial pattern: check that torque is above a threshold and velocity is low and the condition persists for a minimum time. Using all three conditions prevents false triggers from noise, brief collisions, or transient friction events.

We remain in Position mode during contact detection because it allows a controlled creep motion toward the expected clamp position while we watch torque build-up. Once contact is confirmed, we reset torque-ramp state and transition into torque mode clamping. That gives a clean separation: “motion planning in position mode” and “force regulation in torque mode.”

    case Phase::kContactDetect: {
      const bool mode_ok = ModeConfirmedActive(inputs, outputs, OperationMode::Position);
      outputs->target_torque_milli_newton_meter = 0;
      outputs->target_velocity_counts_per_second = 0;
      if (!mode_ok) {
        outputs->target_position_counts = pos;
        return;
      }

      UpdateJerkLimitedSetpoint(inputs, dt, clamp_counts);
      outputs->target_position_counts = ClampI32(
          static_cast<int32_t>(std::llround(g.x_cmd)),
          kMinPositionCounts, kMaxPositionCounts);

      const bool tq_hi = (std::abs(g.tq_f_mNm) >= P.contact_torque_threshold_mNm);
      const bool v_lo  = (std::abs(vel_f_mm_s) <= P.contact_vel_small_mm_s);

      if (tq_hi && v_lo) g.contact_timer_s += dt;
      else g.contact_timer_s = 0.0;

      if (g.contact_timer_s >= P.contact_persist_s) {
        g.phase = Phase::kClampRampTorque;
        g.torque_cmd_Nm = 0.0;
        g.clamp_ramp_timer_s = 0.0;
        g.torque_stable_timer_s = 0.0;
      }
      return;
    }


## Clamping Mechanism

This phase meets the “controlled clamping phase” requirement. We switch to Torque mode (with confirmation) because torque regulation is the correct way to achieve a repeatable final clamp force when compliance and friction vary(requirement was to maintain an effective clamp force of 12.5 Nm transmitted from the motor to vise). The torque ramp uses the shaped ramp + overshoot safety logic from UpdateTorqueCommand, which makes the approach smooth and reduces the chance of force spikes.

We also implement an industrial-style “torque until stable” success condition. Instead of declaring success the moment torque crosses the threshold, we require that measured torque stays within tolerance for a stable duration (for ex 100 ms which is something we set and modify). This accounts for mechanical settling and sensor noise. Finally, a ramp timeout prevents the system from trying forever - if torque can’t reach target in time, we treat it as abnormal and move to a safe fault behavior.

    case Phase::kClampRampTorque: {
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

      const int32_t cmd_mNm = static_cast<int32_t>(std::llround(g.torque_cmd_Nm * 1000.0));
      outputs->target_torque_milli_newton_meter =
          ClampI16(cmd_mNm, kMinTorqueMilliNewtonMeter, kMaxTorqueMilliNewtonMeter);

      const bool in_band = (std::abs(tq_f_Nm - T_target_motor_Nm) <= P.torque_tol_Nm);
      if (in_band) g.torque_stable_timer_s += dt;
      else g.torque_stable_timer_s = 0.0;

      if (g.torque_stable_timer_s >= P.torque_stable_s) {
        g.phase = Phase::kClampHoldTorque;
        g.clamp_hold_timer_s = 0.0;
      }

      if (g.clamp_ramp_timer_s >= P.clamp_ramp_timeout_s) {
        g.phase = Phase::kJamFault;
        g.jam_latched = true;
        g.jam_backoff_goal_counts = ClampI32(pos + MmToCounts(0.5), kMinPositionCounts, kMaxPositionCounts);
      }

      return;
    }


## Smooth unloading/Torque release

Release is handled carefully to avoid “snap-back” or sudden unloading, which can be unsafe or can shift parts unexpectedly. We ramp torque down at a controlled rate, then snap to zero near the end to prevent numerical dithering. This mirrors how real torque systems unload smoothly to avoid shock loads.

We only transition to the opening motion once torque is effectively zero, which prevents the system from fighting itself (opening while still applying clamp torque). This sequencing matters in real clamp systems because it avoids unnecessary wear and reduces the chance of slipping or damaging the material.

    case Phase::kReleaseTorque: {
      const bool mode_ok = ModeConfirmedActive(inputs, outputs, OperationMode::Torque);
      outputs->target_position_counts = pos;
      outputs->target_velocity_counts_per_second = 0;
      if (!mode_ok) {
        outputs->target_torque_milli_newton_meter = 0;
        return;
      }

      const double eT = 0.0 - g.torque_cmd_Nm;
      const double dT = std::clamp(eT, -P.release_Tdot_Nm_s * dt, P.release_Tdot_Nm_s * dt);
      g.torque_cmd_Nm += dT;

      if (std::abs(g.torque_cmd_Nm) < 0.01) g.torque_cmd_Nm = 0.0;

      const int32_t cmd_mNm = static_cast<int32_t>(std::llround(g.torque_cmd_Nm * 1000.0));
      outputs->target_torque_milli_newton_meter =
          ClampI16(cmd_mNm, kMinTorqueMilliNewtonMeter, kMaxTorqueMilliNewtonMeter);

      if (g.torque_cmd_Nm == 0.0) {
        g.phase = Phase::kOpenToHome;
        g.jam_timer_s = 0.0;
      }
      return;
    }

<div align="center">
  <img width="900" height="500" alt="image" src="https://github.com/user-attachments/assets/3b799a9c-e2db-4ba0-b496-4757a5c5b0b1" />
</div>


## Back to original position

This phase returns the vise safely to fully open position. We again use the jerk-limited setpoint generator so motion is smooth and bounded. Jam detection remains enabled because unexpected resistance can also occur while opening (e.g., debris or mechanical binding).

We use a robust “goal reached” condition: not just “position near goal,” but also “velocity small” and “stable for time.” That avoids declaring success while still moving or oscillating. Once settled, we return to idle and clear start/process flags so the cycle is ready for the next command — a clean industrial-style cycle completion.

    case Phase::kOpenToHome: {
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

      const int32_t err = open_counts - pos;
      const bool pos_close = (AbsI32(err) <= kTargetReachedToleranceCounts);
      const bool vel_small = (std::abs(vel_f_mm_s) <= 5.0);

      if (pos_close && vel_small) g.settle_timer_s += dt;
      else g.settle_timer_s = 0.0;

      if (g.settle_timer_s >= 0.050) {
        g.phase = Phase::kIdle;
        g_start_cycle.store(false, std::memory_order_relaxed);
        g_process_done.store(false, std::memory_order_relaxed);
        g.jam_latched = false;
        g.settle_timer_s = 0.0;
      }
      return;
    }


# Conclusion

This implementation reflects a real-world, hardware-conscious control strategy for an EtherCAT-driven servo system operating a mechanical vise. Rather than relying on simplified trajectory commands or idealized assumptions, the controller is structured as an industrial finite-state machine that explicitly separates motion control, contact detection, force regulation, and safety recovery behaviors.

Throughout the design, emphasis was placed on robustness under real hardware conditions: variable cycle times, EtherCAT jitter, sensor noise, mechanical compliance, friction variability, drive saturation, and unexpected obstruction. Motion is governed by adaptive jerk scheduling and distance-based velocity limits to balance speed and stability. Contact detection is based on multi-condition persistence logic rather than single-sample thresholds. Clamping force is achieved using a shaped torque ramp with overshoot protection and stability qualification. Release and reopening are sequenced carefully to prevent shock loading or control conflicts.

The result is a controller that moves aggressively when safe, transitions smoothly into controlled force regulation, qualifies clamp success based on stability rather than instantaneous thresholds, and always maintains deterministic fallback behavior. The architecture reflects how production clamp systems are designed — prioritizing repeatability, safety, and predictable state transitions over theoretical optimality.

This solution demonstrates not only control theory knowledge, but practical system-level thinking aligned with real industrial servo systems.

