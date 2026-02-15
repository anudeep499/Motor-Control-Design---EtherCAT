#pragma once
#include <cstdint>

// -------------------------------------------------
// Timing constraints for the assessment
// -------------------------------------------------
//
// Allowed control loop period range (seconds).
// The harness will call ControlTick(...) once per cycle at the chosen period.
static constexpr double kMinCycleTimeSeconds = 0.000250;  // 250 microseconds
static constexpr double kMaxCycleTimeSeconds = 0.002000;  // 2 milliseconds

// -------------------------------------------------
// Motor and mechanics (test-defined constants)
// -------------------------------------------------

// Torque constant of the motor
// Units: newton-meter per ampere
static constexpr double kMotorTorqueConstantNewtonMeterPerAmpere = 0.0601;

// Lead screw characteristics
// Lead = linear travel per motor revolution
static constexpr double kLeadScrewLeadMillimetersPerRevolution = 1.0;

// Encoder resolution
// Units: pulses per motor revolution (counts used by the drive)
static constexpr int32_t kEncoderCountsPerMotorRevolution = 2048;

// Travel limits (linear position of the vise)
// Units: millimeters
static constexpr double kMinimumPositionMillimeters = 0.0;
static constexpr double kMaximumPositionMillimeters = 40.0;

// -------------------------------------------------
// Signal scaling / limits for evaluation (test-defined)
// -------------------------------------------------

// Position signals (counts)
// - Range: int32_t full range, but harness will clamp/trip on travel limit.
// - Resolution: 1 count.
// - Correspondence: 2048 counts = 1 motor revolution = 1 mm travel.
static constexpr int32_t kMinPositionCounts = static_cast<int32_t>(
    kMinimumPositionMillimeters * kEncoderCountsPerMotorRevolution);
static constexpr int32_t kMaxPositionCounts = static_cast<int32_t>(
    kMaximumPositionMillimeters * kEncoderCountsPerMotorRevolution);

// Velocity signals (counts/second)
// - Range: int32_t full range, but harness enforces a maximum magnitude.
// - Resolution: 1 count per second.
// - Correspondence: 2048 counts/second = 1 revolution/second = 1 mm/second.
static constexpr int32_t kMaxVelocityCountsPerSecond = 200 * kEncoderCountsPerMotorRevolution; // 200 mm/s
static constexpr int32_t kMinVelocityCountsPerSecond = -kMaxVelocityCountsPerSecond;

// Torque signals (millinewton-meter at motor shaft)
// - Range: int16_t full range, but harness enforces a maximum magnitude.
// - Resolution: 1 millinewton-meter.
// - Correspondence: current (amperes) = torque (newton-meter) / torque_constant (newton-meter per ampere).
static constexpr int16_t kMaxTorqueMilliNewtonMeter = 3000;   // 3.000 N·m (test-defined cap)
static constexpr int16_t kMinTorqueMilliNewtonMeter = static_cast<int16_t>(-kMaxTorqueMilliNewtonMeter);

// Torque slew rate limit (millinewton-meter per second)
// - Enforced by harness on commanded target_torque_milli_newton_meter.
// - Effective per-cycle limit: kMaxTorqueSlewMilliNewtonMeterPerSecond * cycle_period_seconds.
static constexpr int32_t kMaxTorqueSlewMilliNewtonMeterPerSecond = 20000; // 20 N·m/s

// Mode switch rate limit
// - Candidate may request mode changes any cycle,
//   but harness will only apply a mode change after it has been requested
//   consistently for this many cycles (debounce).
static constexpr int32_t kModeChangeDebounceCycles = 2;

// -------------------------------------------------
// Control word and status word definitions (explicit values)
// -------------------------------------------------
//
// These are simplified, test-harness-defined bit meanings.
// Candidates must only rely on these definitions.

// Control word (controller -> drive)
namespace ControlWordBit
{
  // Bit 0: Switch On request
  static constexpr uint16_t kSwitchOn = (1u << 0);

  // Bit 1: Enable Voltage (power stage)
  static constexpr uint16_t kEnableVoltage = (1u << 1);

  // Bit 2: Quick Stop inactive (active-low logic)
  // 0 = quick stop active
  // 1 = quick stop inactive
  static constexpr uint16_t kQuickStopInactive = (1u << 2);

  // Bit 3: Enable Operation (accepts setpoints in selected mode)
  static constexpr uint16_t kEnableOperation = (1u << 3);

  // Bit 7: Fault Reset
  static constexpr uint16_t kFaultReset = (1u << 7);
}

// Status word (drive -> controller)
namespace StatusWordBit
{
  // Bit 0: Ready to Switch On
  static constexpr uint16_t kReadyToSwitchOn = (1u << 0);

  // Bit 1: Switched On
  static constexpr uint16_t kSwitchedOn = (1u << 1);

  // Bit 2: Operation Enabled
  static constexpr uint16_t kOperationEnabled = (1u << 2);

  // Bit 3: Fault present
  static constexpr uint16_t kFault = (1u << 3);

  // Bit 4: Voltage Enabled
  static constexpr uint16_t kVoltageEnabled = (1u << 4);

  // Bit 5: Quick Stop Active
  static constexpr uint16_t kQuickStopActive = (1u << 5);

  // Bit 10: Target reached (Position mode only; within tolerance defined below)
  static constexpr uint16_t kTargetReached = (1u << 10);

  // Bit 11: Internal limit active (travel/torque/velocity cap engaged)
  static constexpr uint16_t kInternalLimitActive = (1u << 11);
}

// Target reached tolerance (Position mode)
// - When |position_actual_counts - target_position_counts| <= tolerance,
//   harness sets StatusWordBit::kTargetReached.
static constexpr int32_t kTargetReachedToleranceCounts = 20; // ~0.0098 mm

// -------------------------------------------------
// Control modes (simplified) - explicit numeric values
// -------------------------------------------------
//
// outputs.operation_mode and inputs.operation_mode_display use these exact values.
enum class OperationMode : int8_t
{
  // Value 1: Torque mode
  // Drive interprets target_torque_milli_newton_meter.
  Torque   = 1,

  // Value 2: Velocity mode
  // Drive interprets target_velocity_counts_per_second.
  Velocity = 2,

  // Value 3: Position mode
  // Drive interprets target_position_counts.
  Position = 3
};

// -------------------------------------------------
// Fault codes (drive -> controller) - explicit values
// -------------------------------------------------
enum class FaultCode : uint16_t
{
  None = 0,

  // Exchange() failed or missed cycles (harness may set and latch)
  Communication = 1,

  // Command out of allowed range (position/velocity/torque) or travel exceeded
  CommandLimitViolation = 2,

  // Simulated over-torque / over-current event (based on torque command or measured torque)
  OverTorque = 3,

  // Simulated feedback/encoder fault
  FeedbackFault = 4,

  // Any other fault
  Unknown = 0xFFFF
};

// -------------------------------------------------
// Cyclic process data structures with explicit ranges/resolution
// -------------------------------------------------
struct DriveOutputs   // controller -> drive (sent every cycle)
{
  // control_word:
  // - Valid bits: ControlWordBit::* only. Other bits ignored by harness.
  // - Range: 0..65535, but only bits listed above have meaning.
  uint16_t control_word = 0;

  // operation_mode:
  // - Valid values: 1 (Torque), 2 (Velocity), 3 (Position).
  // - Any other value: harness treats as invalid and latches FaultCode::CommandLimitViolation.
  int8_t operation_mode = 0;

  // target_position_counts:
  // - Used when operation_mode == Position.
  // - Valid range: [kMinPositionCounts, kMaxPositionCounts].
  // - Resolution: 1 count.
  // - Conversion: 2048 counts = 1 mm of travel.
  int32_t target_position_counts = 0;

  // target_velocity_counts_per_second:
  // - Used when operation_mode == Velocity.
  // - Valid range: [kMinVelocityCountsPerSecond, kMaxVelocityCountsPerSecond].
  // - Resolution: 1 count/second.
  // - Conversion: 2048 counts/sec = 1 mm/sec.
  int32_t target_velocity_counts_per_second = 0;

  // target_torque_milli_newton_meter:
  // - Used when operation_mode == Torque.
  // - Valid range: [kMinTorqueMilliNewtonMeter, kMaxTorqueMilliNewtonMeter].
  // - Resolution: 1 mN·m.
  // - Slew limit enforced by harness: kMaxTorqueSlewMilliNewtonMeterPerSecond.
  int16_t target_torque_milli_newton_meter = 0;
};

struct DriveInputs    // drive -> controller (received every cycle)
{
  // status_word:
  // - Bit definitions: StatusWordBit::* above.
  // - Range: 0..65535, but only bits listed above are guaranteed meaningful.
  uint16_t status_word = 0;

  // operation_mode_display:
  // - Echo of the active mode, values: 1, 2, 3.
  // - If the drive is not Operation Enabled, this may still show the last requested mode.
  int8_t operation_mode_display = 0;

  // position_actual_counts:
  // - Range: [kMinPositionCounts, kMaxPositionCounts] during normal operation.
  // - Resolution: 1 count (~0.000488 mm).
  // - Counts increase in the closing direction; decrease in opening direction.
  int32_t position_actual_counts = 0;

  // velocity_actual_counts_per_second:
  // - Range: approximately within [kMinVelocityCountsPerSecond, kMaxVelocityCountsPerSecond]
  //   unless a limit event occurs (then StatusWordBit::kInternalLimitActive may assert).
  // - Resolution: 1 count/second.
  int32_t velocity_actual_counts_per_second = 0;

  // torque_actual_milli_newton_meter:
  // - Range: approximately within [kMinTorqueMilliNewtonMeter, kMaxTorqueMilliNewtonMeter]
  //   unless a limit event occurs.
  // - Resolution: 1 mN·m.
  int16_t torque_actual_milli_newton_meter = 0;

  // fault_code:
  // - One of FaultCode values.
  // - Latches until ControlWordBit::kFaultReset is asserted while Fault bit is set.
  uint16_t fault_code = 0;
};

// -------------------------------------------------
// EtherCAT transport abstraction (stubbed)
// -------------------------------------------------
class EthercatInterface
{
public:
  // Initialize():
  // - Returns true if the drive is discovered and can exchange process data.
  // - On success, the drive starts in "not enabled" state (Voltage disabled).
  bool Initialize();

  // Exchange(outputs, inputs):
  // - Called once per cycle by the harness.
  // - Returns false on communication failure; inputs may be stale in that case.
  // - If communication fails N times consecutively (test-defined), harness may latch FaultCode::Communication.
  bool Exchange(const DriveOutputs& outputs, DriveInputs* inputs);

  // Monotonic time in seconds.
  double GetTimeSeconds() const;

  // Sleep until start_time + cycle_period; returns the actual wake time.
  double SleepUntil(double start_time_seconds,
                    double cycle_period_seconds) const;
};

// -------------------------------------------------
// Helper conversions (provided to avoid guessing)
// -------------------------------------------------
inline int32_t MillimetersToEncoderCounts(double position_mm)
{
  const double revolutions =
      position_mm / kLeadScrewLeadMillimetersPerRevolution;
  return static_cast<int32_t>(
      revolutions * static_cast<double>(kEncoderCountsPerMotorRevolution));
}

inline double EncoderCountsToMillimeters(int32_t counts)
{
  const double revolutions =
      static_cast<double>(counts) /
      static_cast<double>(kEncoderCountsPerMotorRevolution);
  return revolutions * kLeadScrewLeadMillimetersPerRevolution;
}

inline double TorqueMilliNewtonMeterToCurrentAmperes(int16_t torque_mNm)
{
  const double torque_Nm = static_cast<double>(torque_mNm) * 1e-3;
  return torque_Nm / kMotorTorqueConstantNewtonMeterPerAmpere;
}

// -------------------------------------------------
// Candidate-implemented control function
// -------------------------------------------------
// Called exactly once per control cycle.
void ControlTick(const DriveInputs& inputs,
                 DriveOutputs* outputs,
                 double cycle_period_seconds);
