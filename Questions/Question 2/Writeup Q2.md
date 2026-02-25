# **QUESTION 1**
# What is Frequency Response & what does it represent

When we talk about frequency response in a real system like our motor-driven vise, what we’re really asking is:

- If I keep “wiggling” the input at different speeds, how does the hardware respond?


Instead of commanding a simple move to 20 mm, imagine we apply a small sinusoidal torque woth some apmplitude and some frequency. So like let's say we are Torque mode and want to set some amplitude and some frequency at which a sinusoidal input is sent to the system to see how the vise outputs this input, cyclic frequency. To determine how strongly the vise reacts to this signal and how instantaneously it reacts to the input is what the Magnitude and Phase response measure respectively.


Now if we test different frequencies.

- Let’s assume:
- EtherCAT loop approx 1 ms
- Effective total delay approx 3 ms
- Mechanical resonance around 80 Hz(characteristic property of the system and the materials present)

If I apply a 5 Hz oscillation (Time period = 0.2 seconds), the system responds smoothly. The 3 ms delay is tiny compared to 200 ms, so the vise behaves almost like a rigid body. Everything feels stable and predictable and proportional to the input sinusoidal we sent.

If I move up to around 80 Hz (Time period = 12.5 ms), things change. Now the compliance in the lead screw and jaws starts to matter. Small torque variations might produce noticeable vibration. This is where structural resonance shows up and causes instability and is an important part of understanding the safety standards of the system.

At 200 Hz, the motor and screw simply can’t react fast enough. The inertia dominates and the output barely moves. The system naturally filters out those very fast changes.

So frequency response is basically a structured way of understanding how your real mechanical system behaves dynamically & not just in slow moves, but across different excitation speeds. Understanding these parameters are instrumental in designing the approach we take to ramping the position/torque up while clamping or while approaching the clamp position and helps us understand how reactive and sensitive the system is and how to avoid sending it into failure modes!




# What is Magnitude Response & what does it represent

Magnitude response tells us how big the output is compared to the input at each frequency.

Let’s say we measure
- 5 Hz -> 0 dB (normal tracking)
- 75 Hz -> 8 dB (about 2.5× amplification)
- 200 Hz -> − 20 dB (very small response)

At 5 Hz, the vise tracks nicely and behaves as expected. So the input amplitude for that frequency is maintained and it oscillates with the same intensity and is easily controllable by our controller.

At 75 Hz, we see amplification. That means a small torque ripple creates a larger motion response. That’s resonance where energy getting stored and released by the screw and structure. The output movement of the vise moves to greater extremes than what the input signal did for the same frequency and hence it is a cocern for safety and obtaining control of the vise is very difficult because the amplitude is so high.

At 200 Hz, the system can’t respond fast enough, so the output is attenuated. The input signal is too fast to mimic for the output vise movement it just loses amplitude and develops other problems we discuss next in the explanation.

This is extremely important for tuning. If your control bandwidth overlaps with that 75 Hz resonance region, your controller might actually amplify vibration instead of controlling it. So any noise and slight disturbances might be blown up into something that affects the controls of the system.

That’s why engineers follow a common rule of thumb:

- Keep the control bandwidth around 1/3 to 1/5 of the first resonance frequency.
- If resonance is at 75 Hz, a safe bandwidth would be somewhere around 15–25 Hz.
- So magnitude response tells you where the system behaves nicely and where it becomes risky.






# Phase response & why it's so important

Phase response tells us how delayed the output is compared to the input at each frequency.

This delay comes from a few sources and some of them are:
- Inertia
- Compliance
- Filtering
- EtherCAT communication
- Computation time

Using the delay formula:

      PHASE LAG = -360 * FREQUENCY * TIME LAG

If total delay is about 3 ms:

- 5 Hz meant about −5 degrees of phase lag compared to the input
- 20 Hz  meant about −22 degrees of phase lag compared to the input
- 50 Hz meant about −54 degrees of phase lag compared to the input
- 100 Hz meant about −108 degrees of phase lag compared to the input

So as frequency increases, even a fixed delay starts looking like significant phase lag because at higher frequencies the lag has scope to creep more into the systems performance as it makes it difficult for the system to maintain vise movement at higher frequencies!

Now here’s the key point for stability:

If gain crossover happens at 20 Hz and phase there is −130, you have about 50 phase margin & that’s fine and comfortable. If you increase gain and crossover shifts to 40 Hz where phase is −170, now you only have about 10 phase margin so that’s risky and likely to oscillate and create instabilities and these are the oscillations we take care as controls engineers. **So phase response directly tells you how close you are to instability.**


# Implementation in Real Hardware

In a real system like the vise, we don’t guess stability — we measure it. To characterize frequency response, we inject a small sinusoidal torque input (for example +/- 0.2 Nm) and sweep frequency from 1 Hz to 200 Hz while measuring velocity or position. Using Fast Fourier Transforms, we compute the magnitude ratio and phase shift at each frequency and plot a Bode plot.

From this, we might identify:
- First resonance at ~ around 75 Hz
- Gain crossover frequency at which the magnitude intersects the X axis
- Phase margin from the phase crossover angle(-180)
- Noise amplification above 150 Hz

If resonance is 75 Hz, we typically keep control bandwidth around 15–25 Hz (about 1/3 to 1/5 of resonance). This prevents exciting structural vibration in the screw and jaws, which would otherwise cause chatter or long-term wear during 12.5 Nm clamping.

This approach improves reliability because we avoid operating near resonance, reducing mechanical stress and vibration. It also improves serviceability — if, over time, resonance shifts (say from 80 Hz to 70 Hz due to wear or lubrication changes), re-measuring frequency response immediately reveals it. That makes it both a tuning tool and a diagnostic tool. We Students/Engineers typically determine frequency response by applying sine sweeps, chirp signals, or PRBS excitation and analyzing the output using FFT tools (MATLAB, Python, etc.). This gives a data-driven way to tune gains instead of relying on trial-and-error.

**So frequency and magnitude response are not abstract theory — they are practical tools that allow us to design a stable, robust, and long-lasting clamp system.**

The General Theoretical approach would be:

Derive the system transfer function G(s) from the motor inertia, damping, stiffness, and load dynamics. Substitute s = jw to obtain the frequency response G(jw).

Compute:

      Magnitude response = |G(jw)|
      Phase response = angle G(jw)

Plot magnitude (in dB) and phase versus frequency (log scale) to generate the Bode plot.

Use the same G(jw) to construct the Nyquist plot by plotting the real part versus imaginary part of G(jw).

Use the characteristic equation:

      1 + K G(s) = 0

to analyze Root Locus and observe how closed-loop poles move as gain K varies.

# **QUESTION 2**

System states are the minimum set of internal variables that completely describe the dynamic condition of a system at any given moment. In a motor-driven vise, the primary states include:
- position
- velocity
- motor torque (or current).

These states define how the system is moving and how it will evolve next. If we know the current states, we can mathematically predict future behavior using the system model. For example, if the vise is at 20 mm but still moving inward at high velocity, we can predict overshoot. If it is at 20 mm with near-zero velocity and rising torque, we can predict contact and force buildup. States essentially capture the “dynamic memory” of the system. Just like how we use states to apply Markov Decision process algorithms and how we implement state action algorithms similarly we use current states to understand future performance and accuracy.

In the context of the clamping problem, states are directly used for control logic and safety decisions:

- Position state -> determines distance to clamp target
- Velocity state -> helps detect contact (velocity drops while torque increases)
- Torque state -> verifies that 12.5 Nm clamp force is achieved
- **Derived quantities -> acceleration, power, stiffness estimation(very important parameters especially low level control!)**

For example, during contact detection, the controller checks whether torque rises while velocity decreases & this combination of states indicates physical contact. During torque hold, stable torque over time confirms proper clamping. These states also allow prediction: if velocity is high near the clamp point, the controller can reduce aggressiveness to avoid overshoot.

Accurate state measurement or estimation is critical, especially during the final clamping phase where force precision matters. If torque feedback is noisy or biased, the system may under-clamp or over-clamp. If velocity estimation is poor, contact may be detected incorrectly. When states cannot be directly measured (for example, internal compliance deflection), observers or filters are used to estimate them reliably. Root locus analysis helps determine how these state dynamics behave as controller gain increases. Gain represents how strongly the controller reacts to state error. If gain is too high, system poles move toward instability and oscillations occur; if properly selected, the states remain stable and well-damped. Therefore, accurate state knowledge combined with appropriate gain tuning ensures stable, predictable, and safe clamp operation.

# **QUESTION 3 & 4**

Under-sampling in a control system occurs when the digital controller samples the plant too slowly relative to the system’s dynamic movements. In a real motor-driven vise, the analog physical signals (torque, position, velocity) are converted into digital values using an ADC, processed in discrete time, and sent back through a D/A or digital drive interface. Once sampling occurs, the system no longer sees a continuous signal — it sees discrete snapshots spaced by dt. If the sampling frequency fs is too low relative to the highest significant mechanical frequency, the discrete system misrepresents the real dynamics. So for example a system with frequency 30 Hz and you sample it at a frequency 90 Hz, we are accurately able to determine the dynamics of the system but if it were moving at a frequency of 60 Hz then maybe our sampling frequency would not be sufficient and might mislead our control system to thinking the system is moving at a different frequency. According to the Nyquist Sampling Theorem, we require:

      fs >= 2 * f_max

where f_max is the highest frequency present in the signal. However, in control systems, simply satisfying Nyquist is not sufficient — we typically require 5–10 times margin for stability and phase preservation.

From a time-domain perspective, aliasing means the sampled waveform no longer resembles the true oscillation. For example, if the vise has a mechanical resonance at 60 Hz and we sample at 90 Hz, the Nyquist frequency is 50 Hz. The 60 Hz oscillation cannot be represented correctly and “folds” into a lower apparent frequency. The aliased frequency is:

      f_alias = mod(f_signal − n * fs)


So 60 Hz sampled at 90 Hz appears as 30 Hz. In the frequency domain, this is called spectral folding. The controller now believes there is a 30 Hz disturbance — dangerously close to the intended control bandwidth. This can cause the controller to amplify vibration instead of damp it, leading to chatter during clamping.

Under-sampling also introduces effective delay. Digital control behaves approximately like it has half a sample period of delay. If:
- sampling period dt = 10 ms
- effective delay approximately equals 5 ms

The phase lag added is:

      phase_lag = -360 * f * T_delay

At 30 Hz crossover, this equals approximately -36 degrees, significantly reducing phase margin. Reduced phase margin shifts closed-loop poles toward instability towrds the -180 degree mark as seen in Root Locus & Bode plot), potentially causing oscillations in torque hold. Additionally, coarse sampling degrades state estimation. Velocity calculated as:

      v[k] = ( x[k] - x[k-1] ) / dt

becomes noisy when dt is large, which compromises contact detection and clamp stability.

For the vise described earlier (resonance approximately 60 Hz, torque bandwidth approximately 20 Hz), proper sampling criteria would include:
- Sampling >= 10 * control bandwidth, meaning >= 300 Hz
- Preferably >= 5 to 10 * resonance frequency, meaning 300 to 600 Hz
- Practical choice: 1 kHz EtherCAT loop
- Use anti-aliasing filtering before ADC conversion
- Keep total delay small enough to preserve >= 45 degrees phase margin

This ensures the controller operates safely within the baseband (the frequency region of interest below resonance) and avoids spectral folding.

Other techniques I've used/worked with in the past are:

- **Zero-Order Hold (ZOH)**: The digital output stage holds the control signal between samples, which introduces additional high-frequency phase lag and reduces phase margin.

- **Discrete-Time Pole Mapping:** Continuous poles map to discrete poles using
      z = exp(s * T).
  If the sampling period T is too large, poles move closer to the unit circle, reducing damping and potentially causing instability.
- **Computational Jitter:** EtherCAT timing jitter behaves like variable delay, which further reduces phase margin and can cause oscillations or limit cycles in torque hold.

- **Anti-Aliasing Filter Design:** The analog filter cutoff should be below Nyquist, typically around 0.4 to 0.5 times the sampling frequency, balancing noise attenuation with added phase lag.




# Question 5


**Feedforward and feedback** are two complementary control strategies. Feedback control measures the output, compares it to a reference, and corrects any error. It is reactive, instantaneous and ensures stability even when disturbances or modeling errors exist. 
**Feedforward control**, on the other hand, predicts the required input based on a model of the system and applies it directly without waiting for error. It is proactive and improves performance when the system dynamics and information about efficiencies are calibrated and measured.

In the motor-driven vise, **Feedback control** is used during torque clamping. For example:

- Desired clamp torque = 12.5 Nm
- Measured torque = 11 Nm
- Error = +1.5 Nm

The controller increases motor current to eliminate the error. Feedback is essential here because plate stiffness, friction, and compliance vary. Without feedback, the system could under-clamp or over-clamp. So when we are in a situation to provide enough weight to the prediction of the model we operate using real-time error values and gradually reduce those errors and reach the required goal state.

However, feedback alone can be slow and may introduce oscillations if gains are too high!

Feedforward can be used during motion phases. For example:

If the leadscrew pitch and efficiency are known, We can calculate the approximate motor torque needed to generate 12.5 Nm at the screw.

Similarly, during position motion:

We can add inertia compensation:

      torque_ff = J * desired_acceleration

This reduces lag and minimizes tracking error before feedback needs to correct it. Feedforward improves speed and smoothness, especially in jerk-limited motion.

We can also define for which use-cases we would use Feedback and for which we would use Feedforward control

Use feedback when:
- Disturbances are unpredictable
- Load changes frequently
- Exact modeling is difficult
- Stability is critical

Use feedforward when:
- The system model is reasonably accurate
- Motion is repetitive
- You want faster tracking
- You want to reduce feedback gain requirements

In the vise:

Torque hold -> primarily feedback

Motion planning -> feedforward + feedback

In real industrial systems, the best strategy is usually a combination:

      control_output = feedforward_term + feedback_term

Feedforward provides the bulk expected input. Feedback corrects residual error. However, they should not be combined blindly:
- If the model is inaccurate, feedforward can inject incorrect torque.
- Too much feedforward may reduce error signal and hide instability.
- Improper combination can reduce phase margin.

In the vise system, a good design would use:
- Feedforward for predicted clamp torque or motion inertia
- Feedback to guarantee precise 12.5 Nm and reject disturbances

This produces a fast, stable, and robust clamp.

During closing motion (position control), we generate a planned trajectory:
- Desired position: x_ref
- Desired velocity: v_ref
- Desired acceleration: a_ref

The motor torque command is:

      T_cmd = T_ff + T_fb


We estimate required torque from system physics:

      T_ff = J_eq * a_ref + B_eq * v_ref

Where:

      J_eq = equivalent inertia
      
      B_eq = damping

Example:

      If J_eq = 0.01 kg·m2 and a_ref = 50 rad/s2 ->
      T_ff = 0.5 Nm


This supplies the expected torque immediately.

We correct tracking error:

      T_fb = Kp (x_ref − x_meas) + Kd (v_ref − v_meas)

This compensates:
- Friction changes
- Load variation
- Modeling error



