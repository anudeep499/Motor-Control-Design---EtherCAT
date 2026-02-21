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

In a real system like the vise, we don’t guess stability — we measure it. To characterize frequency response, we inject a small sinusoidal torque signal (for example ±0.2 Nm) and sweep frequency from 1 Hz to 200 Hz while measuring velocity or position. Using FFT, we compute the magnitude ratio and phase shift at each frequency and plot a Bode plot.

From this, we might identify:

First resonance at ~80 Hz

Gain crossover around 20 Hz

Phase margin ~45°

Noise amplification above 150 Hz

If resonance is 80 Hz, we typically keep control bandwidth around 15–25 Hz (about 1/3 to 1/5 of resonance). This prevents exciting structural vibration in the screw and jaws, which would otherwise cause chatter or long-term wear during 12.5 Nm clamping.

This approach improves reliability because we avoid operating near resonance, reducing mechanical stress and vibration. It also improves serviceability — if, over time, resonance shifts (say from 80 Hz to 70 Hz due to wear or lubrication changes), re-measuring frequency response immediately reveals it. That makes it both a tuning tool and a diagnostic tool.

Students or engineers typically determine frequency response by applying sine sweeps, chirp signals, or PRBS excitation and analyzing the output using FFT tools (MATLAB, Python, etc.). This gives a data-driven way to tune gains instead of relying on trial-and-error.

So frequency and magnitude response are not abstract theory — they are practical tools that allow us to design a stable, robust, and long-lasting clamp system.
