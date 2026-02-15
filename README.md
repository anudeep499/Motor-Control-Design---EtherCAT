# Motor Control Design – EtherCAT (Controls Engineering Assessment)

  This project implements a real-time supervisory motion controller for an EtherCAT-connected servo drive. The objective of this assessment was to design and implement a robust ControlTick() function that
  
  - Safely brings a drive from OFF to OPERATION ENABLED in a safe organized manner
  - Manages operation mode transitions considering the debounce number
  - Generates smooth motion trajectories (lot more explained in the writeup)
  - Handles communication and fault conditions
  - Respects mechanical travel limits (impedance type controls added to avoid locking and problems with relation to alignment)
  - Behaves as if intended for real hardware and there are functions/steps included to calibrate the behaviour of the system and set torque limits for safety purposes!
  - The drive is assumed to contain internal PID control loops (position / velocity / torque) cascaded control loops.
  
  **This implementation focuses on supervisory control and trajectory generation, not low-level current control.**

<p align="center">
<img src="https://github.com/user-attachments/assets/d9c06a7f-4c80-4ecd-9e66-91f057ea915d" width="500">
</p>
