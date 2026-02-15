# Controls Engineering and Programming Assessment

## Controls Engineer Technical
Purpose: This assessment is designed to evaluate your ability to design, implement, and reason about real world control systems in C++. The focus is on practical motor control, real time behavior, EtherCAT based communication, and software architecture for controls application. You may use reference material, the internet, and AI tools. Using AI is encouraged. Code should be written as if it were intended to run on real hardware.

## Question 1: Motor Controls Problem
You are controlling a motor that drives a mechanical vise through a worm gear and lead screw.  Rotating the motor clockwise closes the vise. Rotating the motor counterclockwise opens the vise. The motor is connected to a motor controller over EtherCAT at a cycle time of 1ms, though this is changeable with hardware and software changes. The motor controller supports multiple operating modes and allows switching between them during operation.  The vise is used to clamp two plates together while a manufacturing process is performed.
The motor controller supports the following modes:
- Torque mode: A current command is sent and the controller regulates motor torque.
- Velocity mode: A velocity command is sent and the controller regulates motor speed.
- Position mode: A position command is sent and the controller regulates motor position.
The controller allows switching between these modes during operation.  
Details:
- The vice lead screw uses an M6 x 1 thread.
- The vise must apply 12.5 Nm of torque at the lead screw to achieve proper clamping force.
- When fully open, the distance between the vise faces is 40 mm.
- When clamping two plates, the distance between the vise faces is 20 mm.
- The system must therefore travel 20 mm from open to clamped.
- The system should move as quickly as is reasonable while maintaining controlled motion and achieving accurate and repeatable final torque.
- Describe the full control strategy you would implement to achieve the following sequence.
- Rapidly close the vise from the open position.
- Detect contact with the plates.
- Transition into a controlled clamping phase.
- Achieve and hold the target torque of 12.5 Nm.
- Safely open the vise back to the fully open position.

Your answer should include which control modes are used during each part of the controls process and why, how you detect contact or transition conditions, how you ensure torque accuracy and stability at the final clamp, how you prevent overshoot, oscillation, or excessive force.
Specify the control loop frequency you would choose and justify it, describe any filtering you would apply to sensor signals or command signals and explain whether you would rely on sensor data read over EtherCAT or direct hardware inputs, and why.
Assume EtherCAT cycle times between 250 microseconds and 2 milliseconds depending on platform.
Provide C++ code that demonstrates how you would implement this system. This code should be written as if it were part of a real project, not pseudocode.  You have been provided a “Controls_Project.hpp” file that provides framework and scaffolding for you to write your controls code with.  You are welcome to use AI to accelerate the process.
Your code may include handling of EtherCAT process data, state management for different phases of motion and clamping, mode switching logic.  Please focus on the controls aspect of the problem.


## Question 2: Controls Concepts and Sampling
Explain the following concepts in the context of real control systems.
- What frequency response and magnitude response represent physically.
- Why do these characteristics matter when tuning and validating a control system.
- How would you characterize what the magnitude and frequency response of a physical system is.
- What system states are and why accurate measurement or estimation is or is not important.
- What under sampling is and how it can cause instability or incorrect behavior.
- What sampling criteria you would follow when designing a control loop for a system like that described in section one.
- Feed forward control and feedback control.  What are each of these conceptually.  When would you use each?  When they should or should not be combined/used in a single controls strategy?


## Question3: Software architecture question: 

You need hardware to control this motor controller and motor combination.  For the master/leader module that controls the motor controller, you have the following options; 
- An ARM based system on module that runs Linux (Can support real time Linux kernel) running at 2Ghz (Such as the Jetson Orin NX).  
- A quad core system on a chip running at 400Mhz (Such as the TI Sitara).  
- A single core 120Mhz micro controller.


All have the ability to support Ethernet (Ethernet master nodes can be converted to EtherCAT if they can meet the timing constraints), which is the communication interface for the motor controller.  To facilitate solving question one, what hardware would you select for communicating with and controlling the motor controller?  Can you please list what the pros and cons are of integrating and using each hardware selection and design from a programming perspective.  Such points might include but are not limited to;
- Advantage/disadvantage of using multiple threads.
- Real time capabilities of each hardware platform.
- Compute capabilities.


Please explain why you would choose the hardware you have chosen, and then explain what coding practices you would follow to implement this code.  
- Would you use multiple threads?  If so, how will you manage shared memory?
- Would you allow multiple threads to access and use the communication stack to read and write from the communication bus (EtherCAT)?
You are working in conjunction with a programmer who is developing and managing the communication stack and all of the overhead for cores, threads, different micro peripherals, etc.  How would you version control the software?  What platforms or tools would you use if any?


