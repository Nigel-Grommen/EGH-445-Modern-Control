EGH445-VTOL-Output-Feedback-Control
EGH445 Modern Control — Assessment 2: Working Model & Technical Report

Output-feedback control of a planar vectored-thrust VTOL aircraft; LQR + integral action state feedback with a Luenberger observer, validated against a nonlinear black-box plant in MATLAB.

Overview
This project implements a complete output-feedback control system for a scaled, planar vectored-thrust VTOL aircraft, controlled by two forces (F1, F2) applied at a fixed offset below its centre of mass. The controller drives the aircraft from the origin to a target hover position and rejects disturbances using only position and angle measurements — no velocity sensors — with all velocity states reconstructed by an observer.

Design specifications:

Nonlinear 3-DOF rigid-body model (x, y, θ), linearised at hover equilibrium
Discrete-time optimal (LQR) state-feedback control with integral action
Luenberger observer reconstructing full state from position/angle-only measurements
Actuator limits enforced: F1 ∈ [-40, 40] N, F2 ∈ [0, 80] N
Validated on a provided nonlinear "black-box" plant, not just the linear model
Robustness tested against disturbances, varying initial conditions, and measurement noise

How It Works
Nonlinear model — Newton-Euler equations of motion for the coupled translational/rotational dynamics, linearised via Jacobian about the hover equilibrium (F1 = 0, F2 = mg) and discretised with zero-order hold
State feedback (Kd) — discrete LQR gain computed via dlqr, tuned through Q/R weighting to balance position tracking against control effort
Integral action (Ki) — augmented tracking states on x/y position eliminate steady-state error under constant disturbances, with conditional integration and anti-windup based on actuator headroom
Luenberger observer (L) — poles placed ~10x faster than the controller poles (in continuous time, then discretised) to reconstruct velocity and full state from x, y, θ measurements only
Kalman filter (extension) — discrete KF compared against the Luenberger observer under injected Gaussian measurement noise, run on the closed-loop linear model
Output-feedback loop — observer estimate feeds the same Kd/Ki control law, with actuator saturation applied at every step

Closed-loop discrete pole locations follow the standard LQR/observer separation principle: controller poles are designed for tracking performance, observer poles are placed faster so state estimation error decays quickly relative to the controlled dynamics.

Repository Structure
File | Description
--- | ---
Assessment_Code.m | Main script — modelling, linearisation, LQR/observer design, nonlinear simulation, all report figures
IntegralController.m | Full-state feedback + integral action controller (function handle for the simulator)
ObserverIntegralController.m | Observer-based output-feedback controller with integral action
printForceAnalysis.m | Actuator effort summary and saturation-limit compliance check
printPerformanceMetrics.m | Overshoot/undershoot/settling-time metrics (2% band)
printPoles.m | Pole locations and discrete-time stability check
*.png | Generated figures — pole maps, step responses, disturbance rejection, robustness tests, Kalman filter comparison
EGH445 VTOL Report.docx | IEEE-style technical report (6-page, two-column)

Note: System_Dynamics.p and System_Simulator.p (the protected black-box plant/simulator provided by the unit) are required to run the code but are not redistributed here.

Results
Case | Behaviour Observed
--- | ---
Open loop | Marginally stable / integrator poles on the unit circle — confirms need for feedback
Closed loop (Kd only) | All poles placed inside the unit circle; stable convergence to target with no integral correction of steady disturbances
Closed loop (Kd + Ki) | Zero steady-state error in x/y position under constant disturbance; some overshoot traded for disturbance rejection
Observer (NL+OBS+IC) | State estimates converge quickly (poles ~10x controller speed); output-feedback response closely tracks full-state response
Kalman filter vs Luenberger | KF gives visibly smoother state estimates under measurement noise; Luenberger is simpler but noisier
Actuator forces | F1 and F2 remain within ±40 N / 0-80 N limits across all tested nominal and disturbed cases
Robustness sweep | System converges to target across multiple disturbance realisations and initial conditions, with transient overshoot/settling time varying by scenario

PCB/hardware: not applicable — this is a simulation-based control design assessment (MATLAB, no physical build).

Tools & Methods
MATLAB — symbolic linearisation, dlqr, place, c2d, ode45 nonlinear simulation
Control System Toolbox — discretisation, pole-zero mapping, LQR/observer design
Symbolic Math Toolbox — Jacobian-based linearisation of the nonlinear equations of motion
Black-box simulator (System_Simulator.p) — closed-loop nonlinear validation with zero-order-hold control and injected disturbances/noise

Author
Nigel Grommen — N11908203
