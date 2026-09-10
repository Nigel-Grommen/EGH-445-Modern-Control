# How to use the dynamics and simultor functions
The majority of your task is to implement the control_law_handle function, which will be used to compute the control input u(kT) at each sampling time kT. The control law will be used in the System_Simulator function, which simulates the system dynamics. Refer to the example. 

## System_Simulator
System_Simulator - Simulates a continuous-time system under discrete, ZOH control.

```
[T, Y, U_zoh] = System_Simulator(system_dynamics_handle, ...
       initial_state, t_span, control_law_handle, sample_time, student_ID, output_function_handle)
```

Inputs:

    system_dynamics_handle: Function handle for the continuous system dynamics.
                            Expected signature: dxdt = f(t, x, u)
                            where u is the control input.
    initial_state:          Column vector of initial state values x(0).
    t_span:                 Time interval for simulation, e.g., [t_start, t_end].
    control_law_handle:     Function handle for the discrete control law.
                            Expected signature: u = g(t, y)
                            where y is the system output at sample time t.
    sample_time:            Sampling period (Ts) for the discrete controller.
    student_ID:             Student ID for personalising the dynamics.
                            It is used to define disturbances, noise, etc.
    output_function_handle: (Optional) Function handle to compute system output y from state x.
                            Expected signature: y = h(t, x)
                            Defaults to y = x if not provided or empty.
    

Outputs:

    T:      Column vector of time points from the ODE solver.
    Y:      Matrix of system outputs at each time point in T (each row is y(t)').
    U_zoh:  Matrix of the piecewise constant ZOH control input applied
            at each time point in T (each row is u(t)').

Author: Guilherme Froes Silva (QUT), 

Date: 2025-03-27

## System_Dynamics
System_Dynamics - Defines the state-space dynamics of a nonlinear system.
    
    dxdt = System_Dynamics(t, x, F_input, student_ID)

  Inputs:

      t       : Current time (scalar). Often unused if F is constant or only state-dependent.
      x       : Current state vector. E.g. [q; q_dot]
      F_input : External force. Can be a scalar value or a function handle
                expecting inputs (t, x), e.g., F_input = @(time, state) sin(time).

  Output:
      dxdt    : State derivative vector. E.g. [q_dot; q_ddot]

  Author: Guilherme Froes Silva
  
  Date: 2025-03-26