# Linear-Model-Predictive-Controller

This assignment focuses on implementing a linear Model Predictive Controller (MPC) with constraints for trajectory tracking in 3D space (x, y, and z axes). You will modify the provided `xy_axis_mpc.m` and `z_axis_mpc.m` files to achieve this.

## Objectives

1.  **Implement a Linear MPC Controller with Constraints:** Modify the `xy_axis_mpc.m` and `z_axis_mpc.m` functions to implement a linear Model Predictive Controller (MPC) with constraints. The constraints for the x-y and z velocities, accelerations, and jerks need to be properly incorporated into the optimization problem.
2.  **Construct the Required Matrices:** Within each MPC function, construct the necessary matrices `H`, `F`, `A`, and `b` that define the optimization problem for the MPC. These matrices will be used as inputs to the `quadprog` function.
3.  **Simulate the Trajectory:** Run the `home_work.m` file after completing the implementation to visualize the simulated 3D trajectory. Ensure that the resulting trajectory in each axis meets the defined constraints and behaves as expected.
4.  **Review Helpful Resources:** Refer to the examples provided in the files `linear_mpc.m`, `linear_mpc_hard_constraint.m`, and `linear_mpc_soft_constraint.m` for guidance on constructing the MPC controller with constraints.

## Instructions

**1. Implement a Linear MPC Controller with Constraints (`xy_axis_mpc.m` and `z_axis_mpc.m`)**

   - Open the files `xy_axis_mpc.m` and `z_axis_mpc.m`.
   - Implement the logic for a linear MPC controller in each file.
   - **Crucially, incorporate the constraints on the velocity, acceleration, and jerk for the respective x-y and z axes.** These constraints are defined within each of these files (`v_max`, `v_min`, `a_max`, `a_min`, `jerk_max`, `jerk_min`). This will involve defining inequality constraints within your optimization problem.

**2. Construct the Required Matrices (`H`, `F`, `A`, `b`)**

   - Within both the `xy_axis_mpc.m` and `z_axis_mpc.m` functions, construct the following matrices:
     - `H`: The Hessian matrix for the quadratic cost function, penalizing deviations from the reference and control effort (jerk).
     - `F`: The gradient vector for the linear part of the cost function, related to the reference trajectory.
     - `A`: The matrix defining the inequality constraints (e.g., $A \cdot \mathbf{z} \leq \mathbf{b}$, where $\mathbf{z}$ is the optimization variable, likely a combination of future jerks and potentially slack variables for soft constraints if you were implementing them).
     - `b`: The vector defining the upper bounds for the inequality constraints.
   - Pay close attention to how these matrices are formed. They should correctly represent the system dynamics (how jerk affects acceleration, velocity, and position over the prediction horizon) and the constraints on velocity, acceleration, and jerk.
   - The control input for each axis will be the future jerk commands over the prediction horizon. The `getPredictionMatrix.m` file provides the matrices (`Tp`, `Tv`, `Ta`) that relate the initial state and future jerks to the predicted positions, velocities, and accelerations.

**3. Simulate the Trajectory (`home_work.m`)**

   - Run the `home_work.m` file. This script:
     - Initializes the initial position (`p_0`), velocity (`v_0`), and acceleration (`a_0`).
     - Defines a time loop to simulate the trajectory over a specified duration.
     - Within the loop:
       - Generates a reference trajectory (`pt`, `vt`, `at`) for the x, y, and z axes over the prediction horizon (`K`).
       - Calls the `xy_axis_mpc.m` function twice (for the x and y axes) and `z_axis_mpc.m` once to compute the optimal jerk command (`j`) for each axis.
       - Applies the first optimal jerk command to update the current state (position, velocity, acceleration) using the `forward.m` function.
       - Logs the states over time.
     - Plots the resulting 3D trajectory and the individual x, y, and z positions over time.
   - Observe the generated trajectory and ensure that:
     - The individual x, y, and z components track their respective reference trajectories.
     - The velocity, acceleration, and jerk in each axis remain within the defined bounds.
     - The overall motion is smooth and controlled.

**4. Review Helpful Resources**

   - The provided files (`linear_mpc.m`, `linear_mpc_hard_constraint.m`, `linear_mpc_soft_constraint.m`) offer valuable examples of how to:
     - Use the `getPredictionMatrix.m` to predict future states.
     - Formulate the quadratic cost function (`H` and `F`).
     - Implement constraints using the `A` and `b` matrices for the `quadprog` solver.
     - Apply the first optimal control input from the solved optimization problem.

By following these instructions and studying the provided code examples, you should be able to successfully implement the constrained linear MPC controller for each axis and generate the desired 3D trajectory. Remember to carefully consider how the constraints on velocity, acceleration, and jerk are incorporated into the `A` and `b` matrices within the `xy_axis_mpc.m` and `z_axis_mpc.m` functions. Good luck!
