# ref_traj_generation.py
import numpy as np
import cvxpy as cp

class ReferenceTrajectoryGenerator:
    """
    Obstacle-agnostic reference trajectory generator using MPC to reach a goal state.
    Uses a quadratic objective to approach the goal while respecting acceleration limits.
    """
    def __init__(self, ego_dynamics, horizon, goal_state, Q=None, R=None, Q_final=None, a_max=1.0):
        """
        ego_dynamics: instance of DoubleIntegratorDynamics (for A, B matrices).
        horizon: planning horizon (number of time steps) for the trajectory.
        goal_state: desired goal state [p_x, p_y, v_x, v_y] (numpy array or list).
        Q: Stage state cost matrix (e.g., weight for being far from goal at intermediate steps).
           If None, default weights (penalize position error) are used.
        R: Control cost matrix (penalizing acceleration magnitude). If None, identity is used.
        Q_final: Final state cost matrix (penalizing deviation from goal at horizon end).
                 If None, defaults to a higher weight on position and velocity.
        a_max: Max acceleration (per axis) constraint.
        """
        self.ego_dynamics = ego_dynamics
        self.A = ego_dynamics.A
        self.B = ego_dynamics.B
        self.state_dim = ego_dynamics.state_dim
        self.control_dim = ego_dynamics.control_dim
        self.horizon = horizon
        # Goal state as numpy array
        self.goal_state = np.array(goal_state)
        # Setup cost matrices
        if Q is None:
            # Default: penalize position error at intermediate steps, not velocity (vel weight 0)
            Q = np.diag([1.0, 1.0] + [0.0] * (self.state_dim - 2))
        else:
            Q = np.array(Q)
            if Q.ndim == 1:
                Q = np.diag(Q)
        if R is None:
            R = np.eye(self.control_dim)
        else:
            R = np.array(R)
            if np.ndim(R) == 0:
                R = float(R) * np.eye(self.control_dim)
            if R.ndim == 1:
                R = np.diag(R)
        if Q_final is None:
            # Final cost: prioritize reaching goal (both position and velocity)
            Q_final = np.eye(self.state_dim)
        else:
            Q_final = np.array(Q_final)
            if Q_final.ndim == 1:
                Q_final = np.diag(Q_final)
        self.Q = Q
        self.R = R
        self.Q_final = Q_final
        self.a_max = a_max
        # Placeholders for result
        self.x_ref_traj = None
        self.u_ref_traj = None
    
    def compute_reference(self, x0):
        """
        Compute an obstacle-free reference trajectory from current state x0 to the goal state.
        Returns the planned states and controls as numpy arrays.
        """
        x0 = np.array(x0)
        N = self.horizon
        # Define CVXPY variables
        x = cp.Variable((self.state_dim, N+1))
        u = cp.Variable((self.control_dim, N))
        # Constraints
        constraints = []
        constraints.append(x[:, 0] == x0)
        for t in range(N):
            constraints.append(x[:, t+1] == self.A @ x[:, t] + self.B @ u[:, t])
        for t in range(N):
            for j in range(self.control_dim):
                constraints.append(u[j, t] <= self.a_max)
                constraints.append(u[j, t] >= -self.a_max)
        # Objective: minimize goal distance and control effort
        cost = 0
        goal = self.goal_state
        for t in range(N):
            x_diff = x[:, t] - goal
            cost += cp.quad_form(x_diff, self.Q)
            cost += cp.quad_form(u[:, t], self.R)
        # Final state cost at time N
        x_final_diff = x[:, N] - goal
        cost += cp.quad_form(x_final_diff, self.Q_final)
        # Solve the optimization problem
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.ECOS, verbose=False)
        # Retrieve solution
        X_opt = x.value
        U_opt = u.value
        if X_opt is None or U_opt is None:
            raise ValueError("Reference trajectory optimization failed.")
        # Store trajectories (time as first dimension)
        self.x_ref_traj = np.array(X_opt.T)
        self.u_ref_traj = np.array(U_opt.T)
        return self.x_ref_traj, self.u_ref_traj
