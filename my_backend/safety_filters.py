# safety_filter.py
import numpy as np
import cvxpy as cp

class SafetyFilter:
    """
    MPC-based safety filter that enforces DR-CVaR safe halfspace constraints.
    It corrects a reference trajectory minimally to ensure safety.
    Provides methods to set up and solve the optimization problem.
    """
    def __init__(self, ego_dynamics, Q=None, R=None, Q_final=None, a_max=1.0):
        """
        ego_dynamics: an instance of DoubleIntegratorDynamics (provides A, B, C matrices).
        Q: State deviation cost matrix (numpy array or list). If None, identity is used.
        R: Control deviation cost matrix. If None, identity is used.
        Q_final: Final state deviation cost matrix (optional). If None, defaults to Q.
        a_max: Maximum acceleration magnitude (per axis) for control constraints.
        """
        self.ego_dynamics = ego_dynamics
        self.A = ego_dynamics.A
        self.B = ego_dynamics.B
        self.C = ego_dynamics.C
        self.state_dim = ego_dynamics.state_dim
        self.control_dim = ego_dynamics.control_dim
        # Cost matrices setup
        if Q is None:
            Q = np.eye(self.state_dim)
        else:
            Q = np.array(Q)
        if Q.ndim == 1:  # if given as diagonal vector
            Q = np.diag(Q)
        if R is None:
            R = np.eye(self.control_dim)
        else:
            R = np.array(R)
        if np.ndim(R) == 0:  # scalar
            R = float(R) * np.eye(self.control_dim)
        if R.ndim == 1:
            R = np.diag(R)
        if Q_final is None:
            Q_final = Q
        else:
            Q_final = np.array(Q_final)
            if Q_final.ndim == 1:
                Q_final = np.diag(Q_final)
        self.Q = Q
        self.R = R
        self.Q_final = Q_final
        # Control bounds
        self.a_max = a_max
        # Placeholders for results
        self.x_traj = None  # filtered state trajectory (NumPy array)
        self.u_traj = None  # filtered control trajectory (NumPy array)
    
    def set_opt_pb_params(self, x0, ref_states, ref_controls, halfspace_normals=None, halfspace_offsets=None):
        """
        Set up the MPC optimization problem with current parameters.
        x0: current ego state (numpy array of shape (state_dim,)).
        ref_states: reference trajectory states (numpy array of shape (N+1, state_dim)).
        ref_controls: reference trajectory controls (numpy array of shape (N, control_dim)).
        halfspace_normals: precomputed safe halfspace normals, shape (N, n_obs, 2) or similar.
        halfspace_offsets: corresponding halfspace offsets, shape (N, n_obs).
        """
        x0 = np.array(x0)
        ref_states = np.array(ref_states)
        ref_controls = np.array(ref_controls)
        N = ref_controls.shape[0]  # number of control steps in horizon
        # Define optimization variables for state and control over horizon
        self.x_var = cp.Variable((self.state_dim, N+1))
        self.u_var = cp.Variable((self.control_dim, N))
        # Build constraints list
        constraints = []
        # Initial state must match current state
        constraints.append(self.x_var[:, 0] == x0)
        # Dynamics constraints for each time step
        for t in range(N):
            constraints.append(self.x_var[:, t+1] == self.A @ self.x_var[:, t] + self.B @ self.u_var[:, t])
        # Control acceleration bounds (per component)
        for t in range(N):
            for j in range(self.control_dim):
                constraints.append(self.u_var[j, t] <= self.a_max)
                constraints.append(self.u_var[j, t] >= -self.a_max)
        # Safe halfspace constraints (for each obstacle at each future step, if provided)
        if halfspace_normals is not None and halfspace_offsets is not None:
            halfspace_normals = np.array(halfspace_normals)
            halfspace_offsets = np.array(halfspace_offsets)
            # Handle case of a single obstacle (reshape to (N,1,2))
            if halfspace_normals.ndim == 2:
                halfspace_normals = halfspace_normals[:, np.newaxis, :]
            if halfspace_offsets.ndim == 1:
                halfspace_offsets = halfspace_offsets[:, np.newaxis]
            N_steps = halfspace_normals.shape[0]
            n_obs = halfspace_normals.shape[1]
            # Impose each halfspace constraint at the corresponding time step
            for t in range(min(N, N_steps)):
                # Extract ego position at time t+1
                pos_t1 = self.C @ self.x_var[:, t+1]  # [p_x, p_y] at step t+1
                for j in range(n_obs):
                    n_vec = halfspace_normals[t, j, :]
                    b_val = halfspace_offsets[t, j]
                    constraints.append(n_vec @ pos_t1 <= b_val)
        # Objective: minimize deviation from reference trajectory (quadratic cost)
        cost = 0
        for t in range(N):
            # Difference from reference state and input at time t
            x_diff = self.x_var[:, t] - ref_states[t]
            u_diff = self.u_var[:, t] - ref_controls[t]
            cost += cp.quad_form(x_diff, self.Q) + cp.quad_form(u_diff, self.R)
        # Final state deviation cost at time N
        x_final_diff = self.x_var[:, N] - ref_states[N]
        cost += cp.quad_form(x_final_diff, self.Q_final)
        # Define the optimization problem
        self.prob = cp.Problem(cp.Minimize(cost), constraints)
    
    def solve_opt_pb(self, solver=cp.ECOS, verbose=False):
        """
        Solve the MPC safety filter QP problem. 
        After solving, populates x_traj and u_traj with the optimized trajectory.
        """
        try:
            self.prob.solve(solver=solver, verbose=verbose)
        except Exception as e:
            # Propagate any solver error
            raise e
        # Retrieve results if solved successfully
        if self.prob.status not in ["infeasible", "unbounded"]:
            X_opt = self.x_var.value
            U_opt = self.u_var.value
            if X_opt is not None and U_opt is not None:
                # Convert to shape (N+1, state_dim) and (N, control_dim)
                self.x_traj = np.array(X_opt.T)
                self.u_traj = np.array(U_opt.T)
        else:
            # If infeasible or unbounded, indicate failure (could implement fallback control here)
            self.x_traj = None
            self.u_traj = None
            raise ValueError("Safety filter optimization problem infeasible or unbounded.")
