# dynamics.py
import numpy as np

class DoubleIntegratorDynamics:
    """
    Double integrator dynamics model for the ego vehicle (2D).
    State x = [p_x, p_y, v_x, v_y] and control u = [a_x, a_y].
    Discrete-time update: 
      p_{t+1} = p_t + v_t * dt + 0.5 * a_t * dt^2
      v_{t+1} = v_t + a_t * dt
    """
    def __init__(self, dt: float):
        self.dt = dt
        # State dimension = 4 (px, py, vx, vy); control dimension = 2 (ax, ay)
        self.state_dim = 4
        self.control_dim = 2
        # Define discrete-time state-transition matrices A and B
        self.A = np.eye(4)
        # Fill in A for position update: p_{t+1} depends on v_t (p part already identity)
        self.A[0, 2] = dt  # p_x depends on v_x
        self.A[1, 3] = dt  # p_y depends on v_y
        # Velocity part: v_{t+1} = v_t (plus input via B)
        # (so A[2,2]=1, A[3,3]=1 already from np.eye)
        # Define input matrix B for acceleration effect
        B_pos = 0.5 * (dt**2) * np.eye(2)  # position gets 0.5*dt^2 * acceleration
        B_vel = dt * np.eye(2)            # velocity gets dt * acceleration
        # Combine into 4x2 matrix
        self.B = np.vstack((B_pos, B_vel))
        # Matrix to extract position from state (2x4): multiplies state to give [p_x, p_y]
        self.C = np.zeros((2, 4))
        self.C[0, 0] = 1.0
        self.C[1, 1] = 1.0
    
    def step(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Propagate the state forward by one time step using the dynamics.
        x: state vector [p_x, p_y, v_x, v_y]
        u: control input [a_x, a_y]
        Returns next state x_next.
        """
        return self.A.dot(x) + self.B.dot(u)


class SingleIntegratorDynamics:
    """
    Single integrator dynamics model for obstacle vehicles (2D).
    State x = [p_x, p_y] and control u = [v_x, v_y] (velocity as control).
    Discrete-time update:
      p_{t+1} = p_t + v_t * dt
    (No acceleration, velocity is directly the control input.)
    """
    def __init__(self, dt: float):
        self.dt = dt
        # State dimension = 2 (px, py); control dimension = 2 (v_x, v_y)
        self.state_dim = 2
        self.control_dim = 2
        # State-transition matrices for single integrator
        self.A = np.eye(2)
        self.B = dt * np.eye(2)
        # For consistency, define C as identity (position is the state itself)
        self.C = np.eye(2)
    
    def step(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Propagate the obstacle state forward by one time step.
        x: state [p_x, p_y]
        u: control [v_x, v_y] (velocity)
        Returns next state position.
        """
        return self.A.dot(x) + self.B.dot(u)
