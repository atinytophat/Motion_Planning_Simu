"""
Motion Planning Functions for KUKA Robot

Generates joint trajectories for MoveJ (joint interpolation) and MoveL (linear motion).
"""

import numpy as np

from forward_kinematics import modDH
from inverse_kinematics_refactored import solve_ik


def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) in degrees to rotation matrix.

    Uses INTRINSIC rotations (rotations about the end-effector's own axes):
    - Roll: rotation about the end-effector's X-axis
    - Pitch: rotation about the end-effector's Y-axis (after roll)
    - Yaw: rotation about the end-effector's Z-axis (after roll and pitch)

    This is more intuitive for tool-centric motion planning.
    """
    # Convert to radians
    r = np.radians(roll)
    p = np.radians(pitch)
    y = np.radians(yaw)

    # Rotation matrices for each axis
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])

    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])

    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])

    # Intrinsic rotations: Rx @ Ry @ Rz (compose rotations about moving axes)
    return Rx @ Ry @ Rz


def cartesian_waypoint_to_transformation_matrix(waypoint):
    """
    Extract a 4x4 transformation matrix from a cartesian waypoint.

    Args:
        waypoint: dict with key:
            - T: 4x4 transformation matrix (list of lists or numpy array)

    Returns:
        4x4 transformation matrix (numpy array)
    """
    if "T" not in waypoint:
        raise ValueError(
            "Cartesian waypoint must contain 'T' (4x4 transformation matrix)"
        )

    T = np.array(waypoint["T"], dtype=float)

    if T.shape != (4, 4):
        raise ValueError(f"Transformation matrix must be 4x4, got shape {T.shape}")

    return T


def joint_waypoint_to_transformation_matrix(waypoint):
    """
    Convert a joint space waypoint to a 4x4 transformation matrix using forward kinematics.
    Uses the same DH parameters and coordinate system as the rest of the system.

    Args:
        waypoint: dict with keys:
            - q: list of 6 joint angles in degrees [theta1...theta6]

    Returns:
        4x4 transformation matrix (numpy array)
    """
    # DH parameters for KUKA KR-10
    DHM = np.array(
        [
            [0, 0, 400],
            [-90, 25, 0],
            [0, 560, 0],
            [-90, 25, 515],
            [90, 0, 0],
            [-90, 0, 90],
        ]
    )
    alpha = DHM[:, 0]
    a = DHM[:, 1]
    d = DHM[:, 2]
    th_offset = np.array([0, 0, -90, 0, 0, 180])

    theta = np.array(waypoint.get("q", [0, 0, 0, 0, 0, 0]))
    # Apply inverse multipliers same as moveJ does
    th_inv = np.array([-1, 1, 1, -1, 1, -1])
    theta_adj = th_inv * theta + th_offset
    T = forward_kinematics_full(theta_adj, alpha, a, d, np.zeros(6))

    return T


def resolve_waypoint_to_theta(waypoint, seed_theta=None):
    """
    Resolve a waypoint (joint or cartesian) to joint angles.

    Args:
        waypoint: dict with either:
            - inputType='joint', q=[theta1...theta6]
            - inputType='pose', x=..., y=..., z=..., roll=..., pitch=..., yaw=...
        seed_theta: (unused, kept for API compatibility) seed joint angles for IK solver

    Returns:
        Array of 6 joint angles in degrees
    """
    if waypoint.get("inputType") == "joint":
        return np.array(waypoint.get("q", [0, 0, 0, 0, 0, 0]))

    elif waypoint.get("inputType") == "pose":
        # Convert cartesian pose to transformation matrix
        T = cartesian_waypoint_to_transformation_matrix(waypoint)

        # Solve IK
        theta, success = solve_ik(T)

        if not success or theta is None:
            raise ValueError(f"Could not solve IK for pose: {waypoint}")

        return theta


def rotation_matrix_to_angle_axis(R):
    """Convert rotation matrix to angle-axis representation."""
    # Ensure R is a proper rotation matrix
    trace = np.trace(R)

    if trace > 3 - 1e-6:  # Identity or near-identity
        return np.array([0, 0, 0])

    angle = np.arccos(np.clip((trace - 1) / 2, -1, 1))

    if angle < 1e-6:
        return np.array([0, 0, 0])

    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (
        2 * np.sin(angle)
    )

    axis = axis / np.linalg.norm(axis)
    return angle * axis


def angle_axis_to_rotation_matrix(angle_axis):
    """Convert angle-axis representation to rotation matrix."""
    angle = np.linalg.norm(angle_axis)

    if angle < 1e-6:
        return np.eye(3)

    axis = angle_axis / angle
    K = np.array(
        [[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]]
    )

    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
    return R


def moveJ(theta_start, theta_end, time_duration, num_points=100):
    """ "
    theta_start: Starting joint angles [deg] - array of 6 values
    theta_end: Ending joint angles [deg] - array of 6 values
    time_duration: Total motion time in seconds
    num_points: Number of trajectory points to generate

    Returns joint trajectory with Cartesian poses computed via FK at each step.
    """
    # DH parameters for KUKA KR-10
    DHM = np.array(
        [
            [0, 0, 400],
            [-90, 25, 0],
            [0, 560, 0],
            [-90, 25, 515],
            [90, 0, 0],
            [-90, 0, 90],
        ]
    )
    alpha = DHM[:, 0]
    a = DHM[:, 1]
    d = DHM[:, 2]
    th_offset = np.array([0, 0, -90, 0, 0, 180])
    th_inv = np.array(
        [-1, 1, 1, -1, 1, -1]
    )  # Inverse joint multipliers (MUST match app.py)

    # Polynomial (cubic) interpolation in joint space
    t = np.linspace(0, time_duration, num_points)
    theta = np.zeros((num_points, 6))
    theta_dot = np.zeros((num_points, 6))
    theta_ddot = np.zeros((num_points, 6))
    cartesian_pos = np.zeros((num_points, 3))

    for j in range(6):
        # Cubic polynomial interpolation for smooth joint trajectory
        theta[:, j] = theta_start[j] + (theta_end[j] - theta_start[j]) * (
            3 * (t / time_duration) ** 2 - 2 * (t / time_duration) ** 3
        )

        # Velocity: derivative of cubic interpolation
        theta_dot[:, j] = (theta_end[j] - theta_start[j]) * (
            6 * t / time_duration**2 - 6 * t**2 / time_duration**3
        )

        # Acceleration: derivative of velocity
        theta_ddot[:, j] = (theta_end[j] - theta_start[j]) * (
            6 / time_duration**2 - 12 * t / time_duration**3
        )

    # Compute Cartesian poses at each interpolated step using FK
    for i in range(num_points):
        try:
            # Apply inverse multipliers BEFORE FK (same as app.py FK endpoint)
            theta_adj = th_inv * theta[i] + th_offset
            T = forward_kinematics_full(theta_adj, alpha, a, d, np.zeros(6))
            cartesian_pos[i] = T[:3, 3]  # Extract TCP position
        except:
            # If FK fails, use previous position or zeros
            if i > 0:
                cartesian_pos[i] = cartesian_pos[i - 1]
            else:
                cartesian_pos[i] = [0, 0, 0]

    return {
        "time": t.tolist(),
        "theta": theta.tolist(),
        "theta_dot": theta_dot.tolist(),
        "theta_ddot": theta_ddot.tolist(),
        "cartesian_pos": cartesian_pos.tolist(),
        "motion_type": "MoveJ",
    }


def moveL(
    theta_start=None,
    theta_end=None,
    time_duration=None,
    num_points=100,
    start_waypoint=None,
    goal_waypoint=None,
):
    """
    Linear motion (MoveL) - moves end-effector in straight line in Cartesian space.

    Can accept either:
    - Direct theta arrays: theta_start, theta_end, time_duration
    - Waypoint objects: start_waypoint, goal_waypoint, time_duration

    When waypoints are provided, computes transformation matrices for each waypoint
    and uses them for motion planning.

    Args:
        theta_start: Starting joint angles [deg] - array of 6 values (optional)
        theta_end: Ending joint angles [deg] - array of 6 values (optional)
        start_waypoint: Starting waypoint dict with inputType and q or T (4x4 matrix) (optional)
        goal_waypoint: Goal waypoint dict with inputType and q or T (4x4 matrix) (optional)
        time_duration: Total motion time in seconds
        num_points: Number of trajectory points to generate

    Returns:
        dict with keys:
            - 'time': time array (seconds)
            - 'theta': joint angles over time (num_points x 6)
            - 'theta_dot': joint velocities (num_points x 6)
            - 'theta_ddot': joint accelerations (num_points x 6)
            - 'cartesian_pos': end-effector positions (num_points x 3)
    """
    # DH parameters for KUKA KR-10
    DHM = np.array(
        [
            [0, 0, 400],
            [-90, 25, 0],
            [0, 560, 0],
            [-90, 25, 515],
            [90, 0, 0],
            [-90, 0, 90],
        ]
    )
    alpha = DHM[:, 0]
    a = DHM[:, 1]
    d = DHM[:, 2]
    th_offset = np.array([0, 0, -90, 0, 0, 180])

    # Check if waypoints are provided - if so, compute T matrices from them
    if start_waypoint is not None and goal_waypoint is not None:
        # Compute transformation matrices from waypoints
        if start_waypoint.get("inputType") == "joint":
            T_start = joint_waypoint_to_transformation_matrix(start_waypoint)
            theta_start_for_seed = np.array(start_waypoint.get("q", [0, 0, 0, 0, 0, 0]))
        else:
            T_start = cartesian_waypoint_to_transformation_matrix(start_waypoint)
            # Solve IK to get theta_start for seed purposes
            theta_start_for_seed, _ = solve_ik(T_start)

        if goal_waypoint.get("inputType") == "joint":
            T_end = joint_waypoint_to_transformation_matrix(goal_waypoint)
        else:
            T_end = cartesian_waypoint_to_transformation_matrix(goal_waypoint)
    else:
        # Use direct theta arrays
        theta_start = (
            np.array(theta_start)
            if theta_start is not None
            else np.array([0, 0, 0, 0, 0, 0])
        )
        theta_end = (
            np.array(theta_end)
            if theta_end is not None
            else np.array([0, 0, 0, 0, 0, 0])
        )
        theta_start_for_seed = theta_start

        # Convert joint angles to transformation matrices (apply inverse multipliers like moveJ)
        th_inv = np.array([-1, 1, 1, -1, 1, -1])
        theta_start_adj = th_inv * theta_start + th_offset
        theta_end_adj = th_inv * theta_end + th_offset
        T_start = forward_kinematics_full(theta_start_adj, alpha, a, d, np.zeros(6))
        T_end = forward_kinematics_full(theta_end_adj, alpha, a, d, np.zeros(6))

    # Compute relative transformation: T_rel = T_start^-1 * T_end
    # This represents the motion from start to end in the end-effector frame
    T_start_inv = np.linalg.inv(T_start)
    T_rel = T_start_inv @ T_end

    # Extract positions and rotations from start and end transformation matrices
    pos_start = T_start[:3, 3]
    pos_end = T_end[:3, 3]
    R_start = T_start[:3, :3]
    R_end = T_end[:3, :3]

    # Convert end rotation to angle-axis relative to start for proper interpolation
    R_rel = R_start.T @ R_end  # Rotation from start frame to end frame
    angle_axis_rel = rotation_matrix_to_angle_axis(R_rel)

    t = np.linspace(0, time_duration, num_points)
    theta = np.zeros((num_points, 6))
    theta_dot = np.zeros((num_points, 6))
    theta_ddot = np.zeros((num_points, 6))
    cartesian_pos = np.zeros((num_points, 3))

    for i in range(num_points):
        t_i = t[i]
        s = t_i / time_duration  # 0 to 1 - normalized time for linear interpolation

        # STRAIGHT LINE interpolation in Cartesian space
        # Position: linear interpolation between start and end positions (straight line)
        pos_interp = (1 - s) * pos_start + s * pos_end

        # Rotation: angle-axis interpolation to smoothly interpolate orientation
        angle_axis_interp = s * angle_axis_rel
        R_interp_rel = angle_axis_to_rotation_matrix(angle_axis_interp)
        R_interp = R_start @ R_interp_rel

        # Build transformation matrix for this point on the line
        T_target = np.eye(4)
        T_target[:3, :3] = R_interp
        T_target[:3, 3] = pos_interp

        # Solve IK for this point on the linear path
        # Get all possible solutions and select the one closest to previous configuration
        try:
            theta_all, success = solve_ik(T_target, return_all=True)

            if success and theta_all is not None and len(theta_all) > 0:
                # Select best solution: closest to previous theta to ensure smooth motion
                if i == 0:
                    # First point: use solution closest to seed
                    best_idx = 0
                    best_distance = np.inf
                    for j, theta_candidate in enumerate(theta_all):
                        dist = np.linalg.norm(theta_candidate - theta_start_for_seed)
                        if dist < best_distance:
                            best_distance = dist
                            best_idx = j
                    theta[i] = theta_all[best_idx]
                else:
                    # Subsequent points: use solution closest to previous theta
                    best_idx = 0
                    best_distance = np.inf
                    for j, theta_candidate in enumerate(theta_all):
                        dist = np.linalg.norm(theta_candidate - theta[i - 1])
                        if dist < best_distance:
                            best_distance = dist
                            best_idx = j
                    theta[i] = theta_all[best_idx]
            else:
                # No solution found, use previous theta
                if i > 0:
                    theta[i] = theta[i - 1]
                else:
                    theta[i] = theta_start_for_seed
        except Exception as e:
            # If IK fails, use previous solution
            if i > 0:
                theta[i] = theta[i - 1]
            else:
                theta[i] = theta_start_for_seed

        # Compute actual cartesian position from FK using the solved joint angles
        T_actual = forward_kinematics_full(theta[i], alpha, a, d, th_offset)
        cartesian_pos[i] = T_actual[:3, 3]
    # Compute velocities using numerical differentiation
    for i in range(num_points):
        if i > 0:
            theta_dot[i] = (theta[i] - theta[i - 1]) / (t[i] - t[i - 1])
            if i > 1:
                theta_ddot[i] = (theta_dot[i] - theta_dot[i - 1]) / (t[i] - t[i - 1])

    return {
        "time": t.tolist(),
        "theta": theta.tolist(),
        "theta_dot": theta_dot.tolist(),
        "theta_ddot": theta_ddot.tolist(),
        "cartesian_pos": cartesian_pos.tolist(),
        "motion_type": "MoveL",
    }


def forward_kinematics_full(theta, alpha, a, d, th_offset):
    """
    Compute full transformation matrix using FK.

    Args:
        theta: Joint angles [deg] - array of 6 values
        alpha, a, d: DH parameters
        th_offset: Joint angle offsets

    Returns:
        T: 4x4 transformation matrix
    """
    theta_adj = theta + th_offset
    T = modDH(a, alpha, d, theta_adj)
    return T


def solve_ik_wrapper(T_target):
    """
    Wrapper for IK computation.
    Uses the refactored inverse_kinematics_refactored module.
    """
    theta, success = solve_ik(T_target)
    if not success or theta is None:
        raise ValueError("No IK solution found for target pose")
    return theta
