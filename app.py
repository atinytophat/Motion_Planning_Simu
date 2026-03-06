"""
Flask Backend API for Motion Planning Web App

Provides REST endpoints for FK, IK, and motion planning functions.
Run with: python app.py
"""

import json

import numpy as np
from flask import Flask, jsonify, request
from flask_cors import CORS
from forward_kinematics import modDH
from inverse_kinematics_refactored import solve_ik
from motion_planning import (
    forward_kinematics_full,
    moveJ,
    moveL,
    resolve_waypoint_to_theta,
)

app = Flask(__name__)
CORS(app)  # Enable CORS for React frontend

# KUKA KR-10 DH Parameters
DHM = np.array(
    [[0, 0, 400], [-90, 25, 0], [0, 560, 0], [-90, 25, 515], [90, 0, 0], [-90, 0, 90]]
)
alpha = DHM[:, 0]
a = DHM[:, 1]
d = DHM[:, 2]
th_offset = np.array([0, 0, -90, 0, 0, 180])
th_inv = np.array([-1, 1, 1, -1, 1, -1])  # Inverse joint multipliers


@app.route("/api/health", methods=["GET"])
def health():
    """Health check endpoint"""
    return jsonify({"status": "ok", "message": "Motion planning API is running"})


@app.route("/api/fk", methods=["POST"])
def forward_kinematics_endpoint():
    """
    Compute forward kinematics

    Request JSON:
    {
        "theta": [angle1, angle2, angle3, angle4, angle5, angle6]  in degrees
    }

    Response: 4x4 transformation matrix
    """
    try:
        data = request.json
        theta = np.array(data["theta"])

        # Compute FK with inverse multipliers applied
        theta_adj = th_inv * theta + th_offset
        T = forward_kinematics_full(theta_adj, alpha, a, d, np.zeros(6))

        return jsonify(
            {
                "success": True,
                "T": T.tolist(),
                "position": T[:3, 3].tolist(),
                "orientation": T[:3, :3].tolist(),
            }
        )
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 400


@app.route("/api/ik", methods=["POST"])
def inverse_kinematics_endpoint():
    """
    Compute inverse kinematics

    Request JSON:
    {
        "T": 4x4 transformation matrix (list of lists)
    }

    Response: Joint angles
    """
    try:
        data = request.json
        T_target = np.array(data["T"])

        # Compute IK
        theta = compute_ik_wrapper(T_target)

        return jsonify({"success": True, "q": theta.tolist()})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 400


@app.route("/api/ik/all", methods=["POST"])
def inverse_kinematics_all_endpoint():
    """
    Compute all inverse kinematics solutions and return closest to seed
    
    Request JSON:
    {
        "T": 4x4 transformation matrix,
        "seedQ": seed joint angles for solution selection
    }
    
    Response: Multiple joint angle solutions with selected closest one
    """
    try:
        data = request.json
        T_target = np.array(data["T"])
        seed_q = np.array(data.get("seedQ", [0, 0, 0, 0, 0, 0]))
        
        # Get all IK solutions
        theta_all, success = solve_ik(T_target, return_all=True)
        
        if not success or theta_all is None:
            raise ValueError("No IK solutions found")
        
        # Ensure theta_all is a list of solutions
        if isinstance(theta_all, np.ndarray):
            if theta_all.ndim == 1:
                # Single solution returned as 1D array
                theta_all = [theta_all]
            else:
                # Multiple solutions as 2D array
                theta_all = [theta_all[i] for i in range(theta_all.shape[0])]
        else:
            # Already a list/iterable
            theta_all = list(theta_all)
        
        if len(theta_all) == 0:
            raise ValueError("No IK solutions found")
        
        # Find closest solution to seed
        best_idx = 0
        best_distance = np.inf
        for j, theta_candidate in enumerate(theta_all):
            theta_candidate = np.array(theta_candidate)
            dist = np.linalg.norm(theta_candidate - seed_q)
            if dist < best_distance:
                best_distance = dist
                best_idx = j
        
        best_solution = np.array(theta_all[best_idx])
        
        return jsonify({
            "success": True,
            "q": best_solution.tolist(),
            "all_solutions": [np.array(t).tolist() for t in theta_all],
            "selected_index": best_idx,
            "num_solutions": len(theta_all)
        })
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 400

def movej_endpoint():
    """
    Generate MoveJ trajectory

    Request JSON - Old format (direct theta):
    {
        "theta_start": [angle1, ..., angle6],
        "theta_end": [angle1, ..., angle6],
        "time": duration_in_seconds,
        "num_points": number_of_trajectory_points (optional, default 100)
    }

    Request JSON - New format (waypoints):
    {
        "start_waypoint": {inputType: 'joint'|'pose', ...},
        "goal_waypoint": {inputType: 'joint'|'pose', ...},
        "time": duration_in_seconds,
        "num_points": number_of_trajectory_points (optional, default 100)
    }
    """
    try:
        data = request.json
        time_duration = data["time"]
        num_points = data.get("num_points", 100)

        # Handle both old format (direct theta) and new format (waypoints)
        if "start_waypoint" in data and "goal_waypoint" in data:
            # New waypoint format
            theta_start = resolve_waypoint_to_theta(data["start_waypoint"])
            theta_end = resolve_waypoint_to_theta(
                data["goal_waypoint"], seed_theta=theta_start
            )
        else:
            # Old format (backward compatibility)
            theta_start = np.array(data["theta_start"])
            theta_end = np.array(data["theta_end"])

        trajectory = moveJ(theta_start, theta_end, time_duration, num_points)

        return jsonify({"success": True, "trajectory": trajectory})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 400


@app.route("/api/moveL", methods=["POST"])
def movel_endpoint():
    """
    Generate MoveL trajectory

    Request JSON - Old format (direct theta):
    {
        "theta_start": [angle1, ..., angle6],
        "theta_end": [angle1, ..., angle6],
        "time": duration_in_seconds,
        "num_points": number_of_trajectory_points (optional, default 100)
    }

    Request JSON - New format (waypoints):
    {
        "start_waypoint": {inputType: 'joint'|'pose', ...},
        "goal_waypoint": {inputType: 'joint'|'pose', ...},
        "time": duration_in_seconds,
        "num_points": number_of_trajectory_points (optional, default 100)
    }
    """
    try:
        data = request.json
        time_duration = data["time"]
        num_points = data.get("num_points", 100)

        # Handle both old format (direct theta) and new format (waypoints)
        if "start_waypoint" in data and "goal_waypoint" in data:
            # New waypoint format - pass waypoints directly to moveL
            trajectory = moveL(
                time_duration=time_duration,
                num_points=num_points,
                start_waypoint=data["start_waypoint"],
                goal_waypoint=data["goal_waypoint"],
            )
        else:
            # Old format (backward compatibility) - pass theta directly
            theta_start = np.array(data["theta_start"])
            theta_end = np.array(data["theta_end"])
            trajectory = moveL(
                theta_start=theta_start,
                theta_end=theta_end,
                time_duration=time_duration,
                num_points=num_points,
            )

        return jsonify({"success": True, "trajectory": trajectory})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 400


@app.route("/api/getCurrentPose", methods=["POST"])
def get_current_pose():
    """
    Get current end-effector pose from joint angles

    Request JSON:
    {
        "theta": [angle1, ..., angle6]
    }
    """
    try:
        data = request.json
        theta = np.array(data["theta"])

        T = forward_kinematics_full(theta, alpha, a, d, th_offset)

        return jsonify(
            {
                "success": True,
                "position": T[:3, 3].tolist(),
                "orientation": T[:3, :3].tolist(),
                "T": T.tolist(),
            }
        )
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 400


def compute_ik_wrapper(T_target):
    """
    Wrapper for IK computation.
    Uses the refactored inverse_kinematics_refactored module.
    """
    theta, success = solve_ik(T_target)
    if not success or theta is None:
        raise ValueError("No IK solution found for target pose")
    return theta


if __name__ == "__main__":
    # Run on port 5000 (development)
    # For production, use a proper WSGI server (gunicorn, etc.)
    app.run(debug=True, host="0.0.0.0", port=5000)
