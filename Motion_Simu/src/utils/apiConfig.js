/**
 * API Configuration and Helper Functions
 * Provides convenient methods to call the Python backend API endpoints
 */

const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:5000/api';

class APIClient {
  constructor(baseUrl) {
    this.baseUrl = baseUrl;
  }

  /**
   * Generic request method
   */
  async request(endpoint, method = 'POST', data = null) {
    try {
      const options = {
        method,
        headers: {
          'Content-Type': 'application/json',
        },
      };

      if (data) {
        options.body = JSON.stringify(data);
      }

      const response = await fetch(`${this.baseUrl}${endpoint}`, options);

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      return { success: false, error: error.message };
    }
  }

  /**
   * Forward Kinematics
   * Compute end-effector pose from joint angles
   *
   * @param {Array} theta - Joint angles [θ1, θ2, θ3, θ4, θ5, θ6] in degrees
   * @returns {Object} { success, T, position, orientation }
   */
  async fk(theta) {
    return this.request('/fk', 'POST', { theta });
  }

  /**
   * Inverse Kinematics
   * Compute joint angles from target end-effector pose
   *
   * @param {Array[Array]} T - 4x4 transformation matrix (array of arrays)
   * @returns {Object} { success, theta }
   */
  async ik(T) {
    return this.request('/ik', 'POST', { T });
  }

  /**
   * MoveJ - Joint Space Interpolation
   * Generate trajectory for joint-space motion
   *
   * @param {Array} theta_start - Starting joint angles
   * @param {Array} theta_end - Ending joint angles
   * @param {Number} time - Motion duration in seconds
   * @param {Number} num_points - Number of trajectory points (optional)
   * @returns {Object} { success, trajectory }
   */
  async moveJ(theta_start, theta_end, time, num_points = 100) {
    return this.request('/moveJ', 'POST', {
      theta_start,
      theta_end,
      time,
      num_points,
    });
  }

  /**
   * MoveL - Linear Cartesian Motion
   * Generate trajectory for linear end-effector motion
   *
   * @param {Array} theta_start - Starting joint angles
   * @param {Array[Array]} T_end - Target transformation matrix
   * @param {Number} time - Motion duration in seconds
   * @param {Number} num_points - Number of trajectory points (optional)
   * @returns {Object} { success, trajectory }
   */
  async moveL(theta_start, T_end, time, num_points = 100) {
    return this.request('/moveL', 'POST', {
      theta_start,
      T_end,
      time,
      num_points,
    });
  }

  /**
   * Get Current Pose
   * Compute end-effector pose from joint angles
   *
   * @param {Array} theta - Joint angles
   * @returns {Object} { success, position, orientation, T }
   */
  async getCurrentPose(theta) {
    return this.request('/getCurrentPose', 'POST', { theta });
  }

  /**
   * Health Check
   * Verify that the API is running
   *
   * @returns {Object} { success, status, message }
   */
  async health() {
    return this.request('/health', 'GET');
  }
}

// Create singleton instance
export const api = new APIClient(API_BASE_URL);

/**
 * Utility Functions
 */

/**
 * Convert transformation matrix to position + quaternion
 */
export function matrixToQuaternion(T) {
  const R = T.slice(0, 3).map(row => row.slice(0, 3));
  const position = [T[0][3], T[1][3], T[2][3]];

  // Simple conversion (there are better methods for numerical stability)
  const trace = R[0][0] + R[1][1] + R[2][2];
  let qw, qx, qy, qz;

  if (trace > 0) {
    const S = Math.sqrt(trace + 1.0) * 2;
    qw = 0.25 * S;
    qx = (R[2][1] - R[1][2]) / S;
    qy = (R[0][2] - R[2][0]) / S;
    qz = (R[1][0] - R[0][1]) / S;
  } else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
    const S = Math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2;
    qw = (R[2][1] - R[1][2]) / S;
    qx = 0.25 * S;
    qy = (R[0][1] + R[1][0]) / S;
    qz = (R[0][2] + R[2][0]) / S;
  } else if (R[1][1] > R[2][2]) {
    const S = Math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2;
    qw = (R[0][2] - R[2][0]) / S;
    qx = (R[0][1] + R[1][0]) / S;
    qy = 0.25 * S;
    qz = (R[1][2] + R[2][1]) / S;
  } else {
    const S = Math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2;
    qw = (R[1][0] - R[0][1]) / S;
    qx = (R[0][2] + R[2][0]) / S;
    qy = (R[1][2] + R[2][1]) / S;
    qz = 0.25 * S;
  }

  return { position, quaternion: [qx, qy, qz, qw] };
}

/**
 * Convert position + quaternion to transformation matrix
 */
export function quaternionToMatrix(position, quaternion) {
  const [x, y, z] = position;
  const [qx, qy, qz, qw] = quaternion;

  const R = [
    [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
    [2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
    [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)],
  ];

  const T = [
    [...R[0], x, 0],
    [...R[1], y, 0],
    [...R[2], z, 0],
    [0, 0, 0, 1],
  ];

  return T;
}

/**
 * Normalize a quaternion
 */
export function normalizeQuaternion(q) {
  const magnitude = Math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  return [q[0]/magnitude, q[1]/magnitude, q[2]/magnitude, q[3]/magnitude];
}

/**
 * Create identity transformation matrix
 */
export function identityMatrix() {
  return [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
}
