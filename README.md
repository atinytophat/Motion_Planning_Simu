# KUKA KR-10 Motion Planning Simulator

A comprehensive real-time motion planning simulator for the KUKA KR-10 R1100-2 collaborative robot featuring forward kinematics (FK), inverse kinematics (IK) solvers, and linear motion (MoveL) planning with Three.js 3D visualization.

## 🚀 Quick Links

- **Live Web Demo:** https://motion-planning-simu2.vercel.app/
  - Frontend deployed on Vercel
  - Requires local Flask backend running (`python app.py`)
- **GitHub Repository:** https://github.com/atinytophat/Motion_Planning_Simu

---

## ✨ Features

### Kinematics Engine
- **Forward Kinematics (FK):** Analytical computation of end-effector pose from joint angles
- **Inverse Kinematics (IK):** Full 6-DOF solution with multiple configurations and singularity handling
- **Wrist Singularity Detection:** Special handling for wrist singularities with optimized joint selection

### Motion Planning
- **MoveJ (Joint Motion):** Direct joint-space interpolation
- **MoveL (Linear Motion):** Cartesian linear motion via transformation matrix interpolation and continuous IK solving
- **Time Scaling:** Multiple motion profiles (linear, cubic, quintic) for smooth acceleration/deceleration
- **Real-time Trajectory Preview:** Live visualization during planning

### Visualization
- **3D Robot Model:** URDF-based KUKA KR-10 with FK-computed joint positions
- **Waypoint Markers:** Visual representation of start/goal positions
- **Trajectory Plotting:** 3D trajectory paths for planned motions
- **Interactive Controls:** Sliders for joint angle adjustment and Cartesian pose control

### Dual-Mode Interface
- **Motion Planning Tab:** Plan complete trajectories with waypoint sequencing
- **Static Pose Tab:** Direct joint/cartesian pose manipulation with real-time FK/IK
- **Responsive Visualization:** Synchronized reactor showing all mode changes

---

## 📋 Prerequisites

### System Requirements
- **Node.js:** v18+ (for React/Vite frontend)
- **Python:** 3.9+ (for Flask backend)
- **npm:** v9+ (Node Package Manager)

### Hardware
- Minimum 2GB RAM
- Modern web browser (Chrome, Firefox, Safari, Edge)

---

## 🔧 Installation

### 1. Clone the Repository
```bash
git clone https://github.com/atinytophat/Motion_Planning_Simu.git
cd Motion_Planning_Simu
```

### 2. Install Backend Dependencies
```bash
pip install -r requirements.txt
```

**If using virtual environment (recommended):**
```bash
python -m venv venv
venv\Scripts\activate  # Windows
source venv/bin/activate  # macOS/Linux
pip install -r requirements.txt
```

**Required Packages:**
- Flask (web server)
- NumPy (numerical computations)
- SciPy (scientific functions)

### 3. Install Frontend Dependencies
```bash
cd Motion_Simu
npm install
cd ..
```

---

## ▶️ Running the Application

### Start the Flask Backend
```bash
python app.py
```
- Backend runs on `http://localhost:5000`
- Health check endpoint: `http://localhost:5000/api/health`

### Start the React Frontend (in a new terminal)
```bash
cd Motion_Simu
npm run dev
```
- Frontend runs on `http://localhost:5173`
- Opens automatically in your default browser

### Access the Application
- **Local:** http://localhost:5173
- **Live (with local backend):** https://motion-planning-simu2.vercel.app/

---

## 📁 Project Structure

```
Motion_Planning_Simu/
├── app.py                          # Flask backend server
├── forward_kinematics.py           # FK solver with DH parameters
├── inverse_kinematics.py           # IK solver (multiple solutions)
├── inverse_kinematics_refactored.py # IK optimization variants
├── motion_planning.py              # Motion planning algorithms
├── requirements.txt                # Python dependencies
│
├── Motion_Simu/                    # React/Vite frontend
│   ├── vite.config.js
│   ├── package.json
│   ├── src/
│   │   ├── App.jsx                 # Main app component
│   │   ├── main.jsx                # Entry point
│   │   ├── components/
│   │   │   ├── RobotVisualizer.jsx       # Three.js 3D robot
│   │   │   ├── MotionControl.jsx         # Trajectory playback
│   │   │   ├── WaypointInput.jsx         # Waypoint editor
│   │   │   └── TrajectoryPlotter.jsx     # Path visualization
│   │   └── utils/
│   │       ├── apiConfig.js              # Backend API wrapper
│   │       ├── kinematicsBridge.js       # FK/IK calls
│   │       ├── planners.js               # MoveJ/MoveL algorithms
│   │       ├── motionProfiles.js         # Time scaling functions
│   │       └── math3d.js                 # Rotation utilities
│   └── public/
│       └── KUKA-KR-10.urdf         # Robot model definition
│
├── MATLAB/                         # Reference implementations
│   ├── FK.m                        # Forward kinematics (MATLAB)
│   ├── IK.m                        # Inverse kinematics (MATLAB)
│   ├── modDH.m                     # DH parameter computation
│   └── [other reference files]
│
└── RoboDK/                         # RoboDK project files
    ├── DK.rdk                      # RoboDK simulation
    └── KUKA_KR10_R1100_2.py       # RoboDK Python API
```

---

## 🎮 Usage Guide

### Motion Planning Tab
1. **Set Waypoints:** Click buttons or manually enter joint/cartesian values
2. **Select Motion Type:** Choose MoveJ or MoveL from dropdown
3. **Configure Motion:** Adjust speed, profile, and sample rate
4. **Generate:** Click "Generate Trajectory" to compute motion plan
5. **Playback:** Use animation controls to visualize the trajectory

### Static Pose Tab
1. **Joint Mode:** Adjust 6 joint sliders (0°-180° range)
2. **Convert to Cartesian:** Click button to compute FK
3. **Cartesian Mode:** Manually set X, Y, Z, Roll, Pitch, Yaw
4. **Convert to Joints:** Click button to compute IK
5. **Send to Waypoint:** Use "→ Start/Goal Waypoint" buttons

### Visualization
- **Rotate View:** Click and drag with mouse
- **Zoom:** Scroll wheel
- **Pan:** Right-click and drag
- **Wireframe:** Toggle in settings (if available)

---

## 🔄 Unit System

**Important:** The system uses consistent internal units:

| Component | Unit | Notes |
|-----------|------|-------|
| Joint Angles | Degrees (°) | All input/output |
| Backend Communication | Millimeters (mm) | T-matrix positions |
| Display/Visualization | Meters (m) | Converted from mm for display |
| DH Parameters | Millimeters (mm) | KUKA KR-10 specs |

**Example:** A position of 1000 mm in the backend displays as 1.0 m in the UI.

---

## 🤖 Robot Parameters

**KUKA KR-10 R1100-2:**
- **DOF:** 6 (full rotation wrist)
- **Reach:** ~1100 mm
- **Base Height:** 400 mm
- **Wrist Offset:** 90 mm

**DH Parameters (Modified DH Convention):**
```
Link | α (deg) | a (mm) | d (mm)
-----|---------|--------|--------
  1  |    0    |   0    |  400
  2  |   -90   |  25    |    0
  3  |    0    |  560   |    0
  4  |   -90   |  25    |  515
  5  |    90   |   0    |    0
  6  |   -90   |   0    |   90
```

---

## 📊 API Endpoints

### Backend (Flask)

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/api/health` | GET | Health check |
| `/api/forward-kinematics` | POST | Compute FK from joint angles |
| `/api/inverse-kinematics` | POST | Compute IK from T-matrix |
| `/api/inverse-kinematics/all` | POST | Get all IK solutions |

**Example Request (FK):**
```json
{
  "theta": [0, 0, 0, 0, 0, 0]
}
```

**Example Request (IK):**
```json
{
  "T": [[1, 0, 0, 1000], [0, 1, 0, 0], [0, 0, 1, 1400], [0, 0, 0, 1]]
}
```

---

## 🧪 Testing

### Run Backend Tests (if available)
```bash
pytest tests/
```

### Manual Testing Checklist
- [ ] FK returns correct end-effector position
- [ ] IK produces valid joint angles
- [ ] MoveL generates smooth linear trajectories
- [ ] UI responds to slider changes in real-time
- [ ] Singularity handling works correctly

---

## 📚 Mathematical Background

### Forward Kinematics
Computed using Modified DH (Denavit-Hartenberg) parameters:
- T = T₁ × T₂ × ... × T₆
- Each Tᵢ = Rot(z, θᵢ) × Trans(0, 0, dᵢ) × Trans(aᵢ, 0, 0) × Rot(x, αᵢ)

### Inverse Kinematics
- **Arm (θ₁, θ₂, θ₃):** Analytical 2D wrist-center position solution
- **Wrist (θ₄, θ₅, θ₆):** Z-Y-X Euler angle decomposition
- **Singularity Handling:** When θ₅ ≈ 0°, only φ = θ₄ + θ₆ is observable

### Linear Motion (MoveL)
- Relative transform: ΔT = T₋₁ₛₜₐᵣₜ × T_goal
- Interpolation: T(τ) = T_start × (ΔT)^s, where s ∈ [0, 1]
- IK solved at each sample point with seed from previous solution

---

## 📂 MATLAB Reference Files

The `MATLAB/` directory contains reference implementations:
- **FK.m** – Forward kinematics script with testing
- **IK.m** – Inverse kinematics with wrist singularity handling
- **modDH.m** – Modified DH transformation computation
- Other utilities for verification and analysis

These are **reference only** and not used by the application.

---

## 🐛 Troubleshooting

### Backend Won't Start
```
Error: Address already in use
```
- Port 5000 is in use. Either kill the process or change port in `app.py`

### Frontend Can't Connect to Backend
```
CORS error or timeout
```
- Ensure Flask is running: `http://localhost:5000/api/health` should return `{"status": "ok"}`
- Check `apiConfig.js` has correct backend URL

### IK Returns No Solution
- Target may be unreachable (outside workspace)
- Check joint limits in `inverse_kinematics.py`
- Try different starting positions

### Visualization Not Loading
- Ensure KUKA-KR-10.urdf exists in `Motion_Simu/public/`
- Check browser console (F12) for errors
- Try a different browser

---

## 📝 License

**Academic Use Only** – ME5463 Course Project

---

## 👨‍💻 Author

**Isamu** – Real-time Motion Planning Course Project (Spring 2026)

---

## 📧 Support

For issues or questions:
1. Check the troubleshooting section above
2. Review `app.py` and `Motion_Simu/src/App.jsx` logs
3. Consult MATLAB reference implementations for kinematics verification

---

## 🎯 Future Enhancements

- [ ] Import/export trajectory files
- [ ] Collision detection
- [ ] Multi-waypoint sequencing with blending
- [ ] Real hardware integration (RoboDK API)
- [ ] Advanced motion profiles (s-curve jerk limiting)
- [ ] Web-based trajectory optimization
