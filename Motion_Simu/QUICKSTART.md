# Motion Planning Web Application - Quick Start Guide

## 📋 Project Overview

This is a complete interactive motion planning application with real-time 3D visualization and trajectory analysis.

### ✨ Key Features:
- **3D Robot Visualization** - Interactive Three.js rendering with waypoint visualization
- **Waypoint Management** - Add, remove, and manage motion waypoints
- **Trajectory Generation** - Automatic linear interpolation between waypoints
- **Real-time Plotting** - Chart.js visualization of trajectory coordinates
- **Responsive UI** - Modern dark-themed interface with smooth interactions

---

## 🚀 Getting Started

### Step 1: Install Dependencies

Due to PowerShell execution policy restrictions, run one of these commands:

**Option A - Using Batch File (Windows):**
```batch
install.bat
```

**Option B - Using Command Prompt Directly:**
```cmd
npm install
```

**Option C - Using Node.js directly:**
Open Command Prompt (cmd) and run:
```cmd
cd /d "C:\Users\Isamu\OneDrive\Desktop\Programming\ME5463 Real Time\RoboDK_Motion_Planning_Simu\Motion_Simu"
npm install
```

**Option D - Bypass PowerShell policy (Advanced):**
```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
npm install
```

### Step 2: Start Development Server

After installation completes, run:

```bash
npm run dev
```

The application will open automatically at `http://localhost:3000`

---

## 🎮 How to Use

### Adding Waypoints

1. **Input Coordinates**
   - Enter X, Y, and Z values in the sidebar input fields
   - Coordinates represent positions in 3D space (in arbitrary units)

2. **Add Waypoint**
   - Click "Add Waypoint" button OR press Enter
   - Waypoint appears in the list below with a color code

3. **Manage Waypoints**
   - View numbered list of all waypoints
   - Click × to remove individual waypoints
   - Click "Clear All" to remove all waypoints

### Example Waypoints

Try adding these waypoints for a simple path:
- Waypoint 1: X=0, Y=0, Z=0
- Waypoint 2: X=3, Y=2, Z=1
- Waypoint 3: X=5, Y=5, Z=2

### Generating Trajectories

1. **Add at least 2 waypoints** (minimum requirement)
2. **Click "Generate Trajectory"**
3. **View Results:**
   - Green line in 3D view shows the path
   - Chart below shows X, Y, Z position changes over time

### Interacting with 3D View

- **Rotate**: Right-click and drag mouse
- **Zoom**: Scroll mouse wheel
- **Pan**: Middle-click and drag
- **Reset**: Refresh the page

---

## 📁 Project Structure

```
Motion_Simu/
├── src/
│   ├── components/
│   │   ├── RobotVisualizer.jsx       # 3D visualization (Three.js)
│   │   ├── RobotVisualizer.css
│   │   ├── WaypointInput.jsx         # Control panel (React)
│   │   ├── WaypointInput.css
│   │   ├── TrajectoryPlotter.jsx     # Chart visualization (Chart.js)
│   │   └── TrajectoryPlotter.css
│   ├── App.jsx                       # Main component
│   ├── App.css
│   ├── index.css
│   └── main.jsx
├── package.json                      # Dependencies list
├── vite.config.js                    # Build configuration
├── index.html                        # HTML entry point
├── install.bat                       # Windows installation batch
├── QUICKSTART.md                     # This file
└── README.md                         # Full documentation
```

---

## 🔧 Available Commands

```bash
# Start development server (hot reload)
npm run dev

# Build for production
npm run build

# Preview production build
npm run preview

# Check for syntax errors
npm run lint
```

---

## 🛠️ Troubleshooting

### Issue: "npm is not recognized"
**Solution**:
- Reinstall Node.js from https://nodejs.org/
- Restart your terminal/IDE
- Verify installation: `node --version && npm --version`

### Issue: "ERESOLVE unable to resolve dependency tree"
**Solution**:
```bash
npm install --legacy-peer-deps
```

### Issue: Port 3000 already in use
**Solution**:
- Change port in `vite.config.js`:
  ```javascript
  server: {
    port: 3001,  // or another available port
    open: true
  }
  ```

### Issue: WebGL not working in 3D view
**Solution**:
- Use a modern browser (Chrome, Firefox, Edge)
- Check that hardware acceleration is enabled
- Try disabling browser extensions

### Issue: Application crashes or freezes
**Solution**:
- Open browser DevTools (F12)
- Check Console tab for error messages
- Clear browser cache
- Restart development server

---

## 📚 Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| react | 18.2.0 | UI framework |
| react-dom | 18.2.0 | DOM rendering |
| three | r128 | 3D graphics |
| chart.js | 4.4.0 | Data visualization |
| react-chartjs-2 | 5.2.0 | Chart.js React wrapper |
| vite | 5.0.12 | Build tool |

---

## 🎨 Customization

### Changing Colors
Edit color variables in `src/index.css`:
```css
:root {
  --color-primary: #667eea;      /* Change primary color */
  --color-secondary: #764ba2;
  --color-success: #48bb78;
  ...
}
```

### Modifying Trajectory Algorithm
Edit `generateTrajectory()` function in `App.jsx` to implement:
- Cubic spline interpolation
- Bezier curves
- RRT algorithm
- Joint-space interpolation

### Customizing Robot Appearance
Edit `RobotVisualizer.jsx` to:
- Change robot geometry
- Add robot links/joints
- Implement 3D model loading
- Add collision visualization

---

## 📖 Learning Resources

- **Three.js**: https://threejs.org/docs/
- **React**: https://react.dev/
- **Chart.js**: https://www.chartjs.org/
- **Vite**: https://vitejs.dev/guide/

---

## ✅ Next Steps

1. ✓ Install dependencies with `npm install`
2. ✓ Run `npm run dev` to start the server
3. ✓ Open http://localhost:3000 in your browser
4. ✓ Try adding waypoints and generating trajectories
5. ✓ Explore the 3D visualization
6. ✓ Check console (F12) for any issues

---

## 💡 Tips & Tricks

- **Keyboard Shortcut**: Press Enter in coordinate inputs to quickly add waypoints
- **Responsive Design**: Works on desktop, tablet, and mobile browsers
- **Real-time Updates**: Changes appear instantly without page refresh
- **Export Data**: Open browser DevTools to export waypoint/trajectory data
- **Performance**: Application handles 1000+ trajectory points smoothly

---

## 📞 Support

If you encounter issues:
1. Check this QUICKSTART.md file
2. Read the full README.md for detailed documentation
3. Check browser console (F12) for error messages
4. Ensure all dependencies are installed: `npm list`
5. Try clearing cache and reinstalling: `rm -rf node_modules && npm install`

---

**Created**: March 2026
**Course**: ME5463 Real Time Motion Planning
**Last Updated**: March 4, 2026
