# Interactive Motion Planning Application

A web-based interactive motion planning and visualization tool built with React, Three.js, and Chart.js. This application allows users to define robot waypoints, generate trajectories, and visualize motion planning in real-time with 3D graphics and trajectory analysis.

## Features

### 🤖 3D Robot Visualization
- Real-time 3D visualization using Three.js
- Interactive scene with grid and axes helpers
- Robot representation with dynamic waypoint visualization
- Color-coded waypoints with numbered labels
- Trajectory path visualization

### 📍 Waypoint Management
- Input waypoints with X, Y, Z coordinates
- Remove individual waypoints or clear all at once
- Real-time waypoint list display
- Numeric input with validation

### 📊 Trajectory Plotting
- Interactive Chart.js graphs showing trajectory over time
- Multi-axis visualization (X, Y, Z positions)
- Responsive charts with hover information
- Real-time trajectory generation

### 🎨 Modern UI/UX
- Dark theme with gradient backgrounds
- Responsive design for desktop and mobile
- Real-time updates and instant feedback
- Smooth animations and transitions

## Technologies

- **Frontend Framework**: React 18.2
- **3D Graphics**: Three.js (r128)
- **Data Visualization**: Chart.js 4.4 with react-chartjs-2
- **Build Tool**: Vite 5.0
- **Styling**: CSS3 with custom properties

## Project Structure

```
.
├── src/
│   ├── components/
│   │   ├── RobotVisualizer.jsx      # Three.js 3D visualization
│   │   ├── WaypointInput.jsx         # Waypoint management UI
│   │   ├── TrajectoryPlotter.jsx     # Chart.js trajectory plotting
│   │   ├── WaypointInput.css
│   │   └── TrajectoryPlotter.css
│   ├── App.jsx                       # Main application component
│   ├── App.css                       # Application styles
│   ├── index.css                     # Global styles
│   └── main.jsx                      # Application entry point
├── index.html                        # HTML template
├── vite.config.js                    # Vite configuration
├── package.json                      # Project dependencies
└── README.md                         # This file
```

## Installation

### Prerequisites
- Node.js 16+ and npm/yarn installed
- Modern web browser with WebGL support

### Setup

1. **Navigate to the project directory:**
   ```bash
   cd Motion_Simu
   ```

2. **Install dependencies:**
   ```bash
   npm install
   ```

3. **Start the development server:**
   ```bash
   npm run dev
   ```

   The application will automatically open at `http://localhost:3000`

## Usage

### Adding Waypoints

1. **Enter Coordinates**: Use the input fields on the left sidebar to enter X, Y, and Z coordinates
2. **Add Waypoint**: Click the "Add Waypoint" button or press Enter
3. **View List**: See your waypoints displayed in the list below
4. **Remove**: Click the × button next to any waypoint to remove it

### Generating Trajectory

1. **Minimum Points**: Add at least 2 waypoints
2. **Generate**: Click the "Generate Trajectory" button
3. **Visualize**: Watch the trajectory appear in:
   - 3D visualization (green line connecting waypoints)
   - Trajectory plot (showing X, Y, Z positions over time)

### Viewing 3D Visualization

- **Pan**: Right-click and drag to rotate the view
- **Zoom**: Scroll to zoom in/out
- **Grid**: Visual reference grid for spatial orientation
- **Axes**: Color-coded axes (Red=X, Green=Y, Blue=Z)
- **Waypoints**: Colored spheres representing waypoints
- **Trajectory**: Green line showing the planned path

## Build

To create a production build:

```bash
npm run build
```

The optimized files will be in the `dist/` directory.

## Preview Production Build

```bash
npm run preview
```

## Development

### Adding New Components

Create new components in `src/components/`:

```jsx
export default function MyComponent() {
  return <div>My Component</div>
}
```

### Modifying Trajectory Generation

Edit the `generateTrajectory` function in `App.jsx` to implement different interpolation methods:
- Linear interpolation (current)
- Cubic spline interpolation
- Bezier curves
- RRT (Rapidly-exploring Random Tree)

### Extending Robot Visualization

Modify `RobotVisualizer.jsx` to:
- Add robot arms/links using Three.js meshes
- Implement forward/inverse kinematics
- Add collision detection visualization
- Support different robot models

## Advanced Features (Future Enhancements)

- [ ] Load/save trajectory files (JSON, CSV)
- [ ] Import robot models (URDF, STL)
- [ ] Collision detection
- [ ] Path optimization algorithms
- [ ] Animation playback controls
- [ ] Trajectory speed/acceleration profiles
- [ ] Export visualizations as images/videos
- [ ] Multi-robot scenarios
- [ ] Real-time ROS integration

## Browser Support

- Chrome/Chromium 90+
- Firefox 88+
- Safari 14+
- Edge 90+

## Performance

- Smooth 60 FPS rendering
- Handles 1000+ trajectory points efficiently
- Responsive UI even with large waypoint sets

## Troubleshooting

### Application won't start
- Ensure Node.js is installed: `node --version`
- Clear node_modules and reinstall: `rm -rf node_modules && npm install`
- Check if port 3000 is available

### 3D visualization not showing
- Ensure WebGL is enabled in your browser
- Try a different browser (Chrome recommended)
- Check browser console for errors (F12)

### Chart not updating
- Verify trajectory generation completed successfully
- Check that waypoints are properly positioned
- Ensure Chart.js is properly imported

## License

This project is created for educational purposes in ME5463 Real Time Motion Planning course.

## Resources

- [Three.js Documentation](https://threejs.org/docs/)
- [React Documentation](https://react.dev/)
- [Chart.js Documentation](https://www.chartjs.org/docs/)
- [Vite Guide](https://vitejs.dev/guide/)

## Contributing

For improvements and bug fixes, please create an issue or pull request.

---

**Created**: March 2026
**Course**: ME5463 Real Time Motion Planning
**Purpose**: Interactive Robot Motion Planning Simulation
