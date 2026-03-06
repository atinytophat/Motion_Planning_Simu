import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { URDFLoader } from 'three/examples/jsm/loaders/URDFLoader.js';

/**
 * RobotVisualizer with URDF Loading
 *
 * Loads KUKA robot from URDF and displays in Three.js
 * Allows real-time joint angle updates
 */

function RobotVisualizer(props) {
  const containerRef = useRef(null);
  const sceneRef = useRef(null);
  const robotRef = useRef(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    if (!containerRef.current) return;

    // ===== THREE.JS SETUP =====
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a1a2e);
    sceneRef.current = scene;

    // Camera
    const camera = new THREE.PerspectiveCamera(
      75,
      containerRef.current.clientWidth / containerRef.current.clientHeight,
      0.1,
      1000
    );
    camera.position.set(1.5, 1.5, 1.5);
    camera.lookAt(0, 0, 0.5);

    // Renderer
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(containerRef.current.clientWidth, containerRef.current.clientHeight);
    renderer.shadowMap.enabled = true;
    containerRef.current.appendChild(renderer.domElement);

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(5, 10, 7);
    directionalLight.castShadow = true;
    directionalLight.shadow.mapSize.width = 2048;
    directionalLight.shadow.mapSize.height = 2048;
    scene.add(directionalLight);

    // Grid
    const gridHelper = new THREE.GridHelper(3, 30, 0x444444, 0x222222);
    scene.add(gridHelper);

    // Axes
    const axesHelper = new THREE.AxesHelper(0.5);
    scene.add(axesHelper);

    // ===== LOAD URDF =====
    const loader = new URDFLoader();
    loader.loadMeshGeometries = true; // Load visual meshes if available

    loader.load(
      '/KUKA-KR-10.urdf', // Path to URDF file
      (robot) => {
        robotRef.current = robot;
        scene.add(robot);
        setLoading(false);
      },
      (progress) => {
        // Loading progress tracked silently
      },
      (err) => {
        setError(`Failed to load URDF: ${err.message}`);
        setLoading(false);
      }
    );

    // ===== ANIMATION LOOP =====
    const animate = () => {
      requestAnimationFrame(animate);
      renderer.render(scene, camera);
    };
    animate();

    // ===== HANDLE RESIZE =====
    const handleResize = () => {
      if (!containerRef.current) return;
      const width = containerRef.current.clientWidth;
      const height = containerRef.current.clientHeight;
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      renderer.setSize(width, height);
    };
    window.addEventListener('resize', handleResize);

    // ===== CLEANUP =====
    return () => {
      window.removeEventListener('resize', handleResize);
      renderer.dispose();
      containerRef.current?.removeChild(renderer.domElement);
    };
  }, []);

  // Update robot pose when joint angles change
  useEffect(() => {
    if (!robotRef.current || !props.jointAngles) return;

    // Convert degrees to radians and update joint angles
    const joints = Object.values(robotRef.current.joints);
    const angles = props.jointAngles; // Should be array of 6 angles in degrees

    joints.forEach((joint, idx) => {
      if (idx < angles.length) {
        // Convert degrees to radians
        joint.setJointValue(THREE.MathUtils.degToRad(angles[idx]));
      }
    });

  }, [props.jointAngles]);

  return (
    <div style={{ width: '100%', height: '100%', position: 'relative' }}>
      <div ref={containerRef} style={{ width: '100%', height: '100%' }} />

      {loading && (
        <div style={{
          position: 'absolute',
          top: '50%',
          left: '50%',
          transform: 'translate(-50%, -50%)',
          color: '#64c8ff',
          fontSize: '18px',
          textAlign: 'center'
        }}>
          <p>Loading KUKA Robot...</p>
          <p style={{ fontSize: '12px', opacity: 0.7 }}>Make sure KUKA-KR-10.urdf is in the public folder</p>
        </div>
      )}

      {error && (
        <div style={{
          position: 'absolute',
          top: '10px',
          left: '10px',
          background: 'rgba(244, 67, 54, 0.2)',
          border: '1px solid rgba(244, 67, 54, 0.5)',
          color: '#ff9999',
          padding: '12px',
          borderRadius: '4px',
          maxWidth: '300px',
          fontSize: '12px'
        }}>
          ❌ {error}
        </div>
      )}
    </div>
  );
}

export default RobotVisualizer;
