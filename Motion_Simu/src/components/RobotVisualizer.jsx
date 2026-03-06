import React, { useEffect, useRef, useState } from 'react'
import * as THREE from 'three'

// Simple OrbitControls for mouse interaction
class SimpleOrbitControls {
  constructor(camera, domElement) {
    this.camera = camera
    this.domElement = domElement
    this.target = new THREE.Vector3(0, 0, 0.5)
    this.radius = this.camera.position.length()
    this.theta = Math.atan2(this.camera.position.y, this.camera.position.x)
    this.phi = Math.acos(Math.max(-1, Math.min(1, this.camera.position.z / this.radius)))

    this.isDragging = false
    this.isRightClick = false
    this.previousMousePosition = { x: 0, y: 0 }

    this.domElement.addEventListener('mousedown', this.onMouseDown.bind(this), false)
    this.domElement.addEventListener('mousemove', this.onMouseMove.bind(this), false)
    this.domElement.addEventListener('mouseup', this.onMouseUp.bind(this), false)
    this.domElement.addEventListener('wheel', this.onMouseWheel.bind(this), false)
  }

  onMouseDown(event) {
    this.isDragging = true
    this.isRightClick = event.button === 2
    this.previousMousePosition = { x: event.clientX, y: event.clientY }
  }

  onMouseMove(event) {
    if (!this.isDragging) return

    const deltaX = event.clientX - this.previousMousePosition.x
    const deltaY = event.clientY - this.previousMousePosition.y

    if (this.isRightClick) {
      // Rotate (right-click drag)
      this.theta -= deltaX * 0.01
      this.phi -= deltaY * 0.01
      this.phi = Math.max(0.1, Math.min(Math.PI - 0.1, this.phi))
    } else {
      // Pan/Drag (left-click drag)
      const panSpeed = 0.005
      const panX = new THREE.Vector3().crossVectors(this.camera.up,
        new THREE.Vector3().subVectors(this.target, this.camera.position).normalize()).normalize()
      panX.multiplyScalar(-deltaX * panSpeed * this.radius)
      const panY = new THREE.Vector3().copy(this.camera.up).multiplyScalar(deltaY * panSpeed * this.radius)
      this.target.add(panX).add(panY)
    }

    this.previousMousePosition = { x: event.clientX, y: event.clientY }
    this.updateCameraPosition()
  }

  onMouseUp() {
    this.isDragging = false
  }

  onMouseWheel(event) {
    event.preventDefault()
    const zoomSpeed = 0.1
    this.radius += event.deltaY > 0 ? zoomSpeed : -zoomSpeed
    this.radius = Math.max(0.5, Math.min(10, this.radius))
    this.updateCameraPosition()
  }

  updateCameraPosition() {
    // Z-up spherical coordinates
    this.camera.position.x = this.target.x + this.radius * Math.sin(this.phi) * Math.cos(this.theta)
    this.camera.position.y = this.target.y + this.radius * Math.sin(this.phi) * Math.sin(this.theta)
    this.camera.position.z = this.target.z + this.radius * Math.cos(this.phi)
    this.camera.lookAt(this.target)
  }

  syncFromCamera() {
    // Recalculate theta, phi, and radius from current camera position (Z-up)
    const relPos = new THREE.Vector3().subVectors(this.camera.position, this.target)
    this.radius = relPos.length()
    this.phi = Math.acos(Math.max(-1, Math.min(1, relPos.z / this.radius)))
    this.theta = Math.atan2(relPos.y, relPos.x)
  }

  dispose() {
    this.domElement.removeEventListener('mousedown', this.onMouseDown.bind(this))
    this.domElement.removeEventListener('mousemove', this.onMouseMove.bind(this))
    this.domElement.removeEventListener('mouseup', this.onMouseUp.bind(this))
    this.domElement.removeEventListener('wheel', this.onMouseWheel.bind(this))
  }
}

function RobotVisualizer({ jointAngles, animatedJointAngles, trajectoryPoints = [], currentFrameIndex = 0, startWaypoint, goalWaypoint, isAnimating, mode, previewWaypoint = null }) {
  const containerRef = useRef(null)
  const sceneRef = useRef(null)
  const cameraRef = useRef(null)
  const jointGroupsRef = useRef([])
  const linksRef = useRef([])
  const thetaGroupsRef = useRef([])
  const baseAxesRef = useRef(null)
  const eeAxesRef = useRef(null)
  const eeFrameRef = useRef(null)
  const eeArrowsRef = useRef({ x: null, y: null, z: null })
  const controlsRef = useRef(null)
  const trajectoryLineRef = useRef(null)
  const trajectoryPositionsRef = useRef([])
  const prevIsAnimatingRef = useRef(false)

  // Target cartesian frame visualization (for cartesian waypoint preview)
  const targetFrameRef = useRef(null)
  const targetFrameArrowsRef = useRef({ x: null, y: null, z: null })
  const [previewCartesianAngles, setPreviewCartesianAngles] = useState(null)

  // Waypoint position markers
  const startWaypointMarkerRef = useRef(null)
  const goalWaypointMarkerRef = useRef(null)

  // Ghost robot refs
  const ghostStartRef = useRef(null)
  const ghostEndRef = useRef(null)
  const ghostStartThetaGroupsRef = useRef([])
  const ghostEndThetaGroupsRef = useRef([])

  const [tMatrix, setTMatrix] = useState(null)
  const [ghostStartAngles, setGhostStartAngles] = useState([0, 0, 0, 0, 0, 0])
  const [ghostEndAngles, setGhostEndAngles] = useState([0, 0, 0, 0, 0, 0])

  // DH Parameters for KUKA KR-10 (from backend FK, mm to m conversion)
  // alpha in degrees, a in mm, d in mm, inv_mult: inverse joint multiplier, theta_offset
  const DH_PARAMS = [
    { alpha_deg: 0, a_mm: 0, d_mm: 400, inv_mult: -1, theta_offset: 0 },       // Joint 1
    { alpha_deg: -90, a_mm: 25, d_mm: 0, inv_mult: 1, theta_offset: 0 },       // Joint 2
    { alpha_deg: 0, a_mm: 560, d_mm: 0, inv_mult: 1, theta_offset: -90 },      // Joint 3
    { alpha_deg: -90, a_mm: 25, d_mm: 515, inv_mult: -1, theta_offset: 0 },    // Joint 4
    { alpha_deg: 90, a_mm: 0, d_mm: 0, inv_mult: 1, theta_offset: 0 },         // Joint 5
    { alpha_deg: -90, a_mm: 0, d_mm: 90, inv_mult: -1, theta_offset: 180 }     // Joint 6
  ]

  // Helper function to get joint angles from waypoint
  const getWaypointAngles = (waypoint) => {
    if (!waypoint) return [0, 0, 0, 0, 0, 0]
    if (waypoint.inputType === 'joint') {
      return waypoint.q || [0, 0, 0, 0, 0, 0]
    }
    // For Cartesian mode, use the IK-computed angles
    return waypoint.q || [0, 0, 0, 0, 0, 0]
  }

  // Helper function to compute EE position from joint angles
  const computeEEPositionFromAngles = (angles) => {
    if (!angles || angles.length !== 6) return new THREE.Vector3(0, 0, 0)

    // DH matrix computation
    const dhMatrix = (a, alpha, d, theta) => {
      const alpha_rad = (alpha * Math.PI) / 180
      const theta_rad = (theta * Math.PI) / 180
      const cos_t = Math.cos(theta_rad)
      const sin_t = Math.sin(theta_rad)
      const cos_a = Math.cos(alpha_rad)
      const sin_a = Math.sin(alpha_rad)
      return [
        [cos_t, -sin_t, 0, a],
        [sin_t * cos_a, cos_t * cos_a, -sin_a, -d * sin_a],
        [sin_t * sin_a, cos_t * sin_a, cos_a, d * cos_a],
        [0, 0, 0, 1]
      ]
    }

    const matmul = (A, B) => {
      const result = [[0,0,0,0], [0,0,0,0], [0,0,0,0], [0,0,0,0]]
      for (let i = 0; i < 4; i++) {
        for (let j = 0; j < 4; j++) {
          for (let k = 0; k < 4; k++) {
            result[i][j] += A[i][k] * B[k][j]
          }
        }
      }
      return result
    }

    let T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

    for (let i = 0; i < 6; i++) {
      const dh = DH_PARAMS[i]
      const thetaWithOffset = angles[i] * dh.inv_mult + dh.theta_offset
      const T_i = dhMatrix(dh.a_mm / 1000, dh.alpha_deg, dh.d_mm / 1000, thetaWithOffset)
      T = matmul(T, T_i)
    }

    return new THREE.Vector3(T[0][3], T[1][3], T[2][3])
  }

  // Create ghost robots when scene is ready
  const createGhostRobots = (scene) => {
    const createGhostKinematicChain = (name, opacity) => {
      const group = new THREE.Group()
      const thetaGroups = []

      const baseGroup = new THREE.Group()
      group.add(baseGroup)

      for (let i = 0; i < 6; i++) {
        const alphaGroup = new THREE.Group()
        baseGroup.add(alphaGroup)

        const thetaGroup = new THREE.Group()
        alphaGroup.add(thetaGroup)
        thetaGroups.push(thetaGroup)
        baseGroup = thetaGroup

        const dh = DH_PARAMS[i]
        const a_m = dh.a_mm / 1000
        const d_m = dh.d_mm / 1000

        const alpha_rad = dh.alpha_deg * Math.PI / 180
        if (alpha_rad !== 0) {
          alphaGroup.rotation.x = alpha_rad
        }

        thetaGroup.position.set(a_m, 0, d_m)

        // Create ghost link (semi-transparent cylinders)
        const linkLength = Math.sqrt(a_m * a_m + d_m * d_m) || 0.1
        if (linkLength > 0.01) {
          const linkGeom = new THREE.CylinderGeometry(0.003, 0.003, linkLength, 8)
          const linkMat = new THREE.MeshBasicMaterial({
            color: name === 'start' ? 0x00ff00 : 0xff0000,
            transparent: true,
            opacity: opacity,
            wireframe: false
          })
          const link = new THREE.Mesh(linkGeom, linkMat)
          link.position.z = linkLength / 2
          thetaGroup.add(link)
        }
      }

      scene.add(group)
      return { group, thetaGroups }
    }

    const start = createGhostKinematicChain('start', 0.3)
    const end = createGhostKinematicChain('end', 0.3)

    ghostStartRef.current = start.group
    ghostEndRef.current = end.group
    ghostStartThetaGroupsRef.current = start.thetaGroups
    ghostEndThetaGroupsRef.current = end.thetaGroups
  }

  useEffect(() => {
    // Convert and update ghost angles when waypoints change
    if (startWaypoint) {
      setGhostStartAngles(getWaypointAngles(startWaypoint))
    }
    if (goalWaypoint) {
      setGhostEndAngles(getWaypointAngles(goalWaypoint))
    }
  }, [startWaypoint, goalWaypoint])

  // Update ghost robot visibility and position based on animation state
  useEffect(() => {
    if (!ghostStartRef.current || !ghostEndRef.current) return

    const show = mode === 'motion' && !isAnimating
    ghostStartRef.current.visible = show
    ghostEndRef.current.visible = show
  }, [isAnimating, mode])

  // Update ghost robot angles
  useEffect(() => {
    // Update start ghost
    for (let i = 0; i < ghostStartThetaGroupsRef.current.length && i < 6; i++) {
      const thetaGroup = ghostStartThetaGroupsRef.current[i]
      const dh = DH_PARAMS[i]
      const thetaWithOffset = ghostStartAngles[i] * dh.inv_mult + dh.theta_offset
      const angleRad = thetaWithOffset * Math.PI / 180
      thetaGroup.rotation.z = angleRad
    }

    // Update end ghost
    for (let i = 0; i < ghostEndThetaGroupsRef.current.length && i < 6; i++) {
      const thetaGroup = ghostEndThetaGroupsRef.current[i]
      const dh = DH_PARAMS[i]
      const thetaWithOffset = ghostEndAngles[i] * dh.inv_mult + dh.theta_offset
      const angleRad = thetaWithOffset * Math.PI / 180
      thetaGroup.rotation.z = angleRad
    }
  }, [ghostStartAngles, ghostEndAngles])

  useEffect(() => {
    if (!containerRef.current) return

    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0x1a1a2e)
    sceneRef.current = scene

    const camera = new THREE.PerspectiveCamera(
      75,
      containerRef.current.clientWidth / containerRef.current.clientHeight,
      0.1,
      1000
    )
    camera.position.set(2, 2, 1.5)
    camera.lookAt(0, 0, 0.5)
    camera.up.set(0, 0, 1)  // Z-axis is up

    const renderer = new THREE.WebGLRenderer({ antialias: true })
    renderer.setSize(containerRef.current.clientWidth, containerRef.current.clientHeight)
    renderer.shadowMap.enabled = true
    renderer.domElement.style.cursor = 'grab'
    containerRef.current.appendChild(renderer.domElement)

    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6)
    scene.add(ambientLight)

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.9)
    directionalLight.position.set(5, 10, 7)
    directionalLight.castShadow = true
    directionalLight.shadow.mapSize.width = 2048
    directionalLight.shadow.mapSize.height = 2048
    scene.add(directionalLight)

    const gridHelper = new THREE.GridHelper(2, 40, 0x444444, 0x222222)
    // Rotate grid 90Â° around X-axis to place it in XY plane (with Z-up)
    gridHelper.rotation.x = Math.PI / 2
    gridHelper.position.z = 0  // Grid at z=0
    scene.add(gridHelper)

    // Create labeled axes at origin
    const axesGroup = new THREE.Group()
    axesGroup.position.set(0, 0, 0)

    // Helper function to create axis label
    const createAxisLabel = (text, x, y, z, color) => {
      const canvas = document.createElement('canvas')
      canvas.width = 64
      canvas.height = 64
      const ctx = canvas.getContext('2d')
      ctx.fillStyle = `#${color.toString(16).padStart(6, '0')}`
      ctx.font = 'bold 48px Arial'
      ctx.textAlign = 'center'
      ctx.textBaseline = 'middle'
      ctx.fillText(text, 32, 32)

      const texture = new THREE.CanvasTexture(canvas)
      const spriteMaterial = new THREE.SpriteMaterial({ map: texture })
      const sprite = new THREE.Sprite(spriteMaterial)
      sprite.scale.set(0.1, 0.1, 0.1)
      sprite.position.set(x, y, z)
      return sprite
    }

    // X axis (red with arrow) - points along +X
    const xArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 0.3, 0xff0000)
    axesGroup.add(xArrow)
    const xLabel = createAxisLabel('X', 0.35, 0, 0, 0xff0000)
    axesGroup.add(xLabel)

    // Y axis (green with arrow) - points along +Y
    const yArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), 0.3, 0x00ff00)
    axesGroup.add(yArrow)
    const yLabel = createAxisLabel('Y', 0, 0.35, 0, 0x00ff00)
    axesGroup.add(yLabel)

    // Z axis (blue with arrow) - points along +Z
    const zArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), 0.3, 0x0000ff)
    axesGroup.add(zArrow)
    const zLabel = createAxisLabel('Z', 0, 0, 0.35, 0x0000ff)
    axesGroup.add(zLabel)

    scene.add(axesGroup)
    baseAxesRef.current = axesGroup

    const controls = new SimpleOrbitControls(camera, renderer.domElement)
    controlsRef.current = controls

    // Create ghost robots - temporarily disabled to debug rendering
    // createGhostRobots(scene)

    // Create hierarchical kinematic chain with lines
    const createKinematicChain = () => {
      const jointGroups = []
      const thetaGroups = []  // Groups that hold theta rotations
      const jointWorldPositions = [] // Track world positions of each joint
      const lines = []

      // Base frame at origin
      const baseGroup = new THREE.Group()
      baseGroup.position.set(0, 0, 0) // Base at origin - DH parameters handle offsets
      scene.add(baseGroup)
      jointGroups.push(baseGroup)
      jointWorldPositions.push(baseGroup.getWorldPosition(new THREE.Vector3()))

      // Create each joint and link
      for (let i = 0; i < 6; i++) {
        // Create alpha group (for DH alpha rotation, applied once)
        const alphaGroup = new THREE.Group()
        jointGroups[i].add(alphaGroup)

        // Create theta group (for joint rotation, updated each frame)
        const thetaGroup = new THREE.Group()
        alphaGroup.add(thetaGroup)
        thetaGroups.push(thetaGroup)
        jointGroups.push(thetaGroup)  // For hierarchical chain

        // Add link between this joint and next
        const dh = DH_PARAMS[i]
        const a_m = dh.a_mm / 1000  // Convert mm to meters
        const d_m = dh.d_mm / 1000

        // Apply DH alpha rotation (rotation around DH X axis, which is THREE.js X in Z-up) to alpha group
        const alpha_rad = dh.alpha_deg * Math.PI / 180
        if (alpha_rad !== 0) {
          alphaGroup.rotation.x = alpha_rad  // DH X rotation = THREE.js X rotation (Z-up convention)
        }

        // Set DH offsets on theta group (Z-up coordinates):
        // d_m is along DH Z-axis (THREE.js Z)
        // a_m is along DH X-axis (THREE.js X)
        thetaGroup.position.set(a_m, 0, d_m)

        // Get world position of this joint
        const worldPos = thetaGroup.getWorldPosition(new THREE.Vector3())
        jointWorldPositions.push(worldPos)
      }

      // Create line objects (will be updated each frame)
      const colors = [0xff8800, 0xff7000, 0xff5800, 0x4488ff, 0x44aaff, 0xcccccc]
      for (let i = 0; i < jointGroups.length; i++) {
        const lineGeom = new THREE.BufferGeometry()
        const lineMat = new THREE.LineBasicMaterial({ color: colors[i] || 0xcccccc, linewidth: 2 })
        const line = new THREE.Line(lineGeom, lineMat)
        scene.add(line)
        lines.push(line)
      }

      return { jointGroups, thetaGroups, lines }
    }

    const { jointGroups, thetaGroups, lines } = createKinematicChain()
    jointGroupsRef.current = jointGroups
    linksRef.current = lines
    thetaGroupsRef.current = thetaGroups

    // Helper function to create joint label
    const createJointLabel = (text, color) => {
      const canvas = document.createElement('canvas')
      canvas.width = 64
      canvas.height = 64
      const ctx = canvas.getContext('2d')
      ctx.fillStyle = `#${color.toString(16).padStart(6, '0')}`
      ctx.font = 'bold 40px Arial'
      ctx.textAlign = 'center'
      ctx.textBaseline = 'middle'
      ctx.fillText(text, 32, 32)
      const texture = new THREE.CanvasTexture(canvas)
      const spriteMaterial = new THREE.SpriteMaterial({ map: texture })
      const sprite = new THREE.Sprite(spriteMaterial)
      sprite.scale.set(0.08, 0.08, 0.08)
      sprite.position.set(0.12, 0.12, 0)
      return sprite
    }

    // Add joint spheres and labels to visualization
    const jointColors = [0xff6b6b, 0xffa94d, 0xffd43b, 0x69db7c, 0x4dabf7, 0xb197fc]
    jointColors.forEach((color, i) => {
      const jointIndex = i + 1  // Skip base group at index 0
      if (jointIndex < jointGroupsRef.current.length) {
        // Create small sphere for joint
        const sphereGeom = new THREE.SphereGeometry(0.01, 12, 12)
        const sphereMat = new THREE.MeshBasicMaterial({ color: color })
        const sphere = new THREE.Mesh(sphereGeom, sphereMat)

        jointGroupsRef.current[jointIndex].add(sphere)

        // Create label for joint
        const label = createJointLabel(`J${i + 1}`, color)
        // Position label slightly offset from joint origin for visibility
        label.position.set(0.1, 0.1, 0.1)

        jointGroupsRef.current[jointIndex].add(label)
      }
    })

    // J6 axes with labels (X', Y', Z')
    const eeFrameGroup = new THREE.Group()
    // Add directly to scene (world space) - not as child of J6
    // This prevents double rotation conflicts
    scene.add(eeFrameGroup)

    eeFrameRef.current = eeFrameGroup

    // Create labeled frame
    const createEEFrameLabel = (text, x, y, z, color) => {
      const canvas = document.createElement('canvas')
      canvas.width = 64
      canvas.height = 64
      const ctx = canvas.getContext('2d')
      ctx.fillStyle = `#${color.toString(16).padStart(6, '0')}`
      ctx.font = 'bold 40px Arial'
      ctx.textAlign = 'center'
      ctx.textBaseline = 'middle'
      ctx.fillText(text, 32, 32)
      const texture = new THREE.CanvasTexture(canvas)
      const spriteMaterial = new THREE.SpriteMaterial({ map: texture })
      const sprite = new THREE.Sprite(spriteMaterial)
      sprite.scale.set(0.12, 0.12, 0.12)
      sprite.position.set(x, y, z)
      return sprite
    }

    // X' axis (red)
    const redLineGeom = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0.2, 0, 0)
    ])
    const lineMat = new THREE.LineBasicMaterial({ color: 0xff0000, linewidth: 3 })
    const eeXArrow = new THREE.Line(redLineGeom, lineMat)
    eeFrameGroup.add(eeXArrow)
    eeArrowsRef.current.x = eeXArrow
    const eeXLabel = createEEFrameLabel("X", 0.25, 0, 0, 0xff0000)
    eeFrameGroup.add(eeXLabel)

    // Y' axis (green)
    const greenLineGeom = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, 0.2, 0)
    ])
    const eeYArrow = new THREE.Line(greenLineGeom, new THREE.LineBasicMaterial({ color: 0x00ff00, linewidth: 3 }))
    eeFrameGroup.add(eeYArrow)
    eeArrowsRef.current.y = eeYArrow
    const eeYLabel = createEEFrameLabel("Y", 0, 0.25, 0, 0x00ff00)
    eeFrameGroup.add(eeYLabel)

    // Z' axis (blue)
    const blueLineGeom = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, 0, 0.2)
    ])
    const eeZArrow = new THREE.Line(blueLineGeom, new THREE.LineBasicMaterial({ color: 0x0000ff, linewidth: 3 }))
    eeFrameGroup.add(eeZArrow)
    eeArrowsRef.current.z = eeZArrow
    const eeZLabel = createEEFrameLabel("Z", 0, 0, 0.25, 0x0000ff)
    eeFrameGroup.add(eeZLabel)

    // Add a small yellow sphere at the end effector origin for visibility
    const eeSphereGeom = new THREE.SphereGeometry(0.015, 16, 16)
    const eeSphereMat = new THREE.MeshBasicMaterial({ color: 0xffff00 })
    const eeSphere = new THREE.Mesh(eeSphereGeom, eeSphereMat)
    eeFrameGroup.add(eeSphere)

    // Create target cartesian frame (for cartesian waypoint preview)
    const targetFrameGroup = new THREE.Group()
    scene.add(targetFrameGroup)
    targetFrameRef.current = targetFrameGroup

    // Create target X axis (red dashed)
    const targetXArrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 0, 0),
      0.08,
      0xff0000,
      0.025,
      0.015
    )
    targetXArrow.line.material.setValues({ transparent: true, opacity: 0.5, linewidth: 2 })
    targetXArrow.cone.material.setValues({ transparent: true, opacity: 0.5 })
    targetFrameGroup.add(targetXArrow)
    targetFrameArrowsRef.current.x = targetXArrow

    // Create target Y axis (green dashed)
    const targetYArrow = new THREE.ArrowHelper(
      new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(0, 0, 0),
      0.08,
      0x00ff00,
      0.025,
      0.015
    )
    targetYArrow.line.material.setValues({ transparent: true, opacity: 0.5, linewidth: 2 })
    targetYArrow.cone.material.setValues({ transparent: true, opacity: 0.5 })
    targetFrameGroup.add(targetYArrow)
    targetFrameArrowsRef.current.y = targetYArrow

    // Create target Z axis (blue dashed)
    const targetZArrow = new THREE.ArrowHelper(
      new THREE.Vector3(0, 0, 1),
      new THREE.Vector3(0, 0, 0),
      0.08,
      0x0000ff,
      0.025,
      0.015
    )
    targetZArrow.line.material.setValues({ transparent: true, opacity: 0.5, linewidth: 2 })
    targetZArrow.cone.material.setValues({ transparent: true, opacity: 0.5 })
    targetFrameGroup.add(targetZArrow)
    targetFrameArrowsRef.current.z = targetZArrow

    // Add a small cyan sphere at the target position
    const targetSphereGeom = new THREE.SphereGeometry(0.015, 16, 16)
    const targetSphereMat = new THREE.MeshBasicMaterial({ color: 0x00ffff, transparent: true, opacity: 0.6 })
    const targetSphere = new THREE.Mesh(targetSphereGeom, targetSphereMat)
    targetFrameGroup.add(targetSphere)

    // Create waypoint position markers
    // Start waypoint marker (green sphere)
    const startMarkerGeom = new THREE.SphereGeometry(0.02, 16, 16)
    const startMarkerMat = new THREE.MeshBasicMaterial({ color: 0x00ff00, transparent: true, opacity: 0.8 })
    const startMarker = new THREE.Mesh(startMarkerGeom, startMarkerMat)
    scene.add(startMarker)
    startWaypointMarkerRef.current = startMarker

    // Goal waypoint marker (red sphere)
    const goalMarkerGeom = new THREE.SphereGeometry(0.02, 16, 16)
    const goalMarkerMat = new THREE.MeshBasicMaterial({ color: 0xff0000, transparent: true, opacity: 0.8 })
    const goalMarker = new THREE.Mesh(goalMarkerGeom, goalMarkerMat)
    scene.add(goalMarker)
    goalWaypointMarkerRef.current = goalMarker

    // Store camera ref for view changes
    cameraRef.current = camera

    // Create trajectory line for animation
    const trajectoryGeometry = new THREE.BufferGeometry()
    const trajectoryMaterial = new THREE.LineBasicMaterial({
      color: 0xff0000,
      linewidth: 3,
      transparent: true,
      opacity: 0.8
    })
    const trajectoryLine = new THREE.Line(trajectoryGeometry, trajectoryMaterial)
    scene.add(trajectoryLine)
    trajectoryLineRef.current = trajectoryLine

    const animate = () => {
      requestAnimationFrame(animate)

      // Update lines to show current joint positions
      if (linksRef.current.length > 0) {
        const positions = []
        for (let i = 0; i < jointGroupsRef.current.length; i++) {
          const worldPos = jointGroupsRef.current[i].getWorldPosition(new THREE.Vector3())
          positions.push(worldPos.x, worldPos.y, worldPos.z)
        }

        for (let i = 0; i < linksRef.current.length; i++) {
          const line = linksRef.current[i]
          if (line.geometry && positions.length > (i + 1) * 3) {
            const start = new THREE.Vector3(
              positions[i * 3],
              positions[i * 3 + 1],
              positions[i * 3 + 2]
            )
            const end = new THREE.Vector3(
              positions[(i + 1) * 3],
              positions[(i + 1) * 3 + 1],
              positions[(i + 1) * 3 + 2]
            )
            line.geometry.setFromPoints([start, end])
          }
        }
      }

      renderer.render(scene, camera)
    }
    animate()

    const handleResize = () => {
      if (!containerRef.current) return
      const width = containerRef.current.clientWidth
      const height = containerRef.current.clientHeight
      camera.aspect = width / height
      camera.updateProjectionMatrix()
      renderer.setSize(width, height)
    }
    window.addEventListener('resize', handleResize)
    renderer.domElement.addEventListener('contextmenu', (e) => e.preventDefault())

    return () => {
      window.removeEventListener('resize', handleResize)
      renderer.domElement.removeEventListener('contextmenu', (e) => e.preventDefault())
      if (controlsRef.current) controlsRef.current.dispose()
      renderer.dispose()
      if (containerRef.current && renderer.domElement.parentNode === containerRef.current) {
        containerRef.current.removeChild(renderer.domElement)
      }
    }
  }, [])

  // Update joint angles with inverse multipliers
  useEffect(() => {
    // Prioritize cartesian preview angles, then joint preview waypoint, then normal angles
    let angles
    if (previewCartesianAngles) {
      angles = previewCartesianAngles
    } else if (previewWaypoint && previewWaypoint.inputType === 'joint') {
      angles = previewWaypoint.q || [0, 0, 0, 0, 0, 0]
    } else {
      angles = animatedJointAngles || jointAngles || [0, 0, 0, 0, 0, 0]
    }

    // Apply rotations to each joint with inverse multipliers
    // All joints rotate about their local Z axis (after alpha transformation)
    for (let i = 0; i < thetaGroupsRef.current.length && i < 6; i++) {
      const thetaGroup = thetaGroupsRef.current[i]
      const dh = DH_PARAMS[i]

      // All joints rotate about their local DH Z axis = THREE.js Z axis (now in Z-up coords)
      // Apply inverse multiplier and theta offset
      const thetaWithOffset = angles[i] * dh.inv_mult + dh.theta_offset
      const angleRad = thetaWithOffset * Math.PI / 180
      thetaGroup.rotation.z = angleRad
    }
  }, [jointAngles, animatedJointAngles, previewWaypoint, previewCartesianAngles])

  // Update end effector frame position and rotation using FK from DH parameters
  useEffect(() => {
    if (!eeFrameRef.current) return

    // Prioritize cartesian preview angles, then joint preview waypoint, then normal angles
    let angles
    if (previewCartesianAngles) {
      angles = previewCartesianAngles
    } else if (previewWaypoint && previewWaypoint.inputType === 'joint') {
      angles = previewWaypoint.q || [0, 0, 0, 0, 0, 0]
    } else {
      angles = animatedJointAngles || jointAngles || [0, 0, 0, 0, 0, 0]
    }

    // Helper: Create DH transformation matrix
    const dhMatrix = (a, alpha, d, theta) => {
      const alpha_rad = (alpha * Math.PI) / 180
      const theta_rad = (theta * Math.PI) / 180
      const cos_t = Math.cos(theta_rad)
      const sin_t = Math.sin(theta_rad)
      const cos_a = Math.cos(alpha_rad)
      const sin_a = Math.sin(alpha_rad)

      return [
        [cos_t, -sin_t, 0, a],
        [sin_t * cos_a, cos_t * cos_a, -sin_a, -d * sin_a],
        [sin_t * sin_a, cos_t * sin_a, cos_a, d * cos_a],
        [0, 0, 0, 1]
      ]
    }

    // Helper: Matrix multiplication
    const matmul = (A, B) => {
      const result = [[0,0,0,0], [0,0,0,0], [0,0,0,0], [0,0,0,0]]
      for (let i = 0; i < 4; i++) {
        for (let j = 0; j < 4; j++) {
          for (let k = 0; k < 4; k++) {
            result[i][j] += A[i][k] * B[k][j]
          }
        }
      }
      return result
    }

    // Compute full T matrix
    let T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

    for (let i = 0; i < 6; i++) {
      const dh = DH_PARAMS[i]
      const thetaWithOffset = angles[i] * dh.inv_mult + dh.theta_offset
      const T_i = dhMatrix(dh.a_mm / 1000, dh.alpha_deg, dh.d_mm / 1000, thetaWithOffset)
      T = matmul(T, T_i)
    }

    // Extract position
    const pos = new THREE.Vector3(T[0][3], T[1][3], T[2][3])

    // Extract rotation matrix and convert to quaternion
    const rotMatrix = new THREE.Matrix3().setFromMatrix4(
      new THREE.Matrix4().set(
        T[0][0], T[0][1], T[0][2], T[0][3],
        T[1][0], T[1][1], T[1][2], T[1][3],
        T[2][0], T[2][1], T[2][2], T[2][3],
        0, 0, 0, 1
      )
    )

    const quat = new THREE.Quaternion().setFromRotationMatrix(
      new THREE.Matrix4().setFromMatrix3(rotMatrix)
    )

    // Update EE frame
    eeFrameRef.current.position.copy(pos)
    eeFrameRef.current.quaternion.copy(quat)

    // Store T matrix for display
    setTMatrix(T)
  }, [jointAngles, animatedJointAngles, previewWaypoint, previewCartesianAngles])

  // Handle cartesian waypoint preview (show target frame and compute IK)
  useEffect(() => {
    if (!previewWaypoint || previewWaypoint.inputType !== 'pose' || !targetFrameRef.current) {
      // Hide target frame if no cartesian preview
      if (targetFrameRef.current) {
        targetFrameRef.current.visible = false
      }
      return
    }

    // Show target frame
    targetFrameRef.current.visible = true

    // Update target frame position and orientation from transformation matrix
    const T = previewWaypoint.T
    if (T && T.length === 4 && T[0].length === 4) {
      // Extract position from T matrix (4th column, first 3 rows) - convert from mm to m if needed
      const pos = new THREE.Vector3(
        T[0][3] / 1000,
        T[1][3] / 1000,
        T[2][3] / 1000
      )
      targetFrameRef.current.position.copy(pos)

      // Extract rotation matrix from T and convert to quaternion
      const rotMatrix = new THREE.Matrix3().setFromMatrix4(
        new THREE.Matrix4().set(
          T[0][0], T[0][1], T[0][2], T[0][3],
          T[1][0], T[1][1], T[1][2], T[1][3],
          T[2][0], T[2][1], T[2][2], T[2][3],
          0, 0, 0, 1
        )
      )

      const quat = new THREE.Quaternion().setFromRotationMatrix(
        new THREE.Matrix4().setFromMatrix3(rotMatrix)
      )

      targetFrameRef.current.quaternion.copy(quat)
    }

    // If previewWaypoint has computed IK angles, use them for robot preview
    if (previewWaypoint.q && previewWaypoint.q.length === 6) {
      setPreviewCartesianAngles(previewWaypoint.q)
    }
  }, [previewWaypoint])

  // Update waypoint markers (start and goal positions)
  useEffect(() => {
    // Helper: Create DH transformation matrix
    const dhMatrix = (a, alpha, d, theta) => {
      const alpha_rad = (alpha * Math.PI) / 180
      const theta_rad = (theta * Math.PI) / 180
      const cos_t = Math.cos(theta_rad)
      const sin_t = Math.sin(theta_rad)
      const cos_a = Math.cos(alpha_rad)
      const sin_a = Math.sin(alpha_rad)

      return [
        [cos_t, -sin_t, 0, a],
        [sin_t * cos_a, cos_t * cos_a, -sin_a, -d * sin_a],
        [sin_t * sin_a, cos_t * sin_a, cos_a, d * cos_a],
        [0, 0, 0, 1]
      ]
    }

    // Helper: Matrix multiplication
    const matmul = (A, B) => {
      const result = [[0,0,0,0], [0,0,0,0], [0,0,0,0], [0,0,0,0]]
      for (let i = 0; i < 4; i++) {
        for (let j = 0; j < 4; j++) {
          for (let k = 0; k < 4; k++) {
            result[i][j] += A[i][k] * B[k][j]
          }
        }
      }
      return result
    }

    // Helper: Compute EE position from angles using FK
    const computeEEPosition = (angles) => {
      let T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

      for (let i = 0; i < 6; i++) {
        const dh = DH_PARAMS[i]
        const thetaWithOffset = (angles[i] || 0) * dh.inv_mult + dh.theta_offset
        const T_i = dhMatrix(dh.a_mm / 1000, dh.alpha_deg, dh.d_mm / 1000, thetaWithOffset)
        T = matmul(T, T_i)
      }

      return new THREE.Vector3(T[0][3], T[1][3], T[2][3])
    }

    // Update start waypoint marker
    if (startWaypointMarkerRef.current && startWaypoint) {
      let pos
      if (startWaypoint.inputType === 'joint' && startWaypoint.q) {
        pos = computeEEPosition(startWaypoint.q)
      } else if (startWaypoint.inputType === 'pose' && startWaypoint.T) {
        // Extract position from transformation matrix (4th column, first 3 rows)
        const T = startWaypoint.T
        pos = new THREE.Vector3(T[0][3] / 1000, T[1][3] / 1000, T[2][3] / 1000)
      }
      if (pos) {
        startWaypointMarkerRef.current.position.copy(pos)
        startWaypointMarkerRef.current.visible = true
      }
    }

    // Update goal waypoint marker
    if (goalWaypointMarkerRef.current && goalWaypoint) {
      let pos
      if (goalWaypoint.inputType === 'joint' && goalWaypoint.q) {
        pos = computeEEPosition(goalWaypoint.q)
      } else if (goalWaypoint.inputType === 'pose' && goalWaypoint.T) {
        // Extract position from transformation matrix (4th column, first 3 rows)
        const T = goalWaypoint.T
        pos = new THREE.Vector3(T[0][3] / 1000, T[1][3] / 1000, T[2][3] / 1000)
      }
      if (pos) {
        goalWaypointMarkerRef.current.position.copy(pos)
        goalWaypointMarkerRef.current.visible = true
      }
    }
  }, [startWaypoint, goalWaypoint])

  // Update trajectory line based on trajectory points
  useEffect(() => {
    if (!trajectoryLineRef.current || !trajectoryPoints || trajectoryPoints.length === 0) return

    // Helper functions (same as in EE frame useEffect)
    const dhMatrix = (a, alpha, d, theta) => {
      const alpha_rad = (alpha * Math.PI) / 180
      const theta_rad = (theta * Math.PI) / 180
      const cos_t = Math.cos(theta_rad)
      const sin_t = Math.sin(theta_rad)
      const cos_a = Math.cos(alpha_rad)
      const sin_a = Math.sin(alpha_rad)

      return [
        [cos_t, -sin_t, 0, a],
        [sin_t * cos_a, cos_t * cos_a, -sin_a, -d * sin_a],
        [sin_t * sin_a, cos_t * sin_a, cos_a, d * cos_a],
        [0, 0, 0, 1]
      ]
    }

    const matmul = (A, B) => {
      const result = [[0,0,0,0], [0,0,0,0], [0,0,0,0], [0,0,0,0]]
      for (let i = 0; i < 4; i++) {
        for (let j = 0; j < 4; j++) {
          for (let k = 0; k < 4; k++) {
            result[i][j] += A[i][k] * B[k][j]
          }
        }
      }
      return result
    }

    const computeEEPosition = (angles) => {
      let T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

      for (let i = 0; i < 6; i++) {
        const dh = DH_PARAMS[i]
        const thetaWithOffset = (angles[i] || 0) * dh.inv_mult + dh.theta_offset
        const T_i = dhMatrix(dh.a_mm / 1000, dh.alpha_deg, dh.d_mm / 1000, thetaWithOffset)
        T = matmul(T, T_i)
      }

      return new THREE.Vector3(T[0][3], T[1][3], T[2][3])
    }

    // Compute trajectory positions up to current frame
    const positions = []
    const endFrame = Math.min(currentFrameIndex + 1, trajectoryPoints.length)

    for (let i = 0; i < endFrame; i++) {
      const point = trajectoryPoints[i]
      const angles = point.q || [0, 0, 0, 0, 0, 0]
      const pos = computeEEPosition(angles)
      positions.push(pos)
    }

    // Update trajectory line geometry
    if (positions.length > 0) {
      trajectoryLineRef.current.geometry.setFromPoints(positions)
    }
  }, [trajectoryPoints, currentFrameIndex])

  // Clear trajectory when animation starts
  useEffect(() => {
    if (isAnimating && !prevIsAnimatingRef.current && trajectoryLineRef.current) {
      trajectoryLineRef.current.geometry.setFromPoints([])
    }
    prevIsAnimatingRef.current = isAnimating
  }, [isAnimating])

  return (
    <div
      ref={containerRef}
      style={{
        width: '100%',
        height: '100%',
        position: 'relative',
        overflow: 'hidden'
      }}
    >
      {/* T-Matrix Display */}
      {tMatrix && (
        <div
          style={{
            position: 'absolute',
            top: '20px',
            right: '20px',
            background: 'linear-gradient(135deg, #2a2a2a 0%, #1e1e1e 100%)',
            border: '2px solid rgba(177, 151, 252, 0.3)',
            borderRadius: '12px',
            padding: '16px',
            boxShadow: '0 8px 32px rgba(0, 0, 0, 0.3)',
            zIndex: 1000
          }}
        >
          <div style={{
            color: '#b197fc',
            fontWeight: 'bold',
            marginBottom: '12px',
            fontSize: '14px',
            borderBottom: '2px solid rgba(177, 151, 252, 0.5)',
            paddingBottom: '8px'
          }}>
            End Effector Matrix (meters)
          </div>
          <div style={{
            fontFamily: 'monospace',
            fontSize: '11px',
            color: '#d0b3ff',
            lineHeight: '1.4',
            letterSpacing: '1px'
          }}>
            {[...tMatrix.slice(0, 3), [0, 0, 0, 1]].map((row, i) => (
              <div key={i} style={{ display: 'flex', gap: '4px', justifyContent: 'center' }}>
                <span>{'['}</span>
                {row.map((val, j) => (
                  <span
                    key={j}
                    style={{
                      width: '50px',
                      textAlign: 'right',
                      color: j === 3 ? '#ffd43b' : '#d0b3ff'
                    }}
                  >
                    {val.toFixed(3)}
                  </span>
                ))}
                <span>{']'}</span>
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  )
}

export default RobotVisualizer
