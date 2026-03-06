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
  }

  onMouseUp(event) {
    this.isDragging = false
  }

  onMouseWheel(event) {
    event.preventDefault()
    const zoomSpeed = 0.1
    this.radius += event.deltaY > 0 ? zoomSpeed : -zoomSpeed
    this.radius = Math.max(0.5, Math.min(10, this.radius))
  }

  dispose() {
    this.domElement.removeEventListener('mousedown', this.onMouseDown.bind(this))
    this.domElement.removeEventListener('mousemove', this.onMouseMove.bind(this))
    this.domElement.removeEventListener('mouseup', this.onMouseUp.bind(this))
    this.domElement.removeEventListener('wheel', this.onMouseWheel.bind(this))
  }

  updateCamera() {
    const x = this.target.x + this.radius * Math.sin(this.phi) * Math.cos(this.theta)
    const y = this.target.y + this.radius * Math.sin(this.phi) * Math.sin(this.theta)
    const z = this.target.z + this.radius * Math.cos(this.phi)
    this.camera.position.set(x, y, z)
    this.camera.lookAt(this.target)
  }
}

const RobotVisualizer = ({
  jointAngles = [0, 0, 0, 0, 0, 0],
  animatedJointAngles = null,
  tMatrixDisplay = null,
  trajectoryPoints = [],
  isAnimating = false,
  startWaypoint = null,
  goalWaypoint = null,
  mode = 'motion'
}) => {
  const containerRef = useRef(null)
  const sceneRef = useRef(null)
  const cameraRef = useRef(null)
  const controlsRef = useRef(null)
  const jointGroupsRef = useRef([])
  const thetaGroupsRef = useRef([])
  const linksRef = useRef([])
  const eeFrameRef = useRef(null)
  const eeArrowsRef = useRef({ x: null, y: null, z: null })
  const trajectoryLineRef = useRef(null)
  const trajectoryPositionsRef = useRef([])
  const baseAxesRef = useRef(null)
  const tMatrixDisplayRef = useRef(null)

  // Ghost robot refs
  const ghostStartRef = useRef(null)
  const ghostEndRef = useRef(null)
  const ghostStartThetaGroupsRef = useRef([])
  const ghostEndThetaGroupsRef = useRef([])

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
    return [0, 0, 0, 0, 0, 0]
  }

  // Update ghost angles when waypoints change
  useEffect(() => {
    if (startWaypoint) {
      setGhostStartAngles(getWaypointAngles(startWaypoint))
    }
    if (goalWaypoint) {
      setGhostEndAngles(getWaypointAngles(goalWaypoint))
    }
  }, [startWaypoint, goalWaypoint])

  // Update ghost robot visibility
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

  // Main scene setup
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
    camera.up.set(0, 0, 1)

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
    gridHelper.rotation.x = Math.PI / 2
    gridHelper.position.z = 0
    scene.add(gridHelper)

    // World axes
    const axesGroup = new THREE.Group()
    axesGroup.position.set(0, 0, 0)

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

    const xArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 0.3, 0xff0000)
    axesGroup.add(xArrow)
    axesGroup.add(createAxisLabel('X', 0.35, 0, 0, 0xff0000))

    const yArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), 0.3, 0x00ff00)
    axesGroup.add(yArrow)
    axesGroup.add(createAxisLabel('Y', 0, 0.35, 0, 0x00ff00))

    const zArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), 0.3, 0x0000ff)
    axesGroup.add(zArrow)
    axesGroup.add(createAxisLabel('Z', 0, 0, 0.35, 0x0000ff))

    scene.add(axesGroup)
    baseAxesRef.current = axesGroup

    const controls = new SimpleOrbitControls(camera, renderer.domElement)
    controlsRef.current = controls

    // Create ghost robots
    const createGhostRobots = () => {
      const createGhostChain = (color, opacity) => {
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
        }

        scene.add(group)
        return { group, thetaGroups }
      }

      const start = createGhostChain(0x00ff00, 0.3)
      const end = createGhostChain(0xff0000, 0.3)

      ghostStartRef.current = start.group
      ghostEndRef.current = end.group
      ghostStartThetaGroupsRef.current = start.thetaGroups
      ghostEndThetaGroupsRef.current = end.thetaGroups
    }

    // createGhostRobots() // Disabled for now - will implement with proper rendering

    // Create kinematic chain
    const jointGroups = []
    const thetaGroups = []
    const lines = []

    const baseGroup = new THREE.Group()
    baseGroup.position.set(0, 0, 0)
    scene.add(baseGroup)
    jointGroups.push(baseGroup)

    for (let i = 0; i < 6; i++) {
      const alphaGroup = new THREE.Group()
      jointGroups[i].add(alphaGroup)
      const thetaGroup = new THREE.Group()
      alphaGroup.add(thetaGroup)
      thetaGroups.push(thetaGroup)
      jointGroups.push(thetaGroup)

      const dh = DH_PARAMS[i]
      const a_m = dh.a_mm / 1000
      const d_m = dh.d_mm / 1000
      const alpha_rad = dh.alpha_deg * Math.PI / 180
      if (alpha_rad !== 0) {
        alphaGroup.rotation.x = alpha_rad
      }
      thetaGroup.position.set(a_m, 0, d_m)
    }

    const colors = [0xff8800, 0xff7000, 0xff5800, 0x4488ff, 0x44aaff, 0xcccccc]
    for (let i = 0; i < jointGroups.length; i++) {
      const lineGeom = new THREE.BufferGeometry()
      const lineMat = new THREE.LineBasicMaterial({ color: colors[i] || 0xcccccc, linewidth: 2 })
      const line = new THREE.Line(lineGeom, lineMat)
      scene.add(line)
      lines.push(line)
    }

    jointGroupsRef.current = jointGroups
    linksRef.current = lines
    thetaGroupsRef.current = thetaGroups

    // Joint labels
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

    const jointColors = [0xff6b6b, 0xffa94d, 0xffd43b, 0x69db7c, 0x4dabf7, 0xb197fc]
    jointColors.forEach((color, i) => {
      const jointIndex = i + 1
      if (jointIndex < jointGroupsRef.current.length) {
        const sphereGeom = new THREE.SphereGeometry(0.01, 12, 12)
        const sphereMat = new THREE.MeshBasicMaterial({ color: color })
        const sphere = new THREE.Mesh(sphereGeom, sphereMat)
        jointGroupsRef.current[jointIndex].add(sphere)

        const label = createJointLabel(`J${i + 1}`, color)
        label.position.set(0.1, 0.1, 0.1)
        jointGroupsRef.current[jointIndex].add(label)
      }
    })

    // End effector frame
    const eeFrameGroup = new THREE.Group()
    scene.add(eeFrameGroup)
    eeFrameRef.current = eeFrameGroup

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

    const redLineGeom = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0.2, 0, 0)
    ])
    const lineMat = new THREE.LineBasicMaterial({ color: 0xff0000, linewidth: 3 })
    const eeXArrow = new THREE.Line(redLineGeom, lineMat)
    eeFrameGroup.add(eeXArrow)
    eeArrowsRef.current.x = eeXArrow
    eeFrameGroup.add(createEEFrameLabel("X", 0.25, 0, 0, 0xff0000))

    const greenLineGeom = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, 0.2, 0)
    ])
    const eeYArrow = new THREE.Line(greenLineGeom, new THREE.LineBasicMaterial({ color: 0x00ff00, linewidth: 3 }))
    eeFrameGroup.add(eeYArrow)
    eeArrowsRef.current.y = eeYArrow
    eeFrameGroup.add(createEEFrameLabel("Y", 0, 0.25, 0, 0x00ff00))

    const blueLineGeom = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(0, 0, 0.2)
    ])
    const eeZArrow = new THREE.Line(blueLineGeom, new THREE.LineBasicMaterial({ color: 0x0000ff, linewidth: 3 }))
    eeFrameGroup.add(eeZArrow)
    eeArrowsRef.current.z = eeZArrow
    eeFrameGroup.add(createEEFrameLabel("Z", 0, 0, 0.25, 0x0000ff))

    const eeSphereGeom = new THREE.SphereGeometry(0.005, 16, 16)
    const eeSphereMat = new THREE.MeshBasicMaterial({ color: 0xffff00 })
    const eeSphere = new THREE.Mesh(eeSphereGeom, eeSphereMat)
    eeFrameGroup.add(eeSphere)

    cameraRef.current = camera

    // T Matrix display panel
    const tMatrixDiv = document.createElement('div')
    tMatrixDiv.style.position = 'absolute'
    tMatrixDiv.style.top = '10px'
    tMatrixDiv.style.right = '10px'
    tMatrixDiv.style.backgroundColor = 'rgba(0, 0, 0, 0.7)'
    tMatrixDiv.style.color = '#00ff00'
    tMatrixDiv.style.padding = '10px'
    tMatrixDiv.style.fontFamily = 'monospace'
    tMatrixDiv.style.fontSize = '12px'
    tMatrixDiv.style.maxWidth = '300px'
    tMatrixDiv.style.zIndex = '100'
    tMatrixDiv.style.whiteSpace = 'pre'
    tMatrixDiv.style.border = '1px solid #00ff00'
    tMatrixDiv.innerHTML = 'T Matrix\n[0, 0, 0, 0]\n[0, 0, 0, 0]\n[0, 0, 0, 0]\n[0, 0, 0, 1]'
    containerRef.current.appendChild(tMatrixDiv)
    tMatrixDisplayRef.current = tMatrixDiv

    // Trajectory line
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

      // Update end effector frame position
      if (eeFrameRef.current && jointGroupsRef.current.length > 0) {
        const lastJoint = jointGroupsRef.current[jointGroupsRef.current.length - 1]
        const eePos = lastJoint.getWorldPosition(new THREE.Vector3())
        eeFrameRef.current.position.copy(eePos)
      }

      if (controls) controls.updateCamera()
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

  // Update joint angles
  useEffect(() => {
    const angles = animatedJointAngles || jointAngles || [0, 0, 0, 0, 0, 0]

    for (let i = 0; i < thetaGroupsRef.current.length && i < 6; i++) {
      const thetaGroup = thetaGroupsRef.current[i]
      const dh = DH_PARAMS[i]
      const thetaWithOffset = angles[i] * dh.inv_mult + dh.theta_offset
      const angleRad = thetaWithOffset * Math.PI / 180
      thetaGroup.rotation.z = angleRad
    }
  }, [jointAngles, animatedJointAngles])

  // Update end effector frame
  useEffect(() => {
    if (!eeFrameRef.current || !thetaGroupsRef.current.length) return
    if (thetaGroupsRef.current.length > 6) {
      const lastJoint = thetaGroupsRef.current[thetaGroupsRef.current.length - 1]
      const pos = lastJoint.getWorldPosition(new THREE.Vector3())
      eeFrameRef.current.position.copy(pos)
    }
  }, [jointAngles, animatedJointAngles])

  return (
    <div
      ref={containerRef}
      style={{
        width: '100%',
        height: '100%',
        position: 'relative',
        overflow: 'hidden'
      }}
    />
  )
}

export default RobotVisualizer
