export function clamp01(value) {
  if (value < 0) return 0
  if (value > 1) return 1
  return value
}

export function cubicTimeScaling(tau) {
  const t = clamp01(tau)
  return 3 * t * t - 2 * t * t * t
}

export function trapezoidTimeScaling(tau, accelFraction = 0.2) {
  const t = clamp01(tau)
  const a = Math.min(Math.max(accelFraction, 0.05), 0.45)
  const t1 = a
  const t2 = 1 - a
  const v = 1 / (1 - a)

  if (t <= t1) {
    return (v / (2 * a)) * t * t
  }

  if (t <= t2) {
    return v * (t - a / 2)
  }

  const dt = 1 - t
  return 1 - (v / (2 * a)) * dt * dt
}

export function timeScaling(tau, profile = 'cubic') {
  if (profile === 'cubic') {
    return cubicTimeScaling(tau)
  }

  return trapezoidTimeScaling(tau)
}
