# LinearDrag Motion Function — Design Spec

Date: 2026-07-29

## Motivation

`src/MotionFunctions.luau` currently ships only `MotionFunctions.Default`, reproducing the
original constant-acceleration symplectic Euler kinematics. Projectiles that should feel air
resistance (arrows, thrown objects, anything that visibly slows down over its flight) have no
built-in option and require a caller to hand-write a custom `MotionFunctions` bundle. This spec
adds a second built-in preset, `MotionFunctions.LinearDrag`, following the exact same
`{PositionFunction, VelocityFunction}` bundle shape as `Default`.

## Goals

- Add `MotionFunctions.LinearDrag`: a closed-form, fixed-timestep motion function pair modeling
  linear (Stokes) drag — drag force proportional to velocity — combined with a constant external
  acceleration (e.g. gravity).
- Match the existing `Default` preset's contract exactly: `(projectile, stepCount) -> Vector3`,
  frozen static table, no per-call allocation beyond what the formula needs.
- Source the per-projectile drag coefficient from `projectile.UserData.DragCoefficient`, since
  `MotionFunctions.LinearDrag` is a static bundle (not a factory) and different projectiles need
  different drag strengths.

## Non-goals

- No factory function / parameterized variant (e.g. `MotionFunctions.LinearDrag(coefficient)`).
  Rejected in favor of reading `UserData.DragCoefficient`, keeping the module's exports uniform
  (every entry is a ready-to-use frozen bundle).
- No per-axis or anisotropic drag (single scalar coefficient applies uniformly to all axes).
- No change to `Default`, `Simulation.luau`, `Types.luau`, or `ProjectileSettings.luau` — the
  motion function plumbing they implement already supports arbitrary bundles.

## Design

### Physics

Linear drag with constant external acceleration obeys `dv/dt = a - k·v`, where `a` is
`projectile.Acceleration` and `k` is the drag coefficient. This has an exact closed-form solution
(no per-step numerical integration needed, consistent with `Default`'s approach):

```
t                = stepCount * FIXED_DELTA_TIME
terminalVelocity = a / k
velocityDelta    = v0 - terminalVelocity
decay            = e^(-k * t)

v(t) = terminalVelocity + velocityDelta * decay
p(t) = p0 + terminalVelocity * t + (velocityDelta / k) * (1 - decay)
```

### `src/MotionFunctions.luau` additions

A small helper resolves and validates the coefficient, called independently from each of
`PositionFunction`/`VelocityFunction` (mirroring how `Default` independently recomputes
`stepDeltaTime` in each rather than sharing state across the two functions):

```lua
local function GetDragCoefficient(projectile: Types.Projectile): number
	local userData = projectile.UserData :: any
	local dragCoefficient = userData and userData.DragCoefficient
	assert(
		typeof(dragCoefficient) == "number" and dragCoefficient ~= 0,
		"[SwiftCast] LinearDrag motion function requires projectile.UserData.DragCoefficient to be a non-zero number"
	)
	return dragCoefficient
end

local function LinearDragPositionFunction(projectile: Types.Projectile, stepCount: number): Vector3
	local dragCoefficient = GetDragCoefficient(projectile)
	local stepDeltaTime = stepCount * FIXED_DELTA_TIME
	local terminalVelocity = projectile.Acceleration / dragCoefficient
	local velocityDelta = projectile.Velocity - terminalVelocity
	local decay = math.exp(-dragCoefficient * stepDeltaTime)
	return projectile.Position
		+ terminalVelocity * stepDeltaTime
		+ (velocityDelta / dragCoefficient) * (1 - decay)
end

local function LinearDragVelocityFunction(projectile: Types.Projectile, stepCount: number): Vector3
	local dragCoefficient = GetDragCoefficient(projectile)
	local stepDeltaTime = stepCount * FIXED_DELTA_TIME
	local terminalVelocity = projectile.Acceleration / dragCoefficient
	local velocityDelta = projectile.Velocity - terminalVelocity
	local decay = math.exp(-dragCoefficient * stepDeltaTime)
	return terminalVelocity + velocityDelta * decay
end

MotionFunctions.LinearDrag = table.freeze({
	PositionFunction = LinearDragPositionFunction,
	VelocityFunction = LinearDragVelocityFunction,
}) :: Types.MotionFunctions
```

### Error handling

`GetDragCoefficient` asserts the coefficient is present, numeric, and non-zero. A missing or
zero coefficient is a caller mistake (zero would divide-by-zero into `terminalVelocity`,
silently producing NaN positions) — it should fail loudly via `assert` rather than degrade
silently to no-drag behavior.

## Testing / verification

No automated test suite exists in this repo (Roblox Luau module, `--!strict`-checked only), same
as the existing `Default` preset. Verification is:

- `luau-analyze` passes with no type errors on the modified file.
- Manual smoke check: for a given `(position, velocity, acceleration, dragCoefficient, stepCount)`
  input, confirm `LinearDrag`'s output approaches `Default`'s output as `dragCoefficient`
  approaches 0 (both model the same underlying acceleration when drag is negligible), and that
  velocity asymptotically approaches `terminalVelocity` as `stepCount` grows.
- Confirm spawning a projectile with `MotionFunctions = SwiftCast.MotionFunctions.LinearDrag` and
  no `UserData.DragCoefficient` set raises the assert immediately rather than silently misbehaving.
