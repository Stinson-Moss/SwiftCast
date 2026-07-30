--!strict
--!optimize 2

local MotionFunctions = {}

local Types = require(script.Parent.Types)
local Config = require(script.Parent.Config)

local FIXED_DELTA_TIME = Config.FixedDeltaTime

-- Closed-form fixed-timestep symplectic Euler integration (velocity updates before position each step)
local function DefaultPositionFunction(projectile: Types.Projectile, stepCount: number): Vector3
	local stepDeltaTime = stepCount * FIXED_DELTA_TIME
	local acceleration = projectile.Acceleration
	return projectile.Position
		+ projectile.Velocity * stepDeltaTime
		+ 0.5 * acceleration * FIXED_DELTA_TIME * FIXED_DELTA_TIME * stepCount * (stepCount + 1)
end

local function DefaultVelocityFunction(projectile: Types.Projectile, stepCount: number): Vector3
	return projectile.Velocity + projectile.Acceleration * (stepCount * FIXED_DELTA_TIME)
end

MotionFunctions.Default = table.freeze({
	PositionFunction = DefaultPositionFunction,
	VelocityFunction = DefaultVelocityFunction,
}) :: Types.MotionFunctions

-- Linear (Stokes) drag with constant external acceleration: dv/dt = a - k*v.
-- Closed-form solution, exact over the full [0, stepCount*FIXED_DELTA_TIME] interval:
--   terminalVelocity = a / k
--   velocityDelta    = v0 - terminalVelocity
--   decay            = e^(-k * t)
--   v(t) = terminalVelocity + velocityDelta * decay
--   p(t) = p0 + terminalVelocity * t + (velocityDelta / k) * (1 - decay)
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

return table.freeze(MotionFunctions)
