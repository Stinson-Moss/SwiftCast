--!strict
--!optimize 2

local ProjectileSettings = {}

local Types = require(script.Parent.Types)
local RaycastFunctions = require(script.Parent.RaycastFunctions)
local MotionFunctions = require(script.Parent.MotionFunctions)

function ProjectileSettings.new(projectileSettings: Types.ProjectileSettingsOptional?): Types.ProjectileSettings
	local projectileSettings: Types.ProjectileSettingsOptional = projectileSettings or {}
	return {
		MaxFlyTime = projectileSettings.MaxFlyTime or math.huge,
		MaxFlyDistance = projectileSettings.MaxFlyDistance or math.huge,
		MaxPiercesPerStep =  projectileSettings.MaxPiercesPerStep or 1,
		RaycastParams = projectileSettings.RaycastParams,
		RaycastFunction = projectileSettings.RaycastFunction or RaycastFunctions.Default,
		MotionFunctions = projectileSettings.MotionFunctions or MotionFunctions.Default,
		CanPierce = projectileSettings.CanPierce or false,
		BlockcastSize = projectileSettings.BlockcastSize,
		SpherecastRadius = projectileSettings.SpherecastRadius,
		ShapecastPart = projectileSettings.ShapecastPart,
	}
end

return table.freeze(ProjectileSettings)