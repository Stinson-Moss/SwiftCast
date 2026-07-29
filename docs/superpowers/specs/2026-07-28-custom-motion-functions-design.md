# Custom Motion Functions — Design Spec

Date: 2026-07-28

## Motivation

`Types.luau` already declares `PositionFunction`/`VelocityFunction` types and threads
`constructorArgs.PositionFunction`/`VelocityFunction` through to `Projectile.PositionAtTime`/
`VelocityAtTime` (`src/Simulation.luau:331-332`), but the simulation never actually calls them
correctly. In `UpdateProjectile` (`src/Simulation.luau:194-270`):

```lua
local newPosition = projectile.PositionAtTime(projectile, time)
local newVelocity = projectile.Velocity
local newPosition = position
    + newVelocity * stepDeltaTime
    + 0.5 * acceleration * FIXED_DELTA_TIME * FIXED_DELTA_TIME * stepCount * (stepCount + 1)
newVelocity += acceleration * stepDeltaTime
```

- Line 1 calls `PositionAtTime` with the global `time` **function** (never invoked, and not a
  number), so this would error the moment a custom function tried to use its second argument.
- The result is immediately discarded anyway: `newPosition` is redeclared with `local` two lines
  later, shadowing the call's result. It is dead code.
- `Types.ProjectileSettings` has no `PositionFunction`/`VelocityFunction` fields at all, and
  `ProjectileSettings.new` never sets defaults for them, so `resolvedProjectileSettings.PositionFunction`
  doesn't resolve to anything meaningful either.

This spec finishes the wiring so a caller can supply their own position/velocity integration and
have it actually drive the simulation, while preserving today's behavior as the built-in default.

## Current behavior being preserved

The existing inline formula is a **closed-form, fixed-timestep symplectic (semi-implicit) Euler
integration**: velocity is advanced first each fixed step (`v += a·dt`), then position is advanced
using that updated velocity (`p += v·dt`). Rather than looping `stepCount` times, the sum across
all `stepCount` steps is computed directly:

- `newVelocity = velocity + acceleration * stepDeltaTime`, where `stepDeltaTime = stepCount * FIXED_DELTA_TIME`.
- `newPosition = position + velocity * stepDeltaTime + 0.5 * acceleration * FIXED_DELTA_TIME^2 * stepCount * (stepCount + 1)`,
  the closed-form sum of `Σ v_i · dt` for `i = 1..stepCount` under symplectic Euler (hence the
  triangular-number `stepCount * (stepCount + 1)` term rather than `stepCount^2`).

This exact formula becomes `MotionFunctions.Default` (see below) — behavior is unchanged for
projectiles that don't opt into a custom motion function.

## Goals

- Let a caller supply their own `PositionFunction`/`VelocityFunction` pair per-projectile (via
  `SpawnProjectile`/`CreateProjectile` constructor args) or per-`ProjectileSettings`, the same way
  `RaycastFunction` is already customizable.
- Ship a `Default` bundle reproducing today's constant-acceleration kinematics exactly.
- Keep the plugin contract simple: **closed-form, fixed-timestep Euler**, not a continuous
  analytic function of absolute time. Functions receive the current projectile state
  (`Position`/`Velocity`/`Acceleration`) plus `stepCount` (how many `FIXED_DELTA_TIME`-sized steps
  to advance) and return the resulting absolute position/velocity. No new clock/time-tracking
  field on `Projectile` is needed — this was considered (an absolute-time analytic-curve contract)
  and rejected in favor of this simpler, drift-free, stateless-per-call contract that matches what
  the original formula already was.

## Non-goals

- No built-in example motion functions beyond `Default` (e.g. no sine-wave/homing presets).
- No change to raycasting/collision logic, `OnStep`/`OnPositionChange` event semantics, or the
  `_Accumulator`/`stepCount` cadence logic that decides *when* a step runs.
- No change to `RaycastFunctions.luau` or its pattern beyond using it as a structural reference.

## Design

### 1. Types (`src/Types.luau`)

Change the function signatures from `(projectile, time)` to `(projectile, stepCount)`:

```lua
export type PositionFunction = (projectile: Projectile, stepCount: number) -> Vector3
export type VelocityFunction = (projectile: Projectile, stepCount: number) -> Vector3

export type MotionFunctions = {
    PositionFunction: PositionFunction,
    VelocityFunction: VelocityFunction,
}
```

`Projectile` fields renamed from `PositionAtTime`/`VelocityAtTime` to `PositionFunction`/
`VelocityFunction` (matching the type name, same pattern as the existing `RaycastFunction:
RaycastFunction` field):

```lua
PositionFunction: PositionFunction,
VelocityFunction: VelocityFunction,
```

`ProjectileConstructorArgs`: replace the existing (unused) `PositionFunction: PositionFunction?`
and `VelocityFunction: VelocityFunction?` fields with a single bundled field:

```lua
MotionFunctions: MotionFunctions?,
```

`ProjectileSettings` (resolved): add

```lua
MotionFunctions: MotionFunctions,
```

`ProjectileSettingsOptional`: add

```lua
MotionFunctions: MotionFunctions?,
```

### 2. New module `src/MotionFunctions.luau`

Mirrors the `RaycastFunctions.luau` pattern (frozen table of named presets), except each entry is
a `{PositionFunction, VelocityFunction}` pair instead of a single function, since position and
velocity integration for a given motion style are coupled and must be supplied together.

```lua
--!strict
--!optimize 2

local MotionFunctions = {}

local Types = require(script.Parent.Types)
local Config = require(script.Parent.Config)

local FIXED_DELTA_TIME = Config.FixedDeltaTime

local function DefaultPositionFunction(projectile, stepCount)
    local stepDeltaTime = stepCount * FIXED_DELTA_TIME
    local acceleration = projectile.Acceleration
    return projectile.Position
        + projectile.Velocity * stepDeltaTime
        + 0.5 * acceleration * FIXED_DELTA_TIME * FIXED_DELTA_TIME * stepCount * (stepCount + 1)
end :: Types.PositionFunction

local function DefaultVelocityFunction(projectile, stepCount)
    return projectile.Velocity + projectile.Acceleration * (stepCount * FIXED_DELTA_TIME)
end :: Types.VelocityFunction

MotionFunctions.Default = table.freeze({
    PositionFunction = DefaultPositionFunction,
    VelocityFunction = DefaultVelocityFunction,
}) :: Types.MotionFunctions

return table.freeze(MotionFunctions)
```

A custom bundle is just a table of the same shape, e.g.:

```lua
local MyMotion: SwiftCast.MotionFunctions = {
    PositionFunction = function(projectile, stepCount)
        -- custom closed-form fixed-timestep integration
    end,
    VelocityFunction = function(projectile, stepCount)
        -- custom closed-form fixed-timestep integration
    end,
}

SwiftCast.SpawnProjectile({
    MotionFunctions = MyMotion,
    Position = origin,
    Velocity = initialVelocity,
    ...
})
```

### 3. `src/Simulation.luau` changes

`NewProjectile` resolves the bundle once (constructor args win over settings, same precedence as
every other overridable field) and stores the two functions directly on the projectile:

```lua
local motionFunctions = constructorArgs.MotionFunctions or resolvedProjectileSettings.MotionFunctions
...
PositionFunction = motionFunctions.PositionFunction,
VelocityFunction = motionFunctions.VelocityFunction,
```

`UpdateProjectile` replaces the broken inline block with calls through the resolved functions.
The `acceleration` local is no longer needed here (only the motion functions touch
`projectile.Acceleration` now):

```lua
local stepCount = accumulator // FIXED_DELTA_TIME
local stepDeltaTime = stepCount * FIXED_DELTA_TIME
projectile._Accumulator = accumulator - stepDeltaTime

local position = projectile.Position
local newPosition = projectile.PositionFunction(projectile, stepCount)
local newVelocity = projectile.VelocityFunction(projectile, stepCount)
```

Everything after this point (`deltaDistance`, pierce/raycast loop, `OnStep`/`OnPositionChange`/
`OnHit`/`OnDestroy` events, committing `Position`/`Velocity`) is unchanged.

### 4. `src/ProjectileSettings.luau`

Require the new module and resolve a default the same way every other optional field is resolved:

```lua
local MotionFunctions = require(script.Parent.MotionFunctions)
...
MotionFunctions = projectileSettings.MotionFunctions or MotionFunctions.Default,
```

### 5. `src/init.luau`

Export the new module and types, following the existing `RaycastFunctions`/`RaycastFunction`
pattern:

```lua
local MotionFunctions = require(script.MotionFunctions)
...
export type PositionFunction = Types.PositionFunction
export type VelocityFunction = Types.VelocityFunction
export type MotionFunctions = Types.MotionFunctions
...
SwiftCast.MotionFunctions = MotionFunctions
```

## Testing / verification

- No existing automated test suite in this repo (Roblox Luau module, `--!strict` type-checked
  only). Verification is: `luau-analyze` (or Rojo/Selene if configured) passes with no type
  errors across all touched files, and a manual smoke check that `MotionFunctions.Default`
  produces bit-for-bit the same `newPosition`/`newVelocity` values the old inline formula did for
  a few sample `(position, velocity, acceleration, stepCount)` inputs.
- Confirm a projectile spawned with a custom `MotionFunctions` bundle (e.g. constant velocity, no
  acceleration term) drives `Position` changes without touching the default kinematics path.

## Migration notes

`ProjectileConstructorArgs.PositionFunction`/`VelocityFunction` are removed in favor of
`MotionFunctions`. This is a breaking signature change, but since the old fields never worked
(the bug above), there is no working call site anywhere that relied on them.
