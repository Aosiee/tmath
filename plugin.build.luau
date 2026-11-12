-- Math: the engine's own math library. Engine layer, its
-- only dependency is glm, so every engine-layer plugin (assetio, gizmo,
-- wisp, debug_draw, the tools) can use it as well as the engine. Namespace
-- TinyEngine::Math; headers resolve as "Math/X.h". No pch and no config
-- defines: the module compiles byte-identically in every flavor.
--
-- First resident: Math::RandomStream (a seeded, copyable xoshiro256**
-- stream with portable distributions - no process-global state).

plugin "math" {
    enabled_by_default = false,
    public_deps = { "glm" },
}

local sanitizers = require "build/sanitizers"

-- The determinism contract (RandomStream.h): no FMA contraction, so
-- x*x + y*y + z*z rounds the same on a target with FMA (ARM64, -mfma) as
-- on one without. A copy - sanitizers.flags is shared by every module.
local flags = { "-ffp-contract=off" }
for _, flag in ipairs(sanitizers.flags) do
    table.insert(flags, flag)
end

module "math" {
    sources = glob("src/Math/*.cpp"),
    public_includes = { "src" },
    deps = { public = { "glm/glm" } },
    flags = flags,
    public_link_flags = sanitizers.link_flags,
    cpp_std = "c++20",
}
