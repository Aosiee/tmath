-- Math: the engine's own math library. Engine layer, its
-- only dependency is glm, so every engine-layer plugin (assetio, gizmo,
-- wisp, debug_draw, the tools) can use it as well as the engine. Namespace
-- TinyEngine::Math. No pch and no config defines: the module compiles
-- byte-identically in every flavor.
--
-- The layout is assetio's: the public headers
-- in include/Math/ (consumers include "Math/X.h"; "Math/Math.h" is the
-- umbrella), the sources in src/, and the tinytest suites in tests/Math/Tests/
-- behind the header-only `math_tests` module, so the suites are reachable
-- by tinytest alone and never sit on a consumer's include path.

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
    sources = glob("src/*.cpp"),
    public_includes = { "include" },
    deps = { public = { "glm/glm" } },
    flags = flags,
    public_link_flags = sanitizers.link_flags,
    cpp_std = "c++20",
}

-- The `math.*` / `math:` doctest suites (header-only; tinytest's main
-- includes "Math/Tests/<Suite>.h").
module "math_tests" {
    sources = {},
    public_includes = { "tests" },
    deps = { public = { "math/math" } },
    cpp_std = "c++20",
}
