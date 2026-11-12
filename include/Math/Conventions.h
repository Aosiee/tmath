#pragma once

// The engine's math conventions, in one place. glm is the
// vector/matrix/quaternion vocabulary; these are the rules every use of it
// follows. What the compiler can check is asserted below.
//
// - Handedness: right-handed (glm's default), cameras look down -Z
//   (glm::lookAt's view space). kRight x kUp == -kForward.
// - Up: +Y.
// - Angles: radians in the math API. Degrees only in serialized and
//   user-facing fields (Transform::Rotation's Euler degrees, the camera's
//   VerticalFovDegrees, the light cone angles); the conversions are Angle.h's.
// - Clip-space depth: the TARGET is Vulkan's [0, 1] (the switch makes
//   GLM_FORCE_DEPTH_ZERO_TO_ONE a public define of the glm plugin and
//   switches the two shader depth linearizations with it). TODAY glm's
//   default [-1, 1] applies; kClipDepthZeroToOne reports which one this
//   build compiles, so nothing has to guess.
// - Colour: stored and computed as linear RGB (Light colour is linear RGB);
//   sRGB only at the edges, where colour enters or leaves the engine. The
//   conversions arrive with Colour.h.
//
// Determinism: the arithmetic-only functions (all of Scalar.h and
// Angle.h - std::fmod is exact) are bit-stable across platforms and
// compilers, given the same evaluation order and no FMA contraction; a
// function that calls a transcendental (sin, cos, atan2, exp, pow...) follows
// the platform libm and is only reproducible on one platform + compiler. The
// math module's own .cpp files compile with -ffp-contract=off
// (plugin.build.luau); the header-inline functions compile with the
// includer's flags (no -march/-mfma anywhere in the tree today, so x86-64
// has no FMA to contract into). RandomStream.h records its own contract.

#include <glm/detail/setup.hpp>
#include <glm/vec3.hpp>

namespace TinyEngine::Math {

#if defined(GLM_FORCE_LEFT_HANDED)
#	error "the engine is right-handed - GLM_FORCE_LEFT_HANDED must not be defined"
#endif
	static_assert((GLM_CONFIG_CLIP_CONTROL & GLM_CLIP_CONTROL_RH_BIT) != 0,
		"the engine is right-handed: glm must build right-handed projections");

	// True once GLM_FORCE_DEPTH_ZERO_TO_ONE is defined; false today.
	inline constexpr bool kClipDepthZeroToOne = (GLM_CONFIG_CLIP_CONTROL & GLM_CLIP_CONTROL_ZO_BIT) != 0;

	// The world basis.
	inline constexpr glm::vec3 kRight(1.0f, 0.0f, 0.0f);
	inline constexpr glm::vec3 kUp(0.0f, 1.0f, 0.0f);
	inline constexpr glm::vec3 kForward(0.0f, 0.0f, -1.0f);

	namespace detail {
		constexpr bool CrossEquals(const glm::vec3& a, const glm::vec3& b, const glm::vec3& expected)
		{
			return a.y * b.z - a.z * b.y == expected.x
				&& a.z * b.x - a.x * b.z == expected.y
				&& a.x * b.y - a.y * b.x == expected.z;
		}
	} // namespace detail

	// Right-handed: X x Y = Z, and the camera's forward is -Z.
	static_assert(detail::CrossEquals(kRight, kUp, -kForward), "right-handed, -Z forward");
	static_assert(kUp.y == 1.0f && kUp.x == 0.0f && kUp.z == 0.0f, "Y up");

} // namespace TinyEngine::Math
