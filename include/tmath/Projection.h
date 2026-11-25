#pragma once

// The projection and view builders, in the Conventions.h
// conventions: right-handed, cameras look down -Z, clip depth [0, 1]
// (Vulkan's). tmath requires
// GLM_FORCE_DEPTH_ZERO_TO_ONE (Conventions.h), so a plain glm::perspective/ortho is the same
// [0, 1] form; these name the RH_ZO forms explicitly, so the convention
// does not depend on a define being in effect at the call site.
//
// Depth, for a perspective projection with near n and far f (view depth z,
// the positive distance along -Z): d = f (z - n) / (z (f - n)), so the near
// plane maps to 0, the far plane to 1, and the inverse is LinearizeDepth.
// The depth buffer stores d directly (minDepth 0, maxDepth 1), so a shader
// or CPU reconstruction reads the stored value as z_ndc - never remaps it
// with * 0.5 + 0.5 (that was the [-1, 1] form).
//
// The reverse-Z builders are for a renderer that switches to reverse Z
// (which also changes its depth compare ops and clears).

#include "tmath/Conventions.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace tmath {

	// A perspective projection: vertical field of view (radians), aspect =
	// width / height. Near -> depth 0, far -> depth 1.
	inline glm::mat4 Perspective(float fovYRadians, float aspect, float nearPlane, float farPlane)
	{
		return glm::perspectiveRH_ZO(fovYRadians, aspect, nearPlane, farPlane);
	}

	// An orthographic projection of the view-space box [left, right] x
	// [bottom, top] x [-near, -far]. Near -> depth 0, far -> depth 1; the
	// depth is linear in z.
	inline glm::mat4 Orthographic(float left, float right, float bottom, float top, float nearPlane, float farPlane)
	{
		return glm::orthoRH_ZO(left, right, bottom, top, nearPlane, farPlane);
	}

	// The view matrix of a camera at eye looking at target (right-handed:
	// the camera looks down its -Z). Independent of the depth convention.
	inline glm::mat4 LookAt(const glm::vec3& eye, const glm::vec3& target, const glm::vec3& up)
	{
		return glm::lookAtRH(eye, target, up);
	}

	// The reverse-Z perspective: near -> depth 1, far -> depth 0 (the depth
	// precision of a float buffer then spreads evenly in log distance). The
	// x/y terms are Perspective's; only the depth row differs.
	inline glm::mat4 PerspectiveReverseZ(float fovYRadians, float aspect, float nearPlane, float farPlane)
	{
		glm::mat4 m = Perspective(fovYRadians, aspect, nearPlane, farPlane);
		m[2][2] = nearPlane / (farPlane - nearPlane);
		m[3][2] = farPlane * nearPlane / (farPlane - nearPlane);
		return m;
	}

	// The reverse-Z perspective with the far plane at infinity: near -> 1,
	// depth -> 0 as the distance grows without bound (d = near / z).
	inline glm::mat4 PerspectiveReverseZInfinite(float fovYRadians, float aspect, float nearPlane)
	{
		glm::mat4 m = Perspective(fovYRadians, aspect, nearPlane, 2.0f * nearPlane);
		m[2][2] = 0.0f;
		m[3][2] = nearPlane;
		return m;
	}

	// The view depth (positive distance along -Z) of a stored depth d from
	// Perspective(.., near, far): the inverse n f / (f - d (f - n)). d = 0
	// gives near, d = 1 gives far. The shaders' ClusterLinearizeDepth and
	// debug_depth.frag's LinearizeDepth are this formula in GLSL.
	constexpr float LinearizeDepth(float depth, float nearPlane, float farPlane)
	{
		return nearPlane * farPlane / (farPlane - depth * (farPlane - nearPlane));
	}

} // namespace tmath
