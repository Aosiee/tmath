#pragma once

// Math/Conventions.h - the conventions as glm actually builds them (the
// compile-time asserts live in the header). It includes the umbrella, so
// the tinytest build also proves Math/Math.h compiles on its own.

#include "Math/Math.h"

#include <doctest/doctest.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>

namespace TinyEngine::Math::conventions_tests {

TEST_CASE("math.conventions: right-handed, Y up, cameras look down -Z")
{
	CHECK(glm::cross(kRight, kUp) == -kForward);
	CHECK(glm::cross(kUp, -kForward) == kRight);
	// A camera at the origin looking along kForward with kUp up has the
	// identity view: glm::lookAt's view space is the convention's.
	const glm::mat4 view = glm::lookAt(glm::vec3(0.0f), kForward, kUp);
	CHECK(view == glm::mat4(1.0f));
	// A point in front of the camera has negative view-space z.
	const glm::vec4 ahead = glm::lookAt(glm::vec3(0.0f, 0.0f, 5.0f), glm::vec3(0.0f), kUp) * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
	CHECK(ahead.z == doctest::Approx(-5.0f));
}

TEST_CASE("math.conventions: the clip depth range glm builds is the one kClipDepthZeroToOne reports")
{
	// [0, 1] since GLM_FORCE_DEPTH_ZERO_TO_ONE became the glm plugin's
	// public define (before the switch this case read [-1, 1]: the plain
	// glm::perspective is the default every caller gets).
	const float n = 0.1f, f = 100.0f;
	const glm::mat4 projection = glm::perspective(DegreesToRadians(60.0f), 16.0f / 9.0f, n, f);
	const glm::vec4 nearClip = projection * glm::vec4(0.0f, 0.0f, -n, 1.0f);
	const glm::vec4 farClip = projection * glm::vec4(0.0f, 0.0f, -f, 1.0f);
	CHECK(kClipDepthZeroToOne);
	CHECK(nearClip.z / nearClip.w == doctest::Approx(0.0f));
	CHECK(farClip.z / farClip.w == doctest::Approx(1.0f));
}

} // namespace TinyEngine::Math::conventions_tests
