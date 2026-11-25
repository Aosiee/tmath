
// tmath/Projection.h - the [0, 1] clip depth (near -> 0, far -> 1) for the
// perspective and the ortho, the x/y terms, the reverse-Z builders (near
// -> 1, far -> 0, the infinite far), the depth linearization
// round-tripped against the matrix (the C++ mirror of the shaders' nf /
// (f - d (f - n))), and LookAt.

#include "tmath/Angle.h"
#include "tmath/Projection.h"

#include "hamr/test.h"

#include <cmath>
#include <limits>

#include <glm/gtc/matrix_transform.hpp>

namespace tmath::projection_tests {

namespace {

	// The stored depth (z_ndc) of a point at view depth z (positive, along -Z).
	float DepthAt(const glm::mat4& projection, float viewDepth)
	{
		const glm::vec4 clip = projection * glm::vec4(0.0f, 0.0f, -viewDepth, 1.0f);
		return clip.z / clip.w;
	}

} // namespace

TEST_CASE("projection: Perspective - near maps to 0, far to 1, the x/y terms are the vertical FOV's")
{
	const float n = 0.1f, f = 1000.0f;
	const float fovY = DegreesToRadians(45.0f), aspect = 16.0f / 9.0f;
	const glm::mat4 p = Perspective(fovY, aspect, n, f);

	CHECK(DepthAt(p, n) == hamr::test::Approx(0.0f));
	CHECK(DepthAt(p, f) == hamr::test::Approx(1.0f));
	// Monotonic in between, and inside [0, 1] at the old clipped band (n..2n).
	CHECK(DepthAt(p, 1.5f * n) > 0.0f);
	CHECK(DepthAt(p, 1.5f * n) < DepthAt(p, 2.0f * n));
	CHECK(DepthAt(p, 10.0f) < DepthAt(p, 100.0f));
	// Outside the planes leaves [0, 1] (Vulkan clips there).
	CHECK(DepthAt(p, 0.5f * n) < 0.0f);
	CHECK(DepthAt(p, 2.0f * f) > 1.0f);

	CHECK(p[1][1] == hamr::test::Approx(1.0f / std::tan(fovY * 0.5f)));
	CHECK(p[0][0] == hamr::test::Approx(p[1][1] / aspect));
	CHECK(p[2][3] == -1.0f);

	// The glm plugin's define makes the plain glm call the same matrix.
	CHECK(p == glm::perspective(fovY, aspect, n, f));
}

TEST_CASE("projection: Orthographic - near maps to 0, far to 1, linear in z")
{
	const glm::mat4 o = Orthographic(-4.0f, 4.0f, -2.0f, 2.0f, 1.0f, 9.0f);
	CHECK(DepthAt(o, 1.0f) == hamr::test::Approx(0.0f));
	CHECK(DepthAt(o, 9.0f) == hamr::test::Approx(1.0f));
	CHECK(DepthAt(o, 5.0f) == hamr::test::Approx(0.5f));
	const glm::vec4 corner = o * glm::vec4(4.0f, 2.0f, -5.0f, 1.0f);
	CHECK(corner.x == hamr::test::Approx(1.0f));
	CHECK(corner.y == hamr::test::Approx(1.0f));
	CHECK(corner.w == 1.0f);

	// The 2D form (a UI or overlay projection): z in [-1, 1]
	// maps to depth [1, 0], so z = 0 sits at 0.5 and z > 0 is no longer
	// clipped (glm's [-1, 1] form gave depth -z).
	const glm::mat4 ui = Orthographic(0.0f, 1920.0f, 0.0f, 1080.0f, -1.0f, 1.0f);
	CHECK((ui * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)).z == hamr::test::Approx(0.5f));
	CHECK((ui * glm::vec4(0.0f, 0.0f, 1.0f, 1.0f)).z == hamr::test::Approx(0.0f));
	CHECK((ui * glm::vec4(0.0f, 0.0f, -1.0f, 1.0f)).z == hamr::test::Approx(1.0f));
	CHECK(ui == glm::ortho(0.0f, 1920.0f, 0.0f, 1080.0f, -1.0f, 1.0f));
}

TEST_CASE("projection: the reverse-Z builders - near maps to 1, far to 0, and the infinite far")
{
	const float n = 0.1f, f = 500.0f;
	const float fovY = DegreesToRadians(60.0f), aspect = 1.5f;
	const glm::mat4 forward = Perspective(fovY, aspect, n, f);
	const glm::mat4 reversed = PerspectiveReverseZ(fovY, aspect, n, f);

	CHECK(DepthAt(reversed, n) == hamr::test::Approx(1.0f));
	CHECK(DepthAt(reversed, f) == hamr::test::Approx(0.0f).epsilon(1e-6));
	CHECK(DepthAt(reversed, 1.0f) > DepthAt(reversed, 10.0f));
	// For the same depth, reversed = 1 - forward (exact in the reals).
	for (float z : { 0.2f, 1.0f, 7.5f, 120.0f })
		CHECK(DepthAt(reversed, z) == hamr::test::Approx(1.0f - DepthAt(forward, z)).epsilon(1e-5));
	// The x/y terms are untouched.
	CHECK(reversed[0][0] == forward[0][0]);
	CHECK(reversed[1][1] == forward[1][1]);
	CHECK(reversed[2][3] == -1.0f);

	const glm::mat4 infinite = PerspectiveReverseZInfinite(fovY, aspect, n);
	CHECK(DepthAt(infinite, n) == hamr::test::Approx(1.0f));
	CHECK(DepthAt(infinite, 10.0f) == hamr::test::Approx(n / 10.0f));
	CHECK(DepthAt(infinite, 1.0e30f) >= 0.0f);
	CHECK(DepthAt(infinite, 1.0e30f) < 1e-20f);
	// The limit at infinity: a direction (w = 0) lands exactly on depth 0.
	const glm::vec4 atInfinity = infinite * glm::vec4(0.0f, 0.0f, -1.0f, 0.0f);
	CHECK(atInfinity.z == 0.0f);
	CHECK(atInfinity.w == 1.0f);
	CHECK(infinite[0][0] == forward[0][0]);
	CHECK(infinite[1][1] == forward[1][1]);
}

TEST_CASE("projection: LinearizeDepth inverts Perspective's depth (the shaders' formula)")
{
	static_assert(LinearizeDepth(0.0f, 0.1f, 1000.0f) == 0.1f * 1000.0f / 1000.0f);

	const float cases[][2] = { { 0.1f, 1000.0f }, { 0.01f, 100.0f }, { 1.0f, 50.0f }, { 0.5f, 0.6f } };
	for (const auto& c : cases)
	{
		const float n = c[0], f = c[1];
		const glm::mat4 p = Perspective(DegreesToRadians(70.0f), 1.0f, n, f);
		CHECK(LinearizeDepth(0.0f, n, f) == hamr::test::Approx(n));
		// At d = 1 the denominator f - (f - n) cancels to n in float (the
		// shaders' too - and the old [-1, 1] inverse's), so a large f/n
		// reads the far plane to ~2.5e-4 relative.
		CHECK(LinearizeDepth(1.0f, n, f) == hamr::test::Approx(f).epsilon(5e-4));
		for (int i = 0; i <= 64; ++i)
		{
			// Log-spaced view depths across [n, f]: depth -> linear round trip.
			const float z = n * std::pow(f / n, (float)i / 64.0f);
			const float d = DepthAt(p, z);
			CHECK(d >= -1e-6f);
			CHECK(d <= 1.0f + 1e-6f);
			CHECK(LinearizeDepth(d, n, f) == hamr::test::Approx(z).epsilon(2e-3));
		}
	}

	// The [-1, 1] inverse (the old shaders') reads a [0, 1] depth wrong: at
	// d = 0 it returns 2nf/(f + n), about twice the near plane - the error
	// the switch to [0, 1] removed.
	const float n = 0.1f, f = 1000.0f;
	const float oldInverse = 2.0f * n * f / ((f + n) - 0.0f * (f - n));
	CHECK(oldInverse == hamr::test::Approx(2.0f * n).epsilon(1e-3));
	CHECK(LinearizeDepth(0.0f, n, f) == hamr::test::Approx(n));
}

TEST_CASE("projection: LookAt is the right-handed view - the camera looks down -Z")
{
	const glm::vec3 eye(3.0f, 4.0f, 5.0f);
	const glm::mat4 view = LookAt(eye, glm::vec3(0.0f), kUp);
	CHECK(view == glm::lookAt(eye, glm::vec3(0.0f), kUp));
	const glm::vec4 target = view * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
	CHECK(target.x == hamr::test::Approx(0.0f).epsilon(1e-6));
	CHECK(target.y == hamr::test::Approx(0.0f).epsilon(1e-6));
	CHECK(target.z == hamr::test::Approx(-glm::length(eye)));
	// The full chain: a point at the near plane in front of the camera lands
	// on depth 0, one at the far plane on depth 1.
	const glm::mat4 vp = Perspective(DegreesToRadians(50.0f), 1.0f, 0.5f, 20.0f) * view;
	const glm::vec3 forward = glm::normalize(-eye);
	const glm::vec4 nearClip = vp * glm::vec4(eye + forward * 0.5f, 1.0f);
	const glm::vec4 farClip = vp * glm::vec4(eye + forward * 20.0f, 1.0f);
	CHECK(nearClip.z / nearClip.w == hamr::test::Approx(0.0f).epsilon(1e-5));
	CHECK(farClip.z / farClip.w == hamr::test::Approx(1.0f));
}

} // namespace tmath::projection_tests
