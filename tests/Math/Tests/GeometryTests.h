#pragma once

// Math/Geometry.h - the primitives' known values
// and edge cases: Plane construction and normalisation (incl. the
// degenerate forms), the AABB's empty state, Expand, Arvo's transform
// against the brute-force 8-corner transform, Sphere, and the Frustum
// extracted from [0, 1]-depth projections (glm's *_ZO, built explicitly -
// the engine's own projections are [0, 1] too).

#include "Math/Angle.h"
#include "Math/Geometry.h"
#include "Math/RandomStream.h"
#include "Math/Scalar.h"

#include <doctest/doctest.h>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <cmath>

namespace TinyEngine::Math::geometry_tests {

namespace {
	bool Near3(const glm::vec3& a, const glm::vec3& b, float eps = 1e-5f)
	{
		return std::abs(a.x - b.x) <= eps && std::abs(a.y - b.y) <= eps && std::abs(a.z - b.z) <= eps;
	}
	float Length(const glm::vec3& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

	// The brute-force bound: all 8 corners through the full matrix.
	AABB CornerTransform(const AABB& box, const glm::mat4& m)
	{
		AABB out;
		for (int i = 0; i < 8; ++i)
		{
			const glm::vec3 corner((i & 1) ? box.Max.x : box.Min.x, (i & 2) ? box.Max.y : box.Min.y,
			                       (i & 4) ? box.Max.z : box.Min.z);
			out.Expand(glm::vec3(m * glm::vec4(corner, 1.0f)));
		}
		return out;
	}

	// A view-projection in the [0, 1] depth convention: a camera at `eye`
	// looking at the origin.
	glm::mat4 PerspectiveZO(float fovYRadians, float aspect, float n, float f, const glm::vec3& eye)
	{
		return glm::perspectiveRH_ZO(fovYRadians, aspect, n, f) * glm::lookAt(eye, glm::vec3(0.0f), kUp);
	}
} // namespace

TEST_CASE("math.geometry: Ray - gizmo's defaults and At")
{
	const Ray ray;
	CHECK(ray.Origin == glm::vec3(0.0f));
	CHECK(ray.Direction == kForward);
	const Ray r{ { 1.0f, 2.0f, 3.0f }, { 0.0f, 1.0f, 0.0f } };
	CHECK(r.At(0.0f) == r.Origin);
	CHECK(r.At(2.5f) == glm::vec3(1.0f, 4.5f, 3.0f));
	CHECK(r.At(-1.0f) == glm::vec3(1.0f, 1.0f, 3.0f)); // At itself does not clamp
}

TEST_CASE("math.geometry: Plane - point/normal, three points, coefficients, normalised")
{
	// Point/normal: the normal is normalised and Distance is the origin's
	// signed distance along it.
	const Plane p = Plane::FromPointNormal({ 0.0f, 3.0f, 0.0f }, { 0.0f, 2.0f, 0.0f });
	CHECK(p.Normal == glm::vec3(0.0f, 1.0f, 0.0f));
	CHECK(p.Distance == 3.0f);
	CHECK(p.SignedDistance({ 5.0f, 7.0f, -2.0f }) == 4.0f);
	CHECK(p.SignedDistance({ 0.0f, 1.0f, 0.0f }) == -2.0f);
	CHECK(p.ClosestPoint({ 5.0f, 7.0f, -2.0f }) == glm::vec3(5.0f, 3.0f, -2.0f));
	CHECK_FALSE(p.IsDegenerate());

	// An oblique normal comes out unit length; points on the plane are at 0.
	const glm::vec3 point(1.0f, -2.0f, 0.5f);
	const Plane oblique = Plane::FromPointNormal(point, { 1.0f, 1.0f, 1.0f });
	CHECK(Length(oblique.Normal) == doctest::Approx(1.0f));
	CHECK(oblique.SignedDistance(point) == doctest::Approx(0.0f));
	CHECK(oblique.SignedDistance(point + oblique.Normal * 2.0f) == doctest::Approx(2.0f));

	// Three points: counter-clockwise seen from +Z gives +Z (right-handed).
	const Plane ccw = Plane::FromPoints({ 0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f });
	CHECK(ccw.Normal == glm::vec3(0.0f, 0.0f, 1.0f));
	CHECK(ccw.Distance == 1.0f);
	const Plane cw = Plane::FromPoints({ 0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 1.0f }, { 1.0f, 0.0f, 1.0f });
	CHECK(cw.Normal == glm::vec3(0.0f, 0.0f, -1.0f));
	CHECK(cw.Distance == -1.0f);
	CHECK(cw.SignedDistance({ 3.0f, 4.0f, 1.0f }) == 0.0f);

	// Coefficients: 0x + 0y + 2z - 4 = 0 is z = 2, positive side +Z.
	const Plane coeff = Plane::FromCoefficients({ 0.0f, 0.0f, 2.0f, -4.0f });
	CHECK(coeff.Normal == glm::vec3(0.0f, 0.0f, 1.0f));
	CHECK(coeff.Distance == 2.0f);
	CHECK(coeff.SignedDistance({ 0.0f, 0.0f, 5.0f }) == 3.0f);
}

TEST_CASE("math.geometry: Plane - the degenerate forms have a zero normal")
{
	const Plane zero = Plane::FromPointNormal({ 1.0f, 2.0f, 3.0f }, glm::vec3(0.0f));
	CHECK(zero.IsDegenerate());
	CHECK(zero.Distance == 0.0f);
	CHECK(zero.SignedDistance({ 9.0f, -9.0f, 9.0f }) == 0.0f);
	// Collinear and coincident points.
	CHECK(Plane::FromPoints({ 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f }, { 2.0f, 2.0f, 2.0f }).IsDegenerate());
	CHECK(Plane::FromPoints({ 1.0f, 2.0f, 3.0f }, { 1.0f, 2.0f, 3.0f }, { 1.0f, 2.0f, 3.0f }).IsDegenerate());
	CHECK(Plane::FromCoefficients({ 0.0f, 0.0f, 0.0f, 5.0f }).IsDegenerate());
	// No NaN anywhere.
	const Plane collinear = Plane::FromPoints({ 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, { 3.0f, 0.0f, 0.0f });
	CHECK_FALSE(std::isnan(collinear.Normal.x + collinear.Normal.y + collinear.Normal.z + collinear.Distance));
}

TEST_CASE("math.geometry: AABB - the empty state, Expand, centre/extents")
{
	const AABB empty;
	CHECK(empty.IsEmpty());
	CHECK(AABB::Empty().IsEmpty());
	CHECK_FALSE(empty.Contains(glm::vec3(0.0f)));
	CHECK(empty.Center() == glm::vec3(0.0f));
	CHECK(empty.Extents() == glm::vec3(0.0f));
	CHECK(empty.Size() == glm::vec3(0.0f));

	// One point: a degenerate (zero-size) box, not an empty one.
	AABB box;
	box.Expand(glm::vec3(1.0f, 2.0f, 3.0f));
	CHECK_FALSE(box.IsEmpty());
	CHECK(box.Min == glm::vec3(1.0f, 2.0f, 3.0f));
	CHECK(box.Max == glm::vec3(1.0f, 2.0f, 3.0f));
	CHECK(box.Extents() == glm::vec3(0.0f));
	CHECK(box.Contains(glm::vec3(1.0f, 2.0f, 3.0f)));

	box.Expand(glm::vec3(-1.0f, 4.0f, 3.0f));
	CHECK(box.Min == glm::vec3(-1.0f, 2.0f, 3.0f));
	CHECK(box.Max == glm::vec3(1.0f, 4.0f, 3.0f));
	CHECK(box.Center() == glm::vec3(0.0f, 3.0f, 3.0f));
	CHECK(box.Extents() == glm::vec3(1.0f, 1.0f, 0.0f));
	CHECK(box.Size() == glm::vec3(2.0f, 2.0f, 0.0f));
	CHECK(box.Contains(glm::vec3(1.0f, 4.0f, 3.0f))); // inclusive faces
	CHECK_FALSE(box.Contains(glm::vec3(1.0f, 4.0f, 3.0001f)));

	// Expanding by the empty box is a no-op; expanding the empty box by a
	// box gives that box.
	const AABB before = box;
	box.Expand(AABB::Empty());
	CHECK(box.Min == before.Min);
	CHECK(box.Max == before.Max);
	AABB grown;
	grown.Expand(before);
	CHECK(grown.Min == before.Min);
	CHECK(grown.Max == before.Max);

	// The constructors order the corners; centre/extents round-trip.
	const AABB corners = AABB::FromCorners({ 2.0f, -1.0f, 5.0f }, { -2.0f, 1.0f, 3.0f });
	CHECK(corners.Min == glm::vec3(-2.0f, -1.0f, 3.0f));
	CHECK(corners.Max == glm::vec3(2.0f, 1.0f, 5.0f));
	const AABB ce = AABB::FromCenterExtents({ 1.0f, 1.0f, 1.0f }, { 0.5f, 2.0f, 0.0f });
	CHECK(ce.Min == glm::vec3(0.5f, -1.0f, 1.0f));
	CHECK(ce.Max == glm::vec3(1.5f, 3.0f, 1.0f));
	CHECK(ce.Center() == glm::vec3(1.0f, 1.0f, 1.0f));
	CHECK(ce.Extents() == glm::vec3(0.5f, 2.0f, 0.0f));
	// An inverted box written by hand is empty.
	CHECK(AABB{ { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 1.0f } }.IsEmpty());
}

TEST_CASE("math.geometry: TransformAABB - Arvo matches the brute-force 8-corner transform")
{
	// Exact cases: dyadic values make every product and sum exact.
	const AABB unit = AABB::FromCorners({ -1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, 1.0f });
	const AABB moved = TransformAABB(unit, glm::translate(glm::mat4(1.0f), { 3.0f, -2.0f, 0.5f }));
	CHECK(moved.Min == glm::vec3(2.0f, -3.0f, -0.5f));
	CHECK(moved.Max == glm::vec3(4.0f, -1.0f, 1.5f));
	const AABB scaled = TransformAABB(unit, glm::scale(glm::mat4(1.0f), { 2.0f, -0.5f, 1.0f }));
	CHECK(scaled.Min == glm::vec3(-2.0f, -0.5f, -1.0f)); // a negative scale flips, the box does not
	CHECK(scaled.Max == glm::vec3(2.0f, 0.5f, 1.0f));
	// A 90-degree turn about Z, written exactly (glm::rotate's cos(pi/2) is not 0).
	glm::mat4 rotZ(0.0f);
	rotZ[0] = { 0.0f, 1.0f, 0.0f, 0.0f };
	rotZ[1] = { -1.0f, 0.0f, 0.0f, 0.0f };
	rotZ[2] = { 0.0f, 0.0f, 1.0f, 0.0f };
	rotZ[3] = { 0.0f, 0.0f, 0.0f, 1.0f };
	const AABB flat = AABB::FromCorners({ 0.0f, 0.0f, 0.0f }, { 4.0f, 1.0f, 2.0f });
	const AABB turned = TransformAABB(flat, rotZ);
	CHECK(turned.Min == glm::vec3(-1.0f, 0.0f, 0.0f));
	CHECK(turned.Max == glm::vec3(0.0f, 4.0f, 2.0f));
	CHECK(TransformAABB(flat, glm::mat4(1.0f)).Min == flat.Min);
	CHECK(TransformAABB(flat, glm::mat4(1.0f)).Max == flat.Max);

	// The empty box stays empty; a point box stays a point.
	CHECK(TransformAABB(AABB::Empty(), rotZ).IsEmpty());
	AABB point;
	point.Expand(glm::vec3(1.0f, 2.0f, 3.0f));
	const AABB pointOut = TransformAABB(point, rotZ);
	CHECK(pointOut.Min == glm::vec3(-2.0f, 1.0f, 3.0f));
	CHECK(pointOut.Max == pointOut.Min);

	// Random affine matrices (rotation * non-uniform scale incl. negative,
	// plus translation) and random boxes: Arvo equals the 8-corner bound up
	// to the rounding of the different summation order.
	RandomStream rng(0x4D41544847454F4Dull);
	for (int i = 0; i < 500; ++i)
	{
		const glm::vec3 axis = glm::normalize(glm::vec3(rng.FloatInRange(-1.0f, 1.0f), rng.FloatInRange(-1.0f, 1.0f), rng.FloatInRange(-1.0f, 1.0f)) + glm::vec3(1e-3f));
		glm::mat4 m = glm::translate(glm::mat4(1.0f), { rng.FloatInRange(-50.0f, 50.0f), rng.FloatInRange(-50.0f, 50.0f), rng.FloatInRange(-50.0f, 50.0f) });
		m = glm::rotate(m, rng.FloatInRange(-kPi, kPi), axis);
		m = glm::scale(m, { rng.FloatInRange(-3.0f, 3.0f), rng.FloatInRange(0.1f, 3.0f), rng.FloatInRange(-3.0f, -0.1f) });
		const glm::vec3 a(rng.FloatInRange(-10.0f, 10.0f), rng.FloatInRange(-10.0f, 10.0f), rng.FloatInRange(-10.0f, 10.0f));
		const glm::vec3 b(rng.FloatInRange(-10.0f, 10.0f), rng.FloatInRange(-10.0f, 10.0f), rng.FloatInRange(-10.0f, 10.0f));
		const AABB box = AABB::FromCorners(a, b);
		const AABB arvo = TransformAABB(box, m);
		const AABB brute = CornerTransform(box, m);
		CHECK(Near3(arvo.Min, brute.Min, 1e-3f));
		CHECK(Near3(arvo.Max, brute.Max, 1e-3f));
	}
}

TEST_CASE("math.geometry: Sphere - containment and the empty sphere")
{
	const Sphere s{ { 1.0f, 0.0f, 0.0f }, 2.0f };
	CHECK_FALSE(s.IsEmpty());
	CHECK(s.Contains({ 3.0f, 0.0f, 0.0f })); // on the surface
	CHECK(s.Contains({ 1.0f, 1.0f, 1.0f }));
	CHECK_FALSE(s.Contains({ 3.0f, 0.001f, 0.0f }));
	const Sphere point{ { 0.0f, 0.0f, 0.0f }, 0.0f };
	CHECK_FALSE(point.IsEmpty());
	CHECK(point.Contains(glm::vec3(0.0f)));
	const Sphere empty{ { 0.0f, 0.0f, 0.0f }, -1.0f };
	CHECK(empty.IsEmpty());
	CHECK_FALSE(empty.Contains(glm::vec3(0.0f)));
}

TEST_CASE("math.geometry: Frustum - planes of a [0, 1]-depth perspective")
{
	// View-space frustum (projection alone): camera at the origin, -Z forward.
	const float n = 0.5f, f = 50.0f;
	const float fovY = DegreesToRadians(90.0f);
	const Frustum fr = Frustum::FromViewProjection(glm::perspectiveRH_ZO(fovY, 1.0f, n, f));

	// Every plane is unit length and points inward (the frustum's interior
	// point is on the positive side of all six).
	const glm::vec3 interior(0.0f, 0.0f, -10.0f);
	for (const Plane& plane : fr.Planes)
	{
		CHECK(Length(plane.Normal) == doctest::Approx(1.0f));
		CHECK(plane.SignedDistance(interior) > 0.0f);
	}
	// Near is z = -n facing -Z, Far is z = -f facing +Z.
	CHECK(Near3(fr.Get(FrustumPlane::Near).Normal, { 0.0f, 0.0f, -1.0f }));
	CHECK(fr.Get(FrustumPlane::Near).Distance == doctest::Approx(n));
	CHECK(Near3(fr.Get(FrustumPlane::Far).Normal, { 0.0f, 0.0f, 1.0f }));
	CHECK(fr.Get(FrustumPlane::Far).Distance == doctest::Approx(-f).epsilon(1e-4));
	// A 90-degree square frustum: the side planes are at 45 degrees.
	const float h = std::sqrt(0.5f);
	CHECK(Near3(fr.Get(FrustumPlane::Left).Normal, { h, 0.0f, -h }));
	CHECK(Near3(fr.Get(FrustumPlane::Right).Normal, { -h, 0.0f, -h }));
	CHECK(Near3(fr.Get(FrustumPlane::Bottom).Normal, { 0.0f, h, -h }));
	CHECK(Near3(fr.Get(FrustumPlane::Top).Normal, { 0.0f, -h, -h }));
	for (FrustumPlane side : { FrustumPlane::Left, FrustumPlane::Right, FrustumPlane::Bottom, FrustumPlane::Top })
		CHECK(fr.Get(side).Distance == doctest::Approx(0.0f));

	// The configured near and far planes, exactly where they were built.
	CHECK(fr.Contains({ 0.0f, 0.0f, -n * 1.01f }));
	CHECK_FALSE(fr.Contains({ 0.0f, 0.0f, -n * 0.99f }));
	CHECK(fr.Contains({ 0.0f, 0.0f, -f * 0.99f }));
	CHECK_FALSE(fr.Contains({ 0.0f, 0.0f, -f * 1.01f }));
	CHECK(fr.Contains({ 9.9f, 0.0f, -10.0f }));
	CHECK_FALSE(fr.Contains({ 10.1f, 0.0f, -10.0f }));
	CHECK_FALSE(fr.Contains({ 0.0f, 0.0f, 1.0f })); // behind the camera

	// With a view: the camera at +Z looking at the origin gives the same
	// frustum moved there.
	const glm::vec3 eye(0.0f, 0.0f, 20.0f);
	const Frustum world = Frustum::FromViewProjection(PerspectiveZO(fovY, 1.0f, n, f, eye));
	CHECK(world.Contains(glm::vec3(0.0f)));
	CHECK(world.Contains(eye + glm::vec3(0.0f, 0.0f, -n * 1.01f)));
	CHECK_FALSE(world.Contains(eye + glm::vec3(0.0f, 0.0f, -n * 0.99f)));
	CHECK_FALSE(world.Contains(eye + glm::vec3(0.0f, 0.0f, 1.0f)));
	CHECK(world.Get(FrustumPlane::Near).SignedDistance(eye) == doctest::Approx(-n));
}

TEST_CASE("math.geometry: Frustum - a [0, 1]-depth orthographic box")
{
	const Frustum fr = Frustum::FromViewProjection(glm::orthoRH_ZO(-4.0f, 4.0f, -2.0f, 2.0f, 1.0f, 9.0f));
	CHECK(Near3(fr.Get(FrustumPlane::Left).Normal, { 1.0f, 0.0f, 0.0f }));
	CHECK(fr.Get(FrustumPlane::Left).Distance == doctest::Approx(-4.0f));
	CHECK(Near3(fr.Get(FrustumPlane::Right).Normal, { -1.0f, 0.0f, 0.0f }));
	CHECK(fr.Get(FrustumPlane::Right).Distance == doctest::Approx(-4.0f));
	CHECK(Near3(fr.Get(FrustumPlane::Top).Normal, { 0.0f, -1.0f, 0.0f }));
	CHECK(fr.Get(FrustumPlane::Top).Distance == doctest::Approx(-2.0f));
	CHECK(Near3(fr.Get(FrustumPlane::Near).Normal, { 0.0f, 0.0f, -1.0f }));
	CHECK(fr.Get(FrustumPlane::Near).Distance == doctest::Approx(1.0f));
	CHECK(fr.Get(FrustumPlane::Far).Distance == doctest::Approx(-9.0f));
	CHECK(fr.Contains({ 4.0f, -2.0f, -1.0f })); // a corner, inclusive
	CHECK_FALSE(fr.Contains({ 4.01f, 0.0f, -5.0f }));
	CHECK_FALSE(fr.Contains({ 0.0f, 0.0f, -0.99f }));
	CHECK_FALSE(fr.Contains({ 0.0f, 0.0f, -9.01f }));
}

TEST_CASE("math.geometry: Frustum - a [-1, 1]-depth matrix puts Near where z_ndc = 0")
{
	// Why FromViewProjection requires [0, 1] depth (Geometry.h): glm's
	// default [-1, 1] form extracts a Near plane at the depth
	// 2nf / (f + n), about twice the configured near - the depth Vulkan
	// clipped the engine's projections at before they switched to [0, 1].
	const float n = 0.5f, f = 50.0f;
	const Frustum fr = Frustum::FromViewProjection(glm::perspectiveRH_NO(DegreesToRadians(60.0f), 1.0f, n, f));
	CHECK(fr.Get(FrustumPlane::Near).Distance == doctest::Approx(2.0f * n * f / (f + n)));
	CHECK_FALSE(fr.Contains({ 0.0f, 0.0f, -n * 1.5f }));
	// The far plane agrees between the two forms.
	CHECK(fr.Get(FrustumPlane::Far).Distance == doctest::Approx(-f).epsilon(1e-4));
}

TEST_CASE("math.geometry: Frustum - a degenerate matrix gives degenerate planes, no NaN")
{
	const Frustum fr = Frustum::FromViewProjection(glm::mat4(0.0f));
	for (const Plane& plane : fr.Planes)
	{
		CHECK(plane.IsDegenerate());
		CHECK_FALSE(std::isnan(plane.Distance));
	}
}

} // namespace TinyEngine::Math::geometry_tests
