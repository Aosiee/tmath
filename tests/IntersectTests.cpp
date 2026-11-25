
// tmath/Intersect.h - known values and the edge cases Intersect.h's rules
// name: rays parallel to a plane, origins inside an AABB/sphere, grazing an
// AABB edge / a sphere tangent / a triangle edge, the zero-direction ray
// (hits nothing), back-facing triangles, boxes and spheres straddling
// frustum planes or sitting by a frustum corner, and the empty volumes.

#include "tmath/Angle.h"
#include "tmath/Geometry.h"
#include "tmath/Intersect.h"
#include "tmath/RandomStream.h"

#include "hamr/test.h"

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/geometric.hpp>

#include <cmath>

namespace tmath::intersect_tests {

namespace {
	const Ray kZeroRay{ { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f } };
} // namespace

TEST_CASE("intersect: ray vs plane - hits, behind, parallel, on the plane")
{
	const Plane ground = Plane::FromPointNormal({ 0.0f, 0.0f, 0.0f }, kUp);
	const Ray down{ { 1.0f, 5.0f, 2.0f }, { 0.0f, -1.0f, 0.0f } };
	REQUIRE(RayPlane(down, ground).has_value());
	CHECK(*RayPlane(down, ground) == 5.0f);
	// Two-sided: from below, looking up.
	const Ray up{ { 0.0f, -3.0f, 0.0f }, { 0.0f, 1.0f, 0.0f } };
	CHECK(RayPlane(up, ground) == 3.0f);
	// The plane behind the origin is no hit.
	const Ray away{ { 0.0f, 5.0f, 0.0f }, { 0.0f, 1.0f, 0.0f } };
	CHECK_FALSE(RayPlane(away, ground).has_value());
	// Parallel (in-plane direction and nearly so) never hits - even with the
	// origin on the plane.
	CHECK_FALSE(RayPlane(Ray{ { 0.0f, 1.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } }, ground).has_value());
	CHECK_FALSE(RayPlane(Ray{ { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } }, ground).has_value());
	CHECK_FALSE(RayPlane(Ray{ { 0.0f, 1.0f, 0.0f }, glm::normalize(glm::vec3(1.0f, -5e-7f, 0.0f)) }, ground).has_value());
	CHECK(RayPlane(Ray{ { 0.0f, 1.0f, 0.0f }, glm::normalize(glm::vec3(1.0f, -1e-4f, 0.0f)) }, ground).has_value());
	// The origin on the plane, crossing it: t = 0.
	CHECK(RayPlane(Ray{ { 4.0f, 0.0f, 0.0f }, { 0.0f, -1.0f, 0.0f } }, ground) == 0.0f);
	// An oblique plane: the hit lies on it.
	const Plane oblique = Plane::FromPointNormal({ 1.0f, 2.0f, 3.0f }, { 1.0f, 2.0f, -1.0f });
	const Ray r{ { -4.0f, -1.0f, 0.5f }, glm::normalize(glm::vec3(1.0f, 1.0f, 0.2f)) };
	const std::optional<float> t = RayPlane(r, oblique);
	REQUIRE(t.has_value());
	CHECK(oblique.SignedDistance(r.At(*t)) == hamr::test::Approx(0.0f).epsilon(1e-5));
	// The zero-direction ray and the degenerate plane hit nothing.
	CHECK_FALSE(RayPlane(kZeroRay, ground).has_value());
	CHECK_FALSE(RayPlane(down, Plane::FromPointNormal({ 0.0f, 0.0f, 0.0f }, glm::vec3(0.0f))).has_value());
}

TEST_CASE("intersect: ray vs plane - the point/normal form agrees with the Plane form")
{
	// On exact
	// inputs both forms give the same t, and they agree on hit/miss.
	const glm::vec3 point(0.0f, 0.0f, -2.0f);
	const glm::vec3 normal(0.0f, 0.0f, 1.0f);
	const Ray forward{ { 0.5f, 0.25f, 3.0f }, { 0.0f, 0.0f, -1.0f } };
	CHECK(RayPlane(forward, point, normal) == 5.0f);
	CHECK(RayPlane(forward, Plane::FromPointNormal(point, normal)) == 5.0f);
	CHECK_FALSE(RayPlane(Ray{ { 0.0f, 0.0f, -3.0f }, { 0.0f, 0.0f, -1.0f } }, point, normal).has_value());
	CHECK_FALSE(RayPlane(Ray{ { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } }, point, normal).has_value());
	CHECK_FALSE(RayPlane(kZeroRay, point, normal).has_value());

	RandomStream rng(0x504C414E45ull);
	for (int i = 0; i < 1000; ++i)
	{
		const glm::vec3 p(rng.FloatInRange(-5.0f, 5.0f), rng.FloatInRange(-5.0f, 5.0f), rng.FloatInRange(-5.0f, 5.0f));
		const glm::vec3 n = rng.UnitVector();
		const Ray ray{ { rng.FloatInRange(-5.0f, 5.0f), rng.FloatInRange(-5.0f, 5.0f), rng.FloatInRange(-5.0f, 5.0f) }, rng.UnitVector() };
		const std::optional<float> a = RayPlane(ray, p, n);
		const std::optional<float> b = RayPlane(ray, Plane::FromPointNormal(p, n));
		REQUIRE(a.has_value() == b.has_value());
		// Away from grazing angles (where t is ill-conditioned) the t agree.
		if (a && std::abs(glm::dot(ray.Direction, n)) > 1e-2f)
			CHECK(*a == hamr::test::Approx(*b).epsilon(1e-3));
	}
}

TEST_CASE("intersect: ray vs AABB - slab hits, inside, grazing, parallel")
{
	const AABB box = AABB::FromCorners({ -1.0f, -1.0f, -1.0f }, { 1.0f, 1.0f, 1.0f });
	CHECK(RayAABB(Ray{ { -5.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } }, box) == 4.0f);
	CHECK(RayAABB(Ray{ { 0.0f, 0.0f, 7.0f }, { 0.0f, 0.0f, -1.0f } }, box) == 6.0f);
	// Diagonal entry through a corner region.
	const Ray diagonal{ { -3.0f, -3.0f, -3.0f }, glm::normalize(glm::vec3(1.0f)) };
	const std::optional<float> td = RayAABB(diagonal, box);
	REQUIRE(td.has_value());
	CHECK(*td == hamr::test::Approx(2.0f * std::sqrt(3.0f)));
	// Missing, and pointing away.
	CHECK_FALSE(RayAABB(Ray{ { -5.0f, 2.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } }, box).has_value());
	CHECK_FALSE(RayAABB(Ray{ { -5.0f, 0.0f, 0.0f }, { -1.0f, 0.0f, 0.0f } }, box).has_value());
	// The origin inside: t = 0, in every direction.
	CHECK(RayAABB(Ray{ { 0.5f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } }, box) == 0.0f);
	CHECK(RayAABB(Ray{ { 0.5f, 0.0f, 0.0f }, glm::normalize(glm::vec3(-1.0f, 2.0f, 0.5f)) }, box) == 0.0f);
	// Grazing: a ray along a face, and along an edge, is inclusive.
	CHECK(RayAABB(Ray{ { -5.0f, 1.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } }, box) == 4.0f);
	CHECK(RayAABB(Ray{ { -5.0f, 1.0f, 1.0f }, { 1.0f, 0.0f, 0.0f } }, box) == 4.0f);
	CHECK_FALSE(RayAABB(Ray{ { -5.0f, 1.0001f, 1.0f }, { 1.0f, 0.0f, 0.0f } }, box).has_value());
	// Through an edge diagonally: enters exactly at the edge (1, 1, 0).
	const Ray edge{ { 0.0f, 2.0f, 0.0f }, glm::normalize(glm::vec3(1.0f, -1.0f, 0.0f)) };
	const std::optional<float> te = RayAABB(edge, box);
	REQUIRE(te.has_value());
	CHECK(*te == hamr::test::Approx(std::sqrt(2.0f)));
	// Parallel to an axis with the origin outside that slab: no NaN, a miss.
	CHECK_FALSE(RayAABB(Ray{ { -1.5f, -5.0f, 0.0f }, { 0.0f, 1.0f, 0.0f } }, box).has_value());
	// Parallel with the origin ON the slab's boundary (the 0/0 case).
	CHECK(RayAABB(Ray{ { -1.0f, -5.0f, 0.0f }, { 0.0f, 1.0f, 0.0f } }, box) == 4.0f);
	// A flat (zero-thickness) box is hit; the empty box never is; the
	// zero-direction ray hits nothing, even from inside.
	const AABB flat = AABB::FromCorners({ -1.0f, 0.0f, -1.0f }, { 1.0f, 0.0f, 1.0f });
	CHECK(RayAABB(Ray{ { 0.0f, 3.0f, 0.0f }, { 0.0f, -1.0f, 0.0f } }, flat) == 3.0f);
	CHECK_FALSE(RayAABB(Ray{ { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } }, AABB::Empty()).has_value());
	CHECK_FALSE(RayAABB(kZeroRay, box).has_value());
}

TEST_CASE("intersect: ray vs sphere - entry, inside, tangent, behind")
{
	const Sphere s{ { 0.0f, 0.0f, -10.0f }, 2.0f };
	CHECK(RaySphere(Ray{ { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, -1.0f } }, s) == 8.0f);
	// A non-unit direction: t is in its units.
	CHECK(RaySphere(Ray{ { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, -2.0f } }, s) == 4.0f);
	// Inside, and on the surface: t = 0.
	CHECK(RaySphere(Ray{ { 0.0f, 1.0f, -10.0f }, { 1.0f, 0.0f, 0.0f } }, s) == 0.0f);
	CHECK(RaySphere(Ray{ { 0.0f, 0.0f, -8.0f }, { 0.0f, 0.0f, 1.0f } }, s) == 0.0f);
	// Tangent: inclusive.
	CHECK(RaySphere(Ray{ { 2.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, -1.0f } }, s) == 10.0f);
	CHECK_FALSE(RaySphere(Ray{ { 2.001f, 0.0f, 0.0f }, { 0.0f, 0.0f, -1.0f } }, s).has_value());
	// Behind the origin, and pointing away.
	CHECK_FALSE(RaySphere(Ray{ { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } }, s).has_value());
	CHECK_FALSE(RaySphere(Ray{ { 0.0f, 0.0f, -20.0f }, { 0.0f, 0.0f, -1.0f } }, s).has_value());
	// The hit point is on the surface.
	const Ray oblique{ { 3.0f, 1.0f, 0.0f }, glm::normalize(glm::vec3(-0.3f, -0.1f, -1.0f)) };
	const std::optional<float> t = RaySphere(oblique, s);
	REQUIRE(t.has_value());
	CHECK(glm::length(oblique.At(*t) - s.Center) == hamr::test::Approx(2.0f));
	// A point sphere is hit dead on; the empty sphere and the zero ray never.
	CHECK(RaySphere(Ray{ { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, -1.0f } }, Sphere{ { 0.0f, 0.0f, -4.0f }, 0.0f }) == 4.0f);
	CHECK_FALSE(RaySphere(Ray{ { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, -1.0f } }, Sphere{ { 0.0f, 0.0f, -4.0f }, -1.0f }).has_value());
	CHECK_FALSE(RaySphere(Ray{ { 0.0f, 0.0f, -10.0f }, { 0.0f, 0.0f, 0.0f } }, s).has_value());
}

TEST_CASE("intersect: ray vs triangle - Moller-Trumbore hits, edges, faces")
{
	// Counter-clockwise seen from +Z: the front face faces +Z.
	const glm::vec3 a(0.0f, 0.0f, 0.0f), b(1.0f, 0.0f, 0.0f), c(0.0f, 1.0f, 0.0f);
	const Ray fromFront{ { 0.25f, 0.25f, 2.0f }, { 0.0f, 0.0f, -1.0f } };
	const std::optional<RayTriangleHit> hit = RayTriangle(fromFront, a, b, c);
	REQUIRE(hit.has_value());
	CHECK(hit->T == 2.0f);
	CHECK(hit->U == 0.25f);
	CHECK(hit->V == 0.25f);
	CHECK(RayTriangle(fromFront, a, b, c, TriangleSides::FrontOnly).has_value());

	// The back face: two-sided hits, front-only culls.
	const Ray fromBack{ { 0.25f, 0.25f, -2.0f }, { 0.0f, 0.0f, 1.0f } };
	REQUIRE(RayTriangle(fromBack, a, b, c).has_value());
	CHECK(RayTriangle(fromBack, a, b, c)->T == 2.0f);
	CHECK_FALSE(RayTriangle(fromBack, a, b, c, TriangleSides::FrontOnly).has_value());
	// The same triangle wound the other way flips which face is culled.
	CHECK(RayTriangle(fromBack, a, c, b, TriangleSides::FrontOnly).has_value());
	CHECK_FALSE(RayTriangle(fromFront, a, c, b, TriangleSides::FrontOnly).has_value());

	// Grazing an edge (V = 0), the hypotenuse (U + V = 1) and a vertex:
	// inclusive. Just past them: a miss.
	const std::optional<RayTriangleHit> onEdge = RayTriangle(Ray{ { 0.5f, 0.0f, 1.0f }, { 0.0f, 0.0f, -1.0f } }, a, b, c);
	REQUIRE(onEdge.has_value());
	CHECK(onEdge->U == 0.5f);
	CHECK(onEdge->V == 0.0f);
	CHECK(RayTriangle(Ray{ { 0.5f, 0.5f, 1.0f }, { 0.0f, 0.0f, -1.0f } }, a, b, c).has_value());
	CHECK(RayTriangle(Ray{ { 1.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, -1.0f } }, a, b, c).has_value());
	CHECK_FALSE(RayTriangle(Ray{ { 0.5f, -0.001f, 1.0f }, { 0.0f, 0.0f, -1.0f } }, a, b, c).has_value());
	CHECK_FALSE(RayTriangle(Ray{ { 0.5f, 0.501f, 1.0f }, { 0.0f, 0.0f, -1.0f } }, a, b, c).has_value());

	// Behind the origin.
	CHECK_FALSE(RayTriangle(Ray{ { 0.25f, 0.25f, -1.0f }, { 0.0f, 0.0f, -1.0f } }, a, b, c).has_value());
	// Parallel (in the triangle's plane, through it): a miss.
	CHECK_FALSE(RayTriangle(Ray{ { -1.0f, 0.25f, 0.0f }, { 1.0f, 0.0f, 0.0f } }, a, b, c).has_value());
	// Degenerate triangles and the zero-direction ray: a miss.
	CHECK_FALSE(RayTriangle(fromFront, a, b, glm::vec3(2.0f, 0.0f, 0.0f)).has_value());
	CHECK_FALSE(RayTriangle(fromFront, a, a, a).has_value());
	CHECK_FALSE(RayTriangle(Ray{ { 0.25f, 0.25f, 0.0f }, { 0.0f, 0.0f, 0.0f } }, a, b, c).has_value());

	// The parallel test is scale-free: a tiny triangle is still hit.
	const float k = 1e-4f;
	const std::optional<RayTriangleHit> tiny = RayTriangle(Ray{ { 0.25f * k, 0.25f * k, 1.0f }, { 0.0f, 0.0f, -1.0f } }, a * k, b * k, c * k);
	REQUIRE(tiny.has_value());
	CHECK(tiny->T == hamr::test::Approx(1.0f));

	// The barycentrics reconstruct the hit point, oblique.
	const glm::vec3 p0(1.0f, -2.0f, 3.0f), p1(4.0f, 0.5f, 2.0f), p2(-1.0f, 3.0f, 0.0f);
	const glm::vec3 target = 0.2f * p0 + 0.5f * p1 + 0.3f * p2;
	const glm::vec3 origin(10.0f, 8.0f, 9.0f);
	const Ray toTarget{ origin, glm::normalize(target - origin) };
	const std::optional<RayTriangleHit> oblique = RayTriangle(toTarget, p0, p1, p2);
	REQUIRE(oblique.has_value());
	CHECK(oblique->U == hamr::test::Approx(0.5f));
	CHECK(oblique->V == hamr::test::Approx(0.3f));
	const glm::vec3 recon = (1.0f - oblique->U - oblique->V) * p0 + oblique->U * p1 + oblique->V * p2;
	CHECK(glm::length(recon - toTarget.At(oblique->T)) == hamr::test::Approx(0.0f).epsilon(1e-4));
	CHECK(glm::length(recon - target) == hamr::test::Approx(0.0f).epsilon(1e-4));
}

TEST_CASE("intersect: AABB and sphere vs frustum - inside, intersect, outside")
{
	// View-space perspective, [0, 1] depth: 90 degrees, near 1, far 100.
	const Frustum fr = Frustum::FromViewProjection(glm::perspectiveRH_ZO(DegreesToRadians(90.0f), 1.0f, 1.0f, 100.0f));

	// Inside, straddling one plane, wholly outside one plane.
	CHECK(ClassifyAABB(fr, AABB::FromCenterExtents({ 0.0f, 0.0f, -20.0f }, glm::vec3(1.0f))) == Containment::Inside);
	CHECK(ClassifyAABB(fr, AABB::FromCenterExtents({ 0.0f, 0.0f, -1.0f }, glm::vec3(0.25f))) == Containment::Intersect); // the near plane
	CHECK(ClassifyAABB(fr, AABB::FromCenterExtents({ 0.0f, 0.0f, -100.0f }, glm::vec3(2.0f))) == Containment::Intersect); // the far plane
	CHECK(ClassifyAABB(fr, AABB::FromCenterExtents({ 20.0f, 0.0f, -20.0f }, glm::vec3(1.0f))) == Containment::Intersect); // the right plane
	CHECK(ClassifyAABB(fr, AABB::FromCenterExtents({ 30.0f, 0.0f, -20.0f }, glm::vec3(1.0f))) == Containment::Outside);
	CHECK(ClassifyAABB(fr, AABB::FromCenterExtents({ 0.0f, 0.0f, 5.0f }, glm::vec3(1.0f))) == Containment::Outside); // behind
	CHECK(ClassifyAABB(fr, AABB::FromCenterExtents({ 0.0f, 0.0f, -0.5f }, glm::vec3(0.25f))) == Containment::Outside); // short of near
	// Enclosing the whole frustum: it straddles every plane.
	CHECK(ClassifyAABB(fr, AABB::FromCenterExtents({ 0.0f, 0.0f, -50.0f }, glm::vec3(500.0f))) == Containment::Intersect);
	// The frustum corner: a box outside the frustum by its right/far edge -
	// past the far plane where it is inside the right plane and past the
	// right plane where it is inside the far one, so it straddles both
	// planes and is wholly outside neither. The per-plane test is
	// conservative and reports Intersect, never a false Outside. A sample
	// grid over the box confirms no point of it is in the frustum.
	const AABB corner = AABB::FromCorners({ 100.5f, -1.0f, -102.0f }, { 102.5f, 1.0f, -99.5f });
	int insideSamples = 0;
	for (int i = 0; i <= 10; ++i)
		for (int j = 0; j <= 10; ++j)
			for (int k = 0; k <= 10; ++k)
				if (fr.Contains(corner.Min + corner.Size() * glm::vec3(i, j, k) * 0.1f))
					++insideSamples;
	CHECK(insideSamples == 0);
	CHECK(fr.Get(FrustumPlane::Right).SignedDistance(corner.Min) > 0.0f);
	CHECK(fr.Get(FrustumPlane::Far).SignedDistance(corner.Max) > 0.0f);
	CHECK(ClassifyAABB(fr, corner) == Containment::Intersect);

	// The empty box is Outside; a point box classifies as its point.
	CHECK(ClassifyAABB(fr, AABB::Empty()) == Containment::Outside);
	AABB point;
	point.Expand({ 0.0f, 0.0f, -10.0f });
	CHECK(ClassifyAABB(fr, point) == Containment::Inside);

	// Spheres: the same cases.
	CHECK(ClassifySphere(fr, Sphere{ { 0.0f, 0.0f, -20.0f }, 1.0f }) == Containment::Inside);
	CHECK(ClassifySphere(fr, Sphere{ { 0.0f, 0.0f, -1.0f }, 0.5f }) == Containment::Intersect);
	CHECK(ClassifySphere(fr, Sphere{ { 20.0f, 0.0f, -20.0f }, 1.0f }) == Containment::Intersect);
	CHECK(ClassifySphere(fr, Sphere{ { 30.0f, 0.0f, -20.0f }, 1.0f }) == Containment::Outside);
	CHECK(ClassifySphere(fr, Sphere{ { 0.0f, 0.0f, 5.0f }, 1.0f }) == Containment::Outside);
	CHECK(ClassifySphere(fr, Sphere{ { 0.0f, 0.0f, -10.0f }, 0.0f }) == Containment::Inside);
	CHECK(ClassifySphere(fr, Sphere{ { 0.0f, 0.0f, -10.0f }, -1.0f }) == Containment::Outside);

	// An orthographic frustum, whose planes come out exact (x, y in
	// [-4, 4], z in [-8, 0]): the faces are inclusive. A volume that fills
	// it exactly is Inside; one touching a face from outside (distance
	// exactly 0) is Intersect; one just past it is Outside.
	const Frustum box = Frustum::FromViewProjection(glm::orthoRH_ZO(-4.0f, 4.0f, -4.0f, 4.0f, 0.0f, 8.0f));
	CHECK(ClassifyAABB(box, AABB::FromCorners({ -4.0f, -4.0f, -8.0f }, { 4.0f, 4.0f, 0.0f })) == Containment::Inside);
	CHECK(ClassifyAABB(box, AABB::FromCorners({ 4.0f, -1.0f, -2.0f }, { 5.0f, 1.0f, -1.0f })) == Containment::Intersect);
	CHECK(ClassifyAABB(box, AABB::FromCorners({ 4.5f, -1.0f, -2.0f }, { 5.0f, 1.0f, -1.0f })) == Containment::Outside);
	CHECK(ClassifySphere(box, Sphere{ { 0.0f, 0.0f, -4.0f }, 4.0f }) == Containment::Inside);
	CHECK(ClassifySphere(box, Sphere{ { 0.0f, 0.0f, -9.0f }, 1.0f }) == Containment::Intersect); // touches the far face
	CHECK(ClassifySphere(box, Sphere{ { 0.0f, 0.0f, -9.5f }, 1.0f }) == Containment::Outside);
}

TEST_CASE("intersect: AABB vs AABB - overlap, touching, disjoint, empty")
{
	const AABB a = AABB::FromCorners({ 0.0f, 0.0f, 0.0f }, { 2.0f, 2.0f, 2.0f });
	CHECK(Overlaps(a, AABB::FromCorners({ 1.0f, 1.0f, 1.0f }, { 3.0f, 3.0f, 3.0f })));
	CHECK(Overlaps(a, AABB::FromCorners({ 0.5f, 0.5f, 0.5f }, { 1.0f, 1.0f, 1.0f }))); // contained
	CHECK(Overlaps(a, a));
	CHECK(Overlaps(a, AABB::FromCorners({ 2.0f, 0.0f, 0.0f }, { 3.0f, 1.0f, 1.0f }))); // a shared face
	CHECK(Overlaps(a, AABB::FromCorners({ 2.0f, 2.0f, 2.0f }, { 3.0f, 3.0f, 3.0f }))); // a shared corner
	CHECK_FALSE(Overlaps(a, AABB::FromCorners({ 2.001f, 0.0f, 0.0f }, { 3.0f, 1.0f, 1.0f })));
	CHECK_FALSE(Overlaps(a, AABB::FromCorners({ 0.0f, 0.0f, -3.0f }, { 2.0f, 2.0f, -0.5f })));
	// Disjoint on one axis only is disjoint.
	CHECK_FALSE(Overlaps(a, AABB::FromCorners({ 0.0f, 5.0f, 0.0f }, { 2.0f, 6.0f, 2.0f })));
	// The empty box overlaps nothing, itself included; a point box inside
	// overlaps.
	CHECK_FALSE(Overlaps(a, AABB::Empty()));
	CHECK_FALSE(Overlaps(AABB::Empty(), AABB::Empty()));
	AABB point;
	point.Expand({ 1.0f, 1.0f, 1.0f });
	CHECK(Overlaps(a, point));
	CHECK(Overlaps(point, a));
}

} // namespace tmath::intersect_tests
