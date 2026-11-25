#pragma once

// The intersection tests between Geometry.h's
// primitives.
//
// The ray tests return the distance t along the ray (the hit is ray.At(t))
// of the FIRST hit at t >= 0, or nullopt. Shared rules:
// - Hits behind the origin (t < 0) do not count.
// - Boundaries are inclusive: a ray grazing an AABB edge, a sphere's
//   tangent or a triangle edge hits.
// - A ray starting inside a solid (AABB, sphere) hits at t = 0.
// - A zero-direction ray hits nothing, in every test (it has no t).
// - The tolerances are kEpsilon's: with a unit Direction and a unit normal
//   they are the cosine below which a ray counts as parallel to a plane
//   (about 5.7e-5 degrees off the plane).
// - The empty AABB / sphere is missed by every test.
//
// Determinism: arithmetic plus std::sqrt - bit-stable given the same
// evaluation order and no FMA contraction (Conventions.h). These inline
// with the includer's flags.

#include "tmath/Constants.h"
#include "tmath/Geometry.h"

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include <cmath>
#include <limits>
#include <optional>

namespace tmath {

	namespace detail {
		inline float Dot(const glm::vec3& a, const glm::vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
		inline glm::vec3 Cross(const glm::vec3& a, const glm::vec3& b)
		{
			return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
		}
	} // namespace detail

	// ---- Ray vs plane -----------------------------------------------------------
	// The point/normal form: the plane through `point` with `normal` (need
	// not be unit; the parallel test is |dot(direction, normal)| < kEpsilon,
	// so unit vectors make it an angle). Two-sided. The arithmetic is kept in this
	// exact order, because callers depend on its rounding:
	// t = dot(point - origin, normal) / denom.
	inline std::optional<float> RayPlane(const Ray& ray, const glm::vec3& point, const glm::vec3& normal)
	{
		const float denom = glm::dot(ray.Direction, normal);
		if (std::abs(denom) < kEpsilon)
			return std::nullopt;
		const float t = glm::dot(point - ray.Origin, normal) / denom;
		if (t < 0.0f)
			return std::nullopt;
		return t;
	}

	// The Plane form. Two-sided; a degenerate plane is never hit.
	inline std::optional<float> RayPlane(const Ray& ray, const Plane& plane)
	{
		const float denom = detail::Dot(ray.Direction, plane.Normal);
		if (std::abs(denom) < kEpsilon)
			return std::nullopt;
		const float t = (plane.Distance - detail::Dot(plane.Normal, ray.Origin)) / denom;
		if (t < 0.0f)
			return std::nullopt;
		return t;
	}

	// ---- Ray vs AABB (slab) -----------------------------------------------------
	// The slab method (Kay-Kajiya): the overlap of the ray's three per-axis
	// entry/exit intervals. An axis the ray is exactly parallel to (a zero
	// Direction component, either sign) is a containment test of the origin
	// on that axis instead of a division - which gives +-inf, or 0 / 0 = NaN
	// for an origin on the slab's boundary, and would leave the answer to how
	// the min/max below happen to order a NaN. Inclusive. Direction need not
	// be unit (t scales with it).
	inline std::optional<float> RayAABB(const Ray& ray, const AABB& box)
	{
		if (box.IsEmpty())
			return std::nullopt;
		const glm::vec3& o = ray.Origin;
		const glm::vec3& d = ray.Direction;
		if (d.x == 0.0f && d.y == 0.0f && d.z == 0.0f)
			return std::nullopt;
		float tNear = 0.0f; // hits behind the origin do not count
		float tFar = std::numeric_limits<float>::infinity();
		for (int axis = 0; axis < 3; ++axis)
		{
			if (d[axis] == 0.0f)
			{
				if (o[axis] < box.Min[axis] || o[axis] > box.Max[axis])
					return std::nullopt;
				continue;
			}
			float t1 = (box.Min[axis] - o[axis]) / d[axis];
			float t2 = (box.Max[axis] - o[axis]) / d[axis];
			if (t2 < t1)
			{
				const float swap = t1;
				t1 = t2;
				t2 = swap;
			}
			tNear = detail::Max(tNear, t1);
			tFar = detail::Min(tFar, t2);
			if (tNear > tFar)
				return std::nullopt;
		}
		return tNear;
	}

	// ---- Ray vs sphere ----------------------------------------------------------
	// The quadratic |o + t d - c|^2 = r^2 (Direction need not be unit). The
	// origin inside or on the sphere hits at t = 0; a tangent ray hits.
	inline std::optional<float> RaySphere(const Ray& ray, const Sphere& sphere)
	{
		if (sphere.IsEmpty())
			return std::nullopt;
		const float a = detail::Dot(ray.Direction, ray.Direction);
		if (a == 0.0f)
			return std::nullopt;
		const glm::vec3 m = ray.Origin - sphere.Center;
		const float c = detail::Dot(m, m) - sphere.Radius * sphere.Radius;
		if (c <= 0.0f)
			return 0.0f; // the origin is inside or on the surface
		const float b = detail::Dot(m, ray.Direction);
		if (b > 0.0f)
			return std::nullopt; // outside and pointing away
		const float discriminant = b * b - a * c;
		if (discriminant < 0.0f)
			return std::nullopt;
		return (-b - std::sqrt(discriminant)) / a;
	}

	// ---- Ray vs triangle (Moller-Trumbore) --------------------------------------
	// The front face is counter-clockwise a, b, c seen from the ray's side:
	// its normal (b - a) x (c - a) faces the ray origin (Plane::FromPoints's
	// convention). TwoSided hits either face; FrontOnly culls the back face.
	enum class TriangleSides { TwoSided, FrontOnly };

	struct RayTriangleHit
	{
		float T = 0.0f;
		// The barycentric coordinates of the hit: it is
		// (1 - U - V) * a + U * b + V * c.
		float U = 0.0f;
		float V = 0.0f;
	};

	// Inclusive: U >= 0, V >= 0, U + V <= 1, so a ray through an edge or a
	// vertex hits. Parallel - |cos| of the ray to the triangle's normal below
	// kEpsilon, a scale-free test (|det| <= kEpsilon |d| |n|) so a small
	// triangle is not mistaken for a parallel one - misses, as does a
	// degenerate (zero-area) triangle and a zero-direction ray.
	inline std::optional<RayTriangleHit> RayTriangle(const Ray& ray, const glm::vec3& a, const glm::vec3& b,
	                                                 const glm::vec3& c, TriangleSides sides = TriangleSides::TwoSided)
	{
		const glm::vec3 e1 = b - a;
		const glm::vec3 e2 = c - a;
		const glm::vec3 p = detail::Cross(ray.Direction, e2);
		const float det = detail::Dot(e1, p); // = -dot(Direction, (b - a) x (c - a))
		const glm::vec3 n = detail::Cross(e1, e2);
		const float threshold = kEpsilon * kEpsilon * detail::Dot(ray.Direction, ray.Direction) * detail::Dot(n, n);
		if (det * det <= threshold)
			return std::nullopt;
		if (sides == TriangleSides::FrontOnly && det < 0.0f)
			return std::nullopt;
		const float inverseDet = 1.0f / det;
		const glm::vec3 s = ray.Origin - a;
		const float u = detail::Dot(s, p) * inverseDet;
		if (u < 0.0f || u > 1.0f)
			return std::nullopt;
		const glm::vec3 q = detail::Cross(s, e1);
		const float v = detail::Dot(ray.Direction, q) * inverseDet;
		if (v < 0.0f || u + v > 1.0f)
			return std::nullopt;
		const float t = detail::Dot(e2, q) * inverseDet;
		if (t < 0.0f)
			return std::nullopt;
		return RayTriangleHit{ t, u, v };
	}

	// ---- Volumes vs frustum -----------------------------------------------------
	// Outside: wholly outside one plane. Inside: wholly inside every plane.
	// Intersect: anything else. The test is per plane, so it is CONSERVATIVE:
	// a box near a frustum corner - outside the frustum but straddling two of
	// its planes' extensions - reports Intersect, never a false Outside. A
	// volume touching a plane from outside (distance exactly 0) is Intersect.
	enum class Containment { Outside, Intersect, Inside };

	// Centre/extents form: per plane, the box's projected half-size on the
	// normal is r = dot(Extents, |Normal|). The empty box is Outside.
	inline Containment ClassifyAABB(const Frustum& frustum, const AABB& box)
	{
		if (box.IsEmpty())
			return Containment::Outside;
		const glm::vec3 center = box.Center();
		const glm::vec3 extents = box.Extents();
		Containment result = Containment::Inside;
		for (const Plane& plane : frustum.Planes)
		{
			const float s = plane.SignedDistance(center);
			const float r = extents.x * std::abs(plane.Normal.x) + extents.y * std::abs(plane.Normal.y)
				+ extents.z * std::abs(plane.Normal.z);
			if (s + r < 0.0f)
				return Containment::Outside;
			if (s - r < 0.0f)
				result = Containment::Intersect;
		}
		return result;
	}

	// The empty sphere is Outside.
	inline Containment ClassifySphere(const Frustum& frustum, const Sphere& sphere)
	{
		if (sphere.IsEmpty())
			return Containment::Outside;
		Containment result = Containment::Inside;
		for (const Plane& plane : frustum.Planes)
		{
			const float s = plane.SignedDistance(sphere.Center);
			if (s < -sphere.Radius)
				return Containment::Outside;
			if (s < sphere.Radius)
				result = Containment::Intersect;
		}
		return result;
	}

	// ---- AABB vs AABB -----------------------------------------------------------
	// Inclusive (touching faces overlap); an empty box overlaps nothing.
	inline bool Overlaps(const AABB& a, const AABB& b)
	{
		if (a.IsEmpty() || b.IsEmpty())
			return false;
		return a.Min.x <= b.Max.x && b.Min.x <= a.Max.x
			&& a.Min.y <= b.Max.y && b.Min.y <= a.Max.y
			&& a.Min.z <= b.Max.z && b.Min.z <= a.Max.z;
	}

} // namespace tmath
