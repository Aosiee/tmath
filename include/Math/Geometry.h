#pragma once

// The geometry primitives - Ray, Plane, AABB, Sphere and Frustum. Plain
// values: no state beyond their fields, no allocation. The tests against
// each other live in Intersect.h.
//
// Determinism (Conventions.h): everything here is arithmetic plus std::sqrt
// (IEEE 754 makes it correctly rounded), so it is bit-stable given the same
// evaluation order and no FMA contraction. These inline with the includer's
// flags.

#include "Math/Constants.h"
#include "Math/Conventions.h"

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <array>
#include <cmath>
#include <limits>

namespace TinyEngine::Math {

	namespace detail {
		constexpr float Min(float a, float b) { return b < a ? b : a; }
		constexpr float Max(float a, float b) { return a < b ? b : a; }

		// v / |v|, or the zero vector when |v| is zero (glm::normalize would
		// divide by zero and give NaNs). |v|^2 is formed directly, so a
		// component beyond about 1e19 (overflow) or below about 1e-22
		// (underflow) also reads as zero.
		inline glm::vec3 NormalizeOrZero(const glm::vec3& v)
		{
			const float lengthSquared = v.x * v.x + v.y * v.y + v.z * v.z;
			if (!(lengthSquared > 0.0f))
				return glm::vec3(0.0f);
			return v / std::sqrt(lengthSquared);
		}
	} // namespace detail

	// ---- Ray --------------------------------------------------------------------
	// Origin + t * Direction for t >= 0. Direction is unit length by
	// convention (the intersect functions state what they assume). Moved down
	// from gizmo (gizmo::Ray is now an alias of this), with gizmo's default
	// direction: kForward.
	struct Ray
	{
		glm::vec3 Origin{ 0.0f };
		glm::vec3 Direction{ kForward };

		glm::vec3 At(float t) const { return Origin + Direction * t; }
	};

	// ---- Plane ------------------------------------------------------------------
	// The points p with dot(Normal, p) == Distance: Normal is unit length and
	// Distance is the plane's signed distance from the origin along it.
	// SignedDistance is positive on the side Normal points to. A degenerate
	// plane (from a zero normal or three collinear points) has a zero Normal
	// and Distance 0: every SignedDistance is 0 and no ray hits it. The inputs
	// must be well inside float range: FromPoints squares the edge cross
	// product, so edges beyond about 1e9 or below about 1e-11 also give a
	// degenerate plane (and RayTriangle's threshold has the same range).
	struct Plane
	{
		glm::vec3 Normal{ kUp };
		float Distance = 0.0f;

		static Plane FromPointNormal(const glm::vec3& point, const glm::vec3& normal)
		{
			const glm::vec3 n = detail::NormalizeOrZero(normal);
			return { n, n.x * point.x + n.y * point.y + n.z * point.z };
		}

		// The normal is (b - a) x (c - a): counter-clockwise a, b, c seen from
		// the side the normal points to (right-handed).
		static Plane FromPoints(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c)
		{
			const glm::vec3 e1 = b - a;
			const glm::vec3 e2 = c - a;
			const glm::vec3 n(e1.y * e2.z - e1.z * e2.y, e1.z * e2.x - e1.x * e2.z, e1.x * e2.y - e1.y * e2.x);
			return FromPointNormal(a, n);
		}

		// The plane a*x + b*y + c*z + d = 0 of the coefficients (a, b, c, d),
		// normalised (the positive side is where the expression is positive).
		static Plane FromCoefficients(const glm::vec4& abcd)
		{
			const glm::vec3 n(abcd.x, abcd.y, abcd.z);
			const float lengthSquared = n.x * n.x + n.y * n.y + n.z * n.z;
			if (!(lengthSquared > 0.0f))
				return { glm::vec3(0.0f), 0.0f };
			const float inverseLength = 1.0f / std::sqrt(lengthSquared);
			return { n * inverseLength, -abcd.w * inverseLength };
		}

		bool IsDegenerate() const { return Normal.x == 0.0f && Normal.y == 0.0f && Normal.z == 0.0f; }

		float SignedDistance(const glm::vec3& p) const
		{
			return Normal.x * p.x + Normal.y * p.y + Normal.z * p.z - Distance;
		}

		glm::vec3 ClosestPoint(const glm::vec3& p) const { return p - Normal * SignedDistance(p); }
	};

	// ---- AABB -------------------------------------------------------------------
	// An axis-aligned box, Min..Max inclusive. The default-constructed box is
	// the EMPTY box (Min = +inf, Max = -inf): it contains nothing, Expand
	// grows it to exactly what it is given, and the intersect functions treat
	// it as missing every test. A box with Min == Max on some axis (a point,
	// a flat box) is not empty. Center/Extents/Size of the empty box are zero.
	struct AABB
	{
		glm::vec3 Min{ std::numeric_limits<float>::infinity() };
		glm::vec3 Max{ -std::numeric_limits<float>::infinity() };

		static constexpr AABB Empty() { return {}; }
		// Min/Max taken per component, so the corners may come in any order.
		static AABB FromCorners(const glm::vec3& a, const glm::vec3& b)
		{
			return { { detail::Min(a.x, b.x), detail::Min(a.y, b.y), detail::Min(a.z, b.z) },
			         { detail::Max(a.x, b.x), detail::Max(a.y, b.y), detail::Max(a.z, b.z) } };
		}
		// Extents are the half-size; a negative extent is taken as its magnitude.
		static AABB FromCenterExtents(const glm::vec3& center, const glm::vec3& extents)
		{
			return FromCorners(center - extents, center + extents);
		}

		// Empty when any axis has Min > Max (or a NaN).
		bool IsEmpty() const { return !(Min.x <= Max.x && Min.y <= Max.y && Min.z <= Max.z); }

		void Expand(const glm::vec3& p)
		{
			Min = { detail::Min(Min.x, p.x), detail::Min(Min.y, p.y), detail::Min(Min.z, p.z) };
			Max = { detail::Max(Max.x, p.x), detail::Max(Max.y, p.y), detail::Max(Max.z, p.z) };
		}
		// Expanding by the empty box is a no-op.
		void Expand(const AABB& other)
		{
			if (other.IsEmpty())
				return;
			Expand(other.Min);
			Expand(other.Max);
		}

		glm::vec3 Center() const { return IsEmpty() ? glm::vec3(0.0f) : (Min + Max) * 0.5f; }
		glm::vec3 Extents() const { return IsEmpty() ? glm::vec3(0.0f) : (Max - Min) * 0.5f; }
		glm::vec3 Size() const { return IsEmpty() ? glm::vec3(0.0f) : Max - Min; }

		// Inclusive: a point on a face is inside.
		bool Contains(const glm::vec3& p) const
		{
			return Min.x <= p.x && p.x <= Max.x && Min.y <= p.y && p.y <= Max.y && Min.z <= p.z && p.z <= Max.z;
		}
	};

	// The box bounding `box` transformed by the AFFINE matrix `m` (its last
	// row is ignored and taken as 0, 0, 0, 1 - a projective matrix needs the
	// 8 corners divided by w instead). Arvo's method ("Transforming
	// axis-aligned bounding boxes", Graphics Gems, 1990): each output axis
	// starts at the translation and adds, per input axis, the smaller and the
	// larger of m_ij * Min_j and m_ij * Max_j. That is exactly the min and max
	// of the 8 transformed corners, in 18 multiplies instead of 8 matrix
	// products. The empty box stays empty. Min and Max must be finite
	// otherwise: an unbounded axis gives 0 * inf = NaN on the other axes.
	inline AABB TransformAABB(const AABB& box, const glm::mat4& m)
	{
		if (box.IsEmpty())
			return box;
		AABB out;
		for (int i = 0; i < 3; ++i) // output axis (glm is column-major: m[column][row])
		{
			float lo = m[3][i];
			float hi = m[3][i];
			for (int j = 0; j < 3; ++j) // input axis
			{
				const float a = m[j][i] * box.Min[j];
				const float b = m[j][i] * box.Max[j];
				lo += detail::Min(a, b);
				hi += detail::Max(a, b);
			}
			out.Min[i] = lo;
			out.Max[i] = hi;
		}
		return out;
	}

	// ---- Sphere -----------------------------------------------------------------
	// Center and Radius (>= 0; a zero radius is a point). A negative radius
	// is the empty sphere: it contains nothing and misses every test.
	struct Sphere
	{
		glm::vec3 Center{ 0.0f };
		float Radius = 0.0f;

		bool IsEmpty() const { return !(Radius >= 0.0f); }

		// Inclusive: a point on the surface is inside.
		bool Contains(const glm::vec3& p) const
		{
			const glm::vec3 d = p - Center;
			return !IsEmpty() && d.x * d.x + d.y * d.y + d.z * d.z <= Radius * Radius;
		}
	};

	// ---- Frustum ----------------------------------------------------------------
	// Six planes, normals pointing INTO the frustum: a point is inside when
	// every SignedDistance is >= 0. The order is FrustumPlane's.
	enum class FrustumPlane : int { Left = 0, Right = 1, Bottom = 2, Top = 3, Near = 4, Far = 5 };

	// A default-constructed Frustum is meaningless (six copies of Plane's
	// default, i.e. the half-space y >= 0) until FromViewProjection fills it.
	struct Frustum
	{
		std::array<Plane, 6> Planes{};

		const Plane& Get(FrustumPlane which) const { return Planes[(size_t)which]; }

		// The Gribb-Hartmann extraction from a view-projection (any
		// world -> clip matrix; a projection alone gives the view-space
		// frustum). The matrix MUST use the [0, 1] clip-depth convention
		// (Conventions.h: Vulkan's - glm's *_ZO projections): Near is the
		// plane z_clip >= 0. Every engine projection is [0, 1] (Projection.h,
		// the glm plugin's GLM_FORCE_DEPTH_ZERO_TO_ONE); given an explicit
		// [-1, 1] matrix (glm's *_NO forms), the extracted Near plane sits
		// where z_ndc = 0 (about twice the camera's near distance), not at
		// the configured near. Nothing in the renderer uses this yet (frustum
		// culling is a later renderer item).
		//
		// Left/Right/Bottom/Top are clip-space -x/+x/-y/+y. The planes are
		// normalised; a degenerate matrix gives degenerate planes
		// (Plane::FromCoefficients).
		static Frustum FromViewProjection(const glm::mat4& viewProjection)
		{
			const glm::mat4& m = viewProjection;
			const glm::vec4 row0(m[0][0], m[1][0], m[2][0], m[3][0]);
			const glm::vec4 row1(m[0][1], m[1][1], m[2][1], m[3][1]);
			const glm::vec4 row2(m[0][2], m[1][2], m[2][2], m[3][2]);
			const glm::vec4 row3(m[0][3], m[1][3], m[2][3], m[3][3]);
			Frustum f;
			f.Planes[(size_t)FrustumPlane::Left] = Plane::FromCoefficients(row3 + row0);   // -w <= x
			f.Planes[(size_t)FrustumPlane::Right] = Plane::FromCoefficients(row3 - row0);  //  x <= w
			f.Planes[(size_t)FrustumPlane::Bottom] = Plane::FromCoefficients(row3 + row1); // -w <= y
			f.Planes[(size_t)FrustumPlane::Top] = Plane::FromCoefficients(row3 - row1);    //  y <= w
			f.Planes[(size_t)FrustumPlane::Near] = Plane::FromCoefficients(row2);          //  0 <= z
			f.Planes[(size_t)FrustumPlane::Far] = Plane::FromCoefficients(row3 - row2);    //  z <= w
			return f;
		}

		// Inclusive: a point on a plane is inside.
		bool Contains(const glm::vec3& p) const
		{
			for (const Plane& plane : Planes)
				if (plane.SignedDistance(p) < 0.0f)
					return false;
			return true;
		}
	};

} // namespace TinyEngine::Math
