#pragma once

// A world transform with a double position and a float basis (rotation x
// scale). Only positions need double precision: rotation, scale and
// directions stay float, as in Unreal 5's large world coordinates and in both
// physics backends. The GPU never sees a double - ToRelative takes the
// translation relative to a render origin in double, then rounds once.
//
// Composition runs the parent's basis in double over the child's local
// position, so a deep hierarchy far from the origin keeps the child offsets
// exact (to the double).

#include "Math/Transform.h"

#include <glm/glm.hpp>

namespace TinyEngine::Math {

	struct WorldTransform
	{
		glm::dvec3 Position{ 0.0 };
		glm::mat3 Basis{ 1.0f }; // rotation * scale (columns: the scaled axes)

		// The engine's TRS (ComposeTransform's convention: T * R * S, Euler
		// degrees applied X->Y->Z, Math/Transform.h).
		static WorldTransform FromLocal(const glm::dvec3& position, const glm::vec3& eulerDegrees, const glm::vec3& scale)
		{
			WorldTransform out;
			out.Position = position;
			out.Basis = glm::mat3(EulerDegreesToMatrix(eulerDegrees)) * glm::mat3(glm::scale(glm::mat4(1.0f), scale));
			return out;
		}

		// this * child: the child expressed in this transform's space.
		WorldTransform operator*(const WorldTransform& child) const
		{
			WorldTransform out;
			out.Position = Position + glm::dmat3(Basis) * child.Position;
			out.Basis = Basis * child.Basis;
			return out;
		}

		// Its inverse (the basis must be invertible: no zero scale axis).
		WorldTransform Inverse() const
		{
			WorldTransform out;
			out.Basis = glm::inverse(Basis);
			out.Position = -(glm::dmat3(out.Basis) * Position);
			return out;
		}

		glm::dvec3 TransformPoint(const glm::dvec3& local) const { return Position + glm::dmat3(Basis) * local; }
		glm::vec3 TransformDirection(const glm::vec3& direction) const { return Basis * direction; }

		// The float 4x4 the GPU draws with: the translation is taken relative
		// to `origin` in double (exact for any representable pair), then
		// rounded once to float.
		glm::mat4 ToRelative(const glm::dvec3& origin) const
		{
			glm::mat4 out(Basis);
			out[3] = glm::vec4(glm::vec3(Position - origin), 1.0f);
			return out;
		}

		// Its TRS in the engine's convention (DecomposeTransform's rules for
		// the basis: positive scales, the gimbal-lock fold).
		void Decompose(glm::dvec3& position, glm::vec3& eulerDegrees, glm::vec3& scale) const
		{
			glm::vec3 unused;
			DecomposeTransform(glm::mat4(Basis), unused, eulerDegrees, scale);
			position = Position;
		}
	};

} // namespace TinyEngine::Math
