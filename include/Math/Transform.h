#pragma once

// Transforms - the engine's Euler convention, the
// TRS composition/decomposition pair (moved here from the Transform
// component, which keeps calling them), and a TRS value with quaternion
// rotation.
//
// The Euler convention (TransformComponent::Rotation, gizmo's snapshots):
// degrees, applied as R = Rx(x) * Ry(y) * Rz(z) on column vectors, so Z
// rotates first and X last. The extraction (RotationToEulerDegrees) returns
// y in [-90, 90] and x, z in (-180, 180]. At gimbal lock (|y| = 90, cos(y)
// below 1e-6) x and z rotate about the same axis; the extraction folds the
// pair into x and returns z = 0: Rx(a) Ry(+90) Rz(c) == Rx(a + c) Ry(+90),
// Rx(a) Ry(-90) Rz(c) == Rx(a - c) Ry(-90). That reproduces the matrix, not
// the original angles.
//
// glm::quat(vec3) is NOT this convention: its Euler constructor takes
// radians and builds Rz * Ry * Rx (X first). EditorCamera's orientation
// uses it on purpose (yaw/pitch, radians); that one stays a glm call.
//
// Determinism: these call std::sin/cos/asin/atan2 (through glm::rotate and
// the extraction), so they follow the platform libm (Conventions.h).

#include "Math/Angle.h"

#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace TinyEngine::Math {

	// The rotation part of the convention as a 4x4 (the exact product
	// ComposeTransform has always built: three glm::rotate matrices).
	inline glm::mat4 EulerDegreesToMatrix(const glm::vec3& eulerDegrees)
	{
		return glm::rotate(glm::mat4(1.0f), glm::radians(eulerDegrees.x), { 1.0f, 0.0f, 0.0f })
			* glm::rotate(glm::mat4(1.0f), glm::radians(eulerDegrees.y), { 0.0f, 1.0f, 0.0f })
			* glm::rotate(glm::mat4(1.0f), glm::radians(eulerDegrees.z), { 0.0f, 0.0f, 1.0f });
	}

	// The Euler angles (degrees) of a pure rotation matrix, in the
	// convention above (the gimbal-lock fold included). The input must be
	// orthonormal with determinant +1; DecomposeTransform normalises the
	// scale out first.
	inline glm::vec3 RotationToEulerDegrees(const glm::mat3& rot)
	{
		float sy = glm::clamp(rot[2][0], -1.0f, 1.0f);
		float y = std::asin(sy);
		float cy = std::cos(y);

		float x, z;
		if (std::abs(cy) > 1e-6f)
		{
			x = std::atan2(-rot[2][1], rot[2][2]);
			z = std::atan2(-rot[1][0], rot[0][0]);
		}
		else
		{
			x = std::atan2(rot[1][2], rot[1][1]);
			z = 0.0f;
		}
		return glm::degrees(glm::vec3(x, y, z));
	}

	// The same rotation as EulerDegreesToMatrix, as a unit quaternion:
	// q = qx * qy * qz (so q rotates v as Rx Ry Rz v).
	inline glm::quat EulerDegreesToQuat(const glm::vec3& eulerDegrees)
	{
		return glm::angleAxis(glm::radians(eulerDegrees.x), glm::vec3(1.0f, 0.0f, 0.0f))
			* glm::angleAxis(glm::radians(eulerDegrees.y), glm::vec3(0.0f, 1.0f, 0.0f))
			* glm::angleAxis(glm::radians(eulerDegrees.z), glm::vec3(0.0f, 0.0f, 1.0f));
	}

	// The convention's Euler angles (degrees) of a unit quaternion (through
	// its rotation matrix, so the extraction rules are RotationToEulerDegrees').
	inline glm::vec3 QuatToEulerDegrees(const glm::quat& rotation)
	{
		return RotationToEulerDegrees(glm::mat3_cast(rotation));
	}

	// The engine's TRS composition: T * R(euler) * S. TransformComponent::
	// GetTransform, TransformSystem's world-preserving reparent and the
	// editor's gizmo commit all go through this pair, so world <-> local is
	// one implementation.
	inline glm::mat4 ComposeTransform(const glm::vec3& position, const glm::vec3& eulerDegrees, const glm::vec3& scale)
	{
		return glm::translate(glm::mat4(1.0f), position) * EulerDegreesToMatrix(eulerDegrees) * glm::scale(glm::mat4(1.0f), scale);
	}

	// The inverse of ComposeTransform for an affine T * R * S matrix. The
	// scale is the column lengths, so it is never negative: a matrix built
	// with an even number of negative scale axes decomposes to positive
	// scales and a rotation that absorbs the pair (it recomposes to the same
	// matrix), but one with an odd number (a reflection, determinant < 0) is
	// not representable here - the "rotation" left after the scale is
	// improper, and the result does not recompose to the input. A zero scale
	// axis leaves that rotation column as the identity's, so unless the
	// rotation is the identity the Euler angles do not match the other two
	// columns (kept as moved: behaviour-identical). TRS::FromMatrix handles
	// the reflection and the zero-axis cases.
	inline void DecomposeTransform(const glm::mat4& m, glm::vec3& position, glm::vec3& eulerDegrees, glm::vec3& scale)
	{
		position = glm::vec3(m[3]);
		scale = {
			glm::length(glm::vec3(m[0])),
			glm::length(glm::vec3(m[1])),
			glm::length(glm::vec3(m[2])),
		};

		glm::mat3 rot;
		for (int c = 0; c < 3; ++c)
		{
			for (int r = 0; r < 3; ++r)
				rot[c][r] = (scale[c] > 1e-8f) ? m[c][r] / scale[c] : (r == c ? 1.0f : 0.0f);
		}

		eulerDegrees = RotationToEulerDegrees(rot);
	}

	// Translation, rotation (a unit quaternion) and scale: the TRS form with
	// no Euler angles in it (no gimbal lock, no angle wrap). ToMatrix is
	// T * R * S, the same order as ComposeTransform.
	struct TRS
	{
		glm::vec3 Translation{ 0.0f };
		glm::quat Rotation = glm::quat::wxyz(1.0f, 0.0f, 0.0f, 0.0f); // the identity (wxyz: unambiguous under GLM_FORCE_QUAT_DATA_XYZW)
		glm::vec3 Scale{ 1.0f };

		glm::mat4 ToMatrix() const
		{
			return glm::translate(glm::mat4(1.0f), Translation) * glm::mat4_cast(Rotation) * glm::scale(glm::mat4(1.0f), Scale);
		}

		static TRS FromEulerDegrees(const glm::vec3& translation, const glm::vec3& eulerDegrees, const glm::vec3& scale)
		{
			return TRS{ translation, EulerDegreesToQuat(eulerDegrees), scale };
		}

		glm::vec3 EulerDegrees() const { return QuatToEulerDegrees(Rotation); }

		// The decomposition of an affine T * R * S matrix. Unlike
		// DecomposeTransform, a reflection (determinant < 0) is kept: the
		// X scale goes negative and the rotation stays proper, so ToMatrix
		// reproduces the input (up to rounding). Which axis carries the sign
		// is a choice - a matrix built with a negative Y scale comes back as
		// a negative X scale and a rotation that includes a half turn.
		// A zero scale axis carries no direction, so its rotation column is
		// rebuilt to complete a proper rotation around the remaining ones
		// (one zero axis: the cross product of the other two, which keeps
		// them exactly; two: any perpendicular pair; three: the identity).
		// ToMatrix then reproduces the input, since the rebuilt columns are
		// scaled by zero; the rotation's component about a lone surviving
		// axis is not recoverable and is a choice.
		static TRS FromMatrix(const glm::mat4& m)
		{
			TRS out;
			out.Translation = glm::vec3(m[3]);
			out.Scale = {
				glm::length(glm::vec3(m[0])),
				glm::length(glm::vec3(m[1])),
				glm::length(glm::vec3(m[2])),
			};
			if (glm::determinant(glm::mat3(m)) < 0.0f)
				out.Scale.x = -out.Scale.x;

			glm::mat3 rot(1.0f);
			bool valid[3];
			int validCount = 0;
			for (int c = 0; c < 3; ++c)
			{
				valid[c] = std::abs(out.Scale[c]) > 1e-8f;
				if (valid[c])
				{
					rot[c] = glm::vec3(m[c]) / out.Scale[c];
					++validCount;
				}
			}
			// A proper rotation's columns are cyclic: c[i] = c[i+1] x c[i+2].
			if (validCount == 2)
			{
				const int k = !valid[0] ? 0 : (!valid[1] ? 1 : 2);
				rot[k] = glm::cross(rot[(k + 1) % 3], rot[(k + 2) % 3]);
			}
			else if (validCount == 1)
			{
				const int i = valid[0] ? 0 : (valid[1] ? 1 : 2);
				const glm::vec3 u = rot[i];
				const glm::vec3 a = std::abs(u.x) <= std::abs(u.y) && std::abs(u.x) <= std::abs(u.z)
					? glm::vec3(1.0f, 0.0f, 0.0f)
					: (std::abs(u.y) <= std::abs(u.z) ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(0.0f, 0.0f, 1.0f));
				rot[(i + 1) % 3] = glm::normalize(glm::cross(a, u));
				rot[(i + 2) % 3] = glm::cross(u, rot[(i + 1) % 3]);
			}
			out.Rotation = glm::normalize(glm::quat_cast(rot));
			return out;
		}
	};

} // namespace TinyEngine::Math
