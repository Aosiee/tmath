#pragma once

// Angles. The math API works in radians; degrees appear only in serialized
// and user-facing fields (Conventions.h), and these are the conversions
// between the two. All arithmetic-only: bit-stable given the same
// evaluation order and no FMA contraction (they inline with the includer's
// flags - Conventions.h, "Determinism").

#include "Math/Constants.h"

#include <cmath>
#include <concepts>
#include <type_traits>

namespace TinyEngine::Math {

	// The conversions multiply by the same literal glm::radians/glm::degrees
	// do, so they are bit-identical to glm's (a tinytest case pins it).
	constexpr float DegreesToRadians(float degrees) { return degrees * kDegToRad; }
	constexpr double DegreesToRadians(double degrees) { return degrees * kDegToRadD; }
	constexpr float RadiansToDegrees(float radians) { return radians * kRadToDeg; }
	constexpr double RadiansToDegrees(double radians) { return radians * kRadToDegD; }

	// The angle wrapped into (-pi, pi]: +pi stays +pi, -pi becomes +pi (the
	// one representative of the half-turn; atan2's upper end). "pi" is kPi /
	// kPiD, the nearest float/double to the real pi. Exact: std::fmod is, and
	// the one +-2pi correction after it subtracts values within a factor of
	// two of each other (Sterbenz), so NormalizeAngle(kTwoPi) is exactly 0
	// and NormalizeAngle(-kPi) exactly kPi. A zero result is always +0 (fmod
	// keeps the dividend's sign, so a negative whole turn would give -0; the
	// final + 0 folds it). A non-finite angle gives NaN.
	template <std::floating_point T>
	inline T NormalizeAngle(T radians)
	{
		constexpr T pi = std::is_same_v<T, float> ? (T)kPi : (T)kPiD;
		constexpr T twoPi = std::is_same_v<T, float> ? (T)kTwoPi : (T)kTwoPiD;
		T r = std::fmod(radians, twoPi); // (-2pi, 2pi)
		if (r > pi)
			r -= twoPi;
		else if (r <= -pi)
			r += twoPi;
		return r + T(0);
	}

	// The signed shortest rotation from `from` to `to`, in (-pi, pi]:
	// ShortestAngleDelta(from, to) added to `from` reaches `to` (modulo a full
	// turn) by the short way round, across the +-pi seam included. A
	// half-turn apart is +pi (NormalizeAngle's convention).
	template <std::floating_point T>
	inline T ShortestAngleDelta(T from, T to)
	{
		return NormalizeAngle(to - from);
	}

} // namespace TinyEngine::Math
