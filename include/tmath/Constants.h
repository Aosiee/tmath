#pragma once

// The math constants. Every value is a constexpr
// literal (no run-time initialisation, no storage that could be written),
// in float and - with the `D` suffix - double. The float literals are the
// correctly-rounded floats of the exact values (kPi is 0x40490FDB, the
// same bits as glm::pi<float>()).

namespace tmath {

	// ---- pi ---------------------------------------------------------------------
	inline constexpr float kPi = 3.14159265358979323846f;
	inline constexpr float kTwoPi = 6.28318530717958647692f;
	inline constexpr float kHalfPi = 1.57079632679489661923f;
	inline constexpr float kInvPi = 0.318309886183790671538f;

	inline constexpr double kPiD = 3.14159265358979323846;
	inline constexpr double kTwoPiD = 6.28318530717958647692;
	inline constexpr double kHalfPiD = 1.57079632679489661923;
	inline constexpr double kInvPiD = 0.318309886183790671538;

	// The degree <-> radian factors (the same literals glm::radians and
	// glm::degrees use, so Angle.h's conversions are bit-identical to them).
	inline constexpr float kDegToRad = 0.01745329251994329576923690768489f;
	inline constexpr float kRadToDeg = 57.295779513082320876798154814105f;
	inline constexpr double kDegToRadD = 0.01745329251994329576923690768489;
	inline constexpr double kRadToDegD = 57.295779513082320876798154814105;

	// ---- Tolerances -------------------------------------------------------------
	// The default ABSOLUTE tolerance (NearlyEqual, IsNearlyZero): the
	// comparison floor near zero, where a relative tolerance shrinks to
	// nothing.
	inline constexpr float kEpsilon = 1e-6f;
	inline constexpr double kEpsilonD = 1e-12;
	// The default RELATIVE tolerance (NearlyEqual): a fraction of the larger
	// magnitude - about 84 float ulps (float epsilon is 1.19e-7), about
	// 4500 double ulps.
	inline constexpr float kRelativeEpsilon = 1e-5f;
	inline constexpr double kRelativeEpsilonD = 1e-12;

	// Doubling/halving by a power of two is exact, so these hold bit for bit.
	static_assert(kTwoPi == 2.0f * kPi && kHalfPi == 0.5f * kPi);
	static_assert(kTwoPiD == 2.0 * kPiD && kHalfPiD == 0.5 * kPiD);
	static_assert((float)kPiD == kPi && (float)kInvPiD == kInvPi);

} // namespace tmath
