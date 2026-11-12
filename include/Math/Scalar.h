#pragma once

// The scalar helpers. Pure, header-only, and arithmetic-only (no libm call
// but std::fmod, which IEEE 754 makes exact), so every result here is
// bit-stable across platforms and compilers given the same evaluation order
// and no FMA contraction - these inline into the includer, with its
// -ffp-contract, so on an FMA target Lerp/Remap/SmoothStep can contract
// (Conventions.h, "Determinism"). Everything but the floating Wrap is
// constexpr (std::fmod is not constexpr before C++23).
//
// The floating-point functions take one std::floating_point type T for all
// arguments (float and double; a mixed call is a compile error, not a silent
// promotion). Clamp, Sign and Square take any arithmetic type.

#include "Math/Constants.h"

#include <cmath>
#include <concepts>
#include <limits>
#include <type_traits>

namespace TinyEngine::Math {

	template <typename T>
	concept Arithmetic = std::is_arithmetic_v<T> && !std::is_same_v<T, bool>;

	namespace detail {
		template <std::floating_point T> constexpr T DefaultAbsTolerance() { return std::is_same_v<T, float> ? (T)kEpsilon : (T)kEpsilonD; }
		template <std::floating_point T> constexpr T DefaultRelTolerance() { return std::is_same_v<T, float> ? (T)kRelativeEpsilon : (T)kRelativeEpsilonD; }
		// std::abs is constexpr only from C++23. -0 stays -0 (the callers
		// compare magnitudes, never bits).
		template <std::floating_point T> constexpr T Abs(T x) { return x < T(0) ? -x : x; }
	} // namespace detail

	// ---- Range ------------------------------------------------------------------
	// std::clamp's semantics: lo when x < lo, hi when hi < x, else x (a NaN x
	// passes through). Precondition: lo <= hi (as std::clamp).
	template <Arithmetic T>
	constexpr T Clamp(T x, T lo, T hi)
	{
		return x < lo ? lo : (hi < x ? hi : x);
	}

	// Clamp to [0, 1].
	template <std::floating_point T>
	constexpr T Saturate(T x)
	{
		return Clamp(x, T(0), T(1));
	}

	// x wrapped into the half-open [min, max), periodically: Wrap(-1, 0, 4) == 3,
	// Wrap(4, 0, 4) == 0. An empty or inverted range (max <= min) returns min.
	// The floating form rests on std::fmod (exact), so the only rounding is the
	// x - min subtraction and the final + min. A non-finite x (NaN, +-inf)
	// gives NaN - never a silently in-range value.
	template <std::floating_point T>
	inline T Wrap(T x, T min, T max)
	{
		const T range = max - min;
		if (!(range > T(0)))
			return min;
		T r = std::fmod(x - min, range);
		if (r < T(0))
		{
			r += range;
			// A tiny negative remainder plus range can round up to range
			// itself; the interval is half-open.
			if (r >= range)
				r = T(0);
		}
		const T result = r + min;
		return result >= max ? min : result; // the + min can round up to max
	}

	// The integral form: exact. Signed T: precondition max - min and x - min
	// do not overflow. Unsigned T has no precondition: x below min takes its
	// own branch, because x - min would wrap modulo 2^N (or, for a type
	// narrower than int, promote to a negative int) and land on the wrong
	// residue.
	template <std::integral T>
	constexpr T Wrap(T x, T min, T max)
	{
		if (!(min < max))
			return min;
		const T range = (T)(max - min);
		if constexpr (std::is_signed_v<T>)
		{
			T r = (T)((x - min) % range);
			if (r < T(0))
				r += range;
			return (T)(r + min);
		}
		else
		{
			if (x >= min)
				return (T)(min + (T)((T)(x - min) % range));
			// min - x - 1 is in [0, min); its residue r counts down from
			// max - 1.
			return (T)(min + (T)(range - T(1) - (T)((T)(min - x - T(1)) % range)));
		}
	}

	// ---- Interpolation ----------------------------------------------------------
	// a + (b - a) * t, unclamped (t outside [0, 1] extrapolates). The form is
	// glm::mix's and the shaders' mix(): exact at t == 0, monotonic, but
	// Lerp(a, b, 1) can differ from b by an ulp when a and b differ in
	// magnitude.
	template <std::floating_point T>
	constexpr T Lerp(T a, T b, T t)
	{
		return a + (b - a) * t;
	}

	// The t for which Lerp(a, b, t) == value, unclamped. a == b (no range to
	// measure against) returns 0.
	template <std::floating_point T>
	constexpr T InverseLerp(T a, T b, T value)
	{
		return a == b ? T(0) : (value - a) / (b - a);
	}

	// value mapped from [inMin, inMax] onto [outMin, outMax], unclamped
	// (InverseLerp then Lerp; an empty input range maps to outMin).
	template <std::floating_point T>
	constexpr T Remap(T value, T inMin, T inMax, T outMin, T outMax)
	{
		return Lerp(outMin, outMax, InverseLerp(inMin, inMax, value));
	}

	// ---- Comparison -------------------------------------------------------------
	// |a - b| <= absTolerance (the floor near zero) OR |a - b| <= relTolerance
	// * max(|a|, |b|) (the scale at large magnitudes). Equal values (equal
	// infinities included) are always equal; a NaN never is.
	template <std::floating_point T>
	constexpr bool NearlyEqual(T a, T b, T absTolerance = detail::DefaultAbsTolerance<T>(),
		T relTolerance = detail::DefaultRelTolerance<T>())
	{
		if (a == b)
			return true;
		const T diff = detail::Abs(a - b);
		if (diff <= absTolerance)
			return true;
		const T absA = detail::Abs(a), absB = detail::Abs(b);
		const T largest = absA > absB ? absA : absB;
		// An infinity unequal to the other value (or a NaN) would make the
		// relative bound infinite and accept anything.
		if (!(largest <= std::numeric_limits<T>::max()))
			return false;
		return diff <= relTolerance * largest;
	}

	// |x| <= absTolerance (a NaN is not near zero).
	template <std::floating_point T>
	constexpr bool IsNearlyZero(T x, T absTolerance = detail::DefaultAbsTolerance<T>())
	{
		return detail::Abs(x) <= absTolerance;
	}

	// ---- Easing and small helpers -----------------------------------------------
	// GLSL's smoothstep: the Hermite 3t^2 - 2t^3 of t = Saturate((x - edge0) /
	// (edge1 - edge0)). Where GLSL leaves edge0 == edge1 undefined, this is a
	// step: 0 below the edge, 1 at or above it.
	template <std::floating_point T>
	constexpr T SmoothStep(T edge0, T edge1, T x)
	{
		if (edge0 == edge1)
			return x < edge0 ? T(0) : T(1);
		const T t = Saturate((x - edge0) / (edge1 - edge0));
		return t * t * (T(3) - T(2) * t);
	}

	// -1, 0 or +1 (as T). Both zeros give 0; a NaN gives 0.
	template <Arithmetic T>
	constexpr T Sign(T x)
	{
		return (T)((T(0) < x) - (x < T(0)));
	}

	template <Arithmetic T>
	constexpr T Square(T x)
	{
		return x * x;
	}

} // namespace TinyEngine::Math
