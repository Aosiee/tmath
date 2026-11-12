#pragma once

// Math/Scalar.h - known values, the edge cases (Wrap of negatives,
// NearlyEqual near zero and at large magnitudes, InverseLerp with a == b)
// and the round trips.

#include "Math/Scalar.h"

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace TinyEngine::Math::scalar_tests {

// The constexpr surface really is constexpr.
static_assert(Clamp(5, 0, 3) == 3 && Clamp(-2, 0, 3) == 0 && Clamp(2, 0, 3) == 2);
static_assert(Saturate(1.5f) == 1.0f && Saturate(-0.5) == 0.0);
static_assert(Wrap(-1, 0, 4) == 3 && Wrap(9, 0, 4) == 1);
static_assert(Wrap(1u, 2u, 5u) == 4u && Wrap<uint8_t>(1, 2, 5) == 4);
static_assert(Lerp(2.0f, 6.0f, 0.25f) == 3.0f);
static_assert(InverseLerp(2.0, 6.0, 3.0) == 0.25);
static_assert(Remap(5.0f, 0.0f, 10.0f, 100.0f, 200.0f) == 150.0f);
static_assert(NearlyEqual(1.0f, 1.0f + 1e-7f) && !NearlyEqual(1.0f, 1.001f));
static_assert(IsNearlyZero(1e-7f) && !IsNearlyZero(1e-3f));
static_assert(SmoothStep(0.0f, 1.0f, 0.5f) == 0.5f);
static_assert(Sign(-3) == -1 && Sign(0) == 0 && Sign(7u) == 1u && Sign(-2.5f) == -1.0f);
static_assert(Square(-3) == 9 && Square(1.5f) == 2.25f);

TEST_CASE("math.scalar: Clamp and Saturate - std::clamp semantics")
{
	CHECK(Clamp(0.5f, 0.0f, 1.0f) == 0.5f);
	CHECK(Clamp(-1.0f, 0.0f, 1.0f) == 0.0f);
	CHECK(Clamp(2.0f, 0.0f, 1.0f) == 1.0f);
	CHECK(Clamp(3.0f, 3.0f, 3.0f) == 3.0f);
	for (float x : { -2.0f, -0.0f, 0.25f, 1.0f, 7.5f })
		CHECK(Clamp(x, -1.0f, 2.0f) == std::clamp(x, -1.0f, 2.0f));
	CHECK(Clamp<int64_t>(-5, -3, 3) == -3);
	CHECK(Saturate(0.75) == 0.75);
	CHECK(Saturate(-std::numeric_limits<float>::infinity()) == 0.0f);
	CHECK(Saturate(std::numeric_limits<float>::infinity()) == 1.0f);
	// A NaN passes through, as with std::clamp.
	CHECK(std::isnan(Clamp(std::numeric_limits<float>::quiet_NaN(), 0.0f, 1.0f)));
}

TEST_CASE("math.scalar: Wrap - negatives, the half-open upper end, empty ranges")
{
	// Integral: exact, periodic in both directions.
	CHECK(Wrap(0, 0, 4) == 0);
	CHECK(Wrap(4, 0, 4) == 0);
	CHECK(Wrap(-1, 0, 4) == 3);
	CHECK(Wrap(-4, 0, 4) == 0);
	CHECK(Wrap(-5, 0, 4) == 3);
	CHECK(Wrap(13, 10, 13) == 10);
	CHECK(Wrap(-7, -3, 3) == -1);
	CHECK(Wrap(5, 3, 3) == 3); // empty range -> min
	CHECK(Wrap(5, 4, 3) == 4); // inverted range -> min
	// The integral form equals the equirect cook's old (x % w + w) % w.
	for (int32_t x = -9; x <= 9; x++)
		CHECK(Wrap(x, 0, 4) == (x % 4 + 4) % 4);
	// Narrow signed: x - min promotes to int and is cast back.
	CHECK(Wrap<int8_t>(-1, 0, 4) == 3);
	CHECK(Wrap<int8_t>(-128, -3, 3) == -2); // -128 = -2 + 6 * -21
	CHECK(Wrap<int16_t>(-7, -3, 3) == -1);
	// Unsigned: x below min must not wrap modulo 2^N (the review's
	// Wrap(1u, 2u, 5u) returned 2, Wrap<uint8_t>(1, 2, 5) returned 1 - outside
	// [min, max) altogether).
	CHECK(Wrap(1u, 2u, 5u) == 4u);
	CHECK(Wrap(0u, 2u, 5u) == 3u);
	CHECK(Wrap(2u, 2u, 5u) == 2u);
	CHECK(Wrap(5u, 2u, 5u) == 2u);
	CHECK(Wrap(9u, 2u, 5u) == 3u);
	CHECK(Wrap(5u, 3u, 3u) == 3u); // empty range -> min
	CHECK(Wrap(5u, 4u, 3u) == 4u); // inverted range -> min (max - min would wrap)
	CHECK(Wrap<uint8_t>(1, 2, 5) == 4);
	CHECK(Wrap<uint8_t>(0, 250, 255) == 250); // 0 = 250 + 5 * -50
	CHECK(Wrap<uint64_t>(0, 1, std::numeric_limits<uint64_t>::max()) == std::numeric_limits<uint64_t>::max() - 1);
	// Every unsigned result is in [min, max) and congruent to x (mod range) -
	// checked against the signed form on the same values.
	for (uint32_t lo = 0; lo <= 6; lo++)
		for (uint32_t hi = lo + 1; hi <= 9; hi++)
			for (uint32_t x = 0; x <= 20; x++)
			{
				const uint32_t w = Wrap(x, lo, hi);
				CHECK(w >= lo);
				CHECK(w < hi);
				CHECK((int64_t)w == Wrap<int64_t>((int64_t)x, (int64_t)lo, (int64_t)hi));
				CHECK((uint8_t)w == Wrap<uint8_t>((uint8_t)x, (uint8_t)lo, (uint8_t)hi));
			}

	// Floating.
	CHECK(Wrap(0.5f, 0.0f, 1.0f) == 0.5f);
	CHECK(Wrap(1.0f, 0.0f, 1.0f) == 0.0f);
	CHECK(Wrap(-0.25f, 0.0f, 1.0f) == 0.75f);
	CHECK(Wrap(-1.0f, 0.0f, 1.0f) == 0.0f);
	CHECK(Wrap(2.75, 0.0, 1.0) == 0.75);
	CHECK(Wrap(-370.0f, -180.0f, 180.0f) == -10.0f);
	CHECK(Wrap(540.0f, -180.0f, 180.0f) == -180.0f);
	CHECK(Wrap(2.0f, 1.0f, 1.0f) == 1.0f); // empty range -> min
	// A tiny negative remainder plus the range rounds to the range itself;
	// the result stays inside [min, max).
	const float tiny = -1e-9f;
	CHECK(Wrap(tiny, 0.0f, 1.0f) >= 0.0f);
	CHECK(Wrap(tiny, 0.0f, 1.0f) < 1.0f);
	// Non-finite -> NaN (never a silently in-range value).
	CHECK(std::isnan(Wrap(std::numeric_limits<float>::infinity(), 0.0f, 1.0f)));
	CHECK(std::isnan(Wrap(std::numeric_limits<double>::quiet_NaN(), -1.0, 1.0)));
	// Every result is in range across a sweep.
	for (int i = -2000; i <= 2000; i++)
	{
		const float x = (float)i * 0.0137f;
		const float w = Wrap(x, -0.5f, 0.75f);
		CHECK(w >= -0.5f);
		CHECK(w < 0.75f);
	}
}

TEST_CASE("math.scalar: Lerp, InverseLerp, Remap - known values, a == b, round trips")
{
	CHECK(Lerp(1.0f, 3.0f, 0.0f) == 1.0f);
	CHECK(Lerp(1.0f, 3.0f, 1.0f) == 3.0f);
	CHECK(Lerp(1.0f, 3.0f, 0.5f) == 2.0f);
	CHECK(Lerp(1.0f, 3.0f, 2.0f) == 5.0f);  // unclamped
	CHECK(Lerp(1.0f, 3.0f, -1.0f) == -1.0f);
	CHECK(Lerp(-2.0, 2.0, 0.75) == 1.0);
	// The equirect cook's old Mix(a, b, t) = a + (b - a) * t, bit for bit.
	for (float a : { -1.5f, 0.1f, 3.7f })
		for (float b : { 0.3f, -2.25f, 1e3f })
			for (float t : { 0.0f, 0.13f, 0.5f, 0.999f, 1.0f })
				CHECK(Lerp(a, b, t) == a + (b - a) * t);

	CHECK(InverseLerp(2.0f, 6.0f, 2.0f) == 0.0f);
	CHECK(InverseLerp(2.0f, 6.0f, 6.0f) == 1.0f);
	CHECK(InverseLerp(2.0f, 6.0f, 10.0f) == 2.0f); // unclamped
	CHECK(InverseLerp(6.0f, 2.0f, 5.0f) == 0.25f); // a reversed range
	// a == b has no range: 0, never a NaN or an infinity.
	CHECK(InverseLerp(3.0f, 3.0f, 3.0f) == 0.0f);
	CHECK(InverseLerp(3.0f, 3.0f, 100.0f) == 0.0f);
	CHECK(InverseLerp(0.0, 0.0, -1.0) == 0.0);

	CHECK(Remap(0.5f, 0.0f, 1.0f, -10.0f, 10.0f) == 0.0f);
	CHECK(Remap(20.0f, 10.0f, 30.0f, 0.0f, 1.0f) == 0.5f);
	CHECK(Remap(40.0f, 10.0f, 30.0f, 0.0f, 1.0f) == 1.5f); // unclamped
	CHECK(Remap(7.0f, 2.0f, 2.0f, 5.0f, 9.0f) == 5.0f); // empty input range -> outMin

	// Round trips: InverseLerp undoes Lerp, Remap there and back is identity.
	for (float t : { -0.5f, 0.0f, 0.2f, 0.5f, 0.8f, 1.0f, 1.7f })
	{
		CHECK(InverseLerp(-3.0f, 5.0f, Lerp(-3.0f, 5.0f, t)) == doctest::Approx(t).epsilon(1e-6));
		const float there = Remap(t, 0.0f, 1.0f, 100.0f, 300.0f);
		CHECK(Remap(there, 100.0f, 300.0f, 0.0f, 1.0f) == doctest::Approx(t).epsilon(1e-6));
	}
}

TEST_CASE("math.scalar: NearlyEqual - the absolute floor near zero, the relative scale at large magnitudes")
{
	// Near zero the relative test alone would fail (it scales to nothing);
	// the absolute floor catches it.
	CHECK(NearlyEqual(0.0f, 1e-7f));
	CHECK(NearlyEqual(1e-7f, -1e-7f));
	CHECK(NearlyEqual(0.0f, -0.0f));
	CHECK_FALSE(NearlyEqual(0.0f, 1e-5f));
	CHECK(NearlyEqual(0.0f, 1e-5f, 1e-4f)); // a wider floor
	// At large magnitudes the absolute floor is far below one ulp; the
	// relative scale takes over.
	CHECK(NearlyEqual(1e8f, 1e8f + 512.0f));
	CHECK_FALSE(NearlyEqual(1e8f, 1e8f + 4096.0f));
	CHECK(NearlyEqual(1e30f, std::nextafter(1e30f, 2e30f)));
	CHECK_FALSE(NearlyEqual(1e30f, 1.001e30f));
	CHECK(NearlyEqual(-1e8f, -1e8f - 512.0f));
	CHECK_FALSE(NearlyEqual(1e8f, -1e8f));
	// Double.
	CHECK(NearlyEqual(0.1 + 0.2, 0.3));
	CHECK(NearlyEqual(0.0, 1e-13));
	CHECK_FALSE(NearlyEqual(0.0, 1e-11));
	CHECK(NearlyEqual(1e15, 1e15 + 1.0));
	CHECK_FALSE(NearlyEqual(1e15, 1e15 + 1e4));
	// The specials: equal infinities match, a NaN never does.
	const float inf = std::numeric_limits<float>::infinity();
	const float nan = std::numeric_limits<float>::quiet_NaN();
	CHECK(NearlyEqual(inf, inf));
	CHECK_FALSE(NearlyEqual(inf, -inf));
	CHECK_FALSE(NearlyEqual(inf, 1e38f));
	CHECK_FALSE(NearlyEqual(nan, nan));
	CHECK_FALSE(NearlyEqual(nan, 0.0f));
	// Symmetric.
	CHECK(NearlyEqual(1.0f, 1.000005f) == NearlyEqual(1.000005f, 1.0f));
}

TEST_CASE("math.scalar: IsNearlyZero")
{
	CHECK(IsNearlyZero(0.0f));
	CHECK(IsNearlyZero(-0.0f));
	CHECK(IsNearlyZero(1e-6f));
	CHECK(IsNearlyZero(-1e-6f));
	CHECK_FALSE(IsNearlyZero(2e-6f));
	CHECK(IsNearlyZero(0.01f, 0.1f));
	CHECK(IsNearlyZero(1e-13));
	CHECK_FALSE(IsNearlyZero(1e-11));
	CHECK_FALSE(IsNearlyZero(std::numeric_limits<float>::quiet_NaN()));
}

TEST_CASE("math.scalar: SmoothStep - GLSL's Hermite, clamped ends, the degenerate edge")
{
	CHECK(SmoothStep(0.0f, 1.0f, 0.0f) == 0.0f);
	CHECK(SmoothStep(0.0f, 1.0f, 1.0f) == 1.0f);
	CHECK(SmoothStep(0.0f, 1.0f, 0.5f) == 0.5f);
	CHECK(SmoothStep(0.0f, 1.0f, 0.25f) == doctest::Approx(0.15625f));
	CHECK(SmoothStep(0.0f, 1.0f, -3.0f) == 0.0f);
	CHECK(SmoothStep(0.0f, 1.0f, 3.0f) == 1.0f);
	CHECK(SmoothStep(2.0, 4.0, 3.0) == 0.5);
	// Reversed edges invert the curve (as GLSL's formula does).
	CHECK(SmoothStep(1.0f, 0.0f, 0.25f) == doctest::Approx(1.0f - 0.15625f));
	// edge0 == edge1: a step.
	CHECK(SmoothStep(1.0f, 1.0f, 0.999f) == 0.0f);
	CHECK(SmoothStep(1.0f, 1.0f, 1.0f) == 1.0f);
	CHECK(SmoothStep(1.0f, 1.0f, 5.0f) == 1.0f);
	// Monotonic on [edge0, edge1].
	float previous = 0.0f;
	for (int i = 0; i <= 100; i++)
	{
		const float s = SmoothStep(0.0f, 1.0f, (float)i / 100.0f);
		CHECK(s >= previous);
		previous = s;
	}
}

TEST_CASE("math.scalar: Sign and Square")
{
	CHECK(Sign(5.0f) == 1.0f);
	CHECK(Sign(-0.001f) == -1.0f);
	CHECK(Sign(0.0f) == 0.0f);
	CHECK(Sign(-0.0f) == 0.0f);
	CHECK(Sign(std::numeric_limits<float>::quiet_NaN()) == 0.0f);
	CHECK(Sign(-std::numeric_limits<double>::infinity()) == -1.0);
	CHECK(Sign<int64_t>(-9) == -1);
	CHECK(Sign(0u) == 0u);
	CHECK(Square(3.0f) == 9.0f);
	CHECK(Square(-0.5) == 0.25);
	CHECK(Square(-12) == 144);
}

} // namespace TinyEngine::Math::scalar_tests
