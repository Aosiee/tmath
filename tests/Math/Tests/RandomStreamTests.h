#pragma once

// Math::RandomStream's determinism contract. The pinned outputs come from
// an independent reference implementation of xoshiro256** + splitmix64 (the
// published algorithm; the {1,2,3,4} state's first outputs are the
// reference implementation's own test vector) - a change that moves any of
// them breaks every recorded replay and seed.

#include "Math/RandomStream.h"

#include <doctest/doctest.h>

#include <array>
#include <cmath>
#include <cstdint>

namespace TinyEngine::Math::random_stream_tests {

TEST_CASE("math: RandomStream - the xoshiro256** reference vector (state 1,2,3,4)")
{
	RandomStream stream;
	const RandomStream::State state = { 1, 2, 3, 4 };
	stream.SetState(state);
	const uint64_t expected[] = { 11520ull, 0ull, 1509978240ull, 1215971899390074240ull,
		1216172134540287360ull, 607988272756665600ull };
	for (uint64_t value : expected)
		CHECK(stream.UInt64() == value);
}

TEST_CASE("math: RandomStream - a fixed seed pins the sequence (splitmix64 seeding)")
{
	RandomStream stream(12345);
	CHECK(stream.GetSeed() == 12345);
	const uint64_t expected[] = { 0xbe6a36374160d49bull, 0x214aaa0637a688c6ull, 0xf69d16de9954d388ull,
		0x0c60048c4e96e033ull, 0x8e2076aeed51c648ull, 0x02bbcc1c1fc50f84ull };
	for (uint64_t value : expected)
		CHECK(stream.UInt64() == value);

	// Reseeding replays it; a copy continues the same sequence independently.
	stream.Seed(12345);
	CHECK(stream.UInt64() == expected[0]);
	RandomStream copy = stream;
	CHECK(copy.UInt64() == expected[1]);
	CHECK(stream.UInt64() == expected[1]);

	// The default stream is seed 0, not entropy.
	RandomStream a, b(0);
	CHECK(a.UInt64() == b.UInt64());
}

TEST_CASE("math: RandomStream - the distributions are pinned for a fixed seed")
{
	{
		RandomStream stream(42);
		const uint32_t expected[] = { 0x15780b2eu, 0x6104d986u, 0xae175332u, 0xecb8ad47u };
		for (uint32_t value : expected)
			CHECK(stream.UInt32() == value);
	}
	{
		RandomStream stream(42);
		const uint32_t expected[] = { 0, 3, 6, 9, 9, 7, 7, 8, 7, 5, 6, 2 };
		for (uint32_t value : expected)
			CHECK(stream.UInt32Below(10) == value);
	}
	{
		RandomStream stream(42);
		const float expected[] = { 0x1.5780bp-4f, 0x1.841364p-2f, 0x1.5c2ea6p-1f, 0x1.d9715ap-1f };
		for (float value : expected)
			CHECK(stream.Float() == value);
	}
	{
		RandomStream stream(42);
		const double expected[] = { 0x1.5780b2e0c2ec0p-4, 0x1.84136619b444ep-2, 0x1.5c2ea66473c93p-1 };
		for (double value : expected)
			CHECK(stream.Double() == value);
	}
	{
		RandomStream stream(7);
		const int32_t expected[] = { 2, -2, 4, 5, 5, 4, -5, -4, -1, -4 };
		for (int32_t value : expected)
			CHECK(stream.IntInRange(-5, 5) == value);
	}
	{
		RandomStream stream(9);
		const uint64_t expected[] = { 2583439685ull, 251489372425ull, 132462250118ull, 732694425399ull };
		for (uint64_t value : expected)
			CHECK(stream.UInt64Below(1000000000039ull) == value);
	}
}

TEST_CASE("math: RandomStream - range bounds are exactly as documented")
{
	RandomStream stream(2026);

	// UInt32Below/UInt64Below: [0, bound); bound 0 and 1 return 0.
	CHECK(stream.UInt32Below(0) == 0);
	CHECK(stream.UInt32Below(1) == 0);
	CHECK(stream.UInt64Below(0) == 0);
	for (int i = 0; i < 10000; i++)
	{
		CHECK(stream.UInt32Below(7) < 7u);
		CHECK(stream.UInt64Below(3) < 3u);
	}

	// IntInRange: both ends inclusive, both reached; a reversed range swaps;
	// the full int32 range is legal.
	bool sawMin = false, sawMax = false;
	for (int i = 0; i < 2000; i++)
	{
		const int32_t v = stream.IntInRange(-2, 2);
		CHECK(v >= -2);
		CHECK(v <= 2);
		sawMin |= v == -2;
		sawMax |= v == 2;
		const int32_t r = stream.IntInRange(3, 1);
		CHECK(r >= 1);
		CHECK(r <= 3);
	}
	CHECK(sawMin);
	CHECK(sawMax);
	CHECK(stream.IntInRange(5, 5) == 5);
	(void)stream.IntInRange(INT32_MIN, INT32_MAX);

	// Float/Double: [0, 1).
	for (int i = 0; i < 10000; i++)
	{
		const float f = stream.Float();
		CHECK(f >= 0.0f);
		CHECK(f < 1.0f);
		const double d = stream.Double();
		CHECK(d >= 0.0);
		CHECK(d < 1.0);
	}

	// FloatInRange/DoubleInRange: [min, max) - max is never returned, even
	// where the rounding of min + span * u would reach it (a huge min with a
	// one-ulp span rounds up half the time).
	const float bigMin = 16777216.0f; // 2^24: the next float is 2^24 + 2
	const float bigMax = std::nextafter(bigMin, 1e30f);
	for (int i = 0; i < 1000; i++)
	{
		const float f = stream.FloatInRange(bigMin, bigMax);
		CHECK(f >= bigMin);
		CHECK(f < bigMax);
		const float g = stream.FloatInRange(-1.5f, 2.5f);
		CHECK(g >= -1.5f);
		CHECK(g < 2.5f);
		const double d = stream.DoubleInRange(10.0, 20.0);
		CHECK(d >= 10.0);
		CHECK(d < 20.0);
	}
	CHECK(stream.FloatInRange(3.0f, 3.0f) == 3.0f);
}

TEST_CASE("math: RandomStream - the bounded range passes a chi-square uniformity bound")
{
	// 100k draws into 10 buckets; 9 degrees of freedom, the p = 0.001
	// critical value is 27.88. A fixed seed, so the verdict is stable (this
	// is a bias check on Lemire's method, not a flaky statistics run).
	for (uint64_t seed : { 1ull, 77ull, 0xC0FFEEull })
	{
		RandomStream stream(seed);
		constexpr uint32_t kBuckets = 10;
		constexpr int kDraws = 100000;
		std::array<int, kBuckets> counts{};
		for (int i = 0; i < kDraws; i++)
			counts[stream.UInt32Below(kBuckets)]++;
		const double expected = (double)kDraws / kBuckets;
		double chiSquare = 0.0;
		for (int c : counts)
			chiSquare += ((double)c - expected) * ((double)c - expected) / expected;
		CHECK(chiSquare < 27.88);
	}
}

TEST_CASE("math: RandomStream - the geometry helpers stay on/inside the unit shapes")
{
	RandomStream stream(31337);
	glm::vec3 meanDirection(0.0f);
	for (int i = 0; i < 5000; i++)
	{
		const glm::vec3 u = stream.UnitVector();
		const float length = std::sqrt(u.x * u.x + u.y * u.y + u.z * u.z);
		CHECK(std::fabs(length - 1.0f) < 1e-5f);
		meanDirection += u;

		const glm::vec3 p = stream.PointInUnitSphere();
		CHECK(p.x * p.x + p.y * p.y + p.z * p.z < 1.0f);

		const glm::vec2 d = stream.PointInUnitDisc();
		CHECK(d.x * d.x + d.y * d.y < 1.0f);
	}
	// Uniform on the sphere: the mean direction is near the origin.
	meanDirection /= 5000.0f;
	CHECK(std::sqrt(meanDirection.x * meanDirection.x + meanDirection.y * meanDirection.y + meanDirection.z * meanDirection.z) < 0.05f);

	// The same seed reproduces the same vectors bit for bit.
	RandomStream a(5), b(5);
	for (int i = 0; i < 16; i++)
	{
		const glm::vec3 va = a.UnitVector(), vb = b.UnitVector();
		CHECK(va.x == vb.x);
		CHECK(va.y == vb.y);
		CHECK(va.z == vb.z);
	}
}

TEST_CASE("math: RandomStream - the geometry helpers are pinned for a fixed seed")
{
	// The components are drawn x, then y, then z: a helper that builds its
	// vector from Float() calls as constructor arguments lands them in a
	// compiler-dependent order (g++ swapped x and z against clang++), so these
	// vectors - checked against the Python reference - pin the sequencing.
	{
		RandomStream stream(5);
		const glm::vec3 expected[] = {
			{ -0x1.8501fap-1f, 0x1.775bbap-2f, 0x1.12f15ap-1f },
			{ 0x1.7f26bcp-1f, 0x1.3ea732p-5f, 0x1.530818p-1f },
			{ 0x1.793432p-7f, 0x1.d3aed8p-1f, -0x1.a090fap-2f },
		};
		for (const glm::vec3& value : expected)
		{
			const glm::vec3 v = stream.UnitVector();
			CHECK(v.x == value.x);
			CHECK(v.y == value.y);
			CHECK(v.z == value.z);
		}
	}
	{
		RandomStream stream(5);
		const glm::vec3 expected[] = {
			{ -0x1.b15578p-2f, 0x1.a2211p-3f, 0x1.324588p-2f },
			{ 0x1.494478p-1f, 0x1.11d7p-5f, 0x1.235a3cp-1f },
			{ 0x1.fdd8p-8f, 0x1.3c11c8p-1f, -0x1.198618p-2f },
		};
		for (const glm::vec3& value : expected)
		{
			const glm::vec3 p = stream.PointInUnitSphere();
			CHECK(p.x == value.x);
			CHECK(p.y == value.y);
			CHECK(p.z == value.z);
		}
	}
	{
		RandomStream stream(5);
		const glm::vec2 expected[] = {
			{ -0x1.b15578p-2f, 0x1.a2211p-3f },
			{ 0x1.324588p-2f, 0x1.494478p-1f },
			{ 0x1.11d7p-5f, 0x1.235a3cp-1f },
		};
		for (const glm::vec2& value : expected)
		{
			const glm::vec2 d = stream.PointInUnitDisc();
			CHECK(d.x == value.x);
			CHECK(d.y == value.y);
		}
	}
}

TEST_CASE("math: RandomStream - the entropy seed is not a constant")
{
	// Two OS-entropy draws colliding is a 2^-64 event.
	CHECK(RandomStream::EntropySeed() != RandomStream::EntropySeed());
}

} // namespace TinyEngine::Math::random_stream_tests
