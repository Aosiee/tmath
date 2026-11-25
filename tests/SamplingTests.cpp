
// tmath/Sampling.h - the radical inverses (exact first values, base 2 by
// bit reversal against the digit loop), Halton and Hammersley, and the
// warps: hemisphere samples above the plane, the cosine-weighted mean, the
// concentric disc inside the unit disc and area-preserving, the uniform
// sphere's zero mean. The statistical checks use fixed seeds, so they are
// deterministic; each tolerance is stated against its standard error.

#include "tmath/Sampling.h"

#include "hamr/test.h"

#include <cmath>
#include <cstdint>

#include <glm/geometric.hpp>

namespace tmath::sampling_tests {

TEST_CASE("sampling: radical inverse - the known first values, exact")
{
	static_assert(ReverseBits32(0u) == 0u);
	static_assert(ReverseBits32(1u) == 0x80000000u);
	static_assert(ReverseBits32(0x80000000u) == 1u);
	static_assert(ReverseBits32(0x0000FFFFu) == 0xFFFF0000u);
	static_assert(ReverseBits32(0x12345678u) == 0x1E6A2C48u);

	// van der Corput, base 2: 0, 1/2, 1/4, 3/4, 1/8, 5/8, 3/8, 7/8.
	const float base2[] = { 0.0f, 0.5f, 0.25f, 0.75f, 0.125f, 0.625f, 0.375f, 0.875f };
	for (uint32_t i = 0; i < 8; i++)
	{
		CHECK(RadicalInverseBase2(i) == base2[i]);
		CHECK(RadicalInverseBase2D(i) == (double)base2[i]);
		CHECK(RadicalInverse(i, 2u) == (double)base2[i]);
	}
	// Base 3: 0, 1/3, 2/3, 1/9, 4/9, 7/9, 2/9, 5/9, 8/9, 1/27.
	const double base3[] = { 0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0 / 9.0, 4.0 / 9.0, 7.0 / 9.0, 2.0 / 9.0, 5.0 / 9.0, 8.0 / 9.0, 1.0 / 27.0 };
	for (uint32_t i = 0; i < 10; i++)
		CHECK(RadicalInverse(i, 3u) == base3[i]);
	// Base 5: 1/5, 2/5, 3/5, 4/5, 1/25, 6/25.
	CHECK(RadicalInverse(1u, 5u) == 1.0 / 5.0);
	CHECK(RadicalInverse(4u, 5u) == 4.0 / 5.0);
	CHECK(RadicalInverse(5u, 5u) == 1.0 / 25.0);
	CHECK(RadicalInverse(6u, 5u) == 6.0 / 25.0);
	// No digits exist below base 2.
	CHECK(RadicalInverse(7u, 0u) == 0.0);
	CHECK(RadicalInverse(7u, 1u) == 0.0);
	static_assert(RadicalInverse(3u, 2u) == 0.75);
	static_assert(Halton(2u, 3u) == 2.0 / 3.0);
}

TEST_CASE("sampling: radical inverse - base 2's bit reversal equals the digit loop, exactly")
{
	RandomStream rng(0x5A11u);
	for (uint32_t i = 0; i < 65536u; i++)
		CHECK(RadicalInverse(i, 2u) == RadicalInverseBase2D(i));
	for (int k = 0; k < 4096; k++)
	{
		const uint32_t i = rng.UInt32();
		CHECK(RadicalInverse(i, 2u) == RadicalInverseBase2D(i));
		// Every base stays inside [0, 1), even at large indices.
		for (uint32_t base : { 3u, 5u, 7u, 11u, 13u, 65537u })
		{
			const double r = RadicalInverse(i, base);
			CHECK(r >= 0.0);
			CHECK(r < 1.0);
		}
	}
	CHECK(RadicalInverseBase2D(0xFFFFFFFFu) == 1.0 - 0x1p-32);
	CHECK(RadicalInverse(0xFFFFFFFFu, 2u) == 1.0 - 0x1p-32);
	// The float form is exact below 2^24 (sampled with a prime stride).
	for (uint32_t i = 0; i < (1u << 24); i += (i < 65536u ? 1u : 251u))
		CHECK(RadicalInverseBase2(i) == (float)RadicalInverseBase2D(i));
}

TEST_CASE("sampling: Halton and Hammersley - the known first points")
{
	const glm::vec2 halton[] = { { 0.0f, 0.0f }, { 0.5f, 1.0f / 3.0f }, { 0.25f, 2.0f / 3.0f }, { 0.75f, 1.0f / 9.0f }, { 0.125f, 4.0f / 9.0f } };
	for (uint32_t i = 0; i < 5; i++)
		CHECK(Halton2D(i) == halton[i]);
	CHECK(Halton(4u, 2u) == 0.125);
	CHECK(Halton(4u, 3u) == 4.0 / 9.0);

	// Hammersley(i, n) = (i / n, base-2 radical inverse).
	const glm::vec2 hammersley[] = { { 0.0f, 0.0f }, { 0.25f, 0.5f }, { 0.5f, 0.25f }, { 0.75f, 0.75f } };
	for (uint32_t i = 0; i < 4; i++)
		CHECK(Hammersley(i, 4u) == hammersley[i]);
	// A power-of-two set is a (0, m, 2)-net: each of n equal strips in y
	// holds exactly one point.
	constexpr uint32_t n = 256;
	bool strip[n] = {};
	for (uint32_t i = 0; i < n; i++)
	{
		const glm::vec2 p = Hammersley(i, n);
		CHECK(p.x == (float)i / (float)n);
		const uint32_t s = (uint32_t)(p.y * (float)n);
		CHECK(s < n);
		CHECK_FALSE(strip[s]);
		strip[s] = true;
	}
}

TEST_CASE("sampling: hemisphere samples are above the plane, unit length, with the expected means")
{
	// The stratified edges: u = 0 and the largest value below 1.
	const float edge = std::nextafter(1.0f, 0.0f);
	for (float ux : { 0.0f, 0.5f, edge })
		for (float uy : { 0.0f, 0.25f, 0.5f, edge })
		{
			CHECK(UniformHemisphereDirection({ ux, uy }).z >= 0.0f);
			CHECK(CosineHemisphereDirection({ ux, uy }).z >= 0.0f);
			CHECK(glm::length(UniformSphereDirection({ ux, uy })) == hamr::test::Approx(1.0f).epsilon(1e-6));
		}

	// N = 65536 fixed-seed samples. The tolerances are about 5 standard
	// errors: cosine-weighted cos(theta) has mean 2/3, sd sqrt(1/18) = 0.236
	// (SE 0.00092, tolerance 0.005); uniform-hemisphere z has mean 1/2, sd
	// 0.289 (SE 0.00113, tolerance 0.006); each uniform-sphere component has
	// mean 0, sd 0.577 (SE 0.00226, tolerance 0.012).
	constexpr int kSamples = 65536;
	RandomStream rng(20261112u);
	double cosineMean = 0.0, uniformZMean = 0.0;
	glm::dvec3 sphereMean(0.0);
	for (int i = 0; i < kSamples; i++)
	{
		const glm::vec3 c = CosineHemisphereDirection(rng);
		const glm::vec3 h = UniformHemisphereDirection(rng);
		const glm::vec3 s = UniformSphereDirection(rng);
		CHECK(c.z >= 0.0f);
		CHECK(h.z >= 0.0f);
		CHECK(std::abs(glm::length(c) - 1.0f) <= 1e-6f);
		CHECK(std::abs(glm::length(h) - 1.0f) <= 1e-6f);
		CHECK(std::abs(glm::length(s) - 1.0f) <= 1e-6f);
		cosineMean += c.z;
		uniformZMean += h.z;
		sphereMean += glm::dvec3(s);
	}
	cosineMean /= kSamples;
	uniformZMean /= kSamples;
	sphereMean /= (double)kSamples;
	CHECK(std::abs(cosineMean - 2.0 / 3.0) <= 0.005);
	CHECK(std::abs(uniformZMean - 0.5) <= 0.006);
	CHECK(std::abs(sphereMean.x) <= 0.012);
	CHECK(std::abs(sphereMean.y) <= 0.012);
	CHECK(std::abs(sphereMean.z) <= 0.012);

	// The constant pdfs integrate to 1 over their domains (4 pi and 2 pi
	// steradians); the cosine pdf is cos(theta) / pi (its integral over the
	// hemisphere is pi / pi).
	CHECK(kUniformSpherePdf * 4.0f * kPi == hamr::test::Approx(1.0f));
	CHECK(kUniformHemispherePdf * kTwoPi == hamr::test::Approx(1.0f));
	CHECK(CosineHemispherePdf(1.0f) == kInvPi);
	CHECK(CosineHemispherePdf(0.0f) == 0.0f);
}

TEST_CASE("sampling: the concentric disc - inside the unit disc, area-preserving")
{
	CHECK(ConcentricDisc({ 0.5f, 0.5f }) == glm::vec2(0.0f));
	// The square's edges reach the circle: u = 0 maps to radius 1.
	CHECK(glm::length(ConcentricDisc({ 0.0f, 0.5f })) == hamr::test::Approx(1.0f));
	CHECK(glm::length(ConcentricDisc({ 0.0f, 0.0f })) == hamr::test::Approx(1.0f));

	// A 512^2 grid of cell centres, inside the unit disc (up to the sin/cos
	// rounding at radius 1).
	//
	// Area-preserving, exactly on the grid: the map sends the square ring
	// max(|2u - 1|) in [a, b) onto the disc ring of radius [a, b), so with the
	// ring edges on cell edges (a = m / 256) the disc ring holds exactly the
	// square ring's cells - (m_b^2 - m_a^2) / 256^2 of them, the ring's area
	// share pi (b^2 - a^2) / pi. 8 rings of about equal area.
	//
	// The angle too: 32 equal sectors each hold 1/32 of the points within
	// 0.004 (a sector edge is a line through the centre crossing at most ~360
	// cells, 0.0014 of the grid, so two edges misplace at most ~0.003).
	constexpr int kGrid = 512, kHalf = kGrid / 2;
	constexpr int kRings = 8, kSectors = 32;
	int edge[kRings + 1];
	for (int k = 0; k <= kRings; k++)
		edge[k] = (int)std::lround(kHalf * std::sqrt((double)k / kRings));
	int ring[kRings] = {}, sector[kSectors] = {};
	for (int j = 0; j < kGrid; j++)
		for (int i = 0; i < kGrid; i++)
		{
			const glm::vec2 u{ ((float)i + 0.5f) / kGrid, ((float)j + 0.5f) / kGrid };
			const glm::vec2 d = ConcentricDisc(u);
			const float r = glm::length(d);
			CHECK(r <= 1.0f + 1e-6f);
			int k = 0;
			while (k < kRings - 1 && r * kHalf >= (float)edge[k + 1])
				k++;
			ring[k]++;
			float angle = std::atan2(d.y, d.x);
			if (angle < 0.0f)
				angle += kTwoPi;
			sector[std::min(kSectors - 1, (int)(angle / kTwoPi * kSectors))]++;
		}
	for (int k = 0; k < kRings; k++)
	{
		const int squareRingCells = 4 * (edge[k + 1] * edge[k + 1] - edge[k] * edge[k]);
		CHECK(ring[k] == squareRingCells);
	}
	const double total = (double)kGrid * kGrid;
	for (int k = 0; k < kSectors; k++)
		CHECK(std::abs(sector[k] / total - 1.0 / kSectors) <= 0.004);

	// Random inputs stay inside too.
	RandomStream rng(7u);
	for (int i = 0; i < 10000; i++)
		CHECK(glm::length(ConcentricDisc(rng)) <= 1.0f + 1e-6f);
}

TEST_CASE("sampling: the RandomStream forms draw u.x then u.y from the caller's stream")
{
	RandomStream a(99u), b(99u);
	for (int i = 0; i < 64; i++)
	{
		const float x0 = b.Float();
		const float y0 = b.Float();
		CHECK(ConcentricDisc(a) == ConcentricDisc({ x0, y0 }));
		const float x1 = b.Float();
		const float y1 = b.Float();
		CHECK(CosineHemisphereDirection(a) == CosineHemisphereDirection({ x1, y1 }));
		const float x2 = b.Float();
		const float y2 = b.Float();
		CHECK(UniformHemisphereDirection(a) == UniformHemisphereDirection({ x2, y2 }));
		const float x3 = b.Float();
		const float y3 = b.Float();
		CHECK(UniformSphereDirection(a) == UniformSphereDirection({ x3, y3 }));
	}
	// The same seed replays the same directions (no hidden state).
	RandomStream c(5u), d(5u);
	for (int i = 0; i < 16; i++)
		CHECK(UniformSphereDirection(c) == UniformSphereDirection(d));
}

} // namespace tmath::sampling_tests
