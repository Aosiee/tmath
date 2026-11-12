#pragma once

// Sampling - low-discrepancy sequences and the standard warps from the unit
// square. No hidden state: every function takes its sample index or its
// uniform inputs explicitly, or a RandomStream& the caller owns (drawn u.x
// then u.y, in sequenced statements - RandomStream.h's determinism contract).
//
// The warps' local frame is tangent space: the hemisphere is z >= 0 around +Z
// (the normal), whatever the world's up (Conventions.h's +Y) - rotate the
// result into a basis around the surface normal. The sequences are
// arithmetic-only and exact (below); the warps call sin/cos/sqrt and follow
// the platform libm for sin/cos (Conventions.h, "Determinism").

#include "Math/Constants.h"
#include "Math/RandomStream.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

namespace TinyEngine::Math {

	// ---- Low-discrepancy sequences ----------------------------------------------
	// The 32 bits reversed (bit 0 <-> bit 31).
	constexpr uint32_t ReverseBits32(uint32_t bits)
	{
		bits = (bits << 16u) | (bits >> 16u);
		bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
		bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
		bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
		bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
		return bits;
	}

	// The base-2 radical inverse (van der Corput): the index's bits mirrored
	// about the binary point, by bit reversal. The double form is exact for
	// every index (32 bits fit a double's 53). The float form is the shaders'
	// (and the PBR CPU twin's) expression, (float)reversed * 2^-32: exact for
	// index < 2^24 (at most 24 significant bits), rounded beyond - where it
	// can round up to 1.0f.
	constexpr double RadicalInverseBase2D(uint32_t index)
	{
		return (double)ReverseBits32(index) * 0x1p-32;
	}

	constexpr float RadicalInverseBase2(uint32_t index)
	{
		return (float)ReverseBits32(index) * 0x1p-32f;
	}

	// The radical inverse in any base >= 2: the base-b digits of index
	// mirrored about the radix point, in [0, 1). The digits accumulate as an
	// exact integer fraction reversed / base^k (k = the digit count; base^k <
	// base * 2^32, so it fits 64 bits), then one division: correctly rounded
	// while base^k <= 2^53, within a few ulp beyond. Base 2 gives exactly
	// RadicalInverseBase2D. A base below 2 returns 0 (no digits exist).
	constexpr double RadicalInverse(uint32_t index, uint32_t base)
	{
		if (base < 2u)
			return 0.0;
		uint64_t reversed = 0, denominator = 1;
		while (index > 0u)
		{
			const uint32_t next = index / base;
			reversed = reversed * base + (index - next * base);
			denominator *= base;
			index = next;
		}
		return (double)reversed / (double)denominator;
	}

	// The Halton sequence's value at index in one dimension (its base, by
	// convention the dimension's prime: 2, 3, 5, 7...). Index 0 is 0 (the
	// sequence is not offset; skip index 0 if the origin is unwanted).
	constexpr double Halton(uint32_t index, uint32_t base)
	{
		return RadicalInverse(index, base);
	}

	// The 2D Halton point (bases 2 and 3). In [0, 1)^2 for index < 2^24 -
	// the warps' precondition; beyond, x can round up to 1.0f (from 2^24 on,
	// RadicalInverseBase2) and y too (from 3^16 - 1 = 43046720, the float
	// cast of the base-3 inverse).
	inline glm::vec2 Halton2D(uint32_t index)
	{
		const float x = RadicalInverseBase2(index);
		const float y = (float)RadicalInverse(index, 3u);
		return { x, y };
	}

	// The 2D Hammersley point i of a set of n: (i / n, the base-2 radical
	// inverse of i). The PBR twin's and the shaders' form. Precondition:
	// 0 < n <= 2^24 and i < n, which keeps both components in [0, 1) (i and
	// n are exact floats; beyond 2^24, i / n can round up to 1.0f).
	inline glm::vec2 Hammersley(uint32_t i, uint32_t n)
	{
		return { (float)i / (float)n, RadicalInverseBase2(i) };
	}

	// ---- Warps from [0, 1)^2 ------------------------------------------------------
	// Each takes the uniform pair u, and has a RandomStream& form.

	// The concentric disc (Shirley-Chiu 1997): the square's concentric
	// squares map to the disc's concentric circles, so the map is
	// area-preserving (a uniform u gives a uniform point) with low
	// distortion. The result is inside the unit disc (length <= 1 up to the
	// sin/cos rounding, the square's edge u = 0 reaching radius 1).
	inline glm::vec2 ConcentricDisc(const glm::vec2& u)
	{
		const float ox = 2.0f * u.x - 1.0f;
		const float oy = 2.0f * u.y - 1.0f;
		if (ox == 0.0f && oy == 0.0f)
			return { 0.0f, 0.0f };
		float r, theta;
		if (std::abs(ox) > std::abs(oy))
		{
			r = ox;
			theta = (kPi / 4.0f) * (oy / ox);
		}
		else
		{
			r = oy;
			theta = kHalfPi - (kPi / 4.0f) * (ox / oy);
		}
		return { r * std::cos(theta), r * std::sin(theta) };
	}

	// Uniform on the unit sphere: z = 1 - 2 u.x, phi = 2 pi u.y. pdf 1/(4 pi).
	inline glm::vec3 UniformSphereDirection(const glm::vec2& u)
	{
		const float z = 1.0f - 2.0f * u.x;
		const float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
		const float phi = kTwoPi * u.y;
		return { r * std::cos(phi), r * std::sin(phi), z };
	}

	// Uniform on the +Z hemisphere: z = u.x in [0, 1). pdf 1/(2 pi).
	inline glm::vec3 UniformHemisphereDirection(const glm::vec2& u)
	{
		const float z = u.x;
		const float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
		const float phi = kTwoPi * u.y;
		return { r * std::cos(phi), r * std::sin(phi), z };
	}

	// Cosine-weighted on the +Z hemisphere (Malley's method: the concentric
	// disc lifted onto the hemisphere). pdf cos(theta) / pi, z >= 0.
	inline glm::vec3 CosineHemisphereDirection(const glm::vec2& u)
	{
		const glm::vec2 d = ConcentricDisc(u);
		const float z = std::sqrt(std::max(0.0f, 1.0f - d.x * d.x - d.y * d.y));
		return { d.x, d.y, z };
	}

	inline constexpr float kUniformSpherePdf = 1.0f / (4.0f * kPi);
	inline constexpr float kUniformHemispherePdf = 1.0f / kTwoPi;
	constexpr float CosineHemispherePdf(float cosTheta) { return cosTheta * kInvPi; }

	namespace detail {
		inline glm::vec2 DrawUniformPair(RandomStream& rng)
		{
			const float x = rng.Float();
			const float y = rng.Float();
			return { x, y };
		}
	} // namespace detail

	inline glm::vec2 ConcentricDisc(RandomStream& rng) { return ConcentricDisc(detail::DrawUniformPair(rng)); }
	inline glm::vec3 UniformSphereDirection(RandomStream& rng) { return UniformSphereDirection(detail::DrawUniformPair(rng)); }
	inline glm::vec3 UniformHemisphereDirection(RandomStream& rng) { return UniformHemisphereDirection(detail::DrawUniformPair(rng)); }
	inline glm::vec3 CosineHemisphereDirection(RandomStream& rng) { return CosineHemisphereDirection(detail::DrawUniformPair(rng)); }

} // namespace TinyEngine::Math
