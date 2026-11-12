#pragma once

// Math/SphericalHarmonics.h - the basis constants against their closed
// forms, TexelSolidAngle summing to 4 pi, the basis orthonormal under the
// cube's texel quadrature, project -> evaluate reconstructing a constant and
// a linear function, and the cosine-lobe irradiance (a constant environment
// L gives pi L).
//
// The quadrature: every texel of a cube map, at its centre direction,
// weighted by TexelSolidAngle (the engine's projection). The face layout is
// a test-local one: any layout whose texel (u, v) spans the same patch
// TexelSolidAngle measures integrates the same.

#include "Math/Constants.h"
#include "Math/RandomStream.h"
#include "Math/SphericalHarmonics.h"

#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <functional>

#include <glm/geometric.hpp>

namespace TinyEngine::Math::sh_tests {

namespace {

	// Face f's texel-centre direction (the cube-map faces +X, -X, +Y, -Y,
	// +Z, -Z, with (x, y) in [-1, 1] across the face).
	glm::vec3 TexelDirection(uint32_t face, uint32_t u, uint32_t v, uint32_t size)
	{
		const float x = ((float)u + 0.5f) / (float)size * 2.0f - 1.0f;
		const float y = ((float)v + 0.5f) / (float)size * 2.0f - 1.0f;
		glm::vec3 d;
		switch (face)
		{
		case 0: d = { 1.0f, -y, -x }; break;
		case 1: d = { -1.0f, -y, x }; break;
		case 2: d = { x, 1.0f, y }; break;
		case 3: d = { x, -1.0f, -y }; break;
		case 4: d = { x, -y, 1.0f }; break;
		default: d = { -x, -y, -1.0f }; break;
		}
		return glm::normalize(d);
	}

	void ForEachTexel(uint32_t size, const std::function<void(const glm::vec3&, float)>& visit)
	{
		for (uint32_t face = 0; face < 6; face++)
			for (uint32_t v = 0; v < size; v++)
				for (uint32_t u = 0; u < size; u++)
					visit(TexelDirection(face, u, v, size), TexelSolidAngle(u, v, size));
	}

	Sh9 Project(uint32_t size, const std::function<float(const glm::vec3&)>& f)
	{
		Sh9 sh;
		ForEachTexel(size, [&](const glm::vec3& d, float w) { ShProjectAdd(sh, d, f(d), w); });
		return sh;
	}

} // namespace

TEST_CASE("math.sh: the basis constants are the closed forms to 6 significant digits")
{
	const double pi = kPiD;
	CHECK(std::abs((double)kShBand0 - std::sqrt(1.0 / (4.0 * pi))) <= 6e-7);
	CHECK(std::abs((double)kShBand1 - std::sqrt(3.0 / (4.0 * pi))) <= 6e-7);
	CHECK(std::abs((double)kShBand2Cross - std::sqrt(15.0 / (4.0 * pi))) <= 6e-7);
	CHECK(std::abs((double)kShBand2Zonal - std::sqrt(5.0 / (16.0 * pi))) <= 6e-7);
	CHECK(std::abs((double)kShBand2Square - std::sqrt(15.0 / (16.0 * pi))) <= 6e-7);
	// The cosine-lobe factors (Ramamoorthi-Hanrahan): pi, 2pi/3, pi/4.
	CHECK(kShCosineLobeBand0 == kPi);
	CHECK((double)kShCosineLobeBand1 == doctest::Approx(2.0 * pi / 3.0).epsilon(1e-7));
	CHECK((double)kShCosineLobeBand2 == doctest::Approx(pi / 4.0).epsilon(1e-7));
	// The basis at the axes.
	const ShBasis9 up = ShEvaluateBasis({ 0.0f, 0.0f, 1.0f });
	CHECK(up[0] == kShBand0);
	CHECK(up[2] == kShBand1);
	CHECK(up[6] == kShBand2Zonal * 2.0f);
	CHECK(up[1] == 0.0f);
	CHECK(up[3] == 0.0f);
	CHECK(up[8] == 0.0f);
	const ShBasis9 x = ShEvaluateBasis({ 1.0f, 0.0f, 0.0f });
	CHECK(x[3] == kShBand1);
	CHECK(x[8] == kShBand2Square);
	CHECK(x[6] == -kShBand2Zonal);
}

TEST_CASE("math.sh: TexelSolidAngle - the six faces sum to 4 pi, one texel per face is 4 pi / 6")
{
	// A whole face (size 1) is exactly a sixth of the sphere.
	CHECK(TexelSolidAngle(0, 0, 1) == doctest::Approx(4.0f * kPi / 6.0f).epsilon(1e-6));
	for (uint32_t size : { 1u, 2u, 3u, 7u, 16u, 64u, 128u })
	{
		double total = 0.0;
		float smallest = 1e9f, largest = 0.0f;
		for (uint32_t v = 0; v < size; v++)
			for (uint32_t u = 0; u < size; u++)
			{
				const float w = TexelSolidAngle(u, v, size);
				CHECK(w > 0.0f);
				// Symmetric about the face's centre lines.
				CHECK(w == doctest::Approx(TexelSolidAngle(size - 1 - u, v, size)).epsilon(1e-4));
				CHECK(w == doctest::Approx(TexelSolidAngle(v, u, size)).epsilon(1e-4));
				smallest = std::min(smallest, w);
				largest = std::max(largest, w);
				total += 6.0 * (double)w;
			}
		INFO("size " << size);
		CHECK(total == doctest::Approx(4.0 * kPiD).epsilon(2e-5));
		// The corner texels subtend less than the centre ones (by up to
		// 3^1.5 = 5.2 as the size grows).
		if (size > 2)
			CHECK(largest / smallest > 1.5f);
	}
}

TEST_CASE("math.sh: the basis is orthonormal under the cube's texel quadrature")
{
	// The Gram matrix of the 9 functions, integrated over a 64^2 cube in
	// double: within 2e-4 of the identity (the midpoint rule's error plus the
	// constants' 6-digit rounding).
	constexpr uint32_t kSize = 64;
	double gram[9][9] = {};
	ForEachTexel(kSize, [&](const glm::vec3& d, float w)
	{
		const ShBasis9 y = ShEvaluateBasis(d);
		for (int i = 0; i < 9; i++)
			for (int j = 0; j < 9; j++)
				gram[i][j] += (double)y[i] * (double)y[j] * (double)w;
	});
	double worst = 0.0;
	for (int i = 0; i < 9; i++)
		for (int j = 0; j < 9; j++)
			worst = std::max(worst, std::abs(gram[i][j] - (i == j ? 1.0 : 0.0)));
	CHECK(worst <= 2e-4);
}

TEST_CASE("math.sh: project -> evaluate reconstructs a constant and a linear function")
{
	constexpr uint32_t kSize = 32;
	RandomStream rng(9u);

	// A constant: only band 0, C0 = L * sqrt(4 pi).
	const float L = 2.5f;
	const Sh9 constant = Project(kSize, [&](const glm::vec3&) { return L; });
	CHECK((double)constant.C[0] == doctest::Approx(L * std::sqrt(4.0 * kPiD)).epsilon(1e-4));
	for (int i = 1; i < 9; i++)
		CHECK(std::abs(constant.C[i]) <= 1e-4f);

	// A linear function a + b.d lies in bands 0-1: band 2 vanishes and the
	// reconstruction is the function.
	const float a = 0.7f;
	const glm::vec3 b{ 0.3f, -0.2f, 0.5f };
	const Sh9 linear = Project(kSize, [&](const glm::vec3& d) { return a + glm::dot(b, d); });
	for (int i = 4; i < 9; i++)
		CHECK(std::abs(linear.C[i]) <= 1e-4f);
	for (int k = 0; k < 256; k++)
	{
		const glm::vec3 d = rng.UnitVector();
		CHECK(ShEvaluate(constant, d) == doctest::Approx(L).epsilon(1e-4));
		CHECK(ShEvaluate(linear, d) == doctest::Approx(a + glm::dot(b, d)).epsilon(2e-4));
	}

	// The RGB form is the scalar form per channel.
	ShRgb9 rgb;
	ForEachTexel(kSize, [&](const glm::vec3& d, float w) { ShProjectAdd(rgb, d, glm::vec3(1.0f, a + glm::dot(b, d), L), w); });
	const Sh9 unit = Project(kSize, [](const glm::vec3&) { return 1.0f; });
	for (int i = 0; i < 9; i++)
	{
		CHECK(rgb.C[i].x == unit.C[i]);
		CHECK(rgb.C[i].y == linear.C[i]);
		CHECK(rgb.C[i].z == constant.C[i]);
	}
}

TEST_CASE("math.sh: the cosine-lobe irradiance - a constant environment L gives pi L")
{
	constexpr uint32_t kSize = 32;
	RandomStream rng(13u);
	const float L = 1.75f;
	const Sh9 constant = Project(kSize, [&](const glm::vec3&) { return L; });
	// A linear environment a + b.d gives E(n) = pi a + (2 pi / 3) b.n (the
	// cosine lobe's band-1 factor).
	const float a = 0.4f;
	const glm::vec3 b{ -0.1f, 0.25f, 0.3f };
	const Sh9 linear = Project(kSize, [&](const glm::vec3& d) { return a + glm::dot(b, d); });
	for (int k = 0; k < 256; k++)
	{
		const glm::vec3 n = rng.UnitVector();
		CHECK(ShIrradiance(constant, n) == doctest::Approx(kPi * L).epsilon(1e-4));
		CHECK(ShIrradiance(linear, n) == doctest::Approx(kPi * a + 2.0f * kPi / 3.0f * glm::dot(b, n)).epsilon(2e-4));
	}
	// ShConvolveCosineLobe scales the bands and nothing else.
	Sh9 ones;
	for (float& c : ones.C)
		c = 1.0f;
	const Sh9 convolved = ShConvolveCosineLobe(ones);
	CHECK(convolved.C[0] == kShCosineLobeBand0);
	for (int i = 1; i <= 3; i++)
		CHECK(convolved.C[i] == kShCosineLobeBand1);
	for (int i = 4; i <= 8; i++)
		CHECK(convolved.C[i] == kShCosineLobeBand2);
}

} // namespace TinyEngine::Math::sh_tests
