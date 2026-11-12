#pragma once

// Order-2 real spherical harmonics (bands 0-2,
// 9 coefficients) - the sky's diffuse irradiance representation - and the
// cube-map texel solid angle its projection weights with. Moved out of
// VulkanCubemap.cpp (the backend keeps the face layout, the half decode and
// the deringing policy; the arithmetic below is the code it ran, operation
// for operation). Inline and pure; TexelSolidAngle calls atan2 and the
// basis is arithmetic-only (Conventions.h, "Determinism").
//
// The basis order is the usual (l, m) one, on a unit direction d = (x, y, z):
//   0: Y00 = 0.282095            (l = 0)
//   1: Y1-1 = 0.488603 y,  2: Y10 = 0.488603 z,  3: Y11 = 0.488603 x
//   4: Y2-2 = 1.092548 xy, 5: Y2-1 = 1.092548 yz,
//   6: Y20 = 0.315392 (3z^2 - 1), 7: Y21 = 1.092548 xz,
//   8: Y22 = 0.546274 (x^2 - y^2)
// The constants are the 6-significant-digit ones the shaders' ShCoeff eval
// uses (so the CPU projection and the GPU evaluation share one basis):
// sqrt(1/4pi), sqrt(3/4pi), sqrt(15/4pi), sqrt(5/16pi), sqrt(15/16pi)
// rounded - math.sh checks each against its closed form.

#include "Math/Constants.h"

#include <array>
#include <cmath>
#include <cstdint>

#include <glm/vec3.hpp>

namespace TinyEngine::Math {

	inline constexpr float kShBand0 = 0.282095f;       // sqrt(1 / (4 pi))
	inline constexpr float kShBand1 = 0.488603f;       // sqrt(3 / (4 pi))
	inline constexpr float kShBand2Cross = 1.092548f;  // sqrt(15 / (4 pi))  (xy, yz, xz)
	inline constexpr float kShBand2Zonal = 0.315392f;  // sqrt(5 / (16 pi))  (3z^2 - 1)
	inline constexpr float kShBand2Square = 0.546274f; // sqrt(15 / (16 pi)) (x^2 - y^2)

	// The cosine lobe's convolution (Ramamoorthi & Hanrahan 2001, A-hat_l):
	// irradiance E(n) = sum_lm A_l L_lm Y_lm(n) for a radiance L projected to
	// L_lm. Band 0 x pi, band 1 x 2pi/3, band 2 x pi/4.
	inline constexpr float kShCosineLobeBand0 = kPi;
	inline constexpr float kShCosineLobeBand1 = 2.0f * kPi / 3.0f;
	inline constexpr float kShCosineLobeBand2 = kPi / 4.0f;

	using ShBasis9 = std::array<float, 9>;

	// The 9 basis values at a UNIT direction.
	inline ShBasis9 ShEvaluateBasis(const glm::vec3& direction)
	{
		const float dx = direction.x, dy = direction.y, dz = direction.z;
		return {
			kShBand0,
			kShBand1 * dy,
			kShBand1 * dz,
			kShBand1 * dx,
			kShBand2Cross * dx * dy,
			kShBand2Cross * dy * dz,
			kShBand2Zonal * (3.0f * dz * dz - 1.0f),
			kShBand2Cross * dx * dz,
			kShBand2Square * (dx * dx - dy * dy),
		};
	}

	// 9 coefficients of a scalar (T = float) or RGB (T = glm::vec3) function.
	template <typename T>
	struct ShCoefficients9
	{
		std::array<T, 9> C{};
	};
	using Sh9 = ShCoefficients9<float>;
	using ShRgb9 = ShCoefficients9<glm::vec3>;

	// Projection, one sample at a time: sh += value x basis(direction) x
	// solidAngle (per coefficient, in that order). Accumulated over a set of
	// samples whose solid angles partition the sphere (a cube map's texels
	// with TexelSolidAngle), sh converges to the projection integral
	// L_lm = integral of L(d) Y_lm(d) dw.
	template <typename T>
	inline void ShProjectAdd(ShCoefficients9<T>& sh, const glm::vec3& direction, const T& value, float solidAngle)
	{
		const ShBasis9 basis = ShEvaluateBasis(direction);
		for (int i = 0; i < 9; i++)
			sh.C[i] += value * basis[i] * solidAngle;
	}

	// The reconstruction at a unit direction: sum of C_i x basis_i, in index
	// order.
	template <typename T>
	inline T ShEvaluate(const ShCoefficients9<T>& sh, const glm::vec3& direction)
	{
		const ShBasis9 basis = ShEvaluateBasis(direction);
		T result(0.0f);
		for (int i = 0; i < 9; i++)
			result += sh.C[i] * basis[i];
		return result;
	}

	// Radiance coefficients -> irradiance coefficients: each band scaled by
	// its cosine-lobe factor. ShEvaluate of the result is the irradiance
	// E(n) (a constant radiance L gives E = pi L everywhere).
	template <typename T>
	inline ShCoefficients9<T> ShConvolveCosineLobe(ShCoefficients9<T> sh)
	{
		sh.C[0] *= kShCosineLobeBand0;
		for (int i = 1; i <= 3; i++)
			sh.C[i] *= kShCosineLobeBand1;
		for (int i = 4; i <= 8; i++)
			sh.C[i] *= kShCosineLobeBand2;
		return sh;
	}

	// The irradiance at normal n from radiance coefficients.
	template <typename T>
	inline T ShIrradiance(const ShCoefficients9<T>& radiance, const glm::vec3& normal)
	{
		return ShEvaluate(ShConvolveCosineLobe(radiance), normal);
	}

	// The EXACT solid angle of cube-map texel (u, v) on a size x size face:
	// the spherical rectangle's area by the corner-atan2 form (the face at
	// distance 1, its texel corners at [-1, 1] coordinates). The six faces'
	// texels sum to 4 pi. (The old point approximation, inv^2 /
	// (x^2 + y^2 + 1)^1.5, under-projected the small faces' SH irradiance by
	// ~13%, which the IBL furnace exposed as the dielectric-vs-metal band
	// mismatch.) Precondition: size > 0, u and v < size.
	inline float TexelSolidAngle(uint32_t u, uint32_t v, uint32_t size)
	{
		const float inv = 1.0f / (float)size;
		const float x0 = (float)u * inv * 2.0f - 1.0f;
		const float x1 = (float)(u + 1) * inv * 2.0f - 1.0f;
		const float y0 = (float)v * inv * 2.0f - 1.0f;
		const float y1 = (float)(v + 1) * inv * 2.0f - 1.0f;
		const auto corner = [](float x, float y)
		{
			return std::atan2(x * y, std::sqrt(x * x + y * y + 1.0f));
		};
		return corner(x1, y1) - corner(x1, y0) - corner(x0, y1) + corner(x0, y0);
	}

} // namespace TinyEngine::Math
