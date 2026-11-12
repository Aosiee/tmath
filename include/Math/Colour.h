#pragma once

// Colour. The engine's colours are linear RGB (Conventions.h); sRGB is an
// edge encoding. Everything here is inline and pure. The exact sRGB curve,
// the blackbody fit and the HSV hue's fmod/floor call libm (pow, log, fmod,
// floor), so they follow the platform libm (Conventions.h, "Determinism");
// Luminance, the fast sRGB forms but their sqrt (IEEE-exact) and the HSV
// arithmetic are arithmetic-only.

#include <algorithm>
#include <cmath>

#include <glm/common.hpp>
#include <glm/vec3.hpp>

namespace TinyEngine::Math {

	// ---- sRGB <-> linear (IEC 61966-2-1) ------------------------------------
	// The exact piecewise curve with the standard's constants: the linear
	// toe below the knee (0.04045 encoded, 0.0031308 linear), the 2.4 power
	// above it. The two constants are the standard's rounded ones, so the
	// pieces meet within ~2.3e-9 (linear) / ~2.9e-8 (encoded), not exactly
	// (math.colour pins the gap). Defined for every real input: below 0 the
	// toe extends linearly, above 1 the power curve continues.
	//
	// SrgbToLinear is the expression BlackbodyColourLinear always used (moved
	// here with it, operation for operation).
	inline float SrgbToLinear(float c)
	{
		return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f);
	}

	inline float LinearToSrgb(float c)
	{
		return c <= 0.0031308f ? c * 12.92f : 1.055f * std::pow(c, 1.0f / 2.4f) - 0.055f;
	}

	inline glm::vec3 SrgbToLinear(const glm::vec3& c) { return { SrgbToLinear(c.r), SrgbToLinear(c.g), SrgbToLinear(c.b) }; }
	inline glm::vec3 LinearToSrgb(const glm::vec3& c) { return { LinearToSrgb(c.r), LinearToSrgb(c.g), LinearToSrgb(c.b) }; }

	// ---- sRGB <-> linear, APPROXIMATE -----------------------------------------
	// Fast forms for bulk work where a few thousandths do not matter (never
	// for a cook or a value that is compared or stored). Both keep the exact
	// linear toe below the knee and approximate the power segment; the domain
	// is [0, 1]. The bounds (math.colour checks them over a 2^20-step sweep):
	//   SrgbToLinearFast: |error| <= 1.7e-3 (a cubic, Taylor's fit);
	//   LinearToSrgbFast: |error| <= 1.6e-3 (three sqrts, the Chilliant fit),
	//     which still round-trips every 8-bit value through the exact
	//     SrgbToLinear after quantising (not 10-bit: the bound exceeds half
	//     a 10-bit step). The fast DECODE has no such guarantee: 45 of the
	//     256 8-bit values miss the round trip through the exact encode.
	// Neither fit meets the toe at the knee (each starts ~1.5e-3 BELOW the
	// toe's value there - the decode by 1.49e-3, the encode by 1.57e-3, a
	// downward jump unclamped), so the fitted segment is clamped below
	// by the toe's knee value: a short flat run just above the knee instead
	// of a downward jump. That keeps both continuous and non-decreasing
	// (measured over every float in [0, 1]: the decode never decreases; the
	// encode only by its sqrt chain's rounding, <= 1.2e-7) inside the same
	// bounds - the clamp's error is the fit's own at the run's end.
	inline float SrgbToLinearFast(float c)
	{
		if (c <= 0.04045f)
			return c / 12.92f;
		return std::max(c * (c * (c * 0.305306011f + 0.682171111f) + 0.012522878f), 0.04045f / 12.92f);
	}

	inline float LinearToSrgbFast(float c)
	{
		if (c <= 0.0031308f)
			return c * 12.92f;
		const float s1 = std::sqrt(c);
		const float s2 = std::sqrt(s1);
		const float s3 = std::sqrt(s2);
		return std::max(0.585122381f * s1 + 0.783140355f * s2 - 0.368262736f * s3, 0.0031308f * 12.92f);
	}

	inline glm::vec3 SrgbToLinearFast(const glm::vec3& c) { return { SrgbToLinearFast(c.r), SrgbToLinearFast(c.g), SrgbToLinearFast(c.b) }; }
	inline glm::vec3 LinearToSrgbFast(const glm::vec3& c) { return { LinearToSrgbFast(c.r), LinearToSrgbFast(c.g), LinearToSrgbFast(c.b) }; }

	// ---- Luminance ----------------------------------------------------------
	// Rec. 709 / sRGB primaries, D65: Y of a LINEAR colour. The weights sum to
	// exactly 1.0f, so Luminance(white) == 1.
	inline constexpr float kLuminanceR = 0.2126f;
	inline constexpr float kLuminanceG = 0.7152f;
	inline constexpr float kLuminanceB = 0.0722f;

	constexpr float Luminance(const glm::vec3& linear)
	{
		return kLuminanceR * linear.r + kLuminanceG * linear.g + kLuminanceB * linear.b;
	}

	// ---- HSV <-> RGB ----------------------------------------------------------
	// Moved down from Wisp's ColourMath.h, operation
	// for operation - the colour pickers must not change. All components in
	// [0, 1], hue in turns (0 = red, 1/3 = green, 2/3 = blue). RgbToHsv
	// returns hue in [0, 1) and gives a grey (max - min <= 1e-6) hue 0 and
	// saturation 0. HsvToRgb wraps a hue >= 1 by whole turns; a NEGATIVE hue
	// is outside its domain (the int % keeps the sign and lands on the last
	// sextant - Wisp never passes one).
	inline glm::vec3 RgbToHsv(const glm::vec3& rgb)
	{
		float r = rgb.r, g = rgb.g, b = rgb.b;
		float maxC = std::max(r, std::max(g, b));
		float minC = std::min(r, std::min(g, b));
		float delta = maxC - minC;
		glm::vec3 hsv{ 0.0f, 0.0f, maxC };
		if (delta > 1e-6f)
		{
			hsv.y = delta / maxC;
			if (maxC == r)
				hsv.x = std::fmod((g - b) / delta, 6.0f);
			else if (maxC == g)
				hsv.x = (b - r) / delta + 2.0f;
			else
				hsv.x = (r - g) / delta + 4.0f;
			hsv.x /= 6.0f;
			if (hsv.x < 0.0f) hsv.x += 1.0f;
		}
		return hsv;
	}

	inline glm::vec3 HsvToRgb(const glm::vec3& hsv)
	{
		float h = hsv.r * 6.0f;
		float s = hsv.g, v = hsv.b;
		int i = (int)std::floor(h) % 6;
		float f = h - std::floor(h);
		float p = v * (1.0f - s);
		float q = v * (1.0f - f * s);
		float t = v * (1.0f - (1.0f - f) * s);
		switch (i)
		{
		case 0:  return { v, t, p };
		case 1:  return { q, v, p };
		case 2:  return { p, v, t };
		case 3:  return { p, q, v };
		case 4:  return { t, p, v };
		default: return { v, p, q };
		}
	}

	// ---- Blackbody --------------------------------------------------------------
	// The blackbody (Planckian-locus) colour for a colour temperature, in
	// LINEAR RGB: the standard Kelvin -> sRGB fit (Helland's, 1000-40000 K,
	// clamped), linearized. White (1, 1, 1) at about 6600 K. Moved from the
	// Light component's header, operation for operation; the light gather
	// multiplies it into the light's colour when UseTemperature is on.
	inline glm::vec3 BlackbodyColourLinear(float kelvin)
	{
		float t = glm::clamp(kelvin, 1000.0f, 40000.0f) / 100.0f;
		float r = t <= 66.0f ? 1.0f : glm::clamp(329.698727446f * std::pow(t - 60.0f, -0.1332047592f), 0.0f, 255.0f) / 255.0f;
		float g = t <= 66.0f
			? glm::clamp(99.4708025861f * std::log(t) - 161.1195681661f, 0.0f, 255.0f) / 255.0f
			: glm::clamp(288.1221695283f * std::pow(t - 60.0f, -0.0755148492f), 0.0f, 255.0f) / 255.0f;
		float b = t >= 66.0f ? 1.0f : (t <= 19.0f ? 0.0f : glm::clamp(138.5177312231f * std::log(t - 10.0f) - 305.0447927307f, 0.0f, 255.0f) / 255.0f);
		glm::vec3 srgb(r, g, b);
		return { SrgbToLinear(srgb.r), SrgbToLinear(srgb.g), SrgbToLinear(srgb.b) };
	}

} // namespace TinyEngine::Math
