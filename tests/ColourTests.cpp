
// tmath/Colour.h - the exact sRGB curve (the 8-bit round trip, the knee), the
// fast forms' stated error bounds, Rec. 709 luminance, the HSV conversions
// and the blackbody fit (pinned outputs).

#include "tmath/Colour.h"

#include "hamr/test.h"

#include <cmath>
#include <cstdint>

namespace tmath::colour_tests {

namespace {

	int QuantizeUnorm8(float c) { return (int)std::lround(c * 255.0f); }

	bool NearlyEqualVec(const glm::vec3& a, const glm::vec3& b, float tolerance)
	{
		return std::abs(a.x - b.x) <= tolerance && std::abs(a.y - b.y) <= tolerance && std::abs(a.z - b.z) <= tolerance;
	}

} // namespace

TEST_CASE("colour: sRGB <-> linear - every 8-bit value round-trips exactly after quantising")
{
	for (int byte = 0; byte < 256; byte++)
	{
		const float encoded = (float)byte / 255.0f;
		const float linear = SrgbToLinear(encoded);
		CHECK(linear >= 0.0f);
		CHECK(linear <= 1.0f);
		CHECK(QuantizeUnorm8(LinearToSrgb(linear)) == byte);
		// The fast encode keeps the 8-bit round trip too (its stated use).
		CHECK(QuantizeUnorm8(LinearToSrgbFast(linear)) == byte);
	}
	// The end points and monotonicity over the 8-bit ramp.
	CHECK(SrgbToLinear(0.0f) == 0.0f);
	CHECK(SrgbToLinear(1.0f) == hamr::test::Approx(1.0f).epsilon(1e-6));
	CHECK(LinearToSrgb(0.0f) == 0.0f);
	CHECK(LinearToSrgb(1.0f) == hamr::test::Approx(1.0f).epsilon(1e-6));
	for (int byte = 1; byte < 256; byte++)
		CHECK(SrgbToLinear((float)byte / 255.0f) > SrgbToLinear((float)(byte - 1) / 255.0f));
	// Known values (the standard's): mid-grey 0.5 encoded is 0.2140 linear;
	// 18% linear grey encodes to 0.4614.
	CHECK(SrgbToLinear(0.5f) == hamr::test::Approx(0.21404114f).epsilon(1e-6));
	CHECK(LinearToSrgb(0.18f) == hamr::test::Approx(0.46135613f).epsilon(1e-6));
	// The vec3 forms are per channel.
	const glm::vec3 c{ 0.1f, 0.5f, 0.9f };
	CHECK(SrgbToLinear(c) == glm::vec3(SrgbToLinear(0.1f), SrgbToLinear(0.5f), SrgbToLinear(0.9f)));
	CHECK(LinearToSrgb(c) == glm::vec3(LinearToSrgb(0.1f), LinearToSrgb(0.5f), LinearToSrgb(0.9f)));
}

TEST_CASE("colour: sRGB - the piecewise knee is continuous to the standard's rounding")
{
	// Decode: the toe's end and the power segment's start at 0.04045 meet
	// within 1e-8 (2.3e-9 in exact arithmetic - the standard's constants are
	// rounded).
	const float kneeEncoded = 0.04045f;
	const float toeEnd = kneeEncoded / 12.92f;
	const float powerStart = std::pow((kneeEncoded + 0.055f) / 1.055f, 2.4f);
	CHECK(std::abs(toeEnd - powerStart) <= 1e-8f);
	CHECK(SrgbToLinear(kneeEncoded) == toeEnd);
	CHECK(SrgbToLinear(std::nextafter(kneeEncoded, 1.0f)) == hamr::test::Approx(powerStart).epsilon(1e-5));
	// Encode: at 0.0031308 the pieces meet within 1e-7 (2.9e-8 exact).
	const float kneeLinear = 0.0031308f;
	const float toeEncoded = kneeLinear * 12.92f;
	const float powerEncoded = 1.055f * std::pow(kneeLinear, 1.0f / 2.4f) - 0.055f;
	CHECK(std::abs(toeEncoded - powerEncoded) <= 1e-7f);
	CHECK(LinearToSrgb(kneeLinear) == toeEncoded);
	// Each knee maps onto the other's (the curves are inverses there).
	CHECK(SrgbToLinear(toeEncoded) == hamr::test::Approx(kneeLinear).epsilon(1e-5));
	CHECK(LinearToSrgb(toeEnd) == hamr::test::Approx(kneeEncoded).epsilon(1e-5));
	// Below 0 the toe extends linearly (no NaN from the power).
	CHECK(SrgbToLinear(-0.5f) == -0.5f / 12.92f);
	CHECK(LinearToSrgb(-0.5f) == -0.5f * 12.92f);
}

TEST_CASE("colour: the fast sRGB forms stay inside their stated error bounds")
{
	// The bounds Colour.h states: decode 1.7e-3, encode 1.6e-3 (absolute,
	// over [0, 1]), on a 2^20-step sweep. Below the knees they are exact.
	constexpr int kSteps = 1 << 20;
	float maxDecodeError = 0.0f, maxEncodeError = 0.0f;
	int decodeDecreases = 0, encodeDecreases = 0;
	float previousDecode = 0.0f, previousEncode = 0.0f;
	for (int i = 0; i <= kSteps; i++)
	{
		const float c = (float)i / (float)kSteps;
		const float decoded = SrgbToLinearFast(c), encoded = LinearToSrgbFast(c);
		maxDecodeError = std::max(maxDecodeError, std::abs(decoded - SrgbToLinear(c)));
		maxEncodeError = std::max(maxEncodeError, std::abs(encoded - LinearToSrgb(c)));
		if (c <= 0.0031308f)
			CHECK(encoded == LinearToSrgb(c));
		if (c <= 0.04045f)
			CHECK(decoded == SrgbToLinear(c));
		// Non-decreasing over the sweep (the knee clamp; unclamped, both
		// stepped DOWN by ~1.5e-3 at the knee).
		if (i > 0 && decoded < previousDecode) decodeDecreases++;
		if (i > 0 && encoded < previousEncode) encodeDecreases++;
		previousDecode = decoded;
		previousEncode = encoded;
	}
	CHECK(decodeDecreases == 0);
	CHECK(encodeDecreases == 0);
	CHECK(maxDecodeError <= 1.7e-3f);
	CHECK(maxEncodeError <= 1.6e-3f);
	// Continuous at the knee: the first float above each knee gives the
	// toe's knee value, and every float in a window across the knee is
	// non-decreasing (the decode exactly; the encode's fit only within its
	// sqrt chain's rounding, which Colour.h bounds by 1.2e-7).
	CHECK(SrgbToLinearFast(std::nextafter(0.04045f, 1.0f)) == SrgbToLinearFast(0.04045f));
	CHECK(LinearToSrgbFast(std::nextafter(0.0031308f, 1.0f)) == LinearToSrgbFast(0.0031308f));
	{
		float c = 0.04045f;
		for (int i = 0; i < (1 << 16); i++)
			c = std::nextafter(c, 0.0f);
		float previous = SrgbToLinearFast(c);
		int decreases = 0;
		for (int i = 0; i < (1 << 17); i++, c = std::nextafter(c, 1.0f))
		{
			const float v = SrgbToLinearFast(c);
			if (v < previous) decreases++;
			previous = v;
		}
		CHECK(decreases == 0);
	}
	{
		float c = 0.0031308f;
		for (int i = 0; i < (1 << 16); i++)
			c = std::nextafter(c, 0.0f);
		float runningMax = LinearToSrgbFast(c), worstDip = 0.0f;
		for (int i = 0; i < (1 << 17); i++, c = std::nextafter(c, 1.0f))
		{
			const float v = LinearToSrgbFast(c);
			worstDip = std::max(worstDip, runningMax - v);
			runningMax = std::max(runningMax, v);
		}
		CHECK(worstDip <= 1.2e-7f);
	}
	// It is an approximation: the error is real (not a disguised exact form).
	CHECK(maxDecodeError > 1e-4f);
	CHECK(maxEncodeError > 1e-4f);
	CHECK(SrgbToLinearFast(glm::vec3(0.5f)) == glm::vec3(SrgbToLinearFast(0.5f)));
	CHECK(LinearToSrgbFast(glm::vec3(0.5f)) == glm::vec3(LinearToSrgbFast(0.5f)));
}

TEST_CASE("colour: Rec. 709 luminance - white is exactly 1, the primaries are the weights")
{
	static_assert(kLuminanceR + kLuminanceG + kLuminanceB == 1.0f);
	CHECK(Luminance(glm::vec3(1.0f)) == 1.0f);
	CHECK(Luminance(glm::vec3(0.0f)) == 0.0f);
	CHECK(Luminance({ 1.0f, 0.0f, 0.0f }) == 0.2126f);
	CHECK(Luminance({ 0.0f, 1.0f, 0.0f }) == 0.7152f);
	CHECK(Luminance({ 0.0f, 0.0f, 1.0f }) == 0.0722f);
	// Linear in the colour: a grey g has luminance g.
	CHECK(Luminance(glm::vec3(0.25f)) == hamr::test::Approx(0.25f).epsilon(1e-7));
	CHECK(Luminance(glm::vec3(4.0f)) == hamr::test::Approx(4.0f).epsilon(1e-7));
}

TEST_CASE("colour: HSV <-> RGB - known hues, greys, the hue wrap and the round trips")
{
	// The primaries and secondaries (hue in turns).
	CHECK(RgbToHsv({ 1.0f, 0.0f, 0.0f }) == glm::vec3(0.0f, 1.0f, 1.0f));
	CHECK(RgbToHsv({ 1.0f, 1.0f, 0.0f }).x == hamr::test::Approx(1.0f / 6.0f));
	CHECK(RgbToHsv({ 0.0f, 1.0f, 0.0f }).x == hamr::test::Approx(1.0f / 3.0f));
	CHECK(RgbToHsv({ 0.0f, 1.0f, 1.0f }).x == hamr::test::Approx(0.5f));
	CHECK(RgbToHsv({ 0.0f, 0.0f, 1.0f }).x == hamr::test::Approx(2.0f / 3.0f));
	CHECK(RgbToHsv({ 1.0f, 0.0f, 1.0f }).x == hamr::test::Approx(5.0f / 6.0f));
	CHECK(HsvToRgb({ 0.0f, 1.0f, 1.0f }) == glm::vec3(1.0f, 0.0f, 0.0f));
	CHECK(NearlyEqualVec(HsvToRgb({ 1.0f / 3.0f, 1.0f, 1.0f }), { 0.0f, 1.0f, 0.0f }, 1e-6f));
	CHECK(NearlyEqualVec(HsvToRgb({ 2.0f / 3.0f, 1.0f, 1.0f }), { 0.0f, 0.0f, 1.0f }, 1e-6f));
	CHECK(HsvToRgb({ 0.5f, 0.5f, 0.8f }) == glm::vec3(0.4f, 0.8f, 0.8f));

	// Greys: hue 0, saturation 0, value = the grey; any hue at saturation 0
	// is that grey exactly.
	for (int i = 0; i <= 16; i++)
	{
		const float g = (float)i / 16.0f;
		CHECK(RgbToHsv(glm::vec3(g)) == glm::vec3(0.0f, 0.0f, g));
		for (int h = 0; h < 12; h++)
			CHECK(HsvToRgb({ (float)h / 12.0f, 0.0f, g }) == glm::vec3(g));
	}
	// Below the 1e-6 spread a colour reads as grey.
	CHECK(RgbToHsv({ 0.5f, 0.5f, 0.5000005f }).y == 0.0f);

	// The RGB -> HSV wrap: red-to-magenta hues come out of the fmod
	// negative and are wrapped into [0, 1).
	const glm::vec3 rose = RgbToHsv({ 1.0f, 0.0f, 0.5f });
	CHECK(rose.x == hamr::test::Approx(11.0f / 12.0f));
	// The HSV -> RGB wrap: whole turns (h >= 1) are the same colour; h == 1
	// is red exactly.
	CHECK(HsvToRgb({ 1.0f, 1.0f, 1.0f }) == glm::vec3(1.0f, 0.0f, 0.0f));
	for (int h = 0; h < 48; h++)
		for (int s = 1; s <= 4; s++)
		{
			const float hue = (float)h / 48.0f, sat = (float)s / 4.0f;
			CHECK(NearlyEqualVec(HsvToRgb({ hue + 1.0f, sat, 0.75f }), HsvToRgb({ hue, sat, 0.75f }), 1e-5f));
			CHECK(NearlyEqualVec(HsvToRgb({ hue + 3.0f, sat, 0.75f }), HsvToRgb({ hue, sat, 0.75f }), 1e-5f));
		}

	// RGB -> HSV -> RGB over a 17^3 grid; every hue in [0, 1).
	for (int r = 0; r <= 16; r++)
		for (int g = 0; g <= 16; g++)
			for (int b = 0; b <= 16; b++)
			{
				const glm::vec3 rgb{ (float)r / 16.0f, (float)g / 16.0f, (float)b / 16.0f };
				const glm::vec3 hsv = RgbToHsv(rgb);
				CHECK(hsv.x >= 0.0f);
				CHECK(hsv.x < 1.0f);
				CHECK(NearlyEqualVec(HsvToRgb(hsv), rgb, 1e-5f));
			}
	// HSV -> RGB -> HSV for chromatic colours (s, v > 0).
	for (int h = 0; h < 36; h++)
		for (int s = 1; s <= 4; s++)
			for (int v = 1; v <= 4; v++)
			{
				const glm::vec3 hsv{ (float)h / 36.0f, (float)s / 4.0f, (float)v / 4.0f };
				const glm::vec3 back = RgbToHsv(HsvToRgb(hsv));
				const float hueDelta = std::abs(back.x - hsv.x);
				CHECK(std::min(hueDelta, 1.0f - hueDelta) <= 1e-5f);
				CHECK(back.y == hamr::test::Approx(hsv.y).epsilon(1e-5));
				CHECK(back.z == hamr::test::Approx(hsv.z).epsilon(1e-5));
			}
}

TEST_CASE("colour: BlackbodyColourLinear - the original Light.h output, bit for bit")
{
	// The pins are the pre-move Light.h function's output (generated from
	// HEAD 3a6095c), at run time on glibc's libm:
	// the fit calls pow/log, so another libm (or a compile-time fold of a
	// constant argument, hence the volatile) may differ in the last bit.
	// TODO(the Windows port): these exact pins are glibc-only - on
	// another CRT compare within 1-2 ulp instead (the old-vs-new bit-for-bit
	// proof was a scratch harness's, on this libm).
	struct Pin
	{
		float Kelvin;
		glm::vec3 Linear;
	};
	const Pin pins[] = {
		{ 500.0f, { 0x1p+0f, 0x1.d870b8p-5f, 0x0p+0f } },
		{ 1000.0f, { 0x1p+0f, 0x1.d870b8p-5f, 0x0p+0f } },
		{ 1500.0f, { 0x1p+0f, 0x1.34a4eap-3f, 0x0p+0f } },
		{ 1900.0f, { 0x1p+0f, 0x1.d6bdccp-3f, 0x0p+0f } },
		{ 2000.0f, { 0x1p+0f, 0x1.ff40b4p-3f, 0x1.1d703cp-8f } },
		{ 2700.0f, { 0x1p+0f, 0x1.8a3c7ep-2f, 0x1.8a2686p-4f } },
		{ 4000.0f, { 0x1p+0f, 0x1.3b6072p-1f, 0x1.86e6dp-2f } },
		{ 5500.0f, { 0x1p+0f, 0x1.b3a56cp-1f, 0x1.76ee22p-1f } },
		{ 6500.0f, { 0x1p+0f, 0x1.fbf1dcp-1f, 0x1.e9a51cp-1f } },
		{ 6600.0f, { 0x1p+0f, 0x1p+0f, 0x1p+0f } },
		{ 7000.0f, { 0x1.c93ac8p-1f, 0x1.c733c2p-1f, 0x1p+0f } },
		{ 10000.0f, { 0x1.2d6792p-1f, 0x1.673a14p-1f, 0x1p+0f } },
		{ 20000.0f, { 0x1.9f69acp-2f, 0x1.22662p-1f, 0x1p+0f } },
		{ 40000.0f, { 0x1.40035ap-2f, 0x1.f3f86p-2f, 0x1p+0f } },
		{ 50000.0f, { 0x1.40035ap-2f, 0x1.f3f86p-2f, 0x1p+0f } },
	};
	for (const Pin& pin : pins)
	{
		volatile float kelvin = pin.Kelvin;
		const glm::vec3 c = BlackbodyColourLinear(kelvin);
		INFO("kelvin " << pin.Kelvin);
		CHECK(c.x == pin.Linear.x);
		CHECK(c.y == pin.Linear.y);
		CHECK(c.z == pin.Linear.z);
	}
	// The shape: warm below ~6600 K (blue < red), white at 6600 K, cool
	// above (red < blue), clamped outside 1000-40000 K.
	volatile float warm = 3000.0f, cool = 12000.0f;
	CHECK(BlackbodyColourLinear(warm).z < BlackbodyColourLinear(warm).x);
	CHECK(BlackbodyColourLinear(cool).x < BlackbodyColourLinear(cool).z);
}

} // namespace tmath::colour_tests
