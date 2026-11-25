
// tmath/Angle.h - the conversions (bit-identical to glm's), NormalizeAngle
// at the +-pi boundaries, the shortest arc across the wrap, and the round
// trips.

#include "tmath/Angle.h"

#include "hamr/test.h"

#include <cmath>
#include <limits>

#include <glm/trigonometric.hpp>

namespace tmath::angle_tests {

static_assert(DegreesToRadians(180.0f) == kPi);
static_assert(RadiansToDegrees(kPiD) == 180.0);

TEST_CASE("angle: degree/radian conversions - known values, glm bit-identity, round trips")
{
	CHECK(DegreesToRadians(0.0f) == 0.0f);
	CHECK(DegreesToRadians(90.0f) == kHalfPi);
	CHECK(DegreesToRadians(180.0f) == kPi);
	CHECK(DegreesToRadians(-360.0f) == hamr::test::Approx(-kTwoPi).epsilon(1e-7));
	CHECK(RadiansToDegrees(kPi) == hamr::test::Approx(180.0f).epsilon(1e-7));
	CHECK(RadiansToDegrees(kHalfPiD) == hamr::test::Approx(90.0).epsilon(1e-15));
	// The same multiply as glm::radians/glm::degrees - bit for bit, so a
	// call site can switch without moving a pixel.
	for (float d : { -720.0f, -181.0f, -45.5f, 0.1f, 33.3f, 60.0f, 179.99f, 1234.5f })
	{
		CHECK(DegreesToRadians(d) == glm::radians(d));
		CHECK(RadiansToDegrees(d) == glm::degrees(d));
		CHECK(DegreesToRadians((double)d) == glm::radians((double)d));
		CHECK(RadiansToDegrees((double)d) == glm::degrees((double)d));
		CHECK(RadiansToDegrees(DegreesToRadians(d)) == hamr::test::Approx(d).epsilon(1e-6));
		CHECK(RadiansToDegrees(DegreesToRadians((double)d)) == hamr::test::Approx((double)d).epsilon(1e-14));
	}
}

TEST_CASE("angle: NormalizeAngle - (-pi, pi], the +-pi boundaries")
{
	CHECK(NormalizeAngle(0.0f) == 0.0f);
	CHECK(NormalizeAngle(1.0f) == 1.0f);
	CHECK(NormalizeAngle(-1.0f) == -1.0f);
	// The boundaries: +pi stays, -pi maps onto +pi (one representative of the
	// half-turn), and the floats either side of them stay on their side.
	CHECK(NormalizeAngle(kPi) == kPi);
	CHECK(NormalizeAngle(-kPi) == kPi);
	CHECK(NormalizeAngle(kPiD) == kPiD);
	CHECK(NormalizeAngle(-kPiD) == kPiD);
	const float belowPi = std::nextafter(kPi, 0.0f);
	const float aboveMinusPi = std::nextafter(-kPi, 0.0f);
	CHECK(NormalizeAngle(belowPi) == belowPi);
	CHECK(NormalizeAngle(aboveMinusPi) == aboveMinusPi);
	// Just past +pi wraps to just above -pi - exactly (pastPi - 2pi).
	const float pastPi = std::nextafter(kPi, 4.0f);
	CHECK(NormalizeAngle(pastPi) > -kPi);
	CHECK(NormalizeAngle(pastPi) == pastPi - kTwoPi);
	CHECK(NormalizeAngle(std::nextafter(-kPi, -4.0f)) == std::nextafter(-kPi, -4.0f) + kTwoPi);
	// Whole turns - exactly 0 (fmod is exact).
	CHECK(NormalizeAngle(kTwoPi) == 0.0f);
	CHECK(NormalizeAngle(-kTwoPi) == 0.0f);
	// ...and +0, never -0 (fmod keeps a negative dividend's sign).
	CHECK(!std::signbit(NormalizeAngle(-kTwoPi)));
	CHECK(!std::signbit(NormalizeAngle(-2.0 * kTwoPiD)));
	CHECK(!std::signbit(NormalizeAngle(-0.0f)));
	CHECK(!std::signbit(ShortestAngleDelta(kTwoPi, 0.0f)));
	CHECK(NormalizeAngle(3.0f * kPi) == hamr::test::Approx(kPi).epsilon(1e-6));
	CHECK(NormalizeAngle(DegreesToRadians(270.0f)) == hamr::test::Approx(DegreesToRadians(-90.0f)).epsilon(1e-6));
	CHECK(NormalizeAngle(DegreesToRadians(-270.0)) == hamr::test::Approx(DegreesToRadians(90.0)).epsilon(1e-12));
	CHECK(NormalizeAngle(100.0 * kTwoPiD + 0.5) == hamr::test::Approx(0.5).epsilon(1e-10));
	// Non-finite -> NaN.
	CHECK(std::isnan(NormalizeAngle(std::numeric_limits<float>::quiet_NaN())));
	CHECK(std::isnan(NormalizeAngle(std::numeric_limits<double>::infinity())));
	// Every result is in (-pi, pi], and the same direction (equal sin/cos).
	for (int i = -500; i <= 500; i++)
	{
		const double a = (double)i * 0.0731;
		const double n = NormalizeAngle(a);
		CHECK(n > -kPiD);
		CHECK(n <= kPiD);
		CHECK(std::cos(n) == hamr::test::Approx(std::cos(a)).epsilon(1e-9));
		CHECK(std::sin(n) == hamr::test::Approx(std::sin(a)).epsilon(1e-9));
	}
}

TEST_CASE("angle: ShortestAngleDelta - the short way round, across the wrap")
{
	CHECK(ShortestAngleDelta(0.0f, 1.0f) == 1.0f);
	CHECK(ShortestAngleDelta(1.0f, 0.0f) == -1.0f);
	CHECK(ShortestAngleDelta(0.5f, 0.5f) == 0.0f);
	// Across the +-pi seam: 170 deg to -170 deg is +20, not -340.
	CHECK(ShortestAngleDelta(DegreesToRadians(170.0f), DegreesToRadians(-170.0f))
		== hamr::test::Approx(DegreesToRadians(20.0f)).epsilon(1e-5));
	CHECK(ShortestAngleDelta(DegreesToRadians(-170.0f), DegreesToRadians(170.0f))
		== hamr::test::Approx(DegreesToRadians(-20.0f)).epsilon(1e-5));
	// Across zero and from unnormalized inputs.
	CHECK(ShortestAngleDelta(DegreesToRadians(350.0), DegreesToRadians(10.0))
		== hamr::test::Approx(DegreesToRadians(20.0)).epsilon(1e-12));
	CHECK(ShortestAngleDelta(DegreesToRadians(10.0), DegreesToRadians(710.0))
		== hamr::test::Approx(DegreesToRadians(-20.0)).epsilon(1e-12));
	// A half-turn apart is +pi (NormalizeAngle's convention), both ways.
	CHECK(ShortestAngleDelta(0.0f, kPi) == kPi);
	CHECK(ShortestAngleDelta(kPi, 0.0f) == kPi);
	// Round trip: from + delta lands on to (modulo a turn).
	for (int i = -40; i <= 40; i++)
		for (int j = -40; j <= 40; j += 7)
		{
			const double from = (double)i * 0.19, to = (double)j * 0.23;
			const double delta = ShortestAngleDelta(from, to);
			CHECK(std::abs(delta) <= kPiD);
			CHECK(NormalizeAngle(from + delta) == hamr::test::Approx(NormalizeAngle(to)).epsilon(1e-9));
		}
}

} // namespace tmath::angle_tests
