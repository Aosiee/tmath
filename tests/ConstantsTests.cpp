
// tmath/Constants.h.

#include "tmath/Constants.h"

#include "hamr/test.h"

#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>

#include <glm/gtc/constants.hpp>

namespace tmath::constants_tests {

TEST_CASE("constants: pi is the correctly-rounded float/double (glm::pi and the replaced copies)")
{
	// 0x40490FDB is the float nearest pi.
	CHECK(std::bit_cast<uint32_t>(kPi) == 0x40490FDBu);
	CHECK(std::bit_cast<uint32_t>(3.14159265358979f) == std::bit_cast<uint32_t>(kPi));
	CHECK(kPi == glm::pi<float>());
	CHECK(kPiD == glm::pi<double>());
	CHECK(kPiD == std::acos(-1.0));
	CHECK(kTwoPi == glm::two_pi<float>());
	CHECK(kHalfPi == glm::half_pi<float>());
	CHECK(kInvPi == glm::one_over_pi<float>());
	CHECK(kTwoPiD == glm::two_pi<double>());
	CHECK(kHalfPiD == glm::half_pi<double>());
	CHECK(kInvPiD == glm::one_over_pi<double>());
}

TEST_CASE("constants: the derived constants agree with pi")
{
	CHECK(kInvPi * kPi == hamr::test::Approx(1.0f).epsilon(1e-7));
	CHECK(kInvPiD * kPiD == hamr::test::Approx(1.0).epsilon(1e-15));
	CHECK(kDegToRad * 180.0f == hamr::test::Approx(kPi).epsilon(1e-7));
	CHECK(kRadToDeg * kPi == hamr::test::Approx(180.0f).epsilon(1e-7));
	CHECK(kDegToRadD * 180.0 == hamr::test::Approx(kPiD).epsilon(1e-15));
	CHECK(kRadToDegD * kPiD == hamr::test::Approx(180.0).epsilon(1e-15));
}

TEST_CASE("constants: the tolerances are ordered and above the type's epsilon")
{
	CHECK(kEpsilon > 0.0f);
	CHECK(kRelativeEpsilon > std::numeric_limits<float>::epsilon());
	CHECK(kEpsilonD > 0.0);
	CHECK(kRelativeEpsilonD > std::numeric_limits<double>::epsilon());
	CHECK(kEpsilonD < (double)kEpsilon);
}

} // namespace tmath::constants_tests
