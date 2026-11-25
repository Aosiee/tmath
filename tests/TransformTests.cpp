
// tmath/Transform.h - the Euler convention (R = Rx Ry Rz, degrees) against
// hand-built matrices, the compose -> decompose round trips (uniform,
// non-uniform and negative scale stated), the Euler <-> quaternion round
// trips away from gimbal lock, the gimbal-lock fold, and TRS (incl. the
// reflection DecomposeTransform cannot represent).

#include "tmath/Transform.h"
#include "tmath/RandomStream.h"

#include "hamr/test.h"

#include <cmath>

#include <glm/gtc/matrix_transform.hpp>

namespace tmath::transform_tests {

namespace {

	bool MatNear(const glm::mat4& a, const glm::mat4& b, float eps)
	{
		for (int c = 0; c < 4; ++c)
			for (int r = 0; r < 4; ++r)
				if (std::abs(a[c][r] - b[c][r]) > eps)
					return false;
		return true;
	}

	bool VecNear(const glm::vec3& a, const glm::vec3& b, float eps)
	{
		return std::abs(a.x - b.x) <= eps && std::abs(a.y - b.y) <= eps && std::abs(a.z - b.z) <= eps;
	}

	// Rotations are equal as rotations: q and -q are the same one.
	bool QuatSameRotation(const glm::quat& a, const glm::quat& b, float eps)
	{
		return std::abs(std::abs(glm::dot(a, b)) - 1.0f) <= eps;
	}

} // namespace

TEST_CASE("transform: the Euler convention is Rx * Ry * Rz in degrees (Z applied first)")
{
	const glm::vec3 e(30.0f, -45.0f, 60.0f);
	const glm::mat4 expected = glm::rotate(glm::mat4(1.0f), glm::radians(e.x), glm::vec3(1, 0, 0))
		* glm::rotate(glm::mat4(1.0f), glm::radians(e.y), glm::vec3(0, 1, 0))
		* glm::rotate(glm::mat4(1.0f), glm::radians(e.z), glm::vec3(0, 0, 1));
	// The exact product (ComposeTransform's, bit for bit).
	CHECK(EulerDegreesToMatrix(e) == expected);

	// Z first: a pure Z rotation of +90 takes +X to +Y; a following X of
	// +90 takes that +Y on to +Z.
	const glm::vec4 x(1.0f, 0.0f, 0.0f, 0.0f);
	CHECK(VecNear(glm::vec3(EulerDegreesToMatrix({ 0.0f, 0.0f, 90.0f }) * x), { 0.0f, 1.0f, 0.0f }, 1e-6f));
	CHECK(VecNear(glm::vec3(EulerDegreesToMatrix({ 90.0f, 0.0f, 90.0f }) * x), { 0.0f, 0.0f, 1.0f }, 1e-6f));

	// The quaternion is the same rotation.
	CHECK(MatNear(glm::mat4_cast(EulerDegreesToQuat(e)), expected, 1e-6f));
	// glm's own Euler constructor is NOT this convention (radians, Rz Ry Rx).
	CHECK_FALSE(MatNear(glm::mat4_cast(glm::quat(glm::radians(e))), expected, 1e-3f));
}

TEST_CASE("transform: ComposeTransform is T * R * S and decomposes back (uniform and non-uniform scale)")
{
	const glm::vec3 p(1.5f, -2.0f, 7.25f);
	const glm::vec3 e(20.0f, 35.0f, -70.0f);
	const glm::vec3 s(2.0f, 0.5f, 3.0f);
	const glm::mat4 m = ComposeTransform(p, e, s);
	CHECK(m == glm::translate(glm::mat4(1.0f), p) * EulerDegreesToMatrix(e) * glm::scale(glm::mat4(1.0f), s));

	glm::vec3 p2, e2, s2;
	DecomposeTransform(m, p2, e2, s2);
	CHECK(VecNear(p2, p, 1e-6f));
	CHECK(VecNear(e2, e, 1e-3f));
	CHECK(VecNear(s2, s, 1e-5f));

	// Random poses away from gimbal lock (|y| <= 80): every one recomposes,
	// and the angles come back where they were (y in [-90, 90], x and z in
	// (-180, 180], which the sampled ranges stay inside).
	RandomStream rng(0x7A11u);
	for (int i = 0; i < 500; ++i)
	{
		const glm::vec3 pi(rng.FloatInRange(-50.0f, 50.0f), rng.FloatInRange(-50.0f, 50.0f), rng.FloatInRange(-50.0f, 50.0f));
		const glm::vec3 ei(rng.FloatInRange(-179.0f, 179.0f), rng.FloatInRange(-80.0f, 80.0f), rng.FloatInRange(-179.0f, 179.0f));
		const glm::vec3 si(rng.FloatInRange(0.05f, 10.0f), rng.FloatInRange(0.05f, 10.0f), rng.FloatInRange(0.05f, 10.0f));
		const glm::mat4 mi = ComposeTransform(pi, ei, si);
		glm::vec3 po, eo, so;
		DecomposeTransform(mi, po, eo, so);
		CHECK(VecNear(eo, ei, 5e-2f));
		CHECK(VecNear(so, si, 1e-4f * 10.0f));
		CHECK(MatNear(ComposeTransform(po, eo, so), mi, 1e-3f));
	}
}

TEST_CASE("transform: negative scale - an even count recomposes, an odd count (a reflection) does not")
{
	const glm::vec3 p(0.0f, 1.0f, 2.0f);
	const glm::vec3 e(10.0f, 20.0f, 30.0f);

	// Two negative axes are a half-turn: the decomposition returns positive
	// scales and a different rotation, and the matrix is the same.
	{
		const glm::mat4 m = ComposeTransform(p, e, { -2.0f, -3.0f, 4.0f });
		glm::vec3 p2, e2, s2;
		DecomposeTransform(m, p2, e2, s2);
		CHECK(VecNear(s2, { 2.0f, 3.0f, 4.0f }, 1e-5f));
		CHECK(MatNear(ComposeTransform(p2, e2, s2), m, 1e-5f));
	}

	// One negative axis is a reflection (det < 0): DecomposeTransform's
	// scales are column lengths (never negative), so the result is NOT the
	// input - the stated limit of the component's Euler form.
	{
		const glm::mat4 m = ComposeTransform(p, e, { -2.0f, 3.0f, 4.0f });
		glm::vec3 p2, e2, s2;
		DecomposeTransform(m, p2, e2, s2);
		CHECK(VecNear(s2, { 2.0f, 3.0f, 4.0f }, 1e-5f));
		CHECK_FALSE(MatNear(ComposeTransform(p2, e2, s2), m, 1e-2f));

		// TRS keeps the reflection on X with a proper rotation.
		const TRS trs = TRS::FromMatrix(m);
		CHECK(trs.Scale.x == hamr::test::Approx(-2.0f).epsilon(1e-5));
		CHECK(trs.Scale.y == hamr::test::Approx(3.0f).epsilon(1e-5));
		CHECK(trs.Scale.z == hamr::test::Approx(4.0f).epsilon(1e-5));
		CHECK(MatNear(trs.ToMatrix(), m, 1e-5f));
	}

	// A negative Y comes back as a negative X plus a half turn - the same
	// matrix, a different (equally valid) split.
	{
		const glm::mat4 m = ComposeTransform(p, e, { 2.0f, -3.0f, 4.0f });
		const TRS trs = TRS::FromMatrix(m);
		CHECK(trs.Scale.x < 0.0f);
		CHECK(trs.Scale.y > 0.0f);
		CHECK(MatNear(trs.ToMatrix(), m, 1e-5f));
	}
}

TEST_CASE("transform: Euler <-> quaternion round trips away from gimbal lock")
{
	RandomStream rng(0xE0E0u);
	for (int i = 0; i < 1000; ++i)
	{
		const glm::vec3 e(rng.FloatInRange(-179.0f, 179.0f), rng.FloatInRange(-85.0f, 85.0f), rng.FloatInRange(-179.0f, 179.0f));
		const glm::quat q = EulerDegreesToQuat(e);
		CHECK(glm::length(q) == hamr::test::Approx(1.0f).epsilon(1e-5));
		const glm::vec3 back = QuatToEulerDegrees(q);
		CHECK(VecNear(back, e, 2e-2f));
		CHECK(QuatSameRotation(EulerDegreesToQuat(back), q, 1e-5f));
	}

	// Known values: the identity, and the single-axis quarter turns.
	CHECK(QuatSameRotation(EulerDegreesToQuat({ 0.0f, 0.0f, 0.0f }), glm::quat::wxyz(1.0f, 0.0f, 0.0f, 0.0f), 0.0f));
	const float h = std::sqrt(0.5f);
	CHECK(QuatSameRotation(EulerDegreesToQuat({ 90.0f, 0.0f, 0.0f }), glm::quat::wxyz(h, h, 0.0f, 0.0f), 1e-6f));
	CHECK(QuatSameRotation(EulerDegreesToQuat({ 0.0f, 90.0f, 0.0f }), glm::quat::wxyz(h, 0.0f, h, 0.0f), 1e-6f));
	CHECK(QuatSameRotation(EulerDegreesToQuat({ 0.0f, 0.0f, 90.0f }), glm::quat::wxyz(h, 0.0f, 0.0f, h), 1e-6f));
	CHECK(VecNear(QuatToEulerDegrees(glm::quat::wxyz(h, h, 0.0f, 0.0f)), { 90.0f, 0.0f, 0.0f }, 1e-3f));
	CHECK(VecNear(QuatToEulerDegrees(glm::quat::wxyz(h, 0.0f, 0.0f, -h)), { 0.0f, 0.0f, -90.0f }, 1e-3f));
	// (A quarter turn about Y is the gimbal-lock case: sqrt(0.5) rounds, so
	// the matrix's sin(y) lands a few ulps below 1, asin amplifies that to
	// ~0.02 degrees, and the 0/0-like atan2s are ill-conditioned - the
	// gimbal-lock case below builds its matrices exactly instead.)

	// Out-of-range input angles come back as the canonical representative
	// of the same rotation (x, z wrap into (-180, 180]).
	const glm::vec3 wrapped = QuatToEulerDegrees(EulerDegreesToQuat({ 190.0f, 10.0f, -200.0f }));
	CHECK(VecNear(wrapped, { -170.0f, 10.0f, 160.0f }, 1e-2f));
}

TEST_CASE("transform: gimbal lock folds z into x and reproduces the matrix, not the angles")
{
	// y = +90: Rx(a) Ry(90) Rz(c) == Rx(a + c) Ry(90).
	{
		const glm::vec3 e(30.0f, 90.0f, 20.0f);
		const glm::mat4 m = EulerDegreesToMatrix(e);
		const glm::vec3 back = RotationToEulerDegrees(glm::mat3(m));
		CHECK(back.y == hamr::test::Approx(90.0f).epsilon(1e-4));
		CHECK(back.z == 0.0f);
		CHECK(back.x == hamr::test::Approx(50.0f).epsilon(1e-3));
		CHECK(MatNear(EulerDegreesToMatrix(back), m, 1e-5f));
	}
	// y = -90: Rx(a) Ry(-90) Rz(c) == Rx(a - c) Ry(-90).
	{
		const glm::vec3 e(30.0f, -90.0f, 20.0f);
		const glm::mat4 m = EulerDegreesToMatrix(e);
		const glm::vec3 back = RotationToEulerDegrees(glm::mat3(m));
		CHECK(back.y == hamr::test::Approx(-90.0f).epsilon(1e-4));
		CHECK(back.z == 0.0f);
		CHECK(back.x == hamr::test::Approx(10.0f).epsilon(1e-3));
		CHECK(MatNear(EulerDegreesToMatrix(back), m, 1e-5f));
	}
	// Through the quaternion and through DecomposeTransform: the same fold.
	{
		const glm::vec3 back = QuatToEulerDegrees(EulerDegreesToQuat({ -15.0f, 90.0f, 40.0f }));
		CHECK(back.z == 0.0f);
		CHECK(back.x == hamr::test::Approx(25.0f).epsilon(1e-3));
		glm::vec3 p, e, s;
		DecomposeTransform(ComposeTransform({ 1.0f, 2.0f, 3.0f }, { -15.0f, 90.0f, 40.0f }, { 2.0f, 2.0f, 2.0f }), p, e, s);
		CHECK(e.z == 0.0f);
		CHECK(e.x == hamr::test::Approx(25.0f).epsilon(1e-3));
	}
}

TEST_CASE("transform: TRS - T * R * S, the Euler forms, FromMatrix round trips, the degenerate scale")
{
	const TRS identity;
	CHECK(identity.ToMatrix() == glm::mat4(1.0f));

	const glm::vec3 p(4.0f, -1.0f, 0.5f);
	const glm::vec3 e(-25.0f, 60.0f, 110.0f);
	const glm::vec3 s(0.25f, 2.0f, 5.0f);
	const TRS trs = TRS::FromEulerDegrees(p, e, s);
	// The same matrix as the Euler composition.
	CHECK(MatNear(trs.ToMatrix(), ComposeTransform(p, e, s), 1e-5f));
	CHECK(VecNear(trs.EulerDegrees(), e, 1e-2f));

	const TRS back = TRS::FromMatrix(trs.ToMatrix());
	CHECK(VecNear(back.Translation, p, 1e-6f));
	CHECK(VecNear(back.Scale, s, 1e-5f));
	CHECK(QuatSameRotation(back.Rotation, trs.Rotation, 1e-5f));

	// Zero scale axes under a non-identity rotation: no NaNs, a unit
	// quaternion, and ToMatrix reproduces the input. One zero axis (each
	// position): the rebuilt column is the cross product of the other two, so
	// the whole rotation comes back. Two zero axes: the surviving column is
	// kept (its rotation about itself is a choice). Three: the identity.
	for (int axis = 0; axis < 3; ++axis)
	{
		glm::vec3 flatScale = s;
		flatScale[axis] = 0.0f;
		const glm::mat4 input = ComposeTransform(p, e, flatScale);
		const TRS flat = TRS::FromMatrix(input);
		CHECK(flat.Scale[axis] == 0.0f);
		CHECK(glm::length(flat.Rotation) == hamr::test::Approx(1.0f));
		CHECK(QuatSameRotation(flat.Rotation, trs.Rotation, 1e-5f));
		CHECK(MatNear(flat.ToMatrix(), input, 1e-5f));

		glm::vec3 lineScale(0.0f);
		lineScale[axis] = s[axis];
		const glm::mat4 lineInput = ComposeTransform(p, e, lineScale);
		const TRS line = TRS::FromMatrix(lineInput);
		CHECK(glm::length(line.Rotation) == hamr::test::Approx(1.0f));
		CHECK(VecNear(glm::mat3_cast(line.Rotation)[axis], glm::vec3(EulerDegreesToMatrix(e)[axis]), 1e-5f));
		CHECK(MatNear(line.ToMatrix(), lineInput, 1e-5f));
	}
	const TRS point = TRS::FromMatrix(ComposeTransform(p, e, glm::vec3(0.0f)));
	CHECK(point.Rotation == glm::quat::wxyz(1.0f, 0.0f, 0.0f, 0.0f));
	CHECK(MatNear(point.ToMatrix(), ComposeTransform(p, e, glm::vec3(0.0f)), 1e-6f));
}

} // namespace tmath::transform_tests
