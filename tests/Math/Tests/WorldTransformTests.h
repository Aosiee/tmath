#pragma once

// Math::WorldTransform - the agreement with ComposeTransform near the origin,
// the composition and the inverse kept exact far from it, ToRelative's single
// rounding, the decomposition.

#include "Math/WorldTransform.h"

#include <doctest/doctest.h>

#include <cmath>

namespace TinyEngine::Math::world_transform_tests {

namespace {

	constexpr double kFar = 8388608.0; // 2^23 m: a float's spacing there is 1 m

	bool MatNear(const glm::mat4& a, const glm::mat4& b, float eps)
	{
		for (int c = 0; c < 4; ++c)
			for (int r = 0; r < 4; ++r)
				if (std::abs(a[c][r] - b[c][r]) > eps)
					return false;
		return true;
	}

} // namespace

TEST_CASE("math: WorldTransform agrees with ComposeTransform at the origin")
{
	const glm::vec3 position(1.5f, -2.0f, 3.25f), euler(30.0f, -45.0f, 10.0f), scale(1.0f, 2.0f, 0.5f);
	const WorldTransform world = WorldTransform::FromLocal(position, euler, scale);
	CHECK(MatNear(world.ToRelative(glm::dvec3(0.0)), ComposeTransform(position, euler, scale), 1e-5f));
}

TEST_CASE("math: WorldTransform composition stays exact far from the origin")
{
	const WorldTransform parent = WorldTransform::FromLocal(glm::dvec3(kFar, 10.0, -kFar), glm::vec3(0.0f, 90.0f, 0.0f), glm::vec3(1.0f));
	const WorldTransform child = WorldTransform::FromLocal(glm::dvec3(0.001, 0.0, 0.0), glm::vec3(0.0f), glm::vec3(1.0f));
	const glm::dvec3 world = (parent * child).Position;
	// A +90 degree yaw turns local +X into world -Z: the millimetre survives.
	CHECK(std::abs(world.x - kFar) < 1e-6);
	CHECK(std::abs(world.y - 10.0) < 1e-9);
	CHECK(std::abs(world.z - (-kFar - 0.001)) < 1e-6);
}

TEST_CASE("math: WorldTransform inverse cancels far from the origin")
{
	const WorldTransform a = WorldTransform::FromLocal(glm::dvec3(kFar + 0.125, -3.0, kFar), glm::vec3(20.0f, 40.0f, 60.0f), glm::vec3(2.0f));
	const WorldTransform identity = a.Inverse() * a;
	CHECK(glm::length(identity.Position) < 1e-6);
	CHECK(MatNear(glm::mat4(identity.Basis), glm::mat4(1.0f), 1e-5f));
}

TEST_CASE("math: WorldTransform::ToRelative subtracts in double, then rounds once")
{
	const WorldTransform far = WorldTransform::FromLocal(glm::dvec3(kFar + 0.25, 0.5, kFar - 0.75), glm::vec3(0.0f), glm::vec3(1.0f));
	const glm::mat4 relative = far.ToRelative(glm::dvec3(kFar, 0.0, kFar));
	CHECK(relative[3].x == 0.25f);
	CHECK(relative[3].y == 0.5f);
	CHECK(relative[3].z == -0.75f);
	// The same transform as a float position cannot hold the quarter metre.
	CHECK((float)(kFar + 0.25) == (float)kFar);
}

TEST_CASE("math: WorldTransform::Decompose round-trips the TRS")
{
	const glm::dvec3 position(kFar, 1.0, -kFar + 0.5);
	const glm::vec3 euler(10.0f, 20.0f, 30.0f), scale(1.0f, 3.0f, 2.0f);
	glm::dvec3 p;
	glm::vec3 e, s;
	WorldTransform::FromLocal(position, euler, scale).Decompose(p, e, s);
	CHECK(p == position);
	CHECK(std::abs(e.x - euler.x) < 1e-3f);
	CHECK(std::abs(e.y - euler.y) < 1e-3f);
	CHECK(std::abs(e.z - euler.z) < 1e-3f);
	CHECK(std::abs(s.y - scale.y) < 1e-5f);
}

} // namespace TinyEngine::Math::world_transform_tests
