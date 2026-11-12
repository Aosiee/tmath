#pragma once

// The engine's random numbers. A small
// seeded value type (Unreal's FRandomStream shape) - copyable, explicitly
// seeded, no process-global state. Gameplay randomness reaches one through
// the World (WorldRandom); identity minting never does (an ID must not come
// from a seedable stream - assetio's RandomId128).
//
// The determinism contract: the same seed gives the same numbers on every
// platform and compiler. So the generator and every distribution are our
// own (the <random> distributions are implementation-defined - libstdc++
// and MSVC disagree for the same seed), and the geometry helpers use only
// +, *, / and sqrt (IEEE-exact), never sin/cos - draw their components in
// sequenced statements (x, then y, then z; never as constructor arguments,
// whose evaluation order is unspecified) and compile with
// -ffp-contract=off (plugin.build.luau: no FMA contraction). The tinytest
// `math:` cases pin the outputs, the geometry helpers included.
//
// The generator is xoshiro256** (Blackman & Vigna), its 256-bit state
// filled from the 64-bit seed by splitmix64.

#include <cstdint>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

namespace TinyEngine::Math {

	class RandomStream
	{
	public:
		using State = uint64_t[4];

		// Seed 0 - a default-constructed stream is deterministic too.
		RandomStream() { Seed(0); }
		explicit RandomStream(uint64_t seed) { Seed(seed); }

		// Restarts the stream: the same seed replays the same sequence.
		void Seed(uint64_t seed);
		// The seed the stream was last (re)started from.
		uint64_t GetSeed() const { return m_Seed; }

		// The raw generator state (save/restore for a replay or a
		// rollback; SetState keeps GetSeed() unchanged).
		void GetState(State& out) const;
		void SetState(const State& state);

		// ---- Integers ---------------------------------------------------------
		// The full 64-bit output.
		uint64_t UInt64()
		{
			const uint64_t result = Rotl(m_State[1] * 5, 7) * 9;
			const uint64_t t = m_State[1] << 17;
			m_State[2] ^= m_State[0];
			m_State[3] ^= m_State[1];
			m_State[1] ^= m_State[2];
			m_State[0] ^= m_State[3];
			m_State[2] ^= t;
			m_State[3] = Rotl(m_State[3], 45);
			return result;
		}
		// The high 32 bits of one UInt64 draw (the better bits).
		uint32_t UInt32() { return (uint32_t)(UInt64() >> 32); }

		// [0, bound), unbiased (Lemire's multiply-and-reject). bound 0
		// returns 0 (an empty range has no member to return).
		uint32_t UInt32Below(uint32_t bound);
		uint64_t UInt64Below(uint64_t bound);

		// [min, max], BOTH ends inclusive; min > max swaps them.
		int32_t IntInRange(int32_t min, int32_t max);

		bool Bool() { return (UInt64() >> 63) != 0; }

		// ---- Reals --------------------------------------------------------------
		// [0, 1): the top 24 (float) / 53 (double) bits scaled by 2^-24 /
		// 2^-53, so every value is exactly representable and 1 never occurs.
		float Float() { return (float)(UInt64() >> 40) * 0x1.0p-24f; }
		double Double() { return (double)(UInt64() >> 11) * 0x1.0p-53; }

		// [min, max): max is never returned (a rounding that would reach it
		// steps back to the next value below); min == max returns min; min >
		// max swaps them.
		float FloatInRange(float min, float max);
		double DoubleInRange(double min, double max);

		// ---- Geometry (rejection sampling: +, *, sqrt only) --------------------
		// Uniform on the unit sphere's surface (length 1 within float
		// rounding).
		glm::vec3 UnitVector();
		// Uniform inside the unit ball: length < 1.
		glm::vec3 PointInUnitSphere();
		// Uniform inside the unit disc (the XY plane): length < 1.
		glm::vec2 PointInUnitDisc();

		// ---- Seeding --------------------------------------------------------------
		// A seed from the OS entropy source (std::random_device - getrandom
		// on libstdc++). The default when a World names no seed.
		static uint64_t EntropySeed();

	private:
		static uint64_t Rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
		// A point in [-1, 1)^3, drawn x then y then z (the sequencing is the
		// contract - see the .cpp).
		glm::vec3 DrawInCube();

		uint64_t m_State[4] = {};
		uint64_t m_Seed = 0;
	};

} // namespace TinyEngine::Math
