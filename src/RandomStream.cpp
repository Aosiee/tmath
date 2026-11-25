#include "tmath/RandomStream.h"

#include <cmath>
#include <random>

#if defined(_MSC_VER)
	#include <intrin.h>
#endif

namespace tmath {

	namespace {

		uint64_t SplitMix64(uint64_t& x)
		{
			x += 0x9e3779b97f4a7c15ull;
			uint64_t z = x;
			z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
			z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
			return z ^ (z >> 31);
		}

		// The full 128-bit product of two 64-bit values (Lemire's 64-bit
		// bounded draw needs the high and low halves).
		void Multiply64(uint64_t a, uint64_t b, uint64_t& high, uint64_t& low)
		{
#if defined(_MSC_VER)
			low = _umul128(a, b, &high);
#else
			const unsigned __int128 product = (unsigned __int128)a * b;
			high = (uint64_t)(product >> 64);
			low = (uint64_t)product;
#endif
		}

	}

	void RandomStream::Seed(uint64_t seed)
	{
		m_Seed = seed;
		uint64_t x = seed;
		for (uint64_t& word : m_State)
			word = SplitMix64(x);
	}

	void RandomStream::GetState(State& out) const
	{
		for (int i = 0; i < 4; i++)
			out[i] = m_State[i];
	}

	void RandomStream::SetState(const State& state)
	{
		for (int i = 0; i < 4; i++)
			m_State[i] = state[i];
	}

	uint32_t RandomStream::UInt32Below(uint32_t bound)
	{
		if (bound == 0)
			return 0;
		uint64_t m = (uint64_t)UInt32() * bound;
		uint32_t low = (uint32_t)m;
		if (low < bound)
		{
			// The rejection threshold: 2^32 mod bound, the values whose
			// product would over-represent the low buckets.
			const uint32_t threshold = (uint32_t)(-bound) % bound;
			while (low < threshold)
			{
				m = (uint64_t)UInt32() * bound;
				low = (uint32_t)m;
			}
		}
		return (uint32_t)(m >> 32);
	}

	uint64_t RandomStream::UInt64Below(uint64_t bound)
	{
		if (bound == 0)
			return 0;
		uint64_t high = 0, low = 0;
		Multiply64(UInt64(), bound, high, low);
		if (low < bound)
		{
			const uint64_t threshold = (uint64_t)(-bound) % bound;
			while (low < threshold)
				Multiply64(UInt64(), bound, high, low);
		}
		return high;
	}

	int32_t RandomStream::IntInRange(int32_t min, int32_t max)
	{
		if (min > max)
		{
			const int32_t t = min;
			min = max;
			max = t;
		}
		const uint64_t span = (uint64_t)((int64_t)max - (int64_t)min) + 1;
		// The whole int32 range is 2^32 values - one raw draw covers it.
		const uint32_t offset = span > 0xFFFFFFFFull ? UInt32() : UInt32Below((uint32_t)span);
		return (int32_t)((int64_t)min + (int64_t)offset);
	}

	float RandomStream::FloatInRange(float min, float max)
	{
		if (min == max)
			return min;
		if (min > max)
		{
			const float t = min;
			min = max;
			max = t;
		}
		const float value = min + (max - min) * Float();
		return value < max ? value : std::nextafter(max, min);
	}

	double RandomStream::DoubleInRange(double min, double max)
	{
		if (min == max)
			return min;
		if (min > max)
		{
			const double t = min;
			min = max;
			max = t;
		}
		const double value = min + (max - min) * Double();
		return value < max ? value : std::nextafter(max, min);
	}

	glm::vec3 RandomStream::DrawInCube()
	{
		// Named locals, never constructor arguments: argument evaluation
		// order is unspecified (g++ and clang++ swap x and z), and the draws
		// must land in the same components on every compiler.
		const float x = Float() * 2.0f - 1.0f;
		const float y = Float() * 2.0f - 1.0f;
		const float z = Float() * 2.0f - 1.0f;
		return glm::vec3(x, y, z);
	}

	glm::vec3 RandomStream::PointInUnitSphere()
	{
		for (;;)
		{
			const glm::vec3 p = DrawInCube();
			if (p.x * p.x + p.y * p.y + p.z * p.z < 1.0f)
				return p;
		}
	}

	glm::vec2 RandomStream::PointInUnitDisc()
	{
		for (;;)
		{
			// Sequenced like DrawInCube (x, then y).
			const float x = Float() * 2.0f - 1.0f;
			const float y = Float() * 2.0f - 1.0f;
			const glm::vec2 p(x, y);
			if (p.x * p.x + p.y * p.y < 1.0f)
				return p;
		}
	}

	glm::vec3 RandomStream::UnitVector()
	{
		// A point in the ball projected onto the sphere is uniform on the
		// surface; the tiny-length cut keeps the division well conditioned.
		for (;;)
		{
			const glm::vec3 p = DrawInCube();
			const float lengthSq = p.x * p.x + p.y * p.y + p.z * p.z;
			if (lengthSq < 1.0f && lengthSq > 1e-12f)
				return p / std::sqrt(lengthSq);
		}
	}

	uint64_t RandomStream::EntropySeed()
	{
		std::random_device device;
		return ((uint64_t)device() << 32) | (uint64_t)device();
	}

} // namespace tmath
