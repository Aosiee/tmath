# Changelog

Each release is an annotated tag (`v0.x.0`, or `v0.x.y` for a release of
fixes only) whose message is the entry below. Versions before 1.0 may change
the API; every entry says what callers need to change.

## 0.1.0

The first release: the math library TinyEngine has used since 2026-11, as a
library of its own.

- `Conventions.h`: right-handed, +Y up, radians, [0, 1] clip depth, linear
  colour; the compile-time checks.
- `Constants.h`, `Scalar.h`, `Angle.h`.
- `Geometry.h` (ray, plane, AABB, sphere, frustum) and `Intersect.h`.
- `Transform.h` (the Euler convention, TRS compose/decompose),
  `WorldTransform.h` (a double position and a float basis, for large
  worlds) and `Projection.h` (perspective, orthographic, reverse Z,
  `LinearizeDepth`, `LookAt`).
- `Colour.h` (exact and fast sRGB, luminance, HSV, blackbody),
  `Sampling.h` (low-discrepancy sequences, sphere and disc warps) and
  `SphericalHarmonics.h` (order-2 SH, irradiance, texel solid angle).
- `RandomStream.h`: xoshiro256** seeded by splitmix64, the same numbers on
  every platform and compiler.
