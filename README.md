# tmath

[![Linux](https://github.com/Aosiee/tmath/actions/workflows/linux.yml/badge.svg?branch=dev)](https://github.com/Aosiee/tmath/actions/workflows/linux.yml)
[![Windows: not supported](https://img.shields.io/badge/Windows-not%20supported-lightgrey)](#building)
[![macOS: not supported](https://img.shields.io/badge/macOS-not%20supported-lightgrey)](#building)

[![Release](https://img.shields.io/github/v/release/Aosiee/tmath)](https://github.com/Aosiee/tmath/releases/latest)

Tiny Math: the math a real-time 3D program needs on top of
[glm](https://github.com/g-truc/glm), with its conventions written down and
checked. glm stays the vocabulary (`glm::vec3`, `glm::mat4`, `glm::quat`);
tmath adds the rules every use of it follows and the functions built on
them: geometry and intersection tests, transforms, projections, colour,
sampling, spherical harmonics and a random stream that gives the same
numbers everywhere.

tmath was written for [TinyEngine](https://github.com/Aosiee/TinyEngine) and
knows nothing about it.

## A taste

```cpp
#include "tmath/tmath.h"

const glm::mat4 proj = tmath::Perspective(tmath::DegreesToRadians(60.0f), 16.0f / 9.0f, 0.1f, 1000.0f);
const float viewDepth = tmath::LinearizeDepth(storedDepth, 0.1f, 1000.0f);

const tmath::Ray ray{ origin, direction };
if (const auto t = tmath::RayPlane(ray, point, normal))
    hit = ray.At(*t);

tmath::RandomStream random(1234);         // the same sequence on every platform
const glm::vec3 dir = random.UnitVector();
```

## The conventions

Written down in `include/tmath/Conventions.h` and checked at compile time
where the compiler can check them:

- **Right-handed**, cameras look down -Z, **+Y up**.
- **Radians** in the API; degrees only in serialized and user-facing
  values, converted with `Angle.h`.
- **Clip depth is [0, 1]** (Vulkan's). `GLM_FORCE_DEPTH_ZERO_TO_ONE` must be
  defined for every translation unit that uses glm; tmath's CMake target
  defines it publicly, and `Conventions.h` refuses to compile without it.
- **Colour is linear RGB**; sRGB is an edge encoding (`Colour.h`).
- **Determinism:** the arithmetic-only functions are bit-stable across
  platforms and compilers given no FMA contraction (tmath compiles with
  `-ffp-contract=off`); functions that call `sin`, `cos`, `pow` and the
  like follow the platform's libm.

## Building

C++20, glm, and for the suite hamr's C++ test library. Linux is what is
checked today; nothing in tmath is Linux-specific.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
build/tmath_tests
```

glm and hamr are fetched at the commits `CMakeLists.txt` pins;
`-DTMATH_GLM_DIR=<dir>` and `-DTMATH_HAMR_DIR=<dir>` use local checkouts
instead. A host build adds tmath with `add_subdirectory` and links
`tmath::tmath`; a host that already has a `glm::glm` target keeps its own.

The full suite, side by side and in every configuration, runs through
[hamr](https://github.com/Aosiee/hamr): `hamr run` (`--tier nightly` adds
the sanitizers).

## Branches and releases

`main` holds releases only, each one tagged. New work is built on a stream
branched from `dev`, lands on `dev` once it passes locally, and is checked
there by CI; when enough has gathered, `dev` becomes the next release. See
[`CONTRIBUTING.md`](CONTRIBUTING.md).

## License

tmath is released under the Odd Fish Society License ([`LICENSE`](LICENSE)), which is
MIT with one added condition. A product that includes tmath credits it in its credits,
about box or documentation, as "Uses tmath - Copyright (c) 2026 Odd Fish Society /
Felipe Lara". A product that uses several Odd Fish Society libraries can credit them
together, on one line or several, such as "Uses kiln, cluau and wisp - Copyright (c)
2026 Odd Fish Society / Felipe Lara". No logo or splash screen is required.