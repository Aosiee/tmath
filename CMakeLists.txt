# tmath - Tiny Math: the math conventions, geometry, intersections,
# transforms, projections, colour, sampling, spherical harmonics and a
# deterministic random stream, on glm. This file is the standalone build and
# what a host's CMake build adds with add_subdirectory.
cmake_minimum_required(VERSION 3.20)

# The version is set once, in the header.
file(STRINGS include/tmath/Version.h TMATH_VERSION_LINE REGEX "^#define TMATH_VERSION_STRING ")
string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" TMATH_VERSION_FROM_HEADER "${TMATH_VERSION_LINE}")
project(tmath VERSION ${TMATH_VERSION_FROM_HEADER} LANGUAGES CXX)

if(CMAKE_VERSION VERSION_LESS 3.21)
    string(COMPARE EQUAL "${CMAKE_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}" PROJECT_IS_TOP_LEVEL)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(FetchContent)

option(TMATH_BUILD_TESTS "Build the suite" ${PROJECT_IS_TOP_LEVEL})
# address | undefined: the sanitizer the library and the suite build with.
set(TMATH_SANITIZE "" CACHE STRING "Build with a sanitizer (address or undefined)")
# Checkouts to use instead of fetching the pinned commits below.
set(TMATH_GLM_DIR "" CACHE PATH "A glm checkout (empty: fetch the pinned commit)")
set(TMATH_HAMR_DIR "" CACHE PATH "A hamr checkout, for the suite's test library (empty: fetch the pinned commit)")

# The pinned dependencies: glm, and hamr's C++ test library for the suite.
set(TMATH_GLM_COMMIT e7970a8b26732f1b0df9690f7180546f8c30e48e)
set(TMATH_HAMR_COMMIT 58f44868db9a1b0e4ef22c412ebfd11b20b5a223)

# Only headers are needed from either, so neither project's own CMake build
# runs: SOURCE_SUBDIR names a directory without a CMakeLists.txt.
function(tmath_source_of name url commit dir_var out_var)
    if(${dir_var})
        set(${out_var} "${${dir_var}}" PARENT_SCOPE)
        return()
    endif()
    FetchContent_Declare(${name} GIT_REPOSITORY ${url} GIT_TAG ${commit} SOURCE_SUBDIR tmath-headers-only)
    FetchContent_MakeAvailable(${name})
    set(${out_var} "${${name}_SOURCE_DIR}" PARENT_SCOPE)
endfunction()

# A host that already has glm (a glm::glm target) keeps its own.
if(NOT TARGET glm::glm)
    tmath_source_of(glm https://github.com/g-truc/glm.git ${TMATH_GLM_COMMIT} TMATH_GLM_DIR TMATH_GLM_SOURCE)
    add_library(tmath_glm INTERFACE)
    target_include_directories(tmath_glm SYSTEM INTERFACE "${TMATH_GLM_SOURCE}")
    add_library(glm::glm ALIAS tmath_glm)
endif()

set(TMATH_SANITIZE_FLAGS "")
if(TMATH_SANITIZE)
    set(TMATH_SANITIZE_FLAGS -fsanitize=${TMATH_SANITIZE} -fno-omit-frame-pointer)
endif()

# The flags every tmath translation unit compiles with. -ffp-contract=off is
# part of the determinism contract (Conventions.h, RandomStream.h): no FMA
# contraction, so the same expression rounds the same on every target.
function(tmath_flags target)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE -ffp-contract=off)
        if(PROJECT_IS_TOP_LEVEL)
            target_compile_options(${target} PRIVATE -Wall -Wextra -Werror)
        endif()
    endif()
    if(TMATH_SANITIZE_FLAGS)
        target_compile_options(${target} PRIVATE ${TMATH_SANITIZE_FLAGS})
        target_link_options(${target} PRIVATE ${TMATH_SANITIZE_FLAGS})
    endif()
endfunction()

add_library(tmath STATIC src/RandomStream.cpp)
add_library(tmath::tmath ALIAS tmath)
target_include_directories(tmath PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(tmath PUBLIC cxx_std_20)
target_link_libraries(tmath PUBLIC glm::glm)
# The clip-depth convention (Conventions.h refuses to compile without it).
target_compile_definitions(tmath PUBLIC GLM_FORCE_DEPTH_ZERO_TO_ONE)
tmath_flags(tmath)

if(TMATH_BUILD_TESTS)
    tmath_source_of(hamr https://github.com/Aosiee/hamr.git ${TMATH_HAMR_COMMIT} TMATH_HAMR_DIR TMATH_HAMR_SOURCE)
    file(GLOB TMATH_TEST_SOURCES CONFIGURE_DEPENDS tests/*.cpp)
    add_executable(tmath_tests ${TMATH_TEST_SOURCES})
    target_include_directories(tmath_tests PRIVATE "${TMATH_HAMR_SOURCE}/include")
    target_link_libraries(tmath_tests PRIVATE tmath)
    tmath_flags(tmath_tests)
endif()
