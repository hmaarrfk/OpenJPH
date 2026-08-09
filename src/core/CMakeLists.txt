
file(GLOB CODESTREAM       "codestream/*.cpp" "codestream/*.h")
file(GLOB CODESTREAM_HWY   "codestream/*_hwy.cpp")
file(GLOB CODESTREAM_AVX2  "codestream/*_avx2.cpp")
file(GLOB CODING           "coding/*.cpp" "coding/*.h")
file(GLOB CODING_AVX2      "coding/*_avx2.cpp")
file(GLOB CODING_HWY       "coding/*_hwy.cpp")
file(GLOB COMMON           "openjph/*.h")
file(GLOB OTHERS           "others/*.cpp" "others/*.c")
file(GLOB TRANSFORM        "transform/*.cpp" "transform/*.h")
file(GLOB TRANSFORM_AVX2   "transform/*_avx2.cpp")
file(GLOB TRANSFORM_HWY    "transform/*_hwy.cpp")

list(REMOVE_ITEM CODESTREAM ${CODESTREAM_HWY} ${CODESTREAM_AVX2})
list(REMOVE_ITEM CODING ${CODING_AVX2} ${CODING_HWY})
list(REMOVE_ITEM TRANSFORM ${TRANSFORM_AVX2} ${TRANSFORM_HWY})
list(APPEND SOURCES ${CODESTREAM} ${CODING} ${COMMON} ${OTHERS} ${TRANSFORM})

source_group("codestream"        FILES ${CODESTREAM})
source_group("coding"            FILES ${CODING})
source_group("common"            FILES ${COMMON})
source_group("others"            FILES ${OTHERS})
source_group("transform"         FILES ${TRANSFORM})

# SIMD support is the generic C++ code plus, on x86-64/i386, three
# hand-written AVX2 survivors that measured faster than their Google
# Highway equivalents, plus the Highway kernels below.  Everything is
# gated at run time; OJPH_DISABLE_SIMD removes all of it.
if(NOT EMSCRIPTEN)
  if (NOT OJPH_DISABLE_SIMD)
    if (("${OJPH_TARGET_ARCH}" MATCHES "OJPH_ARCH_X86_64")
      OR ("${OJPH_TARGET_ARCH}" MATCHES "OJPH_ARCH_I386")
      OR MULTI_GEN_X86_64)

      # the AVX2 survivors: the block decoder, the reversible
      # tx_from_cb32 kernel, and the reversible vertical lifting step
      list(APPEND SOURCES ${CODESTREAM_AVX2} ${TRANSFORM_AVX2} ${CODING_AVX2})
      source_group("codestream" FILES ${CODESTREAM_AVX2})
      source_group("transform" FILES ${TRANSFORM_AVX2})
      source_group("coding" FILES ${CODING_AVX2})

      if (OJPH_ENABLE_HWY)
        # Google Highway kernels: encoder (coding) and decode-side data
        # movement (codestream), gated at run time in
        # ojph_codeblock_fun.cpp and ojph_tile.cpp.  OJPH_ENABLE_HWY is
        # turned off by the top-level CMakeLists when the hwy headers or
        # library are not found.
        list(APPEND SOURCES ${CODING_HWY} ${CODESTREAM_HWY})
        source_group("coding" FILES ${CODING_HWY})
        source_group("codestream" FILES ${CODESTREAM_HWY})
      endif()

      # Set compilation flags.  OJPH_COMPILER_MSVC_TRUE is cl.exe only;
      # clang-cl takes the GCC/Clang branch (it accepts -m machine flags).
      if (OJPH_COMPILER_MSVC_TRUE)
        set_source_files_properties(codestream/ojph_codestream_avx2.cpp PROPERTIES COMPILE_FLAGS "/arch:AVX2")
        set_source_files_properties(coding/ojph_block_decoder_avx2.cpp PROPERTIES COMPILE_FLAGS "/arch:AVX2")
        # the encoder keeps MSVC's pre-dispatch behavior: a single
        # statically selected AVX2 target (no target attributes on MSVC)
        set_source_files_properties(coding/ojph_block_encoder_hwy.cpp PROPERTIES
          COMPILE_FLAGS "/arch:AVX2"
          COMPILE_DEFINITIONS "HWY_COMPILE_ONLY_STATIC=1")
        set_source_files_properties(transform/ojph_transform_avx2.cpp PROPERTIES COMPILE_FLAGS "/arch:AVX2")
      else()
        set_source_files_properties(codestream/ojph_codestream_avx2.cpp PROPERTIES COMPILE_FLAGS -mavx2)
        set_source_files_properties(coding/ojph_block_decoder_avx2.cpp PROPERTIES COMPILE_FLAGS -mavx2)
        set_source_files_properties(transform/ojph_transform_avx2.cpp PROPERTIES COMPILE_FLAGS -mavx2)
        # dynamic dispatch: hwy's foreach_target compiles the encoder once
        # per x86 target (SSE4 through the AVX3 variants) with per-function
        # target attributes, so the TU needs no -m flags and stays runnable
        # on any x86-64.  SSE2 and SSSE3 are dropped; the run-time gate in
        # ojph_codeblock_fun.cpp requires SSE4.1.
        set_source_files_properties(coding/ojph_block_encoder_hwy.cpp PROPERTIES
          COMPILE_DEFINITIONS "HWY_DISABLED_TARGETS=(HWY_SSE2|HWY_SSSE3)")
      endif()
      if (OJPH_ENABLE_HWY)
        # The width-portable data-movement kernels use foreach_target.h
        # run-time dispatch; see the transform/colour block below for the
        # flag rationale.
        if (OJPH_COMPILER_MSVC_TRUE)
          set_source_files_properties(codestream/ojph_codestream_hwy.cpp PROPERTIES COMPILE_FLAGS "/arch:AVX2")
        else()
          set_source_files_properties(codestream/ojph_codestream_hwy.cpp
            PROPERTIES COMPILE_FLAGS "${OJPH_FFP_CONTRACT_OFF}"
            COMPILE_DEFINITIONS
            "HWY_DISABLED_TARGETS=(HWY_SSE2|HWY_SSSE3);HWY_BROKEN_EMU128=0")
        endif()
      endif()
    endif()

  endif()

endif()

## Google Highway SIMD implementations of the wavelet transform
# The Highway translation units are compiled once per Highway target
# through foreach_target.h and dispatch at run time to the best target
# the CPU supports, falling back to the implementations above when no
# compiled target is available.
set(OJPH_USE_HWY FALSE)
if (OJPH_ENABLE_HWY AND (NOT OJPH_DISABLE_SIMD) AND (NOT EMSCRIPTEN)
    AND OJPH_HWY_INCLUDE_DIR)
  set(OJPH_USE_HWY TRUE)
  list(APPEND SOURCES ${TRANSFORM_HWY})
  source_group("transform" FILES ${TRANSFORM_HWY})
  if (("${OJPH_TARGET_ARCH}" MATCHES "OJPH_ARCH_X86_64")
      OR ("${OJPH_TARGET_ARCH}" MATCHES "OJPH_ARCH_I386"))
if (OJPH_COMPILER_MSVC_TRUE)
      # MSVC lacks per-function target attributes below AVX; hwy then
      # compiles the baseline the flags select and better, so /arch:AVX2
      # keeps today's AVX2 floor (plus the AVX3 targets) on Windows
      set_source_files_properties(transform/ojph_transform_hwy.cpp
        transform/ojph_colour_hwy.cpp
        PROPERTIES COMPILE_FLAGS "/arch:AVX2")
    else()
# no target flags: foreach_target.h needs the baseline left low
      # (per-target code is built with function target attributes), so
      # the compiled targets span SSE4 through AVX3_SPR; SSE2/SSSE3 are
      # disabled to bound the target count.  With them disabled, the
      # baseline is hwy's portable fallback; keep that at EMU128 (GCC
      # older than 16 would otherwise pick HWY_SCALAR, which lacks some
      # of the ops used here) -- the fallback only has to compile, since
      # the install gates never dispatch below SSE4.
      # OJPH_FFP_CONTRACT_OFF (-ffp-contract=off, spelled per driver in the
      # top-level CMakeLists) keeps the float (irv/ICT) kernels from fusing
      # mul+add into fma, so their values match the generic implementation
      set_source_files_properties(transform/ojph_transform_hwy.cpp
        transform/ojph_colour_hwy.cpp
        PROPERTIES COMPILE_FLAGS "${OJPH_FFP_CONTRACT_OFF}"
        COMPILE_DEFINITIONS
        "HWY_DISABLED_TARGETS=(HWY_SSE2|HWY_SSSE3);HWY_BROKEN_EMU128=0")
    endif()
  endif()
endif()

## Set debug postfix for different platforms
if (MSVC)
  if (NOT DEFINED CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX "d")
  endif()
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  if (NOT DEFINED CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX "_d")
  endif()
endif()

add_library(ojph ${SOURCES})

if (OJPH_USE_HWY)
  target_compile_definitions(ojph PRIVATE OJPH_ENABLE_HWY)
  target_include_directories(ojph PRIVATE ${OJPH_HWY_INCLUDE_DIR})
  # foreach_target.h re-includes the hwy translation units by the path
  # set in HWY_TARGET_INCLUDE, which is relative to this directory
  # (the transform/colour/codestream TUs and the coding/... encoder TU)
  target_include_directories(ojph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  # run-time dispatch (hwy::SupportedTargets and the chosen-target
  # bookkeeping) lives in the hwy library; the top-level CMakeLists
  # guarantees it was found
  target_link_libraries(ojph PRIVATE ${OJPH_HWY_LIBRARY})
endif()

## The option BUILD_SHARED_LIBS
if (BUILD_SHARED_LIBS AND WIN32)
  target_compile_definitions(ojph PRIVATE OJPH_BUILD_SHARED_LIBRARY)
endif()


## include library version/name
set_target_properties(ojph PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(ojph PUBLIC _FILE_OFFSET_BITS=64)
target_include_directories(ojph PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/openjph>
  $<INSTALL_INTERFACE:include>
)

## This is to check if aligned_alloc or posix_memalign is available
# We want the code to compile for C11 and C++11.
# std::aligned_alloc is only availabe in C++17.
# So here we try to see which API is available and adapt the code to use it
if (NOT MSVC)
  include(CheckSymbolExists)
  check_symbol_exists(aligned_alloc "stdlib.h" OJPH_ALIGNED_ALLOC_EXISTS)
  if (OJPH_ALIGNED_ALLOC_EXISTS)
    target_compile_definitions(ojph PRIVATE OJPH_ALIGNED_ALLOC_EXISTS)
  else()
    check_symbol_exists(posix_memalign "stdlib.h" OJPH_POSIX_MEMALIGN_EXISTS)
    if (OJPH_POSIX_MEMALIGN_EXISTS)
      target_compile_definitions(ojph PRIVATE OJPH_POSIX_MEMALIGN_EXISTS)
    endif()
  endif()
endif()

if (MSVC)
  set(OJPH_LIB_NAME_STRING "ojph.${OPENJPH_VERSION_MAJOR}.${OPENJPH_VERSION_MINOR}")
  set_target_properties(ojph
    PROPERTIES
      OUTPUT_NAME "ojph.${OPENJPH_VERSION_MAJOR}.${OPENJPH_VERSION_MINOR}")
else()
  set(OJPH_LIB_NAME_STRING "ojph.${OPENJPH_VERSION_MAJOR}.${OPENJPH_VERSION_MINOR}")
  set_target_properties(ojph
    PROPERTIES
      SOVERSION "${OPENJPH_VERSION_MAJOR}.${OPENJPH_VERSION_MINOR}"
      VERSION "${OPENJPH_VERSION}")
endif()

install(TARGETS ojph
  EXPORT ojph-targets
)

install(DIRECTORY openjph/
  DESTINATION include/ojph
  FILES_MATCHING
  PATTERN "*.h"
)
