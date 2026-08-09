## building ojph_expand
#######################

file(GLOB OJPH_EXPAND         "ojph_expand.cpp")
file(GLOB OJPH_IMG_IO         "../others/ojph_img_io.cpp")
file(GLOB OJPH_IMG_IO_SSE4    "../others/ojph_img_io_sse41.cpp")
file(GLOB OJPH_IMG_IO_AVX2    "../others/ojph_img_io_avx2.cpp")
file(GLOB OJPH_IMG_IO_H       "../common/ojph_img_io.h")

list(APPEND SOURCES ${OJPH_EXPAND} ${OJPH_IMG_IO} ${OJPH_IMG_IO_H})

source_group("main"        FILES ${OJPH_EXPAND})
source_group("others"      FILES ${OJPH_IMG_IO})
source_group("common"      FILES ${OJPH_IMG_IO_H})

if(EMSCRIPTEN)
  if (OJPH_ENABLE_WASM_SIMD)
    list(APPEND SOURCES ${OJPH_IMG_IO_SSE4})
    source_group("others" FILES ${OJPH_IMG_IO_SSE4})
    set_source_files_properties(${OJPH_IMG_IO_SSE4} PROPERTIES COMPILE_FLAGS -msse4.1)
  endif()
else()
  if (NOT OJPH_DISABLE_SIMD)
    if (("${OJPH_TARGET_ARCH}" MATCHES "OJPH_ARCH_X86_64")
      OR ("${OJPH_TARGET_ARCH}" MATCHES "OJPH_ARCH_I386")
      OR MULTI_GEN_X86_64)

      list(APPEND SOURCES ${OJPH_IMG_IO_SSE4})
      source_group("others" FILES ${OJPH_IMG_IO_SSE4})
      list(APPEND SOURCES ${OJPH_IMG_IO_AVX2})
      source_group("others" FILES ${OJPH_IMG_IO_AVX2})

      # Set compilation flags; clang-cl takes the GCC/Clang branch
      if (OJPH_COMPILER_MSVC_TRUE)
        set_source_files_properties(${OJPH_IMG_IO_AVX2} PROPERTIES COMPILE_FLAGS "/arch:AVX2")
      else()
        set_source_files_properties(${OJPH_IMG_IO_SSE4} PROPERTIES COMPILE_FLAGS -msse4.1)
        set_source_files_properties(${OJPH_IMG_IO_AVX2} PROPERTIES COMPILE_FLAGS -mavx2)
      endif()
    endif()

    if (("${OJPH_TARGET_ARCH}" MATCHES "OJPH_ARCH_ARM") OR MULTI_GEN_ARM64)

    endif()

  endif()

endif()

add_executable(ojph_expand ${SOURCES})
target_include_directories(ojph_expand PRIVATE ../common)
target_link_libraries(ojph_expand PRIVATE ojph $<TARGET_NAME_IF_EXISTS:TIFF::TIFF>)

set_target_properties(ojph_expand PROPERTIES OUTPUT_NAME ojphf_expand)
install(TARGETS ojph_expand)
