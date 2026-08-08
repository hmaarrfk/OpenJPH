#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ojph" for configuration "Release"
set_property(TARGET ojph APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ojph PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C;CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libojph.a"
  )

list(APPEND _cmake_import_check_targets ojph )
list(APPEND _cmake_import_check_files_for_ojph "${_IMPORT_PREFIX}/lib/libojph.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
