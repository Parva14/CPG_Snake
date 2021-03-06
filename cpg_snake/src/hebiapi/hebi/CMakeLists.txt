cmake_minimum_required (VERSION 2.6)
project (hebi_c_api)

if (UNIX AND NOT APPLE)
  if (NOT DEFINED ARCH)
    SET (LIB_SUBDIR lib/linux_x86-64)
    MESSAGE ("Defaulting to x86-64 library; pass -DARCH=<arch> to cmake, where <arch> is arm or x86-64, to manually override.")
  elseif (${ARCH} STREQUAL "arm")
    SET (LIB_SUBDIR lib/linux_arm)
  elseif (${ARCH} STREQUAL "x86-64")
    SET (LIB_SUBDIR lib/linux_x86-64)
  else()
    MESSAGE (FATAL_ERROR "Invalid processor architecture for Linux build: ARCH argument can be arm or x86-64." )
  endif()
elseif (APPLE)
  SET (LIB_SUBDIR lib/macosx_x64)
elseif (WIN32)
  if (NOT DEFINED ARCH)
    SET (LIB_SUBDIR lib/win_x64)
    MESSAGE ("Defaulting to x64 library; pass -DARCH=<arch> to cmake, where <arch> is arm or x86-64, to manually override.")
  elseif (${ARCH} STREQUAL "x86")
    SET (LIB_SUBDIR lib/win_x86)
  elseif (${ARCH} STREQUAL "x64")
    SET (LIB_SUBDIR lib/win_x64)
  else()
    MESSAGE (FATAL_ERROR "Invalid processor architecture for Windows build: ARCH argument can be x86 or x64." )
  endif()
endif ()

set (HEBI_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)
set (HEBI_LINK_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/${LIB_SUBDIR} PARENT_SCOPE)
set (HEBI_LIB_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${LIB_SUBDIR} PARENT_SCOPE)
