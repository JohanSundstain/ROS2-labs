# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mkopylov_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mkopylov_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mkopylov_FOUND FALSE)
  elseif(NOT mkopylov_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mkopylov_FOUND FALSE)
  endif()
  return()
endif()
set(_mkopylov_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mkopylov_FIND_QUIETLY)
  message(STATUS "Found mkopylov: 0.0.0 (${mkopylov_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mkopylov' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mkopylov_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mkopylov_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mkopylov_DIR}/${_extra}")
endforeach()
