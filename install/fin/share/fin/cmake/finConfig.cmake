# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fin_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fin_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fin_FOUND FALSE)
  elseif(NOT fin_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fin_FOUND FALSE)
  endif()
  return()
endif()
set(_fin_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fin_FIND_QUIETLY)
  message(STATUS "Found fin: 0.0.0 (${fin_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fin' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${fin_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fin_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${fin_DIR}/${_extra}")
endforeach()
