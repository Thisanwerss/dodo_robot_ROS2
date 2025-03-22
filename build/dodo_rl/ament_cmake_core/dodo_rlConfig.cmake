# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_dodo_rl_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED dodo_rl_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(dodo_rl_FOUND FALSE)
  elseif(NOT dodo_rl_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(dodo_rl_FOUND FALSE)
  endif()
  return()
endif()
set(_dodo_rl_CONFIG_INCLUDED TRUE)

# output package information
if(NOT dodo_rl_FIND_QUIETLY)
  message(STATUS "Found dodo_rl: 0.1.0 (${dodo_rl_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'dodo_rl' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${dodo_rl_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(dodo_rl_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${dodo_rl_DIR}/${_extra}")
endforeach()
