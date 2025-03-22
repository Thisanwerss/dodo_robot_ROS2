# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_dodo_canbus_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED dodo_canbus_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(dodo_canbus_FOUND FALSE)
  elseif(NOT dodo_canbus_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(dodo_canbus_FOUND FALSE)
  endif()
  return()
endif()
set(_dodo_canbus_CONFIG_INCLUDED TRUE)

# output package information
if(NOT dodo_canbus_FIND_QUIETLY)
  message(STATUS "Found dodo_canbus: 0.1.0 (${dodo_canbus_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'dodo_canbus' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${dodo_canbus_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(dodo_canbus_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${dodo_canbus_DIR}/${_extra}")
endforeach()
