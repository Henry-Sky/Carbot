# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_carbot_odom_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED carbot_odom_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(carbot_odom_FOUND FALSE)
  elseif(NOT carbot_odom_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(carbot_odom_FOUND FALSE)
  endif()
  return()
endif()
set(_carbot_odom_CONFIG_INCLUDED TRUE)

# output package information
if(NOT carbot_odom_FIND_QUIETLY)
  message(STATUS "Found carbot_odom: 0.0.0 (${carbot_odom_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'carbot_odom' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${carbot_odom_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(carbot_odom_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${carbot_odom_DIR}/${_extra}")
endforeach()
