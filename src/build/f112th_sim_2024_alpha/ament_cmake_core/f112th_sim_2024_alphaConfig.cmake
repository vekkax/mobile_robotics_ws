# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_f112th_sim_2024_alpha_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED f112th_sim_2024_alpha_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(f112th_sim_2024_alpha_FOUND FALSE)
  elseif(NOT f112th_sim_2024_alpha_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(f112th_sim_2024_alpha_FOUND FALSE)
  endif()
  return()
endif()
set(_f112th_sim_2024_alpha_CONFIG_INCLUDED TRUE)

# output package information
if(NOT f112th_sim_2024_alpha_FIND_QUIETLY)
  message(STATUS "Found f112th_sim_2024_alpha: 0.0.0 (${f112th_sim_2024_alpha_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'f112th_sim_2024_alpha' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${f112th_sim_2024_alpha_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(f112th_sim_2024_alpha_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${f112th_sim_2024_alpha_DIR}/${_extra}")
endforeach()
