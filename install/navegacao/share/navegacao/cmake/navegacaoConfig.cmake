# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_navegacao_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED navegacao_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(navegacao_FOUND FALSE)
  elseif(NOT navegacao_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(navegacao_FOUND FALSE)
  endif()
  return()
endif()
set(_navegacao_CONFIG_INCLUDED TRUE)

# output package information
if(NOT navegacao_FIND_QUIETLY)
  message(STATUS "Found navegacao: 0.0.1 (${navegacao_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'navegacao' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT navegacao_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(navegacao_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${navegacao_DIR}/${_extra}")
endforeach()
