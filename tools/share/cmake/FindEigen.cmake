# Copyright (c) 2017 sepehr laal (MIT License)
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
##______________________________________________________________________________
##
# This is a general purpose header-only library cmake finder. To use it, modify
# variables in the "Configuration" section below (variables starting with '_').
#
# Caveats: it does not support version / components.
#
# The following variables are set when found:
#  <Package Name>_FOUND
#  <Package Name>_INCLUDES
#  <Package Name>_INCLUDE_DIR  (same as <Package Name>_INCLUDES for convenience)
#  <Package Name>_INCLUDE_DIRS (same as <Package Name>_INCLUDES for convenience)

##______________________________________________________________________________
## Configuration

SET(_package_name "Eigen")
SET(_package_header_candidates "Eigen/src/Core/Array.h")
SET(_package_suffix_candidates "")

##______________________________________________________________________________
## Check for the header files

# Force it to show up in the GUI
IF (NOT ${_package_name}_ROOT)
  SET(${_package_name}_ROOT "" CACHE PATH "${_package_name} root directory.")
ENDIF ()

FIND_PATH(${_package_name}_INCLUDES
    NAMES "${_package_header_candidates}"
    HINTS ${${_package_name}_ROOT} ${CMAKE_INSTALL_PREFIX}
    PATH_SUFFIXES "${_package_suffix_candidates}"
    )

##______________________________________________________________________________
## Actions taken when all components have been found

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
    ${_package_name}
    FOUND_VAR ${_package_name}_FOUND
    REQUIRED_VARS ${_package_name}_INCLUDES
)
# Force it to show up in the GUI
SET(${_package_name}_FOUND ${${_package_name}_FOUND}
    CACHE BOOL "${_package_name} found?")

##______________________________________________________________________________
## Mark advanced variables

SET(${_package_name}_INCLUDE_DIR
    "${${_package_name}_INCLUDES}" CACHE INTERNAL "" FORCE)

SET(${_package_name}_INCLUDE_DIRS
    "${${_package_name}_INCLUDES}" CACHE INTERNAL "" FORCE)

MARK_AS_ADVANCED(
    ${_package_name}_ROOT
    ${_package_name}_FOUND
    ${_package_name}_INCLUDES
)
