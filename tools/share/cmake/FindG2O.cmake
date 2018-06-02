##______________________________________________________________________________
## Configuration

SET(_package_name "G2O")
SET(_package_header_candidates "g2o/core/solver.h")
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

FIND_LIBRARY(${_package_name}_LIBRARY
    NAMES "g2o"
    HINTS "${${_package_name}_ROOT}/install/lib/static"
    )

##______________________________________________________________________________
## Actions taken when all components have been found

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
    ${_package_name}
    FOUND_VAR ${_package_name}_FOUND
    REQUIRED_VARS ${_package_name}_INCLUDES ${_package_name}_LIBRARY
)
# Force it to show up in the GUI
SET(${_package_name}_FOUND ${${_package_name}_FOUND}
    CACHE BOOL "${_package_name} found?")

##______________________________________________________________________________
## Mark advanced variables

IF (${_package_name}_INCLUDE_DIR)
  SET(${_package_name}_INCLUDE_DIR
      "${${_package_name}_INCLUDE_DIR}"
      "${${_package_name}_INCLUDE_DIR}/g2o/core"
      "${${_package_name}_INCLUDE_DIR}/g2o/stuff"
      "${${_package_name}_INCLUDE_DIR}/g2o/types")
ENDIF ()

SET(${_package_name}_INCLUDE_DIR
    "${${_package_name}_INCLUDES}" CACHE INTERNAL "" FORCE)

SET(${_package_name}_INCLUDE_DIRS
    "${${_package_name}_INCLUDES}" CACHE INTERNAL "" FORCE)

SET(${_package_name}_LIBRARIES
    "${${_package_name}_LIBRARY}" CACHE INTERNAL "" FORCE)

MARK_AS_ADVANCED(
    ${_package_name}_ROOT
    ${_package_name}_FOUND
    ${_package_name}_LIBRARY
    ${_package_name}_INCLUDES
)
