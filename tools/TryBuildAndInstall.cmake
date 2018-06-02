INCLUDE(CMakeParseArguments)

# Helper to build and install an external cmake project (with CMakeLists.txt) at once
FUNCTION(TRY_BUILD_AND_INSTALL srcdir)

  IF (MSVC)
    SET(_all_target "ALL_BUILD")
    SET(_install_target "INSTALL")
  ELSE ()
    SET(_all_target "all")
    SET(_install_target "install")
  ENDIF ()

  GET_FILENAME_COMPONENT(_name "${srcdir}" NAME)
  SET(_build_dir "${srcdir}/build")
  SET(_install_dir "${srcdir}/install")
  CMAKE_PARSE_ARGUMENTS("" "" "" "FLAGS" ${ARGN})
  LIST(APPEND _FLAGS "-DCMAKE_INSTALL_PREFIX=${_install_dir}")
  # Make sure they can be cleaned up with 'make clean'
  SET_PROPERTY(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
      "${_build_dir}" "${_install_dir}")

  IF (NOT EXISTS "${_build_dir}")
    MESSAGE(STATUS "Building '${_name}' in '${_build_dir}'.")
    TRY_COMPILE(_build_status ${_build_dir} ${srcdir} ${_name} ${_all_target}
        CMAKE_FLAGS ${_FLAGS})

    IF (NOT _build_status)
      MESSAGE(FATAL_ERROR "Build failed.")
    ENDIF ()
  ELSE ()
    MESSAGE(STATUS "Skipping build step for '${_name}'")
  ENDIF ()

  IF (NOT EXISTS "${_install_dir}")
    MESSAGE(STATUS "Installing '${_name}' in '${_install_dir}'.")
    TRY_COMPILE(_install_status ${_build_dir} ${srcdir} ${_name} ${_install_target}
        CMAKE_FLAGS ${_FLAGS})
  ELSE ()
    MESSAGE(STATUS "Skipping install step for '${_name}'")
  ENDIF ()

ENDFUNCTION()
