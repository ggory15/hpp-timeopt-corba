# Copyright (c) 2011 CNRS
# Author: Florent Lamiraux, Joseph Mirabel
#
# This file is part of hpp-wholebody-step-corba.
# hpp-wholebody-step-corba is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-wholebody-step-corba is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-wholebody-step-corba.  If not, see
# <http://www.gnu.org/licenses/>.

SET(LIBRARY_NAME ${PROJECT_NAME})
SET(IDL_SOURCES
  problem)

# search for python
FINDPYTHON(2.7 EXACT REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/src)
OMNIIDL_INCLUDE_DIRECTORIES(
  ${HPP_CORBASERVER_DATAROOTDIR}/idl ${CMAKE_SOURCE_DIR}/idl
  )
FILE(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/timeopt)

FOREACH(IDL ${IDL_SOURCES})
  GENERATE_IDL_CPP (hpp/corbaserver/timeopt/${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/timeopt)
  GENERATE_IDL_PYTHON (hpp/corbaserver/timeopt/${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/timeopt)

  INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/timeopt/${IDL}.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/corbaserver/timeopt)
  INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/timeopt/${IDL}_idl.py
    DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/timeopt
    )
  INSTALL(FILES ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/timeopt/${IDL}.idl
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/idl/hpp/corbaserver/timeopt)
ENDFOREACH()

# Generate and install common_idl.py from hpp.corbaserver
GENERATE_IDL_PYTHON (hpp/corbaserver/timeopt/common
  ${HPP_CORBASERVER_DATAROOTDIR}/idl/hpp/corbaserver)

ADD_CUSTOM_TARGET(generate_idl_cpp DEPENDS ${ALL_IDL_CPP_STUBS})
ADD_CUSTOM_TARGET(generate_idl_python DEPENDS ${ALL_IDL_PYTHON_STUBS})

IF (NOT CLIENT_ONLY)
  ADD_LIBRARY(${LIBRARY_NAME}
    SHARED
    server.cc
    problem.impl.hh
    problem.impl.cc
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/timeopt/problemSK.cc
    )

  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-timeopt)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-template-corba)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-util)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} yaml-cpp)

  INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

  # Standalone corba server
  ADD_EXECUTABLE (hpp-timeopt-server hpp-timeopt-corba.cc)
  TARGET_LINK_LIBRARIES (hpp-timeopt-server ${LIBRARY_NAME})
  PKG_CONFIG_USE_DEPENDENCY(hpp-timeopt-server hpp-timeopt)
  PKG_CONFIG_USE_DEPENDENCY(hpp-timeopt-server hpp-corbaserver)
  PKG_CONFIG_USE_DEPENDENCY(hpp-timeopt-server hpp-template-corba)
  PKG_CONFIG_USE_DEPENDENCY(hpp-timeopt-server hpp-util)

  INSTALL (TARGETS hpp-timeopt-server DESTINATION ${CMAKE_INSTALL_BINDIR})
ELSE (NOT CLIENT_ONLY)
  ADD_LIBRARY(${LIBRARY_NAME}
    SHARED
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/timeopt/problem.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/timeopt/problemSK.cc
    )

  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} omniORB4)

  INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})
ENDIF (NOT CLIENT_ONLY)

ADD_DEPENDENCIES (${LIBRARY_NAME} generate_idl_cpp)
ADD_DEPENDENCIES (${LIBRARY_NAME} generate_idl_python)

INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/timeopt/__init__.py

  ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/timeopt/common_idl.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/timeopt
  )
INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/timeopt/client.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/timeopt
  )
INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/timeopt/timeOptSolver.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/timeopt
  )

